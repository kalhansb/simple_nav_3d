#include "simple_nav_3d/mapping/local_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "simple_nav_3d/grid_utils.hpp"

using simple_nav_3d::GridIndex;
using simple_nav_3d::flatten;
using simple_nav_3d::in_bounds;
using simple_nav_3d::yaw_from_quaternion;

namespace
{

int64_t world_key_from_grid(int gx, int gy)
{
  return (static_cast<int64_t>(gx) << 32) | static_cast<uint32_t>(gy);
}

void clear_world_cells_along_ray(
  std::unordered_set<int64_t> & clear_keys,
  double start_x,
  double start_y,
  double end_x,
  double end_y,
  double resolution,
  int keep_tail_cells)
{
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double dist = std::hypot(dx, dy);
  if (dist < resolution) {
    return;
  }

  const double safe_tail = static_cast<double>(std::max(0, keep_tail_cells)) * resolution;
  const double clear_dist = std::max(0.0, dist - safe_tail);
  if (clear_dist <= 0.0) {
    return;
  }

  const double step = std::max(0.5 * resolution, 0.05);
  const int steps = std::max(1, static_cast<int>(std::floor(clear_dist / step)));
  for (int i = 1; i <= steps; ++i) {
    const double t = (static_cast<double>(i) * step) / dist;
    if (t >= 1.0) {
      break;
    }
    const double wx = start_x + t * dx;
    const double wy = start_y + t * dy;
    const int gx = static_cast<int>(std::floor(wx / resolution));
    const int gy = static_cast<int>(std::floor(wy / resolution));
    clear_keys.insert(world_key_from_grid(gx, gy));
  }
}

}  // namespace

namespace simple_nav_3d
{

int64_t LocalMapper::world_key(int gx, int gy)
{
  return world_key_from_grid(gx, gy);
}

LocalMapper::LocalMapper(const NodeParameters & params)
: params_(params)
{
}

MapSnapshot LocalMapper::build_map(
  const nav_msgs::msg::Odometry & odom,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & points_msg)
{
  MapSnapshot out;

  const int width = static_cast<int>(std::round(
      params_.ugv_local_plan_window_size_m / params_.ugv_local_plan_resolution_m));
  const int height = width;
  if (width < 10 || height < 10) {
    return out;
  }

  const double half_window = params_.ugv_local_plan_window_size_m * 0.5;
  const double robot_x = odom.pose.pose.position.x;
  const double robot_y = odom.pose.pose.position.y;
  const double yaw = yaw_from_quaternion(odom.pose.pose.orientation);
  const double now_sec =
    static_cast<double>(odom.header.stamp.sec) + 1e-9 * static_cast<double>(odom.header.stamp.nanosec);

  std::vector<uint8_t> occupied(static_cast<size_t>(width * height), 0);
  std::vector<uint8_t> free_evidence(static_cast<size_t>(width * height), 0);
  std::unordered_set<int64_t> hit_keys;
  std::unordered_set<int64_t> clear_keys;
  std::unordered_set<int64_t> ground_free_keys;

  // Collect obstacle-band points (odom frame) for debug visualization.
  std::vector<float> obstacle_pts_x;
  std::vector<float> obstacle_pts_y;
  std::vector<float> obstacle_pts_z;

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double resolution = params_.ugv_local_plan_resolution_m;

  // -----------------------------------------------------------------------
  // PCL RANSAC ground filter: fit a plane to the point cloud, classify
  // inliers as ground, then process non-ground through obstacle pipeline.
  // -----------------------------------------------------------------------

  // Step 1: Range-filter and transform points into odom frame PCL cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_odom(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_robot(new pcl::PointCloud<pcl::PointXYZ>);

  if (points_msg) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*points_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*points_msg, "z");

    const size_t max_pts = points_msg->width * points_msg->height;
    cloud_odom->reserve(max_pts);
    cloud_robot->reserve(max_pts);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float px = *iter_x;
      const float py = *iter_y;
      const float pz = *iter_z;

      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
        continue;
      }

      const double range_m = std::sqrt(
        static_cast<double>(px) * px +
        static_cast<double>(py) * py +
        static_cast<double>(pz) * pz);
      if (range_m < params_.min_range_m || range_m > params_.max_range_m) {
        continue;
      }

      const float wx = static_cast<float>(robot_x + cos_yaw * px - sin_yaw * py);
      const float wy = static_cast<float>(robot_y + sin_yaw * px + cos_yaw * py);
      cloud_odom->push_back(pcl::PointXYZ(wx, wy, pz));
      cloud_robot->push_back(pcl::PointXYZ(px, py, pz));
    }
  }

  // Step 2: RANSAC plane segmentation on robot-frame cloud (z is height).
  pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);

  if (cloud_robot->size() > 10) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(params_.ground_filter_tolerance_m);
    seg.setMaxIterations(100);
    // Ground plane should be roughly perpendicular to Z axis (within ~20 deg).
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(0.35);  // ~20 degrees
    seg.setInputCloud(cloud_robot);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(*ground_inliers, *coefficients);
  }

  // Build a set of ground indices for O(1) lookup.
  std::unordered_set<int> ground_idx_set;
  ground_idx_set.reserve(ground_inliers->indices.size());
  for (const int idx : ground_inliers->indices) {
    ground_idx_set.insert(idx);
  }

  // Step 3: Process each point — ground inliers mark free, others go
  // through obstacle/ceiling classification and ray-casting.
  for (size_t i = 0; i < cloud_odom->size(); ++i) {
    const double wx = static_cast<double>((*cloud_odom)[i].x);
    const double wy = static_cast<double>((*cloud_odom)[i].y);
    const double pz = static_cast<double>((*cloud_odom)[i].z);  // robot-frame z

    if (ground_idx_set.count(static_cast<int>(i)) > 0) {
      // Ground point: mark own cell as free, no ray-casting.
      const int wgx = static_cast<int>(std::floor(wx / resolution));
      const int wgy = static_cast<int>(std::floor(wy / resolution));
      ground_free_keys.insert(world_key(wgx, wgy));
      continue;
    }

    // Height above ground in odom frame (absolute, for obstacle band check).
    const double obstacle_height_m = params_.sensor_mount_height_m + pz;

    // Ceiling points: skip entirely (same height-parallax issue as ground).
    if (obstacle_height_m > params_.max_obstacle_height_m) {
      continue;
    }

    clear_world_cells_along_ray(
      clear_keys, robot_x, robot_y, wx, wy, resolution, 1);

    // Obstacle-band point: mark occupied.
    const int wgx = static_cast<int>(std::floor(wx / resolution));
    const int wgy = static_cast<int>(std::floor(wy / resolution));
    const int64_t cell_key = world_key(wgx, wgy);

    hit_keys.insert(cell_key);
    obstacle_pts_x.push_back(static_cast<float>(wx));
    obstacle_pts_y.push_back(static_cast<float>(wy));
    obstacle_pts_z.push_back(static_cast<float>(obstacle_height_m));

    if (wx < robot_x - half_window || wx > robot_x + half_window ||
      wy < robot_y - half_window || wy > robot_y + half_window)
    {
      continue;
    }

    GridIndex idx{
      static_cast<int>(std::floor((wx - (robot_x - half_window)) / resolution)),
      static_cast<int>(std::floor((wy - (robot_y - half_window)) / resolution))};

    if (in_bounds(idx, width, height)) {
      occupied[flatten(idx, width)] = 1;
    }
  }

  // Apply free-space evidence from rays and ground returns, then obstacle hits.
  // Free evidence is only accepted for cells that:
  //   1. Are not hit as an obstacle in this frame
  //   2. Do not carry unexpired obstacle evidence from recent frames
  for (const int64_t key : clear_keys) {
    if (hit_keys.find(key) != hit_keys.end()) {
      continue;
    }
    if (world_obstacle_last_seen_sec_.find(key) != world_obstacle_last_seen_sec_.end()) {
      continue;
    }
    world_free_last_seen_sec_[key] = now_sec;
  }
  // Ground returns mark their own cell as free (no ray) — safe because the
  // sensor physically observed the ground at that location.
  for (const int64_t key : ground_free_keys) {
    if (hit_keys.find(key) != hit_keys.end()) {
      continue;
    }
    if (world_obstacle_last_seen_sec_.find(key) != world_obstacle_last_seen_sec_.end()) {
      continue;
    }
    world_free_last_seen_sec_[key] = now_sec;
  }
  for (const int64_t key : hit_keys) {
    world_obstacle_last_seen_sec_[key] = now_sec;
    world_free_last_seen_sec_.erase(key);
  }

  // Expire stale obstacle evidence; otherwise the local map can accumulate
  // permanent occupied cells after the robot moves through the scene.
  for (auto it = world_obstacle_last_seen_sec_.begin(); it != world_obstacle_last_seen_sec_.end(); ) {
    const double age_sec = now_sec - it->second;
    const bool stale =
      (params_.ugv_obstacle_persistence_sec <= 0.0) ? (age_sec > 1e-6) :
      (age_sec > params_.ugv_obstacle_persistence_sec);
    if (stale) {
      it = world_obstacle_last_seen_sec_.erase(it);
    } else {
      ++it;
    }
  }

  const double local_min_x = robot_x - half_window;
  const double local_min_y = robot_y - half_window;
  for (int gy = 0; gy < height; ++gy) {
    for (int gx = 0; gx < width; ++gx) {
      const double wx = local_min_x + (static_cast<double>(gx) + 0.5) * params_.ugv_local_plan_resolution_m;
      const double wy = local_min_y + (static_cast<double>(gy) + 0.5) * params_.ugv_local_plan_resolution_m;
      const int wgx = static_cast<int>(std::floor(wx / params_.ugv_local_plan_resolution_m));
      const int wgy = static_cast<int>(std::floor(wy / params_.ugv_local_plan_resolution_m));
      const int64_t key = world_key(wgx, wgy);

      const auto occ_it = world_obstacle_last_seen_sec_.find(key);
      if (occ_it != world_obstacle_last_seen_sec_.end()) {
        occupied[flatten({gx, gy}, width)] = 1;
        continue;
      }

      const auto free_it = world_free_last_seen_sec_.find(key);
      if (free_it != world_free_last_seen_sec_.end()) {
        (void)free_it;
        // Sticky-free behavior: keep free evidence until occupied is observed again.
        free_evidence[flatten({gx, gy}, width)] = 1;
      }
    }
  }

  ++prune_counter_;
  if ((prune_counter_ % 500) == 0) {
    // Keep both maps bounded by pruning cells far from the robot.
    const double max_keep_dist = 2.5 * params_.ugv_local_plan_window_size_m;
    const double max_keep_dist_sq = max_keep_dist * max_keep_dist;

    auto prune_by_distance = [&](std::unordered_map<int64_t, double> & map) {
      for (auto it = map.begin(); it != map.end(); ) {
        const int gy = static_cast<int>(static_cast<uint32_t>(it->first & 0xffffffff));
        const int gx = static_cast<int>(it->first >> 32);
        const double wx = (static_cast<double>(gx) + 0.5) * params_.ugv_local_plan_resolution_m;
        const double wy = (static_cast<double>(gy) + 0.5) * params_.ugv_local_plan_resolution_m;
        const double dx = wx - robot_x;
        const double dy = wy - robot_y;
        if ((dx * dx + dy * dy) > max_keep_dist_sq) {
          it = map.erase(it);
        } else {
          ++it;
        }
      }
    };

    prune_by_distance(world_free_last_seen_sec_);
    prune_by_distance(world_obstacle_last_seen_sec_);
  }

  // Snapshot the raw (pre-inflation) grid for controller avoidance.
  const std::vector<uint8_t> occupied_raw = occupied;

  const double footprint_radius_m =
    0.5 * std::hypot(params_.robot_body_length_m, params_.robot_body_width_m);
  const int footprint_cells =
    static_cast<int>(std::ceil(footprint_radius_m / params_.ugv_local_plan_resolution_m));
  const int extra_inflation_cells = static_cast<int>(
    std::ceil(std::max(0.0, params_.ugv_local_plan_inflation_m) / params_.ugv_local_plan_resolution_m));
  const int effective_inflation_cells =
    footprint_cells + extra_inflation_cells;

  if (effective_inflation_cells > 0) {
    std::vector<uint8_t> inflated = occupied;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (occupied[flatten({x, y}, width)] == 0) {
          continue;
        }
        for (int dy = -effective_inflation_cells;
          dy <= effective_inflation_cells; ++dy)
        {
          for (int dx = -effective_inflation_cells;
            dx <= effective_inflation_cells; ++dx)
          {
            GridIndex n{x + dx, y + dy};
            if (in_bounds(n, width, height)) {
              inflated[flatten(n, width)] = 1;
            }
          }
        }
      }
    }
    occupied.swap(inflated);
  }

  // Ensure a small free-space bubble around robot center to prevent self-trapping
  // due inflated obstacles very close to the platform.
  const GridIndex center{width / 2, height / 2};
  const int clear_radius_cells = std::max(1, footprint_cells + 1);
  for (int dy = -clear_radius_cells; dy <= clear_radius_cells; ++dy) {
    for (int dx = -clear_radius_cells; dx <= clear_radius_cells; ++dx) {
      if (dx * dx + dy * dy > clear_radius_cells * clear_radius_cells) {
        continue;
      }
      GridIndex c{center.x + dx, center.y + dy};
      if (in_bounds(c, width, height)) {
        occupied[flatten(c, width)] = 0;
      }
    }
  }

  out.resolution = params_.ugv_local_plan_resolution_m;
  out.width = width;
  out.height = height;
  out.origin_x = robot_x - half_window;
  out.origin_y = robot_y - half_window;
  out.occupied = occupied;
  out.occupied_raw = occupied_raw;

  out.local_map.header.stamp = odom.header.stamp;
  out.local_map.header.frame_id = odom.header.frame_id.empty() ? params_.odom_frame : odom.header.frame_id;
  out.local_map.info.resolution = static_cast<float>(out.resolution);
  out.local_map.info.width = static_cast<uint32_t>(out.width);
  out.local_map.info.height = static_cast<uint32_t>(out.height);
  out.local_map.info.origin.position.x = out.origin_x;
  out.local_map.info.origin.position.y = out.origin_y;
  out.local_map.info.origin.position.z = 0.0;
  out.local_map.info.origin.orientation.w = 1.0;
  out.local_map.data.assign(static_cast<size_t>(out.width * out.height), -1);
  for (int i = 0; i < out.width * out.height; ++i) {
    if (out.occupied[static_cast<size_t>(i)] != 0) {
      out.local_map.data[static_cast<size_t>(i)] = 100;
    } else if (free_evidence[static_cast<size_t>(i)] != 0) {
      out.local_map.data[static_cast<size_t>(i)] = 0;
    }
  }

  // Build obstacle-band point cloud for visualization (odom frame, xyz).
  {
    const size_t n = obstacle_pts_x.size();
    sensor_msgs::msg::PointCloud2 & cloud = out.obstacle_cloud;
    cloud.header = out.local_map.header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(n);
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::msg::PointField fx, fy, fz;
    fx.name = "x"; fx.offset = 0;  fx.datatype = sensor_msgs::msg::PointField::FLOAT32; fx.count = 1;
    fy.name = "y"; fy.offset = 4;  fy.datatype = sensor_msgs::msg::PointField::FLOAT32; fy.count = 1;
    fz.name = "z"; fz.offset = 8;  fz.datatype = sensor_msgs::msg::PointField::FLOAT32; fz.count = 1;
    cloud.fields = {fx, fy, fz};
    cloud.point_step = 12;
    cloud.row_step = static_cast<uint32_t>(12 * n);
    cloud.data.resize(12 * n);

    auto * dst = cloud.data.data();
    for (size_t i = 0; i < n; ++i) {
      std::memcpy(dst + i * 12,     &obstacle_pts_x[i], 4);
      std::memcpy(dst + i * 12 + 4, &obstacle_pts_y[i], 4);
      std::memcpy(dst + i * 12 + 8, &obstacle_pts_z[i], 4);
    }
  }

  out.valid = true;
  return out;
}

}  // namespace simple_nav_3d
