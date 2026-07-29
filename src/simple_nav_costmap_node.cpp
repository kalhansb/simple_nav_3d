#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "simple_nav_3d/mapping/local_mapper.hpp"
#include "simple_nav_3d/parameters.hpp"

namespace simple_nav_3d
{

using namespace std::chrono_literals;

class SimpleNavCostmapNode : public rclcpp::Node
{
public:
  SimpleNavCostmapNode()
  : Node("simple_nav_costmap")
  {
    params_ = load_and_validate_params(*this);
    mapper_ = std::make_unique<LocalMapper>(params_);

    init_global_map();

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(params_.local_map_topic, map_qos);
    raw_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(params_.local_map_raw_topic, map_qos);
    global_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(params_.global_map_topic, map_qos);
    obstacle_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      params_.local_map_topic + "_obstacle_cloud", rclcpp::SensorDataQoS());

    global_map_.header.stamp = now();
    global_map_pub_->publish(global_map_);

    // Always run sensor pipeline: build local map from odom + point cloud
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        latest_odom_stamp_ = rclcpp::Time(msg->header.stamp);
        last_odom_time_ = now();
        has_odom_ = true;
        RCLCPP_DEBUG(
          get_logger(),
          "incoming odom stamp=%d.%09u",
          static_cast<int>(msg->header.stamp.sec),
          static_cast<unsigned int>(msg->header.stamp.nanosec));
      });

    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.points_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        latest_points_msg_ = msg;
        latest_points_stamp_ = rclcpp::Time(msg->header.stamp);
        last_points_time_ = now();
        has_points_ = true;
        RCLCPP_DEBUG(
          get_logger(),
          "incoming points stamp=%d.%09u",
          static_cast<int>(msg->header.stamp.sec),
          static_cast<unsigned int>(msg->header.stamp.nanosec));
      });

    timer_ = rclcpp::create_timer(this, get_clock(), 50ms, [this]() { on_tick(); });

    // Optionally also subscribe to an external planning map (e.g. from dscovox)
    // and use it directly as the global planning map (replaces sensor-accumulated global map).
    use_external_global_map_ = !params_.external_local_map_topic.empty();
    if (use_external_global_map_) {
      external_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        params_.external_local_map_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          // Publish the dscovox planning map directly as the global map for the planner.
          // Re-stamp with odom frame so the planner can use odom coordinates.
          nav_msgs::msg::OccupancyGrid global = *msg;
          global.header.frame_id = params_.odom_frame;
          global_map_pub_->publish(global);
        });

      RCLCPP_INFO(
        get_logger(),
        "costmap node started: odom=%s points=%s local_map=%s global_map=%s external_map=%s",
        params_.odom_topic.c_str(),
        params_.points_topic.c_str(),
        params_.local_map_topic.c_str(),
        params_.global_map_topic.c_str(),
        params_.external_local_map_topic.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "costmap node started: odom=%s points=%s local_map=%s global_map=%s",
        params_.odom_topic.c_str(),
        params_.points_topic.c_str(),
        params_.local_map_topic.c_str(),
        params_.global_map_topic.c_str());
    }
  }

private:
  void init_global_map()
  {
    const int width = static_cast<int>(std::round(
        params_.ugv_global_map_size_m / params_.ugv_global_map_resolution_m));
    const int height = width;

    global_map_.header.frame_id = params_.odom_frame;
    global_map_.info.resolution = static_cast<float>(params_.ugv_global_map_resolution_m);
    global_map_.info.width = static_cast<uint32_t>(std::max(1, width));
    global_map_.info.height = static_cast<uint32_t>(std::max(1, height));
    global_map_.info.origin.position.x = params_.ugv_global_map_origin_x_m;
    global_map_.info.origin.position.y = params_.ugv_global_map_origin_y_m;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;

    const size_t n = static_cast<size_t>(global_map_.info.width * global_map_.info.height);
    global_map_.data.assign(n, -1);
    global_hit_count_.assign(n, 0);
  }

  void integrate_local_into_global(const nav_msgs::msg::OccupancyGrid & local)
  {
    const double g_res = global_map_.info.resolution;
    const double g_ox = global_map_.info.origin.position.x;
    const double g_oy = global_map_.info.origin.position.y;
    const int g_w = static_cast<int>(global_map_.info.width);
    const int g_h = static_cast<int>(global_map_.info.height);

    const double l_res = local.info.resolution;
    const double l_ox = local.info.origin.position.x;
    const double l_oy = local.info.origin.position.y;
    const int l_w = static_cast<int>(local.info.width);
    const int l_h = static_cast<int>(local.info.height);

    const int min_hits = params_.ugv_global_map_min_hits;

    for (int ly = 0; ly < l_h; ++ly) {
      for (int lx = 0; lx < l_w; ++lx) {
        const int l_idx = ly * l_w + lx;
        const int8_t l_val = local.data[static_cast<size_t>(l_idx)];
        if (l_val < 0) {
          continue;
        }

        const double wx = l_ox + (static_cast<double>(lx) + 0.5) * l_res;
        const double wy = l_oy + (static_cast<double>(ly) + 0.5) * l_res;
        const int gx = static_cast<int>(std::floor((wx - g_ox) / g_res));
        const int gy = static_cast<int>(std::floor((wy - g_oy) / g_res));
        if (gx < 0 || gy < 0 || gx >= g_w || gy >= g_h) {
          continue;
        }

        const size_t g_idx = static_cast<size_t>(gy * g_w + gx);
        int8_t & g_val = global_map_.data[g_idx];

        if (l_val >= 100) {
          // Accumulate obstacle evidence. Only promote to occupied after min_hits
          // consistent observations — filters single-frame sensor noise.
          uint16_t & hits = global_hit_count_[g_idx];
          if (hits < std::numeric_limits<uint16_t>::max()) {
            ++hits;
          }
          if (hits >= static_cast<uint16_t>(min_hits)) {
            g_val = 100;  // Permanent once confirmed.
          }
        } else if (l_val == 0 && g_val != 100) {
          // Free evidence can mark unknown cells as free, but never overwrites
          // a confirmed obstacle — the global map accumulates permanently.
          g_val = 0;
        }
      }
    }

    global_map_.header.stamp = local.header.stamp;
  }

  void on_tick()
  {
    const rclcpp::Time t = now();
    const bool odom_stale = !has_odom_ || (t - last_odom_time_).seconds() > params_.sensor_timeout_sec;
    const bool points_stale = !has_points_ || (t - last_points_time_).seconds() > params_.sensor_timeout_sec;
    if (odom_stale || points_stale) {
      if (!use_external_global_map_) {
        global_map_.header.stamp = t;
        global_map_pub_->publish(global_map_);
      }
      return;
    }

    const double sync_dt_sec = std::abs((latest_odom_stamp_ - latest_points_stamp_).seconds());
    if (sync_dt_sec > params_.sensor_sync_window_sec) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "sync reject: |odom-points|=%.3f s > window=%.3f s (odom=%u.%09u points=%u.%09u)",
        sync_dt_sec,
        params_.sensor_sync_window_sec,
        static_cast<int>(latest_odom_.header.stamp.sec),
        static_cast<unsigned int>(latest_odom_.header.stamp.nanosec),
        static_cast<int>(latest_points_msg_->header.stamp.sec),
        static_cast<unsigned int>(latest_points_msg_->header.stamp.nanosec));
      return;
    }

    const MapSnapshot map = mapper_->build_map(latest_odom_, latest_points_msg_);
    if (map.valid) {
      map_pub_->publish(map.local_map);
      obstacle_cloud_pub_->publish(map.obstacle_cloud);

      // Publish the raw (pre-inflation) map for controller avoidance.
      nav_msgs::msg::OccupancyGrid raw_grid = map.local_map;
      for (size_t i = 0; i < map.occupied_raw.size() && i < raw_grid.data.size(); ++i) {
        if (map.occupied_raw[i] != 0) {
          raw_grid.data[i] = 100;
        } else if (raw_grid.data[i] == 100) {
          // Cell is inflated-only, not a real obstacle — mark unknown for raw.
          raw_grid.data[i] = 0;
        }
      }
      raw_map_pub_->publish(raw_grid);

      if (!use_external_global_map_) {
        integrate_local_into_global(map.local_map);
        global_map_pub_->publish(global_map_);
      }
    }
  }

  NodeParameters params_;
  std::unique_ptr<LocalMapper> mapper_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
  bool use_external_global_map_{false};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr external_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_odom_{false};
  bool has_points_{false};
  nav_msgs::msg::Odometry latest_odom_;
  std::shared_ptr<const sensor_msgs::msg::PointCloud2> latest_points_msg_;
  rclcpp::Time latest_odom_stamp_;
  rclcpp::Time latest_points_stamp_;
  nav_msgs::msg::OccupancyGrid global_map_;
  std::vector<uint16_t> global_hit_count_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_points_time_;
};

}  // namespace simple_nav_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_nav_3d::SimpleNavCostmapNode>());
  rclcpp::shutdown();
  return 0;
}
