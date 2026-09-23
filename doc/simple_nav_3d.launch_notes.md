# simple_nav_3d.launch.py — design notes and history

The long comments of `launch/simple_nav_3d.launch.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [launch_setup](#launch_setup) — 15

## launch_setup

### launch-peers

**The peers launch argument** — attached to `peers_raw = LaunchConfiguration("peers").perform(context)` (line 34)

```text
Comma-separated list of peer robot names (e.g. "rama,charlie") for the
multi-robot DSCovox topology. Empty (default) preserves single-robot
behaviour bit-for-bit: dscovox_node only subscribes to its own
/<robot>/scovox_node/scovox_bin. With peers set, the merger also
subscribes to /<peer>/scovox_node/scovox_bin for each peer, giving
this robot a per-robot fused view of the whole team's mapping.
```

### launch-peer-bin-topic-pattern

**Peer scovox topic pattern and comms** — attached to `peer_bin_pattern = LaunchConfiguration(` (line 43)

```text
Where this robot's merger listens for each PEER's scovox binary. The
default is the peer's own publisher, i.e. today's behaviour unchanged.
Under the message-level comms emulator the peer streams arrive instead on
its relayed copies, and repointing the merger at those is the whole
mechanism by which map sharing obeys the radio model: left on the direct
topics, every robot merges every peer's map instantly and perfectly, and
the comms arm of an experiment silently degenerates into the control arm
while still producing a full set of plausible results.

Placeholders: {peer} (or {robot}) = the peer being subscribed to, {self} =
this robot. The emulator's relay convention is
  "/{self}/rx/{peer}/scovox_node/scovox_bin".

Self is deliberately NOT patterned. A robot's own binary never crosses a
radio link, and routing it through an rx topic would leave the robot
unable to see its own map whenever its own link was down — every arm would
then measure a mapping failure rather than a comms one.
```

### launch-voxel-resolution-cost

**Lidar voxel resolution and its cost** — attached to `voxel_res = float(LaunchConfiguration("voxel_resolution_m").perform(context))` (line 63)

```text
scovox voxel edge length (m) on the lidar path. Default 0.10 = the value
this launch has always used.

It is the single biggest cost driver in a long run, and the cost is cubic:
the lidar path carves free space along the WHOLE ray (carve_band -1) out to
max_range 20 m, so every beam writes ~200 voxels at 0.10 m. Measured on a
2-robot flatforest run, the fused map reached 12.7M voxels by t=550 s and
was still growing linearly with explored area. Everything that touches the
map scales with it -- dscovox integration, the full ScovoxMap publish, and
the planner's ingest plus whole-grid walk -- so past a few million voxels
the planner's map subscription simply stops keeping up. That failure is
silent and dangerous: one robot ran for three minutes on a frozen map,
still driving, still logging steps, its coverage curve flat while its
teammate's kept climbing.

Raising this 0.10 -> 0.20 cuts the count ~8x. It also shrinks the
ScovoxMapBinary delta payload by about as much, which matters beyond
performance whenever the run is under a comms model: the deltas ARE what
the radio carries, so severity calibration must be done at the resolution
the campaign will actually run at, never carried over from another.
```

### launch-global-planning-map-scovox

**The world-fixed exploration planning map** — attached to `plan_glob_size = float(` (line 91)

```text
Second, world-fixed planning map published by scovox_node for an
EXPLORATION planner (explo_planner), on ~/global_planning_map.

It cannot share ~/planning_map with the local nav planner: that one is a
20 m robot-centred crop, and simple_nav_3d's local planner has no window
param of its own — the map extent IS its window, so widening it would put
the whole world through the local A*/corridor mask on the control path.
The exploration planner needs the opposite: a fixed envelope covering the
ROI, because it rejects candidates whose cell is out of bounds and
measures coverage termination over the ROI clipped to the grid. Hence two
publishers over the same voxel grid.

Sizing: the envelope is world-fixed and centred on the world origin, and
so is the planner's ROI, so the side must be at least the ROI SIDE
(= 2 x roi half-extent) to cover it, plus margin for a robot that drifts
outside the ROI — its own cell must be in bounds or the reachability
flood starts nowhere. Default 0 = off, which is what every
non-exploration run wants.
```

### launch-global-planning-map-dscovox

**The merger's fused global planning map** — attached to `dscovox_global_plan_params = {}` (line 132)

```text
The SAME world-fixed envelope, published by the MERGER over the FUSED
grid. scovox_node's copy above sees only what this robot measured itself;
dscovox_node's sees the team map, which is the domain the exploration
planner's candidates and its coverage-termination test actually live in.
Pointing the planner at the local map while it plans over the fused one
is a map-domain mismatch: a candidate in ground the PARTNER surveyed is
unknown-and-therefore-unreachable on the local map, so it is rejected.
Identical envelope and resolution to the scovox copy on purpose — the
two are meant to be comparable cell-for-cell, and the planner ROI is
sized against this side length.

NOTE the topic: ~/global_planning_map, never ~/planning_map. Both the
exploration planner and — since the generation-5 fix — the nav global
planner subscribe to this name. ~/planning_map means the 20 m rolling
crop from scovox_node, which the LOCAL nav planner consumes; the two are
different maps with different jobs and must keep different names.
```

### launch-hard-stop-distance

**Recovery hard-stop distance value** — attached to `"ugv.avoidance_hard_stop_distance_m": 0.4,` (line 260)

```text
Generation 5: raised 0.15 -> 0.4. At 0.15 the recovery state
machine was unreachable — it did not fire once in 72 campaign
robot-runs, including runs where a robot sat immobilised for ten
minutes, because the slowdown scaling above it
  scale = (clearance - hard_stop) / (slowdown - hard_stop)
drives commanded speed to zero as clearance approaches the
threshold, so the robot creeps to a halt just outside it and the
trigger is never crossed. 0.4 sits inside the band the robot can
still reach under power. It must stay strictly below
avoidance_slowdown_distance_m or the span goes non-positive.
```

### launch-scovox-node-dscovox

**scovox_node in dscovox mode** — attached to `nodes.append(Node(` (line 338)

```text
scovox_node: per-robot persistent local map.
- Publishes ScovoxMapBinary snapshots to the dscovox merger. Only
  voxels touched since the last publish are shipped (dirty-set
  tracking); a fresh dscovox connection triggers a full snapshot.
- planning_map is published as a 20x20m rolling crop centered on
  the robot pose (mode=rolling). The underlying voxel grid is fully
  persistent — only the publication is windowed.
```

### launch-dscovox-node-merger

**dscovox_node as per-robot merger** — attached to `dscovox_inputs = dscovox_input_topics()` (line 376)

```text
dscovox_node: merges per-robot scovox snapshots into a global map.
Stores one source grid per robot keyed by header.frame_id of incoming
binaries; rebuilds the fused grid by additive Beta-conjugate
consensus merge across sources at publish_rate_hz.
Multi-robot fused view: subscribe to every team member's scovox_bin
(self + peers). Each robot's dscovox_node is its own per-robot
consensus merger -- there is no central merger. With peers=[] the
input list collapses to the single-robot case.
```

### launch-bin-qos-depth-dscovox

**Merger QoS depth matches the relay** — attached to `"scovox_bin_qos_depth": 4000,` (line 398)

```text
Match the comms emulator's rx_qos_depth. On reconnect the
relay releases a whole outage's backlog in one pass; a
shallower reader here silently discards the excess and the
fused map is permanently holed with no counter recording it.
Harmless without the emulator — it is only a history bound.

500 -> 4000 (2026-08-16). At 500 this was the SHALLOW end of
the chain for the 250 stems/ha world: 861 s outages queue
~1720 deltas at the ~2 Hz share rate, so all three dense cells
finished with their two merged maps 1.5-1.8 % apart and every
gate green. Keep this equal to comms_sim_params.yaml's
rx_qos_depth — raising only one end fixes nothing, because the
burst is discarded at whichever end is shallower.
```

### launch-dscovox-no-planning-map

**Removed dscovox planning_map params** — attached to `}],` (line 413)

```text
REMOVED: a planning_map_* block used to be passed here. Every
key in it was undeclared in dscovox_node, so ROS accepted the
values and nothing read them, while the nav global planner
subscribed to the ~/planning_map topic they described. Those
parameters made a dead topic look configured — the single
most misleading thing in this launch file. The global planner
now subscribes to dscovox's real ~/global_planning_map, whose
geometry comes from dscovox_global_plan_params above.
```

### launch-dscovox-lidar-topology

**The dscovox_lidar mapping topology** — attached to `scovox_fine_extra = {}` (line 425)

```text
Same rolling-mapper + per-robot-merger topology as "dscovox", but
scovox_node integrates the lidar cloud (geometric Beta occupancy,
no semantics). Sensor model mirrors
scovox/config/scovox_lidar_geometric.yaml; share cadence mirrors
scovox_robot_share.yaml (2 Hz coalesced deltas on the wire).
```

### launch-bin-qos-depth-lidar

**Lidar merger QoS depth matches relay** — attached to `"scovox_bin_qos_depth": 4000,` (line 517)

```text
Match the comms emulator's rx_qos_depth. On reconnect the
relay releases a whole outage's backlog in one pass; a
shallower reader here silently discards the excess and the
fused map is permanently holed with no counter recording it.
Harmless without the emulator — it is only a history bound.

500 -> 4000 (2026-08-16). At 500 this was the SHALLOW end of
the chain for the 250 stems/ha world: 861 s outages queue
~1720 deltas at the ~2 Hz share rate, so all three dense cells
finished with their two merged maps 1.5-1.8 % apart and every
gate green. Keep this equal to comms_sim_params.yaml's
rx_qos_depth — raising only one end fixes nothing, because the
burst is discarded at whichever end is shallower.
```

### launch-costmap-node-role

**What the costmap node does per mode** — attached to `costmap_extra = {}` (line 536)

```text
Costmap node:
- dscovox mode: the global planner subscribes directly to the dscovox
  planning_map and the local planner subscribes directly to the
  scovox_node planning_map, so the costmap does NOT forward any external
  map. It only builds the sensor-based local_map used by the controller
  for emergency stops.
- scovox mode: forward scovox_node's planning_map as the global planner's
  input (no separate local planner in this mode).
```

### launch-global-planner-map

**Which map the global planner reads** — attached to `global_planner_extra = {"pipeline.role": "global"}` (line 558)

```text
In dscovox mode the global planner reads dscovox's merged planning map
directly (no costmap forwarding). For UAV it uses the 3D GetRegion
service instead of a 2D map. In other modes it falls back to the
costmap-built global map via topics.planning_map default.

The map is ~/global_planning_map, NOT ~/planning_map. dscovox publishes
two grids and only the former is usable for global planning: the latter
is body-centred, so its origin moves with the robot and a plan is stale
the moment the robot drives. Pointing this node at ~/planning_map was a
silent no-op — nothing has ever published that name — and it left the
global planner inert for the whole campaign history while the local
planner drove alone on a 20 m horizon. That is the direct cause of the
local-minimum traps in sections 28 and 32.9. See the starvation warning
in simple_nav_planner_node.cpp, which now makes the same mistake loud.
```

### launch-local-planner-corridor

**Local planner corridor refinement** — attached to `if mapping in ("dscovox", "dscovox_lidar") and not is_uav:` (line 594)

```text
Reads scovox_node's 20x20m rolling planning_map AND the global planner's
output path. On each replan it slices the global path to the segment
inside the local window, uses that exit point as its A* target, and
masks the local map to a corridor of half-width ugv.local_corridor_radius_m
around the slice. This refines global within the local window without
contradicting it. If the corridor is blocked (new obstacle on the global
path) the local planner retries with a free A*. Side-flip rejection is
still disabled here because the corridor already serves the same role.
```
