# PlaceNav TEB Bridge

This ROS 1 node uses the newest PlaceNav GNM path as the normal TEB global
reference and manages obstacle avoidance with three states.

1. `NORMAL`: continuously accept the newest `/placenav/gnm_path`.
2. `AVOIDING`: when a lethal cost blocks the forward PlaceNav corridor,
   promote TEB's current `local_plan` to the active global reference. Continue
   receiving PlaceNav paths, but retain them only as the future recovery target.
3. `RECOVERING`: after the obstacle has remained clear, restore the newest
   retained PlaceNav path. Return to `NORMAL` after a stable recovery interval.

## Debug topics

- `~navigation_state` (`std_msgs/String`): `NORMAL`, `AVOIDING`, or `RECOVERING`.
- `~active_global_path` (`nav_msgs/Path`): the path currently passed to TEB.
- `~/TebLocalPlannerROS/local_plan` (`nav_msgs/Path`): TEB's optimized local
  trajectory, also used as the avoidance-path seed.

In RViz, compare `/placenav/gnm_path`,
`/placenav_teb_bridge/active_global_path`, and
`/placenav_teb_bridge/TebLocalPlannerROS/local_plan` with different colors.

## State parameters

- `obstacle_trigger_distance`: forward detection radius in metres.
- `obstacle_corridor_width`: maximum distance from the latest PlaceNav path
  for a costmap cell to trigger avoidance.
- `obstacle_min_forward_distance`: ignore costs behind or immediately beneath
  the robot.
- `obstacle_cost_threshold`: minimum Costmap2D cost that triggers avoidance.
- `avoidance_clear_time`: clear duration before restoring PlaceNav.
- `recovery_stable_time`: duration in `RECOVERING` before returning to normal.
- `minimum_avoidance_plan_poses`: reject incomplete TEB local paths.

## Stage check

```bash
roslaunch placenav_teb_bridge stage_closed_loop_test.launch \
  with_obstacle:=true waypoint_step:=0.8
```

Monitor the state and promoted path:

```bash
rostopic echo /placenav_teb_bridge/navigation_state
rostopic echo /placenav_teb_bridge/active_global_path
```

## P1②: reference and stop-cause comparison

`reference_mode:=state_machine` (default) retains the existing NORMAL / AVOIDING /
RECOVERING reference switching. `reference_mode:=latest` always accepts the newest
PlaceNav reference (subject to the existing position/orientation equivalence
thresholds), never promotes a TEB local plan, and publishes state `TRACKING`.
**TEB uses its costmap in both modes, including NORMAL.** The obstacle corridor
parameters control reference switching, not whether TEB avoids obstacles.
The detector still runs for diagnostics in latest mode but does not change the
reference. The costmap and velocity/clearance parameters are unchanged.

Run the same input bag, map, NoMaD and VPR settings for each trial. Restart the
bridge and navigation before replaying each interval. Use an isolated ROS master
and `/test_teb_path/cmd_vel`; do not connect a vehicle controller.

```bash
# Existing reference switching
roslaunch placenav_teb_bridge bridge.launch \
  reference_mode:=state_machine shrink_horizon_backup:=true \
  path_timeout:=1.0 cmd_vel_topic:=/test_teb_path/cmd_vel

# Always use the latest PlaceNav reference
roslaunch placenav_teb_bridge bridge.launch \
  reference_mode:=latest shrink_horizon_backup:=true \
  path_timeout:=1.0 cmd_vel_topic:=/test_teb_path/cmd_vel
```

First compare only `reference_mode`. Then repeat the better-understood mode with
`shrink_horizon_backup:=false` to isolate horizon reduction. This does not disable
TEB's footprint feasibility check. The launch argument overrides the YAML value;
its default is true, matching the current experiment configuration.

`path_timeout` defaults to 0 (disabled for compatibility). Positive values limit
wall-clock time since receipt of a valid input path, checked before/after TEB on
control timer ticks. This is not image-age validation or an independent watchdog:
when simulated time is paused, ROS timer callbacks also pause.

Common fixes applied to both modes for a fair comparison:
- Empty/invalid input clears the active/reference/avoidance plans and publishes a
  stop instead of retaining the last valid command guidance.
- A new path equivalent to an already-completed path is submitted again to TEB;
  an inactive plan is no longer skipped by the equivalence check.

### Diagnostics

```bash
rostopic echo /placenav_teb_bridge/diagnostics
```

This `diagnostic_msgs/DiagnosticArray` is published on each controller tick:
- `message`: `NO_INPUT_PATH`, `INVALID_INPUT_PATH`, `INPUT_TIMEOUT`,
  `ROBOT_TF_UNAVAILABLE`, `SET_PLAN_REJECTED`, `GOAL_REACHED`, `TEB_FAILED`,
  `NONFINITE_COMMAND`, `TEB_ZERO_COMMAND`, or `COMMAND_VALID`.
- `teb_outcome` / `teb_message`: TEB's public return code and explanation.
  It distinguishes plan-transform failure, no local plan, infeasible trajectory,
  and invalid velocity when TEB supplies an explanation. Some failures (such as
  divergence in this TEB version) have an empty explanation: inspect `/rosout`.
  An infeasible trajectory is not evidence of one specific collision location.
- `cells_above_cost_threshold` → `cells_in_forward_radius` →
  `cells_near_path_points`: counts through the existing detection conditions.
  Distance is to sampled reference points, not line segments. A short path can
  therefore miss cells. Unknown cells are excluded by the original detector.
- `detection_status`: `NO_PATH`, `ROBOT_TF_UNAVAILABLE`, `PATH_FRAME_MISMATCH`,
  `CLEAR`, `BLOCKED`. A frame mismatch is not treated as a clear corridor result.
- `waiting_for_teb_seed`: detection passed but no usable TEB seed was received.
- `reference_mode`, `state`, `active_source`, input/active poses and lengths,
  receipt age, input stamp, output velocity.
- `reference_goal_cell_cost`: raw 0–255 cost at the active reference endpoint,
  `outside_map`, or `unavailable`. This is not a footprint test or the endpoint
  of TEB's internally shortened plan.
- `shrink_horizon_enabled`: configured enable flag, **not actual activation**.
  Actual activation is logged by TEB as `Activating reduced horizon backup mode`.

Record at least these outputs alongside the input bag and parameter dump:

```bash
rosbag record --lz4 -O teb_reference_comparison.bag \
  /clock /tf /tf_static /rosout \
  /toponav/viz_info /toponav/localization_diagnostics /placenav/gnm_path \
  /placenav_teb_bridge/diagnostics /placenav_teb_bridge/navigation_state \
  /placenav_teb_bridge/active_global_path \
  /placenav_teb_bridge/TebLocalPlannerROS/local_plan \
  /placenav_teb_bridge/local_costmap/costmap \
  /placenav_teb_bridge/local_costmap/costmap_updates /test_teb_path/cmd_vel
```

Run `tests/check_teb_reference_modes_ros.py --bridge <installed executable>
--mode latest --port <unused port>` (and `--mode state_machine` with a different
unused port) in the sourced ROS environment. These synthetic tests verify
reference adoption, diagnostics and stop/restart behavior. They do not establish
Tsukuba bag performance or closed-loop obstacle avoidance success.
