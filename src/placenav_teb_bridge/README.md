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
