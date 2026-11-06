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

## 安全なTEB経路を保持する比較モード

`reference_mode:=latest retain_safe_path:=true` で有効化する。既定値はfalse。
通常のTEB速度制御とは異なり、TEBで生成した経路を採用した後は、
専用のpure-pursuit追従器が同じ経路を追う。TEBは経路の再取得時に使用する。
NoMaDの新しい経路は受け取り続ける。

毎制御周期に、現在姿勢から保持経路の残り全体を、旋回補間・機体外形・
footprint_paddingとmin_obstacle_dist分を広げた外形で検査する（角部は保守的）。LETHAL、未知、地図外、外形内部の障害物も
拒否する。さらに追従指令の旋回軌跡を検査する。速度・角速度・指令変化量は
TEB設定のmax_vel_x、max_vel_x_backwards、max_vel_theta、acc_lim_x、acc_lim_thetaを使用する。
この検査は記録・受信できた地図に対するもので、未観測障害物や制動を保証しない。

次の場合は保持を解除し、ゼロ速度を出してからTEBで再計画する。
- 残経路または指令軌跡に障害物が入る、地図がcurrentでない、TFが取得できない
- 経路からretention_max_deviation（既定0.20 m）を超えて逸脱
- 終端へ0.12 m以内に到達、追従できない形状、または前後進の切り替わり位置へ到達
- retention_max_duration（既定3秒）を経過
- 新しい参照終点が採用時からretention_goal_shift（既定0.75 m）以上変化、または終端方位が0.78 rad超変化
- 空・無効入力、path_timeout>0の期限切れ、時計巻き戻り

追従の先読み距離はretention_lookahead（既定0.30 m）。保持経路は
`/placenav_teb_bridge/retained_safe_path`、状態は`/placenav_teb_bridge/diagnostics`に出す。
RVizでは保持経路とTEB local_planを別のPathとして表示する。
再計画中はTEBの計算した速度を直接出さず、安全な候補を受信・検査するまで停止する。
保持中はTEBのlocal_planは更新されないため、保持経路の表示を走行対象として見る。

PC比較例（出力先は比較用）：
```bash
roslaunch placenav_teb_bridge bridge.launch \
  reference_mode:=latest retain_safe_path:=true \
  cmd_vel_topic:=/test_teb_path/cmd_vel
```
統合起動でも`roslaunch rwrc26_bringup navigation.launch retain_safe_path:=true`を指定できる。
全候補不適切で停止する場面を安全な経路の継続だけで解消するものではない。
実機での安定性は未検証。まず同じbagで保持ON/OFFを比較する。

検証：`test_retained_path`は外形内部・未知・地図外・経路後半・旋回掃引・後退・
逸脱・速度制限・無効入力を確認する。`tests/check_retained_path_ros.py`は独立した
localhost masterで採用、軽微な更新での継続、障害物出現で停止、再開、空入力、入力期限切れを確認する。
