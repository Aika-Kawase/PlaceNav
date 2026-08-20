# WHILL ROS 1 integration

This integration targets the laboratory WHILL system used for the Tsukuba
Challenge. The first milestone is inference without publishing a command to
the robot.

## Confirmed live interfaces

| Purpose | Topic | Type | Observed rate |
| --- | --- | --- | --- |
| USB camera | `/usb_cam/image_raw` | `sensor_msgs/Image` | about 10 Hz |
| Relayed odometry | `/odom` | `nav_msgs/Odometry` | about 88 Hz |
| Raw WHILL odometry | `/whill/odom` | `nav_msgs/Odometry` | about 88--89 Hz |
| MotionDecision input | `/local_path/cmd_vel` | `geometry_msgs/Twist` | no publisher before navigation starts |
| WHILL command | `/whill/controller/cmd_vel` | `geometry_msgs/Twist` | about 20 Hz |
| Emergency stop | `/emergency_stop` | `std_msgs/Bool` | no publisher observed |

The live `/usb_cam/camera_info` message has valid width and height, but its
`K`, `R`, `P`, and `D` arrays are zero. It is not used by PlaceNav inference.
The image frame is `use_cam`; a `base_link -> use_cam` static transform has
not yet been confirmed.

`/cmd`, `/cmd_vel`, and `/cmd_vel_stamped` were not present in the current
runtime graph. The stamped command used previously was a WVN synchronization
interface and is not required by PlaceNav.

## Safety boundary

The WHILL PlaceNav configuration publishes the PD controller output to
`/cmd_vel_nominal`. It must not be remapped directly to
`/whill/controller/cmd_vel`.

The intended command path is:

```text
PlaceNav -> /waypoint -> PD controller -> /cmd_vel_nominal
         -> Safety Layer -> /local_path/cmd_vel
         -> MotionDecision -> /whill/controller/cmd_vel -> WHILL
```

The current PD controller repeats its last command. Do not connect it to the
robot until a command timeout and the other Safety Layer checks are active.

## Container checks

Run these inside the ROS Noetic development container:

```bash
echo "$ROS_MASTER_URI"
echo "$ROS_IP"
echo "$ROS_HOSTNAME"
rostopic list

nvidia-smi
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.version.cuda)"
python3 -c "import cv2, h5py, scipy, rospy; print('imports OK')"
```

Expected model files:

```text
src/model_weights/gnm_large.pth
src/model_weights/efficientnet_85x85.pth
```

## Record a short route

Start the existing WHILL system, camera, and odometry first. Then run:

```bash
cd ~/ros_ws/src/PlaceNav/src
./record_bag_whill.sh whill_short_route
```

Use a straight, obstacle-free route of 5--10 m at a constant low speed. Stop
recording with `Ctrl+C`.

## Create the topological map

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
cd ~/ros_ws/src/PlaceNav/src
./create_topomap_whill.sh whill_short_route 5.0
```

Terminal 3:

```bash
rosbag play topomaps/bags/whill_short_route.bag
```

At 0.1 m/s, a 5 second sampling period gives approximately 0.5 m node
spacing. Adjust the period according to the actual recording speed.

## Inference-only validation

Do not start the PD controller.

```bash
cd ~/ros_ws/src/PlaceNav/src
./navigate_whill.sh \
  --topomap-dir whill_short_route \
  --wp-model gnm_large \
  --pr-model cosplace \
  --subgoal-mode place_recognition \
  --filter-mode bayesian
```

Monitor:

```bash
rostopic hz /waypoint
rostopic echo /waypoint
rostopic echo /toponav/viz_info
rostopic echo /toponav/reached_goal
```

Optional visualization in another terminal:

```bash
./visualize_whill.sh \
  --topomap-dir whill_short_route \
  --pr-model cosplace \
  --show
```

Waypoint projection is intentionally disabled until the USB camera height and
calibration are validated.

## Blocking checks before motion

- Determine the MotionDecision automatic-mode joystick operation.
- Determine whether MotionDecision can run without front/rear LaserScan data.
- Restore or replace the missing `/emergency_stop` publisher.
- Measure the USB camera optical-center height.
- Add a watchdog between `/cmd_vel_nominal` and `/local_path/cmd_vel`.
- Verify manual override and physical emergency stop.
- Verify positive linear and angular command directions at very low speed.
