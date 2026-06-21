# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **ROS catkin workspace** for an autonomous car course project (课设). The robot uses a STM32-based chassis controlled via serial port, equipped with an RPLiDAR, an Orbbec 3D camera, MPU6050 IMU, and electrical line-tracking sensors. Navigation uses `move_base` with TEB local planner, and the car performs multi-waypoint patrol with YOLOv8-based traffic light detection.

**Target platform:** Linux (Ubuntu + ROS Noetic probably), Jetson Nano or similar ARM SBC.

## Build

```bash
# Build the entire workspace
cd /home/gdut/catkin_roscar2
catkin_make

# Source the workspace
source devel/setup.bash
```

Only one C++ package (`driver`) needs compiling — the others are Python scripts. All C++ code lives in `src/driver/`.

## Architecture

### Data flow (pipeline)
```
STM32 ──serial──> driver_node ──publishes──> /odom, /imu, /PowerVoltage, /ele_sensor
                                             /cmd_vel <──subscribes── keyboard_teleop / ele_line_follower / move_base

RPLiDAR ──serial──> rplidar node ──publishes──> /scan

Orbbec cam ──USB──> orbbec_camera ──publishes──> RGB / depth images
         ──USB──> /usb_cam/image_raw ──subscribed by──> traffic_light_yolo ──publishes──> /traffic_light_status

/traffic_light_status + /scan ──> multi_waypoint_nav ──sends goals──> move_base ──publishes──> /cmd_vel
```

### Package roles

| Package | Type | Role |
|---------|------|------|
| `driver` | C++ | Hardware abstraction — serial comm with STM32, publishes `/odom`, `/imu`, `/PowerVoltage`, `/ele_sensor`; subscribes `/cmd_vel` to control motors |
| `rplidar_ros` | C++ | Slamtec RPLiDAR driver — publishes `/scan` |
| `orbbec_camera` | C++ | Orbbec 3D camera ROS wrapper — RGB + depth streams |
| `robot_pose_ekf` | C++ | EKF fusing odometry + IMU → `/odom_combined` |
| `start_roscar` | Python | **Application layer**: `multi_waypoint_nav.py` (autonomous patrol with hardcoded waypoints + traffic light logic), `keyboard_teleop.py` (manual control), config YAMLs for move_base/costmaps/TEB |
| `traffic_light_yolo` | Python | YOLOv8 on `/usb_cam/image_raw` → detects red/green lights, fuses LiDAR for distance, publishes `/traffic_light_status` |
| `ele_line_follower` | Python | Proportional-control line following using 3 electrical sensors on `/ele_sensor` |
| `roscar_slam` | — | Placeholder (no source yet) |

### Key nodes and topics

- **`driver_node`** (`src/driver/src/ros_car_driver.cpp`): The single most critical node. Reads 30-byte frames from STM32 via serial at 115200 baud (`FRAME_HEADER=0x7B`, `FRAME_TAIL=0x7D`). Integrates velocity → odometry. Computes IMU quaternion attitude. Publishes odom/IMU at loop rate.
- **`multi_waypoint_nav`** (`src/start_roscar/scripts/multi_waypoint_nav.py`): Hardcoded 6+ waypoints with traffic-light-aware navigation. On red light within 1.1m: cancel move_base goal + force zero `/cmd_vel` (emergency stop). Resumes on green.

### Serial protocol (STM32 ↔ driver_node)

- **TX (ROS→STM32):** 11 bytes — `[0x7B, reserved, reserved, Vx_high, Vx_low, Vy_high, Vy_low, Vz_high, Vz_low, BCC, 0x7D]`
- **RX (STM32→ROS):** 30 bytes — `[0x7B, stop_flag, Vx(2B), Vy(2B), Vz(2B), accel_x(2B), accel_y(2B), accel_z(2B), gyro_x(2B), gyro_y(2B), gyro_z(2B), voltage(2B), ele_sensor_1(2B), ele_sensor_2(2B), ele_sensor_3(2B), reserved(2B), BCC, 0x7D]`
- Velocities are transmitted as mm/s × 1000 (short int)

### Navigation config

- **Global planner:** `global_planner/GlobalPlanner`
- **Local planner:** `teb_local_planner/TebLocalPlannerROS`
- **Recovery behaviors disabled** (`recovery_behavior_enabled: false`)
- Robot footprint: polygon `[[-0.09,-0.185], [-0.09,0.185], [0.4,0.185], [0.4,-0.185]]` — an Ackermann car shape with 322mm wheelbase, 750mm minimum turning radius
- Max velocity: 0.15 m/s linear, 1.5 rad/s angular
- Map saved to `src/start_roscar/map/roscar_map.yaml`

### Traffic light logic (multi_waypoint_nav.py)

The navigator subscribes to `/traffic_light_status` (format: `"red,1.25"` or `"green,-1.0"`). When a red light is detected under 1.1m (with consecutive-frame validation), it immediately cancels the move_base goal and publishes zero velocity to `/cmd_vel`. On green, navigation resumes.

### Key hardcoded paths

These are hardcoded in the Python scripts and **must be adjusted** for different machines:
- `/home/gdut/catkin_roscar2/` — workspace root
- `/home/gdut/catkin_roscar2/src/start_roscar/param/cam_lidar_matrix.yaml` — camera-LiDAR calibration
- `/home/gdut/catkin_roscar2/lidar_ranges.txt` — LiDAR debug output
- `/dev/car_driver_tty` — serial port for STM32
- `/usr/lib/aarch64-linux-gnu/libgomp.so.1` — libgomp for Jetson

## Common commands

```bash
# Build
catkin_make

# Launch keyboard teleop (manual control)
rosrun start_roscar keyboard_teleop.py

# Launch multi-waypoint navigation
rosrun start_roscar multi_waypoint_nav.py

# Launch traffic light detection
rosrun traffic_light_yolo traffic_light_yolo.py

# Launch line follower
rosrun ele_line_follower ele_line_follower.py

# Launch driver node
roslaunch driver car_driver.launch

# Save map (from line follower shutdown)
# auto-triggered on Ctrl+C of ele_line_follower, or manually:
rosrun map_server map_saver -f /path/to/map
```
