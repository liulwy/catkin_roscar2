# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **ROS catkin workspace** for an autonomous car course project (课设). The robot uses a STM32-based chassis controlled via serial port, equipped with an RPLiDAR, an Orbbec 3D camera, MPU6050 IMU, and electrical line-tracking sensors. Navigation uses `move_base` with TEB local planner, and the car performs multi-waypoint patrol with YOLOv8-based traffic light detection.

**Target platform:** Linux (Ubuntu + ROS Noetic), Jetson Nano or similar ARM SBC.

**Current car mode:** `senior_omni` (全向轮) — default in `start_roscar.launch`.

**Development divided into two cycles:**
- ✅ **上半周期 (complete):** Path recording, auto-alignment, omnidirectional conversion, keyboard teleop, SLAM mapping
- ⏸️ **下半周期 (paused):** Dynamic obstacle avoidance using ARS408 millimeter-wave radar (module at `src/ars408_ros/`, code exists but not integrated)

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
- **`keyboard_teleop.py`** (`src/start_roscar/scripts/keyboard_teleop.py`): WASD hold-to-move (turtlesim pattern, timeout-based release detection at 0.12s). Supports combo keys (w+d=forward-right, etc.). Also handles path recording (r/p/c) and SLAM map saving (m). Uses `sys.stdin` raw mode (NOT `/dev/tty`, which breaks over SSH).
- **`multi_waypoint_nav`** (`src/start_roscar/scripts/multi_waypoint_nav.py`): Loads waypoints from YAML file (`patrol_path.yaml`) instead of hardcoded. Traffic-light-aware: on red light within 1.1m cancels move_base goal + force zero `/cmd_vel`; resumes on green. Waits for "ready" signal from `wait_for_convergence.py` before starting.

### Serial protocol (STM32 ↔ driver_node)

- **TX (ROS→STM32):** 11 bytes — `[0x7B, reserved, reserved, Vx_high, Vx_low, Vy_high, Vy_low, Vz_high, Vz_low, BCC, 0x7D]`
- **RX (STM32→ROS):** 30 bytes — `[0x7B, stop_flag, Vx(2B), Vy(2B), Vz(2B), accel_x(2B), accel_y(2B), accel_z(2B), gyro_x(2B), gyro_y(2B), gyro_z(2B), voltage(2B), ele_sensor_1(2B), ele_sensor_2(2B), ele_sensor_3(2B), reserved(2B), BCC, 0x7D]`
- Velocities are transmitted as mm/s × 1000 (short int)

### Navigation config

- **Global planner:** `global_planner/GlobalPlanner`
- **Local planner:** `teb_local_planner/TebLocalPlannerROS`
- **Recovery behaviors disabled** (`recovery_behavior_enabled: false`)
- Robot footprint: polygon `[[-0.09,-0.185], [-0.09,0.185], [0.4,0.185], [0.4,-0.185]]`
- Max velocity: 0.15 m/s linear (x+y), 1.5 rad/s angular
- Map saved to `src/start_roscar/map/roscar_map.yaml`
- Path waypoints stored in `src/start_roscar/path/patrol_path.yaml` (loaded by `multi_waypoint_nav.py`)

### Omnidirectional (全向轮) TEB configuration

Converted from Ackermann (`senior_akm`) to holonomic omnidirectional (`senior_omni`). Key TEB changes in `teb_local_planner_params.yaml`:

| Parameter | Before (Ackermann) | After (Omni) | Effect |
|-----------|-------------------|--------------|--------|
| `max_vel_y` | 0.0 | 0.15 | **Activates holonomic mode** (>0 is the switch) |
| `acc_lim_y` | 0.0 | 0.2 | Allow lateral acceleration |
| `min_turning_radius` | 0.750 | 0.0 | Can rotate in place |
| `wheelbase` | 0.322 | 0.0 | No轴距 constraint |
| `weight_kinematics_nh` | 1000 | 1 | **Critical**: relaxes non-holonomic constraint |
| `weight_kinematics_forward_drive` | 1 | 0 | No forward-only penalty |
| `weight_kinematics_turning_radius` | 1 | 0 | No turning-radius penalty |

**Important:** TEB weights are relative — obstacle avoidance (`weight_obstacle: 100`) only works when it outweighs kinematics constraints. Before the omni conversion, `weight_kinematics_nh: 1000` was 10× `weight_obstacle: 100`, causing TEB to prefer collisions over violating Ackermann constraints.

### ⚠️ Costmap obstacle layer topic bug

**Critical lesson:** In `costmap_common_params.yaml`, the obstacle layer's `topic` parameter uses a **relative path**:

```yaml
obstacle_layer:
  observation_sources: scan
  scan:
    topic: scan       # ← BUG: resolved relative to move_base namespace
```

Under `multi_navigation.launch`, move_base runs in namespace `/move_base`, so `topic: scan` can resolve to `/move_base/local_costmap/scan` instead of the actual laser topic `/scan`. This causes the costmap obstacle layer to silently wait for data that never arrives — **no warnings, no errors, costmap stay all zeros.**

**Fix:** Use absolute topic path:
```yaml
  scan:
    topic: /scan      # ← absolute path bypasses namespace issues
```

Or add a remap in the move_base launch:
```xml
<node pkg="move_base" type="move_base" name="move_base">
  <remap from="scan" to="/scan"/>
</node>
```

### AMCL localization

- `odom_model_type: omni-corrected` — correct for omnidirectional robots
- Initial covariance widened: `initial_cov_xx: 0.25`, `initial_cov_yy: 0.25`, `initial_cov_aa: 0.1` (allows self-localization without RViz 2D Pose Estimate)
- `laser_likelihood_max_dist: 2.0` — covers the ~1m landmark distance
- Auto-alignment script `wait_for_convergence.py`: moves forward 0.32m at startup, triggers AMCL particle convergence, then publishes "ready" on `/traffic_light_status`

### Traffic light logic (multi_waypoint_nav.py)

The navigator subscribes to `/traffic_light_status` (format: `"red,1.25"` or `"green,-1.0"`). When a red light is detected under 1.1m (with consecutive-frame validation), it immediately cancels the move_base goal and publishes zero velocity to `/cmd_vel`. On green, navigation resumes.

### Key hardcoded paths

These are hardcoded in the Python scripts and **must be adjusted** for different machines:
- `/home/gdut/catkin_roscar2/` — workspace root
- `/home/gdut/catkin_roscar2/src/start_roscar/param/cam_lidar_matrix.yaml` — camera-LiDAR calibration
- `/home/gdut/catkin_roscar2/lidar_ranges.txt` — LiDAR debug output
- `/dev/car_driver_tty` — serial port for STM32
- `/usr/lib/aarch64-linux-gnu/libgomp.so.1` — libgomp for Jetson

## Launch files

| Launch file | What it starts | When to use |
|-------------|---------------|-------------|
| `slam_mapping.launch` | Sensors + Gmapping + keyboard teleop (`m`=save map) | **键盘建图** — WASD drive around, q exits & saves |
| `keyboard_control.launch` | Driver + keyboard teleop (no SLAM) | **纯遥控** — manual driving only |
| `multi_navigation.launch` | Full nav stack + AMCL + move_base + TEB | **自主导航** — after map is built |
| `start_roscar.launch` | Driver + sensors + EKF (no navigation) | Base sensor stack, included by others |

## Common commands

```bash
# Build
catkin_make

# === 键盘建图（推荐）===
roslaunch start_roscar slam_mapping.launch
#   WASD = 开车  m = 手动保存地图  q = 退出并自动保存地图
#   地图保存到 src/start_roscar/map/roscar_map.yaml
#   路径录制: r=开始  p=保存  c=清除

# Launch keyboard teleop only (manual control, no SLAM)
roslaunch start_roscar keyboard_control.launch

# Launch autonomous navigation (requires pre-built map)
roslaunch start_roscar multi_navigation.launch

# Launch traffic light detection
rosrun traffic_light_yolo traffic_light_yolo.py

# Launch line follower
rosrun ele_line_follower ele_line_follower.py

# Save map manually
rosrun map_server map_saver -f /path/to/map
```
