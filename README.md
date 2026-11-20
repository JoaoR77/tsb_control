# tsb_control

ROS 2 Humble package implementing a 2-DOF (surge velocity and heading) PID control pipeline, force allocation, and Line-of-Sight (LOS) path following for the TSB Autonomous Systems recruitment task. Tested in Gazebo with TurtleBot3.

## Pipeline Overview

```text
Gazebo (/odom, /imu)
   │
   ▼
sensor_processing_node ──► /state/pose, /state/velocity
   │
   ▼
path_following_node (optional) ──► /desired_velocity, /desired_heading
   │
   ▼
pid_node ──► /tau_u, /tau_r
   │
   ▼
allocation_node ──► /current_left, /current_right, /cmd_vel
```

## Package Layout

```text
tsb_control/
├── config/
│   └── pid_params.yaml
├── launch/
│   ├── control_pipeline.launch.py
│   └── path_following.launch.py
├── resource/
│   └── tsb_control
├── tsb_control/
│   ├── __init__.py
│   ├── sensor_processing_node.py
│   ├── pid_node.py
│   ├── allocation_node.py
│   └── path_following_node.py
├── package.xml
├── setup.cfg
└── setup.py
```

## Nodes & Topics

### `sensor_processing_node`
Reads `/odom` and `/imu` from the simulator, converts the orientation quaternion to yaw ($\psi$), and republishes the robot's 2D pose and forward speed at 50 Hz.
* **Subscribes:** `/odom` (`nav_msgs/msg/Odometry`), `/imu` (`sensor_msgs/msg/Imu`)
* **Publishes:** `/state/pose` (`geometry_msgs/msg/Pose2D`), `/state/velocity` (`geometry_msgs/msg/Twist`)

### `pid_node`
Runs two discrete-time PID loops at 50 Hz to track desired surge velocity ($u_d$) and heading ($\psi_d$). Includes integral anti-windup clamping, output limits, and angle wrapping to $[-\pi, \pi]$.
* **Subscribes:** `/state/pose` (`geometry_msgs/msg/Pose2D`), `/state/velocity` (`geometry_msgs/msg/Twist`), `/desired_velocity` (`std_msgs/msg/Float32`), `/desired_heading` (`std_msgs/msg/Float32`)
* **Publishes:** `/tau_u` (`std_msgs/msg/Float32`), `/tau_r` (`std_msgs/msg/Float32`)

### `allocation_node`
Splits surge force ($\tau_u$) and yaw torque ($\tau_r$) into left/right thruster commands and maps them to TurtleBot3 velocity limits on `/cmd_vel`.
* **Subscribes:** `/tau_u` (`std_msgs/msg/Float32`), `/tau_r` (`std_msgs/msg/Float32`)
* **Publishes:** `/current_left` (`std_msgs/msg/Float32`), `/current_right` (`std_msgs/msg/Float32`), `/cmd_vel` (`geometry_msgs/msg/Twist`)

### `path_following_node`
Line-of-Sight (LOS) guidance node that steps through a list of $(x, y)$ waypoints and publishes desired heading and cruising velocity to `pid_node`.
* **Subscribes:** `/state/pose` (`geometry_msgs/msg/Pose2D`), `/waypoints` (`geometry_msgs/msg/PoseArray`)
* **Publishes:** `/desired_velocity` (`std_msgs/msg/Float32`), `/desired_heading` (`std_msgs/msg/Float32`)

## Setup & Build

Requires Ubuntu 22.04 (or WSL 2), **ROS 2 Humble**, and `turtlebot3_gazebo`.

```bash
mkdir -p ~/tsb_ws/src
cd ~/tsb_ws/src
git clone [https://github.com/](https://github.com/)<JoaoR77>/tsb_control.git
cd ~/tsb_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

1. Start Gazebo in one terminal:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

2. **Manual setpoint control** (starts sensor processing, PID, and allocation):
   ```bash
   source ~/tsb_ws/install/setup.bash
   ros2 launch tsb_control control_pipeline.launch.py
   ```
   Send a target heading (rad) and forward velocity (m/s):
   ```bash
   ros2 topic pub /desired_heading std_msgs/msg/Float32 "{data: 1.57}" --once
   ros2 topic pub /desired_velocity std_msgs/msg/Float32 "{data: 0.15}" --once
   ```

3. **Autonomous waypoint following** (launches the full pipeline plus LOS guidance through the waypoints in `config/pid_params.yaml`):
   ```bash
   source ~/tsb_ws/install/setup.bash
   ros2 launch tsb_control path_following.launch.py
   ```

## Parameters

All gains, saturation bounds, thruster spacing, and default waypoints are defined in `config/pid_params.yaml`.

## Known Limitations & Next Steps

* **Post-Mission Drift:** After reaching the final waypoint in `path_following_node`, residual integral windup and sensor noise can cause a slight slow drift. Next steps would be to reset the surge PID integral term when `/desired_velocity` drops to `0.0` and add a small deadband on `/tau_u` and `/tau_r` in `allocation_node`.