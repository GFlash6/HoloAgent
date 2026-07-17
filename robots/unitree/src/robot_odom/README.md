# robot_odom

`robot_odom` subscribes to the Unitree `SportModeState` message and publishes:

- `/robot_imu` (`sensor_msgs/msg/Imu`)
- `/robot_odom` (`nav_msgs/msg/Odometry`)

## Quick Start

### 1. Build the Package

Run the following command from the workspace root:

```bash
colcon build --packages-select robot_odom
```

### 2. Source the Environment

```bash
source install/setup.bash
```

### 3. Start the Node

Option 1: Start with the launch file

```bash
ros2 launch robot_odom imu_extractor_launch.py
```

Option 2: Start directly with `ros2 run`

```bash
ros2 run robot_odom imu_extractor
```

## Notes

- ROS 2 package name: `robot_odom`
- Node name: `robot_odom`
- Executable: `imu_extractor`
- `ros2 run` command: `ros2 run robot_odom imu_extractor`

After startup, the node subscribes to `/lf/odommodestate` and publishes IMU and odometry topics.