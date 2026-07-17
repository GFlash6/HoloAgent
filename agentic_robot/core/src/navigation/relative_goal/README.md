# relative_goal

`relative_goal` is a ROS 2 Python package that converts relative motion commands in the robot frame into absolute target poses in the map frame, then publishes them as `PoseStamped` messages for downstream modules.

## Overview

This node queries the robot's current pose from TF (`map -> base_link`), receives a relative motion command, computes the corresponding absolute position and orientation in the global frame, and publishes the result to a target topic.

It is intended for scenarios such as:

- An upstream module provides only relative motion commands such as "forward / left / rotate"
- A navigation execution module requires a target pose in the global coordinate frame
- Local offsets must be converted into a global target pose based on the robot's current heading

## Node Information

- Package name: `relative_goal`
- Executable entry point (for `ros2 run`): `relative_goal_node`
- Node name (as shown in the ROS graph): `relative_nav_node`

## Input and Output Interfaces

### Subscribed Topic

- `/relative_nav` (`std_msgs/msg/String`)

Message format:

```text
forward,left,degrees
```

Field definitions:

- `forward`: Forward displacement of the robot, in meters
- `left`: Leftward displacement of the robot, in meters
- `degrees`: Rotation angle relative to the current heading, in degrees; positive for left turns and negative for right turns

Example:

```text
1.0,0.0,90
```

This means:

- Move forward by 1.0 meter
- Move left by 0.0 meters
- Rotate 90 degrees to the left relative to the current heading

### Published Topic

- `/object_pose` (`geometry_msgs/msg/PoseStamped`)

The published message is the target pose in the `map` frame.

## Workflow

1. The node listens on `/relative_nav`
2. It receives a relative motion command in string format
3. It queries the current robot pose from TF: `map -> base_link`
4. It rotates the local offset `(forward, left, 0)` into the global coordinate frame
5. It adds the relative rotation angle `degrees` to the current yaw
6. It generates the target `PoseStamped`
7. It publishes the result to `/object_pose`

## Prerequisites

Before running this node, make sure the following conditions are met:

- ROS 2 is installed correctly and the environment has been sourced
- The TF transform `map -> base_link` is available
- The workspace has been built with this package included
- The following dependencies are installed:
  - `rclpy`
  - `std_msgs`
  - `geometry_msgs`
  - `tf2_ros`
  - `tf2_geometry_msgs`
  - `transforms3d`

## Launch Procedure

### 1. Build the Package

Run the following command from the workspace root:

```bash
colcon build --packages-select relative_goal
```

### 2. Source the Environment

```bash
source install/setup.bash
```

If you already use an upper-level ROS 2 environment, source that environment first as well.

### 3. Start the Node

```bash
ros2 run relative_goal relative_goal_node
```

## Usage Example

After the node starts, publish a relative navigation command:

```bash
ros2 topic pub /relative_nav std_msgs/msg/String "{data: '1.0,0.5,30'}" --once
```

This command means:

- Move forward by 1.0 meter
- Move left by 0.5 meters
- Rotate 30 degrees relative to the current heading

The node computes the corresponding global target pose and publishes it to:

```bash
/object_pose
```

You can inspect the output with:

```bash
ros2 topic echo /object_pose
```

## Notes

- The input message must strictly follow the `forward,left,degrees` format. Otherwise, the node reports an error and ignores the message.
- The current implementation uses fixed frames:
  - Global frame: `map`
  - Robot frame: `base_link`
- The output pose always uses `z = 0.0`
- Orientation is computed using yaw only; roll and pitch are not used
- `degrees > 0` means a left turn (counterclockwise), and `degrees < 0` means a right turn (clockwise)
- The TF lookup timeout is 1 second. If TF is unavailable, the node cannot generate a target pose

## Code Location

- Main node file: `relative_goal/relative_goal_node.py`
- Package configuration: `package.xml`
- Python entry-point configuration: `setup.py`