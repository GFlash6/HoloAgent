# navigation

The `navigation` directory currently contains three ROS 2 navigation-related nodes:

1. `semantic_goal_node`: Converts semantic retrieval results into navigation target poses
2. `relative_goal_node`: Converts relative motion commands in the robot frame into target poses in the map frame
3. `nav_executor_node`: Receives target poses or named signals and dispatches navigation or action execution through Nav2

These three nodes are commonly used in the following pipelines:

- Semantic navigation pipeline: `/chat_loc_pub` → HMSG coarse anchor → `/semantic_approach_pose` → online OVO refinement/persistence → `/object_pose` → `nav_executor_node`
- Relative navigation pipeline: `/relative_nav` → `relative_goal_node` → `/object_pose` → `nav_executor_node`

---

## Directory Structure

```text
navigation/
├── nav_executor/
├── relative_goal/
└── semantic_goal/
```

---

## 1. `semantic_goal_node`

### Purpose

`semantic_goal_node` subscribes to semantic query strings on `/chat_loc_pub` and runs two-stage navigation. It first queries HMSG and sends a coarse approach goal to `nav_executor_node` over the internal `/semantic_approach_pose` channel. After arrival it queries the online OVO map, atomically persists the refined map-frame anchor through the HMSG service, and publishes the final standoff pose to `/object_pose`.

### Code Location

- Package directory: `agentic_robot/core/src/navigation/semantic_goal`
- Main node file: `semantic_goal/semantic_goal/semantic_goal_node.py`
- `ros2 run` entry point: `semantic_goal_node`

### Input and Output

#### Subscribed Topic

- `/chat_loc_pub` (`std_msgs/msg/String`)

Expected input format:

```text
floor, room, object
```

For example:

```text
1楼, 茶水间, 咖啡机
```

#### Published Topic

- `/object_pose` (`geometry_msgs/msg/PoseStamped`)

---

## 2. `relative_goal_node`

### Purpose

`relative_goal_node` receives relative displacement commands in the robot frame, queries the current `map -> base_link` pose through TF, converts the command into an absolute target pose in the map frame, and publishes the result to `/object_pose`.

### Code Location

- Package directory: `agentic_robot/core/src/navigation/relative_goal`
- Main node file: `relative_goal/relative_goal/relative_goal_node.py`
- `ros2 run` entry point: `relative_goal_node`

### Input and Output

#### Subscribed Topic

- `/relative_nav` (`std_msgs/msg/String`)

Input format:

```text
forward,left,degrees
```

For example:

```text
1.0,0.5,30
```

#### Published Topic

- `/object_pose` (`geometry_msgs/msg/PoseStamped`)

---

## 3. `nav_executor_node`

### Purpose

`nav_executor_node` is the central navigation execution node. It handles two types of input:

1. It subscribes to `/object_pose` and directly invokes Nav2 to navigate to the target pose
2. It subscribes to `chat_signal_pub` and dispatches actions based on the signal registry, including:
   - `waypoint`
   - `arm_skill`
   - `nav_command`

It also publishes execution result notifications such as `waypoint_reached`.

### Code Location

- Package directory: `agentic_robot/core/src/navigation/nav_executor`
- Main node file: `nav_executor/nav_executor/pubpose.py`
- `ros2 run` entry point: `nav_executor_node`

### Input and Output

#### Subscribed Topics

- `object_pose` (`geometry_msgs/msg/PoseStamped`)
- `chat_signal_pub` (`std_msgs/msg/String`)

#### Published Topics

- `waypoint_reached` (`std_msgs/msg/String`)
- `arm_signal_pub` (`std_msgs/msg/String`)

### Key Parameters

`nav_executor_node` supports the following ROS parameters:

- `robot_name`: Robot config directory name. Default: `unitree`
- `map_name`: Map name. Default: empty string
- `signals_base_dir`: Root directory of the signals YAML files. Default: auto-detected

---

## Common Startup Requirements

Before starting these three nodes, the following conditions are typically required:

1. The workspace has been built successfully
2. The ROS 2 environment and the current workspace have been sourced
3. Core services such as Nav2, TF, the map, and the robot base are already running
4. If you use `semantic_goal_node`, the following are also required:
   - The `fsr_vln` Python dependencies are available
   - The configuration file `semantic_goal/config/visualize_query_graph_demo.yaml` can be loaded correctly
   - Semantic map or scene graph data has been prepared
   - `hmsg_query_server.py` is available on port `8120`
   - Online OVO mapping uses `ros.query_server_enabled: true` and serves port `8121`
5. If you use `relative_goal_node`, the following TF transform must exist:
   - `map -> base_link`
6. If you use `nav_executor_node`, the following are required:
   - The Nav2 action server is running
   - `nav2_simple_commander` is available
   - The signal configuration exists under `robots/<robot_name>/config/` if named signals are used

---

## Build and Environment Setup

Run the following commands from the workspace root:

```bash
colcon build --packages-select semantic_goal relative_goal nav_executor
source install/setup.bash
```

If the system also uses another ROS 2 environment, source the base environment first. For example:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Startup and Shutdown Overview

### Option 1: Start the Three Nodes Separately

#### Start `semantic_goal_node`

```bash
ros2 run semantic_goal semantic_goal_node
```

#### Start `relative_goal_node`

```bash
ros2 run relative_goal relative_goal_node
```

#### Start `nav_executor_node`

```bash
ros2 run nav_executor nav_executor_node
```

### Corresponding Shutdown Methods

#### Stop a Foreground Process Directly

If a node is running in the foreground of the current terminal, press:

```bash
Ctrl+C
```

#### Automatically Find and Stop Specific Nodes

First, list the current ROS 2 nodes:

```bash
ros2 node list
```

You will typically see names such as:

```text
/semantic_goal_node
/relative_nav_node
/waypoint_navigator
```

Then use the following commands to locate and stop the corresponding processes:

```bash
ps -ef | grep -F "ros2 run semantic_goal semantic_goal_node" | grep -v grep | awk '{print $2}' | xargs -r kill
ps -ef | grep -F "ros2 run relative_goal relative_goal_node" | grep -v grep | awk '{print $2}' | xargs -r kill
ps -ef | grep -F "ros2 run nav_executor nav_executor_node" | grep -v grep | awk '{print $2}' | xargs -r kill
```

If a normal `kill` does not stop the process, use:

```bash
ps -ef | grep -F "ros2 run semantic_goal semantic_goal_node" | grep -v grep | awk '{print $2}' | xargs -r kill -9
ps -ef | grep -F "ros2 run relative_goal relative_goal_node" | grep -v grep | awk '{print $2}' | xargs -r kill -9
ps -ef | grep -F "ros2 run nav_executor nav_executor_node" | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

---

## Recommended Usage Patterns

### Scenario A: Semantic Navigation

This scenario is suitable for semantic target navigation tasks such as "go to the coffee machine in the pantry."

#### Terminal 1: Start the Navigation Executor

```bash
ros2 run nav_executor nav_executor_node
```

#### Terminal 2: Start the Semantic Goal Node

```bash
ros2 run semantic_goal semantic_goal_node
```

#### Stop These Two Nodes

If they are running in the foreground, press the following in each terminal:

```bash
Ctrl+C
```

Or stop them from any terminal with:

```bash
ps -ef | grep -F "ros2 run semantic_goal semantic_goal_node" | grep -v grep | awk '{print $2}' | xargs -r kill
ps -ef | grep -F "ros2 run nav_executor nav_executor_node" | grep -v grep | awk '{print $2}' | xargs -r kill
```

#### Terminal 3: Publish a Semantic Query

```bash
ros2 topic pub /chat_loc_pub std_msgs/msg/String "{data: '1楼, 茶水间, 咖啡机'}" --once
```

#### Observe the Output

```bash
ros2 topic echo /object_pose
ros2 topic echo /waypoint_reached
```

---

### Scenario B: Relative Displacement Navigation

This scenario is suitable for local motion targets such as "move forward 1 meter, move left 0.5 meters, then rotate left by 30 degrees."

#### Terminal 1: Start the Navigation Executor

```bash
ros2 run nav_executor nav_executor_node
```

#### Terminal 2: Start the Relative Goal Node

```bash
ros2 run relative_goal relative_goal_node
```

#### Stop These Two Nodes

If they are running in the foreground, press the following in each terminal:

```bash
Ctrl+C
```

Or stop them from any terminal with:

```bash
ps -ef | grep -F "ros2 run relative_goal relative_goal_node" | grep -v grep | awk '{print $2}' | xargs -r kill
ps -ef | grep -F "ros2 run nav_executor nav_executor_node" | grep -v grep | awk '{print $2}' | xargs -r kill
```

#### Terminal 3: Publish a Relative Navigation Command

```bash
ros2 topic pub /relative_nav std_msgs/msg/String "{data: '1.0,0.5,30'}" --once
```

#### Observe the Output

```bash
ros2 topic echo /object_pose
ros2 topic echo /waypoint_reached
```

---

### Scenario C: Named Signal Navigation or Action Dispatch

This scenario is suitable when a chatbot or upper-level scheduler directly sends named signals, such as waypoint navigation, stop navigation, or robotic arm actions.

#### Start `nav_executor_node` with Parameters

```bash
ros2 run nav_executor nav_executor_node --ros-args -p robot_name:=g1 -p map_name:=office
```

If you need to explicitly specify the signals root directory:

```bash
ros2 run nav_executor nav_executor_node --ros-args \
  -p robot_name:=g1 \
  -p map_name:=office \
  -p signals_base_dir:=/workspace/D-Robotics/agentic_robot_system/robots
```

#### Stop `nav_executor_node`

If it is running in the foreground, press:

```bash
Ctrl+C
```

Or stop it from any terminal with:

```bash
ps -ef | grep -F "ros2 run nav_executor nav_executor_node" | grep -v grep | awk '{print $2}' | xargs -r kill
```

#### Publish a Named Signal

```bash
ros2 topic pub /chat_signal_pub std_msgs/msg/String "{data: 'one_point_1'}" --once
```

#### Stop the Current Navigation Task

```bash
ros2 topic pub /chat_signal_pub std_msgs/msg/String "{data: 'stop'}" --once
```

#### Observe the Output

```bash
ros2 topic echo /waypoint_reached
ros2 topic echo /arm_signal_pub
```

---

## Relationship Between the Three Nodes

### `semantic_goal_node` → `nav_executor_node`

- `semantic_goal_node` determines where the target is
- `nav_executor_node` actually navigates the robot to that target

In other words:

- The former handles retrieval and target pose generation
- The latter handles navigation execution

### `relative_goal_node` → `nav_executor_node`

- `relative_goal_node` converts local relative motion into a global target pose
- `nav_executor_node` executes navigation to that target pose

### Common Role of `semantic_goal_node` and `relative_goal_node`

Both nodes publish to the same topic:

- `/object_pose`

In essence, both are target pose generators, while `nav_executor_node` is the target pose executor.

---

## Troubleshooting

### 1. `semantic_goal_node` Fails to Start

Check the following first:

- Whether the `fsr_vln` Python package can be imported
- Whether the Hydra configuration file exists:
  - `semantic_goal/config/visualize_query_graph_demo.yaml`
- Whether the relevant map or scene graph data paths are correct

### 2. `relative_goal_node` Receives Commands but Produces No Output

Check the following first:

- Whether the `/relative_nav` message format is `forward,left,degrees`
- Whether the TF transform `map -> base_link` exists
- Useful commands:

```bash
ros2 topic echo /relative_nav
ros2 run tf2_ros tf2_echo map base_link
```

### 3. `nav_executor_node` Receives `/object_pose` but the Robot Does Not Move

Check the following first:

- Whether Nav2 is running
- Whether the action server is available
- Whether the map, localization, and base control are functioning correctly
- Useful commands:

```bash
ros2 topic echo /object_pose
ros2 topic echo /waypoint_reached
```

### 4. A Named Signal Does Not Take Effect

Check the following first:

- Whether `robot_name` and `map_name` match the configuration under `robots/<robot_name>/config/...`
- Whether the corresponding signal exists in `signals_common.yaml` or `signals.yaml`
- Whether the startup log prints:
  - `Loaded signal registry: [...]`

---

## Related Files

- `agentic_robot/core/src/navigation/semantic_goal/semantic_goal/semantic_goal_node.py`
- `agentic_robot/core/src/navigation/relative_goal/relative_goal/relative_goal_node.py`
- `agentic_robot/core/src/navigation/nav_executor/nav_executor/pubpose.py`
- `robots/<robot_name>/config/signals_common.yaml`
- `robots/<robot_name>/config/maps/<map_name>/signals.yaml`
