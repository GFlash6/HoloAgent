# Multi-Robot Control

This module documents the multi-robot control flow built around `robot_bridge` and the control-center service.

## Step 1: Start `robot_bridge` on Each Robot

`robot_bridge` replaces the older `api_bridge` and `fastapi_robot` split deployment.

```bash
# Build
bash agentic_robot/build.sh -p robot_bridge

# Start with robot ID and control-center URL
ros2 run robot_bridge robot_bridge_node \
  --ros-args \
  -p robot_id:=11 \
  -p control_url:=http://192.168.124.103:8080

# Or run directly with Python for development
ROBOT_ID=11 CONTROL_URL=http://192.168.124.103:8080 \
python3 agentic_robot/services/src/robot_bridge/robot_bridge/robot_bridge_node.py
```

## Step 2: Start the Control Center

Run the control center on the designated host, for example the machine with `ROBOT_ID=13`:

```bash
python3 fastapi_control.py
```

## Step 3: Trigger Manual Tests

```bash
# Trigger all robots {11,12,13,14,15,16} to navigate to point 1
curl -X POST http://192.168.124.103:8080/trigger/one_point_1

# Trigger a specific robot directly through robot_bridge (:8000)
curl -X POST http://192.168.124.101:8000/api/navigation/one_point_1
curl -X POST http://192.168.124.102:8000/api/navigation/one_point_1

# Trigger robot 12 through the control center
curl -X POST "http://192.168.124.103:8080/trigger/one_point_1?robot_id=12"

# Stop all robots through the control center
curl -X POST http://192.168.124.103:8080/trigger/stop

# Stop robot 11 through the control center
curl -X POST "http://192.168.124.103:8080/trigger/stop?robot_id=11"

# Stop one robot directly through robot_bridge
curl -X POST http://192.168.124.101:8000/api/navigation/stop

# Relative navigation for robot 12
curl -X POST http://192.168.124.102:8000/api/relative_nav \
  -H "Content-Type: application/json" \
  -d '{"cmd":"1.0,0.0,0"}'

# Semantic navigation for robot 11
curl -X POST http://192.168.124.101:8000/api/semantic_nav \
  -H "Content-Type: application/json" \
  -d '{"cmd":"unknown,unknown,coffee machine"}'

# Trigger an arm skill on robot 13
curl -X POST http://192.168.124.103:8000/api/arm/wave_above_head
```

## Step 4: Call the Control Center from Other Programs

```python
import requests

requests.post("http://control-center-ip:8080/trigger/one_point_1")
requests.post("http://control-center-ip:8080/trigger/one_point_2?robot_id=11")
requests.post("http://control-center-ip:8080/trigger/stop")
```

## Step 5: Extend Endpoints Through `bridge_config.yaml`

To add a new `robot_bridge` HTTP endpoint, edit `agentic_robot/services/src/robot_bridge/config/bridge_config.yaml`, then restart `robot_bridge` on the target robot.

Example:

```yaml
endpoints:
  - http:
      method: POST
      path: /api/custom_topic
    ros:
      type: topic
      name: /my_custom_topic
      msg_type: std_msgs/String
      body:
        data: "payload"
```

Test it with:

```bash
curl -X POST http://robot-ip:8000/api/custom_topic \
  -H "Content-Type: application/json" \
  -d '{"payload":"hello from http"}'
```

If you want to map a URL path parameter directly into the ROS message, use `@path.<name>`:

```yaml
endpoints:
  - http:
      method: POST
      path: /api/custom_topic/{name}
    ros:
      type: topic
      name: /my_custom_topic
      msg_type: std_msgs/String
      body:
        data: "@path.name"
```

Then call:

```bash
curl -X POST http://robot-ip:8000/api/custom_topic/demo_signal
```

## Notes

- `robot_bridge` endpoints are declared in `agentic_robot/services/src/robot_bridge/config/bridge_config.yaml`; after editing the file, restart `robot_bridge` to load the new mappings.
- The examples in this document assume a fixed internal network topology and should be adapted for deployment.
