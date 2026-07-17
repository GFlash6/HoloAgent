# 2. 每台机器人上的 FastAPI (机器人端)
# fastapi_robot.py (原样，不修改)
from fastapi import FastAPI
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import os
import threading
import uvicorn
import requests

# ================= 新增配置 =================
ROBOT_ID = int(os.getenv("ROBOT_ID", "11"))  # 本地机器 ID
CONTROL_URL = os.getenv("CONTROL_URL", "http://192.168.124.103:8080")
# ===========================================

app = FastAPI()


class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('fastapi_publisher')
        self.publisher = self.create_publisher(
            String, '/api/navigation_command', 10)
        # ===== 新增：订阅 waypoint_reached =====
        self.reached_sub = self.create_subscription(
            String,
            'waypoint_reached',
            self.waypoint_reached_callback,
            10
        )

    def publish_command(self, command: str):
        msg = String()
        msg.data = json.dumps({"command": command})
        self.publisher.publish(msg)

    # ===== 新增：到点回调 =====
    def waypoint_reached_callback(self, msg: String):
        if not msg.data:
            return
        print("robot waypoint reached : ", msg.data)
        try:
            requests.post(
                f"{CONTROL_URL}/robot_reached",
                json={
                    "robot_id": ROBOT_ID,
                    "waypoint_name": msg.data
                },
                timeout=1.0
            )
            self.get_logger().info(
                f"reported waypoint_reached to control ({ROBOT_ID})")
        except Exception as e:
            self.get_logger().error(f"failed to report waypoint_reached: {e}")


ros2_node = None


def start_ros2():
    global ros2_node
    rclpy.init()
    ros2_node = ROS2Publisher()
    rclpy.spin(ros2_node)


threading.Thread(target=start_ros2, daemon=True).start()


@app.post("/api/navigation/one_point_1")
async def trigger_one_point_1():
    if ros2_node:
        ros2_node.publish_command("one_point_1")
        return {"success": True}
    return {"success": False}


@app.post("/api/navigation/one_point_2")
async def trigger_one_point_2():
    if ros2_node:
        ros2_node.publish_command("one_point_2")
        return {"success": True}
    return {"success": False}


@app.post("/api/navigation/one_point_3")
async def trigger_one_point_1():
    if ros2_node:
        ros2_node.publish_command("one_point_3")
        return {"success": True}
    return {"success": False}


@app.post("/api/navigation/one_point_4")
async def trigger_one_point_2():
    if ros2_node:
        ros2_node.publish_command("one_point_4")
        return {"success": True}
    return {"success": False}


@app.post("/api/navigation/multi_point_1")
async def trigger_one_point_1():
    if ros2_node:
        ros2_node.publish_command("multi_point_1")
        return {"success": True}
    return {"success": False}


@app.post("/api/navigation/multi_point_2")
async def trigger_one_point_2():
    if ros2_node:
        ros2_node.publish_command("multi_point_2")
        return {"success": True}
    return {"success": False}


@app.post("/api/navigation/stop")
async def trigger_stop():
    if ros2_node:
        ros2_node.publish_command("stop")
        return {"success": True}
    return {"success": False}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
