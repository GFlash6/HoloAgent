#!/usr/bin/env python3
from omegaconf import DictConfig
import hydra
from fsr_vln import FsrVlnClient
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

_FSR_VLN_REPO = Path(__file__).resolve().parents[5] / "fsr_vln"
if _FSR_VLN_REPO.exists():
    fsr_vln_repo_str = str(_FSR_VLN_REPO)
    if fsr_vln_repo_str not in sys.path:
        sys.path.insert(0, fsr_vln_repo_str)


# pylint: disable=all


class GoalPosePublisher(Node):

    def __init__(self, cfg: DictConfig):
        super().__init__("semantic_goal_node")

        self.publisher_ = self.create_publisher(
            PoseStamped, "/object_pose", 10)
        self.subscription = self.create_subscription(
            String,
            "/chat_loc_pub",
            self.hmsggetgoal_callback,
            10)
        self.count = 0
        self.params = cfg
        self.use_gpt = bool(self.params.main.use_gpt)
        self.client = FsrVlnClient(
            cfg,
            room_name_generate_method="view_embedding",
            default_room_types=[
                "地瓜实验室",
                "地平线展厅",
                "地平线小邮局",
                "长走廊",
                "转角走廊",
                "电梯间",
                "电梯",
            ],
            designated_room_names=[  # 可人工指定房间名字
                "茶水间",
                "走廊",
                "实验室",
            ],
        )

        self.get_logger().info("GoalPosePublisher 节点已启动，正在发布 /goal_pose 话题...")

    def pubpose(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"发布第 {self.count} 个目标位姿: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.count += 1

    def hmsggetgoal_callback(self, msg):
        parsed_answer = msg.data
        print(parsed_answer)

        parsed_parts = [part.strip()
                        for part in parsed_answer.split(",", maxsplit=2)]
        if len(parsed_parts) != 3:
            self.get_logger().error(
                "语音查找接口输入格式错误，期望为 'floor, room, object'"
            )
            return

        floor_query, room_query, object_query = parsed_parts
        query_instruction = "来自语音查找"

        result = self.client.query(
            query_instruction,
            top_k=1,
            use_gpt=self.use_gpt,
            parsed_query=(floor_query, room_query, object_query),
        )

        # print(f"floor_id: {result.floor_id}")
        # print(f"floor_query: {result.floor_query}")
        # print(f"room_query: {result.room_query}")
        # print(f"object_query: {result.object_query}")
        # print("len(targets): ", len(result.targets))

        targets = result.targets

        for target in targets:
            print("object_id: ", target.object_id)
            print("room_id: ", target.room_id)
            print("room_name: ", target.room_name)
            print("obj_center in map: ", target.center_map)
            self.pubpose(target.center_map[0], target.center_map[1])


@hydra.main(version_base=None,
            config_path=get_package_share_directory(
                "semantic_goal") + "/config",
            config_name="visualize_query_graph_demo")
def main(params: DictConfig, args=None):
    rclpy.init(args=args)

    goal_pose_publisher = GoalPosePublisher(params)

    try:
        rclpy.spin(goal_pose_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        goal_pose_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
