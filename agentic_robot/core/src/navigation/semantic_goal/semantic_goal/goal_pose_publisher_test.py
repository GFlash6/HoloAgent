#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from fsr_vln import FsrVlnClient
import hydra
from omegaconf import DictConfig

from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
import os

# pylint: disable=all


class GoalPosePublisher(Node):

    def __init__(self, cfg: DictConfig):
        super().__init__("goal_pose_publisher")

        self.publisher_ = self.create_publisher(
            PoseStamped, "/object_pose", 10)
        self._action_client = ActionClient(
            self, FollowWaypoints, "/follow_waypoints")
        self.count = 0
        self.params = cfg
        self.use_gpt = bool(self.params.main.use_gpt)
        self.client = FsrVlnClient(
            cfg,
            room_name_generate_method="view_embedding",
            default_room_types=[
                "办公区",
                "会议室",
                "电梯间走廊",
                "茶水间",
                "办公休息区",
            ],
        )
        self.hmsggetgoal()

        self.get_logger().info("GoalPosePublisher 节点已启动，正在发布 /goal_pose 话题...")

    def pubpose(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"发布第 {self.count} 个目标位姿: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.count += 1

    def hmsggetgoal(self):
        while True:
            query = input("Enter query: ")
            if query == "q":
                break

            result = self.client.query(
                query,
                top_k=1,
                use_gpt=self.use_gpt,
            )
            print(f"floor_id: {result.floor_id}")
            print(f"room_query: {result.room_query}")
            print(f"object_query: {result.object_query}")
            print("len(targets): ", len(result.targets))

            for target in result.targets:
                print("object_id: ", target.object_id)
                print("room_id: ", target.room_id)
                print("room_name: ", target.room_name)
                print("obj_center in scenegraph: ", target.center_scenegraph)
                print("obj_center in lidarmap: ", target.center_map)
                self.pubpose(target.center_map[0], target.center_map[1])


@hydra.main(version_base=None, config_path="../config",
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
