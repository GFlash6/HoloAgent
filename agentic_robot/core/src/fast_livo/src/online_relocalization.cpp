#include <atomic>
#include <csignal>

#include "online-relo/pose_estimator.h"
#include "rclcpp/rclcpp.hpp"

namespace {
std::atomic_bool stop_requested{false};

void requestStop(int) {
  stop_requested.store(true, std::memory_order_relaxed);
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  // ROS2 does not require AsyncSpinner; multithreading is handled internally.
  auto nh = std::make_shared<rclcpp::Node>("pose_estimator_node");
  auto pose_estimator_node = std::make_shared<pose_estimator>(
      nh);  // 假设 pose_estimator 继承自 rclcpp::Node

  RCLCPP_INFO(nh->get_logger(),
              "\033[1;32m----> Online Relocalization Started.\033[0m");
  pose_estimator_node->run(nh, stop_requested);
  rclcpp::shutdown();
  return 0;
}
