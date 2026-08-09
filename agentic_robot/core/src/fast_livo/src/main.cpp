#include "LIVMapper.h"

#include <atomic>
#include <csignal>

namespace {
std::atomic_bool stop_requested{false};

void requestStop(int) { stop_requested.store(true, std::memory_order_relaxed); }
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping", options);
  mapper.initializeSubscribersAndPublishers(nh, it_);
  mapper.run(nh, stop_requested);
  // mapper.saveKeyFrame();
  rclcpp::shutdown();
  return 0;
}
