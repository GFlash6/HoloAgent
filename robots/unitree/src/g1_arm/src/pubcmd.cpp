#include "unitree/robot/g1/arm/g1_arm_action_error.hpp"
#include "unitree/robot/g1/arm/g1_arm_action_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <unordered_map>

using namespace unitree::robot::g1;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("g1_arm_action_executor");
    auto result_pub = node->create_publisher<std_msgs::msg::String>("arm_action_result", 10);

    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

    auto client = std::make_shared<G1ArmActionClient>();
    client->Init();
    client->SetTimeout(10.f); // All actions will last less than 10 seconds.

    // // 创建ROS2发布者
    // ArmStatusPublisher arm_status_pub(argc, argv);

    // 只打开已有管道，不重建
    std::ifstream velpipe("/tmp/arm_fifo", std::ios::binary);
    if (!velpipe.is_open()) {
        std::cerr << "Failed to open /tmp/arm_fifo. Make sure the pipe exists." << std::endl;
        return -1;
    }

    int value;
    const std::unordered_map<int, std::string> action_names = {
        {1, "turn_back_wave"}, {11, "blow_kiss_with_both_hands"},
        {12, "blow_kiss_with_left_hand"}, {13, "blow_kiss_with_right_hand"},
        {15, "both_hands_up"}, {17, "clamp"}, {18, "high_five"},
        {19, "hug"}, {20, "make_heart_with_both_hands"},
        {21, "make_heart_with_right_hand"}, {22, "refuse"},
        {23, "right_hand_up"}, {24, "ultraman_ray"},
        {25, "wave_under_head"}, {26, "wave_above_head"},
        {27, "shake_hand"}, {28, "box_left_hand_win"},
        {29, "box_right_hand_win"}, {30, "box_both_hand_win"},
        {33, "right_hand_on_heart"}, {34, "both_hands_up_deviate_right"},
        {36, "forward_push"}, {99, "release_arm"},
    };
    while (true) {
        // 读取二进制 int
        velpipe.read(reinterpret_cast<char*>(&value), sizeof(int));

        // 检查是否读到完整 int
        if (velpipe.gcount() == sizeof(int)) {
            std::cout << "Read: " << value << std::endl;
            const auto action_it = action_names.find(value);
            const std::string action_name = action_it == action_names.end()
                ? "unknown_" + std::to_string(value) : action_it->second;
            int32_t ret = client->ExecuteAction(value);
            if (ret != 0) {
                switch (ret) {
                    case UT_ROBOT_ARM_ACTION_ERR_ARMSDK:
                        std::cout << UT_ROBOT_ARM_ACTION_ERR_ARMSDK_DESC << std::endl;
                        break;
                    case UT_ROBOT_ARM_ACTION_ERR_HOLDING:
                        std::cout << UT_ROBOT_ARM_ACTION_ERR_HOLDING_DESC << std::endl;
                        break;
                    case UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID:
                        std::cout << UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID_DESC << std::endl;
                        break;
                    case UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID:
                        std::cout << "The actions are only supported in fsm id {500, 501, 801}" << std::endl;
                        break;
                    default:
                        std::cerr << "Execute action failed, error code: " << ret << std::endl;
                        break;
                }
            }

            int32_t release_ret = 0;
            if (ret == 0 && value != 99) {
                sleep(3);
                release_ret = client->ExecuteAction(99);
            }

            std_msgs::msg::String result;
            if (ret == 0 && release_ret == 0) {
                result.data = "arm_finish:" + action_name;
            } else {
                result.data = "arm_failed:" + action_name + ":" +
                    std::to_string(ret != 0 ? ret : release_ret);
            }
            result_pub->publish(result);
            rclcpp::spin_some(node);

        } else {
            // 检查 EOF
            // if (velpipe.eof()) {
            //     velpipe.clear(); // 清除 EOF 标记
            // }
            usleep(10000);  // 短暂休眠
        }
        
        // // 处理ROS2回调
        // arm_status_pub.spin();
    }

    rclcpp::shutdown();
    return 0;
}
