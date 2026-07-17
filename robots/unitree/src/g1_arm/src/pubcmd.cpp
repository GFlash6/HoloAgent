#include "unitree/robot/g1/arm/g1_arm_action_error.hpp"
#include "unitree/robot/g1/arm/g1_arm_action_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace unitree::robot::g1;

int main(int argc, char * argv[])
{
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
    while (true) {
        // 读取二进制 int
        velpipe.read(reinterpret_cast<char*>(&value), sizeof(int));

        // 检查是否读到完整 int
        if (velpipe.gcount() == sizeof(int)) {
            std::cout << "Read: " << value << std::endl;
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

            // 安全释放动作
            sleep(3);
            client->ExecuteAction(99);
            
            // // 发布动作完成状态
            // arm_status_pub.publish_true();

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

    return 0;
}
