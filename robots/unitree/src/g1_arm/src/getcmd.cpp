#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <queue>
#include <mutex>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <random>
#include <fcntl.h>
#include <cstring>
#include <sys/stat.h>

class CmdSubscriber : public rclcpp::Node
{
public:
  CmdSubscriber()
  : Node("Cmd_subscriber"), fd_(-1)
  {
     RCLCPP_INFO(this->get_logger(), "订阅速度话题启动，等待数据...");
     
     // 检查并创建FIFO
    const char* fifo_path = "/tmp/arm_fifo";
    if(access(fifo_path, F_OK) == -1) {
        RCLCPP_WARN(this->get_logger(), "FIFO %s 不存在，正在创建...", fifo_path);
        if(mkfifo(fifo_path, 0666) == -1) {
            RCLCPP_ERROR(this->get_logger(), "创建FIFO失败: %s", strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "成功创建FIFO: %s", fifo_path);
        }
    }
    
    // 打开 FIFO（非阻塞模式，避免卡住）
    fd_ = open(fifo_path, O_WRONLY | O_NONBLOCK);
    if (fd_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "无法打开 FIFO: %s, 错误: %s", fifo_path, strerror(errno));
        RCLCPP_ERROR(this->get_logger(), "请确保 FIFO 已创建: mkfifo %s", fifo_path);
    } else {
        RCLCPP_INFO(this->get_logger(), "成功打开 FIFO: %s", fifo_path);
        // 设置为阻塞模式（可选，根据需求）
        int flags = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    }
     
    navstatus_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "waypoint_reached", 10, std::bind(&CmdSubscriber::topic_callback, this, std::placeholders::_1));
    armcmd_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chat_signal_pub", 10, std::bind(&CmdSubscriber::topic_callback, this, std::placeholders::_1));
    // add interface for holomotion/motion-tracking
    motion_tracking_cmd_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "motion_tracking", 10, std::bind(&CmdSubscriber::topic_callback, this, std::placeholders::_1));
    
  }
  
  ~CmdSubscriber()
  {
      if (fd_ != -1) {
          close(fd_);
          RCLCPP_INFO(this->get_logger(), "关闭 FIFO");
      }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navstatus_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr armcmd_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_tracking_cmd_subscription_;
  int fd_;
  
  void write_to_fifo(int value) const
  {
      if (fd_ == -1) {
          RCLCPP_ERROR(this->get_logger(), "FIFO 未打开，无法写入");
          return;
      }
      
      ssize_t ret = write(fd_, &value, sizeof(int));
      if (ret == sizeof(int)) {
          RCLCPP_INFO(this->get_logger(), "成功写入 FIFO: %d", value);
      } else {
          RCLCPP_ERROR(this->get_logger(), "写入 FIFO 失败: %s", strerror(errno));
      }
  }
  
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
      // 打印接收到的消息内容到控制台
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      
      // ===== 释放/安全命令 =====
      if(msg->data == "release_arm")
      {
          // 99: 松开手臂 / 释放机械臂
          printf("[release_arm] 释放机械臂，让手臂回到自然状态\n");
          write_to_fifo(99);
      }
      
      // ===== 手势命令 =====
      else if(msg->data == "turn_back_wave")
      {
          // 1: 转身挥手 - 机器人转身并挥手打招呼
          printf("[turn_back_wave] 转身挥手打招呼\n");
          write_to_fifo(1);
      }
      else if(msg->data == "blow_kiss_with_both_hands")
      {
          // 11: 双手飞吻 - 用双手做飞吻动作
          printf("[blow_kiss_with_both_hands] 双手飞吻\n");
          write_to_fifo(11);
      }
      else if(msg->data == "blow_kiss_with_left_hand")
      {
          // 12: 左手飞吻 - 用左手做飞吻动作
          printf("[blow_kiss_with_left_hand] 左手飞吻\n");
          write_to_fifo(12);
      }
      else if(msg->data == "blow_kiss_with_right_hand")
      {
          // 13: 右手飞吻 - 用右手做飞吻动作
          printf("[blow_kiss_with_right_hand] 右手飞吻\n");
          write_to_fifo(13);
      }
      else if(msg->data == "both_hands_up")
      {
          // 15: 双手举起 - 将双手举过头顶
          printf("[both_hands_up] 双手举起\n");
          write_to_fifo(15);
      }
      else if(msg->data == "clamp")
      {
          // 17: 夹取 / 抓夹 - 机械夹动作，用于抓取物体
          printf("[clamp] 机械夹夹取动作\n");
          write_to_fifo(17);
      }
      else if(msg->data == "high_five")
      {
          // 18: 击掌 - 与人击掌的动作
          printf("[high_five] 击掌\n");
          write_to_fifo(18);
      }
      else if(msg->data == "hug")
      {
          // 19: 拥抱 - 张开双臂做出拥抱姿势
          printf("[hug] 拥抱姿势\n");
          write_to_fifo(19);
      }
      else if(msg->data == "make_heart_with_both_hands")
      {
          // 20: 双手比心 - 用双手比出爱心的形状
          printf("[make_heart_with_both_hands] 双手比心 ❤️\n");
          write_to_fifo(20);
      }
      else if(msg->data == "make_heart_with_right_hand")
      {
          // 21: 右手比心 - 用右手比出爱心
          printf("[make_heart_with_right_hand] 右手比心\n");
          write_to_fifo(21);
      }
      else if(msg->data == "refuse")
      {
          // 22: 拒绝 - 摆手表示否定或拒绝
          printf("[refuse] 拒绝摆手\n");
          write_to_fifo(22);
      }
      else if(msg->data == "right_hand_up")
      {
          // 23: 右手举起 - 举起右手
          printf("[right_hand_up] 右手举起\n");
          write_to_fifo(23);
      }
      else if(msg->data == "ultraman_ray")
      {
          // 24: 奥特曼发射光线 - 经典奥特曼发射光线姿势
          printf("[ultraman_ray] 奥特曼发射光线姿势\n");
          write_to_fifo(24);
      }
      else if(msg->data == "wave_under_head")
      {
          // 25: 头下挥手 - 在下巴位置挥手
          printf("[wave_under_head] 头下挥手\n");
          write_to_fifo(25);
      }
      else if(msg->data == "wave_above_head" || msg->data == "one_point_1_waypoint_1")
      {
          // 26: 头上挥手 - 在头顶上方挥手
          printf("[wave_above_head] 头上挥手\n");
          write_to_fifo(26);
      }
      else if(msg->data == "shake_hand")
      {
          // 27: 握手 - 伸出手与人握手
          printf("[shake_hand] 握手\n");
          write_to_fifo(27);
      }
      else if(msg->data == "box_left_hand_win")
      {
          // 28: 左手出拳胜利 - 左手握拳做出胜利姿态
          printf("[box_left_hand_win] 左手出拳胜利\n");
          write_to_fifo(28);
      }
      else if(msg->data == "box_right_hand_win")
      {
          // 29: 右手出拳胜利 - 右手握拳做出胜利姿态
          printf("[box_right_hand_win] 右手出拳胜利\n");
          write_to_fifo(29);
      }
      else if(msg->data == "box_both_hand_win")
      {
          // 30: 双拳胜利姿态 - 双手握拳举高庆祝胜利
          printf("[box_both_hand_win] 双拳胜利姿态\n");
          write_to_fifo(30);
      }
      else if(msg->data == "right_hand_on_heart")
      {
          // 33: 右手放胸前 - 表示真诚、感谢或尊重
          printf("[right_hand_on_heart] 右手放胸前表示诚意\n");
          write_to_fifo(33);
      }
      else if(msg->data == "both_hands_up_deviate_right")
      {
          // 34: 双手举起并向右偏 - 双手上举并向右侧倾斜
          printf("[both_hands_up_deviate_right] 双手举起并向右偏\n");
          write_to_fifo(34);
      }
      else if(msg->data == "forward_push")
      {
          // 36: 向前推 - 双臂向前推的动作
          printf("[forward_push] 向前推\n");
          write_to_fifo(36);
      }
      
      else
      {
          RCLCPP_WARN(this->get_logger(), "Unknown command: '%s', no action taken", msg->data.c_str());
      }
  }
};

int main(int argc, char * argv[])
{
  // 检查并创建FIFO
  const char* fifo_path = "/tmp/arm_fifo";
  if(access(fifo_path, F_OK) == -1) {
      printf("警告: FIFO 文件 %s 不存在\n", fifo_path);
      printf("正在创建 FIFO...\n");
      if(mkfifo(fifo_path, 0666) == -1) {
          printf("创建 FIFO 失败: %s\n", strerror(errno));
          printf("请手动创建: mkfifo %s\n", fifo_path);
          sleep(3);
      } else {
          printf("成功创建 FIFO: %s\n", fifo_path);
      }
  }
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}