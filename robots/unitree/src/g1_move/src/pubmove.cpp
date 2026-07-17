#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>

struct Vel
{
    float x;
    float y;
    float r;
    float duration;
    Vel() : x(0.0f), y(0.0f), r(0.0f), duration(1.0f) {}
    Vel(float a, float b, float c, float d) : x(a), y(b), r(c), duration(d) {}
};

int main(int argc, char * argv[])
{
    unitree::robot::ChannelFactory::Instance()->Init(0,"eth0");
    unitree::robot::g1::LocoClient client;
    client.Init();
    client.SetTimeout(10.f);

    std::ifstream velpipe("/tmp/move_fifo", std::ios::binary);

    if (!velpipe.is_open()) {
        std::cerr << "Failed to open /tmp/move_fifo" << std::endl;
        return -1;
    }

    Vel value;
    while (true) {
        velpipe.read(reinterpret_cast<char*>(&value), sizeof(Vel));

        if (velpipe.gcount() == sizeof(Vel)) {
            // 对 r 和 x 做安全限制
            if(value.x == 0.0f && value.y == 0.0f) {
                if(value.r > 0.0f && value.r < 0.3f) value.r = 0.3f;
                if(value.r < 0.0f && value.r > -0.3f) value.r = -0.3f;
            } else {
                if(value.r > 0.0f && value.r < 0.1f) value.r = 0.1f;
                if(value.r < 0.0f && value.r > -0.1f) value.r = -0.1f;
            }
            if(value.r > 0.5f) value.r = 0.5f;
            if(value.r < -0.5f) value.r = -0.5f;
            if(value.x > 0.5f) value.x = 0.5f;
            if(value.duration > 5.0f) value.duration = 5.0f;

            std::cout << "Read: " << value.x << "," << value.y << "," 
                      << value.r << "," << value.duration << std::endl;

            client.SetVelocity(value.x, value.y, value.r, value.duration);
            std::cout << "finish call move!" << std::endl;
        } else {
            // gcount 不够 sizeof(Vel) 或遇到 EOF
            usleep(10000);  // 短暂休眠
        }

        // 如果遇到 EOF，清除标记以便继续读取
        if (velpipe.eof()) {
            velpipe.clear();  // 清除 EOF 标记
        }
    }

    return 0;
}