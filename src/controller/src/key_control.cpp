// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <iostream>
// #include <termios.h> // 用于捕获键盘输入
// #include <unistd.h>  // 用于读取终端输入

// #include "airsim_ros/Takeoff.h"  // 包含 Takeoff 服务类型
// #include "airsim_ros/VelCmd.h"  // 包含 VelCmd 消息类型
// #include "geometry_msgs/PoseStamped.h"  // 用于订阅 GPS 数据

// // 定义按键
// // 油门
// #define KEYCODE_W 0x77 // w
// #define KEYCODE_S 0x73 // s
// // 偏航角
// #define KEYCODE_A 0x61 // a
// #define KEYCODE_D 0x64 // d
// // 俯仰
// #define KEYCODE_I 0x69 // i
// #define KEYCODE_K 0x6B // k
// // 横滚
// #define KEYCODE_J 0x6A // j
// #define KEYCODE_L 0x6C // l
// // z轴高度

// #define KEYCODE_Q 0x71 // q (退出)

// // 读取键盘输入的类
// class KeyboardReader {
// public:
//     KeyboardReader() {
//         // 设置终端为非标准模式
//         tcgetattr(STDIN_FILENO, &old_termios_);
//         termios new_termios = old_termios_;
//         new_termios.c_lflag &= ~(ICANON | ECHO); // 关闭行缓冲和回显
//         tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
//     }

//     ~KeyboardReader() {
//         // 恢复终端设置
//         tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
//     }

//     char readKey() {
//         char c;
//         if (read(STDIN_FILENO, &c, 1) < 0) {
//             perror("read():");
//             return '\0';
//         }
//         return c;
//     }

// private:
//     termios old_termios_;
// };


// // 目标高度
// const double TARGET_HEIGHT = -50.0;

// double target_height = -50.0;
// // 当前无人机的高度
// double current_height = 0.0;
// double current_x = 0.0;

// double initial_x = 0.0;
// bool init_flag = true;

// // GPS 回调函数
// void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    
//     if(init_flag) {
//         initial_x = msg->pose.position.x;
//         init_flag = false;
//     }

//     current_x = msg->pose.position.x;  // 获取无人机当前高度
//     current_height = msg->pose.position.z;  // 获取无人机当前高度

// }


// int main(int argc, char** argv) {
//     ros::init(argc, argv, "keyboard_control");
//     ros::NodeHandle nh;

//     // 发布速度命令的话题
//     // 订阅 GPS 数据，用于获取当前高度
//     ros::Subscriber gps_sub = nh.subscribe("/airsim_node/drone_1/gps", 10, gpsCallback);

//     // 创建速度指令发布者
//     ros::Publisher vel_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 10);
    
//     KeyboardReader keyboard_reader;
//     geometry_msgs::Twist cmd_vel;

//     ROS_INFO("Use 'WASD' to control the robot, 'Q' to quit.");

//     // 调用起飞服务
//     ros::ServiceClient takeoff_client = nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");

//     // 定义一个服务器
//     airsim_ros::Takeoff takeoff_srv;
//     ROS_INFO("start fly!!");

//     takeoff_srv.request.waitOnLastTask = true;  // 根据服务定义设置参数 

//     if (takeoff_client.call(takeoff_srv)) {
//         ROS_INFO("Takeoff service called successfully!");
//     } else {
//         ROS_ERROR("Failed to call takeoff service.");
//         return 1;
//     }

//     ros::Rate loop_rate(50); // 控制循环频率 
//     ROS_INFO("start while!!"); 

//     while (ros::ok()) {
//         char key = keyboard_reader.readKey(); // 读取按键

//         // 创建速度指令消息
//         airsim_ros::VelCmd vel_cmd;
        
//         if(std::abs((current_x - (initial_x - 5))) < 0.5f) {
//             target_height = -52.0;
//         }

//         double height_error = target_height - current_height;

//         // 根据高度误差调整垂直速度
//         if (std::abs(height_error) > 0.05) {  // 高度误差大于 5cm 时调整
//             vel_cmd.twist.linear.z = 2.5 * height_error;  // 简单比例控制
//         } 

//         switch (key) {
//         // 左端摇杆
//         case KEYCODE_W:
//             ROS_INFO("Moving up");
//             target_height--;
//             // vel_cmd.twist.linear.z = 2.0; // 前进
//             break;
//         case KEYCODE_S:
//             ROS_INFO("Moving down");
//             target_height++;
//             // vel_cmd.twist.linear.z = -2.0; // 后退
//             break;
//         case KEYCODE_A:
//             ROS_INFO("Turning left");
//             vel_cmd.twist.angular.z = 1.0; // 左转
//             break;
//         case KEYCODE_D:
//             ROS_INFO("Turning right");
//             vel_cmd.twist.angular.z = -1.0; // 右转
//             break;

//         // 右端摇杆
//         case KEYCODE_I:
//             ROS_INFO("Moving forward");
//             vel_cmd.twist.linear.x = 4.0; // 前进
//             break;
//         case KEYCODE_K:
//             ROS_INFO("Moving backward");
//             vel_cmd.twist.linear.x = -4.0; // 后退
//             break;
//         case KEYCODE_J:
//             ROS_INFO("Moving left");
//             vel_cmd.twist.linear.y = 1.0; // 左转
//             break;
//         case KEYCODE_L:
//             ROS_INFO("Moving right");
//             vel_cmd.twist.linear.y = -1.0; // 右转
//             break;

//         // 退出
//         case KEYCODE_Q:
//             ROS_INFO("Exiting...");
//             return 0; // 退出程序
//         default:
//             break;
//         }

//         // 发布速度命令
//         vel_pub.publish(vel_cmd);
//         ROS_INFO("Current height: %.2f, Target height: %.2f, Vertical speed: %.2f",
//                  current_height, target_height, vel_cmd.twist.linear.z);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }















// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <iostream>
// #include <thread>    // 用于多线程
// #include <mutex>     // 用于线程安全
// #include <termios.h> // 用于捕获键盘输入
// #include <unistd.h>  // 用于读取终端输入

// #include "airsim_ros/Takeoff.h"         // 包含 Takeoff 服务类型
// #include "airsim_ros/VelCmd.h"          // 包含 VelCmd 消息类型
// #include "geometry_msgs/PoseStamped.h"  // 用于订阅 GPS 数据

// // 定义按键
// // 油门
// #define KEYCODE_W 0x77 // w
// #define KEYCODE_S 0x73 // s
// // 偏航角
// #define KEYCODE_A 0x61 // a
// #define KEYCODE_D 0x64 // d
// // 俯仰
// #define KEYCODE_I 0x69 // i
// #define KEYCODE_K 0x6B // k
// // 横滚
// #define KEYCODE_J 0x6A // j
// #define KEYCODE_L 0x6C // l
// // 退出
// #define KEYCODE_Q 0x71 // q (退出)

// // 读取键盘输入的类
// class KeyboardReader {
// public:
//     KeyboardReader() {
//         // 设置终端为非标准模式
//         tcgetattr(STDIN_FILENO, &old_termios_);
//         termios new_termios = old_termios_;
//         new_termios.c_lflag &= ~(ICANON | ECHO); // 关闭行缓冲和回显
//         tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
//     }

//     ~KeyboardReader() {
//         // 恢复终端设置
//         tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
//     }

//     char readKey() {
//         char c;
//         if (read(STDIN_FILENO, &c, 1) < 0) {
//             perror("read():");
//             return '\0';
//         }
//         return c;
//     }

// private:
//     termios old_termios_;
// };

// // 全局变量和互斥锁
// std::mutex mtx;                  // 用于保护共享资源
// char last_key = '\0';            // 存储最近的按键
// bool exit_flag = false;          // 用于标记退出程序

// // 无人机控制变量
// double target_height = -50.0;    // 目标高度
// double current_height = 0.0;     // 当前无人机高度
// double current_x = 0.0;          // 当前X坐标
// double initial_x = 0.0;          // 初始X坐标
// bool init_flag = true;           // 初始化标志
// bool new_flag = true;

// // GPS 回调函数
// void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//     if (init_flag) {
//         initial_x = msg->pose.position.x;
//         init_flag = false;
//     }

//     current_x = msg->pose.position.x;  // 获取无人机当前X坐标
//     current_height = msg->pose.position.z;  // 获取无人机当前高度
// }

// // 键盘输入线程函数
// void keyboardThread(KeyboardReader& keyboard_reader) {
//     while (!exit_flag) {
//         char key = keyboard_reader.readKey(); // 读取按键
//         std::lock_guard<std::mutex> lock(mtx); // 加锁
//         last_key = key; // 更新最后按键值

//         if (key == KEYCODE_Q) {
//             exit_flag = true; // 设置退出标志
//         }
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "keyboard_control");
//     ros::NodeHandle nh;

//     // 发布速度命令的话题
//     ros::Publisher vel_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 10);

//     // 订阅 GPS 数据
//     ros::Subscriber gps_sub = nh.subscribe("/airsim_node/drone_1/gps", 10, gpsCallback);

//     // 调用起飞服务
//     ros::ServiceClient takeoff_client = nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
//     airsim_ros::Takeoff takeoff_srv;
//     takeoff_srv.request.waitOnLastTask = true;

//     ROS_INFO("Calling takeoff service...");
//     if (takeoff_client.call(takeoff_srv)) {
//         ROS_INFO("Takeoff successful!");
//     } else {
//         ROS_ERROR("Failed to call takeoff service.");
//         return 1;
//     }

//     // 创建键盘输入处理线程
//     KeyboardReader keyboard_reader;
//     std::thread kb_thread(keyboardThread, std::ref(keyboard_reader));

//     // 主线程用于处理 ROS 和速度发布
//     ros::Rate loop_rate(50);
//     while (ros::ok() && !exit_flag) {
//         // 获取最后的键盘输入
//         char key;
//         {
//             std::lock_guard<std::mutex> lock(mtx); // 加锁读取
//             key = last_key;
//             new_flag = true;
//         }

//         // 创建速度指令消息
//         airsim_ros::VelCmd vel_cmd;
//         if(new_flag) {
//             // 根据键盘输入设置速度
//             switch (key) {
//             case KEYCODE_W:
//                 ROS_INFO("Moving up");
//                 target_height += -1.0;
//                 break;
//             case KEYCODE_S:
//                 ROS_INFO("Moving down");
//                 target_height += 1.0;
//                 break;
//             case KEYCODE_A:
//                 ROS_INFO("Turning left");
//                 vel_cmd.twist.angular.z += -1.0;
//                 break;
//             case KEYCODE_D:
//                 ROS_INFO("Turning right");
//                 vel_cmd.twist.angular.z += 1.0;
//                 break;

//             case KEYCODE_I:
//                 ROS_INFO("Moving forward");
//                 vel_cmd.twist.linear.x += 1.0;
//                 break;
//             case KEYCODE_K:
//                 ROS_INFO("Moving backward");
//                 vel_cmd.twist.linear.x += -1.0;
//                 break;
//             case KEYCODE_J:
//                 ROS_INFO("Moving left");
//                 vel_cmd.twist.linear.y += 1.0;
//                 break;
//             case KEYCODE_L:
//                 ROS_INFO("Moving right");
//                 vel_cmd.twist.linear.y += -1.0;
//                 break;
            
//             default:
//                 break;
//         }

//         new_flag = false;
//         }

//         // 根据高度误差调整垂直速度
//         double height_error = target_height - current_height;
//         if (std::abs(height_error) > 0.05) {  // 高度误差大于 5cm 时调整
//             vel_cmd.twist.linear.z = 2.5 * height_error;  // 简单比例控制
//         }

//         // 发布速度命令
//         vel_pub.publish(vel_cmd);

//         ROS_INFO("Current height: %.2f, Target height: %.2f, Vertical speed: %.2f",
//                  current_height, target_height, vel_cmd.twist.linear.z);

//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     // 等待键盘线程结束
//     kb_thread.join();
//     ROS_INFO("Exiting program.");
//     return 0;
// }



















#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <thread>    // 用于多线程
#include <mutex>     // 用于线程安全
#include <termios.h> // 用于捕获键盘输入
#include <unistd.h>  // 用于读取终端输入

#include "airsim_ros/Takeoff.h"         // 包含 Takeoff 服务类型
#include "airsim_ros/VelCmd.h"          // 包含 VelCmd 消息类型
#include "geometry_msgs/PoseStamped.h"  // 用于订阅 GPS 数据

// 按键定义
#define KEYCODE_W 0x77 // w
#define KEYCODE_S 0x73 // s
#define KEYCODE_A 0x61 // a
#define KEYCODE_D 0x64 // d
#define KEYCODE_I 0x69 // i
#define KEYCODE_K 0x6B // k
#define KEYCODE_J 0x6A // j
#define KEYCODE_L 0x6C // l
#define KEYCODE_Q 0x71 // q (退出)

// 键盘读取类
class KeyboardReader {
public:
    KeyboardReader() {
        // 设置终端为非标准模式
        tcgetattr(STDIN_FILENO, &old_termios_);
        termios new_termios = old_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO); // 关闭行缓冲和回显
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    }

    ~KeyboardReader() {
        // 恢复终端设置
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }

    char readKey() {
        char c;
        if (read(STDIN_FILENO, &c, 1) < 0) {
            perror("read():");
            return '\0';
        }
        return c;
    }

private:
    termios old_termios_;
};

// 共享变量和锁
std::mutex mtx;                  // 用于保护共享资源
char last_key = '\0';            // 存储最近按键
bool exit_flag = false;          // 标记是否退出程序
bool key_processed = false;      // 标记当前按键是否已被处理

// 无人机控制变量
double target_height = -50.0;    // 目标高度
double current_height = 0.0;     // 当前无人机高度
double current_x = 0.0;          // 当前X坐标
double initial_x = 0.0;          // 初始X坐标
bool init_flag = true;           // 初始化标志

// GPS回调函数
void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (init_flag) {
        initial_x = msg->pose.position.x;
        init_flag = false;
    }

    current_x = msg->pose.position.x;      // 获取无人机当前X坐标
    current_height = msg->pose.position.z; // 获取无人机当前高度
}

// 键盘输入线程
void keyboardThread(KeyboardReader& keyboard_reader) {
    while (!exit_flag) {
        char key = keyboard_reader.readKey(); // 读取按键

        std::lock_guard<std::mutex> lock(mtx); // 加锁保护
        if (!key_processed) { // 如果按键未被处理，更新按键状态
            last_key = key;
            key_processed = true;
        }

        if (key == KEYCODE_Q) {
            exit_flag = true; // 设置退出标志
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;

    // 发布速度命令话题
    ros::Publisher vel_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 10);

    // 订阅GPS数据
    ros::Subscriber gps_sub = nh.subscribe("/airsim_node/drone_1/gps", 10, gpsCallback);

    // 调用起飞服务
    ros::ServiceClient takeoff_client = nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    airsim_ros::Takeoff takeoff_srv;
    takeoff_srv.request.waitOnLastTask = true;

    ROS_INFO("Calling takeoff service...");
    if (takeoff_client.call(takeoff_srv)) {
        ROS_INFO("Takeoff successful!");
    } else {
        ROS_ERROR("Failed to call takeoff service.");
        return 1;
    }

    // 创建键盘输入处理线程
    KeyboardReader keyboard_reader;
    std::thread kb_thread(keyboardThread, std::ref(keyboard_reader));

    // 主线程
    ros::Rate loop_rate(50); // 控制循环频率
    while (ros::ok() && !exit_flag) {
        char key;
        {
            std::lock_guard<std::mutex> lock(mtx); // 加锁保护
            key = last_key; // 获取当前按键
        }

        // 如果按键未被处理
        if (key_processed) {
            // 创建速度指令消息
            airsim_ros::VelCmd vel_cmd;

            // 根据按键输入设置速度
            switch (key) {
            case KEYCODE_W:
                ROS_INFO("Moving up");
                target_height -= 1.0;
                break;
            case KEYCODE_S:
                ROS_INFO("Moving down");
                target_height += 1.0;
                break;
            case KEYCODE_A:
                ROS_INFO("Turning left");
                vel_cmd.twist.angular.z = 1.0;
                break;
            case KEYCODE_D:
                ROS_INFO("Turning right");
                vel_cmd.twist.angular.z = -1.0;
                break;
            case KEYCODE_I:
                ROS_INFO("Moving forward");
                vel_cmd.twist.linear.x = 4.0;
                break;
            case KEYCODE_K:
                ROS_INFO("Moving backward");
                vel_cmd.twist.linear.x = -4.0;
                break;
            case KEYCODE_J:
                ROS_INFO("Moving left");
                vel_cmd.twist.linear.y = 1.0;
                break;
            case KEYCODE_L:
                ROS_INFO("Moving right");
                vel_cmd.twist.linear.y = -1.0;
                break;
            case KEYCODE_Q:
                ROS_INFO("Exiting...");
                exit_flag = true;
                break;
            default:
                break;
            }

            // 发布速度命令
            vel_pub.publish(vel_cmd);
        
            // 重置按键状态
            {
                std::lock_guard<std::mutex> lock(mtx); // 加锁保护
                last_key = '\0';         // 清空按键
                key_processed = false;   // 标记按键已被处理
            }
        }

        // 根据高度误差调整垂直速度
        double height_error = target_height - current_height;
        if (std::abs(height_error) > 0.05) { // 高度误差大于5cm时调整
            airsim_ros::VelCmd vel_cmd;
            vel_cmd.twist.linear.z = 2.5 * height_error; // 简单比例控制
            vel_pub.publish(vel_cmd);
            ROS_INFO("Current height: %.2f, Target height: %.2f, Vertical speed: %.2f",
                 current_height, target_height, vel_cmd.twist.linear.z);
        }



        ros::spinOnce();
        loop_rate.sleep();
    }

    // 等待键盘线程结束
    kb_thread.join();
    ROS_INFO("Exiting program.");
    return 0;
}