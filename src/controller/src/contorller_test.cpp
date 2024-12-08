#include "ros/ros.h"
#include "airsim_ros/Takeoff.h"  // 包含 Takeoff 服务类型
#include "airsim_ros/VelCmd.h"  // 包含 VelCmd 消息类型
#include "geometry_msgs/PoseStamped.h"  // 用于订阅 GPS 数据

// 目标高度
const double TARGET_HEIGHT = -50.0;

double target_height = -50.0;
// 当前无人机的高度
double current_height = 0.0;
double current_x = 0.0;

double initial_x = 0.0;
bool init_flag = true;

// GPS 回调函数
void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    
    if(init_flag) {
        initial_x = msg->pose.position.x;
        init_flag = false;
    }

    current_x = msg->pose.position.x;  // 获取无人机当前高度
    current_height = msg->pose.position.z;  // 获取无人机当前高度

}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "drone_hover_control");
    ros::NodeHandle nh;

    // 调用起飞服务
    ros::ServiceClient takeoff_client = nh.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    // 定义一个服务器
    airsim_ros::Takeoff takeoff_srv;
    ROS_INFO("start fly!!");

    takeoff_srv.request.waitOnLastTask = true;  // 根据服务定义设置参数 

    if (takeoff_client.call(takeoff_srv)) {
        ROS_INFO("Takeoff service called successfully!");
    } else {
        ROS_ERROR("Failed to call takeoff service.");
        return 1;
    }

    // 订阅 GPS 数据，用于获取当前高度
    ros::Subscriber gps_sub = nh.subscribe("/airsim_node/drone_1/gps", 10, gpsCallback);

    // 创建速度指令发布者
    ros::Publisher vel_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 10);
    
    // 控制频率
    ros::Rate rate(50);  // 10 Hz
    ROS_INFO("start while!!");

    ros::Time start = ros::Time::now();
    while (ros::ok()) {
        // 创建速度指令消息
        airsim_ros::VelCmd vel_cmd;
        
        if(current_x < initial_x - 5) {
            target_height = -53.0;
        }

        double height_error = target_height - current_height;

        // 根据高度误差调整垂直速度
        if (std::abs(height_error) > 0.05) {  // 高度误差大于 5cm 时调整
            vel_cmd.twist.linear.z = 0.5 * height_error;  // 简单比例控制
        } 

        // 其他线速度和角速度设为 0
        vel_cmd.twist.linear.x = 3.0;
        vel_cmd.twist.linear.y = 0.0;
        vel_cmd.twist.angular.x = 0.0;
        vel_cmd.twist.angular.y = 0.0;
        vel_cmd.twist.angular.z = 0.0;

        // 发布速度指令
        vel_pub.publish(vel_cmd);

        ROS_INFO("Current height: %.2f, Target height: %.2f, Vertical speed: %.2f",
                 current_height, target_height, vel_cmd.twist.linear.z);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}