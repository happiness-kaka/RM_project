#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
// #include "sensor_msgs/NavSatFix.h"  // 包含 GPS 消息类型的头文件

// 回调函数，用于处理接收到的 GPS 数据
void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("Received GPS Data:");
    ROS_INFO("x: %f", msg->pose.position.x);  // x轴
    ROS_INFO("y: %f", msg->pose.position.y);  // y轴
    ROS_INFO("z: %f", msg->pose.position.z);  // z轴

    // 打印姿态信息（四元数）
    ROS_INFO("Orientation -> x: %f, y: %f, z: %f, w: %f",
             msg->pose.orientation.x, msg->pose.orientation.y,
             msg->pose.orientation.z, msg->pose.orientation.w);
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "gps_subscriber");
    
    // 创建节点句柄
    ros::NodeHandle nh;

    // 订阅 /airsim_node/drone_1/gps 话题
    ros::Subscriber gps_sub = nh.subscribe("/airsim_node/drone_1/gps", 10, gpsCallback);

    // 进入循环，等待消息
    ros::spin();

    return 0;
}