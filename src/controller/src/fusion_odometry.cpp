#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class GPSIMUFusion
{
public:
    GPSIMUFusion(ros::NodeHandle& nh)
    {
        // 从参数服务器获取输入和输出话题名称（可以直接指定默认值）
        std::string gps_topic, imu_topic, odom_topic;
        nh.param<std::string>("gps_topic", gps_topic, "/airsim_node/drone_1/gps");
        nh.param<std::string>("imu_topic", imu_topic, "/airsim_node/drone_1/imu/imu");
        nh.param<std::string>("odom_topic", odom_topic, "/fused_odometry");

        // 初始化订阅者和发布者
        gps_sub_ = nh.subscribe(gps_topic, 10, &GPSIMUFusion::gpsCallback, this);
        imu_sub_ = nh.subscribe(imu_topic, 10, &GPSIMUFusion::imuCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);

        // 初始化变量
        last_time_ = ros::Time::now();
        is_gps_initialized_ = false;
        is_offset_initialized_ = false;
    }

private:
    ros::Subscriber gps_sub_;   // GPS订阅者
    ros::Subscriber imu_sub_;   // IMU订阅者
    ros::Publisher odom_pub_;   // 里程计发布者
    tf::TransformBroadcaster tf_broadcaster_; // TF 广播器

    geometry_msgs::PoseStamped latest_gps_;   // 最新的 GPS 数据
    sensor_msgs::Imu latest_imu_;             // 最新的 IMU 数据
    bool is_gps_initialized_;                 // GPS 是否已初始化
    bool is_offset_initialized_;              // 偏移量是否已初始化
    ros::Time last_time_;                     // 上次更新的时间

    // 初始偏移量
    double offset_x_;
    double offset_y_;
    double offset_z_;

    // GPS 数据回调函数
    void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& gps_msg)
    {
        latest_gps_ = *gps_msg;
        is_gps_initialized_ = true; // 标记 GPS 已初始化

        // 如果偏移量还未初始化，则记录初始 GPS 位置作为偏移量
        if (!is_offset_initialized_)
        {
            offset_x_ = latest_gps_.pose.position.x;
            offset_y_ = latest_gps_.pose.position.y;
            offset_z_ = latest_gps_.pose.position.z;
            is_offset_initialized_ = true; // 标记偏移量已初始化
            ROS_INFO("GPS offset initialized: x=%.3f, y=%.3f, z=%.3f", offset_x_, offset_y_, offset_z_);
        }
    }

    // IMU 数据回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
    {
        if (!is_gps_initialized_)
        {
            ROS_WARN("Waiting for GPS initialization...");
            return; // 如果 GPS 信息尚未初始化，则跳过
        }

        latest_imu_ = *imu_msg;

        // 获取当前时间
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        // 创建里程计消息
        nav_msgs::Odometry odom_msg;

        // 填充 header
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "map"; // 里程计的全局坐标系
        odom_msg.child_frame_id = "base_link"; // 基于无人机的坐标系

        // 使用 GPS 数据填充位置，减去偏移量以从原点开始
        odom_msg.pose.pose.position.x = latest_gps_.pose.position.x - offset_x_;
        odom_msg.pose.pose.position.y = latest_gps_.pose.position.y - offset_y_;
        odom_msg.pose.pose.position.z = -(latest_gps_.pose.position.z - offset_z_);

        // 使用 IMU 数据填充方向（四元数）
        odom_msg.pose.pose.orientation = latest_imu_.orientation;

        // 使用 IMU 数据填充速度
        odom_msg.twist.twist.linear.x = latest_imu_.linear_acceleration.x * dt;
        odom_msg.twist.twist.linear.y = latest_imu_.linear_acceleration.y * dt;
        odom_msg.twist.twist.linear.z = latest_imu_.linear_acceleration.z * dt;

        // 使用 IMU 数据填充角速度
        odom_msg.twist.twist.angular.x = latest_imu_.angular_velocity.x;
        odom_msg.twist.twist.angular.y = latest_imu_.angular_velocity.y;
        odom_msg.twist.twist.angular.z = latest_imu_.angular_velocity.z;

        // 发布里程计消息
        odom_pub_.publish(odom_msg);

        // 广播 TF 变换
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "map";
        odom_tf.child_frame_id = "base_link";

        // 变换数据
        odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

        // 发送 TF 变换
        tf_broadcaster_.sendTransform(odom_tf);
    }
};

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "gps_imu_fusion");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建融合对象
    GPSIMUFusion fusion(nh);

    // 运行 ROS 循环
    ros::spin();

    return 0;
}