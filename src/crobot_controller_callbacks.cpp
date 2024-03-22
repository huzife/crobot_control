#include "crobot_control/crobot_controller_callbacks.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

namespace crobot_ros {

void Controller_CB::set_velocity_callback() {}

void Controller_CB::get_odom_callback(const crobot::Get_Odom_Resp& resp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // 位姿
    odom.pose.pose.position.x = resp.position_x;
    odom.pose.pose.position.y = resp.position_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(resp.direction);

    // 速度
    odom.twist.twist.linear.x = resp.linear_x;
    odom.twist.twist.linear.y = resp.linear_y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = resp.angular_z;

    odom_pub.publish(odom);
}

void Controller_CB::get_imu_temperature_callback(const crobot::Get_IMU_Temperature_Resp& resp) {
    std_msgs::Float32 temperature;
    temperature.data = resp.temperature;

    imu_temperature_pub.publish(temperature);
}

void Controller_CB::get_imu_data_callback(const crobot::Get_IMU_Data_Resp& resp) {
    sensor_msgs::Imu raw_imu_data;
    raw_imu_data.header.stamp = ros::Time::now();
    raw_imu_data.header.frame_id = "imu_link";

    raw_imu_data.linear_acceleration.x = resp.accel_x;
    raw_imu_data.linear_acceleration.y = resp.accel_y;
    raw_imu_data.linear_acceleration.z = resp.accel_z;
    raw_imu_data.angular_velocity.x = resp.angular_x;
    raw_imu_data.angular_velocity.y = resp.angular_y;
    raw_imu_data.angular_velocity.z = resp.angular_z;

    imu_raw_pub.publish(raw_imu_data);
}

void Controller_CB::init() {
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu/raw_data", 10);
    imu_temperature_pub = nh.advertise<std_msgs::Float32>("imu/temperature", 10);
}

} // namespace crobot_ros
