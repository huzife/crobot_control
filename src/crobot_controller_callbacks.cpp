#include "crobot_control/crobot_controller_callbacks.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <cmath>

namespace crobot_ros {

void Controller_CB::set_speed_callback() {}

void Controller_CB::get_speed_callback(const crobot::Get_Speed_Resp& resp) {
    double vx = resp.linear_x;
    double vy = resp.linear_y;
    double vz = resp.angular_z;

    current_time = ros::Time::now();

    double delta_time = (current_time - last_time).toSec();
    double delta_x = (vx * std::cos(vz) - vy * std::sin(vz)) * delta_time;
    double delta_y = (vx * std::sin(vz) + vy * std::cos(vz)) * delta_time;
    double delta_yaw = vz * delta_time;

    current_x += delta_x;
    current_y += delta_y;
    current_yaw += delta_yaw;

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(current_yaw);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // 位姿
    odom.pose.pose.position.x = current_x;
    odom.pose.pose.position.y = current_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = quat;

    // 速度
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = vz;

    // 发布
    odom_pub.publish(odom);

    last_time = current_time;
}

void Controller_CB::get_imu_temperature_callback(const crobot::Get_IMU_Temperature_Resp& resp) {
    std_msgs::Float32 temperature;
    temperature.data = resp.temperature;

    imu_temperature_pub.publish(temperature);
}

void Controller_CB::get_imu_callback(const crobot::Get_IMU_Data_Resp& resp) {
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
    velocity_pub = nh.advertise<geometry_msgs::Twist>("velocity", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    imu_raw_pub = nh.advertise<sensor_msgs::Imu>("imu/raw_data", 10);
    imu_temperature_pub = nh.advertise<std_msgs::Float32>("imu/temperature", 10);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    current_x = 0.0f;
    current_y = 0.0f;
    current_yaw = 0.0f;
}

} // namespace crobot_ros
