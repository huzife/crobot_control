#include "crobot_control/crobot_controller_callbacks.h"
#include "crobot_control/TempAndHum.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <cmath>

namespace crobot_ros {

void ControllerCB::set_speed_callback() {}

void ControllerCB::get_speed_callback(const crobot::Get_Speed_Resp &resp) {
    // geometry_msgs::Twist vel;
    // vel.linear.x = resp.linear_x;
    // vel.linear.y = resp.linear_y;
    // vel.angular.z = resp.angular_z;
    // velocity_pub.publish(vel);

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

// void ControllerCB::getTempAndHumCallback(const crobot::GetTempAndHumResp &resp) {
//     crobot_control::TempAndHum temp_hum;
//     temp_hum.temperature = resp.temperature;
//     temp_hum.humidity = resp.humidity;
//     temp_hum_pub.publish(temp_hum);
// }

void ControllerCB::init() {
    temp_hum_pub = nh.advertise<crobot_control::TempAndHum>("temp_and_hum", 10);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("velocity", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    current_x = 0.0f;
    current_y = 0.0f;
    current_yaw = 0.0f;
}

}