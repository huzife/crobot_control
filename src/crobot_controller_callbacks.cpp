#include "crobot_control/crobot_controller_callbacks.h"
#include "crobot_control/TempAndHum.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

namespace crobot_ros {

void ControllerCB::setSpeedCallback() {}

void ControllerCB::getSpeedCallback(const crobot::GetSpeedResp &resp) {
    geometry_msgs::Twist vel;
    vel.linear.x = resp.linear_x;
    vel.linear.y = resp.linear_y;
    vel.angular.z = resp.angular_z;
    velocity_pub.publish(vel);
}

void ControllerCB::getTempAndHumCallback(const crobot::GetTempAndHumResp &resp) {
    crobot_control::TempAndHum temp_hum;
    temp_hum.temperature = resp.temperature;
    temp_hum.humidity = resp.humidity;
    temp_hum_pub.publish(temp_hum);
}

void ControllerCB::init() {
    temp_hum_pub = nh.advertise<crobot_control::TempAndHum>("temp_and_hum", 10);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("velocity", 10);
}

}