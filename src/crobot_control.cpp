#include "crobot_control/crobot_control.h"

namespace crobot_ros {

void CRobot_Control::init() {
    if (!nh_private.getParam("port_name", port_name))
        port_name = "/dev/ttyUSB0";

    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",
                                                     10,
                                                     std::bind(&CRobot_Control::twist_subscribe_CB,
                                                               this,
                                                               std::placeholders::_1));

    controller.init(port_name.c_str(),
                    itas109::BaudRate115200,
                    itas109::ParityNone,
                    itas109::DataBits8,
                    itas109::StopOne,
                    itas109::FlowNone,
                    4096);
}

void CRobot_Control::start() {
    init();
    controller.open();

    // create threads
    // std::thread{std::bind(&CRobot_Control::get_speed_func, this)}.detach();
    std::thread{std::bind(&CRobot_Control::get_imu_temperature_func, this)}.detach();
    std::thread{std::bind(&CRobot_Control::get_imu_func, this)}.detach();
}

void CRobot_Control::twist_subscribe_CB(const geometry_msgs::Twist::ConstPtr& msg) {
    crobot::Set_Speed_Req speed_req;
    speed_req.linear_x = msg->linear.x;
    speed_req.linear_y = msg->linear.y;
    speed_req.angular_z = msg->angular.z;

    controller.set_speed(speed_req);
}

void CRobot_Control::get_speed_func() {
    ros::Rate rate(20);
    while (true) {
        rate.sleep();
        controller.get_speed();
    }
}

void CRobot_Control::get_imu_temperature_func() {
    ros::Rate rate(1);
    while (true) {
        rate.sleep();
        controller.get_imu_temperature();
    }
}

void CRobot_Control::get_imu_func() {
    ros::Rate rate(100);
    while (true) {
        rate.sleep();
        controller.get_imu();
    }
}

} // namespace crobot_ros
