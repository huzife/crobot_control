#include "crobot_control/crobot_control.h"

namespace crobot_ros {

Crobot_Control::Crobot_Control(ros::NodeHandle nh,
                               ros::NodeHandle nh_private,
                               crobot::Controller_Callbacks& cbs)
    : nh_(nh),
      nh_private_(nh_private),
      controller_(cbs) {}

Crobot_Control::~Crobot_Control() {
    thread_end_ = true;
    get_odometry_thread_.join();
    get_imu_temperature_thread_.join();
    get_imu_data_thread_.join();
    get_battery_voltage_thread_.join();
}

void Crobot_Control::init() {
    std::string port_name = "/dev/ttyUSB0";
    nh_private_.getParam("port_name", port_name);

    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 10, std::bind(&Crobot_Control::twist_subscribe_CB,
                                  this, std::placeholders::_1));

    controller_.init(port_name.c_str(),
                    itas109::BaudRate115200,
                    itas109::ParityNone,
                    itas109::DataBits8,
                    itas109::StopOne,
                    itas109::FlowNone);
}

void Crobot_Control::start() {
    init();
    controller_.open();

    get_odometry_thread_ =
        std::thread{std::bind(&Crobot_Control::get_odometry_func, this)};
    get_imu_temperature_thread_ =
        std::thread{std::bind(&Crobot_Control::get_imu_temperature_func,
                              this)};
    get_imu_data_thread_ =
        std::thread{std::bind(&Crobot_Control::get_imu_data_func, this)};
    get_ultrasonic_range_thread_ =
        std::thread{std::bind(&Crobot_Control::get_ultrasonic_range_func, this)};
    get_battery_voltage_thread_ =
        std::thread{std::bind(&Crobot_Control::get_battery_voltage_func, this)};
}

void Crobot_Control::twist_subscribe_CB(
    const geometry_msgs::Twist::ConstPtr& msg) {
    controller_.send_request(
        crobot::Set_Velocity_Req{static_cast<float>(msg->linear.x),
                                 static_cast<float>(msg->linear.y),
                                 static_cast<float>(msg->angular.z)});
}

void Crobot_Control::get_odometry_func() {
    ros::Rate rate(20);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_Odometry_Req{});
    }
}

void Crobot_Control::get_imu_temperature_func() {
    ros::Rate rate(1);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_IMU_Temperature_Req{});
    }
}

void Crobot_Control::get_imu_data_func() {
    ros::Rate rate(100);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_IMU_Data_Req{});
    }
}

void Crobot_Control::get_ultrasonic_range_func() {
    ros::Rate rate(20);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_Ultrasonic_Range_Req{});
    }
}

void Crobot_Control::get_battery_voltage_func() {
    ros::Rate rate(1);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_Battery_Voltage_Req{});
    }
}

} // namespace crobot_ros
