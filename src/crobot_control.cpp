#include "crobot_control/crobot_control.h"

namespace crobot_ros {

CRobot_Control::CRobot_Control(ros::NodeHandle nh,
                               ros::NodeHandle nh_private,
                               crobot::Controller_Callbacks& cbs)
    : nh_(nh),
      nh_private_(nh_private),
      controller_(cbs) {}

CRobot_Control::~CRobot_Control() {
    thread_end_ = true;
    get_odom_thread_.join();
    get_imu_temperature_thread_.join();
    get_imu_data_thread_.join();
    get_battery_voltage_thread_.join();
}

void CRobot_Control::init() {
    std::string port_name = "/dev/ttyUSB0";
    nh_private_.getParam("port_name", port_name);

    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 10, std::bind(&CRobot_Control::twist_subscribe_CB,
                                  this, std::placeholders::_1));

    controller_.init(port_name.c_str(),
                    itas109::BaudRate115200,
                    itas109::ParityNone,
                    itas109::DataBits8,
                    itas109::StopOne,
                    itas109::FlowNone);
}

void CRobot_Control::start() {
    init();
    controller_.open();

    get_odom_thread_ =
        std::thread{std::bind(&CRobot_Control::get_odom_func, this)};
    get_imu_temperature_thread_ =
        std::thread{std::bind(&CRobot_Control::get_imu_temperature_func,
                              this)};
    get_imu_data_thread_ =
        std::thread{std::bind(&CRobot_Control::get_imu_data_func, this)};
    get_ultrasonic_range_thread_ =
        std::thread{std::bind(&CRobot_Control::get_ultrasonic_range_func, this)};
    get_battery_voltage_thread_ =
        std::thread{std::bind(&CRobot_Control::get_battery_voltage_func, this)};
}

void CRobot_Control::twist_subscribe_CB(
    const geometry_msgs::Twist::ConstPtr& msg) {
    controller_.send_request(
        crobot::Set_Velocity_Req{static_cast<float>(msg->linear.x),
                                 static_cast<float>(msg->linear.y),
                                 static_cast<float>(msg->angular.z)});
}

void CRobot_Control::get_odom_func() {
    ros::Rate rate(20);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_Odom_Req{});
    }
}

void CRobot_Control::get_imu_temperature_func() {
    ros::Rate rate(1);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_IMU_Temperature_Req{});
    }
}

void CRobot_Control::get_imu_data_func() {
    ros::Rate rate(100);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_IMU_Data_Req{});
    }
}

void CRobot_Control::get_ultrasonic_range_func() {
    ros::Rate rate(20);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_Ultrasonic_Range_Req{});
    }
}

void CRobot_Control::get_battery_voltage_func() {
    ros::Rate rate(1);
    while (!thread_end_) {
        rate.sleep();
        controller_.send_request(crobot::Get_Battery_Voltage_Req{});
    }
}

} // namespace crobot_ros
