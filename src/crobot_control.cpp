#include "crobot_control/crobot_control.h"

namespace crobot_ros {

CRobot_Control::CRobot_Control(ros::NodeHandle nh,
                               ros::NodeHandle nh_private,
                               crobot::Controller_Callbacks& cbs)
    : nh(nh),
      nh_private(nh_private),
      controller(cbs) {}

CRobot_Control::~CRobot_Control() {
    thread_end = true;
    // get_speed_thread.join();
    get_imu_temperature_thread.join();
    get_imu_data_thread.join();
}

void CRobot_Control::init() {
    std::string port_name = "/dev/ttyUSB0";
    nh_private.getParam("port_name", port_name);

    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 10, std::bind(&CRobot_Control::twist_subscribe_CB,
                                  this, std::placeholders::_1));

    controller.init(port_name.c_str(),
                    itas109::BaudRate115200,
                    itas109::ParityNone,
                    itas109::DataBits8,
                    itas109::StopOne,
                    itas109::FlowNone);
}

void CRobot_Control::start() {
    init();
    controller.open();

    // get_speed_thread =
    //    std::thread{std::bind(&CRobot_Control::get_speed_func, this)};
    get_imu_temperature_thread =
        std::thread{std::bind(&CRobot_Control::get_imu_temperature_func,
                              this)};
    get_imu_data_thread =
        std::thread{std::bind(&CRobot_Control::get_imu_data_func, this)};
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
    while (!thread_end) {
        rate.sleep();
        controller.get_speed();
    }
}

void CRobot_Control::get_imu_temperature_func() {
    ros::Rate rate(1);
    while (!thread_end) {
        rate.sleep();
        controller.get_imu_temperature();
    }
}

void CRobot_Control::get_imu_data_func() {
    ros::Rate rate(100);
    while (!thread_end) {
        rate.sleep();
        controller.get_imu();
    }
}

} // namespace crobot_ros
