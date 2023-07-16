#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "com_control/controller.h"
#include "com_control/controller_callbacks.h"
#include <functional>
#include <iostream>

class CB : public crobot::ControllerCallbacks {
public:
    void setSpeedCallback() override {}

    void getSpeedCallback(const crobot::GetSpeedResp &resp) override {
        std::cout << "linear_x: " << resp.linear_x << ' '
                  << "angular_z: " << resp.angular_z << std::endl;
    }

    void getTempAndHumCallback(const crobot::GetTempAndHumResp &resp) override {
        std::cout << "temperature: " << resp.temperature << ' '
                  << "humidity: " << resp.humidity << std::endl;
    } 
};

class Test {
public:
    crobot::Controller controller;
    CB cbs;

    Test() : controller(cbs) {
        controller.init("/dev/ttyUSB0", 
                        itas109::BaudRate115200,
                        itas109::ParityNone,
                        itas109::DataBits8,
                        itas109::StopOne,
                        itas109::FlowNone,
                        1024);
        controller.open();
    }

    void twistSubscribeCB(const geometry_msgs::Twist::ConstPtr &msg) {
        ROS_INFO("receive twist");
        crobot::SetSpeedReq speed_req;
        speed_req.linear_x = msg->linear.x;
        speed_req.linear_y = msg->linear.y;
        speed_req.angular_z = msg->angular.z;

        controller.setSpeed(speed_req);
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "crobot_control");
    ros::NodeHandle nHandle;
    Test t;

    ros::Subscriber twist_sub = nHandle.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
        std::bind(&Test::twistSubscribeCB, &t, std::placeholders::_1));

    ros::Rate rate(10);
    int i = 0;

    while (true) {
        rate.sleep();

        i++;
        if (i == 10) {
            t.controller.getSpeed();
            t.controller.getTempAndHum();
            i = 0;
        }

        ros::spinOnce();
    }


    return 0;
}