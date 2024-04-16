#include "crobot_control/crobot_control.h"
#include "crobot_control/crobot_controller_callbacks.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "crobot_control_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    crobot_ros::Crobot_Control_Callbacks callbacks(nh, nh_private);
    crobot_ros::Crobot_Control crobot_control(nh, nh_private, callbacks);

    crobot_control.start();

    ros::spin();

    return 0;
}
