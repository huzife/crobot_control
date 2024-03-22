#ifndef COM_CONTROL_CONTROLLER_CALLBACK_H
#define COM_CONTROL_CONTROLLER_CALLBACK_H

#include "com_control/message/response.h"

namespace crobot {

class Controller_Callbacks {
public:
    virtual void set_velocity_callback() = 0;

    virtual void get_odom_callback(const Get_Odom_Resp& resp) = 0;

    virtual void get_imu_temperature_callback(const Get_IMU_Temperature_Resp& resp) = 0;

    virtual void get_imu_data_callback(const Get_IMU_Data_Resp& resp) = 0;
};

}

#endif  // COM_CONTROL_CONTROLLER_CALLBACK_H
