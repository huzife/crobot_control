#ifndef COM_CONTROL_CONTROLLER_CALLBACK_H
#define COM_CONTROL_CONTROLLER_CALLBACK_H

#include "com_control/message/response.h"

namespace crobot {

class Controller_Callbacks {
public:
    virtual void set_speed_callback() = 0;

    virtual void get_speed_callback(const Get_Speed_Resp& resp) = 0;

    virtual void get_imu_temperature_callback(const Get_IMU_Temperature_Resp& resp) = 0;

    virtual void get_imu_callback(const Get_IMU_Resp& resp) = 0;
};

}

#endif  // COM_CONTROL_CONTROLLER_CALLBACK_H
