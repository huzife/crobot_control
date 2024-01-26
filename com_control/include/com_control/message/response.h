#ifndef COM_CONTROL_MESSAGE_RESPONSE_H
#define COM_CONTROL_MESSAGE_RESPONSE_H

#include "type.h"
#include <cstdint>
#include <vector>

namespace crobot {

struct Get_Odom_Resp {
    float linear_x;
    float angular_z;
    float position_x;
    float position_y;
    float direction;
};

struct Get_IMU_Temperature_Resp {
    float temperature;
};

struct Get_IMU_Data_Resp {
    float accel_x;
    float accel_y;
    float accel_z;
    float angular_x;
    float angular_y;
    float angular_z;
};

class Response {
private:
    Message_Type type_;
    std::vector<uint8_t> data_;

public:
    Response(const std::vector<uint8_t>& data, uint32_t len);

    Message_Type type();
    Get_Odom_Resp get_odom_resp();
    Get_IMU_Temperature_Resp get_imu_temperature_resp();
    Get_IMU_Data_Resp get_imu_resp();
};

} // namespace crobot

#endif // COM_CONTROL_MESSAGE_RESPONSE_H
