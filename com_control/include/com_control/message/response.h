#ifndef COM_CONTROL_MESSAGE_RESPONSE_H
#define COM_CONTROL_MESSAGE_RESPONSE_H

#include "type.h"
#include "utils.h"
#include <vector>

namespace crobot {

struct Get_Speed_Resp {
    float linear_x;
    float linear_y;
    float angular_z;
};

struct Get_IMU_Temperature_Resp {
    float temperature;
};

struct Get_IMU_Resp {
    float accel_x;
    float accel_y;
    float accel_z;
    float angular_x;
    float angular_y;
    float angular_z;
};

class Response {
private:
    Message_Type type;
    std::vector<uint8_t> data;

public:
    Response(const std::vector<uint8_t>& data, uint32_t len);

    Message_Type get_type();
    Get_Speed_Resp get_speed_resp();
    Get_IMU_Temperature_Resp get_imu_temperature_resp();
    Get_IMU_Resp get_imu_resp();
};

} // namespace crobot

#endif // COM_CONTROL_MESSAGE_RESPONSE_H
