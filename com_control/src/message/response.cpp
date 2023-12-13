#include "com_control/message/response.h"
#include <assert.h>

namespace crobot {

bool Response::parse() {
    // 检查长度
    size_t len = data.size();
    if (len < 5) return false;
    if (len != data[2] + 4) return false;

    // 检查帧头
    if (!(data[0] == 0xFE && data[1] == 0xEF)) return false;

    // 检查校验和
    if (check_sum(data, len) != 0) return false;

    // 检查功能码
    switch (data[3]) {
    case 0x00: type = Message_Type::NONE; break;
    case 0x01: type = Message_Type::SET_SPEED; break;
    case 0x02: type = Message_Type::GET_SPEED; break;
    case 0x03: type = Message_Type::GET_IMU_TEMPERATURE; break;
    case 0x04: type = Message_Type::GET_IMU; break;
    default: return false;
    }

    return true;
}

Message_Type Response::get_type() {
    return type;
}

Get_Speed_Resp Response::get_speed_resp() {
    assert(type == Message_Type::GET_SPEED);

    Get_Speed_Resp resp;
    resp.linear_x = hex_to_float(data, 4);
    resp.linear_y = hex_to_float(data, 8);
    resp.angular_z = hex_to_float(data, 12);

    return resp;
}

Get_IMU_Temperature_Resp Response::get_imu_temperature_resp() {
    assert(type == Message_Type::GET_IMU_TEMPERATURE);

    Get_IMU_Temperature_Resp resp;
    resp.temperature = hex_to_float(data, 4);

    return resp;
}

Get_IMU_Resp Response::get_imu_resp() {
    assert(type == Message_Type::GET_IMU);

    Get_IMU_Resp resp;
    resp.accel_x = hex_to_float(data, 4);
    resp.accel_y = hex_to_float(data, 8);
    resp.accel_z = hex_to_float(data, 12);
    resp.angular_x = hex_to_float(data, 16);
    resp.angular_y = hex_to_float(data, 20);
    resp.angular_z = hex_to_float(data, 24);

    return resp;
}

} // namespace crobot
