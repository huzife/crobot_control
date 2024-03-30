#include "com_control/message/response.h"
#include "com_control/message/utils.h"
#include <assert.h>

namespace crobot {

Response::Response(const std::vector<uint8_t>& data, uint32_t len)
    : data_(data) {
    if (len < 4 ||
        len != data[2] + 3 ||
        data[0] != 0xFE ||
        data[1] != 0xEF)
        type_ = Message_Type::NONE;

    switch (data[3]) {
    case 0x01:
        type_ = Message_Type::SET_VELOCITY;
        break;
    case 0x02:
        type_ = Message_Type::GET_ODOM;
        break;
    case 0x03:
        type_ = Message_Type::GET_IMU_TEMPERATURE;
        break;
    case 0x04:
        type_ = Message_Type::GET_IMU_DATA;
        break;
    case 0x05:
        type_ = Message_Type::GET_ULTRASONIC_RANGE;
        break;
    default:
        type_ = Message_Type::NONE;
        break;
    }
}

Message_Type Response::type() {
    return type_;
}

Get_Odom_Resp Response::get_odom_resp() {
    assert(type_ == Message_Type::GET_ODOM);

    Get_Odom_Resp resp;
    resp.linear_x = hex_to_float(data_, 4);
    resp.linear_y = hex_to_float(data_, 8);
    resp.angular_z = hex_to_float(data_, 12);
    resp.position_x = hex_to_float(data_, 16);
    resp.position_y = hex_to_float(data_, 20);
    resp.direction = hex_to_float(data_, 24);

    return resp;
}

Get_IMU_Temperature_Resp Response::get_imu_temperature_resp() {
    assert(type_ == Message_Type::GET_IMU_TEMPERATURE);

    Get_IMU_Temperature_Resp resp;
    resp.temperature = hex_to_float(data_, 4);

    return resp;
}

Get_IMU_Data_Resp Response::get_imu_resp() {
    assert(type_ == Message_Type::GET_IMU_DATA);

    Get_IMU_Data_Resp resp;
    resp.accel_x = hex_to_float(data_, 4);
    resp.accel_y = hex_to_float(data_, 8);
    resp.accel_z = hex_to_float(data_, 12);
    resp.angular_x = hex_to_float(data_, 16);
    resp.angular_y = hex_to_float(data_, 20);
    resp.angular_z = hex_to_float(data_, 24);

    return resp;
}

Get_Ultrasonic_Range_Resp Response::get_ultrasonic_range_resp() {
    assert(type_ == Message_Type::GET_ULTRASONIC_RANGE);

    Get_Ultrasonic_Range_Resp resp;
    resp.range = (data_[4] << 8) | data_[5];

    return resp;
}

} // namespace crobot
