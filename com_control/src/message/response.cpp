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
    case 0x0: type = Message_Type::NONE; break;
    case 0x1: type = Message_Type::SET_SPEED; break;
    case 0x2: type = Message_Type::GET_SPEED; break;
    // case 0x3: type = Message_Type::GET_TEMP_AND_HUM; break;
    default: return false;
    }

    return true;
}

Message_Type Response::get_type() {
    return type;
}

Get_Speed_Resp Response::get_speed_resp() {
    assert(type == Message_Type::GET_SPEED);
    Get_Speed_Resp speed_info;
    speed_info.linear_x = hex_to_float(data, 4);
    speed_info.linear_y = hex_to_float(data, 8);
    speed_info.angular_z = hex_to_float(data, 12);
    return speed_info;
}

// GetTempAndHumResp Response::get_GetTempAndHumResp() {
// 	assert(type == Message_Type::GET_TEMP_AND_HUM);
// 	GetTempAndHumResp temp_hum_info;
// 	temp_hum_info.temperature = hex_to_float(data, 4);
// 	temp_hum_info.humidity = hex_to_float(data, 8);
// 	return temp_hum_info;
// }

} // namespace crobot