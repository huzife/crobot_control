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
	if (checkSum(data, len) != 0) return false;

	// 检查功能码
	switch (data[3]) {
	case 0x0: type = MessageType::NONE; break;
	case 0x1: type = MessageType::SET_SPEED; break;
	case 0x2: type = MessageType::GET_SPEED; break;
	case 0x3: type = MessageType::GET_TEMP_AND_HUM; break;
	default: return false;
	}

	return true;
}

MessageType Response::getType() {
	return type;
}

GetSpeedResp Response::get_GetSpeedResp() {
	assert(type == MessageType::GET_SPEED);
	GetSpeedResp speed_info;
	speed_info.linear_x = hex_to_float(data, 4);
	speed_info.linear_y = hex_to_float(data, 8);
	speed_info.angular_z = hex_to_float(data, 12);
	return speed_info;
}

GetTempAndHumResp Response::get_GetTempAndHumResp() {
	assert(type == MessageType::GET_TEMP_AND_HUM);
	GetTempAndHumResp temp_hum_info;
	temp_hum_info.temperature = hex_to_float(data, 4);
	temp_hum_info.humidity = hex_to_float(data, 8);
	return temp_hum_info;
}

} // namespace crobot