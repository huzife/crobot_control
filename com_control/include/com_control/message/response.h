#ifndef _COMCONTROL_RESPONSE_H
#define _COMCONTROL_RESPONSE_H

#include "base.h"
#include <vector>

namespace crobot {

class GetSpeedResp {
public:
	float linear_x;
	float linear_y;
	float angular_z;
};

class GetTempAndHumResp {
public:
	float temperature;
	float humidity;
};

class Response {
private:
	MessageType type;
	std::vector<uint8_t> data;

public:
	Response(const std::vector<uint8_t> &data) : data(data) {}
	bool parse();
	MessageType getType();
	GetSpeedResp get_GetSpeedResp();
	GetTempAndHumResp get_GetTempAndHumResp();
};

} // namespace crobot

#endif // _COMCONTROL_RESPONSE_H