#ifndef _COMCONTROL_REQUEST_H
#define _COMCONTROL_REQUEST_H

#include "base.h"
#include <vector>

namespace crobot {

class SetSpeedReq {
public:
	float linear_x;
	float linear_y;
	float angular_z;
};

class Request {
private:
	MessageType type;
	std::vector<uint8_t> data;

public:
	Request(const std::vector<uint8_t> &raw_data, MessageType type);
	Request(MessageType type);
	std::vector<uint8_t> getData();
};

} // namespace crobot

#endif // _COMCONTROL_REQUEST_H