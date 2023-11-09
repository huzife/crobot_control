#ifndef _COMCONTROL_RESPONSE_H
#define _COMCONTROL_RESPONSE_H

#include "base.h"
#include <vector>

namespace crobot {

class Get_Speed_Resp {
public:
    float linear_x;
    float linear_y;
    float angular_z;
};

// class GetTempAndHumResp {
// public:
// 	float temperature;
// 	float humidity;
// };

class Response {
private:
    Message_Type type;
    std::vector<uint8_t> data;

public:
    Response(const std::vector<uint8_t> &data) : data(data) {}
    bool parse();
    Message_Type get_type();
    Get_Speed_Resp get_speed_resp();
    // GetTempAndHumResp get_GetTempAndHumResp();
};

} // namespace crobot

#endif // _COMCONTROL_RESPONSE_H