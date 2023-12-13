#ifndef COM_CONTROL_MESSAGE_REQUEST_H
#define COM_CONTROL_MESSAGE_REQUEST_H

#include "type.h"
#include "utils.h"
#include <vector>

namespace crobot {

class Set_Speed_Req {
public:
    float linear_x;
    float linear_y;
    float angular_z;
};

class Request {
private:
    Message_Type type;
    std::vector<uint8_t> data;

public:
    Request(const std::vector<uint8_t>& raw_data, Message_Type type);
    Request(Message_Type type);
    std::vector<uint8_t> get_data();
};

} // namespace crobot

#endif // COM_CONTROL_MESSAGE_REQUEST_H
