#ifndef _COMCONTROL_MESSAGE_BASE_H
#define _COMCONTROL_MESSAGE_BASE_H

#include <cstdint>
#include <vector>

namespace crobot {

union FloatHex {
    uint8_t hex[4];
    float fval;
};

enum class MessageType {
    NONE,
    SET_SPEED,
    GET_SPEED,
    GET_TEMP_AND_HUM
};

uint8_t checkSum(std::vector<uint8_t> &data, uint32_t len);
void float_to_hex(float fval, std::vector<uint8_t> &hex, int offset);
float hex_to_float(std::vector<uint8_t> &hex, int offset);

}

#endif  // _COMCONTROL_MESSAGE_BASE_H