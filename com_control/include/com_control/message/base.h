#ifndef _COMCONTROL_MESSAGE_BASE_H
#define _COMCONTROL_MESSAGE_BASE_H

#include <cstdint>
#include <vector>

namespace crobot {

union Float_Hex {
    uint8_t hex[4];
    float fval;
};

enum class Message_Type {
    NONE,
    SET_SPEED,
    GET_SPEED,
    INIT_IMU,
    GET_IMU
    // GET_TEMP_AND_HUM
};

uint8_t check_sum(std::vector<uint8_t> &data, uint32_t len);
void float_to_hex(float fval, std::vector<uint8_t> &hex, int offset);
float hex_to_float(std::vector<uint8_t> &hex, int offset);

}

#endif  // _COMCONTROL_MESSAGE_BASE_H