#ifndef COM_CONTROL_MESSAGE_UTILS_H
#define COM_CONTROL_MESSAGE_UTILS_H

#include <cstdint>
#include <vector>

namespace crobot {

union Float_Hex {
    uint8_t hex[4];
    float fval;
};

void float_to_hex(float fval, std::vector<uint8_t>& hex, int offset);
float hex_to_float(const std::vector<uint8_t>& hex, int offset);

}

#endif  // COM_CONTROL_MESSAGE_UTILS_H
