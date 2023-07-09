#include "com_control/message/base.h"

namespace crobot {

uint8_t checkSum(std::vector<uint8_t> &data, uint32_t len) {
	uint8_t sum = 0;
	for (uint16_t i = 0; i < len; i++) {
		sum += data[i];
	}

	return ~sum;
}

void float_to_hex(float fval, std::vector<uint8_t> &hex, int offset) {
	FloatHex fh;
	fh.fval = fval;
	for (int i = 0; i < 4; i++) {
		hex[i + offset] = fh.hex[3 - i];
	}
}

float hex_to_float(std::vector<uint8_t> &hex, int offset) {
	FloatHex fh;
	for (int i = 0; i < 4; i++) {
		fh.hex[3 - i] = hex[i + offset];
	}
    return fh.fval;
}

} // namespace crobot
