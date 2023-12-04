#include "com_control/message/request.h"

namespace crobot {

Request::Request(const std::vector<uint8_t>& raw_data, Message_Type type) {
    size_t len = raw_data.size();

    data.resize(4);
    data[0] = 0xFE;
    data[1] = 0xEF;
    data[2] = len + 1;
    data[3] = static_cast<uint8_t>(type);

    if (len > 0) {
        data.insert(data.end(), raw_data.begin(), raw_data.end());
    }
    data.push_back(check_sum(data, len + 4));
}

Request::Request(Message_Type type) {
    data.resize(5);
    data[0] = 0xFE;
    data[1] = 0xEF;
    data[2] = 0x01;
    data[3] = static_cast<uint8_t>(type);
    data[4] = check_sum(data, 4);
}

std::vector<uint8_t> Request::get_data() {
    return data;
}

} // namespace crobot
