#include "com_control/data_parser.h"

namespace crobot {

Data_Parser::Data_Parser(uint32_t buf_len) {
    this->buf_len = buf_len < 128 ? 128 : buf_len;
    buf.resize(this->buf_len);
}

void Data_Parser::parse(uint8_t data) {
    if (!flag) {
        buf[data_len] = data;

        switch (data) {
        case 0xFE:
            FE_flag = true;
            break;
        case 0xEF:
            if (FE_flag) {
                FE_flag = false;
                FH_flag = true;
                buf[0] = 0xFE;
                buf[1] = 0xEF;
                buf[2] = buf_len;
                data_len = 1;
            }
            break;
        default:
            FE_flag = 0;
            break;
        }
    }

    if (FH_flag && (data_len > buf[2] + 2)) {
        flag = !check_sum(buf, buf[2] + 4);
        FE_flag = false;
        FH_flag = false;
        data_len = 0;
    }

    if (++data_len >= buf_len)
        data_len = 0;
}

Response Data_Parser::get_response() {
    flag = false;
    return Response{buf, buf[2] + 4};
}

} // namespace crobot
