#ifndef COM_CONTROL_DATA_PARSER_H
#define COM_CONTROL_DATA_PARSER_H

#include "com_control/message/response.h"
#include <cstdint>
#include <vector>

namespace crobot {

class Data_Parser {
public:
    Data_Parser(uint32_t buf_len = 128);

    void parse(uint8_t data);

    Response get_response();

    bool success() const {
        return flag;
    }

private:
    bool flag = false;
    bool FE_flag = false;
    bool FH_flag = false;
    std::vector<uint8_t> buf;
    uint32_t buf_len = 0;
    uint32_t data_len = 0;
};

} // namespace crobot

#endif // COM_CONTROL_DATA_PARSER_H
