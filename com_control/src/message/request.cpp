#include "com_control/message/request.h"
#include "com_control/message/type.h"
#include "com_control/message/utils.h"

using namespace std;

namespace crobot {

Set_Speed_Req::Set_Speed_Req(float linear_x, float angular_z)
    : linear_x_(linear_x),
      angular_z_(angular_z) {}

vector<uint8_t> Set_Speed_Req::data() const {
    vector<uint8_t> ret(12);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x09;
    ret[3] = static_cast<uint8_t>(Message_Type::SET_SPEED);
    float_to_hex(linear_x_, ret, 4);
    float_to_hex(angular_z_, ret, 8);

    return ret;
}

vector<uint8_t> Get_Speed_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x01;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_SPEED);

    return ret;
}

vector<uint8_t> Get_IMU_Temperature_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x01;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_IMU_TEMPERATURE);

    return ret;
}

vector<uint8_t> Get_IMU_Data_Req::data() const {
    vector<uint8_t> ret(4);
    ret[0] = 0xFE;
    ret[1] = 0xEF;
    ret[2] = 0x01;
    ret[3] = static_cast<uint8_t>(Message_Type::GET_IMU_DATA);

    return ret;
}

} // namespace crobot
