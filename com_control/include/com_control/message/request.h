#ifndef COM_CONTROL_MESSAGE_REQUEST_H
#define COM_CONTROL_MESSAGE_REQUEST_H

#include <cstdint>
#include <vector>

namespace crobot {

class Request {
public:
    Request() = default;
    virtual ~Request() = default;

    virtual std::vector<uint8_t> data() const = 0;
};

class Set_Speed_Req: public Request {
public:
    Set_Speed_Req(float linear_x, float angular_z);
    ~Set_Speed_Req() = default;

    std::vector<uint8_t> data() const override;

private:
    float linear_x_;
    float angular_z_;
};

class Get_Speed_Req: public Request {
public:
    std::vector<uint8_t> data() const override;
};

class Get_IMU_Temperature_Req: public Request {
public:
    std::vector<uint8_t> data() const override;
};

class Get_IMU_Data_Req: public Request {
public:
    std::vector<uint8_t> data() const override;
};

} // namespace crobot

#endif // COM_CONTROL_MESSAGE_REQUEST_H
