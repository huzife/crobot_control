#ifndef COM_CONTROL_CONTROLLER_H
#define COM_CONTROL_CONTROLLER_H

#include "com_control/controller_callbacks.h"
#include "com_control/listener.h"
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"
#include "com_control/message/request.h"
#include "com_control/message/response.h"
#include <functional>
#include <vector>
#include <mutex>

namespace crobot {

class Controller {
private:
    itas109::CSerialPort sp;
    crobot::Listener listener;
    Controller_Callbacks& callbacks;

public:
    Controller(Controller_Callbacks& cbs):
        listener(sp, std::bind(&Controller::process_data, this, std::placeholders::_1, std::placeholders::_2)),
        callbacks(cbs) {}
    ~Controller();
    void init(const char* port_name,
              itas109::BaudRate baudrate,
              itas109::Parity parity,
              itas109::DataBits databits,
              itas109::StopBits stopbits,
              itas109::FlowControl flow_control,
              uint32_t read_buf_size);
    void open();

    // receive
    void process_data(std::vector<uint8_t> data, uint32_t len);

    // send
    void set_speed(const Set_Speed_Req& speed_req);
    void get_speed();
    void get_imu_temperature();
    void get_imu();

private:
    void write(const std::vector<uint8_t>& data);
};

} // namespace crobot

#endif // COM_CONTROL_CONTROLLER_H
