#ifndef COM_CONTROL_CONTROLLER_H
#define COM_CONTROL_CONTROLLER_H

#include "com_control/controller_callbacks.h"
#include "com_control/data_parser.h"
#include "com_control/listener.h"
#include "com_control/message/request.h"
#include "com_control/message/response.h"
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace crobot {

class Controller {
private:
    itas109::CSerialPort sp;
    crobot::Listener listener;
    Controller_Callbacks& callbacks;
    std::queue<uint8_t> data_queue;
    std::mutex queue_mtx;

public:
    Controller(Controller_Callbacks& cbs):
        listener(sp, std::bind(&Controller::receive_data,
                               this,
                               std::placeholders::_1,
                               std::placeholders::_2)),
        callbacks(cbs) {}
    ~Controller();
    void init(const char* port_name,
              itas109::BaudRate baudrate,
              itas109::Parity parity,
              itas109::DataBits databits,
              itas109::StopBits stopbits,
              itas109::FlowControl flow_control);
    void open();

    // receive
    void receive_data(uint8_t* data, uint32_t len);
    void process_data();

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
