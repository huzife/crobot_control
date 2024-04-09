#ifndef COM_CONTROL_CONTROLLER_H
#define COM_CONTROL_CONTROLLER_H

#include "com_control/controller_callbacks.h"
#include "com_control/listener.h"
#include "com_control/swsr_queue.h"
#include "com_control/message/request.h"
#include "CSerialPort/SerialPort.h"
#include <thread>

namespace crobot {

class Controller {
private:
    itas109::CSerialPort sp_;
    crobot::Listener listener_;
    Controller_Callbacks& callbacks_;
    SWSR_Queue<uint8_t> data_queue_;

    bool thread_end_ = false;
    std::thread base_com_thread_;

public:
    Controller(Controller_Callbacks& cbs);
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
    void process_response(const Response& resp);
    void base_com_func();

    // send
    void send_request(const Request& req);

private:
    void write(const Request& req);
};

} // namespace crobot

#endif // COM_CONTROL_CONTROLLER_H
