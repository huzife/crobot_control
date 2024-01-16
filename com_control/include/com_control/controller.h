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
    itas109::CSerialPort sp;
    crobot::Listener listener;
    Controller_Callbacks& callbacks;
    SWSR_Queue<uint8_t> data_queue;

    bool thread_end = false;
    std::thread process_thread;

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
    void process_data();

    // send
    void send_request(const Request& req);

private:
    void write(const Request& req);
};

} // namespace crobot

#endif // COM_CONTROL_CONTROLLER_H
