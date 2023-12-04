#ifndef _COMCONTROL_LISTENER_H
#define _COMCONTROL_LISTENER_H

#include "CSerialPort/SerialPort.h"
#include <functional>
#include <vector>

namespace crobot {

using Listener_Read_CB = std::function<void(std::vector<uint8_t>, uint32_t)>;

class Listener: public itas109::CSerialPortListener {
private:
    itas109::CSerialPort& sp;
    Listener_Read_CB read_callback;

public:
    Listener(itas109::CSerialPort& sp, Listener_Read_CB cb): sp(sp), read_callback(cb) {}

    void onReadEvent(const char* port_name, uint32_t read_buf_len) override;
};

} // namespace crobot

#endif // _COMCONTROL_LISTENER_H
