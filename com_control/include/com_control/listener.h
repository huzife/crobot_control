#ifndef COM_CONTROL_LISTENER_H
#define COM_CONTROL_LISTENER_H

#include "CSerialPort/SerialPort.h"
#include <functional>
#include <vector>

namespace crobot {


class Listener: public itas109::CSerialPortListener {
public:
    using Listener_Read_Callback = std::function<void(uint8_t*, uint32_t)>;

    static constexpr uint32_t READ_BUF_SIZE = 1024;

private:
    itas109::CSerialPort& sp;
    Listener_Read_Callback read_callback;
    uint8_t read_data[READ_BUF_SIZE];

public:
    Listener(itas109::CSerialPort& sp, Listener_Read_Callback cb): sp(sp), read_callback(cb) {}

    void onReadEvent(const char* port_name, uint32_t read_buf_len) override;
};

} // namespace crobot

#endif // COM_CONTROL_LISTENER_H
