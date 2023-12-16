#include "com_control/listener.h"
#include <iostream>

namespace crobot {

void Listener::onReadEvent(const char* port_name, uint32_t read_buf_len) {
    if (read_buf_len == 0) return;

    // 读数据
    int len = sp.readData(read_data, read_buf_len);
    if (len > 0)
        read_callback(read_data, len);
}

} // namespace crobot
