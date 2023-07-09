#include "com_control/listener.h"

namespace crobot {

void Listener::onReadEvent(const char *port_name, uint32_t read_buf_len) {
	if (read_buf_len == 0) return;

	char *data = new char[read_buf_len + 1];
	if (data == nullptr) return;

	// 读数据
	int len = sp.readData(data, read_buf_len);
	if (len > 0) {
		// TODO: 通过回调处理数据
		read_callback(std::vector<uint8_t>(data, data + len), len);
	}

	delete data;
}

} // namespace crobot