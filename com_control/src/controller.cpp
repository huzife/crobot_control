#include "com_control/controller.h"
#include "com_control/message/response.h"
#include <iostream>

namespace crobot {

Controller::~Controller() {
	if (sp.isOpen()) sp.close();
}

void Controller::init(const char *port_name,
					  itas109::BaudRate baudrate,
					  itas109::Parity parity,
					  itas109::DataBits databits,
					  itas109::StopBits stopbits,
					  itas109::FlowControl flow_control,
					  uint32_t read_buf_size) {
	sp.init(port_name, baudrate, parity, databits, stopbits, flow_control, read_buf_size);
	sp.setReadIntervalTimeout(0);
}

void Controller::open() {
	sp.open();
	sp.connectReadEvent(&listener);
}

void Controller::write(const std::vector<uint8_t> data) {
	sp.writeData(data.data(), data.size());
}

void Controller::processData(std::vector<uint8_t> data, uint32_t len) {
	Response resp(data);
	bool ret = resp.parse();
	if (!ret) return;

	switch (resp.getType()) {
	case MessageType::NONE: break;
	case MessageType::SET_SPEED: callbacks.setSpeedCallback(); break;
	case MessageType::GET_SPEED: callbacks.getSpeedCallback(resp.get_GetSpeedResp()); break;
	}
}

void Controller::setSpeed(const SetSpeedReq &speed_req) {
	std::vector<uint8_t> raw_data(12);
	float_to_hex(speed_req.linear_x, raw_data, 0);
	float_to_hex(speed_req.linear_y, raw_data, 4);
	float_to_hex(speed_req.angular_z, raw_data, 8);
	Request req(raw_data, MessageType::SET_SPEED);
	write(req.getData());
}

void Controller::getSpeed() {
	Request req(MessageType::GET_SPEED);
	write(req.getData());
}

} // namespace crobot
