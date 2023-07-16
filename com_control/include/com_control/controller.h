#ifndef _COMCONTROL_CONTROLLER_H
#define _COMCONTROL_CONTROLLER_H

#include "com_control/controller_callbacks.h"
#include "com_control/listener.h"
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"
#include "com_control/message/request.h"
#include "com_control/message/response.h"
#include <functional>
#include <vector>

namespace crobot {

class Controller {
private:
	itas109::CSerialPort sp;
	crobot::Listener listener;
	ControllerCallbacks &callbacks;

public:
	Controller(ControllerCallbacks &cbs)
		: listener(sp, std::bind(&Controller::processData, this, std::placeholders::_1, std::placeholders::_2)),
		  callbacks(cbs) {}
	~Controller();
	void init(const char *port_name,
			  itas109::BaudRate baudrate,
			  itas109::Parity parity,
			  itas109::DataBits databits,
			  itas109::StopBits stopbits,
			  itas109::FlowControl flow_control,
			  uint32_t read_buf_size);
	void open();

	// receive
	void processData(std::vector<uint8_t> data, uint32_t len);

	// send
	void setSpeed(const SetSpeedReq &speed_req);
	void getSpeed();
	void getTempAndHum();

private:
	void write(const std::vector<uint8_t> data);
};

} // namespace crobot

#endif // _COMCONTROL_CONTROLLER_H