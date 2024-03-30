#include "com_control/controller.h"
#include "com_control/data_parser.h"
#include "com_control/message/response.h"
#include <iostream>

namespace crobot {
Controller::Controller(Controller_Callbacks& cbs)
    : listener(sp, std::bind(&Controller::receive_data, this,
                             std::placeholders::_1,
                             std::placeholders::_2)),
      callbacks(cbs),
      data_queue(1024) {}

Controller::~Controller() {
    if (sp.isOpen()) {
        std::cout << "close serial" << std::endl;
        sp.close();
    }

    thread_end = true;
    process_thread.join();
}

void Controller::init(const char* port_name,
                      itas109::BaudRate baudrate,
                      itas109::Parity parity,
                      itas109::DataBits databits,
                      itas109::StopBits stopbits,
                      itas109::FlowControl flow_control) {
    sp.init(port_name, baudrate, parity, databits, stopbits, flow_control, Listener::READ_BUF_SIZE);
    sp.setReadIntervalTimeout(0);
}

void Controller::open() {
    if (sp.open())
        std::cout << "Serial opened: " << sp.getPortName() << std::endl;
    else
        std::cout << "Failed to open serial: " << sp.getLastErrorMsg() << std::endl;

    // start read
    sp.connectReadEvent(&listener);
    process_thread = std::thread{std::bind(&Controller::process_data, this)};
}

void Controller::write(const Request& req) {
    auto data = req.data();
    if (sp.writeData(data.data(), data.size()) == -1)
        std::cout << "Failed to send data: " << sp.getLastErrorMsg() << std::endl;
}

void Controller::receive_data(uint8_t* data, uint32_t len) {
    for (int i = 0; i < len; ++i) {
        data_queue.push(data[i]);
    }
}

void Controller::process_data() {
    Data_Parser parser;
    while (!thread_end) {
        uint8_t data;
        if (!data_queue.pop(data) || !parser.parse(data))
            continue;

        auto resp = parser.get_response();
        switch (resp.type()) {
        case Message_Type::SET_VELOCITY:
            callbacks.set_velocity_callback();
            break;
        case Message_Type::GET_ODOM:
            callbacks.get_odom_callback(resp.get_odom_resp());
            break;
        case Message_Type::GET_IMU_TEMPERATURE:
            callbacks.get_imu_temperature_callback(resp.get_imu_temperature_resp());
            break;
        case Message_Type::GET_IMU_DATA:
            callbacks.get_imu_data_callback(resp.get_imu_resp());
            break;
        case Message_Type::GET_ULTRASONIC_RANGE:
            callbacks.get_ultrasonic_range_callback(resp.get_ultrasonic_range_resp());
            break;
        default:
            break;
        }
    }
}

void Controller::send_request(const Request& req) {
    write(req);
}

} // namespace crobot
