#include "com_control/controller.h"
#include "com_control/message/response.h"
#include <iomanip>
#include <iostream>

namespace crobot {

Controller::~Controller() {
    if (sp.isOpen()) {
        std::cout << "close serial" << std::endl;
        sp.close();
    }
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
    std::thread{std::bind(&Controller::process_data, this)}.detach();
}

void Controller::write(const std::vector<uint8_t>& data) {
    if (sp.writeData(data.data(), data.size()) == -1)
        std::cout << "Failed to send data: " << sp.getLastErrorMsg() << std::endl;
}

void Controller::receive_data(uint8_t* data, uint32_t len) {
    std::lock_guard<std::mutex> queue_lock{queue_mtx};
    for (int i = 0; i < len; ++i) {
        data_queue.push(data[i]);
    }
}

void Controller::process_data() {
    Data_Parser parser;
    while (true) {
        while (!parser.success()) {
            std::lock_guard<std::mutex> queue_lock{queue_mtx};
            // std::cout << "start parse" << std::endl;
            if (!data_queue.empty()) {
                parser.parse(data_queue.front());
                data_queue.pop();
            }
        }

        auto resp = parser.get_response();
        switch (resp.get_type()) {
        case Message_Type::NONE:
            break;
        case Message_Type::SET_SPEED:
            callbacks.set_speed_callback();
            break;
        case Message_Type::GET_SPEED:
            callbacks.get_speed_callback(resp.get_speed_resp());
            break;
        case Message_Type::GET_IMU_TEMPERATURE:
            callbacks.get_imu_temperature_callback(resp.get_imu_temperature_resp());
            break;
        case Message_Type::GET_IMU:
            callbacks.get_imu_callback(resp.get_imu_resp());
            break;
        case Message_Type::ERROR:
            break;
        }
    }
}

void Controller::set_speed(const Set_Speed_Req& speed_req) {
    std::vector<uint8_t> raw_data(12);
    float_to_hex(speed_req.linear_x, raw_data, 0);
    float_to_hex(speed_req.linear_y, raw_data, 4);
    float_to_hex(speed_req.angular_z, raw_data, 8);
    Request req(raw_data, Message_Type::SET_SPEED);
    write(req.get_data());
}

void Controller::get_speed() {
    Request req(Message_Type::GET_SPEED);
    write(req.get_data());
}

void Controller::get_imu_temperature() {
    Request req(Message_Type::GET_IMU_TEMPERATURE);
    write(req.get_data());
}

void Controller::get_imu() {
    Request req(Message_Type::GET_IMU);
    write(req.get_data());
}

} // namespace crobot
