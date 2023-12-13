#include "com_control/controller.h"
#include "com_control/message/response.h"
#include <iomanip>
#include <iostream>

namespace crobot {

Controller::~Controller() {
    if (sp.isOpen())
        sp.close();
}

void Controller::init(const char* port_name,
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
    if (sp.open())
        std::cout << "Serial opened: " << sp.getPortName() << std::endl;
    else
        std::cout << "Failed to open serial: " << sp.getLastErrorMsg() << std::endl;
    sp.connectReadEvent(&listener);
}

void Controller::write(const std::vector<uint8_t>& data) {
    if (sp.writeData(data.data(), data.size()) == -1)
        std::cout << "Failed to send data: " << sp.getLastErrorMsg() << std::endl;
}

void Controller::process_data(std::vector<uint8_t> data, uint32_t len) {
    Response resp(data);
    bool ret = resp.parse();
    if (!ret) {
        std::cout << "Error response, len = " << len << ", hex data = ";
        for (int i : data) {
            std::cout << std::hex << std::uppercase << std::setw(2)
                      << std::setfill('0') << i << ' ';
        }
        std::cout << std::endl;
        return;
    }

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
