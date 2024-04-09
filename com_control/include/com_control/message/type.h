#ifndef COM_CONTROL_MESSAGE_TYPE_H
#define COM_CONTROL_MESSAGE_TYPE_H

enum class Message_Type: unsigned char {
    SET_VELOCITY,
    GET_ODOM,
    GET_IMU_TEMPERATURE,
    GET_IMU_DATA,
    GET_ULTRASONIC_RANGE,
    GET_BATTERY_VOLTAGE,
    MESSAGE_TYPE_MAX
};

#endif // COM_CONTROL_MESSAGE_TYPE_H
