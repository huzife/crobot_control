#ifndef COM_CONTROL_MESSAGE_TYPE_H
#define COM_CONTROL_MESSAGE_TYPE_H

enum class Message_Type: unsigned char {
    NONE,
    SET_SPEED,
    GET_SPEED,
    GET_IMU_TEMPERATURE,
    GET_IMU,
    ERROR = 0xFF
};

#endif // COM_CONTROL_MESSAGE_TYPE_H
