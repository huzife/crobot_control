#ifndef CROBOT_MESSAGE_TYPE_H
#define CROBOT_MESSAGE_TYPE_H

enum class Message_Type: unsigned char {
    SET_PID_INTERVAL,
    SET_COUNT_PER_REV,
    SET_CORRECTION_FACTOR,
    SET_VELOCITY,
    GET_ODOMETRY,
    RESET_ODOMETRY,
    GET_IMU_TEMPERATURE,
    GET_IMU_DATA,
    GET_ULTRASONIC_RANGE,
    GET_BATTERY_VOLTAGE,
    MESSAGE_TYPE_MAX
};

#endif // CROBOT_MESSAGE_TYPE_H
