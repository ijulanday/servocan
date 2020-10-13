#include "servocan.h"

// generic for write messages
CAN_message_t genericWriteMessage(uint8_t address, uint8_t servoId, uint16_t data) {
    CAN_message_t msg;
    uint8_t lo = data & 0x00FF;
    uint8_t hi = data >> 8;
    msg.id = 0x000;         // specify hitec servo
    msg.buf[0] = 0x96;      // write message header
    msg.buf[1] = servoId;   // servo id
    msg.buf[2] = address;      // address
    msg.buf[3] = 0x02;      // reg length
    msg.buf[4] = lo;        // data low
    msg.buf[5] = hi;        // data high
    // check sum (id + address + reg length + data low + data high) & 0xFF
    msg.buf[6] = (msg.buf[1] + msg.buf[2] + msg.buf[3] + msg.buf[4] + msg.buf[5] ) & 0xFF;
    return msg;
}

// generic for read messages
CAN_message_t genericReadMessage(uint8_t address, uint8_t servoId) {
    CAN_message_t msg;
    msg.id = 0x000;
    msg.buf[0] = 0x96;
    msg.buf[1] = servoId;
    msg.buf[2] = address;
    msg.buf[3] = 0x00;
    // check sum (id + address) & 0xFF
    msg.buf[4] = (servoId + address) & 0xFF;
    return msg;
}

// generic decoder for can messages (re-arranges buf segments to single int)
uint16_t genericDecoder(CAN_message_t msg) {
    return (uint16_t(msg.buf[5]) << 8) | uint16_t(msg.buf[4]);
}

// moves servo to an angle from 0 to 360 degrees
CAN_message_t REG_POSITION_NEW(uint8_t servoId, double usrAngle) {
    return genericWriteMessage(0x1E, servoId, uint16_t(round(usrAngle / 90.0 * 4096.0 )));
}

// +1 rotates servo 360 in the + direction
CAN_message_t REG_TURN_NEW(uint8_t servoId, int16_t rotations) {
    return genericWriteMessage(0x24, servoId, rotations);
} // to enable you must change run mode to continuous, save the config, then restart the servo

// 0: multi-turn mode, 1: servo mode
CAN_message_t REG_RUN_MODE(uint8_t servoId, uint16_t mode) {
    return genericWriteMessage(0x44, servoId, mode);
}

// saves current reg values for reset
CAN_message_t REG_CONFIG_SAVE(uint8_t servoId) {
    return genericWriteMessage(0x70, servoId, 0xFFFF);
}

// resets the servo
CAN_message_t REG_POWER_CONFIG(uint8_t servoId) {
    return genericWriteMessage(0x46, servoId, 0x0001);
}

// sets position max limit(s)
CAN_message_t REG_POS_MAX(uint8_t servoId, double maxAngle) {
    return genericWriteMessage(0xB0, servoId, maxAngle);
}

// sets position min limit(s)
CAN_message_t REG_POS_MIN(uint8_t servoId, double minAngle) {
    return genericWriteMessage(0xB2, servoId, minAngle);
}

// restore factory default
CAN_message_t REG_FACTORY_DEFAULT(uint8_t servoId) {
    return genericWriteMessage(0x6E, servoId, 0x0F0F);
}

// query for servo turn count
CAN_message_t REG_TURN_COUNT(uint8_t servoId) {
    return genericReadMessage(0x18, servoId);
}

// query for low value of current position (0 - 65535, 4096 = 90 deg)
CAN_message_t REG_32BITS_POSITION_L(uint8_t servoId) {
    return genericReadMessage(0x1A, servoId);
}  

// decodes CAN position response integer value between 0 and 360 
uint16_t decodePositionLo(CAN_message_t msg) {
    double raw =  double(genericDecoder(msg));
    return uint16_t(round(raw * 90.0 / 4096.0));
}

// query for high value of position (65535 - 2^31, 4096 = 90 deg)
CAN_message_t REG_32BITS_POSITION_H(uint8_t servoId) {
    return genericReadMessage(0x1C, servoId);
} // TODO: write decoder

// queries position register
CAN_message_t REG_POSITION(uint8_t servoId) {
    return genericReadMessage(0x0C, servoId);
}

// query for current servo velocity
CAN_message_t REG_VOLTAGE(uint8_t servoId) {
    return genericReadMessage(0x12, servoId);
}

// decodes CAN voltage response
double decodeVoltage(CAN_message_t msg) {
    return double(genericDecoder(msg)) / 100;
}

// query mcu temperature
CAN_message_t REG_MCU_TEMPER(uint8_t servoId) {
    return genericReadMessage(0x14, servoId);
}

// decode CAN temperature response 
uint16_t decodeTemp(CAN_message_t msg) {
    return genericDecoder(msg);
} 