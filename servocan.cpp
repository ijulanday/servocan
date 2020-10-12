#include "servocan.h"

// range: 0 - 360
CAN_message_t genericWriteMessageBuilder(uint8_t address, uint8_t servoId, uint16_t data) {
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

CAN_message_t positionMessageBuilder(uint8_t servoId, double usrAngle) {
    return genericWriteMessageBuilder(0x1E, servoId, uint16_t(usrAngle * 4096.0 / 90.0));
}

// +1 rotates servo 360 in the + direction
CAN_message_t turnMessageBuilder(uint8_t servoId, int16_t rotations) {
    return genericWriteMessageBuilder(0x24, servoId, rotations);
}

// 0: multi-turn mode, 1: servo mode
CAN_message_t runModeMessageBuilder(uint8_t servoId, uint16_t mode) {
    return genericWriteMessageBuilder(0x44, servoId, mode);
}

// saves current reg values for reset
CAN_message_t regConfigSaveBuilder(uint8_t servoId) {
    return genericWriteMessageBuilder(0x70, servoId, 0xFFFF);
}

// resets the servo
CAN_message_t resetMessageBuilder(uint8_t servoId) {
    return genericWriteMessageBuilder(0x46, servoId, 0x0001);
}

