#include "servocan.h"

// generic  for write messages
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

// moves servo to an angle from 0 to 360 degrees
CAN_message_t positionMessage(uint8_t servoId, double usrAngle) {
    return genericWriteMessage(0x1E, servoId, uint16_t(usrAngle * 4096.0 / 90.0));
}

// +1 rotates servo 360 in the + direction
CAN_message_t turnMessage(uint8_t servoId, int16_t rotations) {
    return genericWriteMessage(0x24, servoId, rotations);
} // to enable you must change run mode to continuous, save the config, then restart the servo

// 0: multi-turn mode, 1: servo mode
CAN_message_t runModeMessage(uint8_t servoId, uint16_t mode) {
    return genericWriteMessage(0x44, servoId, mode);
}

// saves current reg values for reset
CAN_message_t regConfigSave(uint8_t servoId) {
    return genericWriteMessage(0x70, servoId, 0xFFFF);
}

// resets the servo
CAN_message_t resetMessage(uint8_t servoId) {
    return genericWriteMessage(0x46, servoId, 0x0001);
}

// sets position max limit(s)
CAN_message_t setPositionMax(uint8_t servoId, double maxAngle) {
    return genericWriteMessage(0xB0, servoId, maxAngle);
}

// sets position min limit(s)
CAN_message_t setPositionMin(uint8_t servoId, double minAngle) {
    return genericWriteMessage(0xB2, servoId, minAngle);
}

// restore factory default
CAN_message_t restoreFactoryDefault(uint8_t servoId) {
    return genericWriteMessage(0x6E, servoId, 0x0F0F);
}
