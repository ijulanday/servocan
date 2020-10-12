#include "servocan.h"

// range: 0 - 360
CAN_message_t positionMessageBuilder(double usrAngle) {
    CAN_message_t msg;
    uint16_t angle = usrAngle * 4096.0 / 90.0;
    uint8_t angHi = angle >> 8;
    uint8_t angLo = angle & 0x00FF;
    msg.id = 0x000;
    msg.buf[0] = 0x96;
    msg.buf[1] = 0x00;  // servo id
    msg.buf[2] = 0x1E;  // address
    msg.buf[3] = 0x02;  // reg length
    msg.buf[4] = angLo;  // data low
    msg.buf[5] = angHi;  // data high
    // check sum (id + address + reg length + data low + data high) & 0xFF
    msg.buf[6] = (msg.buf[1] + msg.buf[2] + msg.buf[3] + msg.buf[4] + msg.buf[5] ) & 0xFF;
    return msg;
}

// +1 rotates servo 360 in the + direction
CAN_message_t turnMessageBuilder(int16_t rotations) {
    CAN_message_t msg;
    int8_t rotHi = rotations >> 8;
    return msg;
}