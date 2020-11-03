#include "servocan.h"

CAN_message_t servo_message;


// serial printer for CAN messages
void printRaw(CAN_message_t servo_message) {
    for (int i = 0; i < 6; i++) {
            Serial.print(servo_message.buf[i], HEX); Serial.print(" ");
        }
    Serial.println(" ");
}

// generic for write messages
void genericWriteMessage(uint8_t address, uint8_t servoId, uint16_t data, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
    uint8_t lo = data & 0x00FF;
    uint8_t hi = data >> 8;
    servo_message.id = 0x000;         // specify hitec servo
    servo_message.buf[0] = 0x96;      // write message header
    servo_message.buf[1] = servoId;   // servo id
    servo_message.buf[2] = address;      // address
    servo_message.buf[3] = 0x02;      // reg length
    servo_message.buf[4] = lo;        // data low
    servo_message.buf[5] = hi;        // data high
    // check sum (id + address + reg length + data low + data high) & 0xFF
    servo_message.buf[6] = (servo_message.buf[1] + servo_message.buf[2] + servo_message.buf[3] + servo_message.buf[4] + servo_message.buf[5] ) & 0xFF;
    can->write(servo_message);
}

// generic for read messages
void genericReadMessage(uint8_t address, uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
    servo_message.id = 0x000;
    servo_message.buf[0] = 0x96;
    servo_message.buf[1] = servoId;
    servo_message.buf[2] = address;
    servo_message.buf[3] = 0x00;
    // check sum (id + address) & 0xFF
    servo_message.buf[4] = (servoId + address) & 0xFF;
    can->write(servo_message);
}

// generic decoder for can messages (re-arranges buf segments to single int)
uint16_t genericDecoder(CAN_message_t msg) {
    return (uint16_t(msg.buf[5]) << 8) | uint16_t(msg.buf[4]);
}

// moves servo to an angle from 0 to 360 degrees
void REG_POSITION_NEW(uint8_t servoId, double usrAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(0x1E, servoId, uint16_t(round(usrAngle / 90.0 * 4096.0 )), can);
}

// +1 rotates servo 360 in the + direction
void REG_TURN_NEW(uint8_t servoId, int16_t rotations, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(0x24, servoId, rotations, can);
} // to enable you must change run mode to continuous, save the config, then restart the servo

// 0: multi-turn mode, 1: servo mode
void REG_RUN_MODE(uint8_t servoId, uint16_t mode, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(0x44, servoId, mode, can);
}

// saves current reg values for reset
void REG_CONFIG_SAVE(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(0x70, servoId, 0xFFFF, can);
}

// resets the servo
void REG_POWER_CONFIG(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(0x46, servoId, 0x0001, can);
}

// sets position max limit(s)
void REG_POS_MAX(uint8_t servoId, double maxAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(0xB0, servoId, maxAngle, can);
}

// sets position min limit(s)
void REG_POS_MIN(uint8_t servoId, double minAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(0xB2, servoId, minAngle, can);
}

// restore factory default
void REG_FACTORY_DEFAULT(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(0x6E, servoId, 0x0F0F, can);
}

// query for servo turn count
void REG_TURN_COUNT(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(0x18, servoId, can);
}

// query for low value of current position (0 - 65535, 4096 = 90 deg)
void REG_32BITS_POSITION_L(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(0x1A, servoId, can);
}  

// decodes CAN position response integer value between 0 and 360 
uint16_t decodePositionLo() {
    double raw =  double(genericDecoder(servo_message));
    return uint16_t(round(raw * 90.0 / 4096.0));
}

// query for high value of position (65535 - 2^31, 4096 = 90 deg)
void REG_32BITS_POSITION_H(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(0x1C, servoId, can);
} // TODO: write decoder

// queries position register
void REG_POSITION(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(0x0C, servoId, can);
}

// query for current servo velocity
void REG_VOLTAGE(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(0x12, servoId, can);
}

// decodes CAN voltage response
double decodeVoltage() {
    return double(genericDecoder(servo_message)) / 100;
}

// query mcu temperature
void REG_MCU_TEMPER(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(0x14, servoId, can);
}

// decode CAN temperature response 
uint16_t decodeTemp() {
    return genericDecoder(servo_message);
}

// assigns a servo a new ID. not sure how this works for multiple servos on one can line
void REG_ID(uint8_t servoId, uint16_t newId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(0x32, servoId, newId, can);
}   // requires config save and reset