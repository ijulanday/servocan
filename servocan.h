#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef SERVOCAN_H
#define SERVOCAN_H
CAN_message_t positionMessageBuilder(uint8_t servoId, double usrAngle);
CAN_message_t turnMessageBuilder(uint8_t servoId, int16_t rotations);
CAN_message_t runModeMessageBuilder(uint8_t servoId, uint16_t mode);
CAN_message_t regConfigSaveBuilder(uint8_t servoId);
CAN_message_t resetMessageBuilder(uint8_t servoId);
#endif