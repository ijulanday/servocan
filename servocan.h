#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef SERVOCAN_H
#define SERVOCAN_H
CAN_message_t positionMessage(uint8_t servoId, double usrAngle);
CAN_message_t turnMessage(uint8_t servoId, int16_t rotations);
CAN_message_t runModeMessage(uint8_t servoId, uint16_t mode);
CAN_message_t regConfigSave(uint8_t servoId);
CAN_message_t resetMessage(uint8_t servoId);
CAN_message_t setPositionMax(uint8_t servoId, double maxAngle);
CAN_message_t setPositionMin(uint8_t servoId, double minAngle);
CAN_message_t restoreFactoryDefault(uint8_t servoId);
#endif