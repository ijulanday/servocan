#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

#ifndef SERVOCAN_H
#define SERVOCAN_H
void printRaw(CAN_message_t msg);
CAN_message_t REG_POSITION_NEW(uint8_t servoId, double usrAngle);
CAN_message_t REG_TURN_NEW(uint8_t servoId, int16_t rotations);
CAN_message_t REG_RUN_MODE(uint8_t servoId, uint16_t mode);
CAN_message_t REG_CONFIG_SAVE(uint8_t servoId);
CAN_message_t REG_POWER_CONFIG(uint8_t servoId);
CAN_message_t REG_POS_MAX(uint8_t servoId, double maxAngle);
CAN_message_t REG_POS_MIN(uint8_t servoId, double minAngle);
CAN_message_t REG_FACTORY_DEFAULT(uint8_t servoId);
CAN_message_t REG_TURN_COUNT(uint8_t servoId);
CAN_message_t REG_32BITS_POSITION_L(uint8_t servoId);
uint16_t decodePositionLo(CAN_message_t msg);
CAN_message_t REG_32BITS_POSITION_H(uint8_t servoId);
CAN_message_t REG_POSITION(uint8_t servoId);
CAN_message_t REG_VOLTAGE(uint8_t servoId);
double decodeVoltage(CAN_message_t msg);
CAN_message_t REG_MCU_TEMPER(uint8_t servoId);
uint16_t decodeTemp(CAN_message_t msg);
CAN_message_t REG_ID(uint8_t servoId, uint16_t newId);
#endif