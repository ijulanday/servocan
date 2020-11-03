#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

#ifndef SERVOCAN_H
#define SERVOCAN_H
void printRaw(CAN_message_t msg);
void REG_POSITION_NEW(uint8_t servoId, double usrAngle);
void REG_TURN_NEW(uint8_t servoId, int16_t rotations);
void REG_RUN_MODE(uint8_t servoId, uint16_t mode);
void REG_CONFIG_SAVE(uint8_t servoId);
void REG_POWER_CONFIG(uint8_t servoId);
void REG_POS_MAX(uint8_t servoId, double maxAngle);
void REG_POS_MIN(uint8_t servoId, double minAngle);
void REG_FACTORY_DEFAULT(uint8_t servoId);
void REG_TURN_COUNT(uint8_t servoId);
void REG_32BITS_POSITION_L(uint8_t servoId);
uint16_t decodePositionLo(CAN_message_t msg);
void REG_32BITS_POSITION_H(uint8_t servoId);
void REG_POSITION(uint8_t servoId);
void REG_VOLTAGE(uint8_t servoId);
double decodeVoltage(CAN_message_t msg);
void REG_MCU_TEMPER(uint8_t servoId);
uint16_t decodeTemp(CAN_message_t msg);
void REG_ID(uint8_t servoId, uint16_t newId);

extern CAN_message_t servo_message;

#endif