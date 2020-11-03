#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

#ifndef SERVOCAN_H
#define SERVOCAN_H
void printRaw(CAN_message_t msg);
void REG_POSITION_NEW(uint8_t servoId, double usrAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_TURN_NEW(uint8_t servoId, int16_t rotations, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_RUN_MODE(uint8_t servoId, uint16_t mode, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_CONFIG_SAVE(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_POWER_CONFIG(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_POS_MAX(uint8_t servoId, double maxAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_POS_MIN(uint8_t servoId, double minAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_FACTORY_DEFAULT(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_TURN_COUNT(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_32BITS_POSITION_L(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
uint16_t decodePositionLo(CAN_message_t msg);
void REG_32BITS_POSITION_H(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_POSITION(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_VOLTAGE(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
double decodeVoltage(CAN_message_t msg, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_MCU_TEMPER(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
uint16_t decodeTemp(CAN_message_t msg);
void REG_ID(uint8_t servoId, uint16_t newId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);

extern CAN_message_t servo_message;

#endif