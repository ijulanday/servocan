#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>

/**
 * a note on sending angle data to the servo:
 *      
 *      data = ANGLE_CONVERSION * degrees + 8192
 * 
 * 4096 IS -90 DEGREES, NOT +90 under this schema when using degrees (datasheet defines 0 to 16383 as 0 to 360 deg)
 * 
*/

#define ANGLE_CONVERSION 45.5111111 // 4096 units / 90 degrees

#ifndef SERVOCAN_H
#define SERVOCAN_H

void printRaw(CAN_message_t servo_message);
void REG_POSITION_NEW(uint32_t canId, bool extended, uint8_t servoId, double usrAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_POSITION_NEW_RAW(uint32_t canId, bool extended,  uint8_t servoId, uint16_t regVal, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_TURN_NEW(uint32_t canId, bool extended,  uint8_t servoId, int16_t rotations, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_RUN_MODE(uint32_t canId, bool extended,  uint8_t servoId, uint16_t mode, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_CONFIG_SAVE(uint32_t canId, bool extended,  uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_POWER_CONFIG(uint32_t canId, bool extended,  uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_POSITION_MAX_LIMIT(uint32_t canId, bool extended,  uint8_t servoId, double maxAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_POSITION_MIN_LIMIT(uint32_t canId, bool extended,  uint8_t servoId, double minAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_FACTORY_DEFAULT(uint32_t canId, bool extended,  uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_VELOCITY_MAX(uint32_t canId, bool extended,  uint8_t servoId, int RPM, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_TURN_COUNT(uint32_t canId, uint8_t servoId, bool extended,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
int16_t decodeTurnCount(CAN_message_t msg);
void REG_32BITS_POSITION_L(uint32_t canId, uint8_t servoId, bool extended,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
double decodePositionLo(CAN_message_t msg);
void REG_32BITS_POSITION_H(uint32_t canId, uint8_t servoId, bool extended,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_VOLTAGE(uint32_t canId, uint8_t servoId, bool extended,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
double decodeVoltage(CAN_message_t msg);
void REG_MCU_TEMPER(uint32_t canId, uint8_t servoId, bool extended,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
uint16_t decodeTemp(CAN_message_t msg);
void REG_ID(uint32_t canId, bool extended, uint8_t servoId, uint16_t newId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_CAN_MODE(uint32_t canId, uint16_t servoId, bool mode, bool extended, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_CAN_BUS_ID_H(uint32_t currCanId, uint16_t newCanIdH, uint16_t servoId, bool extended, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
void REG_CAN_BUS_ID_L(uint32_t currCanId, uint16_t newCanIdL, uint16_t servoId, bool extended, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can);
extern CAN_message_t servo_message;

#endif