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
CAN_message_t REG_POSITION_NEW(uint32_t canId, bool extended, uint8_t servoId, double usrAngle);
CAN_message_t REG_POSITION_NEW_RAW(uint32_t canId, bool extended,  uint8_t servoId, uint16_t regVal);
CAN_message_t REG_TURN_NEW(uint32_t canId, bool extended,  uint8_t servoId, int16_t rotations);
CAN_message_t REG_RUN_MODE(uint32_t canId, bool extended,  uint8_t servoId, uint16_t mode);
CAN_message_t REG_CONFIG_SAVE(uint32_t canId, bool extended,  uint8_t servoId);
CAN_message_t REG_POWER_CONFIG(uint32_t canId, bool extended,  uint8_t servoId);
CAN_message_t REG_POSITION_MAX_LIMIT(uint32_t canId, bool extended,  uint8_t servoId, double maxAngle);
CAN_message_t REG_POSITION_MIN_LIMIT(uint32_t canId, bool extended,  uint8_t servoId, double minAngle);
CAN_message_t REG_FACTORY_DEFAULT(uint32_t canId, bool extended,  uint8_t servoId);
CAN_message_t REG_VELOCITY_MAX(uint32_t canId, bool extended,  uint8_t servoId, int RPM);
CAN_message_t REG_VELOCITY(uint32_t canId, bool extended, uint8_t servoId);
CAN_message_t REG_TURN_COUNT(uint32_t canId, bool extended, uint8_t servoId);
int16_t decodeTurnCount(CAN_message_t msg);
CAN_message_t REG_32BITS_POSITION_L(uint32_t canId, bool extended, uint8_t servoId);
double decodePositionLo(CAN_message_t msg);
CAN_message_t REG_32BITS_POSITION_H(uint32_t canId, uint8_t servoId, bool extended);
CAN_message_t REG_VOLTAGE(uint32_t canId, uint8_t servoId, bool extended);
double decodeVoltage(CAN_message_t msg);
CAN_message_t REG_MCU_TEMPER(uint32_t canId, uint8_t servoId, bool extended);
uint16_t decodeTemp(CAN_message_t msg);
CAN_message_t REG_ID(uint32_t canId, bool extended, uint8_t servoId, uint16_t newId);
CAN_message_t REG_CAN_MODE(uint32_t canId, bool extended, uint16_t servoId, bool mode);
CAN_message_t REG_CAN_BUS_ID_L(uint32_t currCanId, bool extended, uint16_t newCanIdL, uint16_t servoId);
CAN_message_t REG_CAN_BUS_ID_H(uint32_t currCanId, bool extended, uint16_t newCanIdH, uint16_t servoId);
CAN_message_t write_REG_USER_1(uint32_t canId, bool extended, uint16_t servoId, uint16_t data);
CAN_message_t write_REG_USER_2(uint32_t canId, bool extended, uint16_t servoId, uint16_t data);
CAN_message_t write_REG_ECHO(uint32_t canId, bool extended, uint16_t servoId, uint16_t data);
CAN_message_t read_REG_USER_1(uint32_t canId, bool extended, uint16_t servoId);
CAN_message_t read_REG_USER_2(uint32_t canId, bool extended, uint16_t servoId);
CAN_message_t read_REG_ECHO(uint32_t canId, bool extended, uint16_t servoId);
CAN_message_t REG_STREAM_TIME(uint32_t canId, bool extended, uint16_t servoId, uint16_t periodMs);
CAN_message_t REG_STREAM_MODE(uint32_t canId, bool extended, uint16_t servoId, bool mode);
// CAN_message_t REG_STREAM_ADDR_0(uint32_t canId, bool extended, uint16_t servoId, uint8_t addr0, uint8_t addr1);
// CAN_message_t REG_STREAM_ADDR_1(uint32_t canId, bool extended, uint16_t servoId, uint8_t addr0, uint8_t addr1);
// CAN_message_t REG_STREAM_ADDR_2(uint32_t canId, bool extended, uint16_t servoId, uint8_t addr0, uint8_t addr1);
// CAN_message_t REG_STREAM_ADDR_3(uint32_t canId, bool extended, uint16_t servoId, uint8_t addr0, uint8_t addr1);

#endif