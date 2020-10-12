#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef SERVOCAN_H
#define SERVOCAN_H
CAN_message_t positionMessageBuilder(double angle);
CAN_message_t turnMessageBuilder(int16_t rotations);
#endif