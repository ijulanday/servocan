#include "servocan.hh"

CAN_message_t servo_message;


// serial printer for CAN messages
void printRaw(CAN_message_t servo_message) {
    for (int i = 0; i < 6; i++) {
            Serial.print(servo_message.buf[i], HEX); Serial.print(" ");
        }
    Serial.println(" ");
}

// generic for write messages
void genericWriteMessage(uint32_t canId, bool extended, uint8_t address, uint8_t servoId, uint16_t data, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
    uint8_t lo = data & 0x00FF;
    uint8_t hi = data >> 8;
    servo_message.id = canId;         // specify hitec servo CAN ID (0x000 if default)
    servo_message.flags.extended = extended;
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
void genericReadMessage(uint32_t canId, bool extended, uint8_t address, uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
    servo_message.id = canId;
    servo_message.flags.extended = extended;
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

// moves servo to an angle from -180 to 180 degrees
void REG_POSITION_NEW(uint32_t canId, bool extended, uint8_t servoId, double usrAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended, 0x1E, servoId, uint16_t(usrAngle*ANGLE_CONVERSION+8192), can);
}

// moves servo to a new angle using raw register values (0 to 360 is 0 - 16383)
void REG_POSITION_NEW_RAW(uint32_t canId, bool extended,  uint8_t servoId, uint16_t regVal, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended,  0x1E, servoId, regVal, can);
}

// +1 rotates servo 360 in the + direction
void REG_TURN_NEW(uint32_t canId, bool extended,  uint8_t servoId, int16_t rotations, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended,  0x24, servoId, rotations, can);
} // to enable you must change run mode to continuous, save the config, then restart the servo

// 0: multi-turn mode, 1: servo mode
void REG_RUN_MODE(uint32_t canId, bool extended,  uint8_t servoId, uint16_t mode, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended,  0x44, servoId, mode, can);
}

// saves current reg values for reset
void REG_CONFIG_SAVE(uint32_t canId, bool extended,  uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended,  0x70, servoId, 0xFFFF, can);
}

// resets servo, applies config changes
void REG_POWER_CONFIG(uint32_t canId, bool extended,  uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended,  0x46, servoId, 0x0001, can);
}

// sets position max limit(s)
void REG_POSITION_MAX_LIMIT(uint32_t canId, bool extended,  uint8_t servoId, double maxAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended,  0xB0, servoId, maxAngle, can);
}

// sets position min limit(s)
void REG_POSITION_MIN_LIMIT(uint32_t canId, bool extended,  uint8_t servoId, double minAngle, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended,  0xB2, servoId, minAngle, can);
}

// restore factory default
void REG_FACTORY_DEFAULT(uint32_t canId, bool extended,  uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended,  0x6E, servoId, 0x0F0F, can);
}

// change velocity max speed
void REG_VELOCITY_MAX(uint32_t canId, bool extended,  uint8_t servoId, int RPM, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
    double speed = RPM * 27.33;
    genericWriteMessage(canId, extended,  0x54, servoId, speed, can);
}

// query for servo turn count
void REG_TURN_COUNT(uint32_t canId, uint8_t servoId, bool extended,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(canId, extended, 0x18, servoId, can);
}

// decodes reported turn count extended,
int16_t decodeTurnCount(CAN_message_t msg) {
      return (int16_t)genericDecoder(msg);
}

// query for low value of current position (0 - 65535, 4096 = 90 deg)
void REG_32BITS_POSITION_L(uint32_t canId, bool extended, uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(canId, extended, 0x1A, servoId, can);
}  

// decodes CAN position response integer value between -180 and 180 
double decodePositionLo(CAN_message_t msg) {
    double raw =  double(genericDecoder(msg));
    return ((raw - 8192) / ANGLE_CONVERSION);
}

// query for high value of position (65535 - 2^31, 4096 = 90 deg)
void REG_32BITS_POSITION_H(uint32_t canId, uint8_t servoId, bool extended,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(canId, extended, 0x1C, servoId, can);
} // TODO: write decoder

// // this is BROKEN and I DON'T KNOW WHY (problem exists on hitec prog too)
// void REG_POSITION(uint8_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
//       genericReadMessage(0x0C, servoId, can);
// }

// query for current servo velocity
void REG_VOLTAGE(uint32_t canId, uint8_t servoId, bool extended,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(canId, extended, 0x12, servoId, can);
}

// decodes CAN voltage response
double decodeVoltage(CAN_message_t msg) {
    return double(genericDecoder(msg)) / 100;
}

// query mcu temperature
void REG_MCU_TEMPER(uint32_t canId, uint8_t servoId, bool extended,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(canId, extended, 0x14, servoId, can);
}

// decode CAN temperature response 
uint16_t decodeTemp(CAN_message_t msg) {
    return genericDecoder(msg);
}

// assigns a servo a new ID. not sure how this works for multiple servos on one can line
void REG_ID(uint32_t canId, bool extended, uint8_t servoId, uint16_t newId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended, 0x32, servoId, newId, can);
}   // requires config save and reset

// sets CAN mode (0 for 2.0, 1 for 2.0B)
void REG_CAN_MODE(uint32_t canId, bool extended, uint16_t servoId, bool mode, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended, 0x6A, servoId, mode, can);
}

// sets high word for CAN bus ID (requires power cycle)
void REG_CAN_BUS_ID_H(uint32_t currCanId, bool extended, uint16_t newCanIdH, uint16_t servoId,  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      if (newCanIdH > 8191) {
            Serial.println("newCanIdH exceeds acceptable value");
            return;
      }
      
      genericWriteMessage(currCanId, extended, 0x3C, servoId, newCanIdH, can);
}

// sets low word for CAN bus ID (requires power cycle)
void REG_CAN_BUS_ID_L(uint32_t currCanId, bool extended, uint16_t newCanIdL, uint16_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      if (newCanIdL > 65535) {
            Serial.println("newCanIdH exceeds acceptable value");
            return;
      }

      genericWriteMessage(currCanId, extended, 0x3E, servoId, newCanIdL, can);
}

// write to user memory 1
void write_REG_USER_1(uint32_t canId, bool extended, uint16_t servoId, uint16_t data, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericWriteMessage(canId, extended, 0xCC, servoId, data, can);
}

// write to user memory 2
void write_REG_USER_2(uint32_t canId, bool extended, uint16_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(canId, extended, 0xCE, servoId, can);
}

// read user memory 1
void read_REG_USER_1(uint32_t canId, bool extended, uint16_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(canId, extended, 0xCC, servoId, can);
}

// read user memory 2
void read_REG_USER_2(uint32_t canId, bool extended, uint16_t servoId, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can) {
      genericReadMessage(canId, extended, 0xCE, servoId, can);
}




