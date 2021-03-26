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
CAN_message_t genericWriteMessage(uint32_t canId, bool extended, uint8_t address, uint8_t servoId, uint16_t data) {
    uint8_t lo = data & 0x00FF;
    uint8_t hi = data >> 8;
    servo_message.len = 7;
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
    return servo_message;
}

// generic for read messages
CAN_message_t genericReadMessage(uint32_t canId, bool extended, uint8_t address, uint8_t servoId) {
    servo_message.len = 5;
    servo_message.id = canId;
    servo_message.flags.extended = extended;
    servo_message.buf[0] = 0x96;
    servo_message.buf[1] = servoId;
    servo_message.buf[2] = address;
    servo_message.buf[3] = 0x00;
    // check sum (id + address) & 0xFF
    servo_message.buf[4] = (servoId + address) & 0xFF;
    return servo_message;
}

// generic decoder for can messages (re-arranges buf segments to single int)
uint16_t genericDecoder(CAN_message_t msg) {
    return (uint16_t(msg.buf[5]) << 8) | uint16_t(msg.buf[4]);
}

// moves servo to an angle from -180 to 180 degrees
CAN_message_t REG_POSITION_NEW(uint32_t canId, bool extended, uint8_t servoId, double usrAngle) {
      return genericWriteMessage(canId, extended, 0x1E, servoId, uint16_t(usrAngle*ANGLE_CONVERSION+8192));
}

// moves servo to a new angle using raw register values (0 to 360 is 0 - 16383)
CAN_message_t REG_POSITION_NEW_RAW(uint32_t canId, bool extended,  uint8_t servoId, uint16_t regVal) {
      return genericWriteMessage(canId, extended,  0x1E, servoId, regVal);
}

// +1 rotates servo 360 in the + direction
CAN_message_t REG_TURN_NEW(uint32_t canId, bool extended,  uint8_t servoId, int16_t rotations) {
      return genericWriteMessage(canId, extended,  0x24, servoId, rotations);
} // to enable you must change run mode to continuous, save the config, then restart the servo

// 0: multi-turn mode, 1: servo mode
CAN_message_t REG_RUN_MODE(uint32_t canId, bool extended,  uint8_t servoId, uint16_t mode) {
      return genericWriteMessage(canId, extended,  0x44, servoId, mode);
}

// saves current reg values for reset
CAN_message_t REG_CONFIG_SAVE(uint32_t canId, bool extended,  uint8_t servoId) {
      return genericWriteMessage(canId, extended,  0x70, servoId, 0xFFFF);
}

// resets servo, applies config changes
CAN_message_t REG_POWER_CONFIG(uint32_t canId, bool extended,  uint8_t servoId) {
      return genericWriteMessage(canId, extended,  0x46, servoId, 0x0001);
}

// sets position max limit(s)
CAN_message_t REG_POSITION_MAX_LIMIT(uint32_t canId, bool extended,  uint8_t servoId, double maxAngle) {
      return genericWriteMessage(canId, extended,  0xB0, servoId, maxAngle);
}

// sets position min limit(s)
CAN_message_t REG_POSITION_MIN_LIMIT(uint32_t canId, bool extended,  uint8_t servoId, double minAngle) {
      return genericWriteMessage(canId, extended,  0xB2, servoId, minAngle);
}

// restore factory default
CAN_message_t REG_FACTORY_DEFAULT(uint32_t canId, bool extended,  uint8_t servoId) {
      return genericWriteMessage(canId, extended,  0x6E, servoId, 0x0F0F);
}

// change velocity max speed
CAN_message_t REG_VELOCITY_MAX(uint32_t canId, bool extended,  uint8_t servoId, int RPM) {
    double speed = RPM * 27.33;
    return genericWriteMessage(canId, extended,  0x54, servoId, speed);
}

// query for current servo velocity
CAN_message_t REG_VELOCITY(uint32_t canId, bool extended, uint8_t servoId) {
     return genericReadMessage(canId, extended, 0x0E, servoId);
}

// query for servo turn count
CAN_message_t REG_TURN_COUNT(uint32_t canId, bool extended, uint8_t servoId) {
    return  genericReadMessage(canId, extended, 0x18, servoId);
}

// decodes reported turn count extended,
int16_t decodeTurnCount(CAN_message_t msg) {
      return (int16_t)genericDecoder(msg);
}

// query for low value of current position (0 - 65535, 4096 = 90 deg)
CAN_message_t REG_32BITS_POSITION_L(uint32_t canId, bool extended, uint8_t servoId) {
      genericReadMessage(canId, extended, 0x1A, servoId);
}  

// decodes CAN position response integer value between -180 and 180 
double decodePositionLo(CAN_message_t msg) {
    double raw =  double(genericDecoder(msg));
    return ((raw - 8192) / ANGLE_CONVERSION);
}

// query for high value of position (65535 - 2^31, 4096 = 90 deg)
CAN_message_t REG_32BITS_POSITION_H(uint32_t canId, uint8_t servoId, bool extended) {
     return genericReadMessage(canId, extended, 0x1C, servoId);
} // TODO: write decoder

// // this is BROKEN and I DON'T KNOW WHY (problem exists on hitec prog too)
// CAN_message_t REG_POSITION(uint8_t servoId) {
//       genericReadMessage(0x0C, servoId);
// }

// query for current servo velocity
CAN_message_t REG_VOLTAGE(uint32_t canId, uint8_t servoId, bool extended) {
      return genericReadMessage(canId, extended, 0x12, servoId);
}

// decodes CAN voltage response
double decodeVoltage(CAN_message_t msg) {
    return double(genericDecoder(msg)) / 100;
}

// query mcu temperature
CAN_message_t REG_MCU_TEMPER(uint32_t canId, uint8_t servoId, bool extended) {
      return genericReadMessage(canId, extended, 0x14, servoId);
}

// decode CAN temperature response 
uint16_t decodeTemp(CAN_message_t msg) {
    return genericDecoder(msg);
}

// assigns a servo a new ID. not sure how this works for multiple servos on one can line
CAN_message_t REG_ID(uint32_t canId, bool extended, uint8_t servoId, uint16_t newId) {
      return genericWriteMessage(canId, extended, 0x32, servoId, newId);
}   // requires config save and reset

// sets CAN mode (0 for 2.0, 1 for 2.0B)
CAN_message_t REG_CAN_MODE(uint32_t canId, bool extended, uint16_t servoId, bool mode) {
      return genericWriteMessage(canId, extended, 0x6A, servoId, mode);
}

// sets high word for CAN bus ID (requires power cycle)
CAN_message_t REG_CAN_BUS_ID_H(uint32_t currCanId, bool extended, uint16_t newCanIdH, uint16_t servoId) {
      if (newCanIdH > 8191) {
            Serial.println("newCanIdH exceeds acceptable value");
            return;
      }
      
      return genericWriteMessage(currCanId, extended, 0x3C, servoId, newCanIdH);
}

// sets low word for CAN bus ID (requires power cycle)
CAN_message_t REG_CAN_BUS_ID_L(uint32_t currCanId, bool extended, uint16_t newCanIdL, uint16_t servoId) {
      if (newCanIdL > 65535) {
            Serial.println("newCanIdH exceeds acceptable value");
            return;
      }

      return genericWriteMessage(currCanId, extended, 0x3E, servoId, newCanIdL);
}

// write to user memory 1
CAN_message_t write_REG_USER_1(uint32_t canId, bool extended, uint16_t servoId, uint16_t data) {
      return genericWriteMessage(canId, extended, 0xCC, servoId, data);
}

// write to user memory 2
CAN_message_t write_REG_USER_2(uint32_t canId, bool extended, uint16_t servoId, uint16_t data) {
      return genericWriteMessage(canId, extended, 0xCE, servoId, data);
}

// write to reg_echo (volitile memory)
CAN_message_t write_REG_ECHO(uint32_t canId, bool extended, uint16_t servoId, uint16_t data) {
      return genericWriteMessage(canId, extended, 0xC6, servoId, data);
}
// read user memory 1
CAN_message_t read_REG_USER_1(uint32_t canId, bool extended, uint16_t servoId) {
      return genericReadMessage(canId, extended, 0xCC, servoId);
}

// read user memory 2
CAN_message_t read_REG_USER_2(uint32_t canId, bool extended, uint16_t servoId) {
      return genericReadMessage(canId, extended, 0xCE, servoId);
}

// read reg echo
CAN_message_t read_REG_ECHO(uint32_t canId, bool extended, uint16_t servoId) {
      return genericReadMessage(canId, extended, 0xC6, servoId);
}

// sets stream time
CAN_message_t REG_STREAM_TIME(uint32_t canId, bool extended, uint16_t servoId, uint16_t periodMs) {
      return genericWriteMessage(canId, extended, 0x2E, servoId, periodMs);
}

// sets stream mode (0 off, 1 on)
CAN_message_t REG_STREAM_MODE(uint32_t canId, bool extended, uint16_t servoId, bool mode) {
      return genericWriteMessage(canId, extended, 0x30, servoId, mode);
}

/** these don't work don't mess with 'em for now
// sets custom stream address 0
CAN_message_t REG_STREAM_ADDR_0(uint32_t canId, bool extended, uint16_t servoId, uint8_t addr0, uint8_t addr1) {
      return genericWriteMessage(canId, extended, 0xE2, servoId, ((uint16_t)(addr0) << 8) | (uint16_t)addr1);
}

// sets custom stream address 1
CAN_message_t REG_STREAM_ADDR_1(uint32_t canId, bool extended, uint16_t servoId, uint8_t addr0, uint8_t addr1) {
      return genericWriteMessage(canId, extended, 0xE4, servoId, ((uint16_t)(addr0) << 8) | (uint16_t)addr1);
}

// sets custom stream address 2
CAN_message_t REG_STREAM_ADDR_2(uint32_t canId, bool extended, uint16_t servoId, uint8_t addr0, uint8_t addr1) {
      return genericWriteMessage(canId, extended, 0xE6, servoId, ((uint16_t)(addr0) << 8) | (uint16_t)addr1);
}

// sets custom stream address 3
CAN_message_t REG_STREAM_ADDR_3(uint32_t canId, bool extended, uint16_t servoId, uint8_t addr0, uint8_t addr1) {
      return genericWriteMessage(canId, extended, 0xE8, servoId, ((uint16_t)(addr0) << 8) | (uint16_t)addr1);
}
*/



