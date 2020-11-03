# servocan
A C++ library for use with HITEC CAN servos on Teensy 4.0 & 4.1.

## implemented commands
| Command                 | Description                             |
| ----------------------- | --------------------------------------- |
| REG_POSITION_NEW&nbsp   |   move to a new angle                   |
| REG_TURN_NEW            |   make a turn (must be in turn mode!)   |
| REG_RUN_MODE            |   change between angular/turn mode      |
| REG_CONFIG_SAVE         |   save register config changes          |
| REG_POWER_CONFIG        |   turn on, off, or restart the servo    |       
| REG_POS_MAX             |   set max angle                         |
| REG_POS_MIN             |   set min angle                         |
| REG_FACTORY_DEFAULT     |   reset to factory defaults             |
| REG_TURN_COUNT          |   request number of completed turns     |
| REG_32BITS_POSITION_L   |   get low position register value       |
| REG_32BITS_POSITION_H   |   get high position register value      |
| REG_POSITION            |   get position                          |
| REG_VOLTAGE             |   get servo voltage                     |
| REG_MCU_TEMPER          |   get MCU temperature                   |
| REG_ID                  |   change servo ID                       |

Note that command effectiveness may vary between servo models.

## usage
Just clone into your project, e.g, under /lib if using [platformio](https://platformio.org/). The library just helps with building messages; handling the protocol is something that needs to be done per application. Here's an example of a state machine that moves a servo in increments of 90 degrees:
```c
#include "servocan.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;

int st;
int pos;
int posNext;

void setup() {
    // set up can stuff
    can1.begin();
    can1.setBaudRate(1000000);
    Serial.begin(9600); 
    st = 4;
}

void loop() {
    // driving state machine
    switch (st) {
        case 1:
            // query for position
            REG_32BITS_POSITION_L(0x00, &can1);
            delay(20);
            st = 2;
            break;
        case 2:
            // move to update state if goal is reached
            if (can1.read(msg)) {
                pos = decodePositionLo(msg);
                (abs(pos - posNext) < 4) ? st = 3 : st = 1;
            }
            break;
        case 3:
            // update goal position and execute on servo
            pos + 90 > 360 ? posNext = 0 : posNext = pos + 90;
            REG_POSITION_NEW(0x00, posNext, &can1);
            delay(20);
            st = 1;
            break;
        case 4:
            // initially update position
            if (can1.read(msg)) {
                pos = decodePositionLo(msg);
                st = 3;
            }
            break;
    }
}
```

## protocol notes

Here are just a few observations relating to the actual servo CAN protocol. These notes can be infered from the relevant servo documentation, so these serve as a quick digest of some important points.

#### Broadcast commands (ID field 0x00)
Broadcasting a packet with the ID 0x00 will effect each servo on the CAN bus, i.e. REG_TURN_NEW will cause all servos on the bus to follow this command regardless of their particular address. The servos responses, however, will show 0x00 in their response packet's ID field. When from multiple devices broadcast to on 0x00, one may need to issue the command multiple times as device responses may not all be captured. Usually, however, all devices receive a command on 0x00 successfully first try. 

#### Config changes
Certain commands that alter the configuration of the servo, i.e. changing the servo's ID or switching between servo and continuous mode, require a configuration save and power cycle. Here's a basic code example, for quick reference:

```c
    //...
    REG_ID(0x00, id, &can1);
    delay(20);
    REG_CONFIG_SAVE(id, &can1);
    delay(20);
    REG_POWER_CONFIG(id, &can1);
    delay(20);
    //...
```   


## dependencies
*   Arduino.h 
*   math.h
*   [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4/)

