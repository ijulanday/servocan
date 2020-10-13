# servocan
A C++ library for use with HITEC CAN servos on Teensy 4.0.

## usage
Just clone into your project, e.g, under /lib if using [platformio](https://platformio.org/). The library just helps with building messages, handling the protocol is something that needs to be done per application. Here's an example of a state machine that moves a servo in increments of 90 degrees:
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

    // get current position and start at state 4
    can1.write(getPositionLo(0x00));
    delay(10);
    st = 4;
}

void loop() {
    // driving state machine
    switch (st) {
        case 1:
            // query for position
            can1.write(getPositionLo(0x00));
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
            can1.write(positionMessage(0x00, posNext));
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

## dependencies
Arduino
[FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4/)

