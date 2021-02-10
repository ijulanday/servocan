/**
 * an example using servocan and keybdb. 
 * 
 * CLI control via serial monitor for teensy for use with Currawong servos
*/

#include "keybdb.h"
#include "servocan.hh"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
char servomode = 's';
char id = 0x00;
double pos;
double posNext;
int turnCount = 0;
double turnDeg = 2;
double lastReadPosition = 0;

KeyBDB mykeybdb(Serial);

void setup() {
    Serial.begin(9600);
    can1.begin();
    can1.setBaudRate(1000000);

    while(!Serial) {
        //wait for serial to start
    }
    
    REG_RUN_MODE(id, 1, &can1);
    delay(20);
    REG_CONFIG_SAVE(id, &can1);
    delay(20);
    REG_POWER_CONFIG(id, &can1);
    delay(200);

    REG_TURN_COUNT(id, &can1);
    delay(20);
    if (can1.read(msg))
        turnCount = decodeTurnCount(msg);

    Serial.println("\n--- Commands ---");
    Serial.println("m ... change mode");
    Serial.print("q ... left "); Serial.print(turnDeg); Serial.print(" / left 1 turn"); Serial.println("");
    Serial.print("e ... right "); Serial.print(turnDeg); Serial.print(" / right 1 turn"); Serial.println("");
    Serial.println("c ... configure servo");
}

void loop() {
    // read input
    mykeybdb.recvOneChar();

    // do stuff
    if (mykeybdb.newData) {
        if (mykeybdb.receivedChar == 'm') {
            mykeybdb.clear();
            Serial.print("Switch mode (s for \'servo\', c for \'continuous\')");
            Serial.print("                                                                                    ");
        }

        if (mykeybdb.receivedChar == 's' && mykeybdb.lastChar == 'm') {
            // set servo mode
            turnCount = 0;
            servomode = 's';
            mykeybdb.clear();
            REG_RUN_MODE(id, 1, &can1);
            delay(20);
            REG_CONFIG_SAVE(id, &can1);
            delay(20);
            REG_POWER_CONFIG(id, &can1);
            delay(20);
            Serial.print("Config changed to SERVO mode!");
            Serial.print("                                                                                    ");
        }

        if (mykeybdb.receivedChar == 'c' && mykeybdb.lastChar == 'm') {
            // set continuous mode
            servomode = 'c';
            mykeybdb.clear();
            REG_RUN_MODE(id, 0, &can1);
            delay(20);
            REG_CONFIG_SAVE(id, &can1);
            delay(20);
            REG_POWER_CONFIG(id, &can1);
            delay(20);
            Serial.print("Config changed to CONTINUOUS mode!");
            Serial.print("                                                                                    ");
        }

        if (mykeybdb.receivedChar == 'e') {
            // left rotate stuff
            if (servomode == 's') {
                REG_32BITS_POSITION_L(id, &can1);
                delay(20);
                if (can1.read(msg)) 
                    pos = decodePositionLo(msg);
                REG_POSITION_NEW(id, pos + turnDeg, &can1); 
                mykeybdb.clear();
                Serial.print("Rotated right... (angle: "); Serial.print(pos); Serial.print(" degrees)");      
            } else {
                turnCount += 1;
                REG_TURN_NEW(id, turnCount, &can1);
                delay(20);
                mykeybdb.clear();
                Serial.print("Did a full rotation right... ("); Serial.print(turnCount); Serial.print(")");
            }
            Serial.print("                                                                                    ");
        }

        if (mykeybdb.receivedChar == 'q') {
            // right rotate stuff
            if (servomode == 's') {
                REG_32BITS_POSITION_L(id, &can1);
                delay(20);
                if (can1.read(msg)) 
                    pos = decodePositionLo(msg);
                REG_POSITION_NEW(id, pos - turnDeg, &can1);    
                mykeybdb.clear();
                Serial.print("Rotated left... (angle: "); Serial.print(pos); Serial.print(" degrees)");      
            } else {
                turnCount -= 1;
                REG_TURN_NEW(id, turnCount, &can1);
                delay(20);
                mykeybdb.clear();
                Serial.print("Did a full rotation left... ("); Serial.print(turnCount); Serial.print(")");
            }
            Serial.print("                                                                                    ");
        }

        if (mykeybdb.receivedChar == 'c') {
            // do servo config
            mykeybdb.clear();
            Serial.print("Key actions: p for \'read Position\', a for \'set mAx position\', i for \'set mIn position\', r for \'reset to default\'");
        }

        if (mykeybdb.receivedChar == 'p' && mykeybdb.lastChar == 'c') {
            mykeybdb.clear();
            REG_32BITS_POSITION_L(id, &can1);
            delay(20);
            if (can1.read(msg)) {
                lastReadPosition = decodePositionLo(msg);
                int raw = (uint16_t(msg.buf[5]) << 8) | uint16_t(msg.buf[4]);
                Serial.print("Position is currently "); Serial.print(lastReadPosition); Serial.print("("); Serial.print(raw); Serial.print(")");
                Serial.print("                                                                                    ");
            } else {
                Serial.print("couldn't read position register (try again?)");
            }
        }

        if (mykeybdb.receivedChar == 'a' && mykeybdb.lastChar == 'c') {
            mykeybdb.clear();
            REG_POSITION_MAX_LIMIT(id, ((double)lastReadPosition * ANGLE_CONVERSION) + 8192, &can1);
            Serial.print("Max limit set to "); Serial.print(lastReadPosition);
            Serial.print("                                                                                    ");
        }

        if (mykeybdb.receivedChar == 'i' && mykeybdb.lastChar == 'c') {
            mykeybdb.clear();
            REG_POSITION_MIN_LIMIT(id, ((double)lastReadPosition * ANGLE_CONVERSION) + 8192, &can1);
            Serial.print("Min limit set to "); Serial.print(lastReadPosition);
            
        } 

        if (mykeybdb.receivedChar == 'r' && mykeybdb.lastChar == 'c') {
            mykeybdb.clear();
            REG_POSITION_MIN_LIMIT(id, 0, &can1);
            delay(20);
            REG_POSITION_MAX_LIMIT(id, 16383, &can1);
            delay(20);
            REG_POSITION_NEW(id, 0, &can1);
            Serial.print("Reset min/max limits to 0, 360 (and went to 0)");
        }
    }

    mykeybdb.newData = false;
}