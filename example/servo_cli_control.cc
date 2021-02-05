/**
 * an example using servocan and keybdb. 
 * 
 * CLI control via serial monitor for teensy for use with Currawong servos
*/

#include "keybdb.h"
#include "servocan.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
char servomode = 's';
char id = 0x00;
int pos;
int posNext;
int turnCount = 0;

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
    Serial.println("q ... left 15 / left 1 turn");
    Serial.println("r ... right 15 / right 1 turn");
}

void loop() {
    // read input
    mykeybdb.recvOneChar();

    // do stuff
    if (mykeybdb.newData) {
        if (mykeybdb.receivedChar == 'm') {
            mykeybdb.clear();
            Serial.print("Switch mode (s for \'servo\', c for \'continuous\')");
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
        }

        if (mykeybdb.receivedChar == 'e') {
            // left rotate stuff
            if (servomode == 's') {
                REG_32BITS_POSITION_L(id, &can1);
                delay(20);
                if (can1.read(msg)) 
                    pos = decodePositionLo(msg);
                REG_POSITION_NEW(id, pos + 15, &can1); 
                mykeybdb.clear();
                Serial.print("Rotated right 15 degrees...");               
            } else {
                turnCount += 1;
                REG_TURN_NEW(id, turnCount, &can1);
                delay(20);
                mykeybdb.clear();
                Serial.print("Did a full rotation right... ("); Serial.print(turnCount); Serial.print(")");
            }
        }

        if (mykeybdb.receivedChar == 'q') {
            // right rotate stuff
            if (servomode == 's') {
                REG_32BITS_POSITION_L(id, &can1);
                delay(20);
                if (can1.read(msg)) 
                    pos = decodePositionLo(msg);
                REG_POSITION_NEW(id, pos - 15, &can1);    
                mykeybdb.clear();
                Serial.print("Rotated left 15 degrees...");             
            } else {
                turnCount -= 1;
                REG_TURN_NEW(id, turnCount, &can1);
                delay(20);
                mykeybdb.clear();
                Serial.print("Did a full rotation left... ("); Serial.print(turnCount); Serial.print(")");
            }
        }
    }

    mykeybdb.newData = false;
}