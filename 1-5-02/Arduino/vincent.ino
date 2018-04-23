#include <NewTone.h>
#include <NewPing.h>
#include <SparkFun_MAG3110.h>
#include <StackArray.h>
#include <serialize.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

typedef enum {
  STOP = 0,
  FORWARD = 1,
  REVERSE = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

#define trig1 9
#define echo1 8
#define trig2 13
#define echo2 12

/*
 * Vincent's configuration constants
 *///

// Number of ticks per revolution from the wheel encoder.
#define COUNTS_PER_REV 185

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled by taking revs * WHEEL_CIRC
#define WHEEL_CIRC          20.1

// Motor control pins. You need to adjust these till Vincent moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10  // Right reverse pin

// Ultrasonic sensor pins
#define trigR               9
#define echoR               8
#define trigL               13
#define echoL               12

// Buzzer pin
#define buzz                7

//PI, for calculating turn circumference
//#define PI 3.141592654

//Vincent's length and breadth in cm
#define VINCENT_LENGTH 16 //19.2
#define VINCENT_BREADTH 8 //12.7

//Vincent's diagonal, will be computed once and stored
float vincentDiagonal=0.0;

//Vincent's turning circumference, will be computed once and stored
float vincentCirc=0.0;

/*
 *    Vincent's State Variables
 */

// Store the ticks from Vincent's left and right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

//Left and Right encoder tiks for turning
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Vincent's left and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variables to kepp track of whether we have moved a commanded dist
unsigned long deltaDist;
unsigned long newDist;

//Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

//Variables for adjusting percentage reduction for motors during calibration
float reduceRightF;
float reduceLeftF;
float reduceRightB;
float reduceLeftB;

// Variable for adjusting speed in main loop
int orig0A;
int orig2A;
int orig0B;
int orig1B;

//Variable for start of auto backtracking
int isBackTrack = 0;

//Stack and structure for keeping track of commands
typedef struct {
  int command;
  float spd;
  float dist_or_angle;
} commands;

StackArray <commands> stack;
StackArray <commands> stack_backup;
StackArray <commands> tempstack;

/*
 * 
 * Vincent Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet) {
    // Reads in data from the serial port and deserializes it.
    // Returns deserialized data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;
    len = readSerial(buffer);
    
    if(len == 0) return PACKET_INCOMPLETE;
    else return deserialize(buffer, len, packet);
}

void sendStatus() {
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs, forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}


void sendDone() {
  TPacket donePacket;
  donePacket.packetType = PACKET_TYPE_RESPONSE;
  donePacket.command = RESP_DONE;
  sendResponse(&donePacket);  
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out over the serial port.
  
  char buffer[PACKET_SIZE];
  int len;
  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and pullup resistors.
 */
 
// Enable pull up resistors on pins 2 and 3
void enablePullups() {
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  
  DDRD |= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR() {
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == REVERSE) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT) leftReverseTicksTurns++;
  else if (dir == RIGHT) leftForwardTicksTurns++;
}

void rightISR() {
  if (dir == FORWARD) rightForwardTicks++;
  else if (dir == REVERSE) rightReverseTicks++;
  else if (dir == LEFT) rightForwardTicksTurns++;
  else if (dir == RIGHT) rightReverseTicksTurns++;
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  
  EIMSK |= 0b00000011;
  EICRA |= 0b00001010;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR (INT0_vect) {
  leftISR();
}

ISR (INT1_vect) {
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 */

// Set up the serial connection. For now we are using Arduino Wiring, 
// you will replace this later with bare-metal code.

void setupSerial() {
  // To replace later with bare-metal.
  //UBRR0L = 103;
  //UBRR0H = 0;
  //UCSR0C = 0b00000110;
  //UCSR0A = 0;
  Serial.begin(9600);
}

// Start the serial connection. For now we are using Arduino wiring and 
// this function is empty. We will replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code later on.
  //UCSR0B = 0b10111000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {
  int count = 0;
  while(Serial.available()) buffer[count++] = Serial.read();
  return count;
}

// Write to the serial port. Replaced later with bare-metal code

void writeSerial(const char *buffer, int len) {
  Serial.write(buffer, len);
}

/*
 * Vincent's motor drivers.
 */

// Set up Vincent's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.

void setupMotors() {
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
  DDRD |= ((1 << DDD6) | (1 << DDD5));
  DDRB |= ((1 << DDB2) | (1 << DDB3));
  
  TCNT0 = 0;
  OCR0A = 0;
  OCR0B = 0;

  TCNT1 = 0;
  OCR1B = 0;

  TCNT2 = 0;
  OCR2A = 0;
}

// Start the PWM for Vincent's motors.
// We will implement this later. For now it is blank.

void startMotors() {
  TCCR0B = 0b00000011;
  TCCR1B = 0b00000011;
  TCCR2B = 0b00000100;
}

// Convert percentages to PWM values

int pwmVal(float speed) {
  if(speed < 0.0) speed = 0;
  if(speed > 100.0) speed = 100.0;
  return (int) ((speed / 100.0) * 255.0);
}

// Move Vincent forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is move forward at half speed.
// Specifying a distance of 0 means Vincent will continue moving forward indefinitely.

void forward(float dist, float speed) {
  reduceRightF = 1.0;
  reduceLeftF = 0.999;
  reduceRightB = 0.99997;
  reduceLeftB = 1.0;
  
  if (!isBackTrack) {
    commands tempstruct;
    tempstruct.command = FORWARD;
    tempstruct.spd = speed;
    tempstruct.dist_or_angle = dist;
    stack.push(tempstruct);
    stack_backup.push(tempstruct);
  }
  
  dir = FORWARD;
  int val = pwmVal(speed);

  // For now we will ignore dist and move forward indefinitely. We will fix this in Week 9.
  if (dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = forwardDist + deltaDist;
  
  OCR0A = val*reduceLeftF;  // LF
  OCR0B = 0;  // LR
  TCCR0A = 0b10000001;
  OCR1B = 0;  // RR
  TCCR1A = 0b00000001;
  OCR2A = val*reduceRightF;  // RF
  TCCR2A = 0b10000001;
  
  PORTD &= 0b11011111;  // set pd5 ie pin 5 to be off
  PORTB &= 0b11111011;  // set pb2 ie pin 10 to be off

  orig0A = OCR0A;
  orig2A = OCR2A;
}

// Reverse Vincent "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is reverse at half speed.
// Specifying a distance of 0 means Vincent will continue reversing indefinitely.

void reverse(float dist, float speed) {
  reduceRightF = 1.0;
  reduceLeftF = 0.999;
  reduceRightB = 0.99997;
  reduceLeftB = 1.0;
    
  if (!isBackTrack) {
    commands tempstruct;
    tempstruct.command = REVERSE;
    tempstruct.spd = speed;
    tempstruct.dist_or_angle = dist;
    stack.push(tempstruct);
    stack_backup.push(tempstruct);
  }
  
  dir = REVERSE;
  int val = pwmVal(speed);

  // For now we will ignore dist and reverse indefinitely. We will fix this in Week 9.
  if (dist > 0) deltaDist = dist;
  else deltaDist = 9999999;
  newDist = reverseDist + deltaDist;
  
  OCR0A = 0;  // LF
  OCR0B = val*reduceLeftB;  // LR
  TCCR0A = 0b00100001;
  OCR1B = (float) val/255*65535*reduceRightB;  // RR
  TCCR1A = 0b00100001;
  OCR2A = 0;  // RF
  TCCR2A = 0b00000001;
  
  PORTD &= 0b10111111;  // set pd6 ie pin 6 to be off
  PORTB &= 0b11110111;  // set pb3 ie pin 11 to be off

  orig0B = OCR0B;
  orig1B = OCR1B;
}

//New function to estimate number of wheel ticks needed to turn an angle

unsigned long computeLDeltaTicks(float ang){
  unsigned long ticks = (unsigned long) ((ang * vincentCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks*0.4;
}

unsigned long computeRDeltaTicks(float ang){
  unsigned long ticks = (unsigned long) ((ang * vincentCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks*0.6;
}

// Turn Vincent left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to turn left indefinitely.

void left(float ang, float speed) {
  reduceRightF = 1.0;
  reduceLeftF = 0.999;
  reduceRightB = 0.99997;
  reduceLeftB = 1.0;
    
  int val = pwmVal(speed);
  dir = LEFT;

  if (ang == 0) deltaTicks = 9999999;
  else deltaTicks = computeLDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks;

  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  //analogWrite(LR, val * reduceLeftF);
  //analogWrite(RF, val * reduceRightF);
  //analogWrite(LF, 0);
  //analogWrite(RR, 0);

  OCR0A = 0;  // LF
  OCR0B = val;  // LR
  TCCR0A = 0b00100001;
  OCR1B = 0;  // RR
  TCCR1A = 0b00000001;
  OCR2A = val;  // RF
  TCCR2A = 0b10000001;

  PORTD &= 0b10111111;  // set pd6 ie pin 6 to be off
  PORTB &= 0b11111011;  // set pb2 ie pin 10 to be off
}

// Turn Vincent right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to turn right indefinitely.

void right(float ang, float speed) {
  reduceRightF = 1.0;
  reduceLeftF = 0.999;
  reduceRightB = 0.99997;
  reduceLeftB = 1.0;
  
  int val = pwmVal(speed);
  dir = RIGHT;

  if (ang == 0) deltaTicks = 9999999;
  else deltaTicks = computeRDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;
  
  OCR0A = val;  // LF
  OCR0B = 0;  // LR
  TCCR0A = 0b10000001;
  OCR1B = (float) val/255*65535;  // RR
  TCCR1A = 0b00100001;
  OCR2A = 0;  // RF
  TCCR2A = 0b00000001;

  PORTD &= 0b11011111;  // set pd5 ie pin 5 to be off
  PORTB &= 0b11110111;  // set pb3 ie pin 11 to be off
}

void adjust() {
  clearCounters();
  dir = FORWARD;
  deltaDist = 200;
  newDist = forwardDist + deltaDist;

  //analogWrite(RF, 255);
  //analogWrite(LF, 255);
  //analogWrite(LR, 0);
  //analogWrite(RR, 0);
  
  OCR0A = 255;  // LF
  OCR0B = 0;  // LR
  TCCR0A = 0b10000001;
  OCR1B = 0;  // RR
  TCCR1A = 0b00000001;
  OCR2A = 255;  // RF
  TCCR2A = 0b10000001;
  
  PORTD &= 0b11011111;  // set pd5 ie pin 5 to be off
  PORTB &= 0b11111011;  // set pb2 ie pin 10 to be off
  
  while(forwardDist < newDist){
    if (forwardDist > 150 && forwardDist < 170) {
      if (leftForwardTicks > rightForwardTicks) reduceLeftF = (float)rightForwardTicks / (float)leftForwardTicks;
      else if (rightForwardTicks > leftForwardTicks) reduceRightF = (float)leftForwardTicks / (float)rightForwardTicks;
    }
  }
  
  stop();
  delay(2000);
  
  dir = REVERSE;  
  newDist = reverseDist + deltaDist;

  //analogWrite(RF, 0);
  //analogWrite(LF, 0);
  //analogWrite(LR, 255);
  //analogWrite(RR, 255);

  OCR0A = 0;  // LF
  OCR0B = 255;  // LR
  TCCR0A = 0b00100001;
  OCR1B = 65535;  // RR
  TCCR1A = 0b00100001;
  OCR2A = 0;  // RF
  TCCR2A = 0b00000001;
  
  PORTD &= 0b10111111;  // set pd6 ie pin 6 to be off
  PORTB &= 0b11110111;  // set pb3 ie pin 11 to be off
  
  while(reverseDist < newDist){
    if (reverseDist > 150 && reverseDist < 170) {
      if (leftReverseTicks > rightReverseTicks) reduceLeftB = (float)rightReverseTicks / (float)leftReverseTicks;
      else if (rightReverseTicks > leftReverseTicks) reduceRightB = (float)leftReverseTicks / (float)rightReverseTicks;
    }
  }

  stop();

  // Reset variables
  deltaDist = 0;
  newDist = 0;
  dir = STOP;
}

// Stop Vincent. To replace with bare-metal code later.
void stop() {
  dir = STOP;
  //analogWrite(LF, 0);
  //analogWrite(LR, 0);
  //analogWrite(RF, 0);
  //analogWrite(RR, 0);

  OCR0A = 0;  // LF
  OCR0B = 0;  // LR
  TCCR0A = 0b00000001;
  OCR1B = 0;  // RR
  TCCR1A = 0b00000001;
  OCR2A = 0;  // RF
  TCCR2A = 0b00000001;
  
  PORTD &= 0b10011111;  // set pd6 pd5 ie pin 6 pin 5 to be off
  PORTB &= 0b11110011;  // set pb3 pb2 ie pin 11 pin 10 to be off
}

/*
 * Vincent's setup and run codes
 */

// Clears all our counters
void clearCounters() {
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  forwardDist=0;
  reverseDist=0;
  reduceRightF = 1.0;
  reduceLeftF = 0.999;
  reduceRightB = 0.99997;
  reduceLeftB = 1.0;
}

// Clears one particular counter
void clearOneCounter(int which) {
  clearCounters();
}

// Initialize Vincent's internal states
void initializeState() {
  clearCounters();
}

void store() {
  commands tempstruct;
  tempstruct.command = -1;
  tempstruct.spd = 0;
  tempstruct.dist_or_angle = -0;
  stack.push(tempstruct);
  stack_backup.push(tempstruct);
}

void backtrack(){
  isBackTrack = 1;

  left (193, 80);
  while(leftReverseTicksTurns < targetTicks);
  deltaTicks = 0;
  targetTicks = 0;
  stop();
  clearCounters();
  
  while(!stack.isEmpty()) {
    int stackcommand;
    float stackspeed;
    float stackdist_or_angle;

    _delay_ms(1000);

    stackcommand = stack.peek().command;
    stackspeed = stack.peek().spd;
    stackdist_or_angle = stack.peek().dist_or_angle;
    stack.pop();

    if(stackcommand == FORWARD) {
      forward(stackdist_or_angle, stackspeed);
      if(deltaDist > 0) {
        while(forwardDist < newDist) {
          if (forwardDist != 0) {
    
            // OCR0A is LF, OCR2A is RF, sonar[0] is L, sonar[1] is R
            if (leftForwardTicks > rightForwardTicks && orig0A - OCR0A < 10 && OCR2A - orig2A < 10) {
              OCR0A -= 10;
              OCR2A += 10;
            }
            else if (rightForwardTicks > leftForwardTicks && orig2A - OCR2A < 10 && OCR0A - orig0A < 10) {
              OCR2A -= 10;
              OCR0A += 10;
            }
          }
        }

        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    
    else if(stackcommand == REVERSE) {
      reverse(stackdist_or_angle,stackspeed);
      if(deltaDist > 0) {
        while(reverseDist < newDist) {
          if (reverseDist != 0) {
    
            // OCR0B is LR, OCR1B is RR, sonar[0] is L, sonar[1] is R
            if (leftReverseTicks > rightReverseTicks && orig0B - OCR0B < 10 && OCR1B - orig1B < 2560) {
              OCR0B -= 10;
              OCR1B += 2560;
            }
            else if (rightReverseTicks > leftReverseTicks && OCR0B - orig0B < 10 && orig1B - OCR1B < 2560) {
              OCR1B -= 2560;
              OCR0B += 10;
            }
          }
        }
        
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    
    else if(stackcommand == LEFT) {
      left(stackdist_or_angle,stackspeed);
      if(deltaTicks > 0) {
        while(leftReverseTicksTurns < targetTicks);
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }    
    else if(stackcommand == RIGHT) {
      right(stackdist_or_angle, stackspeed);
      if(deltaTicks > 0) {
        while(rightReverseTicksTurns < targetTicks);
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (stackcommand == -1) {
      for (int hz = 440; hz < 1000; hz++) {
        NewTone(buzz, hz, 50);
        _delay_ms(5);
      }
      noNewTone(buzz);

      for (int hz = 1000; hz > 440; hz--) {
        NewTone(buzz, hz, 50);
        _delay_ms(5);
      }
      noNewTone(buzz);
    }
  }
  stop();
  isBackTrack = 0;
  sendOK();
}

void store_left(float ang, float speed) {
  commands tempstruct;
  tempstruct.command = RIGHT;
  tempstruct.spd = speed;
  tempstruct.dist_or_angle = ang;
  stack.push(tempstruct);
  stack_backup.push(tempstruct);
}

void store_right(float ang, float speed) {
  commands tempstruct;
  tempstruct.command = LEFT;
  tempstruct.spd = speed;
  tempstruct.dist_or_angle = ang;
  stack.push(tempstruct);
  stack_backup.push(tempstruct);
}

void copystack() {
  while(!stack.isEmpty()) {
    stack.pop();
  }
  while(!stack_backup.isEmpty()) {
    tempstack.push(stack_backup.peek());
    stack_backup.pop();
  }
  while(!tempstack.isEmpty()) {
    stack.push(tempstack.peek());
    stack_backup.push(tempstack.peek());
    tempstack.pop();
  }

  //sendOCR();
}

void stackpop() {
  stack.pop();
  TPacket message;
  int mes;
  char mes1[10];
  mes = stack.count();
  itoa(mes, mes1, 10);
  message.packetType = PACKET_TYPE_MESSAGE;
  strncpy(message.data, mes1, 4);
  sendResponse(&message);
  
}


void handleCommand(TPacket *command) {
  switch(command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;
        
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;
        
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
        
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
        
    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendOK();
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;    

    case COMMAND_ADJUST:
      sendOK();
      adjust();
      break;

    case COMMAND_BACKTRACK:
      sendOK();
      backtrack();
      sendOK();
      break;

    case COMMAND_STORE:
      sendOK();
      store();
      sendOK();
      break;

    case COMMAND_STORE_LEFT:
      sendOK();
      store_left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STORE_RIGHT:
      sendOK();
      store_right((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_COPY_STACK:
      sendOK();
      copystack();
      break;

    case COMMAND_POP_STACK:
      sendOK();
      stackpop();
      break;
      
    default: sendBadCommand();
  }
}

void waitForHello() {
  int exit = 0;

  while(!exit) {
    TPacket hello;
    TResult result;
    
    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK) {
      if(hello.packetType == PACKET_TYPE_HELLO) {
        sendOK();
        exit = 1;
      }
      else sendBadResponse();
    }
    else if(result == PACKET_BAD) sendBadPacket();
    else if(result == PACKET_CHECKSUM_BAD) sendBadChecksum();
  } // !exit
}

void setup() {
  // computes his diagonal and circumference once
  vincentDiagonal = sqrt((VINCENT_LENGTH * VINCENT_LENGTH) + (VINCENT_BREADTH * VINCENT_BREADTH));
  vincentCirc = PI * vincentDiagonal;
  
  // put your setup code here, to run once:
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  //calibrateMag();
  sei();
  pinMode (7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, INPUT);
  pinMode(13, OUTPUT);
  pinMode(12, INPUT);

  dir = STOP;
}

void handlePacket(TPacket *packet) {
  switch(packet->packetType) {
    
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;
    case PACKET_TYPE_RESPONSE: break;
    case PACKET_TYPE_ERROR: break;
    case PACKET_TYPE_MESSAGE: break;
    case PACKET_TYPE_HELLO: break;
  }
}

void loop() {
  TPacket recvPacket; // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK) handlePacket(&recvPacket);
  else if(result == PACKET_BAD) sendBadPacket();
  else if(result == PACKET_CHECKSUM_BAD) sendBadChecksum(); 
  
  //For moving forward exactly    
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist != 0) {
        // OCR0A is LF, OCR2A is RF, sonar[0] is L, sonar[1] is R
        if (leftForwardTicks > rightForwardTicks && orig0A - OCR0A < 10 && OCR2A - orig2A < 10) { 
          OCR0A -= 10;
          OCR2A += 10;
        }
        else if (rightForwardTicks > leftForwardTicks && orig2A - OCR2A < 10 && OCR0A - orig0A < 10) {
          OCR2A -= 10;
          OCR0A += 10;
        }
        
      }

      //Checks if forward dist exceeds newDist(forward dist+deltaDist)
      if(forwardDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
        sendDone();
      }
    }
    else if (dir == REVERSE) {
      if (reverseDist != 0) {
        // OCR0B is LR, OCR1B is RR, sonar[0] is L, sonar[1] is R
        if (leftReverseTicks > rightReverseTicks && orig0B - OCR0B < 10 && OCR1B - orig1B < 2560) {
          OCR0B -= 10;
          OCR1B += 2560;
        }
        else if (rightReverseTicks > leftReverseTicks && OCR0B - orig0B < 10 && orig1B - OCR1B < 2560) {
          OCR1B -= 2560;
          OCR0B += 10;
        }
      }
      if (reverseDist >= newDist) {        
        deltaDist = 0;
        newDist = 0;
        stop();
        sendDone();
      }
    }
    else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  //For Turning exactly 
  if(deltaTicks > 0) {
    if(dir == LEFT) {
      if(leftReverseTicksTurns >= targetTicks){
        deltaTicks = 0;
        targetTicks = 0;
        stop();
        sendDone();
      }
    }
    else if(dir == RIGHT) {
      if(rightReverseTicksTurns >= targetTicks) {
        deltaTicks=0;
        targetTicks=0;
        stop();
        sendDone();
      }
    }
    else if(dir == STOP) {
      deltaTicks=0;
      targetTicks=0;
      stop();
    }
  }
}
