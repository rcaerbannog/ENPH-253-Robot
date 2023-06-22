#include <Arduino.h>

// PIN DEFINITIONS
#define PIN_TAPE_SENSOR_LEFT PA0
#define PIN_TAPE_SENSOR_RIGHT PA1

// ENUMS


// CONSTANTS



// VARIABLES

// Tape following
bool rightOnTape;
bool leftOnTape;
int tapePosErr; // between -2 and +2
int tapePosErrPrev;


// put function declarations here:
int myFunction(int, int);
void updateTapeSensor();
void steering();
void motorControl();

void setup() {
  pinMode(PIN_TAPE_SENSOR_LEFT, INPUT_PULLUP);
  pinMode(PIN_TAPE_SENSOR_RIGHT, INPUT_PULLUP);
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

void updateTapeSensor() {
  
}

/*
 * PID
 */
void motorControl() {

}

/*
 * PID
*/
void steering() {

}