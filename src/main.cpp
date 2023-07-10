#include <Arduino.h>
#include <Servo.h>

// PIN DEFINITIONS
#define PIN_TAPE_SENSOR_LEFT PA4	// as analog voltage reading
#define PIN_TAPE_SENSOR_RIGHT PA5	// as analog voltage reading
#define PIN_STEERING_SERVO PA11	// as PWM servo control
#define PIN_LMOTOR_FWD PB6
#define PIN_LMOTOR_REV PB7
#define PIN_RMOTOR_FWD PB8
#define PIN_RMOTOR_REV PB9



// ENUMS


// CONSTANTS
const int TAPE_SENSOR_THRESHOLD = 500;


// VARIABLES

// Tape following
bool rightOnTape;
bool leftOnTape;
int tapePosErr; // between -2 and +2
int tapePosErrPrev;

// Motor control
int motorDutyCycle = 200; 	// duty cycle fraction out of 255. Manual setting for testing only.


// put function declarations here:
void updateTapeError();
void steering();
void motorControl();

Servo steeringServo;	// might replace with manual PWM control. Test duty cycle to servo angle map with calibration program.

void setup() {
  pinMode(PIN_TAPE_SENSOR_LEFT, INPUT_PULLUP);
  pinMode(PIN_TAPE_SENSOR_RIGHT, INPUT_PULLUP);
  
  pinMode(PIN_LMOTOR_FWD, OUTPUT);
  pinMode(PIN_LMOTOR_REV, OUTPUT);
  pinMode(PIN_RMOTOR_FWD, OUTPUT);
  pinMode(PIN_RMOTOR_REV, OUTPUT);
  
  //pinMode(PIN_STEERING_SERVO, OUTPUT);
  steeringServo.attach(PIN_STEERING_SERVO);
  
  // for testing, add an assertions check to make sure all variables remain in range. (Basically implementing a rep invariant checker.)
  // default testing:
  analogWrite(PIN_LMOTOR_FWD, motorDutyCycle);
  analogWrite(PIN_RMOTOR_FWD, motorDutyCycle);
}

void loop() {
  tapeFollowing();
  delay(100);	// replace with clock absolute reference
}

/*
 * Tape following error determination from reflectance sensors
 * We read the voltage from BEFORE the phototransistor, so high voltage means low reflectance, and on tape
 * Negative error means we are to the left, and positive error to the right
 */
void updateTapeError() {
  int leftTapeSensorValue = analogRead(PIN_TAPE_SENSOR_LEFT);
  int rightTapeSensorValue = analogRead(PIN_TAPE_SENSOR_RIGHT);
  boolean leftOnTape = leftTapeSensorValue > TAPE_SENSOR_THRESHOLD;
  boolean rightOnTape = rightTapeSensorValue > TAPE_SENSOR_THRESHOLD;
  if (leftOnTape && rightOnTape) {
	tapePosErr = 0;  
  } else if (leftOnTape) {
	tapePosErr = -1;
  } else if (rightOnTape) {
	tapePosErr = 1;
  } else if (tapePosErrPrev) {
	tapePosErr = -2;
  } else {
	tapePosErr = 2;
  }
  /* Analog reading: if both sensors read something, fuzz steering angle to difference between sensor readings
  Lower threshold for sensors above which we consider difference
  Otherwise just the maximum angle
  */
  tapePosErrPrev = tapePosErr;
}

/*
 * Non-PID for now
 * Assume for now that we want to go at full speed all the time, no power differences
 */
void tapeFollowing() {
	// Adjust steering to avoid twitching on turns, or is what we have fine? (May use current steering position as reference, or maybe perform smoothing. Google?)
	int steering_angle = 10 * tapePosError;	// degrees
	int servo_pos = 90 + steering_angle;
	steeringServo.write(servo_pos);	
}

