#include <Arduino.h>
#include <Servo.h>

// PIN DEFINITIONS
#define PIN_TAPE_SENSOR_LEFT PA4	// as analog voltage reading
#define PIN_TAPE_SENSOR_RIGHT PA5	// as analog voltage reading
#define PIN_CHECKPOINT_SENSOR_LEFT PA12	// as analog voltage reading
#define PIN_CHECKPOINT_SENSOR_RIGHT PA15	// as analog voltage reading
#define PIN_STEERING_SERVO PA11	// as PWM servo control
#define PIN_LMOTOR_FWD PB6
#define PIN_LMOTOR_REV PB7
#define PIN_RMOTOR_FWD PB8
#define PIN_RMOTOR_REV PB9



// ENUMS


// put function declarations here:
void updateTapeError();
void steering();
void motorControl();

Servo steeringServo;	// might replace with manual PWM control. Test duty cycle to servo angle map with calibration program.

void setup() {
  pinMode(PIN_TAPE_SENSOR_LEFT, INPUT);
  pinMode(PIN_TAPE_SENSOR_RIGHT, INPUT);
  pinMode(PIN_CHECKPOINT_SENSOR_LEFT, INPUT);
  pinMode(PIN_CHECKPOINT_SENSOR_RIGHT, INPUT);
  
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
 * TAPE FOLLOWING
 */
 
const int TAPE_SENSOR_THRESHOLD = 500;
const int CHECKPOINT_SENSOR_THRESHOLD = 500;

const double STEERING_KP = 1.0;
const double MOTOR_POWER_KP = 1.0;

int leftTapeSensorValue = 0;
int rightTapeSensorValue = 0;
int leftCheckpointSensorValue = 0;
int rightCheckpointSensorValue = 0;

boolean leftOnTape = true;
boolean rightOnTape = true;
boolean prevLeftOnTape = true;
boolean prevRightOnTape = false;

int motorDefaultPower = 255; 	// duty cycle fraction out of 255. Manual setting for testing only.

/*
 * Tape following error determination from reflectance sensors
 * We read the voltage from BEFORE the phototransistor, so high voltage means low reflectance, and on tape
 * Negative error means we are to the left, and positive error to the right
 */
void updateTapeSensors() {
  int leftTapeSensorValue = analogRead(PIN_TAPE_SENSOR_LEFT);
  int rightTapeSensorValue = analogRead(PIN_TAPE_SENSOR_RIGHT);
  int leftCheckpointSensorValue = analogRead(PIN_CHECKPOINT_SENSOR_LEFT);
  int rightCheckpointSensorValue = analogRead(PIN_CHECKPOINT_SENSOR_RIGHT);
  
  leftOnTape = leftTapeSensorValue > TAPE_SENSOR_THRESHOLD;
  rightOnTape = rightTapeSensorValue > TAPE_SENSOR_THRESHOLD;
}

/*
 * Non-PID for now
 * Assume for now that we want to go at full speed all the time, no power differences
 */

void tapeFollowing() {
	int INTERSECTION_SENSOR_VALUE = 900;	// out of 1024, set this to the value where both sensors are the same (define as exact middle of tape)
	// Adjust steering to avoid twitching on turns, or is what we have fine? (May use current steering position as reference, or maybe perform smoothing. Google?)
	while (true) {
		updateTapeSensors();
		bool leftOnTape = leftTapeSensorValue > TAPE_SENSOR_THRESHOLD;
		bool rightOnTape = rightTapeSensorValue > TAPE_SENSOR_THRESHOLD;
		
		double maxNormalSteeringAngle = 45;	// degrees
		double maxSteeringAngle = 45; // degrees
		double offTapeSpeed = 128;	// out of 255
		
		if (leftCheckpointSensorValue > CHECKPOINT_SENSOR_THRESHOLD || rightCheckpointSensorValue > CHECKPOINT_SENSOR_THRESHOLD) {
			break;	// continue previous execution
		}
		
		// Define error to right, right steering as +
		double steeringAngle;
		int leftMotorPower;
		int rightMotorPower;
		if (leftOnTape && rightOnTape) {
			// define error as the deviation of the minimum sensor value from its value when centered on the tape
			int error;
			if (leftTapeSensorValue < rightTapeSensorValue) {
				error = max(0, INTERSECTION_SENSOR_VALUE - leftTapeSensorValue);
			} else {
				error = min(0, rightTapeSensorValue - INTERSECTION_SENSOR_VALUE);
			}
			steeringAngle = - STEERING_KP * error;
			// Calculate appropriate differential by inferring turning circle from steering angle, wheelbase, and wheel width
			leftMotorPower = motorDefaultPower;
			rightMotorPower = motorDefaultPower;
		} else if (leftOnTape) {	// we are far to the right!
			steeringAngle = - maxNormalSteeringAngle;
			leftMotorPower = (int) (0.8 * motorDefaultPower);
			rightMotorPower = motorDefaultPower;
		} else if (rightOnTape) {	// we are far to the left!
			steeringAngle = maxNormalSteeringAngle;
			leftMotorPower = motorDefaultPower;
			rightMotorPower = (int) (0.8 * motorDefaultPower);
		} else {	// completely off tape. Refer to previous state and use differential steering.
			// Write a motor to 0 and continue detection
			if (prevLeftOnTape) {
				steeringAngle = - maxSteeringAngle;
				leftMotorPower = 0;
				rightMotorPower = (0.8 * motorDefaultPower);
			} else {
				steeringAngle = maxSteeringAngle;
				leftMotorPower = 150;
				rightMotorPower = (0.8 * motorDefaultPower);
			}
			break;
		}
		
		int servo_pos = 90 + 2 * steering_angle;
		steeringServo.write(servo_pos);	
		// Write duty cycle to appropriate H-bridge
		motorControl(lMotorPower, rMotorPower);
		prevLeftOnTape = leftOnTape;
		prevRightOnTape = rightOnTape;
	}
}

// Add smoothing: don't change unless new motor powers are far enough from previous powers. (Greater dist -> shorter time). 
void motorControl(int lMotorPower, int rMotorPower) {	// replace with left, right; and refactor global variable with underscore
	if (lMotorPower > 0) {
		analogWrite(PIN_LMOTOR_REV, 0);
		analogWrite(PIN_LMOTOR_FWD, lMotorPower);
	} else if (lMotorPower < 0) {
		analogWrite(PIN_LMOTOR_FWD, 0);
		analogWrite(PIN_LMOTOR_REV, -lMotorPower);
	} else {
		analogWrite(PIN_LMOTOR_FWD, 0);
		analogWrite(PIN_LMOTOR_REV, 0);
	}

	if (rMotorPower > 0) {
		analogWrite(PIN_RMOTOR_REV, 0);
		analogWrite(PIN_RMOTOR_FWD, lMotorPower);
	} else if (lMotorPower < 0) {
		analogWrite(PIN_RMOTOR_FWD, 0);
		analogWrite(PIN_RMOTOR_REV, -lMotorPower);
	} else {
		analogWrite(PIN_RMOTOR_FWD, 0);
		analogWrite(PIN_RMOTOR_REV, 0);
	}		
}