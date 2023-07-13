#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// PIN DEFINITIONS
#define PIN_TAPE_SENSOR_LEFT PA4	// as analog voltage reading
#define PIN_TAPE_SENSOR_RIGHT PA5	// as analog voltage reading
#define PIN_CHECKPOINT_SENSOR_LEFT PA12	// as analog voltage reading
#define PIN_CHECKPOINT_SENSOR_RIGHT PA15	// as analog voltage reading
#define PIN_STEERING_SERVO PA_6	// as PWM servo control
// Originally PB6-9, but this interferes with LCD screen for debugging.
// Find out how to use SWV Trace (via pin PB3, TRACE SWO) to debug bluepill
#define PIN_LMOTOR_FWD PA_8
#define PIN_LMOTOR_REV PA_9
#define PIN_RMOTOR_FWD PA_10
#define PIN_RMOTOR_REV PA_11

/*
 * TAPE FOLLOWING
 */
const double WHEELBASE = 200;	// mm
const double WHEELSEP = 70;	// mm
const int TAPE_SENSOR_INTERSECTION_VALUE = 900;
const int TAPE_SENSOR_THRESHOLD = 500;
const int CHECKPOINT_SENSOR_THRESHOLD = 500;
const double STEERING_KP = 0.1;
const double STEERING_KD = 0.0;	// time derivative unit milliseconds 
const double POWER_SCALE = 1.00; // For testing only. Between 0 and 1
const int MOTOR_PWM_FREQ = 50;	// Shared with servos. 

int leftTapeSensorValue = 0;
int rightTapeSensorValue = 0;
int leftCheckpointSensorValue = 0;
int rightCheckpointSensorValue = 0;
bool leftOnTape = true;
bool rightOnTape = true;
bool prevLeftOnTape = true;
bool prevRightOnTape = false;
int prevError = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// put function declarations here:
void writeToDisplay(const char *str);
void updateTapeSensors();
void tapeFollowing();
void steeringControl(double steeringAngleDeg);
void motorControl(double lMotorPower, double rMotorPower);

void setup() {
	pinMode(PIN_TAPE_SENSOR_LEFT, INPUT);
	pinMode(PIN_TAPE_SENSOR_RIGHT, INPUT);
	pinMode(PIN_CHECKPOINT_SENSOR_LEFT, INPUT);
	pinMode(PIN_CHECKPOINT_SENSOR_RIGHT, INPUT);
	
	pinMode(PIN_LMOTOR_FWD, OUTPUT);
	pinMode(PIN_LMOTOR_REV, OUTPUT);
	pinMode(PIN_RMOTOR_FWD, OUTPUT);
	pinMode(PIN_RMOTOR_REV, OUTPUT);

	pinMode(PIN_STEERING_SERVO, OUTPUT);
	
  	// for testing, add an assertions check to make sure all variables remain in range. (Basically implementing a rep invariant checker.)
  	// default testing:
	display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display_handler.display();
	delay(2000);
	display_handler.clearDisplay();
	display_handler.setTextSize(1);
	display_handler.setTextColor(SSD1306_WHITE);

	writeToDisplay("The program has frozen just after setup!");
  	motorControl(POWER_SCALE, POWER_SCALE);
}

void loop() {
  	tapeFollowing();
  	return;
}

void writeToDisplay(const char *str) {
	display_handler.clearDisplay();
	display_handler.setCursor(0, 0);
	display_handler.println(str);
	display_handler.display();
}

/*
 * Tape following error determination from reflectance sensors
 * We read the voltage from BEFORE the phototransistor, so high voltage means low reflectance, and on tape
 * Negative error means we are to the left, and positive error to the right
 */
void updateTapeSensors() {
	leftTapeSensorValue = analogRead(PIN_TAPE_SENSOR_LEFT);
	rightTapeSensorValue = analogRead(PIN_TAPE_SENSOR_RIGHT);
	leftCheckpointSensorValue = analogRead(PIN_CHECKPOINT_SENSOR_LEFT);
	rightCheckpointSensorValue = analogRead(PIN_CHECKPOINT_SENSOR_RIGHT);
	
	leftOnTape = leftTapeSensorValue > TAPE_SENSOR_THRESHOLD;
	rightOnTape = rightTapeSensorValue > TAPE_SENSOR_THRESHOLD;
}

/*
 * Non-PID for now
 * Assume for now that we want to go at full speed all the time, no power differences
 */

void tapeFollowing() {
	const int LOOP_TIME_MILLIS = 60;
	int nextLoopTime = millis() + LOOP_TIME_MILLIS;
	// Adjust steering to avoid twitching on turns, or is what we have fine? (May use current steering position as reference, or maybe perform smoothing. Google?)
	while (true) {
		updateTapeSensors();
		bool leftOnTape = leftTapeSensorValue > TAPE_SENSOR_THRESHOLD;
		bool rightOnTape = rightTapeSensorValue > TAPE_SENSOR_THRESHOLD;
		
		double maxNormalSteeringAngle = 45.0;	// degrees
		double maxSteeringAngle = 45.0; // degrees
		
//		if (leftCheckpointSensorValue > CHECKPOINT_SENSOR_THRESHOLD || rightCheckpointSensorValue > CHECKPOINT_SENSOR_THRESHOLD) {
//			break;	// continue previous execution
//		}
		
		// Define error to right, right steering as +
		double steeringAngle;
		double leftMotorPower;
		double rightMotorPower;
		int error = 9999;
		if (leftOnTape && rightOnTape) {
			// define error as the deviation of the minimum sensor value from its value when centered on the tape
			// so positive error to right of tape
			// and 
			if (leftTapeSensorValue < rightTapeSensorValue) {	// to left of centerline
				error = min(0, leftTapeSensorValue - TAPE_SENSOR_INTERSECTION_VALUE);
			} else {	// to right of centerline
				error = max(0, TAPE_SENSOR_INTERSECTION_VALUE - rightTapeSensorValue);
			}
			// Difference method: error = leftTapeSensorValue - rightTapeSensorValue
			steeringAngle = STEERING_KP * error + STEERING_KD * (error - prevError) / LOOP_TIME_MILLIS;
			// Calculate appropriate differential by inferring turning circle from steering angle, wheelbase, and wheel width later on
			leftMotorPower = POWER_SCALE;
			rightMotorPower = POWER_SCALE;
		} else if (leftOnTape) {	// we are far to the right!
			steeringAngle = maxNormalSteeringAngle;
			leftMotorPower =  0.9 * POWER_SCALE;
			rightMotorPower = POWER_SCALE;
		} else if (rightOnTape) {	// we are far to the left!
			steeringAngle = - maxNormalSteeringAngle;
			leftMotorPower = POWER_SCALE;
			rightMotorPower = 0.9 * POWER_SCALE;
		} else {	// completely off tape. Refer to previous state and use differential steering.
			if (prevLeftOnTape) {	// we are completely off to the right!
				steeringAngle = maxSteeringAngle;
				leftMotorPower = 0;
				rightMotorPower = 0.8 * POWER_SCALE;
			} else {	// we are completely off to the left!
				steeringAngle = -maxSteeringAngle;
				leftMotorPower = 0;
				rightMotorPower = 0.8 * POWER_SCALE;
			}
		}
		steeringControl(steeringAngle);
		motorControl(leftMotorPower, rightMotorPower);
		if (leftOnTape || rightOnTape) {
			prevLeftOnTape = leftOnTape;
			prevRightOnTape = rightOnTape;
		}

		
		display_handler.clearDisplay();
		display_handler.setCursor(0, 0);
		display_handler.printf("Spare time: %d", nextLoopTime - millis());
		display_handler.printf("L: %4d  R: %4d\nError: %4d\n", leftTapeSensorValue, rightTapeSensorValue, error);
		display_handler.print("LMotor: ");
		display_handler.println(leftMotorPower, 3);
		display_handler.print("RMotor: ");
		display_handler.println(rightMotorPower, 3);
		display_handler.print("Steering angle: ");
		display_handler.println(steeringAngle, 1);
		display_handler.display();
		
		while (millis() < nextLoopTime);	// pause until next scheduled control loop
		nextLoopTime += LOOP_TIME_MILLIS;
	}
}

void steeringControl(double steeringAngleDeg) {
	// Assuming Ackerman steering geometry and right wheel direct drive. (Gear ratio???)
	if (abs(steeringAngleDeg) < 1.0) {
		pwm_start(PIN_STEERING_SERVO, 50, 1500, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
	}
	double turnRadius = WHEELBASE / tan(steeringAngleDeg * PI / 180);	// trigonometry methods are by default in radians, so convert to rad
	double rightWheelAngle = (atan(WHEELBASE / (turnRadius - WHEELSEP/2))) * 180 / PI;	// servo write is in degrees, so convert back to deg
	pwm_start(PIN_STEERING_SERVO, 50, 1500 + (int) ((1000/90.0) * rightWheelAngle), TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

// lMotorPower, rMotorPower between 0 and 1
// Add smoothing: don't change unless new motor powers are far enough from previous powers. (Greater dist -> shorter time). 
void motorControl(double lMotorPower, double rMotorPower) {	// replace with left, right; and refactor global variable with underscore
	if (lMotorPower > 0) {
		pwm_stop(PIN_LMOTOR_REV);
		pwm_start(PIN_LMOTOR_FWD, MOTOR_PWM_FREQ, (int) (1023 * lMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else if (lMotorPower < 0) {
		pwm_stop(PIN_LMOTOR_FWD);
		pwm_start(PIN_LMOTOR_REV, MOTOR_PWM_FREQ, (int) (1023 * lMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else {
		pwm_stop(PIN_LMOTOR_FWD);
		pwm_stop(PIN_LMOTOR_REV);
	}

	if (rMotorPower > 0) {
		pwm_stop(PIN_RMOTOR_REV);
		pwm_start(PIN_RMOTOR_FWD, MOTOR_PWM_FREQ, (int) (1023 * lMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else if (rMotorPower < 0) {
		pwm_stop(PIN_RMOTOR_FWD);
		pwm_start(PIN_RMOTOR_REV, MOTOR_PWM_FREQ, (int) (1023 * lMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else {
		pwm_stop(PIN_RMOTOR_FWD);
		pwm_stop(PIN_RMOTOR_REV);
	}
}
