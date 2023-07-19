#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// PIN DEFINITIONS
#define PIN_TAPE_SENSOR_LL PA2
#define PIN_TAPE_SENSOR_RR PA3
#define PIN_TAPE_SENSOR_LEFT PA4	// as analog voltage reading
#define PIN_TAPE_SENSOR_RIGHT PA5	// as analog voltage reading
#define PIN_CHECKPOINT_SENSOR_LEFT PA2	// as analog voltage reading
#define PIN_CHECKPOINT_SENSOR_RIGHT PA1	// as analog voltage reading
#define PIN_STEERING_SERVO PA_6	// as PWM output - servo control

// Motor pins originally PB6-9, but this interferes with LCD screen for debugging.
// Find out how to use SWV Trace (via pin PB3, TRACE SWO) to debug bluepill

#define PIN_LMOTOR_FWD PA_8	// as PWM output - H-bridge driver
#define PIN_LMOTOR_REV PA_9	// as PWM output - H-bridge driver
#define PIN_RMOTOR_FWD PA_10	// as PWM output - H-bridge driver
#define PIN_RMOTOR_REV PA_11	// as PWM output - H-bridge driver

#define PIN_LED_BUILTIN PC13	// DEBUG ONLY: USE TO INDICATE CONTROL LOOP PROGRESSION WITHOUT LCD DISPLAY

#define UNDEFINED 999999999

/*
 * TAPE FOLLOWING
 */
const double WHEELBASE = 200;	// In mm, Lengthwise distance between front and rear wheel axles
const double WHEELSEP = 200;	// In mm, Widthwise distance between front wheels
const int TAPE_SENSOR_FOUR_SETPOINT = 2000;
const int TAPE_SENSOR_SETPOINT = 650;	// The analogRead() value when both tape sensors read the same (centered on tape)
const int TAPE_SENSOR_THRESHOLD = 100;	// The analogRead() value above which we consider the tape sensor to be on tape
// Make the checkpoint sensors deliberately less sensitive to light -> more sensitive to being off tape?
const int CHECKPOINT_SENSOR_THRESHOLD = 175;	// The analogRead() value above which we consider the checkpoint sensor to be on tape
const double STEERING_KP = 0.05;	// Steering angle PID proportionality constant
const double STEERING_KD = 0.0;	// Steering angle PID derivative constant, time derivative unit milliseconds 
double POWER_SCALE = 0.20; // Power setting, scales all power sent to the motors between 0 and 1. (Ideally want this to be 1.)
const int MOTOR_PWM_FREQ = 50;	// In Hz, PWM frequency to H-bridge gate drivers. Currently shared with servos.
const double SERVO_NEUTRAL_PULSEWIDTH = 1550;	// In microseconds, default 1500 us. 
const int STEERING_SERVO_DIRECTION_SIGN = 1;	// Sign variable, +1 or -1. Switches servo direction in case the mounting direction is flipped.

/*
Note: Motor PWM frequency cannot be too high, else gate driver turn-on time 
(~400us, inverted exponential decay voltage increase from high capacitance) significantly reduces motor power.
Voltage at source of top NMOS MOSFETS (and hence motor voltage) is limited for MOSFET to provide enough current
since gate voltage is lower than source voltage.
See scope image sent by Yun in 'general' channel for example on 1 kHz.
*/

int leftTapeSensorValue = 0;	// by analogRead(); lower is further from tape
int rightTapeSensorValue = 0;	// by analogRead(); lower is further from tape
int leftCheckpointSensorValue = 0; // by analogRead(); lower is further from tape
int rightCheckpointSensorValue = 0; // by analogRead(); lower is further from tape
bool leftOnTape = true; // leftTapeSensorValue above TAPE_SENSOR_THRESHOLD
bool rightOnTape = true; // rightTapeSensorValue above TAPE_SENSOR_THRESHOLD
bool prevLeftOnTape = true;	// in case of leaving tape entirely
bool prevRightOnTape = true;	// in case of leaving tape entirely
bool onCheckpoint = false;	// leftCheckPointSensorValue or rightCheckpointSensorValue above CHECKPOINT_SENSOR_THRESHOLD
int prevError = 0;	// from previous control loop, used for derivative control only if prevLeftOnTape and prevRightOnTape.
double prevDerivativeError = 0;	// from previous control loop, used for state recovery in case of checkpoint

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

double debugRightWheelAngle = 0;


// put function declarations here:

void writeToDisplay(const char *str);
void updateTapeSensors();
void tapeFollowing();
double differentialFromSteering(double steeringAngleDeg);
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

	pinMode(PIN_LED_BUILTIN, OUTPUT);
	
  	// for testing, add an assertions check to make sure all variables remain in range. (Basically implementing a rep invariant checker.)
  	// default testing:
	display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display_handler.display();
	delay(2000);
	display_handler.clearDisplay();
	display_handler.setTextSize(1);
	display_handler.setTextColor(SSD1306_WHITE);

	writeToDisplay("Either there is no display code after startup, or the program froze before completing a control loop.");
}

void loop() {
  	tapeFollowing();
  	return;
}

/*
 * Clears display and writes a string. FOR DEBUG!
 * @param str The string to show on the display.
 */ 
void writeToDisplay(const char *str) {
	display_handler.clearDisplay();
	display_handler.setCursor(0, 0);
	display_handler.println(str);
	display_handler.display();
}

/*
 * Update analog readings from tape-following reflectance sensors.
 * We read the voltage from BEFORE the phototransistor, so high voltage means low reflectance, and on tape.
 */
void updateTapeSensors() {
	leftTapeSensorValue = analogRead(PIN_TAPE_SENSOR_LEFT);
	rightTapeSensorValue = analogRead(PIN_TAPE_SENSOR_RIGHT);
	leftCheckpointSensorValue = analogRead(PIN_CHECKPOINT_SENSOR_LEFT);
	rightCheckpointSensorValue = analogRead(PIN_CHECKPOINT_SENSOR_RIGHT);
}

/*
 * Non-PID for now
 * Assume for now that we want to go at full speed all the time, no power differences
 * 
 * Note: current LOOP_TIME_MILLIS is synchronized to multiple of servo and motor PWM period.
 */
void tapeFollowing() {
	/* 
	We assume the checkpoint sensors can see the tape just before and after the main sensors.
	Consider possibility of checkpoint if left or right checkpoint sensor above threshold.
	Then freeze current AND previous state, as this will otherwise be destroyed going over the checkpoint.
	(Rainbow road doesn't change reflectance at current sensitivity, so no false activation there.)
	CASES
	Case 1: Only a checkpoint sensor sees tape. Then continue locking max steering angle.
		We are certainly still off-tape in the same direction. So this is the correct response anyways.
		Includes Case 1a: Both checkpoint sensors see tape. 
	Case 2: A checkpoint sensor sees tape AND so does a line sensor, then lock the steering straight and hope this passes.
		If we were on tape before, then we are probably still
		If we were off tape, then the tape sensors may be seeing the line or checkpoint (or intersection of cross).
		If checkpoint, just lock steering for a bit.  
	Case 3: Only a line sensor sees tape. Then steer using normal PID control.
		Maybe we just got spooked a little bit. This will soon pass.

	If was on tape and went off, maybe recover state by derivative of error wrt time?

	*/

	const int LOOP_TIME_MILLIS = 20;	// Control loop period. Must be enough time for the code inside to execute!
	const double maxNormalSteeringAngleDeg = 30.0;	// degrees; for one sensor off tape
	const double maxSteeringAngleDeg = 30.0; // degrees; for both sensors off tape
	int nextLoopTime = millis() + LOOP_TIME_MILLIS;

	// CONTROL LOOP
	while (true) {
		digitalWrite(PIN_LED_BUILTIN, HIGH);	// DEBUG ONLY, for monitoring control loop progression without LCD display
		updateTapeSensors();
		leftOnTape = leftTapeSensorValue > TAPE_SENSOR_THRESHOLD;
		rightOnTape = rightTapeSensorValue > TAPE_SENSOR_THRESHOLD;

		//if (leftCheckpointSensorValue > CHECKPOINT_SENSOR_THRESHOLD || rightCheckpointSensorValue > CHECKPOINT_SENSOR_THRESHOLD) {
		//	break;	// continue previous execution
		//}
		
		int error = UNDEFINED;	// Unitless; Positive error means to right of tape, negative to left
		double dError = 0;
		double steeringAngleDeg; // In degrees; Positive angle means steering to left (CCW circling)
		double leftMotorPower;	// Between -1 (full reverse) and 1 (full forwards); 0 is off
		double rightMotorPower;	// Between -1 (full reverse) and 1 (full forwards); 0 is off
		
		if (onCheckpoint) {
			// may have to invalidate derivative of next loop, but probably not necessary
			// checkpoint recovery in case of on tape -> completely off tape? (hopefully impossible)
			nextLoopTime = millis();
			continue;	// hold previous course of action. Change this if it causes problems to case-specific decision making.
		}

		if (leftOnTape && rightOnTape) {
			// Use PID from reflectance sensor-determined error
			// ERROR IS DEFINED HERE! Can replace with different method of calculating error. (e.g. difference, or more sensors)
			if (leftTapeSensorValue < rightTapeSensorValue) {	// to left of centerline
				error = min(0, leftTapeSensorValue - TAPE_SENSOR_SETPOINT);
			} else {	// to right of centerline
				error = max(0, TAPE_SENSOR_SETPOINT - rightTapeSensorValue);
			}
			// Difference method: error = leftTapeSensorValue - rightTapeSensorValue
			// Only use derivative term if previous error is not UNDEFINED (both sensors were on tape)
			if (prevLeftOnTape && prevRightOnTape) {
				dError = (error - prevError) / (double) LOOP_TIME_MILLIS;
				steeringAngleDeg = STEERING_KP * error + STEERING_KD * dError;
			} else {
				steeringAngleDeg = STEERING_KP * error;
			}
			
			// Calculate appropriate differential by inferring turning circle from steering angle, wheelbase, and wheel width later on
			if (steeringAngleDeg >= 0) {	// steering to left/CCW, left wheel is inner wheel
				steeringAngleDeg = min(steeringAngleDeg, maxNormalSteeringAngleDeg);
				leftMotorPower = differentialFromSteering(steeringAngleDeg);
				rightMotorPower = 1.0;
			} else {
				steeringAngleDeg = max(steeringAngleDeg, -maxNormalSteeringAngleDeg);
				leftMotorPower = 1.0;
				rightMotorPower = differentialFromSteering(steeringAngleDeg);
			}
		} else if (leftOnTape) {	// We are far to the right, and want to turn left! The right sensor is off the tape.
			steeringAngleDeg = maxNormalSteeringAngleDeg;
			leftMotorPower =  0.9 * differentialFromSteering(steeringAngleDeg);
			rightMotorPower = 0.9;
		} else if (rightOnTape) {	// we are far to the left, and want to turn right! The left sensor is off the tape.
			steeringAngleDeg = - maxNormalSteeringAngleDeg;
			leftMotorPower = 0.9;
			rightMotorPower = 0.9 * differentialFromSteering(steeringAngleDeg);
		} else {	// Both sensors are completely off the tape! Refer to previous state and use differential steering.
			if (prevLeftOnTape) {	// We went completely off to the right! The left sensor was the last to come off the tape.
				steeringAngleDeg = maxSteeringAngleDeg;
				leftMotorPower = 0.9 * differentialFromSteering(steeringAngleDeg);
				rightMotorPower = 0.9;
			} else {	// We went completely off to the left! The right sensor was the last to come off the tape.
				steeringAngleDeg = -maxSteeringAngleDeg;
				leftMotorPower = 0.9;
				rightMotorPower = 0.9 * differentialFromSteering(steeringAngleDeg);
			}
		}

		steeringControl(steeringAngleDeg);
		motorControl(leftMotorPower, rightMotorPower);

		if (!onCheckpoint && (leftOnTape || rightOnTape)) {	// Update previous state in case we go off tape
			prevLeftOnTape = leftOnTape;
			prevRightOnTape = rightOnTape;
			prevError = error;
			prevDerivativeError = dError;
		}

		digitalWrite(PIN_LED_BUILTIN, LOW);
		// COMMENT OUT DEBUG DISPLAY CODE IF THERE IS NO DISPLAY, OTHERWISE EXECUTION WILL STALL
		// WHILE TRYING TO WRITE TO A NON-EXISTENT DISPLAY (UNTIL REQUEST TIMEOUT AFTER ~5 SECONDS)
		
		//display_handler.printf("Loop time: %d\n", nextLoopTime-millis());
		display_handler.clearDisplay();
		display_handler.setCursor(0, 0);
		display_handler.print("Checkpoint: ");
		display_handler.println(onCheckpoint);
		display_handler.printf("L: %4d  R: %4d\nError: %4d\n", leftTapeSensorValue, rightTapeSensorValue, error);
		display_handler.print("LMotor: ");
		display_handler.println(leftMotorPower, 3);
		display_handler.print("RMotor: ");
		display_handler.println(rightMotorPower, 3);
		display_handler.print("Steering angle: ");
		display_handler.println(steeringAngleDeg, 1);
		display_handler.print("R steer angle: ");
		display_handler.print(debugRightWheelAngle, 1);
		display_handler.display();
		
		while (millis() < nextLoopTime);	// pause until next scheduled control loop, to ensure consistent loop time
		nextLoopTime = millis() + LOOP_TIME_MILLIS;
	}
}

/*
 * @param steeringAngleDeg The 'ideal steering angle' TO THE LEFT (CCW) from an imaginary front wheel on the chassis centerline.
 * @return The differential speed ratio of the innter (slower) back wheel to the outer (faster) wheel for this steering angle. 
 * 			Between -1 and 1: 1 for same direction and -1 for opposite direction
 * 			Infer which wheel is inner one from sign of passed in parameter steeringAngleDeg 
*/
double differentialFromSteering(double steeringAngleDeg) {
	if (abs(steeringAngleDeg) < 1.0) {
		return 0;
	}
	double turnRadius = abs(WHEELBASE / tan(steeringAngleDeg * PI / 180));
	double differential = (turnRadius - WHEELSEP / 2) / (turnRadius + WHEELSEP / 2);
	//return differential;
	return 1;
}

/*
 * Commands the servo to move the steering system to the ideal steering angle.
 * 
 * ASSUMES ideal Ackerman geometry and correctly specified wheelbase and front wheel separation. 
 * 
 * ASSUMES direct drive or 1:1 gear ratio drive of right wheel steering axle by servo.
 * 
 * REQUIRES steering servo to be inserted at position corresponding to set SERVO_NEUTRAL_PULSEWIDTH!!!
 * Do this by sending an AD2 pulse at 50 Hz, 3.3V with this pulse width when inserting the servo.
 * Can adjust slightly from default of 1500us to make gears mesh, etc. (But try to avoid this.)
 * 
 * REQUIRES as little backlash in the steering system as possible! (This means tight fits on axles and gears.)
 * 
 * @param steeringAngleDeg The 'ideal steering angle' TO THE LEFT (CCW) from an imaginary front wheel on the chassis centerline.
*/
void steeringControl(double steeringAngleDeg) {
	double rightWheelAngle = 0;
	if (abs(steeringAngleDeg) < 1.0) {
		pwm_start(PIN_STEERING_SERVO, 50, SERVO_NEUTRAL_PULSEWIDTH, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
	}
	double turnRadius = WHEELBASE / tan(steeringAngleDeg * PI / 180);	// trigonometry methods are by default in radians, so convert to rad
	rightWheelAngle = (atan(WHEELBASE / (turnRadius + WHEELSEP/2))) * 180 / PI;	// servo write is in degrees, so convert back to deg	
	// Servo response is linear with 90 degrees rotation to 1000us pulse width (empirical testing of MG90, MG996R)
	pwm_start(PIN_STEERING_SERVO, 50, SERVO_NEUTRAL_PULSEWIDTH + (int) (STEERING_SERVO_DIRECTION_SIGN * (1000/90.0) * rightWheelAngle), 
		TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

	debugRightWheelAngle = rightWheelAngle;
}

/*
 * Sends power to the motor by modulating power through the H-bridges. Scales power by global constant POWER_SCALE.
 * (Current implementation: via the gate drivers.)
 * 
 * IDEAS FOR IMPROVEMENT:
 * Add smoothing: don't change unless new motor powers are far enough from previous powers. (Greater dist -> shorter time). 
 * 	Not necessary if PWM period synchronized with control loop period -- integer multiple?
 * Add initial acceleration/braking 'jerk' when speeding motor from stall or if needing fast motor braking.
 * 	Active braking may inject excessive noise by back-EMF? 
 * 	Acceleration jerk to full power for ~20-100ms to get moving and overcome static friction? 
 * 
 * @param lMotorPower The power to send to the left motor, signed for direction. Between -1 (full reverse) and 1 (full forwards), with 0 being off.
 * @param rMotorPower Same for the right motor, between -1 and 1.
 */
void motorControl(double lMotorPower, double rMotorPower) {
	// if (lMotorPower == 0.0) {
	// 	lMotorPower = 0.01;
	// }
	// if (rMotorPower == 0.0) {
	// 	rMotorPower = 0.01;
	// }
	if (lMotorPower > 0) {	// forwards
		pwm_start(PIN_LMOTOR_REV, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_LMOTOR_FWD, MOTOR_PWM_FREQ, (int) (1023 * POWER_SCALE * lMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else if (lMotorPower < 0) {	// reverse
		pwm_start(PIN_LMOTOR_FWD, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_LMOTOR_REV, MOTOR_PWM_FREQ, (int) (1023 * POWER_SCALE * lMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else {	// unpowered / stop
		pwm_start(PIN_LMOTOR_FWD, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_LMOTOR_REV, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
	}

	if (rMotorPower > 0) {	// forwards
		pwm_start(PIN_RMOTOR_REV, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_RMOTOR_FWD, MOTOR_PWM_FREQ, (int) (1023 * POWER_SCALE * rMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else if (rMotorPower < 0) {	// reverse
		pwm_start(PIN_RMOTOR_FWD, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_RMOTOR_REV, MOTOR_PWM_FREQ, (int) (1023 * POWER_SCALE * rMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else {	// unpowered / stop
		pwm_start(PIN_RMOTOR_FWD, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_RMOTOR_REV, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
	}
}
