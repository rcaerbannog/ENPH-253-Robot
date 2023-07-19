#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// PIN DEFINITIONS

// Motor pins originally PB6-9, but this interferes with LCD screen for debugging.
// Find out how to use SWV Trace (via pin PB3, TRACE SWO) to debug bluepill

#define PIN_LMOTOR_FWD PA_8	// as PWM output - H-bridge driver
#define PIN_LMOTOR_REV PA_9	// as PWM output - H-bridge driver
#define PIN_RMOTOR_FWD PA_10	// as PWM output - H-bridge driver
#define PIN_RMOTOR_REV PA_11	// as PWM output - H-bridge driver

#define PIN_LED_BUILTIN PC13	// DEBUG ONLY: USE TO INDICATE CONTROL LOOP PROGRESSION WITHOUT LCD DISPLAY

#define PIN_CHECKPOINT_SENSOR_LEFT PA2	// as analog voltage reading
#define PIN_CHECKPOINT_SENSOR_RIGHT PA1	// as analog voltage reading
#define PIN_STEERING_SERVO PA_6	// as PWM output - servo control
#define NUM_TAPE_SENSORS 4	// Rep invariant: MUST be the same as length of PINS_TAPE_SENSORS
const int PINS_TAPE_SENSORS[NUM_TAPE_SENSORS] = {PA2, PA3, PA4, PA5};	// as analog voltage readings

/*
 * TAPE FOLLOWING
 */
const double WHEELBASE = 200;	// In mm, Lengthwise distance between front and rear wheel axles
const double WHEELSEP = 200;	// In mm, Widthwise distance between front wheels
double POWER_SCALE = 0.20; // Power setting, scales all power sent to the motors between 0 and 1. (Ideally want this to be 1.)
const int MOTOR_PWM_FREQ = 50;	// In Hz, PWM frequency to H-bridge gate drivers. Currently shared with servos.
const double SERVO_NEUTRAL_PULSEWIDTH = 1550;	// In microseconds, default 1500 us. 
const int STEERING_SERVO_DIRECTION_SIGN = 1;	// Sign variable, +1 or -1. Switches servo direction in case the mounting direction is flipped.
const double MAX_STEERING_ANGLE_DEG = 40.0;

/*
Note: Motor PWM frequency cannot be too high, else gate driver turn-on time 
(~400us, inverted exponential decay voltage increase from high capacitance) significantly reduces motor power.
Voltage at source of top NMOS MOSFETS (and hence motor voltage) is limited for MOSFET to provide enough current
since gate voltage is lower than source voltage.
See scope image sent by Yun in 'general' channel for example on 1 kHz.
*/

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

double debugRightWheelAngle = 0;


// put function declarations here:

void writeToDisplay(const char *str);
void tapeFollowing();
double differentialFromSteering(double steeringAngleDeg);
void steeringControl(double steeringAngleDeg);
void motorControl(double lMotorPower, double rMotorPower);

void setup() {
	pinMode(PIN_LMOTOR_FWD, OUTPUT);
	pinMode(PIN_LMOTOR_REV, OUTPUT);
	pinMode(PIN_RMOTOR_FWD, OUTPUT);
	pinMode(PIN_RMOTOR_REV, OUTPUT);

	for (int pin : PINS_TAPE_SENSORS) pinMode(pin, INPUT);
	pinMode(PIN_CHECKPOINT_SENSOR_LEFT, INPUT);
	pinMode(PIN_CHECKPOINT_SENSOR_RIGHT, INPUT);
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
	try {
		tapeFollowing;
	} 
	catch(char *msg) {
		motorControl(0.0, 0.0);
		writeToDisplay(msg);
		delay(1000000);
	}
	catch(...) {
		motorControl(0.0, 0.0);
		writeToDisplay("Uncaught exception???");
		delay(1000000);
	}
  	
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
 * Non-PID for now
 * Assume for now that we want to go at full speed all the time, no power differences
 * 
 * Note: current LOOP_TIME_MILLIS is synchronized to multiple of servo and motor PWM period.
 * 
 * Note: this function can throw an exception with string message parameter to indicate invalid states.
 * (Those not currently handled in code.)
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

	Integral term MAY be useful, for smoothing or speed reduction. But it could also make us more sluggish and oscillatory.
	Maybe just keep ~10 control loops or so.

	*/

	const int LOOP_TIME_MILLIS = 20;	// Control loop period. Must be enough time for the code inside to execute!
	const double maxNormalSteeringAngleDeg = 30.0;	// degrees; for one sensor off tape
	const double maxSteeringAngleDeg = 30.0; // degrees; for both sensors off tape
	int nextLoopTime = millis() + LOOP_TIME_MILLIS;

	int tape_sensor_values[NUM_TAPE_SENSORS];
	bool on_tape[NUM_TAPE_SENSORS];
	int prevErrorDiscreteState = 0;	// 0, 1, 2, 3, or 4 sensors off tape; + to left, - to right
	int prevError = 0;	// from previous control loop, used for derivative control only if prevLeftOnTape and prevRightOnTape.
	double prevErrorDerivative = 0;	// from previous control loop, used for state recovery in case of checkpoint
	const int TAPE_SENSOR_FOUR_SETPOINT = 2000;
	const int TAPE_SENSOR_SETPOINT = 800;	// The analogRead() value when both tape sensors read the same (centered on tape)
	const int TAPE_SENSOR_THRESHOLD = 450;	// The analogRead() value above which we consider the tape sensor to be on tape
	// Make the checkpoint sensors deliberately less sensitive to light -> more sensitive to being off tape?
	const int CHECKPOINT_SENSOR_THRESHOLD = 175;	// The analogRead() value above which we consider the checkpoint sensor to be on tape
	const double STEERING_KP = 0.09;	// Steering angle PID proportionality constant
	const double STEERING_KD = 0.0;	// Steering angle PID derivative constant, time derivative unit milliseconds 

	// CONTROL LOOP
	while (true) {
		digitalWrite(PIN_LED_BUILTIN, HIGH);	// DEBUG ONLY, for monitoring control loop progression without LCD display
		int errorDiscreteState = 9;
		int error = 9999;	// Unitless; Positive error means to right of tape, negative to left
		double errorDerivative = 999.9;
		double steeringAngleDeg = 999.9; // In degrees; Positive angle means steering to left (CCW circling)
		double leftMotorPower = 1.0;	// Between -1 (full reverse) and 1 (full forwards); 0 is off
		double rightMotorPower = 1.0;	// Between -1 (full reverse) and 1 (full forwards); 0 is off

		bool rightOnCheckpoint = analogRead(PIN_CHECKPOINT_SENSOR_RIGHT > CHECKPOINT_SENSOR_THRESHOLD);
		bool leftOnCheckpoint = analogRead(PIN_CHECKPOINT_SENSOR_LEFT > CHECKPOINT_SENSOR_THRESHOLD);

		for (int i = 0; i < NUM_TAPE_SENSORS; i++) {
			int sensor = PINS_TAPE_SENSORS[i];
			tape_sensor_values[sensor] = analogRead(sensor);
			on_tape[sensor] = tape_sensor_values[i] > TAPE_SENSOR_THRESHOLD;
		}


		if (rightOnCheckpoint || leftOnCheckpoint) {	// freeze current state
			display_handler.print("On checkpoint!");
			display_handler.display();
			if (abs(prevErrorDiscreteState) < 4) {
				steeringControl(0.0);
			}	// inferred: otherwise, continue max steering
			while (millis() < nextLoopTime);	// pause until next scheduled control loop, to ensure consistent loop time
			nextLoopTime = millis() + LOOP_TIME_MILLIS;
			// Implied: else, if completely off tape then continue previous max steering
			// This also freezes the previous state
			// Add a waiting loop in here (separate state)?
		}

		// Determination of discrete error state (digital sensors or debug)
		if (!(on_tape[0] || on_tape[1] || on_tape[2] || on_tape[3])) {
			if (prevErrorDiscreteState > 0)	{
				errorDiscreteState = 4;
			}
			else if (prevErrorDiscreteState < 0) {
				errorDiscreteState = -4;
			} else {
				throw("INVALID STATE: went completely off tape from 0 error. Unstable!");
			}	// If we completely skipped over the tape line, then god help us we can't see that
			// Perhaps do similar open-loop control to checkpoint: keep scanning as often as possible until we return to the tape line
		} 
		else if (on_tape[0]) {
			if (on_tape[1]) {
				if (on_tape[2]) {
					if (on_tape[3]) {
						errorDiscreteState = 0;
					}
					errorDiscreteState = 1;	// right sensors going off tape: off to the right, so positive error state
				}
				errorDiscreteState = 2;
			}
			errorDiscreteState = 3;
		} 
		else if (on_tape[3]) {
			if (on_tape[2]) {
				if (on_tape[1]) {
					errorDiscreteState = -1;	// left sensors going off tape: off to left, so negative error state
				}
				errorDiscreteState = -2;
			}
			errorDiscreteState = -3;
		} else {
			throw("INVALID STATE: not detecting checkpoint, and we have discontinuously on tape sensors.");
		}
			
		
		if (abs(errorDiscreteState) > NUM_TAPE_SENSORS) {
			throw("Error: errorDiscreteState was set incorrectly");
		} else if (errorDiscreteState == NUM_TAPE_SENSORS) {	// we are completely off the tape to the right 
			steeringAngleDeg = maxSteeringAngleDeg;
			//leftMotorPower = differentialFromSteering(maxSteeringAngleDeg);
			leftMotorPower = 1.0;
			rightMotorPower = 1.0;
		} else if (errorDiscreteState == -NUM_TAPE_SENSORS) {	// we are completely off the tape to the left
			steeringAngleDeg = -maxSteeringAngleDeg;
			leftMotorPower = 1.0;
			//rightMotorPower = differentialFromSteering(-maxSteeringAngleDeg);
			rightMotorPower = 1.0;
		} else if (abs(prevErrorDiscreteState) == NUM_TAPE_SENSORS) {	// we were previously off the tape and have just come back on
			// Set steering straight to stabilize for one control loop
			steeringAngleDeg = 0;
			leftMotorPower = 1.0;
			rightMotorPower = 1.0;
			// Calculate present error, and assume derivative with sign depending on direction of approach
			error = TAPE_SENSOR_FOUR_SETPOINT;
			for (int sensor = 0; sensor < NUM_TAPE_SENSORS; sensor++) {
				error -= tape_sensor_values[sensor];	// absolute value of error increases further from the tape
			}
			error = (errorDiscreteState > 0) ? error : -error;	// positive error if off to right, negative to left
			// since we are coming back onto the tape line, our derivative is opposite the direction of previous error
			errorDerivative = (prevErrorDiscreteState > 0) ? -1.0 : 1.0;	
		} else {	// Just plain error control
			error = TAPE_SENSOR_FOUR_SETPOINT;
			for (int sensor = 0; sensor < NUM_TAPE_SENSORS; sensor++) {
				error -= tape_sensor_values[sensor];	// absolute value of error increases further from the tape
			}
			error = (errorDiscreteState > 0) ? error : -error;	// positive error if off to right, negative to left
			errorDerivative = (error - prevError) / (double) LOOP_TIME_MILLIS;

			steeringAngleDeg = STEERING_KP * error + STEERING_KD * errorDerivative;
			steeringAngleDeg = max(-maxNormalSteeringAngleDeg, min(maxNormalSteeringAngleDeg, steeringAngleDeg));	// bound
			leftMotorPower = 1.0;
			rightMotorPower = 1.0;
		}

		steeringControl(steeringAngleDeg);
		motorControl(leftMotorPower, rightMotorPower);

		prevErrorDiscreteState = errorDiscreteState;
		prevError = error;
		prevErrorDerivative = errorDerivative;

		digitalWrite(PIN_LED_BUILTIN, LOW);

		// COMMENT OUT DEBUG DISPLAY CODE IF THERE IS NO DISPLAY, OTHERWISE EXECUTION WILL STALL
		// WHILE TRYING TO WRITE TO A NON-EXISTENT DISPLAY (UNTIL REQUEST TIMEOUT AFTER ~5 SECONDS)
		debugDisplay(tape_sensor_values, errorDiscreteState, error, errorDerivative, 
				leftMotorPower, rightMotorPower, steeringAngleDeg, debugRightWheelAngle);
		
		while (millis() < nextLoopTime);	// pause until next scheduled control loop, to ensure consistent loop time
		nextLoopTime = millis() + LOOP_TIME_MILLIS;
	}
}

void debugDisplay(int tape_sensor_values[], int state, int error, int derivative, 
		int leftMotorPower, int rightMotorPower, int steeringAngleDeg, int rightWheelAngle) {
	display_handler.clearDisplay();
	display_handler.setCursor(0, 0);
	display_handler.printf("%4d %4d %4d %4d\n", tape_sensor_values[0], tape_sensor_values[1], tape_sensor_values[2], tape_sensor_values[3]);
	display_handler.printf("St %1d Err %4d D %4d", state, error, derivative);
	display_handler.print("LMotor: ");
	display_handler.println(leftMotorPower, 3);
	display_handler.print("RMotor: ");
	display_handler.println(rightMotorPower, 3);
	display_handler.print("Steer ");
	display_handler.print(steeringAngleDeg, 1);
	display_handler.print(" R ");
	display_handler.println(rightWheelAngle, 1);
	display_handler.display();
}

/*
 * Calculates speed differential for back driven wheels when turning. SPEED, NOT POWER. 
 * This is probably useless for small steering angles / fast oscillation due to system inertia.
 * ONLY use for large speed differential.
 *
 * @param steeringAngleDeg The 'ideal steering angle' TO THE LEFT (CCW) from an imaginary front wheel on the chassis centerline.
 * @return The differential speed ratio of the innter (slower) back wheel to the outer (faster) wheel for this steering angle. 
 * 			Between -1 and 1: 1 for same direction and -1 for opposite direction
 * 			Infer which wheel is inner one from sign of passed in parameter steeringAngleDeg 
*/
double differentialFromSteering(double steeringAngleDeg) {
	if (abs(steeringAngleDeg) < 1.0) {
		return 1;
	}
	double turnRadius = abs(WHEELBASE / tan(steeringAngleDeg * PI / 180));
	double differential = (turnRadius - WHEELSEP / 2) / (turnRadius + WHEELSEP / 2);
	return differential;
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
	steeringAngleDeg = max(-MAX_STEERING_ANGLE_DEG, (MAX_STEERING_ANGLE_DEG, steeringAngleDeg));	// bound by physical steering limit
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
