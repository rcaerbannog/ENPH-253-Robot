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

#define PIN_HALL_SENSOR PB13	// as digital input
#define PIN_BLOCKMOTOR_FWD PB14	// as digital output (non-PWM, so max speed)
#define PIN_BLOCKMOTOR_REV PB15	// as digital output (non-PWM, so max speed)

#define PIN_LED_BUILTIN PC13	// DEBUG ONLY: USE TO INDICATE CONTROL LOOP PROGRESSION WITHOUT LCD DISPLAY

//#define PIN_CHECKPOINT_SENSOR_LEFT PA1	// as analog input
//#define PIN_CHECKPOINT_SENSOR_RIGHT PA0	// as analog input
#define PIN_STEERING_SERVO PA_6	// as PWM output - servo control
#define NUM_TAPE_SENSORS 6	// Rep invariant: MUST be the same as length of PINS_TAPE_SENSORS
const int PINS_TAPE_SENSORS[NUM_TAPE_SENSORS] = {PA5, PA4, PA3, PA2, PA1, PA0};	// as analog input. In order from left to right sensors.

/*
 * TAPE FOLLOWING
 */
const double WHEELBASE = 125;	// In mm, Lengthwise distance between front and rear wheel axles
const double WHEELSEP = 200;	// In mm, Widthwise distance between front wheels
double POWER_SCALE = 0.50; // Power setting, scales all power sent to the motors between 0 and 1. (Ideally want this to be 1.)
const int MOTOR_PWM_FREQ = 50;	// In Hz, PWM frequency to H-bridge gate drivers. Currently shared with servos.
const double SERVO_NEUTRAL_PULSEWIDTH = 1500;	// In microseconds, default 1500 us. 
const int STEERING_SERVO_DIRECTION_SIGN = 1;	// Sign variable, +1 or -1. Switches servo direction in case the mounting direction is flipped.
const int MAX_STEERING_PULSEWIDTH_MICROS = 2000;	// absolute physical limit of left-driving servo rotation to left. Currently limited by chassis.
const int MIN_STEERING_PULSEWIDTH_MICROS = 1200;	// absolute physical limit of left-driving servo rotation to right. Currently limited by inversion.
const int BOMB_EJECTION_TIME_MILLIS = 1000;
int bombEjectionEndTime = 0;
bool bombEject = false;

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

double debugLeftWheelAngle = 0;


// put function declarations here:
void interruptBombEjection();
void writeToDisplay(const char *str);
double errorFunc(int tape_sensor_vals[], int TAPE_SENSOR_THRESHOLD);
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

	//pinMode(PIN_CHECKPOINT_SENSOR_LEFT, INPUT);
	//pinMode(PIN_CHECKPOINT_SENSOR_RIGHT, INPUT);
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

	// set up block collection system. Hall sensor looks Schmitt triggered (switch time ~1us) and sees no bouncing
	pinMode(PIN_HALL_SENSOR, INPUT);	// using our own resistor instead of Bluepill internal pullup resistor
	pinMode(PIN_BLOCKMOTOR_FWD, OUTPUT);
	pinMode(PIN_BLOCKMOTOR_REV, OUTPUT);
	digitalWrite(PIN_BLOCKMOTOR_REV, LOW);
	digitalWrite(PIN_BLOCKMOTOR_FWD, HIGH);
	bombEject = false;
	attachInterrupt(PIN_HALL_SENSOR, interruptBombEjection, FALLING);	// With pullup resistor, Hall sensor goes low when in strong magnetic field
}

void loop() {
	/*
	try {
		tapeFollowing();
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
  	*/
  	tapeFollowing();
  	return;
}


void interruptBombEjection() {
	digitalWrite(PIN_BLOCKMOTOR_FWD, LOW);
	digitalWrite(PIN_BLOCKMOTOR_REV, HIGH);
	bombEject = true;
	bombEjectionEndTime = millis() + BOMB_EJECTION_TIME_MILLIS;
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
	Then freeze previous state, as this will otherwise be destroyed going over the checkpoint.
	(Rainbow road doesn't change reflectance at current sensitivity, so no false activation there.)

	If was on tape and went off, maybe recover state by derivative of error wrt time?

	Integral term MAY be useful, for smoothing or speed reduction. But it could also make us more sluggish and oscillatory.
	Maybe just keep ~10 control loops or so.

	*/

	const int LOOP_TIME_MILLIS = 20;	// Control loop period. Must be enough time for the code inside to execute!
	const double MAX_NORMAL_STEERING_ANGLE_DEG = 30.0; // soft limit when doing P-D control on tape, degrees
	int nextLoopTime = millis() + LOOP_TIME_MILLIS;

	int tape_sensor_vals[NUM_TAPE_SENSORS] = {0, 0, 0, 0};
	bool on_tape[NUM_TAPE_SENSORS] = {true, true, true, true};
	int prevErrorDiscreteState = 1;	// -2, -1, 0, 1, or 2 (off tape to left, on tape, off tape to right)
	double prevError = 0;	// from previous control loop, used for derivative control only if prevLeftOnTape and prevRightOnTape.
	double prevErrorDerivative = 0;	// from previous control loop, used for state recovery in case of checkpoint
	const int TAPE_SENSOR_THRESHOLD = 200;	// The analogRead() value above which we consider the tape sensor to be on tape
	// Make the checkpoint sensors deliberately less sensitive to light -> more sensitive to being off tape?
	// const int CHECKPOINT_SENSOR_THRESHOLD = 175;	// The analogRead() value above which we consider the checkpoint sensor to be on tape
	const double STEERING_KP = 10.0;	// Steering angle PID proportionality constant
	const double STEERING_KD = 0.0;	// Steering angle PID derivative constant, per control loop time LOOP_TIME_MILLIS 
	// These max angles are not be achieved in reality if the servo limits are more restrictive.
	const double MAX_STEERING_ANGLE_DEG = 40.0;	// upper bound on desired ideal steering angle. MAX_STEERING_PULSEWIDTH_MICROS PROTECTS PHYSICAL LIMIT.
	const double MIN_STEERING_ANGLE_DEG = -40.0;	// lower bound on desired ideal steering angle. MIN_STEERING_PULSEWIDTH_MICROS PROTECTS PHYSICAL LIMIT.

	// CONTROL LOOP
	int loopCounter = 0;
	while (true) {
		digitalWrite(PIN_LED_BUILTIN, HIGH);	// DEBUG ONLY, for monitoring control loop progression without LCD display
		int errorDiscreteState = prevErrorDiscreteState;	// should be between -NUM_TAPE_SENSORS and +NUM_TAPE_SENSORS, positive when off to right
		double error = prevError;	// Unitless; Positive error means to right of tape, negative to left; prevError only for debug
		double errorDerivative = prevErrorDerivative;	// prevErrorDerivative only for debug
		double steeringAngleDeg = 999.9; // In degrees; Positive angle means steering to left (CCW circling)
		double leftMotorPower = 1.0;	// Between -1 (full reverse) and 1 (full forwards); 0 is off
		double rightMotorPower = 1.0;	// Between -1 (full reverse) and 1 (full forwards); 0 is off

		// bool rightOnCheckpoint = analogRead(PIN_CHECKPOINT_SENSOR_RIGHT > CHECKPOINT_SENSOR_THRESHOLD);
		// bool leftOnCheckpoint = analogRead(PIN_CHECKPOINT_SENSOR_LEFT > CHECKPOINT_SENSOR_THRESHOLD);

		/* if (rightOnCheckpoint || leftOnCheckpoint) {	// freeze current state
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
		} */

		for (int i = 0; i < NUM_TAPE_SENSORS; i++) {
			tape_sensor_vals[i] = analogRead(PINS_TAPE_SENSORS[i]);
			on_tape[i] = tape_sensor_vals[i] > TAPE_SENSOR_THRESHOLD;
		}
		
		// Determination of discrete error state (digital sensors or debug)
		bool offTape = true;
		bool onlyLeftmostOn = on_tape[0];
		bool onlyRightmostOn = on_tape[NUM_TAPE_SENSORS - 1];
		for (int sensor = 0; sensor < NUM_TAPE_SENSORS; sensor++) {
			if (on_tape[sensor]) {	// Strictly greater than required for error determination
				offTape = false;
				if (sensor != 0) onlyLeftmostOn = false;
				if (sensor != NUM_TAPE_SENSORS - 1) onlyRightmostOn = false;
			}
		}

		if (offTape) {
			if (prevError >= 0)	{	// relies on prevError not being updated to avoid wiping the check condition
				errorDiscreteState = 2;
			}
			else {	// prevError < 0
				errorDiscreteState = -2;
			}
			// See if using sign of previous derivative helps here
			// If we completely skipped over the tape line, then god help us we can't see that
			// Perhaps do similar open-loop control to checkpoint: keep scanning as often as possible until we return to the tape line
		} else {
			error = errorFunc(tape_sensor_vals, TAPE_SENSOR_THRESHOLD);	// error is calculated here!
			if (onlyLeftmostOn) {
				errorDiscreteState = 1;
			} else if (onlyRightmostOn) {
				errorDiscreteState = -1;
			} else {
				errorDiscreteState = 0;
			}
		}

		// Now deciding what to actually do
		if (errorDiscreteState >= 2) {	// we are completely off the tape to the right 
			steeringAngleDeg = MAX_STEERING_ANGLE_DEG;
			//leftMotorPower = differentialFromSteering(MAX_STEERING_ANGLE_DEG);
			leftMotorPower = -0.15;	// make time-varying (though it wasn't before)
			rightMotorPower = 1.0;
		} else if (errorDiscreteState <= -2) {	// we are completely off the tape to the left
			steeringAngleDeg = MIN_STEERING_ANGLE_DEG;
			leftMotorPower = 1.0;
			//rightMotorPower = differentialFromSteering(-MAX_STEERING_ANGLE_DEG);
			rightMotorPower = -0.15;	// make time-varying (though it wasn't before)
		} else if (errorDiscreteState == 1) {
			steeringAngleDeg = MAX_STEERING_ANGLE_DEG;
			leftMotorPower = differentialFromSteering(steeringAngleDeg);
			rightMotorPower = 1.0;
		} else if (errorDiscreteState == - 1) {
			steeringAngleDeg = MIN_STEERING_ANGLE_DEG;
			leftMotorPower = 1.0;
			rightMotorPower = differentialFromSteering(steeringAngleDeg);
		} else if (abs(prevErrorDiscreteState) > 0) {	// we were previously off the tape and have just come back on
			// Set steering straight to stabilize for one control loop
			steeringAngleDeg = 0;
			leftMotorPower = 1.0;
			rightMotorPower = 1.0;
			// since we are coming back onto the tape line, the sign of the derivative is opposite the direction of previous error
			errorDerivative = (prevErrorDiscreteState > 0) ? -1.0 : 1.0;	
		} else {	// Just plain error control
			// Because we establish error once when coming back onto tape (see previous case), derivative is always well-defined
			errorDerivative = (error - prevError);
			steeringAngleDeg = STEERING_KP * error + STEERING_KD * errorDerivative;	// P-D control
			steeringAngleDeg = max(MIN_STEERING_ANGLE_DEG, min(MAX_STEERING_ANGLE_DEG, steeringAngleDeg));	// bound by 0
			if (steeringAngleDeg >= 0) {
				leftMotorPower = differentialFromSteering(steeringAngleDeg);
				rightMotorPower = 1.0;
			} else {
				leftMotorPower = 1.0;
				rightMotorPower = differentialFromSteering(rightMotorPower);
			}
		}

		steeringControl(steeringAngleDeg);
		motorControl(leftMotorPower, rightMotorPower);
		

		prevErrorDiscreteState = errorDiscreteState;
		prevError = error;
		prevErrorDerivative = errorDerivative;

		digitalWrite(PIN_LED_BUILTIN, LOW);

		// COMMENT OUT DEBUG DISPLAY CODE IF THERE IS NO DISPLAY, OTHERWISE EXECUTION WILL STALL
		// WHILE TRYING TO WRITE TO A NON-EXISTENT DISPLAY (UNTIL REQUEST TIMEOUT AFTER ~5 SECONDS)
		// debugDisplay(tape_sensor_vals, errorDiscreteState, error, errorDerivative, 
		//		leftMotorPower, rightMotorPower, steeringAngleDeg, debugLeftWheelAngle);
		display_handler.clearDisplay();
		display_handler.setCursor(0, 0);
		for (int i = 0; i < NUM_TAPE_SENSORS; i++) display_handler.printf("%4d", tape_sensor_vals[i]);
		display_handler.println();
		display_handler.printf("St %1d", errorDiscreteState);
		display_handler.print(" E ");
		display_handler.print(error, 2);
		display_handler.print(" dE ");
		display_handler.println(errorDerivative, 2);
		display_handler.print("LM ");
		display_handler.print(leftMotorPower, 3);
		display_handler.print(" RM ");
		display_handler.println(rightMotorPower, 3);
		display_handler.print("Steer ");
		display_handler.print(steeringAngleDeg, 1);
		display_handler.print(" L ");
		display_handler.println(debugLeftWheelAngle, 1);
		display_handler.printf("Loop %d\n", loopCounter);
		display_handler.display();

		// handle interrupt resolution / tasks
		// Make a dedicated queue for this later
		if (bombEject && millis() > bombEjectionEndTime) {
			if (digitalRead(PIN_HALL_SENSOR) == HIGH) {	// bomb is gone: Hall sensor does not see magnetic field
				digitalWrite(PIN_BLOCKMOTOR_REV, LOW);
				digitalWrite(PIN_BLOCKMOTOR_FWD, HIGH);
				bombEject = false;
			} else {	// bomb is still there
				bombEjectionEndTime += 1000;	// check again 1000ms later
			}
		}
		
		while (millis() < nextLoopTime);	// pause until next scheduled control loop, to ensure consistent loop time
		nextLoopTime = millis() + LOOP_TIME_MILLIS;
		loopCounter++;
	}
	// right now, the control loop should never end. If we get here there's been an error.
	motorControl(0.0, 0.0);
	delay(10000000);
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
 * Returns the weighted 'center of tape sensor readings' above the on-tape threshold
 * Precondition: tape_sensor_vals is an array of size NUM_TAPE_SENSORS which contains analog sensor readings corresponding to PINS_TAPE_SENSORS
 * Precondition: at least one sensor must be on tape, i.e. above TAPE_SENSOR_THRESHOLD
*/
double errorFunc(int tape_sensor_vals[], int TAPE_SENSOR_THRESHOLD) {
	int sumSensorVals = 0;
	int sumSensorValsPosWeighted = 0;
	for (int i = 0; i < NUM_TAPE_SENSORS; i++) {
		sumSensorVals += max(0, tape_sensor_vals[i] - TAPE_SENSOR_THRESHOLD);
		sumSensorValsPosWeighted += i * max(0, tape_sensor_vals[i] - TAPE_SENSOR_THRESHOLD);
	}
	double error = 0.5 * (NUM_TAPE_SENSORS - 1) - ((double) sumSensorValsPosWeighted) / ((double) sumSensorVals);
	return error;
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
	if (abs(steeringAngleDeg) < 5.0) {	// minimum angle for effect: to prevent power losses where small differential doesn't help much
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
	if (abs(steeringAngleDeg) < 1.0) {
		pwm_start(PIN_STEERING_SERVO, 50, SERVO_NEUTRAL_PULSEWIDTH, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
	}
	double turnRadius = WHEELBASE / tan(steeringAngleDeg * PI / 180);	// trigonometry methods are by default in radians, so convert to rad.
	// For positive steering angle to left, left wheel is inner ( radius - WHEELSEP/2), right wheel is outer (radius + WHEELSEP/2)
	// double rightWheelAngle = (atan(WHEELBASE / (turnRadius + WHEELSEP/2))) * 180 / PI;	// servo write is in degrees, so convert back to deg	
	double leftWheelAngle = (atan(WHEELBASE / (turnRadius - WHEELSEP/2))) * 180 / PI;	// servo write is in degrees, so convert back to deg	
	// Servo response is linear with 90 degrees rotation to 1000us pulse width (empirical testing of MG90, MG996R)
	int pulseWidthMicros = SERVO_NEUTRAL_PULSEWIDTH + (int) (STEERING_SERVO_DIRECTION_SIGN * (1000/90.0) * leftWheelAngle);
	pulseWidthMicros = max(MIN_STEERING_PULSEWIDTH_MICROS, min(MAX_STEERING_PULSEWIDTH_MICROS, pulseWidthMicros));	// bounded by empirical physical limits
	pwm_start(PIN_STEERING_SERVO, 50, pulseWidthMicros, 
		TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

	debugLeftWheelAngle = leftWheelAngle;
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
