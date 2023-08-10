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

#define PIN_HALL_SENSOR PB5	// as digital input
#define PIN_BLOCKMOTOR_IN PB15	// as digital output (non-PWM, so max speed)
#define PIN_BLOCKMOTOR_OUT PB14	// as digital output (non-PWM, so max speed)
#define PIN_BLOCKMOTOR_LEFT_ENCODER PB12	// as digital input
#define PIN_BLOCKMOTOR_RIGHT_ENCODER PB13	// as digital input

#define PIN_LED_BUILTIN PC13	// DEBUG ONLY: USE TO INDICATE CONTROL LOOP PROGRESSION WITHOUT LCD DISPLAY

#define PIN_UDS_FRONT_ECHO PB0	// as digital output
#define PIN_UDS_FRONT_TRIGGER PB1	// as digital input

#define PIN_LAUNCH_SWITCH PB9	// as digital input
#define PIN_BOMB_RELEASE PA15	// as digital output. Allows power to flow through the fishing wire burn resistor. DO NOT KEEP HIGH FOR MORE THAN 10s!


//#define PIN_CHECKPOINT_SENSOR_LEFT PA1	// as analog input
//#define PIN_CHECKPOINT_SENSOR_RIGHT PA0	// as analog input
#define PIN_STEERING_SERVO PA_6	// as PWM output - servo control
#define NUM_TAPE_SENSORS 6	// Rep invariant: MUST be the same as length of PINS_TAPE_SENSORS
const int PINS_TAPE_SENSORS[NUM_TAPE_SENSORS] = {PA0, PA1, PA2, PA3, PA4, PA5};	// as analog input. In order from left to right sensors.
const int TAPE_SENSOR_THRESHOLD = 175;	// The analogRead() value above which we consider the tape sensor to be on tape

/*
 * TAPE FOLLOWING
 */
const double WHEELBASE = 133;	// In mm, Lengthwise distance between front and rear wheel axles
const double WHEELSEP = 170;	// In mm, Widthwise distance between front wheels
const int MOTOR_PWM_FREQ = 100;	// In Hz, PWM frequency to H-bridge gate drivers. Currently shared with servos.
const int SERVO_PWM_FREQ = 100;
const double SERVO_NEUTRAL_PULSEWIDTH = 1500;	// In microseconds, default 1500 us. 
const int MAX_STEERING_PULSEWIDTH_MICROS = 1900;	// (1950) absolute physical limit of left-driving servo rotation to left. Currently limited by chassis.
const int MIN_STEERING_PULSEWIDTH_MICROS = 1180;	// absolute physical limit of left-driving servo rotation to right. Currently limited by inversion.
const int BOMB_EJECTION_TIME_MILLIS = 1000;
volatile uint32_t bombEjectionEndTimeMillis = 0;
volatile bool bombEject = false;
volatile uint32_t lastLeftEncoderPulseMillis = 0, lastRightEncoderPulseMillis = 0;
volatile bool isFull = false;
const int NUM_READINGS = 3;

// DISTANCE SENSOR
volatile uint32_t udsLastPulseMicros = 0, udsEchoStartMicros = 0, udsEchoEndMicros = 0;
volatile bool udsPulse = false;
volatile bool pulseReceived = false;
volatile double lastDistCm = 0;

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
void pollBombRelease();
void pollDistanceSensor();
void pollHallSensor();
double errorFunc(int tape_sensor_vals[], int TAPE_SENSOR_THRESHOLD);
void tapeFollowing();
void steeringControl(double steeringAngleDeg);
void steeringControlManual(int pulseWidthMicros);
void motorControl(double lMotorPower, double rMotorPower);
void uds_irq();
void testCode();
void leftBlockMotorEncoder_irq();
void rightBlockMotorEncoder_irq();

//Define serial pins
HardwareSerial Serial3(PB11, PB10);

uint32_t robotStartTimeMillis, bombReleaseTimeMillis, bombReleaseShutoffTimeMillis;

void setup() {
	pinMode(PIN_LMOTOR_FWD, OUTPUT);
	pinMode(PIN_LMOTOR_REV, OUTPUT);
	pinMode(PIN_RMOTOR_FWD, OUTPUT);
	pinMode(PIN_RMOTOR_REV, OUTPUT);

	for (int pin : PINS_TAPE_SENSORS) pinMode(pin, INPUT);

	//pinMode(PIN_CHECKPOINT_SENSOR_LEFT, INPUT);
	//pinMode(PIN_CHECKPOINT_SENSOR_RIGHT, INPUT);
	pinMode(PIN_STEERING_SERVO, OUTPUT);

	pinMode(PIN_UDS_FRONT_ECHO, INPUT);
	pinMode(PIN_UDS_FRONT_TRIGGER, OUTPUT);
	attachInterrupt(PIN_UDS_FRONT_ECHO, uds_irq, CHANGE);
	// Stall protection
	pinMode(PIN_BLOCKMOTOR_LEFT_ENCODER, INPUT_PULLUP);
	pinMode(PIN_BLOCKMOTOR_RIGHT_ENCODER, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PIN_BLOCKMOTOR_LEFT_ENCODER), leftBlockMotorEncoder_irq, RISING);
	attachInterrupt(digitalPinToInterrupt(PIN_BLOCKMOTOR_RIGHT_ENCODER), rightBlockMotorEncoder_irq, RISING);


	pinMode(PIN_LED_BUILTIN, OUTPUT);

  	// for testing, add an assertions check to make sure all variables remain in range. (Basically implementing a rep invariant checker.)
  	// default testing:
	// display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	// display_handler.display();
	// delay(2000);
	// display_handler.clearDisplay();
	// display_handler.setTextSize(1);
	// display_handler.setTextColor(SSD1306_WHITE);
	// writeToDisplay("Either there is no display code after startup, or the program froze before completing a control loop.");

	// set up block collection system. Hall sensor looks Schmitt triggered (switch time ~1us) and sees no bouncing
	pinMode(PIN_HALL_SENSOR, INPUT_PULLUP);	// using our own resistor instead of Bluepill internal pullup resistor
	pinMode(PIN_BLOCKMOTOR_IN, OUTPUT);
	pinMode(PIN_BLOCKMOTOR_OUT, OUTPUT);
	
	bombEject = false;
	attachInterrupt(PIN_HALL_SENSOR, interruptBombEjection, FALLING);	// With pullup resistor, Hall sensor goes low when in strong magnetic field

	pinMode(PIN_LAUNCH_SWITCH, INPUT_PULLUP);	// will be HIGH when switch off, and LOW when switch on.
	pinMode(PIN_BOMB_RELEASE, OUTPUT);	// will activate resistor

	// Serial moniter setup for testing
	Serial3.begin(9600);
	Serial3.println("Setup done");

	// Robot initialization code. Before each heat, check motor power by going through the startup procedure and seeing if the rear motors turn.
	steeringControl(0.0);
	motorControl(0, 0);
	digitalWrite(PIN_BLOCKMOTOR_OUT, LOW);	// block motor rotation indicates that motors are working
	digitalWrite(PIN_BLOCKMOTOR_IN, HIGH);
	digitalWrite(PIN_BOMB_RELEASE, LOW);

	while (digitalRead(PIN_LAUNCH_SWITCH) == HIGH) {
	}	// Busy loop. Make sure this doesn't get optimized out if we use build flags! Try delayMicroseconds(1)?

	// writeToDisplay("LAUNCH!");
	robotStartTimeMillis = millis();
	bombReleaseTimeMillis = robotStartTimeMillis + 10000;	// 90s after robot start
	bombReleaseShutoffTimeMillis = bombReleaseTimeMillis + 7000;	// resistor burn time 3s. Should be good for 6V 15R or 7.4V 20R.
	// ANY HARDCODED START CODE GOES HERE. MAKE SEPERATE METHOD.

	// If needed, insert hardcoding for start and first turn here.
}

// Conversion to state machine
enum State {TAPE_FOLLOWING, COLLISION_AVOIDANCE};
void loop() {
	// State s = TAPE_FOLLOWING;
	// /*
	// If we are on tape, go to tape following
	// 	Here, if we see an object less than 15 cm in front, slow down until robot out of range. (Apply power/speed scale).
	// Else (if we are off tape)
	// 	If we see an object less than 15 cm in front, it may be a static obstacle. 
	// 	Go to collision avoidance until the object is more than 20 cm away, or we are on tape, or no object is detected.
	
	
	// */
	// if (lastDistCm < 15.0) {
	// 	s = COLLISION_AVOIDANCE;
	// } else {

	// }
	if (digitalRead(PIN_LAUNCH_SWITCH) == LOW) {
		steeringControl(0.0);
		motorControl(0.3, 0.3);
		delay(500);
		while (analogRead(PINS_TAPE_SENSORS[1]) < TAPE_SENSOR_THRESHOLD && analogRead(PINS_TAPE_SENSORS[4]) < TAPE_SENSOR_THRESHOLD) {
			// do nothing
		}
		if (analogRead(PINS_TAPE_SENSORS[1]) > TAPE_SENSOR_THRESHOLD) {
			steeringControl(-30.0);
		} else {
			steeringControl(30.0);
		}
		delay(100);
		
		tapeFollowing();
		steeringControl(0);
		motorControl(0, 0);
		digitalWrite(PIN_BOMB_RELEASE, LOW);
		delay(1000);
	}

	// pollBombRelease();	// for seperate testing of bomb drop
	// testCode();
}

void testCode() {
	steeringControlManual(1900);
	motorControl(0.25, 0.40);
	delay(1000);
	// pollDistanceSensor();
	// display_handler.clearDisplay();
  	// display_handler.setCursor(0, 0);
	// display_handler.print("Pulse micros: ");
	// display_handler.println(udsEchoEndMicros);
	// display_handler.print("Distance: ");
	// display_handler.println(lastDistCm);
	// display_handler.display();

	// motorControl(0.50, 0.50);
	// steeringControlManual(1800);
	// delay(1000);
	// steeringControlManual(1200);
	// delay(1000);
}


void interruptBombEjection() {
	digitalWrite(PIN_BLOCKMOTOR_IN, LOW);
	digitalWrite(PIN_BLOCKMOTOR_OUT, HIGH);
	bombEject = true;
	bombEjectionEndTimeMillis = millis() + BOMB_EJECTION_TIME_MILLIS;
}

void leftBlockMotorEncoder_irq() {
	lastLeftEncoderPulseMillis = millis();
}

void rightBlockMotorEncoder_irq() {
	lastRightEncoderPulseMillis = millis();
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

	// REDUCE THIS TO 20 IN TESTING
	uint32_t lastLoopTimeMillis = micros();
	uint32_t last2LoopTimeMillis = lastLoopTimeMillis - 10000;

	int tape_sensor_vals[NUM_TAPE_SENSORS] = {0, 0, 0, 0, 0, 0};
	bool on_tape[NUM_TAPE_SENSORS] = {true, true, true, true};
	double prevError = 0;	// from previous control loop, used for derivative control only if prevLeftOnTape and prevRightOnTape.
	double prev2Error = 0;

	const double DEFAULT_POWER = 0.55; // Power setting, scales all power sent to the motors between 0 and 1. (Ideally want this to be 1.)
	const double SLOW_DEFAULT_POWER = 0.25;	// The above, but when we want to go slow (e.g. off tape or re-entering)
	const double BRAKESTATE_4_POWER = 0.4;
	const double STEERING_KP = 9.0;	// Steering angle PID proportionality constant
	const double STEERING_KD = 1.0;	// Steering angle PID derivative constant, per control loop time LOOP_TIME_MILLIS 
	const double MOTORDIF_KP = 0.01;
	const double MOTORDIF_KD = 0.002;	
	const double MOTORDIF_TIME_KP = 0.002;	// increases differential if we are completely off tape for a long time
	// MOTORSCALE_KP may have to be reduced at lower DEFAULT_POWER and increased at higher DEFAULT_POWER. 
	// Tune it like any other PID variable if the robot looks unstable / overshoots due to long control system response time.
	bool skipLoop=0;
	int offTapeLoops=0;

	enum motorControlState {NORMAL, OFF_TAPE_BRAKE, OFF_TAPE_STEADY, RECOVERY_INERTIA_COUNTER, RECOVERY_STEADY};	// FOR USE LATER
	int brakeState = 4;	// 0 for not activated, 1 for active, 2 for offTape and active, 3 for onTape and active. Reset to 0 short while after reentering tape.
	uint32_t brake12TimeMillis = 0;
	uint32_t brake34TimeMillis = 0;
	uint32_t brake40TimeMillis = 0;
	// brake system explanation:
	// If we are on tape and we have been on tape for more than XX ms, normal motor control
	// If we are off tape and we have 
	uint32_t endLoopSkipTimeMillis = 0;

	lastLeftEncoderPulseMillis = millis();
	lastRightEncoderPulseMillis = millis();

	// CONTROL LOOP
	int loopCounter = 0;
	while (digitalRead(PIN_LAUNCH_SWITCH) == LOW) {
		digitalWrite(PIN_LED_BUILTIN, HIGH);	// DEBUG ONLY, for monitoring control loop progression without LCD display
		double error;	// Unitless; Positive error means to right of tape, negative to left; prevError only for debug
		double errorDerivative;	// prevErrorDerivative only for debug
		double steeringAngleDeg; // In degrees; Positive angle means steering to left (CCW circling)
		double motorDif;	// Out of 1.0, power added to right motor vs left
		double leftMotorPower;	// Between -1 (full reverse) and 1 (full forwards); 0 is off
		double rightMotorPower;	// Between -1 (full reverse) and 1 (full forwards); 0 is off
		bool offTape = true;
		bool onCheckpoint = false;

		// handle interrupt resolution / tasks
		// Make a dedicated queue for this later
		// pollBombRelease();
		// pollDistanceSensor();
		pollHallSensor();

		uint32_t currentTimeMillis = millis();
		uint32_t currentTimeMicros = micros();
		if (currentTimeMillis < endLoopSkipTimeMillis) {
			continue;
		}
		int tapeSensorsOnLine=0;
		for (int i = 0; i < NUM_TAPE_SENSORS; i++) {
			tape_sensor_vals[i] = analogRead(PINS_TAPE_SENSORS[i]);
			on_tape[i] = tape_sensor_vals[i] > TAPE_SENSOR_THRESHOLD;
			if (on_tape[i]) {
				offTape = false;
				tapeSensorsOnLine++;
			}
		}

		if (offTape) {
			if (prevError >= 0)	{	// relies on prevError not being updated to avoid wiping the check condition
				error = (NUM_TAPE_SENSORS + 1) / 2.0;
			}
			else {	// prevError < 0
				error = - (NUM_TAPE_SENSORS + 1) / 2.0;
			}

			if (brakeState == 0 || brakeState == 3 || brakeState == 4) {	// we were on tape
				brakeState = 1;
				brake12TimeMillis = currentTimeMillis + 250;	// or however many milliseconds you want
			} else if (brakeState == 1 && currentTimeMillis > brake12TimeMillis) {
				brakeState = 2;
			}
			offTapeLoops++;
			// See if using sign of previous derivative helps here
			// If we completely skipped over the tape line, then god help us we can't see that
			// Perhaps do similar open-loop control to checkpoint: keep scanning as often as possible until we return to the tape line
		} else {
			offTapeLoops=0;
			error = errorFunc(tape_sensor_vals, TAPE_SENSOR_THRESHOLD);
			// check if we are on a check point and which one it is
			bool prevH=0;
			for (int i = 1; i < NUM_TAPE_SENSORS; i++){
				if (!on_tape[i]&&on_tape[i-1]){
					prevH=1;
				}
				else if (prevH && on_tape[i]){
					onCheckpoint = true;
					break;
				}
			}
			
			if ((brakeState == 1 || brakeState == 2)) {	// we were previously off tape
				brakeState = 3;
				brake34TimeMillis = currentTimeMillis + 25;	// or however many milliseconds you want. Maybe vary with time in brakeState1?
			} else if ((brakeState == 3 && currentTimeMillis > brake34TimeMillis) || brakeState == 0 && abs(error) >= 2.5) {
				brakeState = 4;
				brake40TimeMillis = currentTimeMillis + 750;
			} else if (brakeState == 4 && currentTimeMillis > brake40TimeMillis) {
				brakeState = 0;
			}
		}

		if (onCheckpoint || tapeSensorsOnLine>3) {
			steeringControl(0);
			motorControl(SLOW_DEFAULT_POWER, SLOW_DEFAULT_POWER);
			endLoopSkipTimeMillis = millis() + 50;
			continue;
		} else {
			// Now deciding what to actually do
			// For now, avoid case-based control
			// Set neutral motor power to 0.5 and add power (fraction or percentage?) to this
			// Encoder-based speed control can come later
			// errorDerivative = (currentTimeMillis > lastLoopTimeMillis) ? 250 * (error - prevError) / (currentTimeMillis - lastLoopTimeMillis) : 0;
			errorDerivative = (currentTimeMicros > last2LoopTimeMillis) ? 1000 * 700 * (error - prev2Error) / (currentTimeMicros - last2LoopTimeMillis) : 0;
			steeringAngleDeg = max(-60.0, min(60.0, STEERING_KP * error + STEERING_KD * errorDerivative));
			steeringControl(steeringAngleDeg);
			motorDif = MOTORDIF_KP * error + MOTORDIF_KD * errorDerivative + ((error >= 0) ? 1 : -1) * MOTORDIF_TIME_KP * offTapeLoops;
			if (brakeState == 0) {
				leftMotorPower = DEFAULT_POWER - motorDif;
				rightMotorPower = DEFAULT_POWER + motorDif;
				motorControl(leftMotorPower, rightMotorPower);
			} else if (brakeState == 1) {
				if (error > 0) {
					motorControl(-0.3, -0.05);
				} else {
					motorControl(-0.05, -0.3);
				}
			} else if (brakeState == 2) {	// try changing this to const differential
				leftMotorPower = SLOW_DEFAULT_POWER - motorDif;
				rightMotorPower = SLOW_DEFAULT_POWER + motorDif;
				motorControl(leftMotorPower, rightMotorPower);
			}else if (brakeState == 4) {	// try changing this to const differential
				leftMotorPower = BRAKESTATE_4_POWER - motorDif;
				rightMotorPower = BRAKESTATE_4_POWER + motorDif;
				motorControl(leftMotorPower, rightMotorPower);
			} else if (brakeState == 3) {
				if (error > 0) {
					motorControl(-0.1, 1);
				} else {
					motorControl(1, -0.1);
				}
			}
			
			prev2Error = prevError;
			prevError = error;

			last2LoopTimeMillis = lastLoopTimeMillis;
			lastLoopTimeMillis = currentTimeMicros;
		}

		digitalWrite(PIN_LED_BUILTIN, LOW);

		// COMMENT OUT DEBUG DISPLAY CODE IF THERE IS NO DISPLAY, OTHERWISE EXECUTION WILL STALL
		// WHILE TRYING TO WRITE TO A NON-EXISTENT DISPLAY (UNTIL REQUEST TIMEOUT AFTER ~5 SECONDS)
		// The display print together with other code takes about 36ms to print, so control loop time of 40ms (25Hz) is fine.
		// display_handler.clearDisplay();
		// display_handler.setCursor(0, 0);
		// for (int i = 0; i < NUM_TAPE_SENSORS; i++) display_handler.printf("%5d", tape_sensor_vals[i]);
		// Serial3.println();
		// Serial3.print(" E ");
		// Serial3.print(error, 2);
		// Serial3.print(" dE ");
		// Serial3.println(errorDerivative, 2);
		// Serial3.print("LM ");
		// Serial3.print(leftMotorPower, 3);
		// Serial3.print(" RM ");
		// Serial3.println(rightMotorPower, 3);
		// Serial3.print("Steer ");
		// Serial3.print(steeringAngleDeg, 1);
		// Serial3.print(" L ");
		// Serial3.println(debugLeftWheelAngle, 1);
		// Serial3.printf("Loop %d\n", loopCounter);
		// // display_handler.display(); (REMOVED)
		// Serial3.printf("Time %d", millis() - (nextLoopTime - LOOP_TIME_MILLIS));// (REMOVED)
		// display_handler.display();
		
		
		// if (millis() >= nextLoopTime) {
		// 	nextLoopTime = millis() + LOOP_TIME_MILLIS;
		// } else {
		// 	while (millis() < nextLoopTime);	// pause until next scheduled control loop, to ensure consistent loop time synced with servo PWM frequency
		// 	nextLoopTime += LOOP_TIME_MILLIS;	// may add emergency handling if we have exceeded the previous loop time
		// }
		loopCounter++;
	}
	// The loop will exit if the launch switch is turned off again. But this does not reset the bluepill, bomb release, or block collection!
}

void pollBombRelease() {
	uint32_t currentTime = millis();
	if (currentTime > bombReleaseShutoffTimeMillis) {
		digitalWrite(PIN_BOMB_RELEASE, LOW);
	} else if (currentTime > bombReleaseTimeMillis) {
		digitalWrite(PIN_BOMB_RELEASE, HIGH);
	}
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

void pollDistanceSensor() {
	if (micros() - udsLastPulseMicros > 60000) {	// only activate UDS when guaranteed that the previous pulse has either been returned or dissipated
		pulseReceived = false;
		digitalWrite(PIN_UDS_FRONT_TRIGGER, LOW);
		delayMicroseconds(2);
		// Sets the trigPin on HIGH state for 10 micro seconds
		digitalWrite(PIN_UDS_FRONT_TRIGGER, HIGH);
		delayMicroseconds(10);
		digitalWrite(PIN_UDS_FRONT_TRIGGER, LOW);
		udsLastPulseMicros = micros();
	}
}

void pollHallSensor() {
	if (isFull) return;

	uint32_t currentTimeMillis = millis();
	if (bombEject && currentTimeMillis > bombEjectionEndTimeMillis) {
		if (digitalRead(PIN_HALL_SENSOR) == HIGH) {	// bomb is gone: Hall sensor does not see magnetic field
			digitalWrite(PIN_BLOCKMOTOR_OUT, LOW);
			digitalWrite(PIN_BLOCKMOTOR_IN, HIGH);
			bombEject = false;
		} else {	// bomb is still there
			bombEjectionEndTimeMillis += 1000;	// check again 1000ms later
		}
	}

	if ((currentTimeMillis - lastLeftEncoderPulseMillis > 3000 && lastLeftEncoderPulseMillis != 0)
		|| (currentTimeMillis - lastRightEncoderPulseMillis > 3000 && lastRightEncoderPulseMillis != 0)) {
			// turn off both motors and disable the bomb detection interrupt to avoid spewing out blocks
			detachInterrupt(digitalPinToInterrupt(PIN_BLOCKMOTOR_LEFT_ENCODER));
			detachInterrupt(digitalPinToInterrupt(PIN_BLOCKMOTOR_RIGHT_ENCODER));
			isFull = true;
			bombEject = false;
			digitalWrite(PIN_BLOCKMOTOR_IN, LOW);
			digitalWrite(PIN_BLOCKMOTOR_OUT, LOW);
	}
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
		pwm_start(PIN_STEERING_SERVO, SERVO_PWM_FREQ, SERVO_NEUTRAL_PULSEWIDTH, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
	}
	double turnRadius = WHEELBASE / tan(steeringAngleDeg * PI / 180);	// trigonometry methods are by default in radians, so convert to rad.
	// For positive steering angle to left, left wheel is inner ( radius - WHEELSEP/2), right wheel is outer (radius + WHEELSEP/2)
	// double rightWheelAngle = (atan(WHEELBASE / (turnRadius + WHEELSEP/2))) * 180 / PI;	// servo write is in degrees, so convert back to deg
	double leftWheelAngle;	
	// Problem if turnRadius > 0 (steeringAngleDeg > 0) but turnRadius - WHEELSEP/2 < 0 (either div0 or leftWheelAngle flips negative)
	if (turnRadius > 0 && turnRadius <= 1.1 * WHEELSEP / 2) {	
		leftWheelAngle = 90;
	} else {
		leftWheelAngle = (atan(WHEELBASE / (turnRadius - WHEELSEP/2))) * 180 / PI;	// servo write is in degrees, so convert back to deg	
	}
	// Servo response is linear with 90 degrees rotation to 1000us pulse width (empirical testing of MG90, MG996R)
	int pulseWidthMicros = SERVO_NEUTRAL_PULSEWIDTH + (int) ((1000.0/90.0) * leftWheelAngle);
	pulseWidthMicros = max(MIN_STEERING_PULSEWIDTH_MICROS, min(MAX_STEERING_PULSEWIDTH_MICROS, pulseWidthMicros));	// bounded by empirical physical limits
	pwm_start(PIN_STEERING_SERVO, SERVO_PWM_FREQ, pulseWidthMicros, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

	debugLeftWheelAngle = leftWheelAngle;
}

/*
 * Commands the servo with a certain pulsewidth corresponding to angle. ALLOWS OVERRIDING OF PHYSICAL LIMITS.
*/
void steeringControlManual(int pulseWidthMicros) {
	// Servo response is linear with 90 degrees rotation to 1000us pulse width (empirical testing of MG90, MG996R)
	pwm_start(PIN_STEERING_SERVO, SERVO_PWM_FREQ, pulseWidthMicros, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);

	debugLeftWheelAngle = (pulseWidthMicros - SERVO_NEUTRAL_PULSEWIDTH) * (90.0/1000.0);
}

/*
 * Sends power to the motor by modulating power through the H-bridges. (Current implementation: via the gate drivers.)
 * 
 * IDEAS FOR IMPROVEMENT:
 * Add smoothing: don't change unless new motor powers are far enough from previous powers. (Greater dist -> shorter time). 
 * 	Not necessary if PWM period synchronized with control loop period -- integer multiple?
 * Add initial acceleration/braking 'jerk' when speeding motor from stall or if needing fast motor braking.
 * 	Active braking may inject excessive noise by back-EMF? 
 * 	Acceleration jerk to full power for ~20-100ms to get moving and overcome static friction? 
 * 
 * @param lMotorPower The power to send to the left motor, signed for direction. 
 * 	Inputs truncated to between -1 (full reverse) and 1 (full forwards), with 0 being off.
 * @param rMotorPower Same for the right motor, truncated to between -1 and 1.
 */
void motorControl(double lMotorPower, double rMotorPower) {
	lMotorPower = max(-0.50, min(1.0, lMotorPower));
	rMotorPower = max(-0.50, min(1.0, rMotorPower));
	if (lMotorPower > 0) {	// forwards
		pwm_start(PIN_LMOTOR_REV, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_LMOTOR_FWD, MOTOR_PWM_FREQ, (int) (1023 * lMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else if (lMotorPower < 0) {	// reverse
		pwm_start(PIN_LMOTOR_FWD, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_LMOTOR_REV, MOTOR_PWM_FREQ, (int) (1023 * - lMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else {	// unpowered / stop
		pwm_start(PIN_LMOTOR_FWD, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_LMOTOR_REV, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
	}

	if (rMotorPower > 0) {	// forwards
		pwm_start(PIN_RMOTOR_REV, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_RMOTOR_FWD, MOTOR_PWM_FREQ, (int) (1023 * rMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else if (rMotorPower < 0) {	// reverse
		pwm_start(PIN_RMOTOR_FWD, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_RMOTOR_REV, MOTOR_PWM_FREQ, (int) (1023 * - rMotorPower), RESOLUTION_10B_COMPARE_FORMAT);
	} else {	// unpowered / stop
		pwm_start(PIN_RMOTOR_FWD, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
		pwm_start(PIN_RMOTOR_REV, MOTOR_PWM_FREQ, 0, RESOLUTION_10B_COMPARE_FORMAT);
	}

	// Serial3.print("LM ");
	// Serial3.print(lMotorPower, 3);
	// Serial3.print(" RM ");
	// Serial3.println(rMotorPower, 3);
}

void uds_irq() {
  if (udsPulse) {
    udsEchoEndMicros = micros();
    udsPulse = false;
    lastDistCm = (udsEchoEndMicros - udsEchoStartMicros) / 58.0;
    if (udsEchoStartMicros - udsLastPulseMicros < 30000) {
      pulseReceived = true;
    } else {
      lastDistCm = 0;
    }
  } else {
    udsEchoStartMicros = micros();
    udsPulse = true;
  }
}