/*******************************************************************************
 * Keyboard-driven Motor Control Firmware
 * Hardware: Arduino Uno + Adafruit PCA9685 (16-channel PWM/servo driver)
 *
 * This firmware receives absolute target angles for 5 servos over Serial,
 * maps them to microsecond pulse widths using per-servo calibration values,
 * and outputs the PWM signals via the PCA9685 driver.
 *
 * Serial Protocol:
 *   Baud Rate: 115200 bps
 *   Delimiter: Newline ('\n') terminated, case-sensitive
 *
 * Commands:
 *   S a0 a1 a2 a3 a4  Set target angles (0-180) for all 5 servos immediately.
 *   M i a [v]         Move a single servo i (0-4) to angle a (0-180).
 *                     Optional parameter v sets the speed in degrees/second.
 *                     If v is omitted or <= 0, the move is immediate.
 *   H                 Home all servos to their center positions (90 deg).
 *   O                 Off / Release: De-energize all servo outputs.
 *   Q                 Query: Ask for current angles. Returns "A a0 a1 a2 a3 a4".
 *   P                 Print current calibration values.
 *
 * Responses:
 *   A accepted command is acknowledged with "OK <Cmd>" or its specific output format.
 *   Invalid or failed commands return "ERR <reason>".
 ******************************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// =============================================================================
// HARDWARE & DRIVER INITIALIZATION
// =============================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// =============================================================================
// SYSTEM CONSTANTS & CONFIGURATION
// =============================================================================

// Servo configurations & Motor Channel Constants
const uint8_t SERVO_COUNT = 5;
const uint8_t ATOP   = 0;
const uint8_t WHEEL  = 1;
const uint8_t ABOT   = 2;
const uint8_t ALEFT  = 3;
const uint8_t ARIGHT = 4;
const uint8_t SERVO_CHANNELS[SERVO_COUNT] = {ATOP, WHEEL, ABOT, ALEFT, ARIGHT};
const int CENTER_ANGLE = 90;

// Servo PWM pulse limits (in microseconds), copied from calibration_results.txt
const uint16_t SERVO_MIN_US[SERVO_COUNT]    = {500,  500,  500,  520,  520};
const uint16_t SERVO_CENTER_US[SERVO_COUNT] = {1500, 1500, 1500, 1490, 1480};
const uint16_t SERVO_MAX_US[SERVO_COUNT]    = {2500, 2500, 2500, 2460, 2440};

// PWM Driver timing settings
const uint16_t SERVO_FREQ = 50;            // Analog servos run at 50Hz
const uint32_t OSCILLATOR_HZ = 27000000;    // PCA9685 internal oscillator frequency

// Communication settings
const unsigned long SERIAL_BAUD = 115200;   // Serial communication speed

// =============================================================================
// GLOBAL STATE VARIABLES
// =============================================================================

float currentAngle[SERVO_COUNT];            // Current interpolated angles (float for precision trajectory)
float targetAngle[SERVO_COUNT];             // Target angles to reach
float servoSpeed[SERVO_COUNT];              // Speed for trajectory interpolation in deg/s (0 = immediate)
bool released[SERVO_COUNT];                 // True if the servo is currently de-energized

unsigned long lastUpdateTime = 0;           // Timer to compute dt for non-blocking movement
char inputBuffer[96];                       // Buffer to store incoming serial data
uint8_t inputLength = 0;                    // Current length of the buffered string

// =============================================================================
// SERVO CONTROL UTILITY FUNCTIONS
// =============================================================================

/**
 * Converts a target angle (0 to 180 degrees) to a PWM pulse width in microseconds
 * using the individual servo's minimum and maximum calibration values.
 */
uint16_t angleToPulse(uint8_t servoIndex, int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN_US[servoIndex], SERVO_MAX_US[servoIndex]);
}

/**
 * Writes the specified angle to a servo immediately (canceling speed-controlled movement).
 */
void writeServoAngle(uint8_t servoIndex, float angle) {
  angle = constrain(angle, 0.0, 180.0);
  targetAngle[servoIndex] = angle;
  servoSpeed[servoIndex] = 0.0;             // Disable speed tracking (immediate move)
  currentAngle[servoIndex] = angle;
  pwm.writeMicroseconds(SERVO_CHANNELS[servoIndex], angleToPulse(servoIndex, (int)round(angle)));
  released[servoIndex] = false;
}

/**
 * Sets target angle and speed for a servo. If speed is <= 0, it moves immediately.
 */
void writeServoAngleWithSpeed(uint8_t servoIndex, float angle, float speed) {
  angle = constrain(angle, 0.0, 180.0);
  targetAngle[servoIndex] = angle;
  if (speed <= 0.0) {
    servoSpeed[servoIndex] = 0.0;
    currentAngle[servoIndex] = angle;
    pwm.writeMicroseconds(SERVO_CHANNELS[servoIndex], angleToPulse(servoIndex, (int)round(angle)));
  } else {
    servoSpeed[servoIndex] = speed;
  }
  released[servoIndex] = false;
}

/**
 * De-energizes (releases) a single servo by setting its PWM output to zero.
 */
void releaseServo(uint8_t servoIndex) {
  pwm.setPWM(SERVO_CHANNELS[servoIndex], 0, 0);
  released[servoIndex] = true;
  servoSpeed[servoIndex] = 0.0;
}

/**
 * De-energizes (releases) all servos.
 */
void releaseAllServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    releaseServo(i);
  }
}

/**
 * Moves all servos to their home (90-degree) position immediately.
 */
void homeAllServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    writeServoAngle(i, CENTER_ANGLE);
  }
}

/**
 * Interpolates current servo positions towards target angles based on elapsed time (dt).
 * This function runs periodically and is non-blocking.
 */
void updateServoTrajectories() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastUpdateTime;
  if (elapsed < 10) return;                 // Update at 100Hz frequency (every 10ms)

  float dt = elapsed / 1000.0;
  lastUpdateTime = now;

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    // Skip if the servo is released or has no active speed limit
    if (released[i] || servoSpeed[i] <= 0.0) {
      continue;
    }

    float diff = targetAngle[i] - currentAngle[i];
    if (abs(diff) < 0.01) {
      currentAngle[i] = targetAngle[i];
      servoSpeed[i] = 0.0;
      continue;
    }

    // Proportional deceleration zone (Ease-out) to prevent inertial shaking at the end
    float current_speed = servoSpeed[i];
    const float DECEL_ZONE = 15.0; // Start slowing down when within 15 degrees of target
    const float MIN_SPEED = 8.0;   // Minimum speed in deg/s to overcome static friction and prevent sticking

    if (abs(diff) < DECEL_ZONE) {
      current_speed = current_speed * (abs(diff) / DECEL_ZONE);
      if (current_speed < MIN_SPEED) {
        current_speed = MIN_SPEED;
      }
    }

    float step = current_speed * dt;

    if (abs(diff) <= step) {
      currentAngle[i] = targetAngle[i];
      servoSpeed[i] = 0.0;                  // Target reached
    } else {
      if (diff > 0.0) {
        currentAngle[i] += step;
      } else {
        currentAngle[i] -= step;
      }
    }

    // Output the updated position to the PCA9685 driver
    pwm.writeMicroseconds(SERVO_CHANNELS[i], angleToPulse(i, (int)round(currentAngle[i])));
  }
}

// =============================================================================
// SERIAL REPORTING & LOGGING FUNCTIONS
// =============================================================================

/**
 * Reports the current servo angles in the format: "A a0 a1 a2 a3 a4"
 */
void reportAngles() {
  Serial.print(F("A"));
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    Serial.print(F(" "));
    Serial.print((int)round(currentAngle[i]));
  }
  Serial.println();
}

/**
 * Prints the active calibration values to the serial monitor.
 */
void printCalibration() {
  Serial.print(F("CAL MIN"));
  for (uint8_t i = 0; i < SERVO_COUNT; i++) { Serial.print(F(" ")); Serial.print(SERVO_MIN_US[i]); }
  Serial.println();

  Serial.print(F("CAL CTR"));
  for (uint8_t i = 0; i < SERVO_COUNT; i++) { Serial.print(F(" ")); Serial.print(SERVO_CENTER_US[i]); }
  Serial.println();

  Serial.print(F("CAL MAX"));
  for (uint8_t i = 0; i < SERVO_COUNT; i++) { Serial.print(F(" ")); Serial.print(SERVO_MAX_US[i]); }
  Serial.println();
}

// =============================================================================
// PARSING & COMMAND PROCESSING
// =============================================================================

/**
 * Parses an integer value from a string token, supporting potential float formats
 * (e.g., "90.0") by parsing them as doubles and truncating them to integers.
 */
bool parseInt(const char *token, long &value) {
  if (token == NULL || token[0] == '\0') return false;
  char *endPtr;
  double parsed = strtod(token, &endPtr);
  if (endPtr == token) return false;
  value = (long)parsed;
  return true;
}

/**
 * Parses and processes a single command line received over the serial interface.
 */
void processCommand(char *line) {
  char *command = strtok(line, " \t");
  if (command == NULL) return;

  // Command 'S': Set all 5 servo angles simultaneously (immediate)
  if (strcmp(command, "S") == 0) {
    int angles[SERVO_COUNT];
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      long a;
      if (!parseInt(strtok(NULL, " \t"), a)) {
        Serial.println(F("ERR S needs 5 angles"));
        return;
      }
      angles[i] = constrain((int)a, 0, 180);
    }
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      writeServoAngle(i, angles[i]);
    }
    Serial.println(F("OK S"));
    return;
  }

  // Command 'M': Move a single servo (M idx angle [speed])
  if (strcmp(command, "M") == 0) {
    long idx, ang;
    char *idxToken = strtok(NULL, " \t");
    char *angToken = strtok(NULL, " \t");
    
    if (!parseInt(idxToken, idx) || !parseInt(angToken, ang) ||
        idx < 0 || idx >= SERVO_COUNT) {
      Serial.println(F("ERR M <0-4> <0-180> [<speed>]"));
      return;
    }

    long speedVal = 0;
    char *speedToken = strtok(NULL, " \t");
    if (speedToken != NULL) {
      if (!parseInt(speedToken, speedVal) || speedVal < 0) {
        Serial.println(F("ERR speed must be positive integer"));
        return;
      }
    }

    writeServoAngleWithSpeed((uint8_t)idx, constrain((int)ang, 0, 180), (float)speedVal);
    Serial.println(F("OK M"));
    return;
  }

  // Command 'H': Home all servos (90 degrees)
  if (strcmp(command, "H") == 0) {
    homeAllServos();
    Serial.println(F("OK H"));
    return;
  }

  // Command 'O': Off (release/de-energize all servos)
  if (strcmp(command, "O") == 0) {
    releaseAllServos();
    Serial.println(F("OK O"));
    return;
  }

  // Command 'Q': Query current angles
  if (strcmp(command, "Q") == 0) {
    reportAngles();
    return;
  }

  // Command 'P': Print calibration settings
  if (strcmp(command, "P") == 0) {
    printCalibration();
    return;
  }

  // Fallback for unrecognized commands
  Serial.println(F("ERR unknown"));
}

// =============================================================================
// ARDUINO STANDARD LIFECYCLE METHODS
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  // Initialize PCA9685 PWM controller
  pwm.begin();
  pwm.setOscillatorFrequency(OSCILLATOR_HZ);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize state and set all servos to safe/released state
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    currentAngle[i] = CENTER_ANGLE;
    targetAngle[i] = CENTER_ANGLE;
    servoSpeed[i] = 0.0;
    released[i] = true;
  }
  releaseAllServos();

  lastUpdateTime = millis();

  // Send ready message to the host PC
  Serial.println(F("READY keyboard_motor_control"));
  Serial.println(F("Commands: S a0..a4 | M i a [v] | H | O | Q | P"));
}

void loop() {
  // Update servo movements asynchronously (non-blocking)
  updateServoTrajectories();

  // Read incoming characters from the serial buffer
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();

    if (incoming == '\r') {
      continue; // Skip carriage return characters
    }

    if (incoming == '\n') {
      inputBuffer[inputLength] = '\0';
      if (inputLength > 0) {
        processCommand(inputBuffer);
      }
      inputLength = 0;
      continue;
    }

    // Append to buffer if there is space remaining
    if (inputLength < sizeof(inputBuffer) - 1) {
      inputBuffer[inputLength++] = incoming;
    }
  }
}
