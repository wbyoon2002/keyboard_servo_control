/***************************************************
  Keyboard-driven motor control firmware
  Arduino Uno + Adafruit PCA9685 (16-channel PWM/servo driver)

  This sketch is the firmware half of the keyboard control system.
  The PC (see motor_keyboard_control.py) does all of the speed /
  position logic and streams absolute target angles for all 5 servos.
  This sketch simply maps each angle to a pulse width using the
  per-servo calibration and writes it to the PCA9685.

  Calibration values are copied from:
    arduino_motor_control/calibration_results.txt

  Serial protocol (115200 baud, newline-terminated, case sensitive):
    S a0 a1 a2 a3 a4   Set all 5 servos to the given angles (0-180 deg)
    M i a              Move a single servo i (0-4) to angle a (0-180)
    H                  Home: move all servos to center (90 deg)
    O                  Off: release (de-energize) all servo outputs
    Q                  Query: reply "A a0 a1 a2 a3 a4" with current angles
    P                  Print the active calibration arrays

  Any line that is not understood is answered with "ERR".
  Every accepted command is acknowledged so the PC can stay in sync.
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t SERVO_COUNT = 5;
const uint8_t SERVO_CHANNELS[SERVO_COUNT] = {0, 1, 2, 3, 4};

// --- Calibration (from calibration_results.txt) ------------------------
const uint16_t SERVO_MIN_US[SERVO_COUNT]    = {500, 500, 500, 520, 520};
const uint16_t SERVO_CENTER_US[SERVO_COUNT] = {1500, 1500, 1500, 1490, 1480};
const uint16_t SERVO_MAX_US[SERVO_COUNT]    = {2500, 2500, 2500, 2460, 2440};
// -----------------------------------------------------------------------

const uint16_t SERVO_FREQ = 50;
const uint32_t OSCILLATOR_HZ = 27000000;
const unsigned long SERIAL_BAUD = 115200;
const int CENTER_ANGLE = 90;

int currentAngle[SERVO_COUNT];
bool released[SERVO_COUNT];

char inputBuffer[96];
uint8_t inputLength = 0;

uint16_t angleToPulse(uint8_t servoIndex, int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN_US[servoIndex], SERVO_MAX_US[servoIndex]);
}

void writeServoAngle(uint8_t servoIndex, int angle) {
  angle = constrain(angle, 0, 180);
  pwm.writeMicroseconds(SERVO_CHANNELS[servoIndex], angleToPulse(servoIndex, angle));
  currentAngle[servoIndex] = angle;
  released[servoIndex] = false;
}

void releaseServo(uint8_t servoIndex) {
  pwm.setPWM(SERVO_CHANNELS[servoIndex], 0, 0);
  released[servoIndex] = true;
}

void releaseAllServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    releaseServo(i);
  }
}

void homeAllServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    writeServoAngle(i, CENTER_ANGLE);
  }
}

void reportAngles() {
  Serial.print(F("A"));
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    Serial.print(F(" "));
    Serial.print(currentAngle[i]);
  }
  Serial.println();
}

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

bool parseInt(const char *token, long &value) {
  if (token == NULL || token[0] == '\0') return false;
  char *endPtr;
  // accept floats but truncate to int (PC may stream "90.0")
  double parsed = strtod(token, &endPtr);
  if (endPtr == token) return false;
  value = (long)parsed;
  return true;
}

void processCommand(char *line) {
  char *command = strtok(line, " \t");
  if (command == NULL) return;

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

  if (strcmp(command, "M") == 0) {
    long idx, ang;
    if (!parseInt(strtok(NULL, " \t"), idx) || !parseInt(strtok(NULL, " \t"), ang) ||
        idx < 0 || idx >= SERVO_COUNT) {
      Serial.println(F("ERR M <0-4> <0-180>"));
      return;
    }
    writeServoAngle((uint8_t)idx, constrain((int)ang, 0, 180));
    Serial.println(F("OK M"));
    return;
  }

  if (strcmp(command, "H") == 0) {
    homeAllServos();
    Serial.println(F("OK H"));
    return;
  }

  if (strcmp(command, "O") == 0) {
    releaseAllServos();
    Serial.println(F("OK O"));
    return;
  }

  if (strcmp(command, "Q") == 0) {
    reportAngles();
    return;
  }

  if (strcmp(command, "P") == 0) {
    printCalibration();
    return;
  }

  Serial.println(F("ERR unknown"));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  pwm.begin();
  pwm.setOscillatorFrequency(OSCILLATOR_HZ);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    currentAngle[i] = CENTER_ANGLE;
    released[i] = true;
  }
  releaseAllServos();

  Serial.println(F("READY keyboard_motor_control"));
  Serial.println(F("Commands: S a0..a4 | M i a | H | O | Q | P"));
}

void loop() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      inputBuffer[inputLength] = '\0';
      if (inputLength > 0) {
        processCommand(inputBuffer);
      }
      inputLength = 0;
      continue;
    }

    if (inputLength < sizeof(inputBuffer) - 1) {
      inputBuffer[inputLength++] = incoming;
    }
  }
}
