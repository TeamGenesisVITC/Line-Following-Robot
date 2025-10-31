#include <Arduino.h>

// Motor pins
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10

// Button pins
const int CALIBRATE_BUTTON_PIN = 11;
const int START_BUTTON_PIN = 12;

// --- Speed Control Variables ---
int BASE_SPEED = 100;         // Base speed for PID line following (0-255)
int CALIBRATION_SPEED = 100;   // Spin speed for calibration (0-255)

const int NUM_SENSORS = 8; // Changed from numSensors
int sensorPins[NUM_SENSORS] = {A7, A6, A5, A4, A3, A2, A1, A0};

bool isBlackLine = 1;
// Thresholds are no longer hard-coded. They will be set by calibration.
float threshold[NUM_SENSORS];
int weight[NUM_SENSORS] = {-8, -4, -2, -1, 1, 2, 4, 8};

// PID constants — tune these
float Kp = 70.0;
float Ki = 0.0;
float Kd = 25.0;

float integral = 0;
float previousError = 0;

// Robot state management
enum RobotState { IDLE, CALIBRATING, RUNNING };
RobotState robotState = IDLE;

void setMotor(int pin1, int pin2, int pwm, int speed) {
  // --- UPDATED as requested ---
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pwm, speed);
  } else if (speed < 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(pwm, -speed);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    analogWrite(pwm, 0);
  }
}

void motor1run(int speed) { setMotor(AIN1, AIN2, PWMA, speed); }
void motor2run(int speed) { setMotor(BIN1, BIN2, PWMB, speed); }

/**
 * @brief Runs the calibration routine.
 * Spins the robot left to find the min/max reading for each sensor,
 * then calculates the midpoint threshold.
 */
void runCalibration() {
  // --- UPDATED variable names as requested ---
  int minValues[NUM_SENSORS];
  int maxValues[NUM_SENSORS];

  // Initialize min/max arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 1023; // Start high
    maxValues[i] = 0;    // Start low
  }

  // Spin left for 3 seconds at calibration speed
  Serial.println("Calibrating: Spinning left...");
  // --- UPDATED to use global variable ---
  motor1run(-CALIBRATION_SPEED);
  motor2run(CALIBRATION_SPEED);

  unsigned long startTime = millis();
  while (millis() - startTime < 3000) { // Spin for 3 seconds
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      // --- UPDATED variable names ---
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
  }

  motor1run(0); // Stop
  motor2run(0);

  // Calculate and set new thresholds
  Serial.println("Calibration Complete. New Thresholds:");
  
  // --- UPDATED with your exact logic ---
  for (int i = 0; i < NUM_SENSORS; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();
}

/**
 * @brief This is your original loop() function, unchanged.
 * It executes the PID line-following logic.
 */
void followLine() {
  int sensor[NUM_SENSORS];
  float error = 0.0;
  int activeCount = 0;

  // Read all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = analogRead(sensorPins[i]);
    sensor[i] = (isBlackLine) ? (val >= threshold[i]) : (val < threshold[i]);
    if (sensor[i]) activeCount++;
    error += sensor[i] * weight[i];
  }

  if (activeCount == 0) {
    motor1run(0);
    motor2run(0);
    return;
  }

  bool leftExtreme = (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0);
  bool rightExtreme = (sensor[7] == 1 && sensor[6] == 0 && sensor[5] == 0);

  // --- Delayless Spin Logic (Unchanged) ---
  if (leftExtreme) {
    Serial.println("Hard left turn (delayless)");
    while (true) {
      motor1run(-100);
      motor2run(100);
      int midLeft = analogRead(sensorPins[3]);
      int midRight = analogRead(sensorPins[4]);
      if ((isBlackLine && midLeft >= threshold[3] && midRight >= threshold[4]) ||
          (!isBlackLine && midLeft < threshold[3] && midRight < threshold[4])) {
        break;
      }
    }
    motor1run(0);
    motor2run(0);
    delay(20);
    return;
  }

  if (rightExtreme) {
    Serial.println("Hard right turn (delayless)");
    while (true) {
      motor1run(100);
      motor2run(-100);
      int midLeft = analogRead(sensorPins[3]);
      int midRight = analogRead(sensorPins[4]);
      if ((isBlackLine && midLeft >= threshold[3] && midRight >= threshold[4]) ||
          (!isBlackLine && midLeft < threshold[3] && midRight < threshold[4])) {
        break;
      }
    }
    motor1run(0);
    motor2run(0);
    delay(20);
    return;
  }

  // --- Normal PID Control (Unchanged logic) ---
  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - previousError;
  previousError = error;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // --- UPDATED to use global variable ---
  int baseSpeed = BASE_SPEED; 
  int leftSpeed = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);

  motor1run(leftSpeed);
  motor2run(rightSpeed);
}


void setup() {
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++) pinMode(sensorPins[i], INPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);

  // Initialize button pins with internal pull-up resistors
  pinMode(CALIBRATE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  delay(3000);
  Serial.println("Robot ready.");
  Serial.println("Press button 11 to calibrate sensors.");
  Serial.println("Press button 12 to start line following.");
}

/**
 * @brief Main loop now acts as a state machine.
 * It checks the robot's state and decides what to do.
 */
void loop() {
  switch (robotState) {
    case IDLE:
      // Robot is doing nothing, waiting for a command
      motor1run(0); // Ensure motors are off
      motor2run(0);

      // Check for calibrate button press (LOW signal means pressed)
      if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
        delay(50); // Simple debounce
        if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW) {
          robotState = CALIBRATING; // Change state to Calibrating
          while (digitalRead(CALIBRATE_BUTTON_PIN) == LOW); // Wait for button release
        }
      }

      // Check for start button press
      if (digitalRead(START_BUTTON_PIN) == LOW) {
        delay(50); // Simple debounce
        if (digitalRead(START_BUTTON_PIN) == LOW) {
          Serial.println("Starting run...");
          // Reset PID variables for a clean start
          integral = 0;
          previousError = 0;
          robotState = RUNNING; // Change state to Running
          while (digitalRead(START_BUTTON_PIN) == LOW); // Wait for button release
        }
      }
      break;

    case CALIBRATING:
      // Run the calibration function once
      runCalibration();
      robotState = IDLE; // Go back to IDLE state when done
      Serial.println("Ready. Press 11 to re-calibrate, 12 to run.");
      break;

    case RUNNING:
      // Execute the line following logic
      followLine();

      // Check for start button press *again* to stop
      if (digitalRead(START_BUTTON_PIN) == LOW) {
        delay(50); // Debounce
        if (digitalRead(START_BUTTON_PIN) == LOW) {
          Serial.println("Run stopped. Returning to idle.");
          robotState = IDLE; // Go back to IDLE state
          while (digitalRead(START_BUTTON_PIN) == LOW); // Wait for button release
        }
      }
      break;
  }
}