#include <Arduino.h>

// ---------------- Motor pins ----------------
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10

// ---------------- Sensor pins ----------------
const int numSensors = 8;
int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ---------------- Sensor arrays ----------------
int minValues[numSensors];
int maxValues[numSensors];
int threshold[numSensors];

// ---------------- Motor helpers ----------------
void setMotor(int pin1, int pin2, int pwm, int speed) {
  speed = constrain(speed, 0, 255); // only forward for now
  if (speed > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pwm, speed);
  } else {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, HIGH);
    analogWrite(pwm, 0);
  }
}

void motor1run(int speed) { setMotor(AIN1, AIN2, PWMA, speed); }
void motor2run(int speed) { setMotor(BIN1, BIN2, PWMB, speed); }

void setup() {
  Serial.begin(9600);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);

  // Initialize min/max arrays with extreme values
  for (int i = 0; i < numSensors; i++) {
    minValues[i] = 1023;  // max possible analogRead
    maxValues[i] = 0;
  }

  Serial.println("Starting calibration: right motor at 50% for 7s...");
}

void loop() {
  int speed = 0.5 * 255;  // 50% speed (PWM ~128)
  unsigned long startTime = millis();

  // Run right motor for 7 seconds
  while (millis() - startTime < 7000) {
    motor2run(speed);
    motor1run(0);

    Serial.print("Sensors: ");
    for (int i = 0; i < numSensors; i++) {
      int val = analogRead(sensorPins[i]);

      // Update min/max
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;

      Serial.print(val);
      Serial.print(" ");
    }
    Serial.println();

    delay(100); // small delay for readability
  }

  // Stop motors after calibration
  motor1run(0);
  motor2run(0);

  // Compute thresholds
  Serial.println("Calibration finished.");
  Serial.print("Min values: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(minValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Max values: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(maxValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Thresholds: ");
  for (int i = 0; i < numSensors; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  while (1); // stop forever after calibration
}
