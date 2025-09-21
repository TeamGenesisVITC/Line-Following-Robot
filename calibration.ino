#include <Arduino.h>

// ===================== Motor Pins =====================
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10

// ===================== Sensor Pins =====================
const int NUM_SENSORS = 8;           // change this if you have fewer/more sensors
int sensors[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};

int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
int threshold[NUM_SENSORS];

// ===================== Motor Helper =====================
void setMotor(int pin1, int pin2, int pwm, int speed) {
  speed = constrain(speed, 0, 255);
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

// ===================== Setup =====================
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);

  // Init min/max
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  // Added starting delay
  Serial.println("Waiting 5 seconds before starting calibration...");
  delay(5000);

  Serial.println("Starting calibration... Motor will run for 7 seconds.");
}

// ===================== Loop =====================
void loop() {
  unsigned long startTime = millis();

  // Run only once
  while (millis() - startTime < 7000) {
    // Run motor 1 (you can change to motor2run)
    motor1run(150);  
    motor2run(0);

    // Print sensor readings in array format
    Serial.print("Sensor values: [");
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensors[i]);
      Serial.print(val);
      if (i < NUM_SENSORS - 1) Serial.print(", ");

      // Update min and max
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
    Serial.println("]");

    delay(100); // prevent spamming too fast
  }

  // Stop motors after calibration
  motor1run(0);
  motor2run(0);

  // Calculate thresholds
  for (int i = 0; i < NUM_SENSORS; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
  }

  // Print results
  Serial.println("Calibration complete.");
  Serial.println("Min Values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(minValues[i]); Serial.print(" ");
  }
  Serial.println();

  Serial.println("Max Values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(maxValues[i]); Serial.print(" ");
  }
  Serial.println();

  Serial.println("Thresholds:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(threshold[i]); Serial.print(" ");
  }
  Serial.println();

  while (1); // stop forever after calibration
}




