#include <Arduino.h>

// Motor pins (Teensy 4.0)
#define AIN1 4
#define AIN2 5
#define BIN1 10
#define BIN2 9
#define PWMA 4
#define PWMB 10

const int numSensors = 8;
int sensorPins[numSensors] = {
  A9, A8, A7, A6, A5, A4, A3, A2
};

void setMotor(int pin1, int pin2, int pwm, int speed){
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

void motor1run(int speed){ setMotor(AIN1, AIN2, PWMA, speed); }
void motor2run(int speed){ setMotor(BIN1, BIN2, PWMB, speed); }

int minWhite[numSensors];
int maxWhite[numSensors];
int minBlack[numSensors];
int maxBlack[numSensors];
int threshold[numSensors];

void clearSerialBuffer() {
  while (Serial.available()) Serial.read();
}

void calibrateSurface(const char* label, int minArr[], int maxArr[]) {
  Serial.print("Place robot on ");
  Serial.print(label);
  Serial.println(" surface, press any key...");

  while (!Serial && millis() < 5000) {}

  unsigned long t0 = millis();
  while (!Serial.available() && millis() - t0 < 30000) {}

  clearSerialBuffer();

  for (int i = 0; i < numSensors; i++) {
    minArr[i] = 4095;
    maxArr[i] = 0;
  }

  unsigned long start = millis();
  while (millis() - start < 5000) {
    for (int i = 0; i < numSensors; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < minArr[i]) minArr[i] = val;
      if (val > maxArr[i]) maxArr[i] = val;
    }
  }

  Serial.print(label);
  Serial.println(" run complete.");

  for (int i = 0; i < numSensors; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(minArr[i]);
    Serial.print(" / ");
    Serial.println(maxArr[i]);
  }
  Serial.println();
}


void setup() {
  Serial.begin(9600);

  analogReadResolution(12);
  analogReadAveraging(8);

  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  calibrateSurface("WHITE", minWhite, maxWhite);
  calibrateSurface("BLACK", minBlack, maxBlack);

  for (int i = 0; i < numSensors; i++) {
    threshold[i] = (((maxWhite[i] + minBlack[i]) / 2) + minBlack[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.println("Calibration complete.");
  Serial.println("Press any key to start...");
  while (!Serial && millis() < 5000) {}

}

void loop() {
  motor1run(130);
  motor2run(130);

  for (int i = 0; i < numSensors; i++) {
    int val = analogRead(sensorPins[i]);
    Serial.print(val <= threshold[i] ? "1 " : "0 ");
  }
  Serial.println();
  delay(200);
}
