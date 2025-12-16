#include <Arduino.h>

const int numSensors = 8;
int sensorPins[numSensors] = {A7, A6, A5, A4, A3, A2, A1, A0};

// Arrays to hold min/max values from calibration
int minWhite[numSensors];
int maxWhite[numSensors];
int minBlack[numSensors];
int maxBlack[numSensors];

// Final thresholds
int threshold[numSensors];

void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();
  }
}

void calibrateSurface(const char* label, int minArr[], int maxArr[]) {
  Serial.print("Place robot on ");
  Serial.print(label);
  Serial.println(" surface, press any key to start...");
  while (!Serial.available()) {}
  clearSerialBuffer();

  // Initialize min/max
  for (int i = 0; i < numSensors; i++) {
    minArr[i] = 1023;
    maxArr[i] = 0;
  }

  unsigned long start = millis();
  while (millis() - start < 5000) { // 5 seconds sampling
    for (int i = 0; i < numSensors; i++) {
      float val = analogRead(sensorPins[i]);
      if (val < minArr[i]) minArr[i] = val;
      if (val > maxArr[i]) maxArr[i] = val;
    }
  }

  Serial.print(label);
  Serial.println(" run complete.");
  for (int i = 0; i < numSensors; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": min=");
    Serial.print(minArr[i]);
    Serial.print(" max=");
    Serial.println(maxArr[i]);
  }
  Serial.println("-------------------------------");
}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Step 1: White run
  calibrateSurface("WHITE", minWhite, maxWhite);
  clearSerialBuffer();
  // Step 2: Black run
  calibrateSurface("BLACK", minBlack, maxBlack);
  clearSerialBuffer();

  // Step 3: Compute thresholds = average of maxWhite and minBlack
  for (int i = 0; i < numSensors; i++) {
    threshold[i] = (((maxWhite[i] + minBlack[i]) / 2)+minBlack[i])/2;
    //threshold[i] = max(maxWhite[i], minBlack[i]);
  }

  Serial.println("Final Thresholds:");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(threshold[i]);
    if (i<numSensors-1){
      Serial.print(", ");
    }
  }
  Serial.println();
  Serial.println("Calibration complete.");

  Serial.println("Full surface, press any key to start...");
  while (!Serial.available()) {}

}

void loop() {
  // Test loop: print binary sensor states
  for (int i = 0; i < numSensors; i++) {
    int val = analogRead(sensorPins[i]);
    if (val <= threshold[i]) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
    Serial.print(" ");
  }
  Serial.println();
  delay(200);
}

