//--------Enter Line Details here---------
bool isBlackLine = 1;             // 1 = black line, 0 = white line
unsigned int numSensors = 8;
//-----------------------------------------

int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];

void setup() {
  Serial.begin(9600);

  // Calibrate sensors first
  calibrate();
}

void loop() {
  readLine();

  // Print sensor array (0 or 1)
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensorArray[i]);
    Serial.print(" ");
  }
  Serial.println();

  delay(100); // small delay for readability
}

void calibrate() {
  // Initialize min and max values
  for (int i = 0; i < numSensors; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  // Do a rough calibration (just take multiple samples)
  for (int j = 0; j < 500; j++) {
    for (int i = 0; i < numSensors; i++) {
      int val = analogRead(i);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
  }

  // Compute thresholds
  for (int i = 0; i < numSensors; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
  }
}

void readLine() {
  for (int i = 0; i < numSensors; i++) {
    if (isBlackLine) {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500; // binary 0 or 1
  }
}
