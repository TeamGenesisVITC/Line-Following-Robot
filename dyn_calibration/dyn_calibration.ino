#include <Arduino.h> 
 
const int numSensors = 8; 
int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7}; 
 
int sensorValue[numSensors]; 
int sensorActive[numSensors]; 

bool isBlackLine = 1; 
float threshold[numSensors] = {25, 25, 25, 25, 25, 25, 25, 25}; 

void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();
  }
}

void setup() { 
  Serial.begin(9600); 
  for (int i = 0; i < numSensors; i++) { 
    pinMode(sensorPins[i], INPUT); 
  } 


  Serial.println("Ready for White Run: ");
  while(!Serial.available()){};
  clearSerialBuffer();
  int count=0;
  int clean=0;

  do{

    bool detect = true;

    Serial.print("Thresholds: ");
    for (int i = 0; i < 8; i++) { 
      Serial.print(threshold[i]);
      Serial.print(" ");
    } 
    Serial.print("  |  ");

    Serial.print("Sensors: ");
    for (int i = 0; i < 8; i++) { 
      int val = analogRead(sensorPins[i]);   
      if (val >= threshold[i]) { 
        detect = false;
        threshold[i]+=0.2;
        Serial.print("1 "); 
      } else { 
        Serial.print("0 "); 
      } 
    } 
    if (detect){
      clean++;
    }
    else{
      clean = 0;
    }

    if (clean==10){
      break;
    }

    Serial.println(); 
    delay(200);
    count++;

  }while(count<2500);



  Serial.println("------------------------------------------------------------------------------------------");


  clearSerialBuffer();
  Serial.println("Ready for Black Run: ");
  while(!Serial.available()){};
  clearSerialBuffer();
  count=0;
  clean = 0;

  do{

    bool detect = true;

    Serial.print("Thresholds: ");
    for (int i = 0; i < 8; i++) { 
      Serial.print(threshold[i]);
      Serial.print(" ");
    } 
    Serial.print("  |  ");

    Serial.print("Sensors: ");
    for (int i = 0; i < 8; i++) { 
      int val = analogRead(sensorPins[i]);   
      if (val >= threshold[i]) { 
        Serial.print("1 "); 
      } else { 
        detect = false;
        threshold[i]-=0.20;
        Serial.print("0 "); 
      } 
    } 

    if (detect){
      clean++;
    }
    else{
      clean = 0;
    }

    if (clean==10){
      break;
    }

    Serial.println(); 
    delay(200);
    count++;

  }while(count<2500);

  Serial.println("------------------------------------------------------------------------------------------");

  clearSerialBuffer();
  Serial.println("Ready for Full Run: ");
  while(!Serial.available()){};
  clearSerialBuffer();

  Serial.print("Final Thresholds: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();
} 

void loop() { 
  
  for (int i = 0; i < 8; i++) { 
      int val = analogRead(sensorPins[i]);   
      if (val >= threshold[i]) { 
        Serial.print("1 "); 
      } else { 
        Serial.print("0 "); 
      } 
  } 
  Serial.println(); 
  delay(200);
  
} 
