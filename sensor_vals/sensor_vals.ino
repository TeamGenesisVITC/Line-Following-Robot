#include <Arduino.h> 

// Number of sensors 
const int numSensors = 8; 

// Analog pins connected to sensors 
int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7}; 

// Variables to store readings 
int sensorValue[numSensors]; 
int sensorActive[numSensors]; 

// Threshold for detecting the line 
int threshold[numSensors] = {30, 30, 30, 30, 30, 30, 30, 30}; 

// Set to 1 if line is black, 0 if line is white 
bool isBlackLine = 1; 

void setup() { 
  Serial.begin(9600); // Initialize serial communication 
  for (int i = 0; i < numSensors; i++) { 
    pinMode(sensorPins[i], INPUT); 
  } 
} 

void loop() { 
  Serial.print("Sensors: "); 
  for (int i = 0; i < 8; i++) { 
    int val = analogRead(sensorPins[i]); // read sensor 
    Serial.print(val);
    Serial.print(" ");
  } 
  Serial.println(); 
  delay(200); 
} 
