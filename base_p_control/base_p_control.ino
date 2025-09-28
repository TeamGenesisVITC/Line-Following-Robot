#include <Arduino.h> 

// Motor pins
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10

const int numSensors = 8; 
int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7}; 

bool isBlackLine = 1; 
float threshold[numSensors] = {26, 26, 26, 26, 25, 25, 25, 24}; 
int weight[numSensors] = {-4, -3, -2, -1, 1, 2, 3, 4};

void clearSerialBuffer() {
  while (Serial.available()) {
    Serial.read();
  }
}

void setMotor(int pin1, int pin2, int pwm, int speed){
    speed = constrain(speed, 0, 255); // only forward for now
    if(speed > 0){
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

void setup() { 
  Serial.begin(9600); 
  for (int i = 0; i < numSensors; i++) { 
    pinMode(sensorPins[i], INPUT); 
  }
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
}

void loop() { 
  
  int sensor[numSensors];
  float error = 0.0;

  for (int i = 0; i < 8; i++) { 
      int val = analogRead(sensorPins[i]);   
      if (val >= threshold[i]) { 
        Serial.print("1 "); 
        sensor[i]=1;
      } else { 
        Serial.print("0 "); 
        sensor[i]=0;
      } 
      error+=sensor[i]*weight[i];
  }
  Serial.println();
  
  float correction = map(error, -24, 24, 0, 255);

  if (error<0){
    motor2run(255-correction);
    motor1run(255);
    Serial.println();
  }else{
    motor1run(255-correction);
    motor2run(255);
  }

}