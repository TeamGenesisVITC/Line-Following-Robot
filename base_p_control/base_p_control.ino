#include <Arduino.h> 

// Motor pins
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10

const int numSensors = 8; 
int sensorPins[numSensors] = {A7, A6, A5, A4, A3, A2, A1, A0}; 

bool isBlackLine = 1; 
float threshold[numSensors] = {864, 846, 839, 837, 824, 809, 794, 806}; 
int weight[numSensors] = {-8, -4, -2, -1, 1, 2, 4, 8};

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
  delay(5000);
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
  
  float correction = map(error, -15, 15, -80, 80);
  correction = abs(correction);
  Serial.println(error);

  if (error<0){
    // if (error<-8){
    //   motor1run(0);
    //   Serial.print("Left motor: "); Serial.print(0);
    //   motor2run(80);
    //   Serial.print(" Right motor: "); Serial.print(80);
    //   Serial.println();
    // }
    // else{
    //   motor1run(60-correction);
    //   Serial.print("Left motor: "); Serial.print(60-correction);
    //   motor2run(60);
    //   Serial.print(" Right motor: "); Serial.print(60);
    //   Serial.println();
    // }
    if (error<=-6){
      delay(200);
    }
    motor1run(60-correction);
    Serial.print("Left motor: "); Serial.print(80-correction);
    motor2run(60);
    Serial.print(" Right motor: "); Serial.print(80);
    Serial.println();
    if (error<=-6){
      delay(800);
    }
  }else{
    // if (error>8){
    //   motor2run(0);
    //   Serial.print("Left motor: "); Serial.print(80);
    //   motor1run(80);
    //   Serial.print(" Right motor: "); Serial.print(0);
    //   Serial.println();
    // }
    // else{
    //   motor2run(60-correction);
    //   Serial.print("Left motor: "); Serial.print(60);
    //   motor1run(60);
    //   Serial.print(" Right motor: "); Serial.print(60-correction);
    //   Serial.println();
    // }
    if (error>=6){
      delay(200);
    }
    motor2run(60-correction);
    Serial.print("Left motor: "); Serial.print(80);
    motor1run(60);
    Serial.print(" Right motor: "); Serial.print(80-correction);
    Serial.println();
    if (error>=6){
      delay(800);
    }
  }
  delay(100);

}