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

bool isBlackLine = 0;
float threshold[numSensors] = {919, 903, 898, 897, 879, 873, 875, 875};
int weight[numSensors] = {-8, -4, -2, -1, 1, 2, 4, 8};

// PID constants — tune these
float Kp = 40.0;
float Ki = 0.0;
float Kd = 25.0;

float integral = 0;
float previousError = 0;

// TURNING STATE VARIABLES
bool isTurning = false;
int turnDirection = 0; // -1 = left, 1 = right, 0 = not turning

void setMotor(int pin1, int pin2, int pwm, int speed){
    speed = constrain(speed, -130, 130);
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

void motor1run(int speed){ setMotor(AIN1, AIN2, PWMA, speed); }
void motor2run(int speed){ setMotor(BIN1, BIN2, PWMB, speed); }

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numSensors; i++) pinMode(sensorPins[i], INPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  delay(3000);
}

int count = 0;
int lastNode = 0;
int currNode = 0;
int dist = 0;
int dir = 0;
int x = 0;
int y = 0;
int nodes[150][2];
int matrix[10][10];

unsigned long prev = millis();

void loop() {
  int sensor[numSensors];
  float error = 0.0;
  int activeCount = 0;
  bool found = false;

  // Read all sensors
  for (int i = 0; i < numSensors; i++) {
    int val = analogRead(sensorPins[i]);
    sensor[i] = (isBlackLine) ? (val >= threshold[i]) : (val < threshold[i]);
    if (sensor[i]) activeCount++;
    error += sensor[i] * weight[i];
  }

  // Check middle sensors for line presence
  bool middleOnLine = (sensor[3] == 1 || sensor[4] == 1);

  bool leftExtreme = (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0);
  bool rightExtreme = (sensor[7] == 1 && sensor[6] == 0 && sensor[5] == 0);

  // --- IF CURRENTLY TURNING, KEEP TURNING UNTIL MIDDLE SENSORS DETECT LINE ---
  if (isTurning) {
    if (turnDirection == -1) {
      // Turning left
      motor1run(-130);
      motor2run(130);
    } else if (turnDirection == 1) {
      // Turning right
      motor1run(130);
      motor2run(-130);
    }

    // Check if turn is complete (middle sensors back on line)
    int midLeft = analogRead(sensorPins[3]);
    int midRight = analogRead(sensorPins[4]);
    
    if ((isBlackLine && midLeft >= threshold[3] && midRight >= threshold[4]) ||
        (!isBlackLine && midLeft < threshold[3] && midRight < threshold[4])) {
      // Turn complete
      isTurning = false;
      turnDirection = 0;
      motor1run(0);
      motor2run(0);
      delay(20);
    }
    return; // Skip rest of loop while turning
  }

  // --- DETECT START OF TURN ---
  if (leftExtreme) {

    unsigned long now = millis();
    dist = now-prev;
    prev = now;

    if (dir==0){
      y+=dist;
    }else if(dir==1){
      x+=dist;
    }else if(dir==2){
      y-=dist;
    }else{
      x-=dist;
    }

    Serial.println("Left Turn");
    currNode = count++;
    for(int i=0; i<=count; i++){
      if(nodes[i][0]<=(x+500) && nodes[i][0]>=(x-500) && nodes[i][1]<=(y+500) && nodes[i][1]>=(y-500)){
        currNode = i;
        count--;
        found = true;
        break;
      }
    }

    if(!found){
      nodes[currNode][0] = x;
      nodes[currNode][1] = y;
    }

    if(lastNode!=currNode){
      Serial.print("(x, y) : ");
      Serial.print(x);
      Serial.print(" ");
      Serial.print(y);
      Serial.print(" ");
      Serial.print(dir);
      Serial.println();

      Serial.print("Node ");
      Serial.print(currNode);
      Serial.println();

      matrix[lastNode][currNode] = dist;
      Serial.print(lastNode);
      Serial.print(" ");
      Serial.print(currNode);
      Serial.print(" ");
      Serial.print(dist);
      Serial.println();

      lastNode = currNode;
      dist = 0;
      dir = (dir-1)%4;
      if(dir==-1){
        dir=3;
      }
    }

    // Set turning state
    isTurning = true;
    turnDirection = -1; // Left turn
    return;
  }

  else if (rightExtreme) {

    unsigned long now = millis();
    dist = now-prev;
    prev = now;

    if (dir==0){
      y+=dist;
    }else if(dir==1){
      x+=dist;
    }else if(dir==2){
      y-=dist;
    }else{
      x-=dist;
    }

    Serial.println("Right turn");
    currNode = count++;
    for(int i=0; i<=count; i++){
      if(nodes[i][0]<=(x+500) && nodes[i][0]>=(x-500) && nodes[i][1]<=(y+500) && nodes[i][1]>=(y-500)){
        currNode = i;
        count--;
        found = true;
        break;
      }
    }

    if(!found){
      nodes[currNode][0] = x;
      nodes[currNode][1] = y;
    }

    if(lastNode!=currNode){
      Serial.print("(x, y, d) : ");
      Serial.print(x);
      Serial.print(" ");
      Serial.print(y);
      Serial.print(" ");
      Serial.print(dir);
      Serial.println();

      Serial.print("Node ");
      Serial.print(currNode);
      Serial.println();

      matrix[lastNode][currNode] = dist;
      Serial.print(lastNode);
      Serial.print(" ");
      Serial.print(currNode);
      Serial.print(" ");
      Serial.print(dist);
      Serial.println();
      lastNode = currNode;
      dist = 0;
      dir = (dir+1)%4;
    }

    // Set turning state
    isTurning = true;
    turnDirection = 1; // Right turn
    return;
  }

  // --- ONLY RUN IF ERROR IS NOT ZERO OR MIDDLE SENSORS ARE ON LINE ---
  if (activeCount == 0 || (error == 0 && !middleOnLine)) {
    motor1run(0);
    motor2run(0);
    return;
  }

  // --- Normal PID Control ---
  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - previousError;
  previousError = error;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int baseSpeed = 130;
  int leftSpeed = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);

  motor1run(leftSpeed);
  motor2run(rightSpeed);
}