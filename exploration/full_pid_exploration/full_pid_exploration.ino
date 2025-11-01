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
float threshold[numSensors] = {940, 902, 883, 874, 851, 863, 862, 866};
int weight[numSensors] = {-8, -4, -2, -1, 1, 2, 4, 8};

// PID constants — tune these
float Kp = 40.0;
float Ki = 0.0;
float Kd = 25.0;

float integral = 0;
float previousError = 0;

void setMotor(int pin1, int pin2, int pwm, int speed){
    speed = constrain(speed, -80, 80);
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

bool turning = false;
unsigned long turnStartTime = 0;

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

  if (activeCount == 0) {
    motor1run(0);
    motor2run(0);
    return;
  }

  bool leftExtreme = (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0);
  bool rightExtreme = (sensor[7] == 1 && sensor[6] == 0 && sensor[5] == 0);

  // --- Delayless Spin Logic ---
  if (leftExtreme) {
    
    Serial.println("Left Turn");
    currNode = count++;
    for(int i=0; i<=count; i++){
      if(nodes[i][1]<=(x+30) && nodes[i][1]>=(x-30) && nodes[i][2]<=(y+30) && nodes[i][2]>=(y-30)){
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

    //Serial.println("Hard left turn (delayless)");
    // Spin left until center sensors detect line again
    while (true) {
      motor1run(-100);
      motor2run(100);

      int midLeft = analogRead(sensorPins[3]);
      int midRight = analogRead(sensorPins[4]);

      if ((isBlackLine && midLeft >= threshold[3] && midRight >= threshold[4]) ||
          (!isBlackLine && midLeft < threshold[3] && midRight < threshold[4])) {
        // Line reacquired
        break;
      }
    }
    motor1run(0);
    motor2run(0);
    delay(20);
    return;
  }

  else if (rightExtreme) {
    Serial.println("Right turn");
    currNode = count++;
    for(int i=0; i<=count; i++){
      if(nodes[i][1]<=(x+45) && nodes[i][1]>=(x-30) && nodes[i][2]<=(y+45) && nodes[i][2]>=(y-30)){
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

    //Serial.println("Hard right turn (delayless)");
    // Spin right until center sensors detect line again
    while (true) {
      motor1run(100);
      motor2run(-100);

      int midLeft = analogRead(sensorPins[3]);
      int midRight = analogRead(sensorPins[4]);

      if ((isBlackLine && midLeft >= threshold[3] && midRight >= threshold[4]) ||
          (!isBlackLine && midLeft < threshold[3] && midRight < threshold[4])) {
        // Line reacquired
        break;
      }
    }
    motor1run(0);
    motor2run(0);
    delay(20);
    return;
  }

  // --- Normal PID Control ---
  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - previousError;
  previousError = error;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int baseSpeed = 200;
  int leftSpeed = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);

  motor1run(leftSpeed);
  motor2run(rightSpeed);
  dist++;
  if (dir==0){
    y++;
  }else if(dir==1){
    x++;
  }else if(dir==2){
    y--;
  }else{
    x--;
  }
}
