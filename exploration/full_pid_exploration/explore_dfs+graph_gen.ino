#include <Arduino.h>

/* ================= USER EDIT ================= */
#define ENCODER_PIN 2        // MUST be 2 or 3
#define CALIB_BUTTON 11
#define START_BUTTON 12

#define MAX_NODES 50
#define NUM_SENSORS 8
#define CALIBRATION_SPEED 120
/* ============================================ */

// ---------------- MOTOR PINS ----------------
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10

// ---------------- SENSOR CONFIG ----------------
int sensorPins[NUM_SENSORS] = {A7,A6,A5,A4,A3,A2,A1,A0};

bool isBlackLine = 1;
float threshold[NUM_SENSORS] = {940,902,883,874,851,863,862,866};
int weight[NUM_SENSORS] = {-8,-4,-2,-1,1,2,4,8};

// ---------------- PID CONSTANTS ----------------
float Kp = 40.0;
float Ki = 0.0;
float Kd = 25.0;

float integral = 0;
float previousError = 0;

// ---------------- ENCODER ----------------
volatile uint16_t encoderCount = 0;
void encoderISR() { encoderCount++; }

// ---------------- GRAPH STRUCTURES ----------------
struct Edge {
  uint16_t dist;
  uint8_t  mask;
};

Edge graph[MAX_NODES][4];
uint16_t nodes[MAX_NODES][2];

uint8_t nodeCount = 0;
uint8_t lastNode = 0;
uint8_t currNode = 0;

// direction: 0=E,1=S,2=W,3=N
uint8_t dir = 0;

// virtual position
uint16_t x = 0;
uint16_t y = 0;

// mask bits
#define BIT_N 1
#define BIT_E 2
#define BIT_S 4
#define BIT_W 8

// ---------------- STATE ----------------
bool calibrated = false;
bool started = false;

// ---------------- MOTOR CONTROL ----------------
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

// ---------------- TURN MASK (ONLY ADDITION) ----------------
uint8_t computeTurnMask(int sensor[]) {
  uint8_t mask = 0;

  // forward
  if (sensor[3] || sensor[4]) {
    if (dir == 0) mask |= BIT_E;
    else if (dir == 1) mask |= BIT_S;
    else if (dir == 2) mask |= BIT_W;
    else mask |= BIT_N;
  }

  // left
  if (sensor[0]) {
    uint8_t d = (dir + 3) % 4;
    if (d == 0) mask |= BIT_E;
    else if (d == 1) mask |= BIT_S;
    else if (d == 2) mask |= BIT_W;
    else mask |= BIT_N;
  }

  // right
  if (sensor[7]) {
    uint8_t d = (dir + 1) % 4;
    if (d == 0) mask |= BIT_E;
    else if (d == 1) mask |= BIT_S;
    else if (d == 2) mask |= BIT_W;
    else mask |= BIT_N;
  }

  return mask;
}

// ---------------- CALIBRATION ----------------
void runCalibration() {
  int minValues[NUM_SENSORS];
  int maxValues[NUM_SENSORS];

  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  motor1run(-CALIBRATION_SPEED);
  motor2run(CALIBRATION_SPEED);

  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
  }

  motor1run(0);
  motor2run(0);

  for (int i = 0; i < NUM_SENSORS; i++)
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(sensorPins[i], INPUT);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  pinMode(CALIB_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);

  nodes[0][0] = 0;
  nodes[0][1] = 0;
  nodeCount = 1;

  for (int d = 0; d < 4; d++) {
    graph[0][d].dist = 0;
    graph[0][d].mask = 0;
  }
}

// ---------------- LOOP ----------------
void loop() {

  if (!calibrated && digitalRead(CALIB_BUTTON) == LOW) {
    delay(200);
    runCalibration();
    calibrated = true;
  }

  if (calibrated && !started && digitalRead(START_BUTTON) == LOW) {
    delay(200);
    encoderCount = 0;
    started = true;
  }

  if (!started) {
    motor1run(0);
    motor2run(0);
    return;
  }

  int sensor[NUM_SENSORS];
  float error = 0;
  int activeCount = 0;
  bool found = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = analogRead(sensorPins[i]);
    sensor[i] = isBlackLine ? (val >= threshold[i]) : (val < threshold[i]);
    if (sensor[i]) activeCount++;
    error += sensor[i] * weight[i];
  }

  if (activeCount == 0) return;

  bool leftExtreme  = (sensor[0] && !sensor[1] && !sensor[2]);
  bool rightExtreme = (sensor[7] && !sensor[6] && !sensor[5]);

  // ---------------- LEFT NODE ----------------
  if (leftExtreme) {

    uint16_t dist = encoderCount;
    encoderCount = 0;

    if (dir==0) y+=dist;
    else if(dir==1) x+=dist;
    else if(dir==2) y-=dist;
    else x-=dist;

    currNode = nodeCount++;
    for(uint8_t i=0;i<nodeCount;i++){
      if(nodes[i][0]>=(x-30)&&nodes[i][0]<=(x+30)&&
         nodes[i][1]>=(y-30)&&nodes[i][1]<=(y+30)){
        currNode=i;
        nodeCount--;
        found=true;
        break;
      }
    }

    if(!found){
      nodes[currNode][0]=x;
      nodes[currNode][1]=y;
      for(int d=0;d<4;d++){
        graph[currNode][d].dist=0;
        graph[currNode][d].mask=0;
      }
    }

    if(lastNode!=currNode){
      uint8_t mask = computeTurnMask(sensor);

      graph[lastNode][dir].dist = dist;
      graph[lastNode][dir].mask = mask;

      graph[currNode][(dir+2)%4].dist = dist;
      graph[currNode][(dir+2)%4].mask = mask;

      lastNode = currNode;
      dir = (dir + 3) % 4;
    }

    while(true){
      motor1run(-100); motor2run(100);
      if(analogRead(sensorPins[3])>=threshold[3] &&
         analogRead(sensorPins[4])>=threshold[4]) break;
    }
    motor1run(0); motor2run(0);
    delay(20);
    return;
  }

  // ---------------- RIGHT NODE ----------------
  if (rightExtreme) {

    uint16_t dist = encoderCount;
    encoderCount = 0;

    if (dir==0) y+=dist;
    else if(dir==1) x+=dist;
    else if(dir==2) y-=dist;
    else x-=dist;

    currNode = nodeCount++;
    for(uint8_t i=0;i<nodeCount;i++){
      if(nodes[i][0]>=(x-30)&&nodes[i][0]<=(x+45)&&
         nodes[i][1]>=(y-30)&&nodes[i][1]<=(y+45)){
        currNode=i;
        nodeCount--;
        found=true;
        break;
      }
    }

    if(!found){
      nodes[currNode][0]=x;
      nodes[currNode][1]=y;
      for(int d=0;d<4;d++){
        graph[currNode][d].dist=0;
        graph[currNode][d].mask=0;
      }
    }

    if(lastNode!=currNode){
      uint8_t mask = computeTurnMask(sensor);

      graph[lastNode][dir].dist = dist;
      graph[lastNode][dir].mask = mask;

      graph[currNode][(dir+2)%4].dist = dist;
      graph[currNode][(dir+2)%4].mask = mask;

      lastNode = currNode;
      dir = (dir + 1) % 4;
    }

    while(true){
      motor1run(100); motor2run(-100);
      if(analogRead(sensorPins[3])>=threshold[3] &&
         analogRead(sensorPins[4])>=threshold[4]) break;
    }
    motor1run(0); motor2run(0);
    delay(20);
    return;
  }

  // ---------------- PID CONTROL ----------------
  integral += error;
  integral = constrain(integral,-50,50);
  float derivative = error - previousError;
  previousError = error;

  float correction = Kp*error + Ki*integral + Kd*derivative;

  int baseSpeed = 200;
  motor1run(constrain(baseSpeed + correction,0,255));
  motor2run(constrain(baseSpeed - correction,0,255));
}
