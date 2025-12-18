#include <Arduino.h>

// DRV8833 Motor pins (2-wire control per motor)
#define L_IN1 4   // Left motor
#define L_IN2 5
#define R_IN1 10  // Right motor
#define R_IN2 9

// IR sensor array (8 sensors)
const int numSensors = 8;
int sensorPins[numSensors] = {A9, A8, A7, A6, A5, A4, A3, A2};

bool isBlackLine = 0;
float threshold[numSensors] = {1640, 1481, 1509, 1643, 1450, 1694, 2127, 1966};
int weight[numSensors] = {-8, -4, -2, -1, 1, 2, 4, 8};

// PID constants — tune these
float Kp = 40.0;
float Ki = 0.0;
float Kd = 25.0;

float integral = 0;
float previousError = 0;

// DRV8833 motor control: speed range -255 to 255
void setMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    // Forward
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  } else if (speed < 0) {
    // Reverse
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  } else {
    // Brake
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void motor1run(int speed) { setMotor(L_IN1, L_IN2, speed); }
void motor2run(int speed) { setMotor(R_IN1, R_IN2, speed); }

void setup() {
  Serial.begin(9600);
  
  // Configure ADC to match calibration code
  analogReadResolution(12);  // 12-bit: 0-4095 range
  analogReadAveraging(8);    // Average 8 samples for stability
  
  // Setup sensor pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  // Setup motor pins
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  
  // Initial brake
  motor1run(0);
  motor2run(0);
  
  delay(3000);  // Startup delay
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
    Serial.print(val);
    Serial.print(' ');
    sensor[i] = (isBlackLine) ? (val >= threshold[i]) : (val < threshold[i]);
    if (sensor[i]) activeCount++;
    error += sensor[i] * weight[i];
  }

  // Print binary sensor states
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensor[i]);
    Serial.print(' ');
  }
  Serial.println();

  // Lost line - stop
  if (activeCount == 0) {
    motor1run(0);
    motor2run(0);
    return;
  }

  bool leftExtreme = (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0);
  bool rightExtreme = (sensor[7] == 1 && sensor[6] == 0 && sensor[5] == 0);

  // --- LEFT TURN DETECTION ---
  if (leftExtreme) {
    unsigned long now = millis();
    dist = now - prev;
    prev = now;

    // Update position based on direction
    if (dir == 0) {
      y += dist;
    } else if (dir == 1) {
      x += dist;
    } else if (dir == 2) {
      y -= dist;
    } else {
      x -= dist;
    }

    Serial.println("Left Turn");
    currNode = count++;
    
    // Check if this location already exists as a node
    for (int i = 0; i <= count; i++) {
      if (nodes[i][0] <= (x + 30) && nodes[i][0] >= (x - 30) && 
          nodes[i][1] <= (y + 30) && nodes[i][1] >= (y - 30)) {
        currNode = i;
        count--;
        found = true;
        break;
      }
    }

    // New node
    if (!found) {
      nodes[currNode][0] = x;
      nodes[currNode][1] = y;
    }

    // Update graph if moved to new node
    if (lastNode != currNode) {
      Serial.print("(x, y) : ");
      Serial.print(x);
      Serial.print(" ");
      Serial.print(y);
      Serial.print(" ");
      Serial.print(dir);
      Serial.println();

      Serial.print("Node ");
      Serial.println(currNode);

      matrix[lastNode][currNode] = dist;
      Serial.print(lastNode);
      Serial.print(" ");
      Serial.print(currNode);
      Serial.print(" ");
      Serial.println(dist);

      lastNode = currNode;
      dist = 0;
      dir = (dir - 1) % 4;
      if (dir == -1) {
        dir = 3;
      }
    }

    // Execute hard left turn - spin until line reacquired
    while (true) {
      motor1run(-100);  // Left motor reverse
      motor2run(100);   // Right motor forward

      int midLeft = analogRead(sensorPins[3]);
      int midRight = analogRead(sensorPins[4]);

      // Check if center sensors detect line
      if ((isBlackLine && midLeft >= threshold[3] && midRight >= threshold[4]) ||
          (!isBlackLine && midLeft < threshold[3] && midRight < threshold[4])) {
        break;
      }
    }
    
    motor1run(0);
    motor2run(0);
    delay(20);
    return;
  }

  // --- RIGHT TURN DETECTION ---
  else if (rightExtreme) {
    unsigned long now = millis();
    dist = now - prev;
    prev = now;

    // Update position based on direction
    if (dir == 0) {
      y += dist;
    } else if (dir == 1) {
      x += dist;
    } else if (dir == 2) {
      y -= dist;
    } else {
      x -= dist;
    }

    Serial.println("Right turn");
    currNode = count++;
    
    // Check if this location already exists as a node
    for (int i = 0; i <= count; i++) {
      if (nodes[i][0] <= (x + 45) && nodes[i][0] >= (x - 30) && 
          nodes[i][1] <= (y + 45) && nodes[i][1] >= (y - 30)) {
        currNode = i;
        count--;
        found = true;
        break;
      }
    }

    // New node
    if (!found) {
      nodes[currNode][0] = x;
      nodes[currNode][1] = y;
    }

    // Update graph if moved to new node
    if (lastNode != currNode) {
      Serial.print("(x, y, d) : ");
      Serial.print(x);
      Serial.print(" ");
      Serial.print(y);
      Serial.print(" ");
      Serial.print(dir);
      Serial.println();

      Serial.print("Node ");
      Serial.println(currNode);

      matrix[lastNode][currNode] = dist;
      Serial.print(lastNode);
      Serial.print(" ");
      Serial.print(currNode);
      Serial.print(" ");
      Serial.println(dist);
      
      lastNode = currNode;
      dist = 0;
      dir = (dir + 1) % 4;
    }

    // Execute hard right turn - spin until line reacquired
    while (true) {
      motor1run(100);   // Left motor forward
      motor2run(-100);  // Right motor reverse

      int midLeft = analogRead(sensorPins[3]);
      int midRight = analogRead(sensorPins[4]);

      // Check if center sensors detect line
      if ((isBlackLine && midLeft >= threshold[3] && midRight >= threshold[4]) ||
          (!isBlackLine && midLeft < threshold[3] && midRight < threshold[4])) {
        break;
      }
    }
    
    motor1run(0);
    motor2run(0);
    delay(20);
    return;
  }

  // --- NORMAL PID LINE FOLLOWING ---
  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - previousError;
  previousError = error;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Base speed and differential steering
  int baseSpeed = 100;  // Adjusted for DRV8833 (was 200 for L298N)
  int leftSpeed = constrain(baseSpeed + correction, -255, 255);
  int rightSpeed = constrain(baseSpeed - correction, -255, 255);

  motor1run(leftSpeed);
  motor2run(rightSpeed);
}