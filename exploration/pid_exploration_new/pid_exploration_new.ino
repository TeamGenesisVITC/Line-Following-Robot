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
float threshold[numSensors] = {974, 957, 950, 945, 934, 925, 909, 915};
int weight[numSensors] = {-8, -4, -2, -1, 1, 2, 4, 8};

float Kp = 40.0;
float Ki = 0.0;
float Kd = 25.0;

float integral = 0;
float previousError = 0;

void setMotor(int pin1, int pin2, int pwm, int speed)
{
  speed = constrain(speed, -80, 80);
  if (speed > 0)
  {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pwm, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(pwm, -speed);
  }
  else
  {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    analogWrite(pwm, 0);
  }
}

void motor1run(int speed) { setMotor(AIN1, AIN2, PWMA, speed); }
void motor2run(int speed) { setMotor(BIN1, BIN2, PWMB, speed); }

void setup()
{
  Serial.begin(9600);
  for (int i = 0; i < numSensors; i++)
    pinMode(sensorPins[i], INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  delay(3000);
}

bool choice[3];

void loop()
{
  choice[0] = false;
  choice[1] = false;
  choice[2] = false;
  int sensor[numSensors];
  float error = 0.0;
  int activeCount = 0;
  bool found = false;

  // Read all sensors
  for (int i = 0; i < numSensors; i++)
  {
    int val = analogRead(sensorPins[i]);
    sensor[i] = (isBlackLine) ? (val > threshold[i]) : (val <= threshold[i]);
    Serial.print(val);
    Serial.print(" ");
    if (sensor[i])
      activeCount++;
    error += sensor[i] * weight[i];
  }
  Serial.println();

  if (activeCount == 0)
  {
    motor1run(0);
    motor2run(-0); // slow spin search
    delay(20);
    return;
  }

  choice[0] = (sensor[0] || sensor[1] || sensor[2]);
  choice[1] = (sensor[2] || sensor[3] || sensor[4] || sensor[5]);
  choice[2] = (sensor[5] || sensor[6] || sensor[7]);

  if (choice[0] || choice[1])
  {
    delay(300);

    for (int i = 0; i < numSensors; i++)
    {
      int val = analogRead(sensorPins[i]);
      sensor[i] = isBlackLine ? (val > threshold[i]) : (val <= threshold[i]);
    }

    choice[1] = (sensor[2] || sensor[3] || sensor[4] || sensor[5]);
  }

  // for (int i = 0; i < numSensors; i++)
  // {
  //   Serial.print(sensor[i]);
  //   Serial.print(' ');
  // }

  // Serial.print('|');

  // for (int i = 0; i < 3; i++)
  // {
  //   Serial.print(choice[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  if (choice[0] && false)
  {
    unsigned long start = millis();
    while (millis() - start < 800)
    { // 800 ms timeout
      motor1run(-80);
      motor2run(80);

      int midLeft = analogRead(sensorPins[3]);
      int midRight = analogRead(sensorPins[4]);

      bool onLine =
          isBlackLine ? (midLeft > threshold[3] || midRight > threshold[4]) : (midLeft <= threshold[3] || midRight <= threshold[4]);

      if (onLine)
        break;
    }

    // fallback
    motor1run(60);

    motor2run(60);
    // delay(40);
  }

  else if (choice[2] && false)
  {
    unsigned long start = millis();
    while (millis() - start < 800)
    { // 800 ms timeout
      motor1run(80);
      motor2run(-80);

      int midLeft = analogRead(sensorPins[3]);
      int midRight = analogRead(sensorPins[4]);

      bool onLine =
          isBlackLine ? (midLeft > threshold[3] || midRight > threshold[4]) : (midLeft <= threshold[3] || midRight <= threshold[4]);

      if (onLine)
        break;
    }

    // fallback
    motor1run(60);
    motor2run(60);
    // delay(40);
  }

  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - previousError;
  previousError = error;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -80, 80);

  int baseSpeed = 80;
  int leftSpeed = constrain(baseSpeed + correction, 0, 255);
  int rightSpeed = constrain(baseSpeed - correction, 0, 255);

  motor1run(leftSpeed);
  motor2run(rightSpeed);
  Serial.print("Speeds: ");
  Serial.print(leftSpeed);
  Serial.print(" | ");
  Serial.print(rightSpeed);
  Serial.println();
}