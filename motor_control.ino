#include <Arduino.h>

// Motor pins
#define AIN1 4
#define AIN2 3
#define BIN1 6
#define BIN2 7
#define PWMA 9
#define PWMB 10

// Motor helper
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
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  Serial.println("Enter a value 0.0-1.0 for motor speed, 2 to stop:");
}

bool toggleMotor = true; // true = right, false = left

void loop() {
  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    input.trim();
    float val = input.toFloat();

    if(val == 2.0){
      motor1run(0);
      motor2run(0);
      Serial.println("Motors stopped.");
      while(1); // stop forever
    }

    int speed = int(val * 100); // convert 0.0-1.0 to 0-100 scale
    speed = map(speed, 0, 100, 0, 255); // scale to PWM 0-255

    if(toggleMotor){
      motor2run(speed); // right motor
      motor1run(0);
      Serial.print("Right motor: "); Serial.println(speed);
    } else {
      motor1run(speed); // left motor
      motor2run(0);
      Serial.print("Left motor: "); Serial.println(speed);
    }
    delay(5000);
    motor1run(0);
    motor2run(0);
    toggleMotor = !toggleMotor; // switch motor next time
  }
}
