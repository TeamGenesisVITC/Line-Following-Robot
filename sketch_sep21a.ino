#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <Arduino.h>

//---------------- Motor pins ----------------
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define MOTOR_ENABLE 5

//---------------- User Config ----------------
bool isBlackLine = 1;          // 1=black line, 0=white line
unsigned int numSensors = 8;   // Set how many sensors you have
int sensorPins[16] = {0,1,2,3,4,5,6,7}; // Add pins for more sensors (up to 16)
int sensorThresholdDefault = 500;       // default threshold if not calibrated

float Kp = 0.03, Ki = 0, Kd = 0.2;
int lfSpeed = 100; // desired speed
int currentSpeed = 30; // starting speed

//---------------- Global variables ----------------
int sensorValue[16], sensorArray[16], minValues[16], maxValues[16], threshold[16], sensorWeight[16];
float error = 0, previousError = 0, integral = 0, PIDvalue = 0;
int activeSensors = 0;
int onLine = 1;

//---------------- Motor helper ----------------
void setMotor(int pin1, int pin2, int pwm, int speed){
    speed = constrain(speed, -255, 255);
    if(speed > 0){
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        analogWrite(pwm, speed);
    } else if(speed < 0){
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        analogWrite(pwm, abs(speed));
    } else {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, HIGH);
        analogWrite(pwm, 0);
    }
}

void motor1run(int speed){ setMotor(AIN1, AIN2, PWMA, speed); }
void motor2run(int speed){ setMotor(BIN1, BIN2, PWMB, speed); }

//---------------- Ramp speed ----------------
void rampSpeed(){
    if(currentSpeed < lfSpeed) currentSpeed++;
    else if(currentSpeed > lfSpeed) currentSpeed--;
}

//---------------- Initialize sensor weights ----------------
void initSensorWeights(){
    int centerTimesTwo = numSensors - 1;
    for(int i=0;i<numSensors;i++){
        sensorWeight[i] = centerTimesTwo - 2*i;
    }
}

//---------------- Read sensors ----------------
void readAndMapSensors(){
    onLine = 0;
    for(int i=0;i<numSensors;i++){
        int raw = analogRead(sensorPins[i]);   // change to digitalRead() if using digital sensors
        // Map readings to 0..1000
        sensorValue[i] = constrain(map(raw, minValues[i], maxValues[i], 0, 1000), 0, 1000);
        // Decide if sensor is active
        sensorArray[i] = (sensorValue[i] > threshold[i]) ? 1 : 0;
        if(sensorArray[i]) onLine = 1;
    }
}

//---------------- Compute line position ----------------
float computePositionFromSensors(){
    long numerator = 0, denominator = 0;
    activeSensors = 0;
    for(int i=0;i<numSensors;i++){
        numerator += (long)sensorValue[i] * (long)sensorWeight[i];
        denominator += sensorValue[i];
        if(sensorValue[i] > 50) activeSensors++;
    }
    if(denominator == 0){ onLine = 0; return previousError; }
    onLine = 1;
    return (float)numerator / (float)denominator;
}

//---------------- Update PID ----------------
void updatePID(){
    error = computePositionFromSensors();
    integral += error;
    float Pterm = Kp * error;
    float Iterm = Ki * integral;
    float Dterm = Kd * (error - previousError);
    PIDvalue = Pterm + Iterm + Dterm;
    previousError = error;
}

//---------------- Apply PID to motors ----------------
void applyPIDtoMotors(){
    int left = currentSpeed + (int)PIDvalue;
    int right = currentSpeed - (int)PIDvalue;
    left = constrain(left, 0, 255);
    right = constrain(right, 0, 255);
    motor1run(left);
    motor2run(right);
}

//---------------- Calibration ----------------
void calibrate(){
    for(int i=0;i<numSensors;i++){
        minValues[i] = analogRead(sensorPins[i]);
        maxValues[i] = minValues[i];
    }
    // rotate robot slightly for calibration
    for(int j=0;j<10000;j++){
        motor1run(70);
        motor2run(-70);
        for(int i=0;i<numSensors;i++){
            int val = analogRead(sensorPins[i]);
            if(val < minValues[i]) minValues[i] = val;
            if(val > maxValues[i]) maxValues[i] = val;
        }
    }
    for(int i=0;i<numSensors;i++){
        threshold[i] = (minValues[i] + maxValues[i]) / 2;
        Serial.print(threshold[i]); Serial.print(" ");
    }
    Serial.println();
    motor1run(0); motor2run(0);
}

//---------------- Setup ----------------
void setup(){
    sbi(ADCSRA, ADPS2); cbi(ADCSRA, ADPS1); cbi(ADCSRA, ADPS0);
    Serial.begin(9600);

    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(11, INPUT_PULLUP); pinMode(12, INPUT_PULLUP); pinMode(13, OUTPUT);
    pinMode(MOTOR_ENABLE, OUTPUT); digitalWrite(MOTOR_ENABLE, HIGH);

    initSensorWeights();
}

//---------------- Main Loop ----------------
void loop(){
    while(digitalRead(11)){} delay(1000); // wait for button1
    calibrate();
    while(digitalRead(12)){} delay(1000); // wait for button2

    while(1){
        readAndMapSensors();
        rampSpeed();
        updatePID();
        if(onLine){
            applyPIDtoMotors();
            digitalWrite(13, HIGH);
        } else {
            digitalWrite(13, LOW);
            if(error < 0) motor1run(0), motor2run(currentSpeed);
            else motor1run(currentSpeed), motor2run(0);
        }
    }
}
--------------------------------------------------------------------------------------------------------------------------TEST CODING FOR 8 SENSORSS:
int sensors[8] = {2, 3, 4, 5, 6, 7, 8, 9};  // Pins for 8 sensors

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 8; i++) {
    pinMode(sensors[i], INPUT);
  }
}

void loop() {
  Serial.print("Sensors: ");
  for (int i = 0; i < 8; i++) {
    int val = digitalRead(sensors[i]);
    Serial.print(val);
    Serial.print(" ");
  }
  Serial.println();
  delay(200);
}

