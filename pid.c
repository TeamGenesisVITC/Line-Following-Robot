#include "pid.h"

void PID_init(PIDController *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prevError = 0;
    pid->integral = 0;
}

float PID_compute(PIDController *pid, float error) {
    float P = error;
    pid->integral += error;
    float D = error - pid->prevError;

    float output = (pid->Kp * P) + (pid->Ki * pid->integral) + (pid->Kd * D);

    pid->prevError = error;
    return output;
}