//pid header file
#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prevError;
    float integral;
} PIDController;

void PID_init(PIDController *pid, float Kp, float Ki, float Kd);
float PID_compute(PIDController *pid, float error);

#endif
