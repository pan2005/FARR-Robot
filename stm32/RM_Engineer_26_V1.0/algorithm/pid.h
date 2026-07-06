#ifndef PID_H
#define PID_H

typedef struct {
    float Kp, Ki, Kd;
    float integral_limit;
    float output_limit;

    float target;
    float current_measure;
    float error;
    float last_error;
    float integral;
    float output;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float i_limit, float out_limit);
float PID_Calculate(PID_t *pid, float target, float measure);

#endif