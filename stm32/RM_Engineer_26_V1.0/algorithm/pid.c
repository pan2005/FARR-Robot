#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float i_limit, float out_limit) {
    pid->Kp = Kp; pid->Ki = Ki; pid->Kd = Kd;
    pid->integral_limit = i_limit;
    pid->output_limit = out_limit;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float PID_Calculate(PID_t *pid, float target, float measure) {
    pid->target = target;
    pid->current_measure = measure;
    pid->error = target - measure;

    // 积分并限幅
    pid->integral += pid->error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float p_out = pid->Kp * pid->error;
    float i_out = pid->Ki * pid->integral;
    float d_out = pid->Kd * (pid->error - pid->last_error);

    pid->output = p_out + i_out + d_out;

    // 输出限幅
    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;

    pid->last_error = pid->error;
    return pid->output;
}