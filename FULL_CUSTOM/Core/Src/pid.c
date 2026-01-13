/*
 * pid.c
 *  Created on: 13 ene 2026
 *      Author: lemaayun
 *      Description: PID, it has layer over layer of iteration
 *      it varely work as intended and since i didnt had branches
 *      the most stable version of the PID Implementation is lost in my mind
 */

#include "pid.h"
#include <math.h>

static inline float clamp(float v, float min, float max)
{
    if (v > max) return max;
    if (v < min) return min;
    return v;
}

static inline float absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

void PID_Init(PID *pid,
              float kp, float ki, float kd,
              float Ts,
              float integralMax,
              float outputMax,
              float derivFilterTau)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->Ts = Ts;

    pid->integralMax = integralMax;
    pid->outputMax = outputMax;

    // Calcular alfa para filtro derivada: alfa = Ts / (Ts + tau)
    // tau típico = 0.01 a 0.1 segundos
    if (derivFilterTau > 0.0f) {
        pid->derivFilterAlpha = Ts / (Ts + derivFilterTau);
    } else {
        pid->derivFilterAlpha = 1.0f; // Sin filtrado
    }

    pid->setpoint = 0.0f;
    pid->enabled = 0;

    PID_Reset(pid);
}

void PID_Reset(PID *pid)
{
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->derivFiltered = 0.0f;
}

void PID_Enable(PID *pid, uint8_t enable)
{
    if (enable && !pid->enabled) {
        // Al habilitar, resetear estados
        PID_Reset(pid);
    }
    pid->enabled = enable;
}

void PID_SetSetpoint(PID *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

float PID_Update(PID *pid, float measurement)
{
    if (!pid->enabled) {
        return 0.0f;
    }

    // Calcular error
    float error = pid->setpoint - measurement;

    // Término proporcional
    float P = pid->kp * error;

    // Término integral con anti-windup
    pid->integral += error * pid->Ts;
    pid->integral = clamp(pid->integral, -pid->integralMax, pid->integralMax);
    float I = pid->ki * pid->integral;

    // Término derivativo (derivada sobre error, filtrada)
    float errorDeriv = (error - pid->prevError) / pid->Ts;

    // Filtro EMA de primer orden para la derivada
    pid->derivFiltered += pid->derivFilterAlpha * (errorDeriv - pid->derivFiltered);
    float D = pid->kd * pid->derivFiltered;

    pid->prevError = error;

    // Salida total
    float output = P + I + D;

    // Anti-windup condicional: si la salida satura, detener integración
    if (absf(output) > pid->outputMax) {
        // Revertir última integración
        pid->integral -= error * pid->Ts;
        output = clamp(output, -pid->outputMax, pid->outputMax);
    }

    return output;
}

void PID_SetGains(PID *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
