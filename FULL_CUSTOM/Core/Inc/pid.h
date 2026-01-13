/*
 * pid.h
 *  Created on: 13 ene 2026
 *      Author: lemaayun
 *      Description: Motorized Fader implementation
 *      been working on it way before i started to use github to create experimentation branches.
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

typedef struct {
    // Ganancias
    float kp;
    float ki;
    float kd;

    // Tiempo de muestreo
    float Ts;

    // Estados internos
    float integral;
    float prevError;      // Para derivada sobre error
    float derivFiltered;  // Derivada filtrada

    // Límites
    float integralMax;
    float outputMax;

    // Setpoint
    float setpoint;

    // Filtro derivada (alfa = Ts/(Ts + tau))
    float derivFilterAlpha;

    // Estado
    uint8_t enabled;

} PID;

// Inicialización
void PID_Init(PID *pid,
              float kp, float ki, float kd,
              float Ts,
              float integralMax,
              float outputMax,
              float derivFilterTau);

// Control
void PID_Enable(PID *pid, uint8_t enable);
void PID_Reset(PID *pid);
void PID_SetSetpoint(PID *pid, float setpoint);
float PID_Update(PID *pid, float measurement);

// Ajustes en tiempo de ejecución
void PID_SetGains(PID *pid, float kp, float ki, float kd);

#endif /* INC_PID_H_ */
