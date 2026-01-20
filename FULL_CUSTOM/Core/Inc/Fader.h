/*
 * Fader.h
 *  Created on: 13 ene 2026
 *      Author: lemaayun
 *      Description: Motorized Fader implementation
 *      been working on it way before i started to use github to create experimentation branches.
 */

#ifndef INC_FADER_H_
#define INC_FADER_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "MCP23017.h"
#include "pid.h"

typedef enum {
    FADER_IDLE = 0,      // Motor apagado, esperando comando
    FADER_MOVING,        // Control PID activo
    FADER_USER_CONTROL   // Usuario está moviendo el fader
} FaderState;

typedef struct {
    // Hardware
    I2C_HandleTypeDef *i2c;
    TIM_HandleTypeDef *pwmTimer;
    MCP23017_HandleTypeDef *mcp;
    uint8_t hBridgePins[2];  // [0]=PORTA pin, [1]=PORTB pin

    // Control
    PID *pid;
    FaderState state;

    float position;
    float lastPosition;
    float setpoint;

    // Umbrales de configuración
    float activationThreshold;  // Error mínimo para activar movimiento
    float stopThreshold;        // Error para considerar llegada
    float userMoveThreshold;    // Delta mínimo para detectar movimiento manual

    // PWM
    float minPWM;  // PWM mínimo para vencer fricción estática

} Fader;

// Inicialización
void Fader_Init(Fader *fader,
                uint8_t pinIndex,
                I2C_HandleTypeDef *i2c,
                TIM_HandleTypeDef *pwmTimer,
                MCP23017_HandleTypeDef *mcp,
                PID *pid);

// Configuración
void Fader_SetThresholds(Fader *fader,
                         float activation,
                         float stop,
                         float userMove);

void Fader_SetMinPWM(Fader *fader, float minPWM);

// Control
void Fader_SetTarget(Fader *fader, float target);
void Fader_SetPosition(Fader *fader, float position);
void Fader_Task(Fader *fader);  // Llamar periódicamente (ej. cada 1ms)

// Estado
FaderState Fader_GetState(const Fader *fader);
float Fader_GetPosition(const Fader *fader);
float Fader_GetError(const Fader *fader);

#endif /* INC_FADER_H_ */
