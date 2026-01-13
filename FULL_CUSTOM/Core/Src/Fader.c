/*
 * Fader.c
 *  Created on: 13 ene 2026
 *      Author: lemaayun
 *      Description: Motorized Fader implementation
 *      been working on it way before i started to use github to create experimentation branches.
 */

#include "Fader.h"
#include <math.h>

static inline float absf(float x)
{
    return (x < 0.0f) ? -x : x;
}


static void Fader_StopMotor(Fader *f)
{
    f->mcp->gpio[MCP23017_PORTA] &= ~(1 << f->hBridgePins[0]);
    f->mcp->gpio[MCP23017_PORTB] &= ~(1 << f->hBridgePins[1]);

    mcp23017_write_gpio(f->mcp, MCP23017_PORTA);
    mcp23017_write_gpio(f->mcp, MCP23017_PORTB);
}

static void Fader_RunMotorForward(Fader *f)
{
    // Dirección A: PORTA=1, PORTB=0
    f->mcp->gpio[MCP23017_PORTB] &= ~(1 << f->hBridgePins[1]);
    mcp23017_write_gpio(f->mcp, MCP23017_PORTB);

    f->mcp->gpio[MCP23017_PORTA] |= (1 << f->hBridgePins[0]);
    mcp23017_write_gpio(f->mcp, MCP23017_PORTA);
}

static void Fader_RunMotorReverse(Fader *f)
{
    // Dirección B: PORTA=0, PORTB=1
    f->mcp->gpio[MCP23017_PORTA] &= ~(1 << f->hBridgePins[0]);
    mcp23017_write_gpio(f->mcp, MCP23017_PORTA);

    f->mcp->gpio[MCP23017_PORTB] |= (1 << f->hBridgePins[1]);
    mcp23017_write_gpio(f->mcp, MCP23017_PORTB);
}

static void Fader_ApplyPWM(Fader *f, float pwmValue)
{
    // Clamp PWM entre 0 y ARR del timer
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(f->pwmTimer);
    if (pwmValue < 0.0f) pwmValue = 0.0f;
    if (pwmValue > (float)arr) pwmValue = (float)arr;

    __HAL_TIM_SET_COMPARE(f->pwmTimer, TIM_CHANNEL_1, (uint32_t)pwmValue);
}

static void Fader_ApplyOutput(Fader *f, float controlOutput)
{
    float magnitude = absf(controlOutput);

    // Dead-band: si la salida es muy pequeña, detener motor
    if (magnitude < f->minPWM) {
        Fader_StopMotor(f);
        Fader_ApplyPWM(f, 0.0f);
        return;
    }

    // Aplicar PWM
    Fader_ApplyPWM(f, magnitude);

    // Seleccionar dirección
    if (controlOutput > 0.0f) {
        Fader_RunMotorForward(f);
    } else {
        Fader_RunMotorReverse(f);
    }
}

// Inicialización
void Fader_Init(Fader *fader,
                uint8_t pinIndex,
                I2C_HandleTypeDef *i2c,
                TIM_HandleTypeDef *pwmTimer,
                MCP23017_HandleTypeDef *mcp,
                PID *pid)
{
    fader->i2c = i2c;
    fader->pwmTimer = pwmTimer;
    fader->mcp = mcp;
    fader->pid = pid;

    fader->hBridgePins[0] = pinIndex;  // PORTA
    fader->hBridgePins[1] = pinIndex;  // PORTB

    fader->state = FADER_IDLE;
    fader->position = 0.0f;
    fader->lastPosition = 0.0f;
    fader->setpoint = 0.0f;

    // Valores por defecto (ajustables después)
    fader->activationThreshold = 50.0f;  // ~1% del rango ADC 12-bit
    fader->stopThreshold = 10.0f;        // ~0.2% del rango
    fader->userMoveThreshold = 20.0f;    // Delta para detectar usuario
    fader->minPWM = 15.0f;               // PWM mínimo (ajustar según motor)

    // Asegurar motor apagado
    Fader_StopMotor(fader);
}

void Fader_SetThresholds(Fader *fader,
                         float activation,
                         float stop,
                         float userMove)
{
    fader->activationThreshold = activation;
    fader->stopThreshold = stop;
    fader->userMoveThreshold = userMove;
}

void Fader_SetMinPWM(Fader *fader, float minPWM)
{
    fader->minPWM = minPWM;
}

void Fader_SetTarget(Fader *fader, float target)
{
    fader->setpoint = target;
}

void Fader_SetPosition(Fader *fader, float position)
{
    fader->position = position;
}

void Fader_Task(Fader *fader)
{
    float error = fader->setpoint - fader->position;
    float delta = fader->position - fader->lastPosition;
    fader->lastPosition = fader->position;

    switch (fader->state)
    {
        case FADER_IDLE:
            // Motor apagado, esperando comando
            Fader_StopMotor(fader);
            PID_Enable(fader->pid, 0);

            // Si hay error significativo, activar movimiento
            if (absf(error) > fader->activationThreshold) {
                PID_SetSetpoint(fader->pid, fader->setpoint);
                PID_Enable(fader->pid, 1);
                fader->state = FADER_MOVING;
            }
            break;

        case FADER_MOVING:
            // Usuario interrumpe movimiento
            if (absf(delta) > fader->userMoveThreshold) {
                Fader_StopMotor(fader);
                PID_Enable(fader->pid, 0);
                fader->setpoint = fader->position;  // Actualizar setpoint
                fader->state = FADER_USER_CONTROL;
                break;
            }

            // Llegó al destino
            if (absf(error) < fader->stopThreshold) {
                Fader_StopMotor(fader);
                PID_Enable(fader->pid, 0);
                fader->state = FADER_IDLE;
                break;
            }

            // Actualizar PID y aplicar salida
            float controlOutput = PID_Update(fader->pid, fader->position);
            Fader_ApplyOutput(fader, controlOutput);
            break;

        case FADER_USER_CONTROL:
            // Usuario está moviendo el fader
            Fader_StopMotor(fader);
            PID_Enable(fader->pid, 0);

            // Si el usuario deja de mover, volver a idle
            if (absf(delta) < fader->userMoveThreshold / 2.0f) {
                fader->state = FADER_IDLE;
            }
            break;
    }
}

FaderState Fader_GetState(const Fader *fader)
{
    return fader->state;
}

float Fader_GetPosition(const Fader *fader)
{
    return fader->position;
}

float Fader_GetError(const Fader *fader)
{
    return fader->setpoint - fader->position;
}
