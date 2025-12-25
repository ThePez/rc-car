/*
 *****************************************************************************
 * File: motors.h
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

#include "driver/mcpwm_prelude.h"

// A = Right, B = Left
#define MOTOR_ENA 16
#define MOTOR_ENB 17
#define MOTOR_INT_1 18
#define MOTOR_INT_2 19
#define MOTOR_INT_3 4
#define MOTOR_INT_4 5

// Throttle speeds / Duty cycle's in us
#define MOTOR_SPEED_MIN 0
#define MOTOR_SPEED_MAX 800
#define NUMBER_OF_MOTORS 2

typedef enum { MOTOR_A, MOTOR_B } MotorIndex_t;
typedef enum { FORWARD, REVERSE, BRAKE, COAST } MotorMode_t;

esp_err_t motor_init(void);

void drive_motor(MotorIndex_t motor, int16_t speed);

esp_err_t pwm_set_duty_cycle(MotorIndex_t motor, uint16_t duty_cycle);

#endif
