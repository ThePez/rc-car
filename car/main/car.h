/*
 *****************************************************************************
 * File: car.h
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#ifndef DRONE_H
#define DRONE_H

// STD C lib headers
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

// KConfig header
#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define SYS_STACK (configMINIMAL_STACK_SIZE * 2)
#define SYS_PRIO (tskIDLE_PRIORITY + 2)

#define ADC_MIN 0
#define ADC_MAX 4095

///////////////////////////// Structures & Enums /////////////////////////////

typedef enum { STANDARD, DUAL } ControlMode_t;

typedef struct {
    double leftJoyX;
    double leftJoyY;
    double rightJoyX;
    double rightJoyY;
} RemoteSetPoints_t;

typedef struct {
    int16_t motorLeft;
    int16_t motorRight;
} MotorPeriods_t;

typedef struct {
    RemoteSetPoints_t* setPoints;
    MotorPeriods_t* motorSignalPeriods;
    ControlMode_t mode;
    uint8_t failsafeActive;
    uint8_t isRemoteLinked;
    uint16_t batteryLevel;
} BlackBox_t;

///////////////////////////////// Prototypes /////////////////////////////////

void drive_controller(void* pvParams);

void failsafe(BlackBox_t* box);

void process_remote_data(BlackBox_t* box, uint16_t* payload);

void update_motors(BlackBox_t* box);

/**
 * @brief Initializes a periodic timer to signal that remote data can be
 * returned.
 *
 * Creates and starts an ESP timer that periodically triggers a notification
 * to the flight controller task, allowing it to send motor state over radio.
 *
 * @param periodUS The timer period in microseconds.
 */
void timer_task_callback_init(int periodUS, BlackBox_t* box, void (*cb)(void*));

void battery_callback(void* args);

void remote_data_callback(void* args);

#endif
