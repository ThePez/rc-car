/*
 *****************************************************************************
 * File: car.c
 * Author: Jack Cairns
 * Date: 27-06-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "car.h"
#include "common_functions.h"
#include "mcp3208.h"
#include "motors.h"
#include "nrf24l01plus.h"

#define TAG "CAR"
#define FAILSAFE_TIMEOUT_US 1000000 // 1 second
#define ADC_CS_PIN 33

////////////////////////////// Global Variables //////////////////////////////

// Task Handles
TaskHandle_t driveController = NULL;

//////////////////////////////////////////////////////////////////////////////

void app_main(void) {
    /* Physical Hardware setup */

    // Setup the PWM and motor pins
    motor_init();

    // SPI Bus Setup for radio and mcp3208 (ADC chip)
    spi_bus_setup(HSPI_HOST);

    // FreeRTOS Task setup for components
    radio_module_init(&spiHMutex, HSPI_HOST);
    mcpx_task_init(&spiHMutex, 0x02, HSPI_HOST, ADC_CS_PIN);

    // System controller
    xTaskCreate(&drive_controller, "Car_Task", SYS_STACK, NULL, SYS_PRIO, &driveController);
}

void timer_task_callback_init(int periodUS, BlackBox_t* box, void (*cb)(void*)) {

    esp_timer_create_args_t config = {
        .callback = cb,                    // Function to execute
        .dispatch_method = ESP_TIMER_TASK, // Where the function is called from
        .arg = box                         // Input argument
    };

    esp_timer_handle_t timerHandle = NULL;
    esp_timer_create(&config, &timerHandle);
    esp_timer_start_periodic(timerHandle, periodUS);
}

void battery_callback(void* args) {

    BlackBox_t* box = (BlackBox_t*) args;
    if (!box) {
        return;
    }

    if (!mcpxQueue) {
        return;
    }

    uint16_t batteryLevel;
    if (xQueueReceive(mcpxQueue, &batteryLevel, pdMS_TO_TICKS(MCPx_DELAY)) == pdTRUE) {
        box->batteryLevel = batteryLevel;
        // ESP_LOGI(TAG, "Battery: %d", box->batteryLevel);
    }
}

void remote_data_callback(void* args) {
    BlackBox_t* box = (BlackBox_t*) args;

    if (!box) {
        return;
    }

    // Only send telemetry when we have a valid, active link
    if (!box->isRemoteLinked || box->failsafeActive) {
        return;
    }

    int16_t package[RADIO_PAYLOAD_WIDTH / 2]; // 16 words

    // Angles: pitch, roll, yaw     = 3
    // Rates: pitch, roll, yaw      = 3
    // Flight mode                  = 1
    // PID outputs pitch, roll, yaw = 3
    // Motor periods A, B, C, D     = 4
    // Battery percentage           = 1
    // Total                       = 15

    // Angles
    package[0] = 0;
    package[1] = 0;
    package[2] = 0;
    // Rate
    package[3] = 0;
    package[4] = 0;
    package[5] = 0;
    // Flight mode
    package[6] = (int16_t) box->mode;
    // PID outputs
    package[7] = 0;
    package[8] = 0;
    package[9] = 0;
    // Motor periods
    package[10] = box->motorSignalPeriods->motorLeft;
    package[11] = box->motorSignalPeriods->motorRight;
    package[12] = 0;
    package[13] = 0;
    // Battery level
    package[14] = box->batteryLevel;

    xQueueSendToBack(radioTransmitterQueue, package, 0);
}

BlackBox_t* black_box_init(void) {

    BlackBox_t* box = pvPortMalloc(sizeof(BlackBox_t));
    if (!box) {
        ESP_LOGE(TAG, "Black Box init failed");
        return NULL;
    }

    box->setPoints = pvPortMalloc(sizeof(RemoteSetPoints_t));
    if (box->setPoints) {
        memset(box->setPoints, 0, sizeof(RemoteSetPoints_t));
    } else {
        ESP_LOGE(TAG, "Remote Setpoint data failed");
        goto free_box;
    }

    box->motorSignalPeriods = pvPortMalloc(sizeof(MotorPeriods_t));
    if (box->motorSignalPeriods) {
        memset(box->motorSignalPeriods, 0, sizeof(MotorPeriods_t));
    } else {
        ESP_LOGE(TAG, "Motor Period data failed");
        goto free_setpoint;
    }

    box->mode = STANDARD;
    box->isRemoteLinked = 0;
    box->failsafeActive = 1;
    return box;

    /* Clean up paths */
free_setpoint:
    vPortFree(box->setPoints);
free_box:
    vPortFree(box);
    return NULL;
}

void black_box_destroy(BlackBox_t* box) {
    // Free the motor speed container
    vPortFree(box->motorSignalPeriods);
    // Free the remote setpoints
    vPortFree(box->setPoints);
    // Free the container
    vPortFree(box);
}

void failsafe(BlackBox_t* box) {
    if (box->failsafeActive) {
        return;
    }

    ESP_LOGE(TAG, "FAILSAFE TRIGGERED: No radio link for at least 1s");
    box->failsafeActive = 1;
    box->isRemoteLinked = 0;

    // Force motors to MIN_THROTTLE and skip PID update
    box->motorSignalPeriods->motorLeft = 0;
    box->motorSignalPeriods->motorRight = 0;
    
    // Update PWM and direction
    drive_motor(MOTOR_B, box->motorSignalPeriods->motorLeft);
    drive_motor(MOTOR_A, box->motorSignalPeriods->motorRight);
}

void set_control_mode(ControlMode_t mode, BlackBox_t* box) {
    if ((box != NULL) && (mode != box->mode)) {
        box->mode = mode;
        ESP_LOGI(TAG, "Switched to %s mode", mode == STANDARD ? "Stardard" : "Dual");
    }
}

void process_remote_data(BlackBox_t* box, uint16_t* payload) {

    // Control mode
    set_control_mode((payload[5] == STANDARD) ? STANDARD : DUAL, box);
    // Left Joystick Y
    box->setPoints->leftJoyY = mapf(payload[2], 0, ADC_MAX, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
    // Left Joystick X
    box->setPoints->leftJoyX = mapf(payload[3], 0, ADC_MAX, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
    // Right Joystick Y
    box->setPoints->rightJoyY = mapf(payload[6], 0, ADC_MAX, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
    // Right Joystick X
    box->setPoints->rightJoyX = mapf(payload[4], 0, ADC_MAX, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
}

double apply_deadzone(double v) {
    return fabs(v) < 50 ? 0 : v;
}

void update_motors(BlackBox_t* box) {

    int16_t left, right;
    ESP_LOGI(TAG, "Left X: %f Y: %f, Right X: %f Y: %f", box->setPoints->leftJoyX, box->setPoints->leftJoyY,
             box->setPoints->rightJoyX, box->setPoints->rightJoyY);

    if (box->mode == STANDARD) {

        double throttle = apply_deadzone(box->setPoints->leftJoyY);
        double yaw = apply_deadzone(box->setPoints->leftJoyX);

        // Scalled turning at speed (prevents flips)
        yaw = yaw * (MOTOR_SPEED_MAX - fabs(throttle)) / MOTOR_SPEED_MAX;

        // Motor mixing
        double rawLeft = throttle + yaw;
        double rawRight = throttle - yaw;

        // Find max magnitude
        double maxMag = MAX(fabs(rawLeft), fabs(rawRight));

        // Scale down to range [0, MAX] while keeping turn ratio
        if (maxMag > MOTOR_SPEED_MAX) {
            left = rawLeft * MOTOR_SPEED_MAX / maxMag;
            right = rawRight * MOTOR_SPEED_MAX / maxMag;
        } else {
            left = rawLeft;
            right = rawRight;
        }
    } else {
        left = apply_deadzone(box->setPoints->leftJoyY);
        right = apply_deadzone(box->setPoints->rightJoyY);
    }

    // Store signals
    box->motorSignalPeriods->motorLeft = left;
    box->motorSignalPeriods->motorRight = right;

    // Update PWM and direction
    drive_motor(MOTOR_B, left);
    drive_motor(MOTOR_A, right);
}

void drive_controller(void* pvParams) {

    // Setup the container
    BlackBox_t* box = black_box_init();
    if (box == NULL) {
        vTaskDelete(NULL);
    }

    // Radio payload container
    uint16_t* payload = pvPortMalloc(sizeof(uint16_t) * (RADIO_PAYLOAD_WIDTH / 2));
    if (payload == NULL) {
        black_box_destroy(box);
        vTaskDelete(NULL);
    }

    // Wait until both input queues are created
    while (!radioReceiverQueue || !radioTransmitterQueue) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Failsafe tracking - last time we got a SETPOINT_UPDATE
    uint64_t lastRemoteUpdateTime = esp_timer_get_time();

    // Initialise a callback task for returning info back to the remote
    timer_task_callback_init(100000, box, remote_data_callback); // Interval of 100ms
    timer_task_callback_init(50000, box, battery_callback);      // Interval of 50ms

    while (1) {

        uint64_t now = esp_timer_get_time();
        if (xQueueReceive(radioReceiverQueue, payload, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Process it
            process_remote_data(box, payload);
            // Update flags
            lastRemoteUpdateTime = now;
            box->isRemoteLinked = 1;
            box->failsafeActive = 0;
            // Update Motors
            update_motors(box);
        }

        if (now - lastRemoteUpdateTime > FAILSAFE_TIMEOUT_US && !box->failsafeActive) {
            failsafe(box);
        }
    }
}
