/*
 *****************************************************************************
 * File: motor.c
 * Author: Jack Cairns
 * Date: 13-07-2025
 * Brief:
 * REFERENCE: None
 *****************************************************************************
 */

#include "motors.h"
#include "common_functions.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "PWM";
static mcpwm_cmpr_handle_t esc_pwm_comparators[2] = {NULL};

#define CHECK_ERR(code, msg)                                                                                           \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            ESP_LOGE(TAG, msg);                                                                                        \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

#define CHECK_ERR_NO_LOG(code)                                                                                         \
    do {                                                                                                               \
        esp_err_t err = (code);                                                                                        \
        if (err != ESP_OK) {                                                                                           \
            return err;                                                                                                \
        }                                                                                                              \
    } while (0)

// Prototypes//
static esp_err_t set_motor_mode(gpio_num_t int1, gpio_num_t int2, MotorMode_t mode);
static esp_err_t pwm_init(void);
static esp_err_t motor_int_pin_init(void);

// Functions //

esp_err_t motor_init(void) {
    CHECK_ERR_NO_LOG(motor_int_pin_init());

    CHECK_ERR_NO_LOG(pwm_init());
    return ESP_OK;
}

esp_err_t pwm_set_duty_cycle(MotorIndex_t motor, uint16_t duty_cycle) {

    // Ensure that the motor index given is valid
    if (!(motor < NUMBER_OF_MOTORS)) {
        ESP_LOGE(TAG, "Invalid motor index: %d (max: %d)", motor, NUMBER_OF_MOTORS - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // Limit the requested motor speed between the min/max
    duty_cycle = (uint16_t) constrainf(duty_cycle, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

    // Update the comparator used by the specified motor
    CHECK_ERR(mcpwm_comparator_set_compare_value(esc_pwm_comparators[motor], duty_cycle),
              "Failed to set PWM duty cycle for motor");

    return ESP_OK;
}

void drive_motor(MotorIndex_t motor, int16_t speed) {
    uint8_t int1, int2;
    if (motor == MOTOR_A) {
        int1 = MOTOR_INT_1;
        int2 = MOTOR_INT_2;
    } else {
        int1 = MOTOR_INT_3;
        int2 = MOTOR_INT_4;
    }

    if (speed > 0) {
        set_motor_mode(int1, int2, FORWARD);
        pwm_set_duty_cycle(motor, speed);
    } else if (speed < 0) {
        set_motor_mode(int1, int2, REVERSE);
        pwm_set_duty_cycle(motor, -speed);
    } else {
        pwm_set_duty_cycle(motor, 0);
        set_motor_mode(int1, int2, COAST);
    }
}

static esp_err_t pwm_init(void) {
    // Create and configure the timers
    mcpwm_timer_handle_t pwm_timer = NULL;
    mcpwm_timer_config_t pwm_timer_config = {
        .resolution_hz = 1000000,                // 1MHz
        .period_ticks = 1000,                    // 1ms -> 1kHz
        .group_id = 0,                           // Group 0
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP, // Count up from 0 then reset to 0
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT   // Default clock source
    };

    CHECK_ERR(mcpwm_new_timer(&pwm_timer_config, &pwm_timer), "Failed to create PWM timer");

    // Connect the operators to the timers
    mcpwm_oper_handle_t operators[2] = {NULL};
    // Operator must be in the same group as timer
    mcpwm_operator_config_t operator_configs = {.group_id = 0};
    for (uint8_t i = 0; i < 2; i++) {
        CHECK_ERR(mcpwm_new_operator(&operator_configs, &operators[i]), "Failed to create PWM operator");
        CHECK_ERR(mcpwm_operator_connect_timer(operators[i], pwm_timer), "Failed to connect operator to timer");
    }

    // Create and configure the Comparators and Generators
    mcpwm_comparator_config_t cmpr_config = {0};
    mcpwm_gen_handle_t generators[2] = {NULL};
    mcpwm_generator_config_t generator_configs[2] = {};
    generator_configs[0].gen_gpio_num = MOTOR_ENA;
    generator_configs[1].gen_gpio_num = MOTOR_ENB;

    for (uint8_t i = 0; i < 2; i++) {
        CHECK_ERR(mcpwm_new_comparator(operators[i], &cmpr_config, &esc_pwm_comparators[i]),
                  "Failed to create PWM comparator");
        CHECK_ERR(mcpwm_new_generator(operators[i], &generator_configs[i], &generators[i]),
                  "Failed to create PWM generator");
    }

    // Configure generator actions: HIGH on timer period, LOW on compare match
    for (uint8_t i = 0; i < 2; i++) {
        CHECK_ERR(mcpwm_generator_set_action_on_timer_event(
                      generators[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                                                  MCPWM_GEN_ACTION_HIGH)),
                  "Failed to set timer action for generator");

        CHECK_ERR(mcpwm_generator_set_action_on_compare_event(
                      generators[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, esc_pwm_comparators[i],
                                                                    MCPWM_GEN_ACTION_LOW)),
                  "Failed to set compare action for generator");

        // Set the initial compare values to MOTOR_SPEED_MIN
        pwm_set_duty_cycle(i, MOTOR_SPEED_MIN);
    }

    // Enable and start the timer
    CHECK_ERR(mcpwm_timer_enable(pwm_timer), "Failed to enable PWM timer");
    CHECK_ERR(mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP), "Failed to start PWM timer");

    ESP_LOGI(TAG, "ESC PWM initialization completed successfully");
    return ESP_OK;
}

static esp_err_t motor_int_pin_init(void) {
    CHECK_ERR(gpio_set_direction(MOTOR_INT_1, GPIO_MODE_OUTPUT), "Motor Int1 setup failed");
    CHECK_ERR_NO_LOG(gpio_set_level(MOTOR_INT_1, 0));
    CHECK_ERR(gpio_set_direction(MOTOR_INT_2, GPIO_MODE_OUTPUT), "Motor Int2 setup failed");
    CHECK_ERR_NO_LOG(gpio_set_level(MOTOR_INT_2, 0));
    CHECK_ERR(gpio_set_direction(MOTOR_INT_3, GPIO_MODE_OUTPUT), "Motor Int3 setup failed");
    CHECK_ERR_NO_LOG(gpio_set_level(MOTOR_INT_3, 0));
    CHECK_ERR(gpio_set_direction(MOTOR_INT_4, GPIO_MODE_OUTPUT), "Motor Int4 setup failed");
    CHECK_ERR_NO_LOG(gpio_set_level(MOTOR_INT_4, 0));
    return ESP_OK;
}

static esp_err_t set_motor_mode(gpio_num_t int1, gpio_num_t int2, MotorMode_t mode) {
    switch (mode) {
    case FORWARD:
        CHECK_ERR_NO_LOG(gpio_set_level(int1, 1));
        CHECK_ERR_NO_LOG(gpio_set_level(int2, 0));
        break;

    case REVERSE:
        CHECK_ERR_NO_LOG(gpio_set_level(int1, 0));
        CHECK_ERR_NO_LOG(gpio_set_level(int2, 1));
        break;

    case COAST: // Same levels for int's but ENA/B is now low
    case BRAKE:
        CHECK_ERR_NO_LOG(gpio_set_level(int1, 0));
        CHECK_ERR_NO_LOG(gpio_set_level(int2, 0));
        break;
    }

    return ESP_OK;
}
