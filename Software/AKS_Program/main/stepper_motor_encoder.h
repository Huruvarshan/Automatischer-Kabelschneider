/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

// Stepper motor configuration
#define STEP_MOTOR_GPIO_EN                          4 //Enable pin of the motor driver
#define STEP_MOTOR_GPIO_DIR                         6 //Direction pin of the motor driver
#define STEP_MOTOR_GPIO_STEP                        5 //Step pin of the motor driver
#define STEP_MOTOR_GPIO_MS1                         7 //Microstep pin 1 of the motor driver
#define STEP_MOTOR_GPIO_MS2                         8 //Microstep pin 2 of the motor driver
#define STEP_MOTOR_ENABLE_LEVEL                     0 //Enable level of the motor driver; 0 for !ENABLE, 1 for ENABLE
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE               1 //Spin direction of the motor driver; 1 for clockwise, 0 for counterclockwise
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE        STEP_MOTOR_SPIN_DIR_CLOCKWISE //Spin direction of the motor driver

#define STEP_MOTOR_RESOLUTION_HZ                    1000000 // Resolution of the motor driver in Hz
#define STEP_MOTOR_ACCEL_DECEL_FREQ                 3000 // Frequency of the motor driver in Hz
#define STEP_MOTOR_ACCEL_DECEL_SAMPLES              50 // Number of samples for acceleration and deceleration
#define STEP_MOTOR_FULL_ROTATION_STEPS              3200 // Number of steps for a full rotation, excluding acceleration and deceleration
#define STEP_MOTOR_FULL_ROTATION_AFTER_ACCEL_DECEL  3100 // STEP_MOTOR_FULL_ROTATION_STEPS - 2 * STEP_MOTOR_ACCEL_DECEL_SAMPLES = 3200 - 100

#define FEEDER_DIAMETER_mm                          50 // Diameter of the feeder in mm
#define FEEDER_CIRCUMFERENCE_mm                     157 // FEEDER_DIAMETER_mm * π ≈ 50 * 3.14159 = 157.08, rounded to integer
#define STEPS_PER_mm                                20 // STEP_MOTOR_FULL_ROTATION_STEPS / FEEDER_CIRCUMFERENCE_mm ≈ 3200 / 157.08 = 20.37, rounded to integer
#define STEPS_PER_um                                20372 //Steps per micrometer
#define MIN_LENGTH_mm                               5 // (STEP_MOTOR_ACCEL_DECEL_SAMPLES * 2) / STEPS_PER_mm ≈ (50 * 2) / 20 = 5

/**
 * @brief Stepper motor curve encoder configuration
 */
typedef struct {
    uint32_t resolution;    // Encoder resolution, in Hz
    uint32_t sample_points; // Sample points used for deceleration phase. Note: |end_freq_hz - start_freq_hz| >= sample_points
    uint32_t start_freq_hz; // Start frequency on the curve, in Hz
    uint32_t end_freq_hz;   // End frequency on the curve, in Hz
} stepper_motor_curve_encoder_config_t;

/**
 * @brief Stepper motor uniform encoder configuration
 */
typedef struct {
    uint32_t resolution; // Encoder resolution, in Hz
} stepper_motor_uniform_encoder_config_t;

/**
 * @brief Create stepper motor curve encoder
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating step motor encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_stepper_motor_curve_encoder(const stepper_motor_curve_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

/**
 * @brief Create RMT encoder for encoding step motor uniform phase into RMT symbols
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating step motor encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_stepper_motor_uniform_encoder(const stepper_motor_uniform_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif
