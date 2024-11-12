/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"
#include "led_strip.h" 
#include "esp_random.h"
#include "math.h"


#define STEP_MOTOR_GPIO_EN                          1 //Enable pin of the motor driver
#define STEP_MOTOR_GPIO_DIR                         5 //Direction pin of the motor driver
#define STEP_MOTOR_GPIO_STEP                        4 //Step pin of the motor driver
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




void stepperMotorTask(void *pvParameters);
void ledStripTask(void *pvParameters);

void app_main(void)
{
    xTaskCreate(stepperMotorTask, "stepperMotorTask", 8192, NULL, 4, NULL);
    xTaskCreate(ledStripTask, "ledStripTask", 8192, NULL, 5, NULL);    
}

void stepperMotorTask(void *pvParameters){
    static const char *TAG = "example";
    ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    ESP_LOGI(TAG, "Set spin direction");
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

    ESP_LOGI(TAG, "Create motor encoders");
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 500,
        .end_freq_hz = STEP_MOTOR_ACCEL_DECEL_FREQ,
    };
    rmt_encoder_handle_t accel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = STEP_MOTOR_ACCEL_DECEL_FREQ,
        .end_freq_hz = 500,
    };
    rmt_encoder_handle_t decel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));

    ESP_LOGI(TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    const static uint32_t accel_samples = STEP_MOTOR_ACCEL_DECEL_SAMPLES;
    const static uint32_t uniform_speed_hz = STEP_MOTOR_ACCEL_DECEL_FREQ;
    const static uint32_t decel_samples = STEP_MOTOR_ACCEL_DECEL_SAMPLES;

    uint16_t length = 150; // Length of the feeding in mm
    
    ESP_LOGI(TAG, "Steps per millimeter: %d", STEPS_PER_mm); 
    ESP_LOGI(TAG, "Steps per micrometer: %d", STEPS_PER_um); 
    
    while (1) {
        // acceleration phase
        tx_config.loop_count = 0;
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));

        // uniform phase
        //tx_config.loop_count = counter;
        tx_config.loop_count = (length * (STEPS_PER_um/1000)) - STEP_MOTOR_ACCEL_DECEL_SAMPLES;
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));

        // deceleration phase
        tx_config.loop_count = 0;
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
        // wait all transactions finished
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
        
        vTaskDelay(pdMS_TO_TICKS(2000));        
    }
}

void ledStripTask(void *pvParameters){

    led_strip_handle_t led_strip;

    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = 8, // The GPIO that connected to the LED strip's data line
        .max_leds = 4, // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812, // LED strip model
        .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
    };

    led_strip_rmt_config_t rmt_config = {
    #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
    #else
        .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false, // whether to enable the DMA feature
    #endif
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    while (1)
    {
        led_strip_set_pixel(led_strip, 0, (esp_random()%50), (esp_random()%50), (esp_random()%50)); 
        /*led_strip_set_pixel(led_strip, 1, 10, 10, 10); 
        led_strip_set_pixel(led_strip, 2, 10, 10, 10); 
        led_strip_set_pixel(led_strip, 3, 10, 10, 10); */
        led_strip_refresh(led_strip);

        vTaskDelay(pdMS_TO_TICKS(250));
    } 
}