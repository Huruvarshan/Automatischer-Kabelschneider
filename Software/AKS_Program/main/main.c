#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "math.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "string.h"
#include "stepper_motor_encoder.h"
#include "led_strip.h" 
#include "driver/touch_pad.h"

#define TOUCH_GPIO 1 // GPIO for touch sensor
#define THRESHOLD 80000 // Threshold for touch sensor
#define LED_STRIP_GPIO 14 // GPIO for LED strip
#define LED_STRIP_NUM_LEDS 12 // Number of LEDs in the strip

void EspNowTask(void *pvParameters); 
void StepperTask(void *pvParameters); 
void SafetyTask(void *pvParameters);
void LedTask(void *pvParameters); 

char *mac_to_str(char *buffer, uint8_t *mac);
void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status); 
void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len); 



SemaphoreHandle_t feederStructMutex; 
SemaphoreHandle_t triggerSemaphore;

// Structure for incoming data from the display unit
struct feeder_in_t {
    uint16_t id; // ID from the display unit. Gets from display unit (R)
    uint16_t setAmount; // The set amount configured by the user. Gets from display unit (R) 
    uint16_t setLength; // The set length configured by the user. Gets from display unit (R)
    bool flagStartStop; // Start/Stop flag. Defines if it should stop or continue working. Gets from display unit (R/W) (1 = start, 0 = stop)
    bool flagAbort; // Abort flag. Defines if the job should get aborted. Gets from display unit (R) (1 = abort, 0 = no abort)
} feeder_incoming;

// Structure for outgoing data to the display unit 
struct feeder_out_t {
    uint16_t id; // Sends to display unit (W)
    uint16_t processedAmount; // Sends to display unit (W) 
    bool flagStartStop; // Sends to display unit (W) (1 = start, 0 = stop)
    bool flagAbort; // Sends to display unit (W) (1 = abort, 0 = no abort)
    bool runOut; // Sends to display unit (W) (1 = run out, 0 = no run out)
} feeder_outgoing; 


void app_main(void)
{
    uint8_t my_mac[6];
    char my_mac_str[13];

    feederStructMutex = xSemaphoreCreateMutex(); 
    triggerSemaphore = xSemaphoreCreateBinary(); 

    //feeder.incoming.id = //set to gpio mäuseklavier
    feeder_incoming.setAmount = 0; // Edit to 0 later
    feeder_incoming.setLength = 0; // Edit to 0 later
    feeder_incoming.flagStartStop = 1; 
    feeder_incoming.flagAbort = 0; 

    //feeder_outgoing.id //set to gpio mäuseklavier
    feeder_outgoing.processedAmount = 0; 
    feeder_outgoing.flagStartStop = 1; 
    feeder_outgoing.flagAbort = 0;
    feeder_outgoing.runOut = 0;



    xTaskCreate(StepperTask, "StepperTask", 8192, NULL, 2, NULL);
    xTaskCreate(EspNowTask, "EspNowTask", 8192, NULL, 2, NULL);
    xTaskCreate(SafetyTask, "SafetyTask", 8192, NULL, 3, NULL);
    xTaskCreate(LedTask, "LedTask", 8192, NULL, 1, NULL);
}



void EspNowTask(void *pvParameters){
    uint8_t macDisplay[6] = {0xe4, 0x65, 0xb8, 0x7e, 0x27, 0x98}; // MAC address of the display unit 
    uint8_t my_mac[6];
    char my_mac_str[13];
    // Define a local instance of the feeder_out_t struct for transmission
    struct feeder_out_t feeder_outgoing_data;
    
    esp_efuse_mac_get_default(my_mac);
    ESP_LOGI("MAC_ADDRESS", "My mac %s", mac_to_str(my_mac_str, my_mac));
    
    // Initialize NVS and networking components
    nvs_flash_init();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_receive));
    
    // Configure peer (display unit)
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    memcpy(peer.peer_addr, macDisplay, 6);
    peer.channel = 0;  
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    xSemaphoreTake(feederStructMutex, portMAX_DELAY);

    // Copy the global feeder_outgoing struct data to local struct
    // (assuming feeder_outgoing is a global variable)
    memcpy(&feeder_outgoing_data, &feeder_outgoing, sizeof(struct feeder_out_t));

    // Log the data being sent (optional)
    ESP_LOGI("ESP_NOW", "Sending data: id=%d, processed=%d, start/stop=%d, abort=%d, runOut=%d", 
    feeder_outgoing_data.id, 
    feeder_outgoing_data.processedAmount,
    feeder_outgoing_data.flagStartStop,
    feeder_outgoing_data.flagAbort,
    feeder_outgoing_data.runOut);

    // Send the struct directly through ESP-NOW
    ESP_ERROR_CHECK(esp_now_send(NULL, (uint8_t *)&feeder_outgoing_data, sizeof(struct feeder_out_t)));

    // Release the mutex
    xSemaphoreGive(feederStructMutex);
    
    while (1)
    {
        // Wait for trigger semaphore from another task
        if(xSemaphoreTake(triggerSemaphore, portMAX_DELAY) == pdTRUE){
            // Access shared data with mutex protection
            xSemaphoreTake(feederStructMutex, portMAX_DELAY);
            
            // Copy the global feeder_outgoing struct data to local struct
            // (assuming feeder_outgoing is a global variable)
            memcpy(&feeder_outgoing_data, &feeder_outgoing, sizeof(struct feeder_out_t));
            
            // Log the data being sent (optional)
            ESP_LOGI("ESP_NOW", "Sending data: id=%d, processed=%d, start/stop=%d, abort=%d, runOut=%d", 
                   feeder_outgoing_data.id, 
                   feeder_outgoing_data.processedAmount,
                   feeder_outgoing_data.flagStartStop,
                   feeder_outgoing_data.flagAbort,
                   feeder_outgoing_data.runOut);
            
            // Send the struct directly through ESP-NOW
            ESP_ERROR_CHECK(esp_now_send(NULL, (uint8_t *)&feeder_outgoing_data, sizeof(struct feeder_out_t)));
            
            // Release the mutex
            xSemaphoreGive(feederStructMutex);
           
            // Delay to prevent flooding the network
            //vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    xSemaphoreTake(feederStructMutex, portMAX_DELAY); //Take MUTEX to access the feeder struct

    char buffer[13];
    switch (status)
    {
    case ESP_NOW_SEND_SUCCESS:
        ESP_LOGI("MAC_ADDRESS", "message sent to %s", mac_to_str(buffer, (uint8_t *)mac_addr));
        break;
    case ESP_NOW_SEND_FAIL:
        ESP_LOGE("MAC_ADDRESS", "message sent to %s failed", mac_to_str(buffer, (uint8_t *)mac_addr));
        break;
    }

    xSemaphoreGive(feederStructMutex); //Give MUTEX to allow other tasks to access the feeder struct
}

void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    // Take mutex to safely access the shared feeder struct
    xSemaphoreTake(feederStructMutex, portMAX_DELAY);
    
    // Log the MAC address of the sender
    ESP_LOGI("MAC_ADDRESS", "Got message from " MACSTR, MAC2STR(esp_now_info->src_addr));
    
    // Check if the data size matches our expected struct size
    if (data_len == sizeof(struct feeder_in_t)) {
        struct feeder_in_t temp_incoming;
        
        // Cast the received data to our temp struct 
        memcpy(&temp_incoming, data, sizeof(struct feeder_in_t));
        
        // Check if we've completed the previous job
        if (feeder_outgoing.processedAmount >= feeder_incoming.setAmount) {
            // Job is complete, reset the counter for the new job
            feeder_outgoing.processedAmount = 0;
            ESP_LOGI("ESP_NOW", "Previous job complete. Resetting processed amount for new job.");
        }
        
        // Now copy to our global struct
        memcpy(&feeder_incoming, &temp_incoming, sizeof(struct feeder_in_t));
        
        // Log the received data for debugging
        ESP_LOGI("ESP_NOW", "Received struct data: id=%d, setAmount=%d, setLength=%d, startStop=%d, abort=%d",
               feeder_incoming.id,
               feeder_incoming.setAmount,
               feeder_incoming.setLength,
               feeder_incoming.flagStartStop,
               feeder_incoming.flagAbort);
    } else {
        // Handle unexpected data size
        ESP_LOGW("ESP_NOW", "Received data with unexpected size: %d bytes (expected %d bytes)", 
                data_len, sizeof(struct feeder_in_t));
    }
    
    // Release the mutex
    xSemaphoreGive(feederStructMutex);
}

char *mac_to_str(char *buffer, uint8_t *mac)
{
    // Make sure the buffer is properly initialized
    memset(buffer, 0, 18);  // Clear buffer (allocate at least 18 bytes for the buffer)
    
    // Format MAC address with proper null termination
    snprintf(buffer, 18, MACSTR, MAC2STR(mac));
    
    return buffer;
}

void StepperTask(void *pvParameters){
    static const char *STEPPER_TAG = "STEPPER TASK";
    ESP_LOGI(STEPPER_TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN | 1ULL << STEP_MOTOR_GPIO_MS1 | 1ULL << STEP_MOTOR_GPIO_MS2,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    gpio_set_level(STEP_MOTOR_GPIO_MS1, 1);
    gpio_set_level(STEP_MOTOR_GPIO_MS2, 1);

    ESP_LOGI(STEPPER_TAG, "Create RMT TX channel");
    rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    ESP_LOGI(STEPPER_TAG, "Set spin direction");
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_LOGI(STEPPER_TAG, "Enable step motor");
    gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

    ESP_LOGI(STEPPER_TAG, "Create motor encoders");
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

    ESP_LOGI(STEPPER_TAG, "Enable RMT channel");
    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    ESP_LOGI(STEPPER_TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    const static uint32_t accel_samples = STEP_MOTOR_ACCEL_DECEL_SAMPLES;
    const static uint32_t uniform_speed_hz = STEP_MOTOR_ACCEL_DECEL_FREQ;
    const static uint32_t decel_samples = STEP_MOTOR_ACCEL_DECEL_SAMPLES;
    
    ESP_LOGI(STEPPER_TAG, "Steps per millimeter: %d", STEPS_PER_mm); 
    ESP_LOGI(STEPPER_TAG, "Steps per micrometer: %d", STEPS_PER_um); 

    
    while (1)
    {
        xSemaphoreTake(feederStructMutex, portMAX_DELAY); //Take MUTEX to access the feeder struct

        if((feeder_incoming.setAmount > feeder_outgoing.processedAmount) && (feeder_incoming.flagStartStop && !feeder_incoming.flagAbort && feeder_outgoing.flagStartStop && !feeder_outgoing.flagAbort && !feeder_outgoing.runOut)){
            // acceleration phase
            tx_config.loop_count = 0; 
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));

            // uniform phase
            //tx_config.loop_count = counter;
            tx_config.loop_count = (feeder_incoming.setLength * (STEPS_PER_um/1000)) - STEP_MOTOR_ACCEL_DECEL_SAMPLES;
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));

            // deceleration phase
            tx_config.loop_count = 0;
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
            // wait all transactions finished
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));



            feeder_outgoing.processedAmount ++; 
            ESP_LOGI(STEPPER_TAG, "Feeding finished. Processed amount: %d", feeder_outgoing.processedAmount);
            ESP_LOGI(STEPPER_TAG, "Remaining amount: %d", feeder_incoming.setAmount - feeder_outgoing.processedAmount); 

            xSemaphoreGive(triggerSemaphore); //Give trigger to the ESP-NOW task to send the data to the display unit
        } //else dann motor off

        xSemaphoreGive(feederStructMutex); //Give MUTEX to allow other tasks to access the feeder struct
        vTaskDelay(pdMS_TO_TICKS(2500)); //Delay the task for 2 seconds  
    }
}

void SafetyTask(void *pvParameters){ //auch für runout benutzen
    touch_pad_init();
    touch_pad_config(TOUCH_GPIO); 
    touch_pad_denoise_t denoise = {
        /* The bits to be cancelled are determined according to the noise level. */
        .grade = TOUCH_PAD_DENOISE_BIT4,
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
    };
    touch_pad_denoise_set_config(&denoise);
    touch_pad_denoise_enable();
    /* Enable touch sensor clock. Work mode is "timer trigger". */
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();
    

    uint32_t touch_value = 0;
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    while (1)
    {
        xSemaphoreTake(feederStructMutex, portMAX_DELAY); // Take MUTEX to access the feeder struct
        touch_pad_read_raw_data(TOUCH_GPIO, &touch_value);    // read raw data.
        //ESP_LOGI("TOUCH", "Touch value: %" PRIu32, touch_value);

        if (touch_value > THRESHOLD) {
            ESP_LOGW("TOUCH", "Touch detected! Touch value: %" PRIu32, touch_value);

            feeder_outgoing.flagAbort = 1; // Set flagAbort to 1
            
            // Abort the stepper motor task and disabling the motor
            gpio_set_level(STEP_MOTOR_GPIO_EN, !STEP_MOTOR_ENABLE_LEVEL);

            xSemaphoreGive(triggerSemaphore); // Trigger the ESP-NOW task
        }

        xSemaphoreGive(feederStructMutex); // Release MUTEX
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

void LedTask(void *pvParameters){

    led_strip_config_t strip_config = {
    .strip_gpio_num = LED_STRIP_GPIO,  // The GPIO that connected to the LED strip's data line
    .max_leds = LED_STRIP_NUM_LEDS,                 // The number of LEDs in the strip,
    .led_model = LED_MODEL_WS2812, // LED strip model, it determines the bit timing
    .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color component format is G-R-B
    .flags = {
        .invert_out = false, // don't invert the output signal
        }
    };

    /// RMT backend specific configuration
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,    // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // RMT counter clock frequency: 10MHz
        .mem_block_symbols = 64,           // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }
    };

    /// Create the LED strip object
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    
    static bool led_state = false; // For flashing behavior
    
    while (1)
    {   
        xSemaphoreTake(feederStructMutex, portMAX_DELAY); // Take MUTEX to access the feeder struct
        for (int i = 0; i < LED_STRIP_NUM_LEDS; i++) {
            if (feeder_incoming.flagAbort || feeder_outgoing.runOut || feeder_outgoing.flagAbort) {
            // Flash red (abort or runout)
            if (led_state) {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 0)); // Red
            } else {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0)); // Turn off LEDs
            }
            } else {
            switch (feeder_incoming.flagStartStop) {
                case 0: // No job or paused
                // Solid green (no job)
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 255, 0)); // Green
                break;
    
                case 1: // Job in progress
                // Solid yellow
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 255, 0)); // Yellow
                break;
    
                default:
                ESP_LOGW("LED_TASK", "Unexpected flagStartStop value: %d", feeder_incoming.flagStartStop);
                break;
            }
            }
        }
    
        ESP_ERROR_CHECK(led_strip_refresh(led_strip)); // Refresh the LED strip
        xSemaphoreGive(feederStructMutex); // Release MUTEX
    
        led_state = !led_state; // Toggle the LED state for flashing behavior
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 1 second
    }
}