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
//#include "AKS.h"
#include "stepper_motor_encoder.h"
//#include "led_strip.h" 
//#include "tmc2208.h" //Include wenn die MS1 und MS2 Pins nicht mehr gehardwired sind. 


void EspNowTask(void *pvParameters); 
void StepperTask(void *pvParameters); 

char *mac_to_str(char *buffer, uint8_t *mac);
void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status); 
void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len); 

SemaphoreHandle_t feederStructMutex; 

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

    //feeder.incoming.id = //set to gpio mäuseklavier
    feeder_incoming.setAmount = 10; // Edit to 0 later
    feeder_incoming.setLength = 100; // Edit to 0 later
    feeder_incoming.flagStartStop = 1; 
    feeder_incoming.flagAbort = 0; 

    //feeder_outgoing.id //set to gpio mäuseklavier
    feeder_outgoing.processedAmount = 0; 
    feeder_outgoing.flagStartStop = 1; 
    feeder_outgoing.flagAbort = 0;
    feeder_outgoing.runOut = 0;

    xTaskCreate(StepperTask, "StepperTask", 8192, NULL, 4, NULL);
}



void EspNowTask(void *pvParameters){
    uint8_t macDisplay[6] = {0xe4, 0x65, 0xb8, 0x7e, 0x27, 0x98}; 
    uint8_t my_mac[6];
    char send_buffer[250];
    char my_mac_str[13];
    //char send_buffer[250];


    esp_efuse_mac_get_default(my_mac);
    ESP_LOGI("MAC_ADDRESS", "My mac %s", mac_to_str(my_mac_str, my_mac));

    nvs_flash_init();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_receive));

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    memcpy(peer.peer_addr, macDisplay, 6);
    peer.channel = 0;  
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));


    while (1)
    {
        sprintf(send_buffer, "Hello from %s. Es stinkt!", my_mac_str);
        ESP_ERROR_CHECK(esp_now_send(NULL, (uint8_t *)send_buffer, strlen(send_buffer)));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
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
}

void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
  ESP_LOGI("MAC_ADDRESS", "got message from " MACSTR, MAC2STR(esp_now_info->src_addr));
  printf("message: %.*s\n", data_len, data);
}

char *mac_to_str(char *buffer, uint8_t *mac)
{
    sprintf(buffer, MACSTR, MAC2STR(mac));
    return buffer;
}

void StepperTask(void *pvParameters){
    static const char *STEPPER_TAG = "STEPPER TASK";
    ESP_LOGI(STEPPER_TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

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
        } 

        xSemaphoreGive(feederStructMutex); //Give MUTEX to allow other tasks to access the feeder struct
        vTaskDelay(pdMS_TO_TICKS(2500)); //Delay the task for 2 seconds  
    }
}