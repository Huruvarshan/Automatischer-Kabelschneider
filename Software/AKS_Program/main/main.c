#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"
//#include "led_strip.h" 
#include "esp_random.h"
#include "math.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "string.h"
#include "AKS.h"


char *mac_to_str(char *buffer, uint8_t *mac);

struct feeder_t {
    uint16_t setAmount;        // Received from external source
    uint16_t setLength;        // Received from external source
    uint16_t processedAmount;  // Sent to external device
    bool flagStartStop;        // Received from external source
    bool flagAbort;            // Received from external source
    bool flagError;            // Sent to external device
    bool flagUpdate;           // Internal: indicates new data is available
} feeder;


void app_main(void)
{
    uint8_t my_mac[6];
    char my_mac_str[13];

    esp_efuse_mac_get_default(my_mac);
    ESP_LOGI("MAC_ADDRESS", "My mac: %s", mac_to_str(my_mac_str, my_mac));
}


// Convert MAC address to string
char *mac_to_str(char *buffer, uint8_t *mac)
{
    sprintf(buffer, MACSTR, MAC2STR(mac));
    return buffer;
}
