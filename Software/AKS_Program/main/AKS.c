// AKS.c

#include <stdio.h>
#include "AKS.h"
 
// Function declarations 
/*char *mac_to_str(char *buffer, uint8_t *mac);*/
void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);

// Function definitions

// Convert MAC address to string
/*char *mac_to_str(char *buffer, uint8_t *mac)
{
    // sprintf(buffer, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // below is another way to do this
    sprintf(buffer, MACSTR, MAC2STR(mac));
    return buffer;
}*/

// Callback function when message is sent
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

// Callback function when message is received
void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
  ESP_LOGI("MAC_ADDRESS", "got message from " MACSTR, MAC2STR(esp_now_info->src_addr));
  printf("message: %.*s\n", data_len, data);
}