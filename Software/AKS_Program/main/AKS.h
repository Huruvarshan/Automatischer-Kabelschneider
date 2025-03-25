#ifndef AKS_H
#define AKS_H

#include <stdint.h>
#include <esp_now.h>
#include <esp_log.h>

// Function declarations
/*char *mac_to_str(char *buffer, uint8_t *mac);*/
void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);

#endif // AKS_H