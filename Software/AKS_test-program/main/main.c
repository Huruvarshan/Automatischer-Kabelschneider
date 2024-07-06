#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"

#define PIN 17

void app_main(void)
{  
  gpio_set_direction(PIN, GPIO_MODE_OUTPUT);
  int isOn = 0;
  while (true)
  {
    isOn = !isOn;
    gpio_set_level(PIN, isOn);
    vTaskDelay(pdMS_TO_TICKS(1000));    
  }
}