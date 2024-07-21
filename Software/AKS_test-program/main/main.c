#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"

#define PIN 1 // Defining the GPIO pin number

/*
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
}*/

void taskGetInputState(){
  while (1)
  {
    if(gpio_get_level(PIN))
    {
      printf("IO is 1 \n");
    } else {
      printf("IO is 0 \n");
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void app_main(void){
  gpio_set_direction(PIN, GPIO_MODE_INPUT); // Set the GPIO as a input
  gpio_pullup_en(PIN); // Enable the pull-up resistor 
  xTaskCreate(&taskGetInputState, "taskGetInputState", 2048, NULL, 1, NULL); // Create the task that will read the GPIO state
}