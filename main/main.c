#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO GPIO_NUM_10  // Define the GPIO pin for the LED

void app_main(void)
{
    // Configure the LED GPIO as an output
    esp_rom_gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_GPIO, 1);            // Turn the LED on
        vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait for 1 second
        gpio_set_level(LED_GPIO, 0);            // Turn the LED off
        vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait for 1 second
    }
}