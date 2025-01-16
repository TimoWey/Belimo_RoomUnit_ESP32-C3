#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO GPIO_NUM_10
#define TEMP_ADC_CHANNEL ADC_CHANNEL_2
#define ADC_RAW_MAX 4095  // 12-bit ADC (2^12 - 1)
#define VOLTAGE_MIN 0     // mV at 0°C
#define VOLTAGE_MAX 2500  // mV at 50°C
#define TEMP_MIN 0.0f     // Temperature at VOLTAGE_MIN
#define TEMP_MAX 50.0f    // Temperature at VOLTAGE_MAX

// Filter settings
#define FILTER_SIZE 16    // Increased from 1 to 16 samples
#define SAMPLE_INTERVAL 100 // Decreased to 10ms between samples

// Function to get filtered ADC reading
float get_filtered_voltage(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t cali_handle) {
    float sum = 0;
    
    // Take multiple readings
    for(int i = 0; i < FILTER_SIZE; i++) {
        int adc_raw;
        int voltage;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, TEMP_ADC_CHANNEL, &adc_raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, adc_raw, &voltage));
        sum += voltage;
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL));
    }
    
    // Return average
    return sum / FILTER_SIZE;
}

void led_blink_task(void *pvParameters) {
    while(1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Configure LED GPIO
    esp_rom_gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // ADC Init
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC Config
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, TEMP_ADC_CHANNEL, &config));

    // ADC Calibration Init
    adc_cali_handle_t adc1_cali_handle = NULL;
    
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle));

    // Initial reading to stabilize
    get_filtered_voltage(adc1_handle, adc1_cali_handle);

    // Create LED blink task
    xTaskCreate(led_blink_task, "led_blink_task", 2048, NULL, 5, NULL);

    while (1) {
        // Get filtered voltage reading
        float voltage = get_filtered_voltage(adc1_handle, adc1_cali_handle);
        int adc_raw;
        adc_oneshot_read(adc1_handle, TEMP_ADC_CHANNEL, &adc_raw);
        
        // Convert voltage to temperature (0-2.5V maps to 0-50°C)
        float temperature = (voltage * 50.0f) / VOLTAGE_MAX;
        
        // Print detailed readings
        printf("ADC Raw: %d, Voltage: %.2fV, Temperature: %.1f°C\n", adc_raw, voltage/1000.0f, temperature);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL));
    }
}