#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_log.h"

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

// Add these definitions
#define WIFI_SSID "ItHurtsWhenIP"
#define WIFI_PASS "11vcwec2"
#define MQTT_BROKER_URL "mqtt://192.168.0.68"
#define MQTT_PORT 1883
#define MQTT_TOPIC "sensors/temperature"

static const char *TAG = "TEMP_SENSOR";
static esp_mqtt_client_handle_t mqtt_client = NULL;

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

static void wait_for_wifi(void)
{
    // Wait for WiFi connection
    int retry = 0;
    while (retry < 10) {
        wifi_ap_record_t ap_info;
        esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "WiFi connected with IP:");
            return;
        }
        ESP_LOGI(TAG, "Waiting for WiFi connection... (%d/10)", retry + 1);
        retry++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGE(TAG, "Failed to connect to WiFi!");
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_LOGI(TAG, "Setting WiFi configuration...");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    wait_for_wifi();
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected to broker at %s", MQTT_BROKER_URL);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGE(TAG, "MQTT Disconnected from broker. Will auto-reconnect...");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error Event");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGE(TAG, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno,
                        strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT Attempting connection to broker...");
            break;
        default:
            ESP_LOGI(TAG, "Other MQTT event id: %d", event->event_id);
            break;
    }
}

static void mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
        .broker.address.port = MQTT_PORT,
        .session.keepalive = 15,
    };
    
    ESP_LOGI(TAG, "Initializing MQTT client for broker %s:%d", MQTT_BROKER_URL, MQTT_PORT);
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client!");
        return;
    }
    
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

// Modify app_main() to initialize WiFi and MQTT
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    wifi_init();

    // Initialize MQTT
    mqtt_init();

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
        
        // Create JSON string for MQTT
        char mqtt_data[100];
        snprintf(mqtt_data, sizeof(mqtt_data), 
                "{\"adc_raw\":%d,\"voltage\":%.2f,\"temperature\":%.1f}", 
                adc_raw, voltage/1000.0f, temperature);
        
        // Publish to MQTT
        if (mqtt_client != NULL) {
            int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, mqtt_data, 0, 1, 0);
            if (msg_id != -1) {
                ESP_LOGI(TAG, "Published: %s", mqtt_data);
            }
        } else {
            ESP_LOGE(TAG, "MQTT client not initialized");
        }
        
        // Print detailed readings
        printf("ADC Raw: %d, Voltage: %.2fV, Temperature: %.1f°C\n", 
               adc_raw, voltage/1000.0f, temperature);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL));
    }
}