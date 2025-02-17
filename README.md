# Belimo RoomUnit ESP32-C3

This project reads Temperature, Humidity, and CO2 levels from a Belimo Room Unit sensor using a Seeed XIAO ESP32-C3 and publishes the data via MQTT.

## Hardware Requirements

- Seeed XIAO ESP32-C3 board (https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/)
- Belimo Room Unit sensor
- Power supply (USB or external)
- Connecting wires

## Wiring

Connect the Belimo Room Unit to the ESP32-C3 as follows:

| Belimo Signal | ESP32-C3 Pin | ADC Channel |
|---------------|--------------|-------------|
| Temperature   | GPIO2/A2     | ADC2        |
| Humidity      | GPIO3/A3     | ADC3        |
| CO2           | GPIO4/A4     | ADC4        |
| GND           | GND          | -           |

## Software Setup

### Prerequisites

1. Install ESP-IDF (Espressif IoT Development Framework)
2. Clone this repository
3. Navigate to the project directory

### Configuration

1. Configure the WiFi and other project settings using:
```bash
idf.py menuconfig
```

Under "Example Configuration":
- Set WiFi SSID
- Set WiFi Password

2. Update the following parameters in `main/main.c` according to your setup:
```c
#define MQTT_BROKER_URL "mqtt://192.168.0.68"    // Your MQTT broker IP/hostname
#define MQTT_PORT 1883                           // Your MQTT broker port
#define MQTT_TOPIC_TEMP "sensors/temperature_2"  // MQTT topic for temperature
#define MQTT_TOPIC_HUMIDITY "sensors/humidity_2" // MQTT topic for humidity
#define MQTT_TOPIC_CO2 "sensors/co2_2"          // MQTT topic for CO2
```

3. Optional settings you can modify:
- `LED_BLINK_ENABLED`: Set to `true` to enable status LED blinking
- `FILTER_SIZE`: Number of samples for reading averaging (default: 16)
- `SAMPLE_INTERVAL`: Time between samples in ms (default: 100)

4. Flash to your device:

```bash
idf.py -p [PORT] flash
```
```bash
idf.py monitor
```

## MQTT Data Format

The device publishes JSON data to three topics:

1. Temperature (sensors/temperature):

```json
{"voltage": 1.23, "temperature": 24.5}
```

2. Humidity (sensors/humidity):

```json
{"voltage": 1.23, "humidity": 50.0}
```

3. CO2 (sensors/co2):

```json
{"voltage": 1.45, "humidity": 45.2}
```

{"voltage": 1.67, "co2": 850.0}


## Sensor Ranges

- Temperature: 0°C to 50°C
- Humidity: 0% to 100%
- CO2: 0 to 2000 ppm

## LED Indicator

The onboard LED (GPIO10) can be enabled for status indication by setting `LED_BLINK_ENABLED` to `true` in the code.

## Troubleshooting

1. If the device fails to connect to WiFi:
   - Check your WiFi credentials
   - Ensure the device is within range of your WiFi network
   - Monitor the serial output for connection status

2. If MQTT publishing fails:
   - Verify your MQTT broker is running
   - Check the broker URL and port
   - Ensure your network allows MQTT traffic

## License

This project is open-source and available under the MIT License.

## Contributing

Feel free to submit issues and pull requests to improve this project.