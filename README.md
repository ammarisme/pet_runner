
# Pet Collar & Bot Project (IoT / RL)

This project aims to develop an ESP32-based pet collar that tracks a pet's movement and sends relevant data to a remote server. The collar connects to an open Wi-Fi network and uses BLE to communicate with a nearby robot device. The project includes features like RSSI (signal strength) monitoring, distance calculation using an accelerometer, and MQTT communication.

## Components Used
- ESP32 Development Board x 2
- ADXL345 Accelerometer
- Wi-Fi Network (Open, no password)
- BLE-enabled Robot Device
- MQTT Broker

## Libraries Required
- WiFi
- PubSubClient
- BLEDevice
- HTTPClient
- Adafruit_Sensor
- Adafruit_ADXL345_U


```

## Setup Instructions

### 1. Clone the Repository
Clone the repository to your local machine:
```sh
git clone https://github.com/yourusername/your_project.git
cd your_project
```

### 2. Configure the Project
Ensure your project includes the necessary components in `CMakeLists.txt`.

#### Top-level CMakeLists.txt:
```cmake
cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(your_project)
```

#### `main/CMakeLists.txt`:
```cmake
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi esp_http_client mqtt freertos adafruit_adxl345)
```

### 3. Build and Flash the Project
Use the ESP-IDF build system to build and flash the project.

#### Build the Project:
```sh
idf.py build
```

#### Flash the Project:
```sh
idf.py flash
```

#### Monitor the Output:
```sh
idf.py monitor
```

## Features
- **Wi-Fi Connectivity:** Connects to an open Wi-Fi network and logs the IP address.
- **BLE Communication:** Scans for nearby BLE devices and connects to a robot device when found.
- **Accelerometer Data:** Uses the ADXL345 accelerometer to measure acceleration and calculate displacement.
- **MQTT Communication:** Publishes RSSI and displacement data to a specified MQTT channel every 5 seconds.
- **HTTP Communication:** Registers the device with a robot via an HTTP POST request to receive the MQTT channel ID.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
