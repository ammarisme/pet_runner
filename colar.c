#include <WiFi.h>
#include <PubSubClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// WiFi credentials for an open network
const char* ssid = "your_SSID"; // No password needed for open networks

// MQTT server
const char* mqtt_server = "your_MQTT_BROKER_ADDRESS";

// Accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

const int sampleRate = 100; // Sample rate in Hz
const float dt = 1.0 / sampleRate; // Time step in seconds
float velocity[3] = {0, 0, 0}; // Velocity in X, Y, Z directions
float displacement[3] = {0, 0, 0}; // Displacement in X, Y, Z directions

unsigned long lastMsg = 0;
bool deviceConnected = false;
String mqttChannelID = "your_mqtt_channel"; // Example channel ID

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid); // No password needed for open networks

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(mqttChannelID.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(BLEUUID((uint16_t)0x181D))) {
      BLEDevice::getScan()->stop();
      BLEClient* pClient = BLEDevice::createClient();
      pClient->connect(&advertisedDevice);

      if (pClient->isConnected()) {
        deviceConnected = true;
        Serial.println("Connected to robot");
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  if (!accel.begin()) {
    Serial.println("No ADXL345 detected ... Check your wiring!");
    while (1);
  }

  accel.setRange(ADXL345_RANGE_16_G);

  BLEDevice::init("Pet_Collar");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30, false);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  sensors_event_t event;
  accel.getEvent(&event);

  float acceleration[3] = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

  for (int i = 0; i < 3; i++) {
    velocity[i] += acceleration[i] * dt;
  }

  for (int i = 0; i < 3; i++) {
    displacement[i] += velocity[i] * dt;
  }

  long rssi = WiFi.RSSI();

  unsigned long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    String msg = String("RSSI: ") + String(rssi) + " dBm, " +
                 "Distance X: " + String(displacement[0]) + " m, " +
                 "Y: " + String(displacement[1]) + " m, " +
                 "Z: " + String(displacement[2]) + " m";

    Serial.println(msg);
    client.publish(mqttChannelID.c_str(), msg.c_str());
  }

  delay(1000 / sampleRate); // Maintain the sample rate
}
