#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// WiFi credentials for an open network
const char* ssid = "your_SSID"; // No password needed for open networks

// MQTT server
const char* mqtt_server = "your_MQTT_BROKER_ADDRESS";
const char* rest_api_server = "http://your_rest_api_server/api/register"; // REST API endpoint

// Motor Pins
#define LEFT_MOTOR_PIN1 5
#define LEFT_MOTOR_PIN2 18
#define RIGHT_MOTOR_PIN1 19
#define RIGHT_MOTOR_PIN2 21
#define ACCELERATE_MOTOR_PIN1 22
#define ACCELERATE_MOTOR_PIN2 23
#define REVERSE_MOTOR_PIN1 25
#define REVERSE_MOTOR_PIN2 26

// Treat Box Motor Pins
#define TREATBOX_OPEN_PIN1 13
#define TREATBOX_OPEN_PIN2 12
#define TREATBOX_CLOSE_PIN1 14
#define TREATBOX_CLOSE_PIN2 27

// Ultrasonic Sensor Pins
#define TRIG_PIN1 32
#define ECHO_PIN1 33
#define TRIG_PIN2 4
#define ECHO_PIN2 2
#define TRIG_PIN3 15
#define ECHO_PIN3 16

WiFiClient espClient;
PubSubClient client(espClient);

String mqttChannelID;
String collar_id;
String robo_id = "robot_123"; // Example robot ID

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

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  String message = String((char*)payload).substring(0, length);
  if (message == "target achieved") {
    openTreatBox();
  } else {
    // Check for obstacles and navigate
    navigateRobot();
  }
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
      collar_id = advertisedDevice.getAddress().toString().c_str();

      // Make a POST request to the REST API
      HTTPClient http;
      http.begin(rest_api_server);
      http.addHeader("Content-Type", "application/json");
      String postData = "{\"collar_id\":\"" + collar_id + "\",\"robo_id\":\"" + robo_id + "\"}";
      int httpResponseCode = http.POST(postData);

      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println(httpResponseCode);
        Serial.println(response);

        // Assuming the response contains the channel ID as a plain string
        mqttChannelID = response;
        client.setServer(mqtt_server, 1883);
        client.setCallback(callback);
        client.subscribe(mqttChannelID.c_str());
      } else {
        Serial.printf("POST request failed: %d\n", httpResponseCode);
      }

      http.end();
    }
  }
};

void setup() {
  Serial.begin(115200);
  
  // Setup Motor Pins
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(ACCELERATE_MOTOR_PIN1, OUTPUT);
  pinMode(ACCELERATE_MOTOR_PIN2, OUTPUT);
  pinMode(REVERSE_MOTOR_PIN1, OUTPUT);
  pinMode(REVERSE_MOTOR_PIN2, OUTPUT);

  // Setup Treat Box Motor Pins
  pinMode(TREATBOX_OPEN_PIN1, OUTPUT);
  pinMode(TREATBOX_OPEN_PIN2, OUTPUT);
  pinMode(TREATBOX_CLOSE_PIN1, OUTPUT);
  pinMode(TREATBOX_CLOSE_PIN2, OUTPUT);

  // Setup Ultrasonic Sensor Pins
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(ECHO_PIN3, INPUT);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  BLEDevice::init("Robot");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30, false);
}

long measureDistance(int trigPin, int echoPin) {
  long duration, distance;
  
  // Send a 10us pulse to the TRIG_PIN
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the duration of the pulse on the ECHO_PIN
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  distance = duration * 0.034 / 2;

  return distance;
}

void navigateRobot() {
  long distanceLeft = measureDistance(TRIG_PIN1, ECHO_PIN1);
  long distanceFront = measureDistance(TRIG_PIN2, ECHO_PIN2);
  long distanceRight = measureDistance(TRIG_PIN3, ECHO_PIN3);

  // Determine obstacle direction
  if (distanceFront < 10) {
    Serial.println("Obstacle detected in front, moving backward");
    moveBackward();
  } else if (distanceLeft < 10) {
    Serial.println("Obstacle detected on the left, turning right");
    turnRight();
  } else if (distanceRight < 10) {
    Serial.println("Obstacle detected on the right, turning left");
    turnLeft();
  } else {
    Serial.println("No obstacle, moving forward");
    moveForward();
  }
}

void moveForward() {
  digitalWrite(ACCELERATE_MOTOR_PIN1, HIGH);
  digitalWrite(ACCELERATE_MOTOR_PIN2, LOW);
  digitalWrite(REVERSE_MOTOR_PIN1, LOW);
  digitalWrite(REVERSE_MOTOR_PIN2, LOW);
}

void moveBackward() {
  digitalWrite(ACCELERATE_MOTOR_PIN1, LOW);
  digitalWrite(ACCELERATE_MOTOR_PIN2, LOW);
  digitalWrite(REVERSE_MOTOR_PIN1, HIGH);
  digitalWrite(REVERSE_MOTOR_PIN2, LOW);
}

void turnLeft() {
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void turnRight() {
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void openTreatBox() {
  // Open the treat box
  digitalWrite(TREATBOX_OPEN_PIN1, HIGH);
  digitalWrite(TREATBOX_OPEN_PIN2, LOW);
  delay(2000); // Keep it open for 2 seconds
  
  // Close the treat box
  digitalWrite(TREATBOX_OPEN_PIN1, LOW);
  digitalWrite(TREATBOX_OPEN_PIN2, LOW);
  digitalWrite(TREATBOX_CLOSE_PIN1, HIGH);
  digitalWrite(TREATBOX_CLOSE_PIN2, LOW);
  delay(2000); // Keep it closed for 2 seconds
  
  // Stop the treat box motors
  digitalWrite(TREATBOX_CLOSE_PIN1, LOW);
  digitalWrite(TREATBOX_CLOSE_PIN2, LOW);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
