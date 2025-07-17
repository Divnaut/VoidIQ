#include <Arduino.h>
#include "shared_defs.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TaskScheduler.h>
// #include <WrapperFreeRTOS.h>
//change as per your own configuration:
const char * ssid="wifi_ssid";
const char * password="wifi_password";
const char * mqtt_server = "broker_ip";
//set your own topic: 
#define mqtt_topic "wh1/sensors/height/"
#define MAX_READINGS 10

WiFiClient espClient;
PubSubClient client(espClient);

size_t received_msg_length;
uint8_t incomingData[sizeof(struct sensor_data)];
sensor_data received_data;

void setup_wifi();
void setup_mqtt();
void mqtt_reconnect();

void setup(){
  Serial.begin(9600);
  setup_wifi();
  setup_mqtt();
  Serial.printf("Size of sensor_data: %d\n", sizeof(sensor_data));

}

void loop() {
  mqtt_reconnect();
  client.loop();

  static uint8_t buffer[sizeof(sensor_data)];
  static size_t bytes_read = 0;

  while (Serial.available()) {
    uint8_t incoming = Serial.read();

    if (bytes_read == 0) {
      if (incoming == 0xA5) {
        buffer[0] = incoming;
        bytes_read = 1;
      }
    } else {
      buffer[bytes_read++] = incoming;
      if (bytes_read == sizeof(sensor_data)) {
        memcpy(&received_data, buffer, sizeof(sensor_data));
        bytes_read = 0;

        // Construct and publish MQTT payload
        char payload[128];
        snprintf(payload, sizeof(payload),
                 "{\"node_id\":%d,\"sensor_id\":%d,\"distance\":%d,\"wh_no\":1}",
                 received_data.node_id,
                 received_data.sensor_id,
                 received_data.distance);

        Serial.printf("Publishing: %s\n", payload);
        if (!client.publish(mqtt_topic, payload)) {
          Serial.println("MQTT publish failed");
        }
      }
    }
  }
}
//----- WiFi Setup -----
void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
}

void setup_mqtt() {
  Serial.printf("connecting to mqtt server %s", mqtt_server);
  client.setServer(mqtt_server, 1883);
  client.setKeepAlive(30);  
  mqtt_reconnect();
}


void mqtt_reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Master")) {
      Serial.println("connected");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5s");
      delay(1000);
    }
  }
}
