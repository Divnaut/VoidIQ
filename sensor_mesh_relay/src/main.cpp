#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "shared_defs.h"
#include <vector>

WiFiClient espClient;
//PubSubClient client(espClient);

// ----- FreeRTOS Queue -----
QueueHandle_t msg_queue;

// ----- Pin Vector-----
std::vector<SensorPin> pins = {
  {23, 22, 3}, 
  {18, 19, 4}   
};

//----- Function Prototypes -----
void on_recv_callback(const uint8_t *mac_addr, const uint8_t *data, int data_len);
// void task_forward_to_mqtt(void *params);
void task_sleep_controller(void *params);
void take_readings(void *vparams);
void transmit_data(void *vparams);
void initEspNow();

// ----- ESP-NOW Settings -----
esp_now_peer_info_t peerInfo;

//----- Task Handles -----
TaskHandle_t task_take_readings = NULL; // handle for the take_readings task
TaskHandle_t task_transmit_data = NULL; // handle for the transmit_data task

// ----- Setup -----
void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true); // Enable promiscuous mode
  esp_wifi_set_channel(2, WIFI_SECOND_CHAN_NONE); // Set channel to default channel 2
  esp_wifi_set_promiscuous(false); // Disable promiscuous mode

  initEspNow();

  Serial.print("This device MAC");
  Serial.println(WiFi.macAddress());

  esp_now_register_recv_cb(on_recv_callback);
 
  msg_queue = xQueueCreate(40, sizeof(sensor_data)); // Initialize the queue   
  xTaskCreatePinnedToCore(
    take_readings,           // Task function
    "UltrasonicReadings",    // Name
    2048,                    // Stack size
    &pins,                   // Parameter (pointer to pins vector)
    1,                       // Priority
    &task_take_readings,     // Task handle
    1                        // Core
  );
  xTaskCreatePinnedToCore(
    transmit_data,           // Task function
    "TransmitData",          // Name
    4096,                    // Stack size
    NULL,                    // Parameter (not used)
    1,                       // Priority
    &task_transmit_data,     // Task handle
    1                        // Core
  );
  Serial.printf("Size of sensor_data: %d\n", sizeof(sensor_data));
}

void initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW failed to initialize");
    while (1);
  }
  // memcpy(peerInfo.peer_addr, transmit_address, 6);
  // peerInfo.ifidx   = (wifi_interface_t)ESP_IF_WIFI_STA;
  // peerInfo.encrypt = false; 
  peerInfo.channel = 0; // Use default channel, will be set to the current channel of the station

  // if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  //   Serial.println("ESP NOW pairing failure");
  //   while (1);
  // }
}

// void mqtt_setup() {
//   client.setServer(mqtt_server, 1883);
//   client.setKeepAlive(60);  
//   mqtt_connect();
// }

// ----- ESP-NOW Callback -----
void on_recv_callback(const uint8_t *mac_addr, const uint8_t *data, int data_len){
  //add received data to queue:
  //incoming data will be in the form of a msg struct
  Serial.printf("Received data of length: %d\n", data_len);
  Serial.printf("Received data from MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  msg incoming_message;
  sensor_data incoming_data;
  memcpy(&incoming_message, data, sizeof(msg));
  for (int i = 0; i < incoming_message.num_data; i++) {
    incoming_data = incoming_message.readings[i];
    //BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSend(msg_queue, &incoming_data, portMAX_DELAY);
    Serial.printf("Data moved to Queue: Sensor ID %d: Distance = %d cm\n", incoming_data.sensor_id, incoming_data.distance);
  }
}

void task_sleep_controller(void *params) {
  vTaskDelay(pdMS_TO_TICKS(20000)); // Wait 20 seconds (active window)

  //Serial.println("Entering deep sleep for 20 seconds...");
  //esp_sleep_enable_timer_wakeup(20 * 1000000);  // 20s in microseconds
 

  //esp_deep_sleep_start();  // Never returns
  vTaskDelete(NULL); // Delete this task after sleep
}
// ----- Sensor Reading Task -----
void take_readings(void *vparams){
  std::vector<SensorPin>* pins = static_cast<std::vector<SensorPin>*>(vparams); 
  sensor_data sensor_data;// cast the parameter to the correct type
  while(1){
    for (int i = 0; i < pins->size(); i++) {
      int trigPin = (*pins)[i].trig;
      int echoPin = (*pins)[i].echo;
      long distance = 0;
      int j=0;
      int flag = 0; 
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
      //loop will exit if either 3 non zero and <440 readings are obtained, otherwise return 0;
      while(1){
        digitalWrite(trigPin, LOW);
        ets_delay_us(2);
        //delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        ets_delay_us(10);
        digitalWrite(trigPin, LOW);

        int temp_distance = pulseIn(echoPin, HIGH)*0.0343/2;
        if(temp_distance > 2 && temp_distance < 440){
          distance += temp_distance;
          j++;
        } 
        else {
          if(flag >= 6){
            distance = 0; // reset distance if we have 3 invalid readings
            break; 
          }
          flag++;
          vTaskDelay(pdMS_TO_TICKS(300));
          //delay(300);
          continue; // skip this reading if no echo is received
        }
        vTaskDelay(pdMS_TO_TICKS(300));
        if(j >= 8) break; // exit loop after 8 valid readings per sensor
        
      }
      //distance = distance>=2 && distance<=440 ? distance : 0; // filter out invalid distances

      Serial.printf("Sensor %d: Distance = %d cm\n", (int)(*pins)[0].id, distance);
      //fill your message struct here if needed
      sensor_data.sensor_id= (*pins)[i].id; //sensor_id is the second element of the pair
      sensor_data.distance = (j > 0) ? (distance / j) : 0;
      //sensor_data.distance = (int)distance/3; // average the distance over 3 readings
      Serial.printf("Sensor ID %d: Distance = %d cm\n", sensor_data.sensor_id, sensor_data.distance);
      xQueueSend(msg_queue, &sensor_data, portMAX_DELAY);
    }
    //vTaskDelete(NULL); //delete task after readings are taken 
    vTaskDelay(pdMS_TO_TICKS(300)); // Delay to avoid busy-waiting, adjust as needed
  }
}

void transmit_data(void *vparams) {
  sensor_data s;
  
  while (true) {
    if (xQueueReceive(msg_queue, &s, portMAX_DELAY)) {
      s.start_byte = 0xA5;
      Serial.write((uint8_t*)(&s), sizeof(sensor_data));
      Serial.write('\n');
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Adjust delay as needed
  }
}

// ----- Loop -----
void loop() {

}