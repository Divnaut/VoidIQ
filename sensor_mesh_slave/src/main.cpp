#include<Arduino.h>
#include "freertos/FreeRTOS.h"
#include "WiFi.h"
#include "esp_now.h"
#include "shared_defs.h"

//#include "stdlib.h"
#include <vector>
#include <esp_wifi.h>
//this is what will get passed to the esp_now_send function
msg message;


const uint8_t transmit_address[]={0x00, 0x4B, 0x12, 0x33, 0x4B, 0x00}; //Reciever address: Towards master node
// const uint8_t BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
constexpr char WIFI_SSID[] = "div_wifi";
// Create peer interface
esp_now_peer_info_t peerInfo;
//creating task handles:
static TaskHandle_t task_take_readings = NULL; // handle for the print_string task
static TaskHandle_t task_transmit_data = NULL; // handle for the print_asterisk task
static QueueHandle_t msg_queue = NULL; // queue to hold the messages to be sent
// static QueueHandle_t channel_queue = NULL; // queue to hold the channel messages
std::vector<SensorPin> pins = {
  {33, 32, 1}
};

void transmit_data(void *vparams);
void take_readings(void *vparams);
void on_recv_callback(const uint8_t *mac_addr, const uint8_t *data, int data_len);
// int32_t get_wifi_channel(const char* ssid);
// void handle_channel_type_message(void * vparams);

//void initWiFi();
void initEspNow();
// void broadcast_channel(uint32_t ch);
void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true); // Enable promiscuous mode
  esp_wifi_set_channel(2, WIFI_SECOND_CHAN_NONE); // Set channel to default channel 2
  esp_wifi_set_promiscuous(false); // Disable promiscuous mode

  initEspNow();

  Serial.print("This device MAC");
  Serial.println(WiFi.macAddress());


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
}

// int32_t get_wifi_channel(const char* ssid){
//   if (int32_t n = WiFi.scanNetworks()) {
//     for (uint8_t i=0; i<n; i++) {
//       if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
//         return WiFi.channel(i);
//       }
//     }
//   }
//   return 0;
// }

// void broadcast_channel(uint32_t ch) {
//   msg chan_msg;
//   chan_msg.type = MSG_TYPE_CHANNEL;
//   chan_msg.readings[0].sensor_id = ch;
//   chan_msg.num_data = 1;
//   esp_now_send(BROADCAST_ADDR, (uint8_t*)&chan_msg, sizeof(chan_msg));
//   Serial.printf("[Master] Broadcasted channel: %d\n", ch);
// }

// void initWiFi() {
//   msg channel_msg;
//   WiFi.mode(WIFI_MODE_STA);
//   int32_t channel = get_wifi_channel(WIFI_SSID);
//   Serial.println("WiFi channel: " + String(channel));
//   channel_msg.type = MSG_TYPE_CHANNEL;
//   channel_msg.num_data = 1;
//   channel_msg.readings[0].sensor_id = channel; // Set the channel as sensor_id
//   // WiFi.printDiag(Serial);
//   //xQueueSend(channel_queue, &channel_msg, portMAX_DELAY);
//   esp_wifi_set_promiscuous(true);
//   esp_wifi_set_channel(2, WIFI_SECOND_CHAN_NONE);
//   esp_wifi_set_promiscuous(false);
//   broadcast_channel(channel);
//   esp_wifi_set_promiscuous(true);
//   esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
//   esp_wifi_set_promiscuous(false);

//   // WiFi.printDiag(Serial);

// }

// void handle_channel_type_message(void * vparams) {
//   while(1){
//     msg incoming_message;
//     if (xQueueReceive(channel_queue, &incoming_message, portMAX_DELAY) == pdTRUE) {
//       current_millis = millis();
//       if (current_millis - prev_millis > 5000) {
//         int new_ch = incoming_message.readings[0].sensor_id;
//         Serial.printf("Received channel sync: %d\n", new_ch);
//         esp_wifi_set_promiscuous(true);
//         esp_wifi_set_channel(new_ch, WIFI_SECOND_CHAN_NONE);
//         esp_wifi_set_promiscuous(false);
//         prev_millis = current_millis; // Update the last received time
//         // Broadcast again to help cascade
//         esp_now_send(BROADCAST_ADDR, (uint8_t*)&incoming_message, sizeof(incoming_message));
//       }
//     }
//   }  
// }

void initEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP NOW failed to initialize");
    while (1);
  }
  memcpy(peerInfo.peer_addr, transmit_address, 6);
  peerInfo.ifidx   = (wifi_interface_t)ESP_IF_WIFI_STA;
  peerInfo.encrypt = false; 
  peerInfo.channel = 0; // Use default channel, will be set to the current channel of the station

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP NOW pairing failure");
    while (1);
  }
}



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
          vTaskDelay(pdMS_TO_TICKS(250));
          //delay(300);
          continue; // skip this reading if no echo is received
        }
        vTaskDelay(pdMS_TO_TICKS(250));
        if(j >= 8) break; // exit loop after 8 valid readings per sensor
        
      }
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
    //msg message; //holds the final message that get's transmitted 
    while (1) {
        sensor_data data;
        // Wait for a message to be available in the queue
        if (xQueueReceive(msg_queue, &data, portMAX_DELAY)) {
          message.add(data);  
        }
        if (message.ready()) {
          // Send the message using esp_now_send
          esp_err_t result = esp_now_send(transmit_address, (uint8_t*)&message, sizeof(message));
          // for(int i=0; i<3; i++){
          //   esp_now_send(transmit_address, (uint8_t*)&message, sizeof(message));
          //   vTaskDelay(pdMS_TO_TICKS(300)); // Delay to avoid busy-wait
          // }
          //Serial.println("Message sent successfully");
          // Uncomment the following lines if you want to check the result of esp_now_send
          // esp_err_t = 
          if (result == ESP_OK) {
            Serial.println("Message sent successfully");
          } else {
            Serial.printf("Error sending message: %d\n", result);
          }
          message.reset(); // Reset the message after sending
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Adjust delay as needed
    }
}

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

void loop() {
  // The main loop is empty because the tasks handle everything
  //vTaskDelay(pdMS_TO_TICKS(2000));
}
