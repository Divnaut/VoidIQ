// shared_defs.h
#pragma once
#include <cstdint>

#define DEFAULT_CHANNEL 1
#define MSG_TYPE_DATA 1
#define MSG_TYPE_CHANNEL 2
#define uS_TO_S_FACTOR 1000000ULL
#pragma pack(push, 1)
struct sensor_data {
  uint8_t start_byte=0xA5;
  uint8_t node_id=0;
  uint8_t sensor_id;
  uint16_t distance;
};
#pragma pack(pop)

struct SensorPin {
  int trig;
  int echo;
  int id;
};

static const int MAX_READINGS = 10;


struct msg {
  int type = MSG_TYPE_DATA;
  int num_data = 0;
  sensor_data readings[MAX_READINGS];

  bool ready() const { return num_data >= 5; }
  bool full() const { return num_data >= MAX_READINGS; }
  void add(const sensor_data &d) { readings[num_data++] = d; }
  void reset() { num_data = 0; }
};