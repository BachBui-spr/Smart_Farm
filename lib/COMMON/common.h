#ifndef COMMON_H
#define COMMON_H
#include <Arduino.h>

#define MAX_INIT_TIMEOUT  210000U

enum ErrorCode
{
  MODBUS_OK,
  MODBUS_BUSY  = 1,
  MODBUS_ERROR = 2,
  MODBUS_IDLE  = 3,
  CRC_INVALID  = 1001,
  ERROR_RETRY  = 1002
};

enum SystemStatus
{
  SYSTEM_OK,
  SYSTEM_ERROR,
  SYSTEM_BUSY,
  SYSTEM_IDLE,
  SYSTEM_INVALID,
  SYSTEM_OVERFLOW
};

typedef struct {
    String id;
    double lat; 
    double lng;
} Stations;

typedef struct
{
  String   command;
  String   bus_number;
  String   state = "wait";
  int      route;
  int      direction;
  String   next_station;
  float    longtitude, latitude, altitude, speed, course;
  float    temperature, humidity;
  String   station_id;
} data_frame;

typedef struct
{
  String    apn;
  String    gprs_user;
  String    gprs_pass;
  String    broker;
  uint16_t  port;
  String    client_id;
  String    username;
  String    password;
  String    topic_pub;
  String    topic_sub;
} module_sim_startup_data;

typedef struct 
{
  char      publish_topic [256];
  char      payload[256];
} module_sim_publish_data;

typedef struct
{
  int       direction;
  String    station;
  String    state;
  String    bus_number;

  float     latitude;
  float     longtitude;
} response_frame;

#ifdef DEBUG_SMARTBUS
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(x) \
        do                 \
        {                  \
        } while (0)
    #define DEBUG_PRINTLN(x) \
        do                   \
        {                    \
        } while (0)
    #define DEBUG_PRINTF(...) \
        do                    \
        {                     \
        } while (0)
#endif // SMARTBUS_DEBUG

#endif  // COMMON_H