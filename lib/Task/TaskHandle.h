#ifndef TASK_HANDLE_H
#define TASK_HANDLE_H
#pragma once

#include <Arduino.h>
#include "RS485.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"


#define MODBUS_SLAVE_ID   1
#define SHTC3_REG_START   0x0000
#define SHTC3_REG_COUNT   2
#define MODBUS_BAUD       9600
#define SCALE_DIVISOR     10.0f


#define MODBUS_TASK_STACK    4096
#define MODBUS_TASK_PRIO     1
#define LOGGER_TASK_STACK    4096
#define LOGGER_TASK_PRIO     2


typedef struct {
  uint32_t tick_ms;
  bool     ok;
  uint16_t raw_temp;
  uint16_t raw_humi;
  float    temp_c;
  float    humi_pct;
  int      err;   
} Reading_t;


extern Modbus_RS485 modbus;
extern QueueHandle_t gReadingQueue;
extern SemaphoreHandle_t gSerialMutex;

void ModbusTask_setup();

extern void ModbusTaskHandle(void* arg);
extern void LoggerTaskHandle(void* arg);
bool Modbus_readHoldingRegs(uint8_t slaveId, uint16_t startAddr,
                     uint16_t quantity, uint16_t* out_regs,
                     int* o_err);


#endif 
