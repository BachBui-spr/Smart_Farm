// #include <Arduino.h>
// #include "TaskHandle.h"

// void setup() {
//   Serial.begin(9600);
//   ModbusTask_setup();
// }

// void loop() {
// }


#include<Arduino.h>
#include "RS485.h"          // Thư viện RS485 của bạn (class Modbus_RS485)

#define MODBUS_SHT3C_SLAVE_ID   2        // Địa chỉ slave của module SHTC3-485
#define MODBUS_SHT3C_REG_START   0x0000   // Địa chỉ thanh ghi bắt đầu (Temp)
#define MODBUS_SHT3C_REG_COUNT   2        // Đọc 2 regs: Temp, Humi

#define SCALE_DIVISOR     10.0

#define MODBUS_LIGHT_SENSOR_ID   1        // Địa chỉ slave của module LightSensor-485
#define MODBUS_LIGHT_SENSOR_REG_START   0x0003   // Địa chỉ thanh ghi bắt đầu (Light Intensity)
#define MODBUS_LIGHT_SENSOR_REG_COUNT   1        // Đọc 2 regs: Light Intensity
#define MODBUS_BAUD       4800

#define MODBUS_RAIN_SENSOR_ID   3        // Địa chỉ slave của module WaterSensor-485
#define MODBUS_RAIN_SENSOR_REG_START   0x0000   // Địa chỉ thanh ghi bắt đầu (Water Level)
#define MODBUS_RAIN_SENSOR_REG_COUNT   1        // Đọc 4 regs: Water Level


Modbus_RS485 modbus;

// In mảng bytes ở dạng hex cho debug
static void printHex(const uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();
}

bool readHoldingRegs(uint8_t slaveID, uint16_t startArr, uint16_t quantity, uint16_t* out){
    uint8_t reqData[4];
    reqData[0] = (startArr >> 8) & 0xFF;
    reqData[1] = (startArr >> 0) & 0XFF;
    reqData[2] = (quantity >> 8) & 0xFF;
    reqData[3] = (quantity >> 0) & 0xFF;

    const uint16_t expectedLen = 1 + 1 + 1 + (quantity * 2) + 2;
    uint8_t rxBuf[64];
    if(expectedLen > sizeof(rxBuf)){
        Serial.println("[ERR] expectedLen quá lớn cho rxBuf");
        return false;
    }
    ErrorCode ec = modbus.RS485_masterTransmit(
        slaveID,
        0x03,
        reqData, 4,
        rxBuf, expectedLen
    );
    if(ec != MODBUS_OK){
        Serial.printf("[ERR] RS485_masterTransmit thất bại: %d\n", (int)ec);
        return false;
    }

    if(rxBuf[0] != slaveID || rxBuf[1] != 3){
        Serial.println("[ERR] Phản hồi không hợp lệ (ID/FC sai)");
        printHex(rxBuf, expectedLen);
        return false;
    }

    if(rxBuf[2] != quantity * 2){
        Serial.println("[ERR] Phản hồi không hợp lệ (Byte count sai)");
        printHex(rxBuf, expectedLen);
        return false;
    }

    if(!modbus.checkCrc(rxBuf, expectedLen)){
        Serial.println("[ERR] CRC phản hồi không hợp lệ");
        printHex(rxBuf, expectedLen);
        return false;
    }
    if(slaveID == MODBUS_LIGHT_SENSOR_ID){
    for(int i = 0, j = 3; i < quantity; i++, j += 2){
        out[i] = (rxBuf[j] << 8) | rxBuf[j + 1];
    }   
    } else if(slaveID == MODBUS_SHT3C_SLAVE_ID){
    for (uint16_t i = 0; i < quantity; i++) {
    uint8_t hi = rxBuf[3 + i * 2];
    uint8_t lo = rxBuf[3 + i * 2 + 1];
    out[i] = (uint16_t)hi << 8 | lo;
    }
    } else if(slaveID == MODBUS_RAIN_SENSOR_ID){
    out[0] = rxBuf[3];  
    }
    return true;
}

void setup() {
  Serial.begin(4800);
  delay(200);

  Serial.println("===== SHTC3 RS485 Modbus test (FC=03) =====");

  modbus.begin(MODBUS_BAUD);
  Serial.println("Modbus RS485 Light Sensor started.");
}


void loop() {
  uint16_t modbus_light_regs[MODBUS_LIGHT_SENSOR_REG_COUNT] = {0};

  if(readHoldingRegs(MODBUS_LIGHT_SENSOR_ID, MODBUS_LIGHT_SENSOR_REG_START, MODBUS_LIGHT_SENSOR_REG_COUNT, modbus_light_regs)){
      Serial.printf("Light Intensity: %u lux\n", modbus_light_regs[0]);
  } else {
      Serial.println("Failed to read Light Sensor");
  }

  uint16_t modbus_sht3c_regs[MODBUS_SHT3C_REG_COUNT] = {0};
  if (readHoldingRegs(MODBUS_SHT3C_SLAVE_ID, MODBUS_SHT3C_REG_START, MODBUS_SHT3C_REG_COUNT, modbus_sht3c_regs)) {
    float temperature = modbus_sht3c_regs[0] / SCALE_DIVISOR;
    float humidity    = modbus_sht3c_regs[1] / SCALE_DIVISOR;

    Serial.printf("Humi: %.2f %% RH | Temp: %.2f °C (raw: 0x%04X 0x%04X)\n",
                  temperature, humidity, modbus_sht3c_regs[0], modbus_sht3c_regs[1]);
  } else {
    Serial.println("[WARN] Đọc Modbus thất bại");
  }
  
  uint16_t modbus_rain_regs[MODBUS_RAIN_SENSOR_REG_COUNT] = {0};
  if(readHoldingRegs(MODBUS_RAIN_SENSOR_ID, MODBUS_RAIN_SENSOR_REG_START, MODBUS_RAIN_SENSOR_REG_COUNT, modbus_rain_regs)){
      //Serial.printf("Water Level: %u mm\n", modbus_rain_regs[0]);
      if(modbus_rain_regs[0] == 0){
        Serial.println("No Rain");
      } else {
        Serial.println("Rain");
      }
  } else {
      Serial.println("Failed to read Rain Sensor");
  }

  delay(2000);
}