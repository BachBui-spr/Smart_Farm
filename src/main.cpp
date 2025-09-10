// main.cpp
#include <Arduino.h>
#include "RS485.h"          // Thư viện RS485 của bạn (class Modbus_RS485)

#define MODBUS_SLAVE_ID   1        // Địa chỉ slave của module SHTC3-485
#define SHTC3_REG_START   0x0000   // Địa chỉ thanh ghi bắt đầu (Temp)
#define SHTC3_REG_COUNT   2        // Đọc 2 regs: Temp, Humi
#define MODBUS_BAUD       9600      

#define SCALE_DIVISOR     10.0

Modbus_RS485 modbus;  // Đối tượng từ thư viện của bạn

// In mảng bytes ở dạng hex cho debug
static void printHex(const uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();
}

// Đọc N holding registers, trả kết quả vào out[] (big-endian -> uint16)
bool readHoldingRegs(uint8_t slaveId, uint16_t startAddr, uint16_t quantity, uint16_t* out) {
  // Payload cho FC=03: [StartHi][StartLo][QtyHi][QtyLo]
  uint8_t reqData[4];
  reqData[0] = (startAddr >> 8) & 0xFF;
  reqData[1] = (startAddr >> 0) & 0xFF;
  reqData[2] = (quantity  >> 8) & 0xFF;
  reqData[3] = (quantity  >> 0) & 0xFF;

  // Phản hồi FC=03: [ID][FC][ByteCount][Data...][CRC_L][CRC_H]
  const uint16_t expectedLen = 1 + 1 + 1 + (quantity * 2) + 2; // = 5 + 2N
  uint8_t rxBuf[64];
  if (expectedLen > sizeof(rxBuf)) {
    Serial.println("[ERR] expectedLen quá lớn cho rxBuf");
    return false;
  }

  ErrorCode ec = modbus.RS485_masterTransmit(
    slaveId,        // slave id
    0x03,           // Function code: Read Holding Registers
    reqData, 4,     // payload 4 byte
    rxBuf, expectedLen
  );

  if (ec != MODBUS_OK) {
    Serial.printf("[ERR] RS485_masterTransmit thất bại: %d\n", (int)ec);
    return false;
  }

  // Kiểm tra khung trả về cơ bản
  if (rxBuf[0] != slaveId) {
    Serial.printf("[ERR] Sai SlaveID: expect %u got %u\n", slaveId, rxBuf[0]);
    return false;
  }
  if (rxBuf[1] != 0x03) {
    Serial.printf("[ERR] Sai FunctionCode: expect 0x03 got 0x%02X\n", rxBuf[1]);
    return false;
  }
  uint8_t byteCount = rxBuf[2];
  if (byteCount != quantity * 2) {
    Serial.printf("[ERR] Sai ByteCount: expect %u got %u\n", quantity * 2, byteCount);
    Serial.print("Raw frame: "); printHex(rxBuf, expectedLen);
    return false;
  }

  // Ghép 2 byte big-endian -> uint16
  for (uint16_t i = 0; i < quantity; i++) {
    uint8_t hi = rxBuf[3 + i * 2];
    uint8_t lo = rxBuf[3 + i * 2 + 1];
    out[i] = (uint16_t)hi << 8 | lo;
  }

  return true;
}

void setup() {
  Serial.begin(9600);
  delay(200);

  Serial.println("===== SHTC3 RS485 Modbus test (FC=03) =====");
  // Khởi tạo thư viện RS485 của bạn (bên trong sẽ set EN_PIN, SoftwareSerial v.v.)
  modbus.begin(MODBUS_BAUD);
  Serial.println("RS485 begin OK");
}

void loop() {
  uint16_t regs[SHTC3_REG_COUNT] = {0};

  if (readHoldingRegs(MODBUS_SLAVE_ID, SHTC3_REG_START, SHTC3_REG_COUNT, regs)) {
    // Giải mã giá trị (tùy module: *10 hoặc *100)
    float temperature = regs[0] / SCALE_DIVISOR;
    float humidity    = regs[1] / SCALE_DIVISOR;

    Serial.printf("Humi: %.2f %% RH | Temp: %.2f °C (raw: 0x%04X 0x%04X)\n",
                  temperature, humidity, regs[0], regs[1]);
  } else {
    Serial.println("[WARN] Đọc Modbus thất bại");
  }

  delay(2000);
}
