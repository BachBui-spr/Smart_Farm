#include "TaskHandle.h"

Modbus_RS485 modbus;
QueueHandle_t gReadingQueue = nullptr;
SemaphoreHandle_t gSerialMutex = nullptr;


static void printHex(const uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();
}


bool Modbus_readHoldingRegs(uint8_t slaveId,
                     uint16_t startAddr,
                     uint16_t quantity,
                     uint16_t* out_regs,
                     int* o_err)
{
  if (o_err) *o_err = 0;

  uint8_t reqData[4];
  reqData[0] = (startAddr >> 8) & 0xFF;
  reqData[1] = (startAddr >> 0) & 0xFF;
  reqData[2] = (quantity  >> 8) & 0xFF;
  reqData[3] = (quantity  >> 0) & 0xFF;

  const uint16_t expectedLen = 1 + 1 + 1 + (quantity * 2) + 2; 
  uint8_t rxBuf[64];
  if (expectedLen > sizeof(rxBuf)) {
    if (o_err) *o_err = -100;
    return false;
  }

  ErrorCode ec = modbus.RS485_masterTransmit(
      slaveId, 0x03, reqData, 4, rxBuf, expectedLen);

  if (ec != MODBUS_OK) {
    if (o_err) *o_err = -200;
    return false;
  }

  if (rxBuf[1] & 0x80) {
    if (o_err) *o_err = 0x100 | rxBuf[2];
    return false;
  }

  if (rxBuf[0] != slaveId) { if (o_err) *o_err = -201; return false; }
  if (rxBuf[1] != 0x03)    { if (o_err) *o_err = -202; return false; }
  if (rxBuf[2] != quantity * 2) {
    if (o_err) *o_err = -203;
    Serial.print("Raw frame: "); printHex(rxBuf, expectedLen);
    return false;
  }

  for (uint16_t i = 0; i < quantity; i++) {
    uint8_t hi = rxBuf[3 + i * 2];
    uint8_t lo = rxBuf[3 + i * 2 + 1];
    out_regs[i] = ((uint16_t)hi << 8) | lo;
  }

  return true;
}

void ModbusTaskHandle(void* arg) {
  modbus.begin(MODBUS_BAUD);

  const TickType_t period = pdMS_TO_TICKS(1000);
  TickType_t lastWake = xTaskGetTickCount();

  while(1){
    Reading_t r{};
    r.tick_ms = millis();

    uint16_t regs[SHTC3_REG_COUNT] = {0};
    int err = 0;
    bool ok = Modbus_readHoldingRegs(MODBUS_SLAVE_ID, SHTC3_REG_START,
                              SHTC3_REG_COUNT, regs, &err);
    
    r.ok       = ok;
    r.err      = ok ? 0 : err;
    r.raw_temp = regs[0];
    r.raw_humi = regs[1];
    r.temp_c   = regs[0] / SCALE_DIVISOR;
    r.humi_pct = regs[1] / SCALE_DIVISOR;

    (void)xQueueSend(gReadingQueue, &r, pdMS_TO_TICKS(100));
    vTaskDelayUntil(&lastWake, period);
  }
}


void LoggerTaskHandle(void* arg) {
  for (;;) {
    Reading_t r{};
    if (xQueueReceive(gReadingQueue, &r, portMAX_DELAY) == pdTRUE) {
      if (gSerialMutex) xSemaphoreTake(gSerialMutex, portMAX_DELAY);

      if (r.ok) {
        Serial.printf("[t=%lu ms] Humi: %.2f %%RH | Temp: %.2f °C  (raw: 0x%04X 0x%04X)\n",
                      r.tick_ms, r.humi_pct, r.temp_c,
                      r.raw_temp, r.raw_humi);
      } else {
        if ((r.err & 0xFF00) == 0x0100) {
          uint8_t ex = (uint8_t)(r.err & 0xFF);
          Serial.printf("[t=%lu ms] Modbus EXCEPTION, code=0x%02X\n", r.tick_ms, ex);
        } else {
          Serial.printf("[t=%lu ms] Đọc Modbus thất bại, err=%d\n", r.tick_ms, r.err);
        }
      }

      if (gSerialMutex) xSemaphoreGive(gSerialMutex);
    }
  }
}


void ModbusTask_setup() {
  delay(200);
  Serial.println("\n===== SHTC3 RS485 Modbus (FreeRTOS) =====");

  gReadingQueue = xQueueCreate(8, sizeof(Reading_t));
  gSerialMutex  = xSemaphoreCreateMutex();

  xTaskCreate(ModbusTaskHandle, "ModbusTask", MODBUS_TASK_STACK,
              nullptr, MODBUS_TASK_PRIO, nullptr);

  xTaskCreate(LoggerTaskHandle, "LoggerTask", LOGGER_TASK_STACK,
              nullptr, LOGGER_TASK_PRIO, nullptr);
              vTaskDelay(1000 / portTICK_PERIOD_MS);
  }