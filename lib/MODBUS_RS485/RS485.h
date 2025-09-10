#ifndef RS485
#define RS485

#include <stdlib.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#include "common.h"
#include "crc.h"

#define MODBUS_TX 5
#define MODBUS_RX 19
#define EN_PIN    18

#define MODBUS_TIMEOUT      1500
#define MODBUS_SLAVE_ID     0x01
#define MODBUS_BUFFER_SIZE  256

// extern HardwareSerial RS485_Com;

enum ModbusState{
    SLAVE_STATE_IDLE,     
    SLAVE_STATE_RECEIVE,  
    SLAVE_STATE_RESPONSE, 
    SLAVE_STATE_CHECK,    
    SLAVE_STATE_PROCESS,  
    SLAVE_STATE_WAIT     
};

class Modbus_RS485 {
    public:
    void begin (uint32_t baud_rate = 9600);
    
    bool isLineBusy ();
    bool checkCrc(uint8_t *buf, uint16_t len);
    
    ErrorCode RS485_transmit (const uint8_t * buffer, uint16_t size);
    ErrorCode RS485_transmitReceive (const uint8_t * buffer, uint16_t txSize, uint8_t * rxBuffer, uint16_t rxSize);
    
    uint8_t RS485_receive (uint8_t * buffer, uint16_t size);
    
    ErrorCode RS485_masterTransmit (uint8_t slave_id, uint8_t function_code, uint8_t * data, uint16_t data_length, uint8_t * rxBuffer, uint16_t rxSize);

    void modbus_slave_fsm(uint8_t *buf, uint16_t len);
};

#endif