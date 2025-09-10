#include "RS485.h"

static SoftwareSerial   RS485_Com(MODBUS_RX, MODBUS_TX);
static uint32_t         last_byte_time = 0;

static ModbusState modbus_state = SLAVE_STATE_IDLE;
static uint8_t modbus_rx_buffer[MODBUS_BUFFER_SIZE] = {0};
static uint8_t modbus_tx_buffer[MODBUS_BUFFER_SIZE] = {0};

void Modbus_RS485::begin (uint32_t baud_rate) {
    // Define pin modes for TX and RX
    pinMode (EN_PIN, OUTPUT);

    RS485_Com.begin (baud_rate); // RX, TX pins for RS485
    digitalWrite (EN_PIN, LOW); // Set RS485 to receive mode
}

bool Modbus_RS485::isLineBusy () {
    // This function can be implemented to check if the RS485 line is busy
    // For now, it returns false as a placeholder
    uint32_t startTime = millis();
    while (millis() - startTime < 600) { // Wait for 100 ms
        if (!RS485_Com.available()) {
            return false; // Line is busy if data is available
        }
    }
    return true;
}

bool Modbus_RS485::checkCrc(uint8_t *buf, uint16_t len) {
    // return crc16(buf, len - 2) == ((buf [len - 1] << 8) | buf [len - 1]); 
    uint16_t received_crc = buf[len - 2] | (buf[len - 1] << 8);  // LSB first
    uint16_t calculated_crc = crc16(buf, len - 2);
    return calculated_crc == received_crc;
}

ErrorCode Modbus_RS485::RS485_transmit (const uint8_t * buffer, uint16_t size) {
    uint8_t retry = 0;
    while (retry < 3) {
        uint32_t startTime = millis();
        if (isLineBusy ()) {
            DEBUG_PRINTLN ("RS485 line is busy, cannot send data.\n");
            return MODBUS_BUSY;
        }

        DEBUG_PRINTF("RS485 Transmit: %s\n", buffer);
        digitalWrite (EN_PIN, HIGH); // Set RS485 to transmit mode
        RS485_Com.write ((buffer), size);
        RS485_Com.flush (); // Ensure all data is sent
        digitalWrite (EN_PIN, LOW); // Set RS485 back to receive mode
  
        while (millis () - startTime < 400) { // Wait for 100 ms
            uint8_t * data_frame = new uint8_t[size]; 
            if (RS485_receive(data_frame, size) == MODBUS_OK) {
                if (checkCrc(data_frame, size)) {
                    delete[] data_frame; // Free allocated memory
                    return MODBUS_OK; // Data received successfully
                } 
                DEBUG_PRINTLN ("CRC check failed.\n");
            }
            delete [] data_frame;
        }
        vTaskDelay(random(200, 400));
        retry++;
    }
    return MODBUS_ERROR; // Error occurred
}

ErrorCode Modbus_RS485::RS485_transmitReceive (const uint8_t * buffer, uint16_t txSize, uint8_t * rxBuffer, uint16_t rxSize) {
    if (buffer == nullptr || rxBuffer == nullptr || txSize == 0 || rxSize == 0) {
        return MODBUS_ERROR; // Invalid parameters
    }

    uint8_t retry = 0;
    while (retry < 3) {
        uint32_t startTime = millis();

        // Check on busy line
        if (isLineBusy ()) {
            DEBUG_PRINTLN ("RS485 line is busy, cannot send data.\n");
            return MODBUS_BUSY;
        }

        digitalWrite (EN_PIN, HIGH); // Set RS485 to transmit mode
        RS485_Com.write (buffer, txSize);
        RS485_Com.flush (); // Ensure all data is sent
        digitalWrite (EN_PIN, LOW); // Set RS485 back to receive mode
        
        while (millis () - startTime < 400) { // Wait for 400 ms
            if (RS485_receive(rxBuffer, rxSize) == MODBUS_OK) {
                if (checkCrc(rxBuffer, rxSize)) {
                    return MODBUS_OK; // Data received successfully
                } 
                DEBUG_PRINTLN ("CRC check failed.\n");
            }
        }
        retry++;
    }
    return MODBUS_ERROR; // Error occurred
}

uint8_t Modbus_RS485::RS485_receive(uint8_t *buffer, uint16_t size) {
    if (buffer == nullptr || size == 0) return MODBUS_ERROR;

    uint16_t received = 0;
    uint32_t last_byte_time = millis();

    // Check till all bytes are received or timeout occurs
    while (received < size) {
        if (RS485_Com.available()) {
            buffer[received++] = RS485_Com.read();
            last_byte_time = millis(); // Reset last byte time
        } else if (millis() - last_byte_time > MODBUS_TIMEOUT) {
            return MODBUS_IDLE; // Timout occurred
        }
    }
    return MODBUS_OK;
}

ErrorCode Modbus_RS485::RS485_masterTransmit (uint8_t slave_id, uint8_t function_code, uint8_t * data, uint16_t data_length, uint8_t * rxBuffer, uint16_t rxSize) {
    ErrorCode error_code = MODBUS_OK;
    if (data_length > MODBUS_BUFFER_SIZE - 4) {
        return MODBUS_ERROR;
    }

    uint8_t * tx_buffer = (uint8_t *) malloc(sizeof(uint8_t) * (data_length + 4));
    tx_buffer[0] = slave_id;
    tx_buffer[1] = function_code;
    
    for (uint16_t i = 0; i < data_length; i++) {
        tx_buffer[i + 2] = data[i];
    }
    uint16_t crc = crc16(tx_buffer, data_length + 2);
    tx_buffer[data_length + 2] = crc & 0xFF; // LSB
    tx_buffer[data_length + 3] = (crc >> 8) & 0xFF; // MSB
    if (rxBuffer) {
        error_code = RS485_transmitReceive(tx_buffer, data_length + 4, rxBuffer, rxSize);
    } else {
        error_code = RS485_transmit(tx_buffer, data_length + 4);
    }
    free (tx_buffer);
    return error_code; // Transmission successful
}


void Modbus_RS485::modbus_slave_fsm(uint8_t *buf, uint16_t len){
    static uint16_t index = 0;
    static uint32_t lastReceiveTime = 0;

    switch (modbus_state)
    {
    case SLAVE_STATE_IDLE: {
        if(RS485_Com.available()){
            index = 0;
            lastReceiveTime = millis();
            modbus_rx_buffer[index++] = RS485_Com.read();
            modbus_state = SLAVE_STATE_RECEIVE;
        }
        break;}

    case SLAVE_STATE_RECEIVE: {

        while(RS485_Com.available()){
            modbus_rx_buffer[index++] = RS485_Com.read();
            lastReceiveTime = millis();
            if(index >= MODBUS_BUFFER_SIZE){
                modbus_state = SLAVE_STATE_CHECK;
                break;
            }
        }
        if(millis() - lastReceiveTime > 5){
            modbus_state = SLAVE_STATE_CHECK;
        }
        break;
    }
    case SLAVE_STATE_CHECK: {
         if (index < 4) {
                modbus_state = SLAVE_STATE_IDLE;  // quá ngắn
                break;
            }
            if (!checkCrc(modbus_rx_buffer, index)) {
                RS485_Com.write("CRC sai\n");
                modbus_state = SLAVE_STATE_IDLE;
                break;
            }

            modbus_state = SLAVE_STATE_PROCESS;
        
        break;
    }
    case SLAVE_STATE_PROCESS:
    {
        uint8_t slave_id = modbus_rx_buffer[0];
        if (slave_id != MODBUS_SLAVE_ID) {
            modbus_state = SLAVE_STATE_IDLE;
            break;
        }

        uint8_t func_code = modbus_rx_buffer[1];
        if (func_code == 0x03) {
           
            modbus_tx_buffer[0] = slave_id;
            modbus_tx_buffer[1] = func_code;
            modbus_tx_buffer[2] = 4; // số byte data (2 regs)
            modbus_tx_buffer[3] = 0x00; // data high
            modbus_tx_buffer[4] = 0x10; // data low
            modbus_tx_buffer[5] = 0x00;
            modbus_tx_buffer[6] = 0x20;

            uint16_t crc = crc16(modbus_tx_buffer, 7);
            modbus_tx_buffer[7] = crc & 0xFF;
            modbus_tx_buffer[8] = (crc >> 8) & 0xFF;

            modbus_state = SLAVE_STATE_RESPONSE;
        } else {
            RS485_Com.write("Unsupported function code\n");
            modbus_state = SLAVE_STATE_IDLE;
        }

        break;
    }
        
    case SLAVE_STATE_RESPONSE:
        digitalWrite(EN_PIN, HIGH);
        RS485_Com.write(modbus_tx_buffer, 9);
        RS485_Com.flush();
        digitalWrite(EN_PIN, LOW);
        modbus_state = SLAVE_STATE_WAIT;
        break;
    case SLAVE_STATE_WAIT:
        delay(5);
        modbus_state = SLAVE_STATE_IDLE;
        break;            
    default:
        break;
    }
}