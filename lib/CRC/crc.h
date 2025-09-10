/* 




*/
#ifndef INC_CRC_H
#define INC_CRC_H

#include <stdio.h>
#include <stdint.h>

extern uint16_t crc16_lookup_table[256];

uint16_t crc16(unsigned char *data, unsigned char num_bytes);

#endif