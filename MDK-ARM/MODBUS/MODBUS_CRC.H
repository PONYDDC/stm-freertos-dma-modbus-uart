#ifndef _MODBUS_CRC_H_
#define _MODBUS_CRC_H_
#include "stm32f3xx.h"
 
 
 
 
uint16_t  Modbus_CRC16( uint8_t *puchMsg, uint16_t usDataLen );
 
 
#endif

