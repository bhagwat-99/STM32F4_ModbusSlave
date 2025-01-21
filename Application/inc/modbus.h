#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "stdio.h"
//Defines
#define MODBUS_MAX_FRAME_LENGTH	256 //The maximum Modbus RTU message length is 256 bytes.

// function code
#define FC_ReadHoldingRegister	0x03


struct ModbusQuery_t{
	uint8_t 	SlaveID;
	uint8_t 	FunctionCode;
	uint16_t 	StartAddr;
	uint16_t 	Length;
};

void TaskUartTransmit();
uint16_t CRC16(uint8_t *data, uint8_t length);
void TaskModbusCommunication();

#endif
