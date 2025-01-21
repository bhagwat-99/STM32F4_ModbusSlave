#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "stdio.h"
//Defines
#define MODBUS_MAX_FRAME_LENGTH	256 //The maximum Modbus RTU message length is 256 bytes.
#define MODBUS_MIN_FRAME_LENGTH	4
#define SLAVE_ID 17
#define MAX_HOLDING_REGISTERS 200

// function code
#define FC_RHR	0x03

// exceptio code
#define EC_IllegalDataVal 0x03


struct ModbusQuery_t{
	uint8_t 	SlaveID;
	uint8_t 	FunctionCode;
	uint16_t 	StartAddr;
	uint16_t 	Length;
};

#define MAX_BUF_SIZE	256
typedef struct {
	uint8_t MB_RxBuf[MAX_BUF_SIZE]; // circular buffer to store received byte in uart interrupt;
	uint16_t BufInIndex;
	uint16_t BufOutIndex;

}MB_RxBuf_t;

void TaskUartTransmit();
uint16_t CRC16(uint8_t *data, uint8_t length);
void TaskModbusCommunication();

uint8_t SetHoldingRegister(uint16_t RegAdd, uint16_t RegValue);

#endif
