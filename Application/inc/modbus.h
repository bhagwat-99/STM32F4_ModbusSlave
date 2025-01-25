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
#define FC_WSHR	0x06 // write single holding register
#define FC_WMHR	0x10 // write multiple holding register

// exceptio code
#define EC_IllegalDataVal 0x03



struct ModbusReceiver_t {
	uint8_t SlaveID;
	uint8_t IsStartBitRcvd;
	uint8_t IsFrameTimedOut;
	uint16_t InterFrameTimeOut; // In us; Will use 150 us for 115200 baud rate.
	uint8_t RxFrame[255];
	uint16_t RxFrameLength;
	uint8_t TxFrame[255];
	uint16_t TxFrameLength;
	uint8_t ReceivedByte;

};

#define MAX_BUF_SIZE	256
typedef struct {
	uint8_t MB_RxBuf[MAX_BUF_SIZE]; // circular buffer to store received byte in uart interrupt;
	uint16_t BufInIndex;
	uint16_t BufOutIndex;

}MB_RxBuf_t;


// Modbus Related Function
uint8_t InitHoldingRegisters();
int8_t SetHoldingRegister(uint16_t RegAdd, uint16_t RegValue);
uint8_t Handle_FC_WSHR();

void MB_InitReceiver();
void MB_ReceiverISR(void);
void MB_ProcessFrame(void);

void MB_SendMessage(uint8_t * pTxData, uint8_t length);

//Utility Functions
uint16_t CRC16(uint8_t *data, uint8_t length);

#endif
