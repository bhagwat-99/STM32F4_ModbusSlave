#include "modbus.h"
#include "Timer.h"
#include "main.h"
#include "string.h"

typedef uint32_t TimerMs_t;

extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

uint16_t HoldingRegisters[MAX_HOLDING_REGISTERS] = {0};
struct ModbusReceiver_t MB_Receiver;


void MB_SendMessage(uint8_t * pTxData, uint8_t length){

	HAL_UART_Transmit(&huart2, pTxData, length, 10); // replace with respective HAL functions

}

uint8_t Handle_FC_RHR() {

	uint8_t retVal = 0;

	MB_Receiver.TxFrame[1] = MB_Receiver.RxFrame[1]; // function code

    // Extract starting address
    uint16_t start_address = ((uint16_t)MB_Receiver.RxFrame[2] << 8) | MB_Receiver.RxFrame[3];

    // Extract quantity of registers to read
    uint16_t NoOfRegs = ((uint16_t)MB_Receiver.RxFrame[4] << 8) | MB_Receiver.RxFrame[5];

    // Validate starting address and quantity
    if ((start_address + NoOfRegs > MAX_HOLDING_REGISTERS) || (NoOfRegs == 0)) {

    	MB_Receiver.TxFrame[1] = 	MB_Receiver.TxFrame[1] | 0x80;
    	MB_Receiver.TxFrame[2] =  	EC_IllegalDataVal;

    	// Calculate CRC for the response
		uint16_t crc = CRC16(MB_Receiver.TxFrame, 3); // Calculate CRC for address, function code + 0x80, and exception code
		MB_Receiver.TxFrame[3] = crc & 0xFF;                 // CRC Low byte
		MB_Receiver.TxFrame[4] = (crc >> 8) & 0xFF;          // CRC High byte

		MB_Receiver.TxFrameLength = 5;

    }
    else{

    	MB_Receiver.TxFrame[2] = NoOfRegs * 2; // Register size 16 bits -> 2 bytes

		// Copy register values to the response
		for (uint16_t i = 0; i < NoOfRegs; i++) {
			MB_Receiver.TxFrame[3 + (i * 2)] = (HoldingRegisters[start_address + i] >> 8) & 0xFF; // High byte
			MB_Receiver.TxFrame[4 + (i * 2)] = HoldingRegisters[start_address + i] & 0xFF;        // Low byte
		}

		// Calculate CRC for the response
		uint16_t crc = CRC16(MB_Receiver.TxFrame, 3 + NoOfRegs * 2); // Calculate CRC for address, function code, and data
		MB_Receiver.TxFrame[3 + NoOfRegs*2] = crc & 0xFF;                 // CRC Low byte
		MB_Receiver.TxFrame[4 + NoOfRegs*2] = (crc >> 8) & 0xFF;          // CRC High byte

		// Set total response length
		MB_Receiver.TxFrameLength = 3 + NoOfRegs*2 + 2; // Slave ID + Function code + Byte count + Data + CRC
    }

    MB_SendMessage(MB_Receiver.TxFrame, MB_Receiver.TxFrameLength);


    return retVal; // Successfully processed
}


uint8_t InitHoldingRegisters(){

	HoldingRegisters[0x006b] = 0xae41;
	HoldingRegisters[0x006c] = 0x5652;
	HoldingRegisters[0x006d] = 0x4340;

	return 0;

}

// Function code Write Holding Register
uint8_t Handle_FC_WSHR(){

	uint16_t RegAdd = (uint16_t)MB_Receiver.RxFrame[2] << 8 | MB_Receiver.RxFrame[3];

	uint16_t RegValue = (uint16_t)MB_Receiver.RxFrame[4] << 8 | MB_Receiver.RxFrame[5];

	HoldingRegisters[RegAdd] = RegValue;

	return 0;

}

void MB_InitReceiver(){

	MB_Receiver.SlaveID = 0x11; // 17 Decimal
	MB_Receiver.InterFrameTimeOut = 150; // 150 us for 115200
	MB_Receiver.IsFrameTimedOut = 0;
	MB_Receiver.IsStartBitRcvd = 0;
	MB_Receiver.RxFrameLength = 0;
	MB_Receiver.TxFrameLength = 0;
	MB_Receiver.ReceivedByte = 0;
}

void MB_ReceiverISR(void){ // Will be called in UART Receive interrupt. Inline to avoid function call in ISR.

	if(MB_Receiver.IsFrameTimedOut) return; // Frame Received and processing is going on somewhere else. Flag will be cleared once processign done

	// Have we received Start byte already
	if(MB_Receiver.IsStartBitRcvd){

		HAL_TIM_Base_Stop_IT(&htim6);
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		HAL_TIM_Base_Start_IT(&htim6); // start 150us inter frame timeout;


		if(MB_Receiver.RxFrameLength >= 256){ // crossed max frame length limit. Restart

			MB_Receiver.IsStartBitRcvd = 0;
			MB_Receiver.RxFrameLength = 0;
			return;
		}

		// Insert the new byte in frame;
		MB_Receiver.RxFrame[MB_Receiver.RxFrameLength] = MB_Receiver.ReceivedByte;
		MB_Receiver.RxFrameLength++;

	}
	else if(MB_Receiver.ReceivedByte == MB_Receiver.SlaveID){

		MB_Receiver.RxFrame[MB_Receiver.RxFrameLength] = MB_Receiver.ReceivedByte;
		MB_Receiver.IsStartBitRcvd = 1;
		MB_Receiver.RxFrameLength++;
	}

}

void MB_ProcessFrame(void){ // This will be triggered from Timer interrupt

	// Check minimum frame length
    // Ensure the request contains at least 8 bytes
	//Slave ID | FC | addH | addL | LenH | Len Low | crcL | crcH
	//   0     |  1 |  2   |  3   |  4   |    5    |   6  |   7

	if(MB_Receiver.RxFrameLength < 8){

		//TODO Handle the exception
		MB_Receiver.RxFrameLength = 0;
		MB_Receiver.InterFrameTimeOut = 0;
		return;
	}

	uint16_t CrcRcvd = (uint16_t)MB_Receiver.RxFrame[MB_Receiver.RxFrameLength - 2] | (uint16_t)MB_Receiver.RxFrame[MB_Receiver.RxFrameLength - 1] << 8;

	uint16_t CrcCalculated = CRC16(MB_Receiver.RxFrame, MB_Receiver.RxFrameLength - 2);

	if(CrcRcvd != CrcCalculated){

		// crc error
		//TODO Handle the exception
		MB_Receiver.RxFrameLength = 0;
		MB_Receiver.InterFrameTimeOut = 0;
		return;
	}

	MB_Receiver.TxFrame[0] = MB_Receiver.RxFrame[0];

	switch(MB_Receiver.RxFrame[1]){

		case FC_RHR:
			Handle_FC_RHR();
			break;

		case FC_WSHR:
			Handle_FC_WSHR();
			break;

		default:
			break;
	}

	MB_Receiver.TxFrameLength = 0;
	MB_Receiver.RxFrameLength = 0;
	MB_Receiver.InterFrameTimeOut = 0; // ready to receive next frame

}

// Function to calculate CRC-16
uint16_t CRC16(uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
