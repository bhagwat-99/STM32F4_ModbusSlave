#include "modbus.h"
#include "Timer.h"
#include "main.h"
#include "string.h"

typedef uint32_t TimerMs_t;

extern UART_HandleTypeDef huart2;

char txData[30] = "Hello from stm32\r\n";


uint8_t MB_TxFrame[MODBUS_MAX_FRAME_LENGTH];
uint8_t MB_RxFrame[MODBUS_MAX_FRAME_LENGTH];
uint8_t MB_TxFrameLength = 0;
uint8_t MB_RxFrameLength = 0;

uint16_t HoldingRegisters[MAX_HOLDING_REGISTERS] = {0};

MB_RxBuf_t sRxBuf;


//TimerMs_t TimerUartTrasmit = 0;
//void TaskUartTransmit(void){
//	if((TimerMs_t)(GetTickCounter() - TimerUartTrasmit) < 3000) return;
//	//execute every 3 sec
//
//	//Send the string on uart
//	HAL_UART_Transmit(&huart2, (uint8_t *)txData, strlen(txData), 10);
//
//	TimerUartTrasmit = GetTickCounter();
//
//}


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

void MB_PrepareMessage(){

		MB_TxFrame[0] = 0x11; // decimal 17 : slave ID of device
		MB_TxFrame[1] = FC_RHR; // function code
		MB_TxFrame[2] = 0x00; // address 40108 -> 0x006b
		MB_TxFrame[3] = 0x6b;
		MB_TxFrame[4] = 0x00; // read three registers -> 0x0003
		MB_TxFrame[5] = 0x03;

		CRC16(MB_TxFrame, 6);
		MB_TxFrame[6] = 0x76; // crc -> 0xb994 // lsb first
		MB_TxFrame[7] = 0x87;

		MB_TxFrameLength = 7;

}

void Uart_SendMessage(uint8_t * pTxData, uint8_t length){
	HAL_UART_Transmit(&huart2, pTxData, length, 10);

}

uint8_t Handle_FC_RHR() {
	uint8_t retVal = 0;

	MB_TxFrame[1] = MB_RxFrame[1]; // function code

    // Ensure the request contains at least 8 bytes -> Slave ID | FC | addH | addL | LenH | Len Low | crcL | crcH
	//                                                    0     |  1 |  2   |  3   |  4   |    5    |   6  |   7
    if (MB_RxFrameLength < 8 ) {
    	retVal = 1;
    	memset(MB_RxFrame, 0, (size_t)MODBUS_MAX_FRAME_LENGTH);
        return retVal; // Invalid request length
    }

    // Extract starting address
    uint16_t start_address = ((uint16_t)MB_RxFrame[2] << 8) | MB_RxFrame[3];

    // Extract quantity of registers to read
    uint16_t NoOfRegs = ((uint16_t)MB_RxFrame[4] << 8) | MB_RxFrame[5];

    // Validate starting address and quantity
    if ((start_address + NoOfRegs > MAX_HOLDING_REGISTERS) || (NoOfRegs == 0)) {
    	MB_TxFrame[1] = MB_TxFrame[1] | 0x80;
    	MB_TxFrame[2] =  EC_IllegalDataVal;

    	// Calculate CRC for the response
		uint16_t crc = CRC16(MB_TxFrame, 3); // Calculate CRC for address, function code + 0x80, and exception code
		MB_TxFrame[3] = crc & 0xFF;                 // CRC Low byte
		MB_TxFrame[4] = (crc >> 8) & 0xFF;          // CRC High byte

		MB_TxFrameLength = 5;

    }
    else{

		MB_TxFrame[2] = NoOfRegs * 2; // Register size 16 bits -> 2 bytes

		// Copy register values to the response
		for (uint16_t i = 0; i < NoOfRegs; i++) {
			MB_TxFrame[3 + (i * 2)] = (HoldingRegisters[start_address + i] >> 8) & 0xFF; // High byte
			MB_TxFrame[4 + (i * 2)] = HoldingRegisters[start_address + i] & 0xFF;        // Low byte
		}

		// Calculate CRC for the response
		uint16_t crc = CRC16(MB_TxFrame, 3 + NoOfRegs * 2); // Calculate CRC for address, function code, and data
		MB_TxFrame[3 + NoOfRegs*2] = crc & 0xFF;                 // CRC Low byte
		MB_TxFrame[4 + NoOfRegs*2] = (crc >> 8) & 0xFF;          // CRC High byte

		// Set total response length
		MB_TxFrameLength = 3 + NoOfRegs*2 + 2; // Slave ID + Function code + Byte count + Data + CRC
    }

    Uart_SendMessage(MB_TxFrame, MB_TxFrameLength);


    return 0; // Successfully processed
}

uint8_t MB_SendResponse(){
	uint8_t retVal = 0;

	// Clear the Tx frame
	memset(MB_TxFrame, 0, (size_t)MODBUS_MAX_FRAME_LENGTH);

	MB_TxFrame[0] = SLAVE_ID;

	switch(MB_RxFrame[1]){
	case FC_RHR:
		Handle_FC_RHR();
		memset(MB_RxFrame, 0, (size_t)MODBUS_MAX_FRAME_LENGTH); // should't come here
		memset(MB_TxFrame, 0, (size_t)MODBUS_MAX_FRAME_LENGTH); // should't come here
		MB_RxFrameLength = 0;
		MB_TxFrameLength = 0;

		break;
	default:
		memset(MB_TxFrame, 0, (size_t)MODBUS_MAX_FRAME_LENGTH); // should't come here
		memset(MB_RxFrame, 0, (size_t)MODBUS_MAX_FRAME_LENGTH); // should't come here
		MB_RxFrameLength = 0;
		MB_TxFrameLength = 0;
		break;
	}



	return retVal; // reached end of function this is success
}


uint8_t MB_RxTimeout = 1;
uint8_t MB_RxFrameRcvd = 0;

uint8_t MB_VerifyFrame(){

	// check if slave id match
	if(MB_RxFrame[0] != SLAVE_ID) return 1; // slave id does not match

	uint16_t CrcRcvd = (uint16_t)MB_RxFrame[MB_RxFrameLength - 2] | (uint16_t)MB_RxFrame[MB_RxFrameLength - 1] << 8;

	uint16_t CrcCalculated = CRC16(MB_RxFrame, MB_RxFrameLength - 2);

	if(CrcRcvd != CrcCalculated) return 1;

	return 0; // success
}


TimerMs_t TimerModbusTask = 0;
void TaskModbusCommunication(void){
	if((TimerMs_t)(GetTickCounter() - TimerModbusTask) < 5) return; //Task frequency 5ms

	// data received in circular buffer?
	while(sRxBuf.BufInIndex != sRxBuf.BufOutIndex){ // There is data to process
		//HAL_UART_Transmit(&huart2, sRxBuf.MB_RxBuf + sRxBuf.BufOutIndex , 1, 10);
		MB_RxFrame[MB_RxFrameLength] = sRxBuf.MB_RxBuf[sRxBuf.BufOutIndex];


		if(MB_RxFrameLength > MODBUS_MAX_FRAME_LENGTH){ // more length more than valid length 255
			// Invalid frame; clear the frame buffer and index
			memset(MB_RxFrame, 0, (size_t)MODBUS_MAX_FRAME_LENGTH);
			MB_RxFrameLength = 0;
		}
		else{
			MB_RxFrameLength++;
		}


		if(sRxBuf.BufOutIndex >= MAX_BUF_SIZE - 1){
			sRxBuf.BufOutIndex = 0;
		}
		else{
			sRxBuf.BufOutIndex++;
		}

	}

	if((MB_RxTimeout > 0) && (MB_RxFrameLength >= MODBUS_MIN_FRAME_LENGTH)){ // Timeout happened

			MB_RxFrameRcvd = 1;
			//MB_RxTimeout = 0; // temporaririly turning it off
	}

	if(MB_RxFrameRcvd > 0){
		MB_RxFrameRcvd = 0;

		if(MB_VerifyFrame() != 0){
			// who have a problem
			memset(MB_RxFrame, 0, (size_t)MODBUS_MAX_FRAME_LENGTH);
			// handle it
		}
		else{
			MB_SendResponse();
		}
	}

//	MB_PrepareMessage();
//	MB_SendMessage();

	TimerModbusTask = GetTickCounter();


}

uint8_t SetHoldingRegister(uint16_t RegAdd, uint16_t RegValue){

	HoldingRegisters[0x006b] = 0xae41;
	HoldingRegisters[0x006c] = 0x5652;
	HoldingRegisters[0x006d] = 0x4340;


}


