#include "modbus.h"
#include "Timer.h"
#include "main.h"
#include "string.h"

typedef uint32_t TimerMs_t;

extern UART_HandleTypeDef huart2;

char txData[30] = "Hello from stm32\r\n";


uint8_t MB_TxFrame[MODBUS_MAX_FRAME_LENGTH];
uint8_t MB_RxFrame[MODBUS_MAX_FRAME_LENGTH];
uint8_t MB_TxFrameLength;
uint8_t MB_RxFrameLength;


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
		MB_TxFrame[1] = FC_ReadHoldingRegister; // function code
		MB_TxFrame[2] = 0x00; // address 40108 -> 0x006b
		MB_TxFrame[3] = 0x6b;
		MB_TxFrame[4] = 0x00; // read three registers -> 0x0003
		MB_TxFrame[5] = 0x03;

		CRC16(MB_TxFrame, 6);
		MB_TxFrame[6] = 0x76; // crc -> 0xb994 // lsb first
		MB_TxFrame[7] = 0x87;

		MB_TxFrameLength = 7;

}

void MB_SendMessage(){
	HAL_UART_Transmit(&huart2, MB_TxFrame, MB_TxFrameLength+1, 10);


}

TimerMs_t TimerModbusTask = 0;
void TaskModbusCommunication(void){
	if((TimerMs_t)(GetTickCounter() - TimerModbusTask) < 10000) return; //Task frequency 50ms

	// data received in circular buffer?
	while(sRxBuf.BufInIndex != sRxBuf.BufOutIndex){ // There is data to process
		HAL_UART_Transmit(&huart2, sRxBuf.MB_RxBuf + sRxBuf.BufOutIndex , 1, 10);
		if(sRxBuf.BufOutIndex >= MAX_BUF_SIZE - 1){
			sRxBuf.BufOutIndex = 0;
		}
		else{
			sRxBuf.BufOutIndex++;
		}

	}
//	MB_PrepareMessage();
//	MB_SendMessage();

	TimerModbusTask = GetTickCounter();


}


