#include "Timer.h"
#include "main.h"
#include "modbus.h"

typedef uint32_t TimerMs_t;
extern struct ModbusReceiver_t MB_Receiver;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

TimerMs_t TimerRedLed = 0;
void TaskControlRedLed(){
	if((TimerMs_t)(GetTickCounter() - TimerRedLed) < 2000) return;

	//Toggle the red led
	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);

	TimerRedLed = GetTickCounter();
}



volatile uint32_t u32TickCounter = 0;

uint32_t GetTickCounter(){

	return u32TickCounter;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM7){ // 1ms time for regular counter

		u32TickCounter++;

	}
	if(htim->Instance == TIM6){ // 150 us inter-frame timeout for modbus rtu
		HAL_TIM_Base_Stop_IT(&htim6); // stop timer for processing
		MB_Receiver.IsFrameTimedOut = 1; // if pre-emted by uart interrupt we not want to over write receive buffer by new frame without processing this frame
		MB_ProcessFrame();
		MB_Receiver.IsFrameTimedOut = 0;



	}

}
