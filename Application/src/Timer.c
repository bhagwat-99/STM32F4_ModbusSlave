#include "Timer.h"
#include "main.h"


typedef uint32_t TimerMs_t;

volatile uint32_t u32TickCounter = 0;

uint32_t GetTickCounter(){

	return u32TickCounter;
}


TimerMs_t TimerRedLed = 0;

void TaskControlRedLed(){
	if((TimerMs_t)(GetTickCounter() - TimerRedLed) < 2000) return;

	//Toggle the red led
	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);

	TimerRedLed = GetTickCounter();
}


//extern volatile uint32_t u32TickCounter;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	u32TickCounter++;
}
