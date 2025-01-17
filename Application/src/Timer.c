#include "Timer.h"
#include "main.h"

volatile uint32_t u32TickCounter = 0;

uint32_t GetTickCounter(){

	return u32TickCounter;
}

typedef uint32_t TimerMs_t;


TimerMs_t TimerRedLed = 0;

void TaskControlRedLed(){
	if((TimerMs_t)(GetTickCounter() - TimerRedLed) < 2000) return;

	//TODO
	//Toggle the red led
	HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);

	TimerRedLed = GetTickCounter();
}
