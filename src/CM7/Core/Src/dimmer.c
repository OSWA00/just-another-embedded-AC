#include "dimmer.h"

extern UART_HandleTypeDef huart4;
static unsigned int value;

void dimmerSet(unsigned int value_)
{
	value = value_;
	HAL_UART_Transmit(&huart4, (uint8_t*)value, 1, 10);
}
