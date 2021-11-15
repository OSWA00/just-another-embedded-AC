#include "stm32h7xx_hal.h"

typedef struct Sensor 
{
    GPIO_TypeDef *port;
    uint16_t pin;
} sensor;

sensor DS18B20_init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void delay_us (int us);

void set_pin_output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void set_pin_input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

uint8_t DS18B20_start (sensor s);

void DS18B20_write (uint8_t data, sensor s);

uint8_t DS18B20_read (sensor s);

float read_temp(sensor s);