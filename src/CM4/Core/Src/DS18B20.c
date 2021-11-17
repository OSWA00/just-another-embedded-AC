#include "DS18B20.h"

// Use this struct to initialize and get readings
// from the sensor
sensor DS18B20_init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	// Initialize a struct sensor
    sensor s;
	// Assign port and pin
    s.port = GPIOx;
    s.pin = GPIO_Pin;
	// Return the struct
    return s;
}

void delay_us (int us)
{
	// Cycle for the amount of microseconds
    for (int i=0; i < us; i++){
		// Cycle for 1 microsecond
		// Using 400 MHz and 200 MHz: 18 for M7; 4 for M4
		// Change accordingly
        for (int j=0; j < 4; j++); 
    }
}

void set_pin_output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void set_pin_input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DS18B20_start (sensor s)
{
	uint8_t Response = 0;
	set_pin_output(s.port, s.pin);   // set the pin as output
	HAL_GPIO_WritePin (s.port, s.pin, 0);  // pull the pin low
	delay_us (480);   // delay according to datasheet

	set_pin_input(s.port, s.pin);    // set the pin as input
	delay_us (80);    // delay according to datasheet

	// if the pin is low i.e the presence pulse is detected
	if (!(HAL_GPIO_ReadPin (s.port, s.pin))) Response = 1;    
	else Response = -1;

	delay_us (400); // 480 us delay totally.

	return Response;
}

void DS18B20_write (uint8_t data, sensor s)
{
	set_pin_output(s.port, s.pin);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			set_pin_output(s.port, s.pin);  // set as output
			HAL_GPIO_WritePin (s.port, s.pin, 0);  // pull the pin LOW
			delay_us (1);  // wait for 1 us

			set_pin_input(s.port, s.pin);  // set as input
			delay_us (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			set_pin_output(s.port, s.pin); // set as output
			HAL_GPIO_WritePin (s.port, s.pin, 0);  // pull the pin LOW
			delay_us (50);  // wait for 60 us

			set_pin_input(s.port, s.pin); // set as input
		}
	}
}

uint8_t DS18B20_read (sensor s)
{
	uint8_t value=0;

	set_pin_input(s.port, s.pin); // set as input

	for (int i=0;i<8;i++)
	{
		set_pin_output(s.port, s.pin);   // set as output

		HAL_GPIO_WritePin (s.port, s.pin, 0);  // pull the data pin LOW
		delay_us (1);  // wait for > 1us

		set_pin_input(s.port, s.pin);  // set as input
		if (HAL_GPIO_ReadPin (s.port, s.pin))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay_us (50);  // wait for 60 us
	}
	return value;
}

float read_temp(sensor s)
{
    uint8_t Temp_byte1, Temp_byte2;
    uint16_t TEMP;

    float Temperature = 0;
    uint8_t Presence = 0;

    Presence = DS18B20_start (s);
    HAL_Delay (1);
    DS18B20_write (0xCC, s);  // skip ROM
    DS18B20_write (0x44, s);  // convert t
    HAL_Delay (800);

    Presence = DS18B20_start (s);
    HAL_Delay(1);
    DS18B20_write (0xCC, s);  // skip ROM
    DS18B20_write (0xBE, s);  // Read Scratch-pad

	// The sensor gives 9 bytes of data
	// The read only gives 8 at a time
    Temp_byte1 = DS18B20_read(s);
    Temp_byte2 = DS18B20_read(s);
	// Get the whole reading together
    TEMP = (Temp_byte2<<8)|Temp_byte1;
	// Convert it to a float
    Temperature = (float)TEMP/16;

    return Temperature;
}
