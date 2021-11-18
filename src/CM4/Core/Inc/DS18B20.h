#include "stm32h7xx_hal.h" // Hal for h7 family

// Structure with the port and pin of the sensor
typedef struct Sensor 
{
    GPIO_TypeDef *port;
    uint16_t pin;
} sensor;

// Function to initiliaze the sensor
// Receives port and pin of the sensor
// Returns a struct sensor to use with the other functions
sensor DS18B20_init(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

// Function to make delays in microseconds
// Receives the amount of microseconds
void delay_us (int us);

// Function to set a pin as output
// Recives the port an pin to change
void set_pin_output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

// Function to set a pin as input
// Recives the port an pin to change
void set_pin_input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

// Function to start the sensor
// Receives a struct sensor
// Returns 1 if it detect a sensor,
// -1 if it fail
uint8_t DS18B20_start (sensor s);

// Function to write data in the sensor
// Receives the data and the sensor to write
void DS18B20_write (uint8_t data, sensor s);

// Function to read data from the sensor
// Receives the sensor to read from
// Returns the reading
uint8_t DS18B20_read (sensor s);

// Function to read the temperature in Celsius
// Receives the sensor to read from
// Returns a float with the temperature
uint16_t read_temp(sensor s);