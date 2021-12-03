// Temperature plant PID control program
// Owen Jauregui - A01638122 
// Oswaldo Hernandez - A01274570
// Luis Sánchez - A01638029
// Diego Limón - A01638247
                     
// Libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>

/***********************************************************************/
// Pins for sensor
#define temperaturePin  9

// Pins for actuators
#define fan_1Pin  10
#define fan_2Pin  11

/***********************************************************************/
// Variables para FreeRTOS
TaskHandle_t  xTemp;
TaskHandle_t  xPID;
TaskHandle_t  xFans;
TaskHandle_t  xDimmer;
TaskHandle_t  xLCD;
TaskHandle_t  xADC;

QueueHandle_t qFans;
QueueHandle_t qDimmer;

SemaphoreHandle_t mutexTemp;
SemaphoreHandle_t mutexRef;

/***********************************************************************/
// Control variables
int   state = 0;
float t     = 0;
float temp  = 0;
float reference = 0;

// Instancia a las clases OneWire, DallasTemperature y LiquidCrystal
OneWire oneWireObjeto(temperaturePin);
DallasTemperature sensorDS18B20(&oneWireObjeto);
LiquidCrystal_I2C lcd(0x27,16, 2);

/***********************************************************************/

//Period
float T = 0.6;

//PID Controller parameters
float Kp = 50;
float Ki = 1;
float Kd = 20;
float Ti = Kp/Ki;
float Td = Kd/Kp;

// Constants
float q0 = Kp + Kp*T/Ti + Kp*Td/T;
float q1 = -Kp - 2*Kp*Td/T;
float q2 = Kp*Td/T;

// Error signal inicialization
float e = 0, e1 = 0, e2 = 0;

// Control inizialization
float u = 0, u1 = 0;

/***********************************************************************/

// Air pressure sensor reading

void xReadTemp(){
  float temp_;
  for(;;){
    sensorDS18B20.requestTemperatures();
    temp_ = sensorDS18B20.getTempCByIndex(0);
    if (xSemaphoreTake(mutexTemp, 10) == pdTRUE){
      temp = temp_;
      xSemaphoreGive(mutexTemp);
    }
    vTaskDelay(600/portTICK_PERIOD_MS);
  }
}

/***********************************************************************/

void xControl()
{
  float  reference_, temp_;
  for(;;){   
        int8_t fans, dim;
       
        t += T;
        if (xSemaphoreTake(mutexTemp, 10) == pdTRUE){
          temp_ = temp;
          xSemaphoreGive(mutexTemp);
        }
        if (xSemaphoreTake(mutexRef, 10) == pdTRUE){
          reference_ = reference;
          xSemaphoreGive(mutexRef);
        }
 
        e = reference_ - temp_;
  
      
        u = u1 + q0*e + q1*e1 + q2*e2;
      
        if(u < -10) {
          u = -10;
        } else if (u > 90) {
          u = 90;
        }
      
        dim  = 20;
        fans =  0;
      
        if (u < 0){
          fans = 3;
        }
        else if (u < 10){
          fans = 2;
        }
        else if (u > 20) {
          dim = (uint8_t)u;
        }
        xQueueSend(qFans,  &fans, pdFALSE);
        xQueueSend(qDimmer, &dim, pdFALSE);
        
        u1 = u;
        e2 = e1;
        e1 = e;
        Serial.println(u);
        vTaskDelay(600/portTICK_PERIOD_MS);
  }
}

/**********************************************************************/

void xTurn_Fans() {
    int8_t fan_conf;
    for(;;){
      if(xQueueReceive(qFans, &fan_conf, 10) == pdPASS) {
          digitalWrite(fan_1Pin, (fan_conf & 0x01));
          digitalWrite(fan_2Pin, (fan_conf & 0x02));
      }
      vTaskDelay(1/portTICK_PERIOD_MS);
    }
}

/**********************************************************************/

void xSend_Dimmer() {
    int8_t level;
    for(;;){
      if(xQueueReceive(qDimmer, &level, 10) == pdPASS) {
          Serial1.write(level);
      }
    }
    vTaskDelay(1/portTICK_PERIOD_MS);
}

/**********************************************************************/

void xRead_Pot() {
    for(;;){
      if (xSemaphoreTake(mutexRef, 10) == pdTRUE){
        reference = analogRead(0) / 34.1333 + 20;
        xSemaphoreGive(mutexRef);
      }
      vTaskDelay(400/portTICK_PERIOD_MS);
    }
    
}

/**********************************************************************/

void xWrite_LCD() {
  float temp_, reference_;
    for(;;){
      if (xSemaphoreTake(mutexTemp, 10)== pdTRUE) {
        temp_ = temp;
        xSemaphoreGive(mutexTemp);
      }
      if (xSemaphoreTake(mutexRef, 10)== pdTRUE) {
        reference_ = reference;
        xSemaphoreGive(mutexRef);
      }
        lcd.clear();
        lcd.print("Temp: ");
        lcd.print(temp_, 2);
        lcd.setCursor(0, 1);
        lcd.print("Ref:  ");
        lcd.print(reference_);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

/***********************************************************************/

void setup() {
  // Serial monitor for testing
  Serial.begin(9600);
  Serial1.begin(115200,SERIAL_8E1);
  
  // Iniciamos el bus 1-Wire
  sensorDS18B20.begin(); 

  // Iniciamos la LCD

  lcd.init();
  lcd.backlight();
  
  // Pin modes
  pinMode(fan_1Pin, OUTPUT);
  pinMode(fan_2Pin, OUTPUT);
  digitalWrite(fan_1Pin, 0);
  digitalWrite(fan_2Pin, 0);

  qFans   = xQueueCreate(3, sizeof(int8_t));
  qDimmer = xQueueCreate(3, sizeof(int8_t));
  
  mutexTemp = xSemaphoreCreateMutex();
  mutexRef  = xSemaphoreCreateMutex();
  
   xTaskCreate( xReadTemp,
                "Read temperature",
                1024,
                NULL,
                10,
                &xTemp);
   xTaskCreate( xControl,
                "PID function",
                1024,
                NULL,
                7,
                &xPID);
   xTaskCreate( xTurn_Fans,
                "Fans Function",
                1024,
                NULL,
                3,
                &xFans);

   xTaskCreate( xSend_Dimmer,
                "Dimmer Function",
                1024,
                NULL,
                3,
                &xDimmer);

   xTaskCreate( xRead_Pot,
                "ADC Funtion",
                1024,
                NULL,
                5,
                &xADC);
                
   xTaskCreate( xWrite_LCD,
                "LCD Function",
                1024,
                NULL,
                1,
                &xLCD);
        
  vTaskStartScheduler();          
}

/***********************************************************************/

void loop(){}
