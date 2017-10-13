/*  
 *  Sigfox Code Example
 *  
 *  Copyright (C) 2016 Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com 
 *  
 *  Modified by DP ITB 2017
 */

#include <Wire.h>

// Cooking API libraries
#include <arduinoUART.h>
#include <arduinoUtils.h>

// Sigfox library
#include <arduinoSigfox.h>

//////////////////////////////////////////////
uint8_t socket = SOCKET0;     //Asign to UART0
//////////////////////////////////////////////

unsigned long pausetime = 60000*5;  // pause in ms
unsigned long previousMillis = 0; 

// define variable to create a speficic frame to be sent
uint8_t data[12];
uint8_t size;

// define error variable
uint8_t error;

// define error pin
const int error_led =  13;

// define analogs pins
const int temperature_pin = A0;
const int light_pin = A1;


// define digitals pins
const int DIGITAL1 = 2;
const int DIGITAL2 = 3;
const int DIGITAL3 = 4;
const int DIGITAL4 = 5;
const int DIGITAL5 = 6;
const int DIGITAL6 = 7;
const int DIGITAL7 = 8;
const int DIGITAL8 = 9;

// define vars for sensors
float temperature;
uint8_t light;
uint8_t digitalPins;
uint8_t digital1;
uint8_t digital2;
uint8_t digital3;
uint8_t digital4;
uint8_t digital5;
uint8_t digital6;
uint8_t digital7;
uint8_t digital8;

// define union-type variables
union
{
  uint8_t  value1[4];
  float    value2;
}temp_union;


void setup() 
{
  //Initialize pins
  pinMode(error_led, OUTPUT);
  for (int i=2; i==9; i++) pinMode(i, INPUT);

  //////////////////////////////////////////////
  // 1. switch on              
  //////////////////////////////////////////////  
  error = Sigfox.ON(socket);
  
  // Check sending status
  if( error == 0 ) 
  {
    //"Switch ON OK"     
    digitalWrite(error_led, LOW);
  }
  else 
  {
    //"Switch ON ERROR" 
    digitalWrite(error_led, HIGH);
  } 
}


void loop() 
{
  unsigned long currentMillis = millis();
  //Wait "pausetime" in minutes to send data array
  if (currentMillis - previousMillis >= pausetime) 
  {
    // save the last time in send data
    previousMillis = currentMillis;
    
    //////////////////////////////////////////////
    // 2. create array with sensor data
    // Sigfox Back-END device "Display type" must be: 
    // Temp::float:32 Battery::uint:8 Digital8::bool:7 Digital7::bool:6
    //       Digital6::bool:5 Digital5::bool:4 Digital4::bool:3
    //       Digital3::bool:2 Digital2::bool:1 Digital1::bool:0
    //////////////////////////////////////////////
  
    // 2. Reading sensors:
    // 2.1. simulate temperature raw reading.
    temperature = (float)analogRead(temperature_pin);  
   
    // 2.2. simulate battery level raw reading
    light = (uint8_t)analogRead(light_pin);  
 
    // 2.3. Digital pins reading
    digital1 = digitalRead(DIGITAL1);
    digital2 = digitalRead(DIGITAL2);
    digital3 = digitalRead(DIGITAL3);
    digital4 = digitalRead(DIGITAL4);
    digital5 = digitalRead(DIGITAL5);
    digital6 = digitalRead(DIGITAL6);
    digital7 = digitalRead(DIGITAL7);
    digital8 = digitalRead(DIGITAL8);
  
    //Digital Pins (bitmap):
    digitalPins |= digital1;
    digitalPins |= digital2 << 1;
    digitalPins |= digital3 << 2;
    digitalPins |= digital4 << 3;
    digitalPins |= digital5 << 4;
    digitalPins |= digital6 << 5;
    digitalPins |= digital7 << 6;
    digitalPins |= digital8 << 7;  
  
    // 2.4. Fill structure fields
    temp_union.value2 = ((temperature*0.00488)-0.5)/0.01;  
  
    // fill 'data' buffer with data
    data[0] = temp_union.value1[3]; // send scaled temperature value big-endian
    data[1] = temp_union.value1[2];
    data[2] = temp_union.value1[1];
    data[3] = temp_union.value1[0];
    data[4] = analogRead(temperature_pin); //send raw temperature
    data[5] = light;
    data[6] = digitalPins;
    size = 7;
         
    //Final Frame to send in "data"
  
    //////////////////////////////////////////////
    // 3. send data
    //////////////////////////////////////////////
   
    //3. Sending packet:
    error = Sigfox.send(data,size);
  
    // Check sending status
    if( error == 0 ) 
    {
      //"Sigfox packet sent OK" flash led
      digitalWrite(error_led, HIGH);
      delay(100);
      digitalWrite(error_led, LOW);     
    }
    else 
    {
      //"Sigfox packet sent ERROR" 
      digitalWrite(error_led, HIGH);
    } 
  }
  //////////////////////////////////////////////
  // 4. sleep pause_time (minutes)
  //////////////////////////////////////////////
}
