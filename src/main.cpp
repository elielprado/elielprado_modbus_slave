#include <Arduino.h>
#include <ModbusRtu.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <math.h>

// assign the Arduino pin that must be connected to RE-DE RS485 transceiver
#define TXEN	2 
#define pulsos_por_volta 550
const int buttonPin = 7; // the number of the pushbutton pin
const int ledPinR = 13; // the number of the LED pin
const int ledPinY = 12; // the number of the LED pin
const int ledPinG = 11; // the number of the LED pin
const int forward_pwm = 5; // the number of the FORWARD PWM pin
const int backward_pwm = 6; // the number of the BACKWARD PWM pin
const byte Encoder_C1=3; // the number of the Encoder C1 pin
const byte Encoder_C2=4; // the number of the Encoder C2 pin
byte Encoder_C1Last;
int pulso = 0;
boolean volta = 0;
uint16_t elapsedTime;
boolean direction;

volatile unsigned long rpm = 0;
volatile unsigned long timePulso = 0;
volatile unsigned long timeCountOld = 0;
volatile bool valorOld = 0;

// assign pins to KY-15 module
#define DHTPIN 8
#define DHTTYPE DHT11

// variable for pwm led
int pwm_value = 0;


//Define our DHT object
DHT dht(DHTPIN, DHTTYPE);


// data array for modbus network sharing
uint16_t data[20];


/*
Data Variable Map:
Pos (16 bit each) : Meaning
//READ ONLY
0 : Temperature (int) (AI)
1 : Temperature Sensor Status (BOOL) (DO)
2 : Humidity (int) (AI)
3 : Humidity Sensor Status (DO) (OUTPUT)
4 : Push Button (BOOL) (DI) (INPUT)
5 : RPM (int) (AO)
//WRITE ONLY
6 : LEDG (BOOL) (DO) (OUTPUT)
7 : LEDY (BOOL) (DO) (OUTPUT)
8 : LEDR (BOOL) (DO) (OUTPUT)
9 : FORWARD OR REVERSE MOTOR (BOOL) (AO) (OUTPUT)
10 : PWM MOTOR (INT) (AO) (OUTPUT)
11 : Cooler RPM (int) (AI) (INPUT)

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */

Modbus slave(1,2,TXEN); // this is slave @1 and RS-485

unsigned long previousMillis = 0;
unsigned long interval = 2000;
void EncoderInit();
void analogReadFunc();

void setup() {  
  Serial.begin( 9600 ); // baud-rate max tested 115200
  
  slave.start();
  dht.begin();
  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinY, OUTPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(forward_pwm, OUTPUT);
  pinMode(backward_pwm, OUTPUT);
  EncoderInit();
}

void loop() {
  unsigned long currentMillis = millis();
  slave.poll( data, 12 ); // poll registers from slave
  
  if(digitalRead(buttonPin)){ // if button is pressed
    data[4] = 1;
  }
  else{
    data[4] = 0;
  }

  if(currentMillis - previousMillis > interval) //this will run every 2 seconds (refresh speed recommended for the temperature sensor)
  {
  float myTemp = dht.readTemperature();
  float myHumidity = dht.readHumidity();
  
  
  if(myTemp >= 0 && myTemp <= 50)
  {
    data[0] = myTemp*100;
    data[1] = 0; //good data
  }else
  {
    data[1] = 1; //temperature status error
  }
  if(myHumidity >= 0 && myHumidity <=100)
  {
    data[2] = myHumidity;
    data[3] = 0; //good data
  }else{
    data[3] = 1; //humidity status error
  } 


  previousMillis = currentMillis;
  }
    
  pwm_value = data[10];
  if(data[9]==0) //Set the rotation direction of the motor
  {
  //PWM MOTOR
   analogWrite(backward_pwm,0);
   analogWrite(forward_pwm,pwm_value);
  }else{
  //PWM MOTOR
   analogWrite(forward_pwm,0);
   analogWrite(backward_pwm,pwm_value);    
  }



  if(data[6] == 1)
  {
    digitalWrite(ledPinG, HIGH);
  }
  else{
    digitalWrite(ledPinG, LOW);
  }

  if(data[7] == 1)
  {
    digitalWrite(ledPinY, HIGH);
  }
  else{
    digitalWrite(ledPinY, LOW);
  }
  if(data[8] == 1)
  {
    digitalWrite(ledPinR, HIGH);
  }
  else{
    digitalWrite(ledPinR, LOW);
  }

}

void EncoderInit()
{
  pinMode(Encoder_C2, INPUT);
  attachInterrupt(0, analogReadFunc, CHANGE);
}

void analogReadFunc() // Faz a leitura do sinal Analogico
{   
      if(pwm_value!=0){
        timePulso= pulseIn(Encoder_C2, HIGH);
        rpm = (unsigned long)(60000000/(pulsos_por_volta*timePulso));
        data[5] = rpm;
      }else {
        timePulso = 0;
        rpm=0;
        data[5] = rpm;
      }
}