#include <Arduino.h>
#include <ModbusRtu.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// assign the Arduino pin that must be connected to RE-DE RS485 transceiver
#define TXEN	2 
const int buttonPin = 7; // the number of the pushbutton pin
const int ledPinR = 13; // the number of the LED pin
const int ledPinY = 12; // the number of the LED pin
const int ledPinG = 11; // the number of the LED pin
const int forward_pwm = 5; // the number of the FORWARD PWM pin
const int backward_pwm = 6; // the number of the BACKWARD PWM pin
const byte Encoder_C1=3; // the number of the Encoder C1 pin
const byte Encoder_C2=4; // the number of the Encoder C2 pin
byte Encoder_C1Last;
uint16_t elapsedTime;
boolean direction;
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
6 : LEDR (BOOL) (DO) (OUTPUT)
7 : LEDY (BOOL) (DO) (OUTPUT)
8 : LEDG (BOOL) (DO) (OUTPUT)
9 : FORWARD OR REVERSE MOTOR (BOOL) (AO) (OUTPUT)
10 : PWM MOTOR (INT) (AO) (OUTPUT)




*/
/*uint16_t au16data[16] = {
  3, 1415, 9265, 4, 2, 7182, 28182, 8, 0, 0, 0, 0, 0, 0, 1, -1};*/

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus slave(1,0,TXEN); // this is slave @1 and RS-485

unsigned long previousMillis = 0;
unsigned long interval = 2000;

void setup() {  
  Serial.begin( 9600 ); // baud-rate at 19200
  
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
  slave.poll( data, 11 ); // poll registers from slave
  if(digitalRead(buttonPin)){
    data[4] = 1;
  }
  else{
    data[4] = 0;
  }
  if(currentMillis - previousMillis > interval)
  {
  
  float myTemp = dht.readTemperature();
  float myHumidity = dht.readHumidity();
  
  
  //if variables are numbers, then they can be saved to the array
  if(myTemp >= 0 && myTemp < 51)
  {
    data[0] = myTemp*100;
    data[1] = 0; //good data
  }else
  {
    data[1] = 1; //temperature status error
  }
  if(myHumidity >= 0 && myHumidity < 101)
  {
    data[2] = myHumidity;
    data[3] = 0; //good data
  }else{
    data[3] = 1; //humidity status error
  }
  
  Serial.print("Temperature: ");
  Serial.println(data[0]);
  Serial.print("Temperature status: ");
  
  if(data[1] == 1)
  {
    Serial.println("BAD");
  }
  else{
    Serial.println("GOOD");
  }
  Serial.print("Humidity: ");
  Serial.println(data[2]);
  Serial.print("Humidity status: ");

  if(data[3] == 1)
  {
    Serial.println("BAD");
  }
  else{
    Serial.println("GOOD");
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
  data[10] = elapsedTime;
  previousMillis = currentMillis;
  }

  if(data[6] == 1)
  {
    digitalWrite(ledPinG, HIGH);
  }
  else{
    digitalWrite(ledPinG, LOW);
  }
  //data[0] = myTemp;
  //data[1] = myHumidity;
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
  attachInterrupt(0, calculapulso, CHANGE);
}

void calculapulso()
{
  int Lstate = digitalRead(Encoder_C1);
  if ((Encoder_C1Last == LOW) && Lstate == HIGH)
  {
    int val = digitalRead(Encoder_C2);
    if (val == LOW && direction)
    {
      direction = false; //Reverse
    }
    else if (val == HIGH && !direction)
    {
      direction = true;  //Forward
    }
  }
  Encoder_C1Last = Lstate;

  if (!direction)  elapsedTime++;
  else  elapsedTime--;
}