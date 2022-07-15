#include <Arduino.h>
#include <ModbusRtu.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// assign the Arduino pin that must be connected to RE-DE RS485 transceiver
#define TXEN	4 
const int buttonPin = 7; // the number of the pushbutton pin
const int ledPin = 13; // the number of the LED pin

// assign pins to KY-15 module
#define DHTPIN 2
#define DHTTYPE DHT11

//Define our DHT object
DHT dht(DHTPIN, DHTTYPE);


// data array for modbus network sharing
uint16_t data[16];


/*
Data Variable Map:
Pos (16 bit each) : Meaning
0 : Temperature (int) (AI)
1 : Temperature Sensor Status (BOOL) (DO)
2 : Humidity (int) (AI)
3 : Humidity Sensor Status (DO) (OUTPUT)
4 : Push Button (BOOL) (DI) (INPUT)
5 : LED (BOOL) (DO) (OUTPUT)




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
Modbus slave(1,2,TXEN); // this is slave @1 and RS-485

unsigned long previousMillis = 0;
unsigned long interval = 5000;

void setup() {
  Serial.begin( 9600 ); // baud-rate at 19200
  
  slave.start();
  dht.begin();
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);

}

void loop() {
  unsigned long currentMillis = millis();
  slave.poll( data, 6 );
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

  previousMillis = currentMillis;
  }

  if(data[5] == 1)
  {
    digitalWrite(ledPin, HIGH);
  }
  else{
    digitalWrite(ledPin, LOW);
  }
  //data[0] = myTemp;
  //data[1] = myHumidity;
  

}