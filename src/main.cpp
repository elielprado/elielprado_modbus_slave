#include <Arduino.h>
#include <ModbusRtu.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// assign the Arduino pin that must be connected to RE-DE RS485 transceiver
#define TXEN	4 

// assign pins to KY-15 module
#define DHTPIN 2
#define DHTTYPE DHT11

//Define our DHT object
DHT dht(DHTPIN, DHTTYPE);


// data array for modbus network sharing
uint16_t data[16];
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
}

void loop() {
  unsigned long currentMillis = millis();
  slave.poll( data, 16 );
  if(currentMillis - previousMillis > interval)
  {
  
  float myTemp = dht.readTemperature();
  float myHumidity = dht.readHumidity();
  
  
  //if variables are numbers, then they can be saved to the array
  if(myTemp >= 0 && myTemp < 51)
  {
    data[0] = myTemp*100;
  }else
  {
    data[9] = 1; //temperature status error
  }
  if(myHumidity >= 0 && myHumidity < 101)
  {
    data[8] = myHumidity;
  }else{
    data[10] = 1; //humidity status error
  }
  
  Serial.print("Temperature: ");
  Serial.println(data[0]);
  Serial.print("Temperature status: ");
  
  if(data[9] == 1)
  {
    Serial.println("BAD");
  }
  else{
    Serial.println("GOOD");
  }
  Serial.print("Humidity: ");
  Serial.println(data[8]);
  Serial.print("Humidity status: ");

  if(data[10] == 1)
  {
    Serial.println("BAD");
  }
  else{
    Serial.println("GOOD");
  }
  previousMillis = currentMillis;
  }
  //data[0] = myTemp;
  //data[1] = myHumidity;
  

}