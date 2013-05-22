#include <Wire.h>
#include <Time.h>
#include <SD.h>
#include "RTClib.h"

#define LOG_INTERVAL 5000 // mills between entries (reduce to take more/faster data)
#define SYNC_INTERVAL 10*LOG_INTERVAL // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL 1 // echo data to serial port
#define WAIT_TO_START 0 // Wait for serial input in setup()

int BMP085_ADDRESS = 0x77;
int tmp102Address = 0x48;
int hih4030Pin = A0; 
int temt6000Pin = A1;

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

const unsigned char OSS = 0;  
const int chipSelect = 10;

RTC_DS1307 RTC; // define the Real Time Clock object

int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
long b5; 

File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}


void setup(void)
{
  Serial.begin(9600);
  Wire.begin();
  bmp085Calibration();
  Serial.println();
  
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break; // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif //ECHO_TO_SERIAL
  }
  

  logfile.println("millis,stamp,datetime,pressure,atm,inHg,altitude,celsius,fahrenheit,relativeHumidity,lightIntensity");
#if ECHO_TO_SERIAL
  Serial.println("millis,stamp,datetime,pressure,atm,inHg,altitude,celsius,fahrenheit,relativeHumidity,lightIntensity");
#endif //ECHO_TO_SERIAL
 
}


void loop(void)
{
  DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  digitalWrite(greenLEDpin, HIGH);
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m); // milliseconds since start
  logfile.print(", ");
#if ECHO_TO_SERIAL
  Serial.print(m); // milliseconds since start
  Serial.print(", ");
#endif

  // fetch the time
  now = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
#endif //ECHO_TO_SERIAL
  
  
  //BMP085 Sensor //
  float temperature1 = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pressure = bmp085GetPressure(bmp085ReadUP());
  float atm = pressure / 101325; // "standard atmosphere"
  float inHg = float((atm)*29.92);
  float altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters 

  logfile.print(", ");
  logfile.print(pressure);
  logfile.print(", ");
  logfile.print(atm);
  logfile.print(", "); 
  logfile.print(inHg);
  logfile.print(", ");
  logfile.print(altitude);
  
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(pressure);
  Serial.print(", ");
  Serial.print(atm);
  Serial.print(", ");
  Serial.print(inHg);
  Serial.print(", ");
  Serial.print(altitude);
#endif
  
 //TMP102 Sensor//
  float celsius = getTemperature();
  float fahrenheit = (1.8 * celsius) + 32;  
  
  logfile.print(", ");
  logfile.print(celsius);
  logfile.print(", ");
  logfile.print(fahrenheit);
  
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(celsius);
  Serial.print(", ");
  Serial.print(fahrenheit);
#endif
  
  //HIH-4030 Sensor//
  float temperature = 0x48; 
  float relativeHumidity  = getHumidity(temperature);
  
  logfile.print(", ");
  logfile.print(relativeHumidity);
  
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(relativeHumidity);
#endif
  
  //TEMT6000 Sensor//
  int value = analogRead(temt6000Pin);

  logfile.print(", ");
  logfile.print(value);
  
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(value);
#endif

  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  
}

//BMP085 Sensor//

void bmp085Calibration(){
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}


float bmp085GetTemperature(unsigned int ut){
  long x1, x2;
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;
  float temp = ((b5 + 8)>>4);
  temp = temp /10;
  return temp;
}


long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  long temp = p;
  return temp;
}


char bmp085Read(unsigned char address){
  unsigned char data;
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available());
  return Wire.read();
}


int bmp085ReadInt(unsigned char address){
  unsigned char msb, lsb;
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2);
  msb = Wire.read();
  lsb = Wire.read();
  return (int) msb<<8 | lsb;
}


unsigned int bmp085ReadUT(){
  unsigned int ut;
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  delay(5);
  ut = bmp085ReadInt(0xF6);
  return ut;
}


unsigned long bmp085ReadUP(){
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  delay(2 + (3<<OSS));
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  return up;
}

void writeRegister(int deviceAddress, byte address, byte val){
  Wire.beginTransmission(deviceAddress);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

int readRegister(int deviceAddress, byte address){
  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); 
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1);
  while(!Wire.available()){}
  v = Wire.read();
  return v;
}

float calcAltitude(float pressure){
  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;
  return C;
}

//TMP102 Sensor//

float getTemperature(){
  Wire.requestFrom(tmp102Address,2); 
  byte MSB = Wire.read();
  byte LSB = Wire.read();
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 
  float celsius = TemperatureSum*0.0625;
  return celsius;
}

//HIH-4030 Sensor//

float getHumidity(float degreesCelsius){
  float supplyVolt = 5.0;
  int HIH4030_Value = analogRead(hih4030Pin);
  float voltage = HIH4030_Value/1023. * supplyVolt;

  float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
  float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius);

  return trueRH;
}
