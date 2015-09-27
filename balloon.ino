/*  This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License,

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include <SoftwareSerial.h>   //Serial emulator for GPS
#include <TinyGPS++.h>          //To talk to GPS system
#include <Wire.h>             //To talk to MPU gyros
#include <Adafruit_BMP085.h> //For BMP temp/press sensor
#include <string.h>
#include <util/crc16.h>      //For NTX crc check
#include <math.h>
#include "DHT.h"

Adafruit_BMP085 bmp;        //Calleh BMP085 functions
TinyGPSPlus gps;                //Call GPS functions
//AltSoftSerial ss(12, 13);
SoftwareSerial ss(12, 13); //Initiate software serial bus for GPS. Pin 8 is TX, 9 is RX.
DHT dht(8,DHT22);


float realalt;
float temp;
float AmbPress;
long seq = 1;
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AccelX,AccelY,AcelcZ,Tmp,GyX,GyY,GyZ;

//NTX stuff. Pin below will be used to send TTY TX to NTX
#define RADIOPIN 10
char datastring[150]; //string will contain data stream
String data;

 //FLIGHT INFORMATION

 String CallSign = "$$BLAFSEN1"; // $$ required
 char GPSTime[32];
// String GPSTime = "00:00:00";
 double GPSLat; //= 10.565656;
 double GPSLong; //= -82.202020;
 String strLat;
 String strLong;
 float GPSAlt; //= //320;
 int GPSSpeed; //= 22;
 int GPSTrack; //= //356;
 float IntTemp; //= //000;
 float exTemp;
 int Hum;
 float BaroAlt;
 float AccelZ;
 String comma = ",";
 String dcomma = ":";
 String Sys;

 float zG = 1.00;
 float zRot = 0;


// Pin definitions
int intPin = 12;  // This can be changed, 2 and 3 are the Arduinos ext int pins

bool act = 0;

//Simple error detection and reporting part
/* 
 *  0: General sys status
 *  1: BMP sensor issues
 *  2: MPU platform issues
 *  3: GPS issues;
 *  4: DHT/Ext sensor grid issues;
 *  5: NTX issues;
 *  6: OVHT detect issues - RESERVED for future
 *  7: Electrical issues - low voltage etc . RESERVED
 *  8: Nice to know info msgs
 *  
 */
String Err[10];

void setup() {
   dht.begin();   
     
//  Waking up MPU gyros     
 Wire.begin();
 Wire.beginTransmission(MPU);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission(true);
  //Initiate serial bus for debugging
  Serial.begin(9600);

  //Setting up board for NTX radio
  pinMode(11, OUTPUT);    //Set pin 11 to OUT, to enable NTX thru Ven
  digitalWrite(11, HIGH); //Open pin 11 to enable radio (3.3V)
  pinMode(RADIOPIN,OUTPUT); //Set RADIOPIN to OUT to send TX data
  setPwmFrequency(RADIOPIN, 1);
}


void loop() {
  seq = seq + 1; //Number sequencer to comply with Habitat telemetry tx string
  bmp_retrieve(103200);
  delay(1000);
  mpu_retrieve();
  delay(1000);
  gps_retrieve();
  delay(1000);
  dhtRead();
  //readAna(0); - ENABLE WHEN EXTERNAL ANALOG TEMP SENSOR IS CONNECTED readAna(PIN);
  ErrCheck();
  send_data();
  delay(3000);
  }

void send_data() {


// $$CALLSIGN,sentence_id,time,latitude,longitude,altitude,speed,course,internal temperature, ext temp, hum, baro,alt, ambpress, z-g, z-rotation*CHECKSUM\n 

   //change coords in desired format
   strLat = String(GPSLat, 6);
   strLong = String(GPSLong, 6);
   //set up datastring
   data = CallSign + comma +  seq + comma + GPSTime + comma + strLat + comma + strLong + comma + GPSAlt + comma + GPSSpeed + comma + GPSTrack + comma + IntTemp + comma + exTemp + comma + Hum + comma + BaroAlt + comma + AmbPress + comma + zRot + comma + Sys;
   Serial.println(data); //DEBUG print data to serial
   snprintf(datastring,150, data.c_str()); // Puts the text in the datastring
   unsigned int CHECKSUM = gps_CRC16_checksum(datastring); // Calculates the checksum for this datastring
   char checksum_str[6];
   sprintf(checksum_str, "*%04X\n", CHECKSUM);
   strcat(datastring,checksum_str);
   rtty_txstring (datastring);
   data = ""; //Erase data string for new round.
  }

void bmp_retrieve(long QNH) {
   if (!bmp.begin()) {
   // Serial.println("ERROR: BMP085 not found"); //Only for debugging
    Err[1] = "BMPFAIL";
  }
  BaroAlt = bmp.readAltitude(QNH);
  IntTemp = bmp.readTemperature();
  AmbPress = bmp.readPressure();
 }


void mpu_retrieve() {
  float AcX,AcY,AcZ,Tmp,GyX,GyY;
  Wire.beginTransmission(MPU);
  Wire.write(0x39);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  zG = AcZ;
  zRot = GyZ / 100;
  //IntTemp=Wire.read()<<8|Wire.read();
 }


void gps_retrieve() {

  ss.begin(9600); //SoftSerial for GPS
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  unsigned long fix_age;
  String dP = ":";

 //Parsing location info
 if (gps.location.isValid())
  {
    GPSLat = gps.location.lat();
    GPSLong = gps.location.lng();
  }
    
  //Parsing movement info
  if (gps.speed.isValid()) GPSSpeed = gps.speed.kmph();
  if (gps.course.isValid()) GPSTrack = gps.course.value();
  if (gps.altitude.isValid()) GPSAlt = gps.altitude.meters();
 
  //Parsing date info
  if (gps.date.isValid()) 
  {
    //char GPSTime[32];
    sprintf(GPSTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    //Serial.print(sz);

    
  //  int GPSTimeH = (gps.time.hour()); 
  //  int GPSTimeM = (gps.time.minute());
  //  int GPSTimeS = (gps.time.second());
      
  //  if (GPSTimeH < 10 || GPSTimeM < 10 || GPSTimeS < 10)
  //  {
  //    GPSTimeH = printf("%02d", GPSTimeH); 
  //    GPSTimeM = printf("%02d", GPSTimeM);
   //   GPSTimeS = printf("%02d", GPSTimeS);    
   // }
  //  if(GPSTimeH == -1 || isnan(GPSTimeH)) GPSTime = "12:00:00";
  //  GPSTime = GPSTimeH + dcomma + GPSTimeM + dcomma + GPSTimeS;
  } 

 smartDelay(1000);

if (millis() > 5000 && gps.charsProcessed() < 10) Err[3] = "3FAIL";
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis(); //GPS reader
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
  ss.end();
}
 
//NTX functions from UKHAB

  void rtty_txstring (char * string) {
    /* Simple function to sent a char at a time to
    ** rtty_txbyte function.
    ** NB Each char is one byte (8 Bits)
    */
 
    char c;
    c = *string++;
 
    while ( c != '\0') {
        rtty_txbyte (c);
        c = *string++;
    }
}
void rtty_txbyte (char c) {
    /* Simple function to sent each bit of a char to
    ** rtty_txbit function.
    ** NB The bits are sent Least Significant Bit first
    **
    ** All chars should be preceded with a 0 and
    ** proceed with a 1. 0 = Start bit; 1 = Stop bit
    **
    */
 
    int i;
 
    rtty_txbit (0); // Start bit
 
    // Send bits for for char LSB first
 
    for (i=0;i<7;i++) { // Change this here 7 or 8 for ASCII-7 / ASCII-8
        if (c & 1) rtty_txbit(1);
        else rtty_txbit(0);
 
        c = c >> 1;
    }
    rtty_txbit (1); // Stop bit
    rtty_txbit (1); // Stop bit
}
 
void rtty_txbit (int bit) {
    if (bit) {
        // high
        analogWrite(RADIOPIN,110);
    }
    else {
        // low
        analogWrite(RADIOPIN,100);
    }
 
    //delayMicroseconds(3370); // 300 baud
    delayMicroseconds(10000); // For 50 Baud uncomment this and the line below.
    //delayMicroseconds(10150); // You can't do 20150 it just doesn't work as the
    // largest value that will produce an accurate delay is 16383
    // See : http://arduino.cc/en/Reference/DelayMicroseconds
}
 
uint16_t gps_CRC16_checksum (char *string) {
    size_t i;
    uint16_t crc;
    uint8_t c;
 
    crc = 0xFFFF;
 
    // Calculate checksum ignoring the first two $s
    for (i = 2; i < strlen(string); i++) {
        c = string[i];
        crc = _crc_xmodem_update (crc, c);
    }
 
    return crc;
}
 
void setPwmFrequency(int pin, int divisor) {
    byte mode;
    if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
        switch(divisor) {
            case 1:
                mode = 0x01;
                break;
            case 8:
                mode = 0x02;
                break;
            case 64:
                mode = 0x03;
                break;
            case 256:
                mode = 0x04;
                break;
            case 1024:
                mode = 0x05;
                break;
            default:
                return;
        }
 
        if(pin == 5 || pin == 6) {
            TCCR0B = TCCR0B & 0b11111000 | mode;
        }
        else {
            TCCR1B = TCCR1B & 0b11111000 | mode;
        }
    }
    else if(pin == 3 || pin == 11) {
        switch(divisor) {
            case 1:
                mode = 0x01;
                break;
            case 8:
                mode = 0x02;
                break;
            case 32:
                mode = 0x03;
                break;
            case 64:
                mode = 0x04;
                break;
            case 128:
                mode = 0x05;
                break;
            case 256:
                mode = 0x06;
                break;
            case 1024:
                mode = 0x7;
                break;
            default:
                return;
        }
        TCCR2B = TCCR2B & 0b11111000 | mode;
    }
}




double Thermistor(int RawADC)
{
  double Temp;
  Temp = log(10000.0 * ((1024.0 / RawADC - 1)));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  Temp = Temp - 273.15;            // Convert Kelvin to Celcius
  Temp = Temp + 43;
  
  //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  return Temp;
}


void readAna(int AnaPin1)
{
  int readVal = analogRead(AnaPin1);
  exTemp =  Thermistor(readVal);

  Serial.println(readVal);
  Serial.println(exTemp);  // display tempature
  //Serial.println(readVal);  // display tempature
}

void ErrCheck()
{
  act = 0;
  //Clear previous error flags
  int i;
  for (i = 0; i < 9; i = i + 1) {
    Err[i] = "";
  }
  
  //We check all sensor values and status values. If any of them is failing or otherwise not working, we raise associated error flags.9
  if(strlen(GPSTime) == 0) { Err[3] = Err[3] + "3Time"; act = 1; }
  if(isnan(GPSLong) || isnan(GPSLat)) { Err[3] = Err[3] + "3Coord"; act = 1; }
  if(isnan(GPSAlt) || isnan(GPSSpeed) || isnan(GPSTrack)) { Err[3] = Err[3] + "3Nav"; act = 1; }
  if(isnan(IntTemp)) { Err[1] = Err[1] + "1ITmp"; act = 1; }
  if(isnan(exTemp)) {  Err[4] = Err[4] + "4ETmp"; act = 1; }
  if(Hum == 0) {  Err[4] = Err[4] + "4Hum"; act = 1; }
  if(isnan(BaroAlt)) {  Err[1] = Err[1] + "1BAlt"; act = 1; }
  if(isnan(AmbPress)) {  Err[1] = Err[1] + "1APress"; act = 1; }
  if(AmbPress < 300) {  Err[8] = Err[8] + "AltLim"; act = 1; }
  if(isnan(zRot)) {  Err[2] = Err[2] + "3IMU"; act = 1; }
  
  //if(sizeof(Err) > 2)
  if(act == 1)
   {
    Sys = "";
    int Arr;
    for(int Arr = 0 ; Arr < 10; Arr = Arr + 1)
    {
     Sys = Sys + Err[Arr];
    } 
   } else {
      Sys = "OK";
    }       
}

void dhtRead() //simple function to to talk to DHT library to read out desired values. 
{
  delay(1200);
  Hum = dht.readHumidity();
  exTemp = dht.readTemperature();
}
