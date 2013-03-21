/*
 HABJOE1 Arduino X Flight script
 HABJOE - Andrew Myatt and Joseph Myatt Jan/2013

 This example connects the GPS via Software Serial.
 Initialise the GPS Module in Flight Mode and then echo's out the NMEA Data to the Arduinos onboard Serial port.
 
 This code is in the public domain.
 Incorporating code by:
 J Coxon (http://ukhas.org.uk/guides:falcom_fsa03)
 
 Incorprating modified elements of acceleroMMA7361 - Library for retrieving data from the MMA7361 accelerometer. 
 Copyright 2011-2012 Jef Neefs (neefs@gmail.com) and Jeroen Doggen (jeroendoggen@gmail.com)
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 NTX2 Radio. 2012 by M0UPU as part of a UKHAS Guide on linking NTX2 Modules to Arduino.
 RTTY code from Rob Harrison Icarus Project. http://ukhas.org.uk
 UBX code. http://ukhas.org.uk
 
 I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg

 + Various others sources of code!!
  
 This software is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
*/ 
#include <SdFat.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

/***************************************************
 * Main program
 **************************************************/
#define LED_NTX 6
#define LED_SMS 5
#define LED_ORANGE 7
#define LED_WHITE 4
#define LED_RED 3
#define LED_BLUE 2
#define LED_GREEN 1
#define LED_OFF 0
//
#define WAIT_ARD_Y  10000
// Define filenames
//
#define SD_LOG_FILE        "HABJOE.CSV"
//
// Arduino Pin assignment
//
// Note VCC 3.3 Volts is connected to AREF

//int PIN_GPS_RX = 0;              //Fixed:Note: RX on Board is connected to RX on GPS Board
//int PIN_GPS_TX = 1;              //Fixed:Note: TX on Board is connected to TX on GPS Board
static int PIN_ARDY_TX = 3;        //Note: Arduino Y TX
static int PIN_ARDY_RX = 4;        //Note: Arduino Y RX
static int PIN_HALT = 2;           //Note: High Altitude Signal
static int PIN_LED_GREEN = 6;      //Fixed: Blue GREEN
static int PIN_LED_BLUE = 7;       //Fixed: Blue LED
static int PIN_SPI_CS = 8;         //Fixed: Card Select for SD Card
static int PIN_LED_RED = 9;        //Fixed: Red LED
static int PIN_AC_SENS = 10;       //Fixed: A Sensitivity (Note also needed for SPI to be OUTPUT)
//static int PIN_SPI_MOSI = 11;    //Fixed: MOSI
//static int PIN_SPI_MISO = 12;    //Fixed: MISO
//static int PIN_SPI_SCK = 13;     //Fixed: SCK

static int PIN_AC_Z = A2;          //Fixed: A Z
static int PIN_AC_Y = A1;          //Fixed: A Y
static int PIN_AC_X = A0;          //Fixed: A X

//int PIN_AC_SLP = -1;     //Not assigned
//int PIN_AC_ST = -1;      //Not assigned

unsigned long CountRec = 0;

//Define structures
TinyGPS gps;              //GPS
SdFat SD;
SdFile dataFile;
SoftwareSerial arduinoY(PIN_ARDY_RX, PIN_ARDY_TX);  // Arduino Y

//Define Global Variables

char comm[] = ",";
char NotValid[] = "***";
unsigned long gblARDYElapsed;
float         glb_ref_Lat =0.0;
float         glb_ref_Long =0.0;
float         glb_ref_Alt= 0.0;
int           AccOffSets[3];
boolean       AccSensi;

/// setOffSets( int offSetX, int offSetY, int offSetZ): Sets the offset values for the x,y,z axis.
/// The parameters are the offsets expressed in G-force (100 = 1 G)
/// Offsets are added to the raw datafunctions
void setOffSets(int xOffSet, int yOffSet, int zOffSet) {
    AccOffSets[0]= map(xOffSet,0,3300,0,1024);
    AccOffSets[1]= map(yOffSet,0,3300,0,1024);
    AccOffSets[2]= map(zOffSet,0,3300,0,1024);
}

/// setSensitivity sets the sensitivity to +/-1.5 G (HIGH) or +/-6 G (LOW) using a boolean HIGH (1.5 G) or LOW (6 G)
void setSensitivity(boolean sensi) {
  AccSensi = sensi;
  digitalWrite(PIN_AC_SENS, !sensi);
}

/// getXRaw(): Returns the raw data from the X-axis analog I/O port of the Arduino as an integer
int getXRaw() {
  return analogRead(PIN_AC_X)+AccOffSets[0]+2;
}

/// getYRaw(): Returns the raw data from the Y-axis analog I/O port of the Arduino as an integer
int getYRaw() {
  return analogRead(PIN_AC_Y)+AccOffSets[1]+2;
}

/// getZRaw(): Returns the raw data from the Z-axis analog I/O port of the Arduino as an integer
int getZRaw() {
  return analogRead(PIN_AC_Z)+AccOffSets[2];
}

/// getAccelXYZ(int *_XAxis, int *_YAxis, int *_ZAxis) returns all axis at once as pointers
void getAccelXYZ(int *_XAxis, int *_YAxis, int *_ZAxis) {
  int sum[3];
  sum[0] = 0;
  sum[1] = 0;
  sum[2] = 0;
  for (int i = 0;i<10;i++) {
    sum[0] = sum[0] + AccMapG(getXRaw());
    sum[1] = sum[1] + AccMapG(getYRaw());
    sum[2] = sum[2] + AccMapG(getZRaw());
  }
  *_XAxis = sum[0]/10;
  *_YAxis = sum[1]/10;
  *_ZAxis = sum[2]/10;
}

/// mapMMA7361V: calculates and returns the voltage value derived from the raw data. Used in getXVoltage, getYVoltage, getZVoltage
int AccMapV(int value) {
    return map(value,0,1024,0,3300);
}

/// mapMMA7361G: calculates and returns the accelerometer value in degrees derived from the raw data. Used in getXAccel, getYAccel, g
int AccMapG(int value) {
  if(AccSensi == false) {
      return map(value,0,1024,-825,800);
  } else {
      return map(value,0,1024,-206,206);
  }
}

/// calibrate(): Sets X and Y values via setOffsets to zero. The Z axis will be set to 100 = 1G
/// WARNING WHEN CALIBRATED YOU HAVE TO MAKE SURE THE Z-AXIS IS PERPENDICULAR WITH THE EARTHS SURFACE
/*
void AccCalibrate(SdFile &dataFile) {
  double var = 5000;
  double sumX = 0;
  double sumY = 0;
  double sumZ = 0;
  for (int i = 0;i<var;i++) {
    sumX = sumX + AccMapV(getXRaw());
    sumY = sumY + AccMapV(getYRaw());
    sumZ = sumZ + AccMapV(getZRaw());
  }
    if (AccSensi == false) {
    setOffSets(1672 - sumX / var,1671 - sumY / var, + 1876 - sumZ / var);
  } else  {
    setOffSets(1650 - sumX / var,1650 - sumY / var, + 2450 - sumZ / var);
  }
    char BufString[10];
    int i = 1672 - sumX / var;
    sprintf(BufString, "%d", i);
    dataFile.println(BufString);
    i = 1671 - sumY / var;
    sprintf(BufString, "%d", i);
    dataFile.println(BufString);
    i = + 1876 - sumZ / var;
    sprintf(BufString, "%d", i);
    dataFile.println(BufString);
    i =1650 - sumX / var;
    sprintf(BufString, "%d", i);
    dataFile.println(BufString);
    i =1650 - sumY / var;
    sprintf(BufString, "%d", i);
    dataFile.println(BufString);
    i = + 2450 - sumZ / var;
    sprintf(BufString, "%d", i);
    dataFile.println(BufString);

  if (abs(getOrientation())!=3) {
    setOffSets(0,0,0);
  }
}
*/
/// getOrientation returns which axis perpendicular with the earths surface x=1,y=2,z=3 is positive or
/// negative depending on which side of the axis is pointing downwards
int getOrientation() {
  int gemiddelde = 10;
  int x = 0;
  int y = 0;
  int z = 0;
  int xAbs = 0;
  int yAbs = 0;
  int zAbs = 0;
  for(int i = 0; i<gemiddelde ; i++)              //We take in this case 10 measurements to average the error a little bit
  {
    int xi = 0;
    int yi = 0;
    int zi = 0;
    getAccelXYZ(&xi, &yi, &zi);
    x = x+xi;
    y = y+yi;
    z = z+zi;
  }
  x= x/gemiddelde;
  y = y/gemiddelde;
  z = z/gemiddelde;
  xAbs = abs(100-abs(x));
  yAbs = abs(100-abs(y));
  zAbs = abs(100-abs(z));
  if (xAbs<yAbs&&xAbs<zAbs) {
    if (x>0) {
      return 1;
    }
    return -1;
  }
  if (yAbs<xAbs&&yAbs<zAbs) {
    if (y>0) {
      return 2;
    }
    return -2;
  }
  if (zAbs<xAbs&&zAbs<yAbs) {
    if (z>0) {
      return 3;
    }
    return -3;
  }
  return 0;
}

/// getTotalVector returns the magnitude of the total acceleration vector as an integer
int getTotalVector(){
  return sqrt(square(AccMapG(getXRaw())) + square(AccMapG(getYRaw())) + square(AccMapG(getZRaw())));
}

/***************************************************
 * LED STATUS DISPLAY FUNTIONS
 * LED is a Common Anode, therefore the pin is the cathode
 * and must be set LOW to be on, and HIGH to be turned off! 
 **************************************************/
void LED_Status(int stat, int intDelay){

  if (stat == LED_RED) {
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_BLUE, HIGH);  
  } 
  else if (stat == LED_BLUE) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_BLUE, LOW);  
  } 
  else if (stat == LED_GREEN){
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_BLUE, HIGH);  
  } 
  else if (stat == LED_WHITE){
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_BLUE, LOW); 
  } else {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_BLUE, HIGH); 
  }
  if (intDelay > 0 ) delay(intDelay);
}

void LED_Status(int stat){
  LED_Status(stat, 1000);
}


    

/***************************************************
 * MBLOK Specific Setup functions
 * 
 **************************************************/
/*
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
  Serial.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0;  // CK_A
  ackPacket[9] = 0;  // CK_B

  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }
  }
}
*/
/***************************************************
 * SEND Location functions
 * 
 * 
 **************************************************/

void gpsdump(TinyGPS &gps, SdFile &dataFile) {
  int Ac_degree_x, Ac_degree_y, Ac_degree_z, Ac_Force, Ac_Orientation;
  int year;
  byte month, day, hour, minute, second, hundredths;
  float lFlAngle, lFlSpeed;
  char SDString[150] = "";
  char AYString[100] = "$$HABJOE,";
  char BufString[25];
  unsigned short sats;
  unsigned long age, ihop;

  CountRec++;
  
  // We get Position and Accelerations at as closer time together as possible

  getAccelXYZ(&Ac_degree_x,&Ac_degree_y,&Ac_degree_z);     // returns G where 100 = 1 G
  Ac_Force = getTotalVector();                             // total Vector or force returns G where 100 = 1 G
  Ac_Orientation = getOrientation();                       // x=1,y=2,z=3 is positive or negative
  gps.f_get_position(&glb_ref_Lat, &glb_ref_Long, &age); 
  glb_ref_Alt = gps.f_altitude();
  if (glb_ref_Alt > 8000) {
    digitalWrite(PIN_HALT, HIGH);
  } else {
    digitalWrite(PIN_HALT, LOW);
  }
  lFlAngle = gps.f_course();
  lFlSpeed = gps.f_speed_kmph();
  sats = gps.satellites();
  ihop = gps.hdop();
  //Date Time
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  // age and Date Time
  if (age == TinyGPS::GPS_INVALID_AGE) {
    strcat(SDString, NotValid);  //Age
    strcat(SDString,comm);    
    strcat(SDString, NotValid);  //Date
    strcat(AYString, NotValid);  //Date
  } else {
    sprintf(BufString, "%ld", age);
    strcat(SDString,BufString);
    strcat(SDString,comm);
//    sprintf(BufString, "%02d/%02d/%02d %02d:%02d:%02d.%02d", month, day, year, hour, minute, second, hundredths);
    sprintf(BufString, "%02d:%02d:%02d",hour, minute, second);
    strcat(SDString,BufString);
    strcat(AYString,BufString);

  }
  strcat(SDString,comm);
  strcat(AYString,comm);
  
  //Lat
  if (glb_ref_Lat == TinyGPS::GPS_INVALID_F_ANGLE) {
    strcat(SDString, NotValid);
    strcat(AYString, NotValid);
  } else {
    dtostrf(glb_ref_Lat,10,5,BufString);  
    strcat(SDString,BufString);
    strcat(AYString,BufString);
  }
  strcat(SDString,comm);
  strcat(AYString,comm);
  //Long
  if (glb_ref_Long == TinyGPS::GPS_INVALID_F_ANGLE) {
    strcat(SDString, NotValid);
    strcat(AYString, NotValid);
  } else {
    dtostrf(glb_ref_Long,10,5,BufString);  
    strcat(SDString,BufString);
    strcat(AYString,BufString);
  }
  strcat(SDString,comm);
  strcat(AYString,comm);
  //Alt
  if (glb_ref_Alt == TinyGPS::GPS_INVALID_F_ALTITUDE) {
    strcat(SDString, NotValid);
    strcat(AYString, NotValid);
  } else {
    dtostrf(glb_ref_Alt,7,1,BufString);  
    strcat(SDString,BufString);
    strcat(AYString,BufString);
  }
  strcat(SDString,comm);
  strcat(AYString,comm);
  // Angle
  if (lFlAngle == TinyGPS::GPS_INVALID_F_ANGLE) {
    strcat(SDString, NotValid);
  } else {
    dtostrf(lFlAngle,5,1,BufString);  
    strcat(SDString,BufString);
  }
  strcat(SDString,comm);

  // Speed
  if (lFlSpeed == TinyGPS::GPS_INVALID_F_SPEED) {
    strcat(SDString, NotValid);
    strcat(AYString, NotValid);
  } else {
    dtostrf(lFlSpeed,5,1,BufString);  
    strcat(SDString,BufString);
    strcat(AYString,BufString);
  }
  strcat(SDString,comm);
  strcat(AYString,comm);
  
  //Acc X
  sprintf(BufString, "%d", Ac_degree_x);
  strcat(SDString,BufString);
  strcat(SDString,comm);

  //Acc Y
  sprintf(BufString, "%d", Ac_degree_y);
  strcat(SDString,BufString);
  strcat(SDString,comm);

  //Acc Z
  sprintf(BufString, "%d", Ac_degree_z);
  strcat(SDString,BufString);
  strcat(SDString,comm);

  //Acc Force
  sprintf(BufString, "%d", Ac_Force);
  strcat(SDString,BufString);
  strcat(AYString,BufString);
  strcat(SDString,comm);
  strcat(AYString,comm);  
  //Satellites
  if (sats == TinyGPS::GPS_INVALID_SATELLITES) {
    strcat(SDString, NotValid);
    strcat(AYString, NotValid);

  } else {
    sprintf(BufString, "%hd", sats);
    strcat(SDString,BufString);
    strcat(AYString,BufString);
  }
  strcat(SDString,comm);
  //Acc Orientation
  sprintf(BufString, "%d", Ac_Orientation);
  strcat(SDString,BufString);

  //HDOP
  if (ihop == 0) {
    strcat(SDString, NotValid);
  } else {
    sprintf(BufString, "%ld", ihop);
    strcat(SDString,BufString);
  }
  strcat(SDString,comm);

  //Count
  sprintf(BufString, "%ld", CountRec);
  strcat(SDString,BufString);
  strcat(SDString,comm);

  //Store it
  dataFile.println(SDString);
  dataFile.sync();
  unsigned long m = millis();
  //Send it
  if ((m - gblARDYElapsed) > WAIT_ARD_Y){
      arduinoY.print(AYString);
      arduinoY.print('\n');
      gblARDYElapsed = millis();
  }
}

void setup() {

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  LED_Status(LED_OFF,0);
  pinMode(PIN_HALT,OUTPUT);
  pinMode(PIN_AC_SENS,OUTPUT);
  pinMode(PIN_AC_X, INPUT);
  pinMode(PIN_AC_Y, INPUT);
  pinMode(PIN_AC_Z, INPUT);
  pinMode(PIN_SPI_CS, OUTPUT);  //Chip Select Pin for the SD Card
  analogReference(EXTERNAL);
  setSensitivity(HIGH);
//  setOffSets(-131,-103,247);
//  setSensitivity(LOW);
  setOffSets(-153,-124,821);

  //Calibrate the Accelerometer
  
  //Connect to the SD Card
  if(!SD.init(SPI_HALF_SPEED, PIN_SPI_CS)) {
    LED_Status(LED_WHITE);
    LED_Status(LED_RED);
    LED_Status(LED_OFF);
    return;
  }
  //Check if Log file exists
  boolean  bolFileexists = false;
  if (SD.exists(SD_LOG_FILE)) bolFileexists = true;
  //Open Logfile
  dataFile.open(SD_LOG_FILE, O_CREAT | O_WRITE | O_APPEND);
  if (!dataFile.isOpen()) {
    LED_Status(LED_WHITE,500);
    LED_Status(LED_RED,500);
    LED_Status(LED_WHITE,500);
    LED_Status(LED_RED,500);
    LED_Status(LED_OFF);
    return;
  }
  //Create header its Logfile new
  if (!bolFileexists) {
//     AccCalibrate(dataFile);
    char SDString[] = "Age,Date Time, Latitude, Longitude, Altitude,  Sats, Angle, Speed (Km), GX, GY, GZ, GForce, Orien, Seq, HD,";
    dataFile.println(SDString);
    dataFile.sync();
    LED_Status(LED_BLUE);
  } 

  //Open Serial interfaces GPS is on Hardware Serial
  Serial.begin(9600);
  arduinoY.begin(19200);
  LED_Status(LED_WHITE,500);
  LED_Status(LED_GREEN,500);
  gblARDYElapsed = millis();
  digitalWrite(PIN_HALT, LOW);
}

void loop() {
  LED_Status(LED_OFF,0);

  unsigned long start = millis();
  bool newdata = false;
  bool LookOnce = false;
  while (!LookOnce || (millis() - start) < 900) {
    LookOnce = true;
    if (Serial.available()) {
      if (gps.encode(Serial.read())) newdata = true;
    }
  }
  if (!newdata) {
    LED_Status(LED_WHITE);
  } else {
    LED_Status(LED_BLUE,0);
  }
  gpsdump(gps,dataFile);  
}



/*
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
*/




