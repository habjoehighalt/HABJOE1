/*
 HABJOE1 Arduino Y Flight script
 HABJOE - Andrew Myatt and Joseph Myatt Jan/2013


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

#include <stdio.h>
#include <SoftwareSerial.h>
#include <util/crc16.h>


/***************************************************
 * Main program
 **************************************************/

#define LED_GREEN 1
#define LED_OFF 0
#define WAIT_SMS  180000
#define INDATASIZE 128
// Define Phone AT commands

//#define PHONE_SMS_NUMBER "AT+CMGW=\"+4xxxxxxxx\""     //The number to send the SMS
//#define PHONE_AT  "AT"                                   //wake up cell phone
//#define PHONE_RESET_COMMAND "AT*SWRESET"                 // Resets the phone
//#define PHONE_SET_SMS_TXT_MODE_COMMAND "AT+CMGF=1"       //  Puts the phone into SMS texting mode ( as opposed to MMS, etc.)
//#define PHONE_SEND_MSG "AT+CMGS="                        //  Command used to send a message
//#define PHONE_SEND_SMS  "AT+CMSS=1"                      // Sends message at index of 1
//#define PHONE_DELETE_SMS  "AT+CMGD=1"                    // Deletes message at index of 1
//#define PHONE_TURN_OFF_COMMAND "AT+CFUN=0"               // Turns OFF the phone's transmitter
//#define PHONE_TURN_ON_COMMAND "AT+CFUN=1"                // Turns ON the Phone's Transmitter
//#define PHONE_BATT_CHARGE_CHECK_COMMAND  "AT+CBC"        // This command will return the battery status (0-100)
//#define PHONE_BATT_CHARGE_CHECK_COMMAND  "AT+CBC"        // This command will return the battery status (0-100)
//#define PHONE_SIGNAL_STRENGTH_CHECK_COMMAND  "AT+CSQ"    // This command returns the signal strength (0-30)

static char PHONE_SMS_NUMBER[] = "AT+CMGW=\"+4xxxxxxxx\"";    //The number to send the SMS
static char PHONE_AT[]  = "AT";                                   //wake up cell phone
static char PHONE_SET_SMS_TXT_MODE_COMMAND[] ="AT+CMGF=1";       //  Puts the phone into SMS texting mode ( as opposed to MMS, etc.)
static char PHONE_SEND_SMS[] = "AT+CMSS=1";                     // Sends message at index of 1
static char PHONE_DELETE_SMS[] = "AT+CMGD=1";                    // Deletes message at index of 1

//
// Arduino Pin assignment
//
static int PIN_PHN_TX = 2;        //Note: Phone TX
static int PIN_PHN_RX = 3;        //Note: Phone RX
//static int PIN_AX_RX = 4;         //Fixed:Note: RX on Board is connected to RX on Arduino Board X
//static int PIN_AX_TX = 11;        //Fixed:Note: RX on Board is connected to TX on Arduino Board X
static int PIN_HALT = 10;         //Note: High Altitude Set recieved as High when high altitude and phone transmitter to be switched off.
static int PIN_NTX_TX = 9;        //Note: NTX Transmitter out
static int PIN_LED_GREEN = 13;    //Fixed: GREEN

//Define structures
SoftwareSerial mobPhone(PIN_PHN_RX, PIN_PHN_TX); // Mobile Phone RX, TX pins

//Define Global Variables
unsigned long gblSMSElapsed;
int glb_phone_trans = LOW;
//int glb_change = 0;

void setup() {
  pinMode(PIN_HALT, INPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_NTX_TX,OUTPUT);
  LED_Status(LED_GREEN,0);
  Serial.begin(19200);
  mobPhone.begin(4800);
  Serial.begin(19200); 
  gblSMSElapsed = millis();
}

//char command[256];
//char commandClean[256];
char command[128];
char commandClean[60];
char commandBuffer[128];
int commandBufferSize = 0;

void readCommandBuffer(int bytesToRead) {
  int i = 0;
	char c = 0;
	while (i < 128 && (i < bytesToRead || bytesToRead <= 0)) {
		while (!Serial.available())
			;
		c = Serial.read();
		if (c == '\r' || c == '\n') {
			break;
		}
		commandBuffer[i] = c;
		i++;
	}
	commandBufferSize = i;
}

void readCommand() {
	command[0] = '\0';
	readCommandBuffer(0);
	if (strncmp(commandBuffer, "RCV", 3) == 0) {
		commandBuffer[commandBufferSize] = '\0';
		int expectedSize = atoi(commandBuffer + 4);
		if (expectedSize <= 0 || expectedSize > 1024) {
			return;
		}
		int bytesRead = 0;
		while (bytesRead < expectedSize) {
			readCommandBuffer(expectedSize - bytesRead);
			memcpy(command + bytesRead, commandBuffer, commandBufferSize);
			bytesRead += commandBufferSize;
		}
		command[bytesRead] = '\0';
	} else {
		memcpy(command, commandBuffer, commandBufferSize);
		command[commandBufferSize] = '\0';
	}
}

void LED_Status(int stat, int intDelay){
  if (stat == LED_GREEN){
    digitalWrite(PIN_LED_GREEN, HIGH);
  } 
  else {
    digitalWrite(PIN_LED_GREEN, LOW); 
  }
  if (intDelay > 0 ) delay(intDelay);
}

void LED_Status(int stat){
  LED_Status(stat, 1000);
}

/***************************************************
 * NTX2 Transmit functions
 * 
 **************************************************/

void rtty_txstring ( char * string) {

  //Simple function to sent a char at a time to rtty_txbyte function. 
  char c;
  c = *string++;    
  while (c != '\n'){
    rtty_txbyte (c);
    c = *string++;
  }
  rtty_txbyte (c);

}

void rtty_txbyte (char c){
  /* Simple function to sent each bit of a char to 
   ** rtty_txbit function. 
   ** NB The bits are sent Least Significant Bit first
   **
   ** All chars should be preceded with a 0 and 
   ** proceded with a 1. 0 = Start bit; 1 = Stop bit
   **/

  int i;
  rtty_txbit (0); // Start bit
  // Send bits for for char LSB first 

  for (i=0;i<7;i++) {// Change this here 7 or 8 for ASCII-7 / ASCII-8
    if (c & 1) rtty_txbit(1); 
    else rtty_txbit(0); 
    c = c >> 1;
  }
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit){
  if (bit) {
    digitalWrite(PIN_NTX_TX, HIGH); 
  } else {
    digitalWrite(PIN_NTX_TX, LOW);
  }

  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
  delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.
}

uint16_t gps_CRC16_checksum (char *string){
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

/***************************************************
 * PHONE TRANSMITTER ON/OFF
 * 
 * 
 **************************************************/

void phoneTX() {
  int val = digitalRead(PIN_HALT);
  if (val != glb_phone_trans) {
      glb_phone_trans=val;    
  } 
  // Only change the status after 20 consistent readings
  //if (glb_change > 20) {
  //  glb_change = 0;
  //  if (glb_phone_trans == HIGH) {
  //    glb_phone_trans = LOW;
  //    mobPhone.println(PHONE_TURN_ON_COMMAND);  // The payload is coming down Turn on transmitter, no need to wait
  //  
  //  } else if (glb_phone_trans == LOW) {
  //    glb_phone_trans = HIGH;  
  //    mobPhone.println(PHONE_TURN_OFF_COMMAND);  // The payload is going Up Turn off transmitter, no need to wait
  //  }
  //}
}


void loop() {
  LED_Status(LED_GREEN,0);
  if (Serial.available()) {
	readCommand();
	// "command" now contains the full command
      commandClean[0]='\0';
      int i = 0;
      int j = 0;
      while (command[i] != '\n' && i<128 && j<60){
        if (command[i] != char(32)){
          commandClean[j] = command[i];
          j++;
        }
        i++;
      }
      commandClean[j] = '\n';
   
      // Send to transmitter     
      LED_Status(LED_GREEN,0);
      unsigned int CHECKSUM = gps_CRC16_checksum(commandClean);  // Calculates the checksum for this datastring
      char checksum_str[6];
      sprintf(checksum_str, "*%04X\n", CHECKSUM);
      strcat(commandClean,checksum_str); 
	  Serial.println(commandClean);
      rtty_txstring (commandClean);  
      LED_Status(LED_OFF,0);

	  // Send to Mobile Phone
      unsigned long m = millis(); 
      if ((m - gblSMSElapsed) > WAIT_SMS) {
		if (glb_phone_trans == LOW){
			// Send to mobile phone
			mobPhone.println(PHONE_AT);          // wakes up phone
			LED_Status(LED_GREEN,250);
			LED_Status(LED_OFF,250);
			mobPhone.println(PHONE_DELETE_SMS);  // deletes message at index of 1
			LED_Status(LED_GREEN,250);
			LED_Status(LED_OFF,250);
			mobPhone.println(PHONE_SET_SMS_TXT_MODE_COMMAND); // Puts phone into SMS mode
			LED_Status(LED_GREEN,250);
			LED_Status(LED_OFF,500);    
			mobPhone.println(PHONE_SMS_NUMBER); // My number
			LED_Status(LED_GREEN,250);
			LED_Status(LED_OFF,500);    
			mobPhone.print(commandClean);         //data
			mobPhone.write(byte(26));             // (signals end of message)
			LED_Status(LED_GREEN,250);
			LED_Status(LED_OFF,500);
			mobPhone.println(PHONE_SEND_SMS);     // Sends message at index of 1
		}
		gblSMSElapsed = millis();
      }

  } else {
    LED_Status(LED_OFF,0);
    LED_Status(LED_GREEN,500);
  }
  
  phoneTX();
  LED_Status(LED_OFF,0);
}
