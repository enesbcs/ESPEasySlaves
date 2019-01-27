/****************************************************************************************************************************\
   Arduino project "ESP Easy" � Copyright www.esp8266.nu

   This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
   This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
   You received a copy of the GNU General Public License along with this program in file 'License.txt'.

   IDE download    : https://www.arduino.cc/en/Main/Software
   ESP8266 Package : https://github.com/esp8266/Arduino

   Source Code     : https://sourceforge.net/projects/espeasy/
   Support         : http://www.esp8266.nu
   Discussion      : http://www.esp8266.nu/forum/

   Additional information about licensing can be found at : http://www.gnu.org/licenses
  \*************************************************************************************************************************/

// This file is to be loaded onto an Arduino Pro Mini so it will act as a simple IO extender to the ESP module/Raspberry PI.
// Communication between Master and Arduino is using the I2C bus, so only two wires needed.
// It best to run the Pro Mini on 3V3, although the 16MHz versions do not officially support this voltage level on this frequency.
// That way, you can skip levelconverters on I2C.
// Arduino Mini Pro uses A4 and A5 for I2C bus. ESP I2C can be configured but they are on GPIO-4 and GPIO-5 by default.

#include <Wire.h>

#define I2C_MSG_IN_SIZE    4
#define I2C_MSG_OUT_SIZE   4

#define CMD_DIGITAL_WRITE  1
#define CMD_DIGITAL_READ   2
#define CMD_ANALOG_WRITE   3
#define CMD_ANALOG_READ    4
#define CMD_ASK_MAX_FUNC   0x10

volatile byte I2CReceived  = 0;
volatile byte I2CDataReady = 0;
volatile uint8_t sendBuffer[I2C_MSG_OUT_SIZE];
uint8_t EmptBuffer[I2C_MSG_OUT_SIZE];
volatile int value = 0;
volatile byte port = 0;
volatile byte cmd = 0;

void setup()
{
  for (byte x = 0; x < sizeof(sendBuffer); x++)
    sendBuffer[x] = 0;
  for (byte x = 0; x < sizeof(EmptBuffer); x++)
    EmptBuffer[x] = 0xff;
  Wire.begin(0x3f);  // Wire.begin(0x7f); // 0x7f is outside valid range as per https://www.totalphase.com/support/articles/200349176-7-bit-8-bit-and-10-bit-I2C-Slave-Addressing
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  if (I2CReceived) { // if command arrived through i2c, do it
    switch (cmd)
    {
      case CMD_DIGITAL_WRITE:
        pinMode(port, OUTPUT);
        digitalWrite(port, value);
        break;
      case CMD_DIGITAL_READ:
        if (I2CDataReady) {
          delay(1);       // if there is unread data in buffer from previous transaction, give some more time to read by remote Master
        }
        I2CDataReady = 0; // after this point, data will be overwritten
        pinMode(port, INPUT_PULLUP);
        sendBuffer[0] = digitalRead(port);
        sendBuffer[1] = 0;
        I2CDataReady = 1;
        break;
      case CMD_ANALOG_WRITE:
        analogWrite(port, value);
        break;
      case CMD_ANALOG_READ:
        if (I2CDataReady) {
          delay(1);       // if there is unread data in buffer from previous transaction, give some more time to read by remote Master
        }
        I2CDataReady = 0; // after this point, data will be overwritten
        int valueRead = analogRead(port); // analogRead() is a slow function, make sure to not use in an ISR function like receiveEvent()
        sendBuffer[0] = valueRead & 0xff;
        sendBuffer[1] = valueRead >> 8;
        I2CDataReady = 1;
        break;
    }
    if (cmd == CMD_ASK_MAX_FUNC) { // for some reason this is not working inside switch-case
      if (I2CDataReady) {
        delay(1);       // if there is unread data in buffer from previous transaction, give some more time to read by remote Master
      }
      I2CDataReady = 0; // after this point, data will be overwritten
      sendBuffer[0] = CMD_ASK_MAX_FUNC;
      sendBuffer[1] = 0xfe;
      sendBuffer[2] = CMD_ANALOG_READ; // feedback maximum supported function number
      sendBuffer[3] = 2;               // sketch version
      I2CDataReady = 1;                // sign data ready for requestevent()
    }
    I2CReceived = 0; // sign that remotely requested operation is ended, we are ready for the next one
  }
}

void receiveEvent(int count)   // try to avoid complex instructions in ISR
{
  if (count >= I2C_MSG_IN_SIZE) // make sure to also handle larger incoming packages
  {
    byte lcmd;
    byte lport;
    int lvalue;
    byte ival;

    for (byte i = 0; i <= count; i++) { // simply read all data above 4, do not leave data in the buffer
      ival = Wire.read();
      switch (i) {
        case 0:
          lcmd = ival;
          break;
        case 1:
          lport = ival;
          break;
        case 2:
          lvalue = ival;
          break;
        case 3:
          if (I2CReceived == 0) {    // if loop is not ready for serving request, just skip it, sorry (or implement a queue if needed)
            value = (ival << 8 | lvalue);
            cmd = lcmd;
            port = lport;
            I2CReceived = 1; // only sign ready if all data arrived, loop will continue processing if it can
          }
          break;
      }

    }
  }
}

void requestEvent()
{
  if (I2CDataReady) {
    Wire.write((const uint8_t*)sendBuffer, I2C_MSG_OUT_SIZE);
    I2CDataReady = 0; // data can be gathered once!
  } else {
    Wire.write((const uint8_t*)EmptBuffer, I2C_MSG_OUT_SIZE); // if no incoming command arrived, respond with 0xff,0xff,0xff,0xff
  }
}
