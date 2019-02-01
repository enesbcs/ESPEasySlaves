/********************************************************************************************************************\

    Arduino project "ESP Easy" Copyright www.esp8266.nu
    This program is free software: you can redistribute it and/or modify it under the terms of the GNU
    General Public License as published by the Free Software Foundation, either version 3 of the License,
    or (at your option) any later version.
    This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
    without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details.
    You received a copy of the GNU General Public License along with this program in file 'License.txt'.
    IDE download : https://www.arduino.cc/en/Main/Software
    ESP8266 Package : https://github.com/esp8266/Arduino
    Source Code : https://sourceforge.net/projects/espeasy/
    Support : http://www.esp8266.nu
    Discussion : http://www.esp8266.nu/forum/
    Additional information about licensing can be found at : http://www.gnu.org/licenses
    ******************************************************************************************************************/
// This file is to be loaded onto an Arduino Pro Mini so it will act as a simple IO extender to the ESP module.
// Communication between ESP and Arduino is using the I2C bus, so only two wires needed.
// It is possible to run the Pro Mini on 3V3, although the 16MHz versions do not officially support this at 16MHzl
// By working on 3.3volt you can skip levelconverters on I2C, but no guarantee that it will work stable.
// Arduino Mini Pro uses A4 and A5 for I2C bus. ESP I2C can be configured but they are on GPIO-4 and GPIO-5 by default.

#include <Wire.h>

#define I2C_MSG_IN_SIZE 4
#define I2C_MSG_OUT_SIZE 4

#define CMD_DIGITAL_WRITE 1
#define CMD_DIGITAL_READ 2
#define CMD_ANALOG_WRITE 3
#define CMD_ANALOG_READ 4
#define CMD_ASK_MAX_FUNC   0x10

volatile byte I2CReceived  = 0;
volatile byte I2CDataReady = 0;
volatile uint8_t sendBuffer[I2C_MSG_OUT_SIZE];
uint8_t EmptBuffer[I2C_MSG_OUT_SIZE];
volatile int value = 0;
volatile byte port = 0;
volatile byte cmd = 0;
int ESPrms=0; // value to be sent over I2C to ESP

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

/* ******************************************************************************************************************
  This part in the loop found here: [url] https://forum.arduino.cc/index.php?topic=179541.0 [/url] tnx to dc42
  constant 75.7576 depends on the sensitivity of the ACS712 module
  Sensitivity Min<Typ<Max mV/A
  for 30A: 64< 66< 68 mV/A > constant = 5/.066 = 75.76
  for 10A: 96<100<104 mV/A > constant = 5/.1 = 50.00
  for 5A: 180<185<190 mV/A > constant = 5/.185 = 27.03
*/

const int currentPin = A7; // ADC pin used for ACS712
const unsigned long sampleTime = 100000UL; // sample over 100ms, it is an exact number of cycles for both 50Hz and 60Hz mains
const unsigned long numSamples = 250L; // number of samples 400 microsecond each
const unsigned long sampleInterval = sampleTime / numSamples; // sampling interval, must be longer than then ADC conversion time
int adc_zero = 514; // relative zero of the ACS712 for me 514 was best. 511
int valueRead; // value read on ADC

void handleI2C()
{
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
        if (port <= 4) valueRead = analogRead(port); // port <=4 to read analog value A0,A1,A2,A3 - A4 & A5 are I2C
        if (port > 79) valueRead = ESPrms; // port is any number >4 up to 255
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
      sendBuffer[3] = 0x11;            // sketch version
      I2CDataReady = 1;                // sign data ready for requestevent()
    }
    I2CReceived = 0; // sign that remotely requested operation is ended, we are ready for the next one
  }
}

void loop()
{
  unsigned long currentAcc = 0;
  unsigned int count = 0;
  handleI2C();
  unsigned long prevMicros = micros() - sampleInterval ;
  while (count < numSamples)
  {
    handleI2C();
    if (micros() - prevMicros >= sampleInterval)
    {
      long adc_raw = analogRead(currentPin) - adc_zero;
      currentAcc += (unsigned long)(adc_raw * adc_raw);
      ++count;
      prevMicros += sampleInterval;
    }
  }
  float rms = sqrt((float)currentAcc / (float)numSamples) * (27.03 / 1024.0); // see note above for this 27.03 value
  ESPrms = 1000 * rms; // conversion of float Ampere into integer milliAmpere needed for I2C communication
  handleI2C();
}

void receiveEvent(int count)
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
