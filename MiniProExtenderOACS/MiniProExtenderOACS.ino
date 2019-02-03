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

/*
  AC Light Control

  Updated by Robert Twomey rtwomey@u.washington.edu

  Changed zero-crossing detection to look for RISING edge rather
  than falling. (originally it was only chopping the negative half
  of the AC wave form).

  Also changed the dim_check() to turn on the Triac, leaving it on
  until the zero_cross_detect() turn's it off.

  Ryan McLaughlin ryanjmclaughlin@gmail.com

  The hardware consists of an Triac to act as an A/C switch and
  an opto-isolator to give us a zero-crossing reference.
  The software uses two interrupts to control dimming of the light.
  The first is a hardware interrupt to detect the zero-cross of
  the AC sine wave, the second is software based and always running
  at 1/128 of the AC wave speed. After the zero-cross is detected
  the function check to make sure the proper dimming level has been
  reached and the light is turned on mid-wave, only providing
  partial current and therefore dimming our AC load.

  Thanks to http://www.andrewkilpatrick.org/blog/?page_id=445
  and http://www.hoelscher-hi.de/hendrik/english/dimmer.htm

*/
#include <Wire.h>
#include <TimerOne.h> // Avaiable from http://www.arduino.cc/playground/Code/Timer1

#define I2C_MSG_IN_SIZE 4
#define I2C_MSG_OUT_SIZE 4

#define CMD_DIGITAL_WRITE 1
#define CMD_DIGITAL_READ 2
#define CMD_ANALOG_WRITE 3
#define CMD_ANALOG_READ 4
volatile uint8_t sendBuffer[I2C_MSG_OUT_SIZE];
int ESPrms = 0; // value to be sent over I2C to ESP

int triac;
int dim;

volatile int i = 0; // Variable to use as a counter
volatile boolean zero_cross = 0; // Boolean to store a "switch" to tell us if we have crossed zero

int triac1; // Output to Opto Triac1
int triac2; // Output to Opto Triac2
int triac3; // Output to Opto Triac3
int triac4; // Output to Opto Triac4
int triac5; // Output to Opto Triac5
int triac6; // Output to Opto Triac6

int LED = 10; // LED for testing

int dim1; // Dimming level (0-128) 0 = on, 128 = 0ff
int dim2; // Dimming level (0-128) 0 = on, 128 = 0ff
int dim3; // Dimming level (0-128) 0 = on, 128 = 0ff
int dim4; // Dimming level (0-128) 0 = on, 128 = 0ff
int dim5; // Dimming level (0-128) 0 = on, 128 = 0ff
int dim6; // Dimming level (0-128) 0 = on, 128 = 0ff

int freqStep = 65; // or 78 based on power supply : This is the delay-per-brightness step in microseconds.
// It is calculated based on the frequency of your voltage supply (50Hz or 60Hz)
// and the number of brightness steps you want.
//
// The only tricky part is that the chopper circuit chops the AC wave twice per
// cycle, once on the positive half and once at the negative half. This meeans
// the chopping happens at 120Hz for a 60Hz supply or 100Hz for a 50Hz supply.

// To calculate freqStep you divide the length of one full half-wave of the power
// cycle (in microseconds) by the number of brightness steps.
//
// (1000000 uS / 120 Hz) / 128 brightness steps = 65 uS / brightness step or (1000000 uS / 100 Hz) / 128 brightness steps = 78 uS / brightness step
//
// 1000000 us / 120 Hz = 8333 uS, length of one half-wave or 100 Hz =

void setup()
{
  Wire.begin(0x3f);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(9600);

  pinMode(triac1, OUTPUT); // Set the Triac1 pin as output
  pinMode(triac2, OUTPUT); // Set the Triac2 pin as output
  pinMode(triac3, OUTPUT); // Set the Triac3 pin as output
  pinMode(triac4, OUTPUT); // Set the Triac4 pin as output
  pinMode(triac5, OUTPUT); // Set the Triac5 pin as output
  pinMode(triac6, OUTPUT); // Set the Triac6 pin as output */

  pinMode(LED, OUTPUT); // Set the LED pin as output

  attachInterrupt(0, zero_cross_detect, RISING); // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
  Timer1.initialize(freqStep); // Initialize TimerOne library for the freq we need
  Timer1.attachInterrupt(dim_check, freqStep);
  // Use the TimerOne Library to attach an interrupt
  // to the function we use to check to see if it is
  // the right time to fire the triac. This function
  // will now run every freqStep in microseconds.
}

/* ******************************************************************************************************************
  This part in the loop found here: [url] https://forum.arduino.cc/index.php?topic=179541.0 [/url] tnx to dc42
  constant 75.7576 depends on the sensitivity of the ACS712 module
  Sensitivity Min<Typ<Max mV/A
  for 30A: 64< 66< 68 mV/A > constant = 5/.066 = 75.76
  for 10A: 96<100<104 mV/A > constant = 5/.1 = 50.00
  for 5A: 180<185<190 mV/A > constant = 5/.185 = 27.03
*/

const int currentPin = A0; // ADC pin used for ACS712
const unsigned long sampleTime = 100000UL; // sample over 100ms, it is an exact number of cycles for both 50Hz and 60Hz mains
const unsigned long numSamples = 250L; // number of samples 400 microsecond each
const unsigned long sampleInterval = sampleTime / numSamples; // sampling interval, must be longer than then ADC conversion time
int adc_zero = 514; // relative zero of the ACS712 for me 514 was best. 511
int valueRead;

void zero_cross_detect() {
  zero_cross = true; // set the boolean to true to tell our dimming function that a zero cross has occured
  i = 0;
  // since the control pin stays high, the TRIAC won't 'unlatch'
  // when zero-crossing, so I need to put the pins to LOW
  digitalWrite(triac1, LOW);
  digitalWrite(triac2, LOW);
  digitalWrite(triac3, LOW);
  digitalWrite(triac4, LOW);
  digitalWrite(triac5, LOW);
  digitalWrite(triac6, LOW);

  // writing pins is like 10 times faster if
  // we write the register directly
  // instead of using 'digitalWrite'

}

// Turn on the TRIAC at the appropriate time
void dim_check() {
  if (zero_cross == true) {

    if (i >= dim1)  {
      digitalWrite(triac1, HIGH);  // turn on triac1
      i = 0; // reset time step counter
      zero_cross = false; //reset zero cross detection
    }

    if (i >= dim2) {
      digitalWrite(triac2, HIGH);  // turn on triac2

      i = 0; // reset time step counter
      zero_cross = false; //reset zero cross detection
    }


    if (i >= dim3) {
      digitalWrite(triac3, HIGH);  // turn on triac3

      i = 0; // reset time step counter
      zero_cross = false; //reset zero cross detection
    }


    if (i >= dim4) {
      digitalWrite(triac4, HIGH);  // turn on triac4

      i = 0; // reset time step counter
      zero_cross = false; //reset zero cross detection
    }

    if (i >= dim5) {
      digitalWrite(triac5, HIGH); // turn on triac5

      i = 0; // reset time step counter
      zero_cross = false; //reset zero cross detection
    }

    if (i >= dim6) {
      digitalWrite(triac6, HIGH);  // turn on triac6
      i = 0; // reset time step counter
      zero_cross = false; //reset zero cross detection
    }

    else {

      i++; // increment time step counter

    }


  }

}

void loop()
{
  if (triac == 4) {
    dim1 = dim;

  }
  if (triac == 5) {
    dim2 = dim;
  }

  if (triac == 6) {
    dim3 = dim;
  }
  if (triac == 7) {
    dim4 = dim;
  }

  if (triac == 8) {
    dim5 = dim;
  }

  if (triac == 9) {
    dim6 = dim;
  }

  dim_check();

  //dim = dim / 8; // set dimmer value to global integer dim
  //dim = analogRead(POT_pin) / 8; // read dimmer value from potentiometer
  analogWrite(LED, dim); // write dimmer value to the LED, for debugging

  Serial.print("dim = ");
  Serial.print(dim);

  //Serial.print("triac = ");
  //Serial.print(triac);

  Serial.print(" t1 = ");
  Serial.print(triac1);
  Serial.print(" d1 = ");
  Serial.print(dim1);
  Serial.print(" t2 = ");
  Serial.print(triac2);
  Serial.print(" d2 = ");
  Serial.print(dim2);
  Serial.print(" t3 = ");
  Serial.print(triac3);
  Serial.print(" d3 = ");
  Serial.print(dim3);
  Serial.print(" t4 = ");
  Serial.print(triac4);
  Serial.print(" d4 = ");
  Serial.print(dim4);
  Serial.print(" t5 = ");
  Serial.print(triac5);
  Serial.print(" d5 = ");
  Serial.print(dim5);
  Serial.print(" t6 = ");
  Serial.print(triac6);
  Serial.print(" d6 = ");
  Serial.print(dim6);

  // Serial.println(" ESPrms = ");
  // Serial.print(ESPrms);

  delay(10000);
  Serial.print('\n');

}

void receiveEvent(int count)
{
  if (count >= I2C_MSG_IN_SIZE)
  {
    byte cmd = Wire.read();
    byte port = Wire.read();
    int value = Wire.read();
    value += Wire.read() * 256;
    switch (cmd)
    {
      case CMD_DIGITAL_WRITE:
        pinMode(port, OUTPUT);
        digitalWrite(port, value);
        break;
      case CMD_DIGITAL_READ:
        pinMode(port, INPUT_PULLUP);
        clearSendBuffer();
        sendBuffer[0] = digitalRead(port);
        break;
      case CMD_ANALOG_WRITE:
        //analogWrite(port,value);
        triac = (port);
        dim = (value);
        if (port == 4) triac1 = (port);
        if (port == 5) triac2 = (port);
        if (port == 6) triac3 = (port);
        if (port == 7) triac4 = (port);
        if (port == 8) triac5 = (port);
        if (port == 9) triac6 = (port);
        break;

      case CMD_ANALOG_READ:

        if (port <= 4) valueRead = analogRead(port); // port <=4 to read analog value A0,A1,A2,A3 - A4 & A5 are I2C
        if (port > 79) valueRead = ESPrms; // port is any number >4 up to 255
        sendBuffer[0] = valueRead & 0xff;
        sendBuffer[1] = valueRead >> 8;

        break;
    }

  }
}

void clearSendBuffer()
{
  for (byte x = 0; x < sizeof(sendBuffer); x++)
    sendBuffer[x] = 0;
}

void requestEvent()
{
  Wire.write((const uint8_t*)sendBuffer, sizeof(sendBuffer));
}
