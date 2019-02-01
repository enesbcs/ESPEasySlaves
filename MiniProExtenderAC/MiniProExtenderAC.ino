/*
  AC Light Control

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

/*
  Modified by Mark Chester mark@chesterfamily.org

  to use the AC line frequency (half-period) as a reference point
  and fire the triacs based on that plus a count of dimming steps.
  Tracks the line frequency and adjusts accordingly. Can set up to
  an estimated 512 steps of dimmer resolution.

*/
#include <Wire.h>
#include <TimerOne.h> // http://www.arduino.cc/playground/Code/Timer1

int dim1; // Dimming level (0-128) 0 = on, 128 = 0ff
int dim2; // Dimming level (0-128) 0 = on, 128 = 0ff
int dim3; // Dimming level (0-128) 0 = on, 128 = 0ff
int dim4; // Dimming level (0-128) 0 = on, 128 = 0ff
//int dim5; // Dimming level (0-128) 0 = on, 128 = 0ff

int triac1 = 4; // Output to Opto Triac1
int triac2 = 5; // Output to Opto Triac2
int triac3 = 6; // Output to Opto Triac3
int triac4 = 7; // Output to Opto Triac4

// General
unsigned long int ZeroXTime[4] = {0, 0, 0, 0}; // Timestamp in micros() of the zero crossing interrupts
unsigned long int DimStep; // How many micros() in each step of dimming
unsigned long int AvgPeriod; // The average line voltage period in micros()
unsigned long int PeriodResync = 3000; // Number of milliseconds between line freq measurements
unsigned long int ResetPeriod = PeriodResync; // The timestamp in millis() when we will measure the period again
unsigned long int DimRes = 256; // How many steps of dimmer resolution
volatile unsigned long int DimStepCounter; // For counting Timer1 interrupts
volatile unsigned long int FireTriac[4] = {0, 0, 0, 0}; // When it's OK to fire the triacs, in counts of DimRes
volatile boolean zero_cross = 0; // Tels us we've crossed the zero line
//byte TriacPin[4] = {4,5,6,7}; // Which digital IO pins to use
byte TriacPin[4] = {triac1, triac2, triac3, triac4}; // Which digital IO pins to use

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

void setup() { // Begin setup
  for (byte x = 0; x < sizeof(sendBuffer); x++)
    sendBuffer[x] = 0;
  for (byte x = 0; x < sizeof(EmptBuffer); x++)
    EmptBuffer[x] = 0xff;
  Wire.begin(0x3f);  // Wire.begin(0x7f); // 0x7f is outside valid range as per https://www.totalphase.com/support/articles/200349176-7-bit-8-bit-and-10-bit-I2C-Slave-Addressing
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  attachInterrupt(0, zero_cross_detect, FALLING); // Attach an Interupt to Pin 2 (interupt 0) for Zero Cross Detection
  pinMode(triac1, OUTPUT); // Set the Triac pin as output
  pinMode(triac2, OUTPUT); // Set the Triac pin as output
  pinMode(triac3, OUTPUT); // Set the Triac pin as output
  pinMode(triac4, OUTPUT); // Set the Triac pin as output
  Timer1.initialize(); // Start up the Timer1 timer
  measure_half_period(); // Initially measure the half period
} // End setup

void measure_half_period() {
  zero_cross = 0; // Clearing this here increases the accuracy of the measurement
  byte F = 0; // Frequency counter counter ;)
  while ( F < 4 ) { // This loop takes 4 zero cross samples
    if ( zero_cross ) { // Only run if a zero cross is detected
      ZeroXTime[F] = micros(); // Set the new current zero cross time in micros()
      zero_cross = 0; // Reset zero_cross
    }
    F++; // Bump the counter for the next sample
  } // Now we calc the length of each DimStep
  DimStep = (((ZeroXTime[1] - ZeroXTime[0]) + (ZeroXTime[2] - ZeroXTime[1]) + (ZeroXTime[3] - ZeroXTime[2])) / 3) / DimRes;
  Timer1.attachInterrupt(fire_triacs, DimStep); // (Re)Associate fire_triacs() with the Timer1 interrupt and the latest DimStep period
  ResetPeriod = ResetPeriod + PeriodResync; // Set the next time when we'll measure the half period again
}

void zero_cross_detect() { // function to be fired at the zero crossing
  zero_cross = 1; // set a variable that's picked up later
  DimStepCounter = 0; // Reset the step counter for the next round of triac firings
}

void fire_triacs() { // Called every DimStep (Timer1 interrupt, checks FireTriac[n] and fires if it's time
  if ( FireTriac[0] == DimStepCounter ) { // Is it time to fire?
    digitalWrite(triac1, HIGH); // Fire the Triac mid-phase
    delayMicroseconds(2);
    digitalWrite(triac1, LOW); // Turn off the Triac gate (Triac will not turn off until next zero cross)
  }
  if ( FireTriac[1] == DimStepCounter ) { // Is it time to fire?
    digitalWrite(triac2, HIGH); // Fire the Triac mid-phase
    delayMicroseconds(2);
    digitalWrite(triac2, LOW); // Turn off the Triac gate (Triac will not turn off until next zero cross)
  }
  if ( FireTriac[2] == DimStepCounter ) { // Is it time to fire?
    digitalWrite(triac3, HIGH); // Fire the Triac mid-phase
    delayMicroseconds(2);
    digitalWrite(triac3, LOW); // Turn off the Triac gate (Triac will not turn off until next zero cross)
  }
  if ( FireTriac[3] == DimStepCounter ) { // Is it time to fire?
    digitalWrite(triac4, HIGH); // Fire the Triac mid-phase
    delayMicroseconds(2);
    digitalWrite(triac4, LOW); // Turn off the Triac gate (Triac will not turn off until next zero cross)
  }
  DimStepCounter++; // This counter increments every time fire_triacs runs
}

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
        //analogWrite(port, value);
        if (port == 4) triac1 = (port);
        if (port == 5) triac2 = (port);
        if (port == 6) triac3 = (port);
        if (port == 7) triac4 = (port);
        if (port == 4) dim1 = (value);
        if (port == 5) dim2 = (value);
        if (port == 6) dim3 = (value);
        if (port == 7) dim4 = (value);
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
      sendBuffer[3] = 0x10;               // sketch version
      I2CDataReady = 1;                // sign data ready for requestevent()
    }
    I2CReceived = 0; // sign that remotely requested operation is ended, we are ready for the next one
  }
 
}

void loop() { // Main Loop
  if ( millis() >= ResetPeriod ) { // Measure the half period every PeriodResync milliseconds to prevent drift
    measure_half_period();
  }
  handleI2C();
  FireTriac[0] = (DimRes * dim1) / 1024; // Read input and calc the next triac fire time
  FireTriac[1] = (DimRes * dim2) / 1024; // Read input and calc the next triac fire time
  FireTriac[2] = (DimRes * dim3) / 1024; // Read input and calc the next triac fire time
  FireTriac[3] = (DimRes * dim4) / 1024; // Read input and calc the next triac fire time
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
            I2CReceived = 1;
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
