/*
  CER - Complex Envelope Recorder

  This device is used to record and playback 0-5V envelopes.
  Creative Commons License
  CER by Pantala Labs is licensed
  under a Creative Commons Attribution 4.0 International License.
  Gibran Curtiss Salomao. MAR/2018 - CC-BY-SA
*/

#include <Wire.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;
#define DAC_RESOLUTION (8)

//#include <SparkFun_ADXL345.h>
//ADXL345 adxl = ADXL345();

#define       MAXREADINGS           512
#define       us_TRIGGERDEBOUNCE    200000UL
#define       us_DEFAULTINTERVAL    10000UL
#define       us_MININTERVAL        350UL
#define       us_MAXINTERVAL        40000UL
#define       LDRLOWLIMIT           180     //must be higher than minimum reading
#define       LDRHILIMIT            840     //must be lower than maximumreading
#define       ACCELOFFSET           2048
#define       CVINWAVETYPE          5
#define       LDRINWAVETYPE         6


//digital pins
#define       PLAYTRIGGERPIN        2
#define       RECORDFLAGPIN         3
#define       REPEATMODEPIN         4
#define       LEDRECORDINGPIN       6

//analog pins
#define       WAVETYPEPIN           0
#define       ANALOGLDRINPIN        1
#define       STRETCHPIN            2
#define       ENDPLAYPIN            3
#define       L2CSDA                4
#define       L2CSCL                5
#define       STARTPLAYPIN          6
#define       ANALOGCVINPIN         7

unsigned int  potTableCursor = 0;  //{STARTPLAYPIN, ENDPLAYPIN, STRETCHPIN, WAVETYPE};

unsigned int  waveTable[MAXREADINGS];

unsigned int  recEofCursor         = 0;
//unsigned int recEofCv             = 0;
//unsigned int recEofLdr            = 0;
unsigned int  playCursor           = 0;
unsigned int  startPlayCursor      = 0;
unsigned int  endPlayCursor        = MAXREADINGS - 1;
unsigned int  interval             = 0;
unsigned int  waveType             = 0;

boolean       repeatMode            = false;
boolean       flagRecordNew         = false;
boolean       stateRecording        = false;
boolean       flagStopRecording     = false;
boolean       statePlaying          = false;
boolean       flagStopEnvelope      = false;
boolean       flagStartDac          = false;

unsigned long us_lastRecordTime  = 0UL;
unsigned long us_lastPlayTime    = 0UL;
unsigned long us_lastTriggerTime = 0UL;

unsigned long us_now = micros();

unsigned int  oldCVread        = 0;
unsigned int  reading          = 0;
boolean       interpolate      = false;

boolean debug = true;

#include "sin_512.h"
#include "saw_512.h"
#include "ram_512.h"
#include "sqr_512.h"
#include "tri_512.h"

#define NOTENOUGHTVOLTAGE 1015

void setup()
{
  pinMode(PLAYTRIGGERPIN,   INPUT);
  attachInterrupt(digitalPinToInterrupt(PLAYTRIGGERPIN), int_externalTrigger,   RISING);
  pinMode(RECORDFLAGPIN,    INPUT);
  pinMode(REPEATMODEPIN,    INPUT);
  pinMode(LEDRECORDINGPIN,  OUTPUT);

  if (debug) {
    Serial.begin(9600);
  }
  triTableGen();
  
  dac.begin(0x62);
  setDac(0);
}

void loadWavetable(int index) {
  switch (index) {
    case 0 :
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(sqrwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 1 :
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(triwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 2:
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(rampwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 3:
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(sawwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 4:
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = pgm_read_word(&(sinwave[i]));
      }
      recEofCursor = MAXREADINGS - 1;
      break;
    case 5:
    case 6:
      //cv or ldr in
      for (int i = 0; i < MAXREADINGS; i++) {
        waveTable[i] = 0;
      }
      break;
  }
}

void int_externalTrigger() {
  if ( us_now > (us_lastTriggerTime + us_TRIGGERDEBOUNCE) ) {
    if ( (waveType == CVINWAVETYPE) || (waveType == LDRINWAVETYPE) ) {
      us_lastTriggerTime = us_now;
      //first external trigger to start recording
      if (flagRecordNew) {
        flagRecordNew         = false;
        stateRecording        = true;
        flagStopRecording     = false;
        statePlaying          = false;
        flagStopEnvelope      = false;
        flagStartDac          = true;
        recEofCursor          = 0;
        digitalWrite(LEDRECORDINGPIN, HIGH);
        return;
      }
      //second external trigger to end recording
      if (stateRecording) {
        stateRecording        = false;
        flagStopEnvelope      = false;
        flagStopRecording     = true;
        if (repeatMode) {
          statePlaying        = true;
        }
        flagStopEnvelope      = false;
        interpolate           = true;
        return;
      }
    }
    //else , just play recorded envelope
    statePlaying            = true;
    flagStopEnvelope        = false;
    playCursor              = startPlayCursor;
    interpolate             = true;
  }
}

void loop() {
  us_now = micros();

  //read user inputs
  repeatMode = digitalRead(REPEATMODEPIN);

  //read only one pot / loop
  switch (potTableCursor) {
    case 0:
      startPlayCursor = analogRead(STARTPLAYPIN);
      //prevents not enough voltage on pot
      if (startPlayCursor > 1018) {
        startPlayCursor = NOTENOUGHTVOLTAGE;
      }
      startPlayCursor = map(startPlayCursor, 1023, 0, endPlayCursor, 0);
      break;
    case 1:
      endPlayCursor = analogRead(ENDPLAYPIN);
      //prevents not enough voltage on pot
      if (endPlayCursor > 1018) {
        endPlayCursor = NOTENOUGHTVOLTAGE;
      }
      endPlayCursor = map(endPlayCursor, 1023, 0, startPlayCursor, recEofCursor);
      break;
    case 2:
      reading = analogRead(STRETCHPIN);
      //prevents not enough voltage on pot
      if (reading > 1018) {
        reading = NOTENOUGHTVOLTAGE;
      }
      interval = map(reading, 0, 1023, us_MININTERVAL, us_MAXINTERVAL);
      break;
    case 3:
      reading = analogRead(WAVETYPEPIN);
      reading = map(reading, 0, 1023, 0, 7);
      reading = constrain(reading, 0, 6);
      if (reading != waveType) {
        waveType = reading;
        loadWavetable(waveType);
      }
      break;
  }

  //flags next trigger to record new envelope
  if ( (digitalRead(RECORDFLAGPIN) == HIGH) && ( us_now > (us_lastTriggerTime + us_TRIGGERDEBOUNCE)) ) {
    us_lastTriggerTime = us_now;
    flagRecordNew   = true;
  }

  //stop recording
  if (flagStopRecording) {
    digitalWrite(LEDRECORDINGPIN, LOW);
    flagStopRecording   = false;
    stateRecording      = false;
    playCursor          = startPlayCursor;
    recEofCursor        = recEofCursor - 1;

    //    if (waveType==CVINWAVETYPE) {
    //      for (int i = 0; i <= recEofCursor; i++) {
    //        cvTable[i] = waveTable[i];
    //      }
    //      recEofCv  = recEofCursor;
    //    } else if (waveType==LDRINWAVETYPE) {
    //      for (int i = 0; i <= recEofCursor; i++) {
    //        cvTable[i] = waveTable[i];
    //      }
    //      recEofLdr = recEofCursor;
    //    }

    if (repeatMode) {
      statePlaying = true;
    } else {
      statePlaying = false;
      setDac(0);
    }
  }

  //update recording
  //starts dac if needed
  //  if ( stateRecording && flagStartDac && (recEofCursor == 0) ) {
  //    start_dac();
  //  }
  //if still recording condition and its time to save new sample

  if ( stateRecording && ( micros() >= (us_lastRecordTime + interval) ) ) {
    us_lastRecordTime = micros();                       //save this time
    waveTable[recEofCursor] = readSourceVoltage(recEofCursor, waveType);  //read source voltage from waveType option
    setDac(waveTable[recEofCursor]);                    //send voltage on DAC
    recEofCursor++;                                     //set next rec position
    if (recEofCursor >= MAXREADINGS) {                  //overflow?
      endPlayCursor = recEofCursor;
      flagStopRecording = true;                         //think about to create a button to an ENDLESS recording !!!!!!!!!!!!!!!
    }
  }

  //update playing
  //if still playing condition and its time to play new sample
  if ( statePlaying && ( micros() >= (us_lastPlayTime + interval) ) ) {
    us_lastPlayTime = micros();                     //save this time
    setDac(waveTable[playCursor]);                  //send voltage on DAC
    playCursor++;                                   //set next play position
    interpolate     = true;                         //flags need interpolation
    if (playCursor > endPlayCursor) {              //EOF ?
      playCursor = startPlayCursor;
      if (!repeatMode) {                            //dont rewind
        statePlaying = false;                       //kill transmission
        setDac(0);
      }
    }
  }

  //interpolate voltage
  //if still playing condition and its time to interpolate sample
  if ( (waveType == CVINWAVETYPE) || (waveType == LDRINWAVETYPE)
       && statePlaying
       && interpolate
       && (playCursor < endPlayCursor)
       && ( micros() >= (us_lastPlayTime + (interval / 2)) )
     ) {
    interpolate = false;
    setDac((waveTable[playCursor] + waveTable[playCursor - 1]) / 2); //send interpolated voltage on DAC
  }

  //change reading pot time
  potTableCursor++;
  if (potTableCursor > 3) {
    potTableCursor = 0;
  }
}

//set voltage on DAC
void setDac(int voltage) {
  dac.setVoltage(voltage, false);
}

//read source voltage and keep it between 0 - 1023
int readSourceVoltage(int arrayPosition, int mode) {
  int thisCVread;
  //--------------------------------------------------
  //accelerometer soften and scale Y reading
  //  int x, y, z;
  //  adxl.readAccel(&x, &y, &z);                             //read
  //  thisCVread = ACCELOFFSET + (y * 15);                    //scale

  if ( (mode == CVINWAVETYPE) || (mode == LDRINWAVETYPE) ) {
    if (mode == CVINWAVETYPE) {
      //synthesizer CV in on A(0) , no smootheness
      thisCVread = 4 * analogRead(ANALOGCVINPIN);
    } else if (mode == LDRINWAVETYPE) {
      //--------------------------------------------------
      //LDR soften, scale and invert readings
      thisCVread = analogRead(ANALOGLDRINPIN);                          //read
      thisCVread = constrain(thisCVread, LDRLOWLIMIT, LDRHILIMIT);      //constrain
      thisCVread = map(thisCVread, LDRLOWLIMIT, LDRHILIMIT, 0, 4095);   //scale
      if (arrayPosition == 0) {                                         //set old value
        oldCVread = thisCVread;
      } else {
        oldCVread = waveTable[arrayPosition - 1];
      }
    }
    return thisCVread;
  }
  return 0;
}

void start_dac() {
  //  adxl.powerOn();
  //  adxl.setRangeSetting(8);            // range settings 2g, 4g, 8g or 16g, Higher Values = Wider Range / Lower Values =  More Sensitivity
  //  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to Default: Set to 1 // SPI pins ATMega328: 11, 12 and 13
  //  adxl.setActivityXYZ(0, 1, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  //  adxl.setActivityThreshold(135);     // 62.5mg unit // Inactivity thresholds (0-255), Higher Values = smooth, Lower Values = crispy
  //  adxl.setInactivityXYZ(0, 1, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  //  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  //  adxl.setTimeInactivity(5);          // How many seconds of no activity is inactive?
  //  adxl.setTapDetectionOnXYZ(0, 0, 0); // No taps
}

