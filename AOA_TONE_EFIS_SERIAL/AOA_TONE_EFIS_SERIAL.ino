////////////////////////////////////////////////////
////////////////////////////////////////////////////
// AOA EFIS Serial to audio tone 
// Supports: Dynon EFIS data using a Arduino Due board
// Ver 1.5
// Christopher Jones 6/10/2016
//
// - Read data in from Serial3 on DUE board. (Dynon is at 115200 baud)
// - Serial is used for debugging to computer IDE. (115200 baud)
// - Audio ouput on pin 2
// - If no serial data is detected audio tone turns off and LED1 is off
// - MUTE_AUDIO_UNDER_IAS define is used to only activate audio tones if above a given airspeed
// 

#include <DueTimer.h>         // timer lib functions for using DUE timers and callbacks.
#include <stdint.h>
#include <Arduino.h>

#include <Gaussian.h>         // gaussian lib used for avg out AOA values.
#include <LinkedList.h>       // linked list is also required.
#include <GaussianAverage.h>  // more info at https://github.com/ivanseidel/Gaussian

// Output serial debug infomation. (comment out the following line to turn off serial debug output)
#define SHOW_SERIAL_DEBUG 

// Average out AOA values to give a smoother transtition between values.
//#define USE_AOA_AVERAGE
// set how many previous AOA values we store to create avg and help smooth out the data values.
#define AOA_HISTORY_MAX       5

// set Freq of callback function used to create the pulse tones.
#define FREQ_OF_FUNCTION      100

// set the sample rate of how often to pull the AOA value from the serial stream.
// example 
// 1 = sample every AOA value received from the efis.
// 5 = skip every 5 lines then sample the AOA value.
#define SERIAL_SAMPLE_RATE    3

// Min airspeed in knots before the audio tone is turned on
// This is useful because if your on the ground you don't need a beeping in your ear.
#define MUTE_AUDIO_UNDER_IAS  35

// dynon expected string length
#define DYNON_SERIAL_LEN      53

// Tone Pulse Per Sec (PPS) for high and low tones.
#define HIGH_TONE_PPS_MAX     6.5
#define HIGH_TONE_PPS_MIN     1.5
#define LOW_TONE_PPS_MAX      8.5
#define LOW_TONE_PPS_MIN      1.5


#define TONE_PIN          2     // TIOA0
#define PIN_LED1          13    // internal LED for showing AOA status.
#define PIN_LED2          54    // external LED for showing serial input.
#define TONE1600hz        1600 
#define TONE400hz         400
#define PULSE_TONE        1
#define SOLID_TONE        2
#define TONE_OFF          3

uint8_t toneState = false;
unsigned char toneMode = PULSE_TONE;  // current mode of tone.  PULSE_TONE, SOLID_TONE, or TONE_OFF
boolean highTone = false;             // are we playing high tone or low tone?
uint32_t toneFreq = 0;                // store current freq of tone playing.
float pps = 0;                        // store current PPS of tone (used for debuging) 
int liveAOA;                          // realtime AOA value.
int AOA = 0;                          // avaraged AOA value is stored here.
int lastAOA = 0;                      // save last AOA value here.
int ASI = 0;                          // live Air Speed Indicated
unsigned int cyclesWOSerialData = 0;  // keep track if not serial data is recieved.
GaussianAverage myAverageAOA = GaussianAverage(AOA_HISTORY_MAX);
static Tc *chTC = TC0;
static uint32_t chNo = 0;

// vars for converting AOA scale value to PPS scale value.
int OldRange,  OldValue;
float NewRange, NewValue;

char inChar;                    // store single serial input char here.
#define MAXSIZE 70              // max length of string
char input[MAXSIZE+1];          // buffer for full serial data input string.
int inputPos = 0;               // current postion of serial data.
char tempBuf[50];               // misc char buffer used for debug

void setup() {
  // put your setup code here, to run once:
  pinMode(TONE_PIN, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  // timer callback is used for turning tone on/off.
  Timer4.attachInterrupt(tonePlayHandler);
  Timer4.setFrequency(FREQ_OF_FUNCTION);  // set how often we should run the callback function.
  Timer4.start();

  Serial.begin(115200);   //Init hardware serial port (ouput to computer for debug)
  Serial3.begin(115200);  //Init hardware serial port (input from EFIS)

  configureToneTimer();   //setup timer used for tone
}

// We use our own counter for how often we should pause between tones (pulses per sec PPS)
int cycleCounter = 0;
int cycleCounterResetAt = FREQ_OF_FUNCTION;
void tonePlayHandler(){
  cycleCounter++;
  // check if our counter has reach the reset mark.  if so flip the tone on/off and start again.
  // cycleCounterResetAt is set by the PPS set function.
  if(cycleCounter >= cycleCounterResetAt) {
    toneState = toneState ^ 1;  // flip tone state
    cycleCounter = 0;
  } else {
    return;
  }
  
  if(toneMode==TONE_OFF) {
    setFrequencytone(0);  // if tone off skip the rest.
    return;
  }
  if(toneMode==SOLID_TONE) {  // check for a solid tone.
#ifdef SHOW_SERIAL_DEBUG    
    Serial.println("SOLID TONE");
#endif    
    setFrequencytone(TONE400hz);
    return; // skip the rest
  }

  // cylce tone on/off depending on toneState which is flipped in code above.
  if(toneState) {
     digitalWrite(PIN_LED1, digitalRead(PIN_LED1)^1);  // cycle led on/off
     //sprintf(tempBuf, "handler() AOA:%i ASI:%i tone: %i PPS:%f handlerFreq: %i",AOA,ASI,toneFreq,pps, handlerFreq);
     //Serial.println(tempBuf);
     if(highTone)
        setFrequencytone(TONE1600hz);
     else 
        setFrequencytone(TONE400hz);

  } else {
    setFrequencytone(0);
  }
}

void checkAOA_Dynon() {
  if(ASI <= MUTE_AUDIO_UNDER_IAS) {
#ifdef SHOW_SERIAL_DEBUG    
  // show audio muted and debug info.
  sprintf(tempBuf, "AUDIO MUTED: Airspeed to low. Min:%i ASI:%i",MUTE_AUDIO_UNDER_IAS, ASI);
  Serial.println(tempBuf);
#endif
  }
  
  if(lastAOA == AOA) {
    return; // skip this if the AOA value has not changed.
  }
  
  // check AOA value and set tone and pauses between tones according to 
  if(AOA >= 90) {
    // play 20 pps 1600hz tone
    highTone = true;
    setPPSTone(20);
    toneMode = PULSE_TONE;
  } else if(AOA >= 70) {
    // play 1600hz tone at Pulse Rate 1.5 PPS to 6.2 PPS (depending on AOA value)
    highTone = true;
    OldValue = AOA-69;
    toneMode = PULSE_TONE;
    // scale number using this. http://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
    OldRange = 20 - 1;  //(OldMax - OldMin)  
    NewRange = HIGH_TONE_PPS_MAX - HIGH_TONE_PPS_MIN; // (NewMax - NewMin)  
    NewValue = (((OldValue - 1) * NewRange) / OldRange) + HIGH_TONE_PPS_MIN; //(((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    setPPSTone(NewValue);
  } else if(AOA >= 60) {
    // play a steady 400hz tone
    highTone = false;
    toneMode = SOLID_TONE;
    //setFrequencytone(TONE400hz);    
  } else if(AOA > 20) {
    toneMode = PULSE_TONE;
    highTone = false;
    // play 400hz tone at Pulse Rate 1.5 PPS to 8.2 PPS (depending on AOA value)
    // scale number using this. http://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
    OldValue = AOA-20;
    OldRange = 40 - 1;  //(OldMax - OldMin)  
    NewRange = LOW_TONE_PPS_MAX - LOW_TONE_PPS_MIN; // (NewMax - NewMin)  
    NewValue = (((OldValue - 1) * NewRange) / OldRange) + LOW_TONE_PPS_MAX; //(((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    setPPSTone(NewValue);
  } else {
    toneMode = TONE_OFF;
  }

  lastAOA = AOA;
#ifdef SHOW_SERIAL_DEBUG    
  // show serial debug info.
  sprintf(tempBuf, "AOA:%i Live:%i ASI:%i PPS:%f cycleCounterResetAt: %i",AOA, liveAOA, ASI,pps, cycleCounterResetAt);
  Serial.println(tempBuf);
#endif
}

void setPPSTone(float newPPS) {
  // set PPS by setting cycleCounterResetAt which is used in the tonePlayHandler()
  cycleCounterResetAt = (1/(newPPS*1.5)) * FREQ_OF_FUNCTION;
  pps = newPPS;  // store pps for debug purposes.
}

// counter for how often to cycle led and sample AOA from serial data.
int LedCountDown = SERIAL_SAMPLE_RATE; 
// main loop of app
void loop() {
      // check for serial in from efis.
      while(Serial3.available()) {
        inChar = (char)Serial3.read();
        input[inputPos]=inChar;
        inputPos++;
        cyclesWOSerialData = 0;
        if (inChar == '\n' && inputPos == DYNON_SERIAL_LEN) {  // is EOL?
          // get AOA from dynon efis string.  details of format can be found from dynon pdf manual.
          tempBuf[0] = input[39]; //
          tempBuf[1] = input[40]; //
          tempBuf[2] = '\0'; //term string
          liveAOA = strtol(tempBuf, NULL, 10); //convert to int

          // get AOA value
#ifdef USE_AOA_AVERAGE
          myAverageAOA += liveAOA;            // store and calcute the mean average AOA from previous values.
          AOA = myAverageAOA.process().mean;
#else          
          AOA = liveAOA;  // else just use the live AOA value.
#endif
      
          // get ASI (4 digits) but we only want the first 3 because its in meters per second.
          tempBuf[0] = input[20]; //
          tempBuf[1] = input[21]; //
          tempBuf[2] = input[22]; //
          tempBuf[3] = '\0'; //
          ASI = strtol(tempBuf, NULL, 10) * 1.943; //convert to int (and from m/s to knots)

          // check how often we want to sample the serial data.
          LedCountDown --;
          if(LedCountDown<1) {
             digitalWrite(PIN_LED2, digitalRead(PIN_LED2)^1);  // cycle serial RX led on/off
             LedCountDown = SERIAL_SAMPLE_RATE;
             checkAOA_Dynon();  // check AOA data from dynon and store result into AOA var.
             // TODO: check if we have a different EFIS and create seperate parse function.
          }
          inputPos = 0;  // reset postion to start of buffer string for next round.
        }
        // else safety check, if we went past the number of expected chars then reset to 0.
        if(inputPos>DYNON_SERIAL_LEN) inputPos = 0;
        
      }

      // check if no serial has been sent in a while.
      cyclesWOSerialData++;
      if(cyclesWOSerialData > 55500) {
        toneMode = TONE_OFF;        // stop tone
        cyclesWOSerialData == 0;
        digitalWrite(PIN_LED2, 0);  // no EFIS serial data so turn off led.
#ifdef SHOW_SERIAL_DEBUG    
        Serial.println("NO DATA");
#endif
      } 
}



void configureToneTimer() {
  // Configure TONE_PIN pin as timer output
  // pmc_enable_periph_clk( ID_PIOB ) ;
  int result = PIO_Configure( PIOB,
            PIO_PERIPH_B,
            PIO_PB25B_TIOA0,
            PIO_DEFAULT);

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
 TC_Configure(chTC, chNo,
         TC_CMR_TCCLKS_TIMER_CLOCK4 |
         TC_CMR_WAVE |         // Waveform mode
         TC_CMR_WAVSEL_UP_RC | // Counter running up and reset when equals to RC
         TC_CMR_ACPA_SET |     // RA compare sets TIOA
         TC_CMR_ACPC_CLEAR );  // RC compare clears TIOA
         
  chTC->TC_CHANNEL[chNo].TC_IER=TC_IER_CPCS;  // RC compare interrupt
  chTC->TC_CHANNEL[chNo].TC_IDR=~TC_IER_CPCS;
  //NVIC_EnableIRQ(TC0_IRQn);
}

void setFrequencytone(uint32_t frequency)
{
  
  if(frequency < 20 || frequency > 20000) {
    //Serial.print("cancel tone: ");Serial.println(frequency);
    TC_Stop(chTC, chNo);
    toneFreq = frequency;
    return;
  }

  if(toneFreq == frequency) {
    // if the new frequency is the same as the current freq then don't do anything.
    return;
  }
  
  const uint32_t rc = VARIANT_MCK / 128 / frequency; 
  const uint32_t ra = rc >> 2; // 50% duty cycle 
  const uint32_t rb = ra >> 2; // 20% duty cycle 

  //Serial.print("rc=");Serial.println(rc);
  //Serial.print("ra=");Serial.println(ra);
  //Serial.print("rb=");Serial.println(rb);

  TC_Stop(chTC, chNo);
  TC_SetRC(chTC, chNo, rc);    // set frequency
  TC_SetRA(chTC, chNo, ra);    
  TC_SetRB(chTC, chNo, rb);    
  TC_Start(chTC, chNo);
  toneFreq = frequency;
  
}

