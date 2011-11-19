/* -*- compile-command: "/cygdrive/e/Program Files/Atmel/AVR Studio 5.0/AVR ToolChain/bin/make.exe -C default  all && bash -c ./__flash.sh"; -*- */

#include "debug.h"

#include "config.h"


typedef enum { false, true } bool;
typedef enum { low, high } loglvl;

#include "defines.h"

#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "lowlevel.h"


//#define NO_SENSOR_TOGGLE
//#define NO_WATER_ALARM
//#define NO_LED
//#define NO_BUZZ
//#define NO_BUTTON
//#define NO_WATCHDOG

/* sleep mode gets in the way of ADC 
http://www.mikrocontroller.net/topic/42689

 the buzzer makes strange noise because of this
 */
//#define NO_SLEEP_MODE
//#define TESTING

typedef struct {
  uint32_t waitTime, nextTime;
#define te_Clear(te) (((te)->nextTime = 0))
#define te_Init(te) (((te)->nextTime =(millis() + (te)->waitTime)))
} timedEvent;
bool te_Check(timedEvent *te);


typedef struct {
  uint32_t waitOnTime, waitOffTime, nextTime;
  bool state;
#define tt_Clear(tt) (((tt)->nextTime = 0),((tt)->state = false))
} timedToggle;
bool tt_Check(timedToggle *tt);


/* timing */
volatile uint32_t run_time; // process run time in milliseconds
#define millis()  (run_time + 0UL)
#define msec(ms)  ((uint32_t)(ms))     // milliseconds into ms
#define sec(s)    ((uint32_t) msec(s) * 1000UL) // seconds into ms
#define minu(m)   ((uint32_t) sec(m) * 60UL)  //minutes into ms
#define hour(h)   ((uint32_t) minu((h) * 60UL))
#define day(d)    ((uint32_t) hour(d) * 24UL)
#define week(w)   ((uint32_t) day(w) * 7UL)
#define MAX_TIMER_INTVERAL                       day(7)


// configuration / globals
////////////////////////////////////////////////////////////////////////////////////////////////

// configuration (see globals for explanation) /////////
#define           watSensOut_Pin                (C, PORT3) //(D, PORT7)           
#define           watSensIn_Pin                  0 //BB:5;  
#define           watSensIn_Pin_Digi            (C, watSensIn_Pin)
#define           multiButton_Pin               (D, PIN2) //(B, PIN0)  // digital input
#define           alarmLed_Pin                  (D, PORT4) // (B, PORT1)
#define           alarmRelay_Pin                (D, PORT0) // digital output
#define           alarmBuzz_Pin                 (D, PORT5) // (B, PORT2)  // digital output

#define           ALARM_BUZZ_DELAY              hour( 10)              //avoid buzzing at night
#define           ALARM_AUTO_OFF                hour( 12)              //delay the auto-off to make sure its not just temporally below the electrodes 
#define           ALARM_LED_BLINK_BUZZING       msec(150), msec( 200)  //fast blink before alarm reset button is pressed
#define           ALARM_LED_BLINK_A             msec(150), msec(1200)  //slow blink after alarm reset button was pressed
#define           ALARM_LED_BLINK_B             msec( 20), msec(5000)  //very slow blink if water level low
#define           ALARM_BUZZ_INTERVAL           msec(100), msec( 400)
#define           WAT_SENS_TOGGLE                sec( 10),  sec(  10),
#define           WAT_SENS_SAMPLE                sec(  1)



#ifdef TESTING
#define           ALARM_BUZZ_DELAY              sec( 3)
#define           ALARM_AUTO_OFF                sec(30)

#endif


// globals //////////////////////////////////////////

/*
  Water sensor : two metal electrodes shortened by water (Ron ~
  10k..30k). The measure voltage polarity is toggled every xx seconds to
  avoid too much electrolysis toggle interval should be small enough
  to prevent electrolysis from messing up reading out the sensor
*/

// digital output. provides current for water sensor
// symmetric toggling
timedToggle       watSensOut_Interval           = { WAT_SENS_TOGGLE };


// analog input. read out water sensor
// up and down thresholds: {water_detected = (Value < Low || High < Value);}
const uint16_t    watSensIn_ThreshLow           = 350;
const uint16_t    watSensIn_ThreshHigh          = 650;

static bool       watSensIn_Get(void); // returns true, if water is detected

/* Water Level : determined by cumulative water sensor readings

   watLvl_High only changes after a certain number of
   successful read samples. If number of samples for highCount is too
   high, the electrolysis may kick in (But only if the reversing current by
   watSensOut is not working properly). If number of samples for lowCount
   is too high, a connected pump may run dry
*/

// number lows/highs in order required to recognize input change
const unsigned    watLvl_HighRequired            = 4;
const unsigned    watLvl_LowRequired             = 8;
timedEvent        watLvl_SampleInterval          = { WAT_SENS_SAMPLE };                  

bool              watLvl_High;
static void       watLvl_Check(void);

/* Multi Button : user button for battery test, turning off alarm-buzzer, reseting
   alarm-LED */
timedEvent        multiButton_SampleInterval      = { msec(100) };

bool              multiButton_State;
#define           multiButton_PinGet() (multiButton_State = (GETPIN(multiButton_Pin) == 0))
void              multiButton_Check(void);

timedEvent        alarm_AutoOff                    = { ALARM_AUTO_OFF };


/* Alarm LED : signals water level too high.  Alarm goes on when water level
   too high, and stays on until multi-button is pressed *while* water
   level is back to normal */
timedToggle       alarmLed_Blink                   = { ALARM_LED_BLINK_A };  // blink intervals
timedToggle       alarmLed_Blink_2                 = { ALARM_LED_BLINK_B };  // blink intervals
timedToggle       alarmLed_Blink_3                 = { ALARM_LED_BLINK_BUZZING };  // blink intervals

bool              alarmLed_State;
static void       alarmLed_State_Set(bool on);
                      
/* Alarm Buzzer : signals water level too high Alarm activates if
   water level too high, and stays on until multi-button is pressed */

timedToggle       alarmBuzz_Interval               = { ALARM_BUZZ_INTERVAL  };  // buzz intervals
timedEvent        alarmBuzz_Delay                  = { ALARM_BUZZ_DELAY };
typedef enum { st_off, st_on, st_delayed } alarmBuzz_State_T; 
alarmBuzz_State_T              alarmBuzz_State;
static void       alarmBuzz_State_Set(alarmBuzz_State_T  on);


/* Alarm Relay : relay to turn on/off a pump.  Alarm activates if
   water level too high. Turns off if water level is back to
   normal. */

bool              alarmRelay_State;
static void       alarmRelay_State_Set(bool on);


// Battery/Function Test 
#define           testBatt_State                     (!alarmLed_State && multiButton_State)






// timer related code
////////////////////////////////////////////////////////////////////////////////

int
compareTimes(uint32_t a, uint32_t b) {
	if (a == b) return 0;
	if (a < b && b - a < MAX_TIMER_INTVERAL) return -1;  
	if (a > b && a - b > MAX_TIMER_INTVERAL) return -1; 
	return 1;
}

bool 
tt_Check(timedToggle *tt) {
  if (compareTimes(millis(), tt->nextTime) < 0)
   return false;

  tt->state = !tt->state;
  tt->nextTime += (tt->state ? tt->waitOnTime : tt->waitOffTime);
  return true;
}


bool 
te_Check(timedEvent *te) {
  if (compareTimes(millis(), te->nextTime) < 0)
    return false;

  te->nextTime += te->waitTime;
  return true;
}


// input related code
////////////////////////////////////////////////////////////////////////////////////


static bool 
watSensIn_Get() {

#ifdef SIMU
  return true;
#endif
  uint16_t watSensIn_Value = analogRead(watSensIn_Pin);
  return ((watSensIn_Value < watSensIn_ThreshLow) || (watSensIn_ThreshHigh < watSensIn_Value));
}


static void 
watLvl_Check() {
  static uint8_t           watLvl_HighCount;
  static uint8_t           watLvl_LowCount;


  if (watSensIn_Get()) {
    DEBUG_TRACE("watSense sample: in water");
    ++watLvl_HighCount;
    watLvl_LowCount = 0;
  } else {
    DEBUG_TRACE("watSense sample: no water");
    watLvl_HighCount = 0;
    ++watLvl_LowCount;
  }
        
  if (!(watLvl_HighCount < watLvl_HighRequired)) {
    watLvl_High = true;
    watLvl_HighCount = watLvl_LowCount = 0; 
  }

  if (!(watLvl_LowCount < watLvl_LowRequired)) {
    watLvl_High = false;
    watLvl_HighCount = watLvl_LowCount = 0; 
  }
}

// output related code
////////////////////////////////////////////////////////////////////////////////////
static void
alarmLed_State_Set(bool on)
{ 
  alarmLed_State = on;
  tt_Clear(&alarmLed_Blink);
  tt_Clear(&alarmLed_Blink_2);
  tt_Clear(&alarmLed_Blink_3);
}

static void  
alarmBuzz_State_Set(alarmBuzz_State_T state)
{ 
  alarmBuzz_State = state;
  tt_Clear(&alarmBuzz_Interval);
  te_Init(&alarmBuzz_Delay);
}



static void 
alarmRelay_State_Set(bool on) {
  if (alarmRelay_State == on)
    return;
  alarmRelay_State = on;
  PUTPIN(alarmRelay_Pin, on);
}



// interrupt handlers
//////////////////////////////////////////////////////////////////////////
ISR(TIMER0_OVF_vect) {
  run_time  += TIMER0_OVF_MS;
}


// main loop
//////////////////////////////////////////////////////////////////////////
void
main_loop() {

#ifndef NO_WATCHDOG


#endif


  // read input
  //////////////

#ifndef NO_WATER_ALARM  
  if (te_Check(&watLvl_SampleInterval)) {
    watLvl_Check();
    // Relay directly follows water level  
    alarmRelay_State_Set(watLvl_High);
  }
#endif


#ifndef NO_BUTTON
  if (te_Check(&multiButton_SampleInterval)) {
    multiButton_PinGet();
  } 
#endif


  // process input
  ////////////////
  {
    bool outAlarmLed = false;
    bool outAlarmBuzz = false;
    bool outTest = false;

    if (watLvl_High)
      te_Init(&alarm_AutoOff);
    else
      if (te_Check(&alarm_AutoOff))
	{
	  alarmLed_State_Set(false);
	  alarmBuzz_State_Set(st_off);
	}

#ifndef NO_LED
    if (watLvl_High && !alarmLed_State) {
      alarmLed_State_Set(true);
      alarmBuzz_State_Set(st_delayed);
    }

    if (alarmLed_State && multiButton_State && !watLvl_High)
      alarmLed_State_Set(false);
     
    if (alarmLed_State) {
      tt_Check(&alarmLed_Blink);
      tt_Check(&alarmLed_Blink_2);
      tt_Check(&alarmLed_Blink_3);
      if ((alarmLed_Blink_3.state && watLvl_High &&  alarmBuzz_State != st_off) ||
	  (alarmLed_Blink.state && watLvl_High && alarmBuzz_State == st_off) ||
	  (alarmLed_Blink_2.state && !watLvl_High))
        outAlarmLed = true;
    }
#endif

#ifndef NO_BUZZ
    if (alarmBuzz_State && multiButton_State)
      alarmBuzz_State_Set(st_off);

    switch (alarmBuzz_State) {

    case st_on:
    if (multiButton_State)
      alarmBuzz_State_Set(st_off);

      tt_Check(&alarmBuzz_Interval);
      if (alarmBuzz_Interval.state) {
        outAlarmBuzz = true;
      }
      break;

    case st_delayed:
	    if (multiButton_State)
      alarmBuzz_State_Set(st_off);

      if (te_Check(&alarmBuzz_Delay))
	alarmBuzz_State_Set(st_on);
      break;

    default:
      break;
      }
#endif

#ifndef NO_BATT_TEST
    if (testBatt_State) {
      outTest = true;
    }
#endif


    // write output
    ///////////////
    PUTPIN(alarmLed_Pin, !(outAlarmLed || outTest));
    PUTPIN(alarmBuzz_Pin, (outAlarmBuzz || outTest));

#ifndef NO_SENSOR_TOGGLE  
    if (tt_Check(&watSensOut_Interval)) {
      PUTPIN(watSensOut_Pin, watSensOut_Interval.state);
    }
#endif
  }
     
}


// initialize hardware
//////////////////////////////////////////////////////////////////////////////
static void 
init_ports() 
{
  DEBUG_ENABLE(9600);

  // avoid floating not connected pins by activating internal pull up resistors
  // (enable them on all pins for now, and disable them later for connected pins)
  PORTB = 0xff;
  PORTC = 0xff;
  PORTD = 0xff;

  CLRPIN(watSensIn_Pin_Digi);
    

  SET_DDR_OUT(watSensOut_Pin);

#ifndef SIMU
  analogReference(DEFAULT);
#endif

  SET_DDR_OUT(alarmLed_Pin);
  alarmLed_State_Set(alarmLed_State);

  SET_DDR_OUT(alarmBuzz_Pin);
  alarmBuzz_State_Set(alarmBuzz_State);

  SET_DDR_OUT(alarmRelay_Pin);
  alarmRelay_State_Set(alarmRelay_State);

  SET_DDR_IN(multiButton_Pin);
  SETPIN(multiButton_Pin); // enable pull up resistor for input  

  DEBUG_TRACE("leaving setup()");
}

  
static void 
init_tick_timer() {

#if (TIMER0_OVF_MS == 8) && (F_CPU == 8000000UL)
  /* 8ms tick interrupt */
  TCNT0 = 6;
  TCCR0 = (1<<CS02);
  TIMSK = (1<<TOV0);
#else
#error "not implemented"
#endif
}


int
main() {

	
#if 1
   // force timer variable overrun for testing purpose
	run_time = (uint32_t)~0 - minu(5);
#endif	

#ifndef SIMU
  init_ports();
  init_tick_timer();
#endif

#ifndef NO_SLEEP_MODE
  set_sleep_mode(SLEEP_MODE_IDLE);
#endif


#ifndef NO_WATCHDOG
  wdt_enable (WDTO_1S);
#endif

  sei();
  
  for(;;) {

#ifndef NO_SLEEP_MODE
    if(!alarmBuzz_State) {
      sleep_mode();
    }
#endif
    main_loop();

#ifndef NO_WATCHDOG
    wdt_reset();
#endif

   }
 
}

