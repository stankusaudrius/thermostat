#include <EepromAT24C32.h>
#include <RtcDateTime.h>
#include <RtcDS1307.h>
#include <RtcDS3231.h>
#include <RtcTemperature.h>
#include <RtcUtility.h>
#include <Wire.h> // must be included here so that Arduino library object file references work

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/io.h>

/* example for timer start/stop
#define START_TIMER0 TCCR0B |= (1<<CS0)|(1<<CS2)
#define STOP_TIMER0  TCCR0B &= 0B11111000
#define CLEAR_TIMER0 TCNT0 = 0

 */
#define PIN_REED_SW1 2 //PD2
#define PIN_REED_SW2 3 //PD3
#define INT_REED_SW1 INT0 //PD2
#define INT_REED_SW2 INT1 //PD3
#define PIN_SW1 8 // PB0 
#define PIN_SW2 9 // PB1
#define PCINT_SW1 PCINT0 // PB0
#define PCINT_SW2 PCINT1 // PB1
#define PIN_RTC A0 // PC0
#define PCINT_RTC PCINT8 //PC0
#define PIN_DRIVER_1A 12 // PB4
#define PIN_DRIVER_2A 11 // PB3
#define PIN_RST A1 // PC1
#define PIN_DRIVER_EN 13 // PB5

#define CYCLES_WDT 3
#define CYCLES_FOR_RESET 1000
#define CYCLES_BEFORE_SLEEP 10000

volatile int  f_wdt=1;      // flag to carry out battery check
volatile int  cntWDT = 0;   // Counts WDT wake up cycles
volatile int  cntTimer1Int = 0;

enum          stateMain {stateSleep, stateAwakeManual, stateAwakeRTC};
volatile char stateMain = stateSleep;

unsigned int cntResetCycles_uint16 = 0;
unsigned int cnt_to_sleep = 0;

volatile bool bRST_done = 0;
volatile bool interuptFlagRTC = 0;
volatile bool bReedSwitch1 = 0;
volatile bool bReedSwitch2 = 0;

RtcDS3231<TwoWire> Rtc(Wire);

void setup() {
  // Set all pins to input pullup to save power
  DDRB = 0x00;
  PORTB = 0xff;
  DDRC = 0x00;
  PORTC = 0xff;
  DDRD = 0x00;
  PORTD = 0xff;  
  
  // Initialize required pins
  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);
  pinMode(PIN_REED_SW1, INPUT_PULLUP);
  pinMode(PIN_REED_SW2, INPUT_PULLUP);  
  pinMode(PIN_RTC, INPUT_PULLUP);
  pinMode(PIN_DRIVER_EN, OUTPUT);
  pinMode(PIN_DRIVER_1A, OUTPUT);
  pinMode(PIN_DRIVER_2A, OUTPUT);
 // pinMode(LED_BUILTIN, OUTPUT);    
  pinMode(PIN_RST, OUTPUT);

  digitalWrite(PIN_DRIVER_1A, LOW);
  digitalWrite(PIN_DRIVER_2A, LOW);
  digitalWrite(PIN_DRIVER_EN, LOW);
  digitalWrite(PIN_RST, HIGH);  
  
  setupWatchDogTimer();
  
  Rtc.Begin();

  //Set DS3231 to compile time
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  Rtc.SetDateTime(compiled + 10);
  Rtc.SetIsRunning(true);
      
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth); 
  RtcDateTime now = Rtc.GetDateTime();
 
  // Alarm 1 set to trigger every day when 
  // the hours, minutes, and seconds match
  /*
  RtcDateTime alarmTime = now + 10; // into the future
  DS3231AlarmOne alarm1(
          alarmTime.Day(),
          alarmTime.Hour(),
          alarmTime.Minute(), 
          alarmTime.Second(),
          DS3231AlarmOneControl_HoursMinutesSecondsMatch);
          */
  DS3231AlarmOne alarm1(
          0,
          0,
          0, 
          30,
          DS3231AlarmOneControl_SecondsMatch   );
          //DS3231AlarmOneControl_HoursMinutesSecondsMatch);
          //to_do change to DS3231AlarmOneControl_HoursMinutesSecondsMatch
  Rtc.SetAlarmOne(alarm1);

  // Alarm 2 set to trigger at the top of the minute
  DS3231AlarmTwo alarm2(
          0,
          0,
          0, 
          DS3231AlarmTwoControl_OncePerMinute);
          //to_do change to DS3231AlarmTwoControl_HoursMinutesMatch          
  Rtc.SetAlarmTwo(alarm2);
  
  // throw away any old alarm state before we ran
  Rtc.LatchAlarmsTriggeredFlags();
  
  // setup external interupt 
  //attachInterrupt(PCINT_RTC, InteruptServiceRoutine, FALLING);
  EICRA |= (1 << ISC11) | (1<<ISC01);    // set INT0 and INT1 to trigger on falling edge
  EIMSK |= (1 << INT0) | (1<<INT1);     // Turns on INT0 an INT1
  PCICR |= (1<<PCIE1) | (1<<PCIE0);    // turns on pin change interrupts
  PCMSK0 = (1<<PCINT_SW1) | (1<<PCINT_SW2);    // turn on PCINT interrupts on pins PB0, PB1
  PCMSK1 = (1<<PCINT_RTC);      // turn on PCINT interrupts on pins PC0
  sei();   
}

void loop() {
  switch (stateMain)
  {
    case stateSleep:
      // Wait until the watchdog has triggered a wake up.
      if(f_wdt != 1) 
      {
        digitalWrite(PIN_DRIVER_1A, LOW);
        digitalWrite(PIN_DRIVER_2A, LOW);
        digitalWrite(PIN_DRIVER_EN, LOW);        
        digitalWrite(PIN_RST, HIGH);  
        //digitalWrite(LED_BUILTIN, LOW);   
        enterSleep();
        // Continue here after sleep
        
        //digitalWrite(LED_BUILTIN, HIGH); 

      }
      // Maybe check battery here or read some sensor
      f_wdt = 0;
      break;
    
    case stateAwakeManual:

      if (!digitalRead(PIN_SW1))
      {
        digitalWrite(PIN_DRIVER_1A, HIGH);
      } else
      {
        digitalWrite(PIN_DRIVER_1A, LOW);
      }
    
      if (!digitalRead(PIN_SW2))
      {
        digitalWrite(PIN_DRIVER_2A, HIGH);
      } else
      {
        digitalWrite(PIN_DRIVER_2A, LOW);
      }
    
      if (cntResetCycles_uint16 <= CYCLES_FOR_RESET)
      {
        cntResetCycles_uint16++;
      }
  
      if ((cntResetCycles_uint16 > CYCLES_FOR_RESET))
      {
        cntResetCycles_uint16 = 0;
        digitalWrite(PIN_RST, HIGH);
        bRST_done = 1;
        
      }
      if (bRST_done && cnt_to_sleep <= CYCLES_BEFORE_SLEEP)
      {
        cnt_to_sleep++;
      }
      
      if ((cnt_to_sleep > CYCLES_BEFORE_SLEEP) && digitalRead(PIN_SW1) && digitalRead(PIN_SW2))
      {
        cntResetCycles_uint16 = 0;
        bRST_done = 0;
        stateMain = stateSleep;  
       
      }

      break;
      
      case stateAwakeRTC:      
      
      //getTime

      // if time hour == morning
            // turn left
        // else if hour == evening
            // turn right

        if (Alarmed())
        {

            //stateMain = stateSleep;                                     
          
        }
        if (bReedSwitch1)
        {
          bReedSwitch1 = 0;
          stateMain = stateSleep;
        }
        if (bReedSwitch2)
        {
          bReedSwitch2 = 0;
          stateMain = stateSleep;
        }
      break;
    
      default:
        ;
      break;

  }
}

// Watchdog Interrupt Service. This is executed when watchdog timed out.
ISR(WDT_vect) {

  if(f_wdt == 0) {
    if (cntWDT < CYCLES_WDT)
    {
      cntWDT++;
    } else
    {
      f_wdt=1;
      cntWDT = 0;      
    }
  }
}

// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer() {
  // The MCU Status Register (MCUSR) is used to tell the cause of the last
  // reset, such as brown-out reset, watchdog reset, etc.
  // NOTE: for security reasons, there is a timed sequence for clearing the
  // WDE and changing the time-out configuration. If you don't use this
  // sequence properly, you'll get unexpected results.

  // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
  MCUSR &= ~(1<<WDRF);

  // Configure the Watchdog timer Control Register (WDTCSR)
  // The WDTCSR is used for configuring the time-out, mode of operation, etc

  // In order to change WDE or the pre-scaler, we need to set WDCE (This will
  // allow updates for 4 clock cycles).

  // Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
  // bit must be set in order to change WDE or the watchdog pre-scalers.
  // Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
  // clock cycles then it will be reset by hardware.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /**
   *  Setting the watchdog pre-scaler value with VCC = 5.0V and 16mHZ
   *  WDP3 WDP2 WDP1 WDP0 | Number of WDT | Typical Time-out at Oscillator Cycles
   *  0    0    0    0    |   2K cycles   | 16 ms
   *  0    0    0    1    |   4K cycles   | 32 ms
   *  0    0    1    0    |   8K cycles   | 64 ms
   *  0    0    1    1    |  16K cycles   | 0.125 s
   *  0    1    0    0    |  32K cycles   | 0.25 s
   *  0    1    0    1    |  64K cycles   | 0.5 s
   *  0    1    1    0    |  128K cycles  | 1.0 s
   *  0    1    1    1    |  256K cycles  | 2.0 s
   *  1    0    0    0    |  512K cycles  | 4.0 s
   *  1    0    0    1    | 1024K cycles  | 8.0 s
  */
  WDTCSR  = (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
  // Enable the WD interrupt (note: no reset).
  WDTCSR |= _BV(WDIE);
}

// Enters the arduino into sleep mode.
void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  //PCICR |= (1<<PCIE0);// | (1<<PCIE1);    // turns on pin change interrupts
  //PCMSK0 = 0b00000111;    // turn on interrupts on pins PB0, PB1, &amp;amp; PB4
  //PCMSK1 = 0b00000001;
  //sei();                 // enables interrupts
  // Enter sleep mode.
  EIFR = 0xFF;
  sleep_mode();

  // The program will continue from here after the WDT timeout
  //cli();                                  // Disable interrupts
  
  //PCMSK0 &=~ (1<<PCINT0);
  //PCMSK0 &=~ (1<<PCINT1);
  //PCMSK0 &=~ (1<<PCINT2);  
  //PCMSK1 &=~ (1<<PCINT8);  
  // First thing to do is disable sleep.
  sleep_disable();

  // Re-enable the peripherals.
  power_all_enable();
}

ISR(PCINT0_vect)
{
  digitalWrite(PIN_RST, LOW);
  digitalWrite(PIN_DRIVER_EN, HIGH);
  bRST_done = 0;
  stateMain = stateAwakeManual;

}

ISR(PCINT1_vect)
{
    interuptFlagRTC = true;
    stateMain = stateAwakeRTC;
}

bool Alarmed()
{
    bool wasAlarmed = false;
    if (interuptFlagRTC)  // check our flag that gets sets in the interupt
    {
        wasAlarmed = true;
        interuptFlagRTC = false; // reset the flag
        
        // this gives us which alarms triggered and
        // then allows for others to trigger again
        DS3231AlarmFlag flag = Rtc.LatchAlarmsTriggeredFlags();

        if (flag & DS3231AlarmFlag_Alarm1)
        {
          Motor_right();
          //delay(2000);
        }
        if (flag & DS3231AlarmFlag_Alarm2)
        {
          Motor_left();
          //delay(2000); 
        }
    }
    return wasAlarmed;
}

void Motor_right(void)
{
  digitalWrite(PIN_DRIVER_EN, HIGH);
  digitalWrite(PIN_DRIVER_1A, LOW);
  digitalWrite(PIN_DRIVER_2A, HIGH);
    ICR1 = 11718;
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << WGM13);
    // Mode 4, CTC on OCR1A

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS12) | (1 << CS10);
}

void Motor_left(void)
{
  digitalWrite(PIN_DRIVER_EN, HIGH);
  digitalWrite(PIN_DRIVER_1A, HIGH);
  digitalWrite(PIN_DRIVER_2A, LOW);
    ICR1 = 11718;
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << WGM13);
    // Mode 4, CTC on OCR1A

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS12) | (1 << CS10); 
}

void Motor_stop(void)
{
  digitalWrite(PIN_DRIVER_EN, LOW);
  digitalWrite(PIN_DRIVER_1A, LOW);
  digitalWrite(PIN_DRIVER_2A, LOW);
  TCCR1B &= ~(1 << CS12);
  TCCR1B &=~ (1 << CS10);
}

ISR (INT0_vect)
{
  if (PORTB & (1<<PC5))
  {
    Motor_stop();
    bReedSwitch1 = 1;
  }
}
ISR (INT1_vect)
{
  if (PORTB & (1<<PC5))
  {
    Motor_stop();
    bReedSwitch2 = 1;
  }
}

ISR (TIMER1_COMPA_vect)
{
  if (cntTimer1Int <= 10)
  {
    cntTimer1Int++;
  } else
  {
    cntTimer1Int = 0;

    Motor_stop();
    stateMain = stateSleep;
  }
    // action to be done every 1 sec
}
