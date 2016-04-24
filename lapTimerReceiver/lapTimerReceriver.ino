/*
  Infrared (IR) Coded-Signal Lap Timer
  8/9/2011 James M. Eli

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  ---

  bill of materials:
    (1) arduino 16MHz pro mini 5v
    (1) micro switch
    (1) panasonic pna4602M ir detector/or equivalent
    (1) 4-digit 7-segment display
    (1) 9v battery connector
    (1) radio shack enclosure
    (1) on/off switch

  7-segment pin outs:
   LCD - Arduino - Port - LCD function
    1  -  A0  -  Port C0  -  Digit 0
    2  -  A1  -  Port C1  -  Digit 1
    6  -  A2  -  Port C2  -  Digit 2
    8  -  A3  -  Port C3  -  Digit 3
    14 -  D1  -  Port D0  -  Segment A
    16 -  D2  -  Port D1  -  Segment B
    13 -  D3  -  Port D2  -  Segment C
    3  -  D4  -  Port D3  -  Segment D
    5  -  D5  -  Port D4  -  Segment E
    11 -  D6  -  Port D5  -  Segment F
    15 -  D7  -  Port D6  -  Segment G

  PNA4602M pin outs:
    IR - Arduino - Port
     1 - D8  - Port B0
     2 - GND - Ground
     3 - VCC - 5V

  Switch pin outs:
    SW - Arduino - Port
     1 - D9  - Port B1
     2 - GND - Ground

  9v Battery pins:
    BT - Arduino
     + - RAW (switched)
     - - GND

  operation:
    1. interrupting power during inital 5 secs. (number of laps in memory is displayed) inverts display.
    2. holding button during power up enters "erase memory mode":
      a. releasing button while '0000' is displayed will erase memory.
      b. releasing button while '8888' is displayed aborts erase function.
    3. pressing button during inital 5 secs. (number of laps in memory is displayed) increments ignore time.
    4. at anytime other time, pressing button steps thru lap time memory.

  to do:
    add lap counter
    add additional beacons (i.e. hot lap...)
*/

//required to make delay functions work
#define F_CPU 16000000UL //16MHz internal osc 
#include <avr/delay.h>
#include <EEPROM.h>  //add eeprom access

//
//definitions
//

//a loop counter used to debounce buttons
#define DEBOUNCE_COUNTER       10
//a simple delay loop counter used inside button routines
#define DELAY_COUNTER          125      //see StepThruLaps() & IncrementIgnore()

//default ignore false/unwanted detector hits duration in ms
#define IGNORE_DEFAULT         60000UL  //60 seconds
#define MIN_IGNORE             10000UL  //10 seconds minimum (should be longer than freeze time)
#define MAX_IGNORE             120000UL //2 minutes maximum
#define IGNORE_STEP            5000UL   //5 seconds per step

//number of seconds the lap time is frozen on the display after crossing start/finish line.
#define FREEZE_DISPLAY_DEFAULT 7000UL   //7 seconds

//eeprom addresses
//used to detect off/on cycle within intial 5 seconds of powerup
#define INVERT_STATUS_ADDRESS  0        //byte
#define STARTUP_FLAG_ADDRESS   1        //byte
//ignore detector time
#define EEPROM_IGNORE_ADDRESS  2        //long
//lap times saved in eeprom
#define LAP_ADDRESS            6        //uint16_t (number of laps stored)
#define LAP_TIME_ADDRESS       8        //uint32_t (each lap ms)

#define TRUE                   1
#define FALSE                  0
#define OK                     1
#define RESET                  0

//maimum # of laps
#define MAX_LAPS               250      //atmega168: (512-6)/4=126 atmega328: (1024-6)/4=254

//software version
#define VERSION                1001

//measured aim beacon pattern [inverted by PNA4602M]:
//high 6ms/low 624us/high 1.2ms/low 624us/high 1.2ms/low 624us [repeat]
//beacon pulse duration +/- tolerance
#define TOLERANCE              36
#define AIM_MAX_PULSE_US       (624 + TOLERANCE)
#define AIM_MIN_PULSE_US       (624 - TOLERANCE)
//we require reciept of 3 valid tokens
#define TOKEN_TARGET           3

//
//declare globals
//

//lap counter
volatile uint16_t lap;
//number of laps stored in eeprom
uint16_t laps_completed;
//first lap of the current session
uint16_t first_lap;
//lap time (in milliseconds)
volatile uint32_t lap_time;
//selectable duration to ignore becaon transmitters
uint32_t ignore_detector_duration;
//length of time dispaly freezes lap time after crossing start/finish line
uint32_t freeze_display_duration;
//stored lap times
uint32_t laps[MAX_LAPS];
//start & finish time [us] of ir beacon pulses
uint32_t start_edge;
uint32_t pulse_us;
//valid ir beacon pulse counter
uint8_t tokens;
//display status [right-side up = FALSE / inverted = TRUE]
uint8_t invert_status;
//hold number to display
uint8_t d[10];
//7-segment digits
const uint8_t *segment, segment_array[20] = {
  //GFEDCBAx right side up, 0, 1, 2... 7, 8, 9
  0b01111110, 0b00001100, 0b10110110, 0b10011110, 0b11001100,
  0b11011010, 0b11111010, 0b00001110, 0b11111110, 0b11011110,
  //inverted, 0, 1, 2... 7, 8, 9
  0b01111110, 0b01100000, 0b10110110, 0b11110010, 0b11101000,
  0b11011010, 0b11011110, 0b01110000, 0b11111110, 0b11111010
};
//millisecond counter
volatile uint32_t my_timer0_millis;

//
//eeprom library routines for read/write int/long
//

//read word from EEPROM
uint16_t EepromRead16(uint16_t address) {
  uint16_t value = word(EEPROM.read(address), EEPROM.read(address + 1));
  return value;
}

//read double word from EEPROM
uint32_t EepromRead32(uint16_t address) {
  //use word read function for reading upper part
  uint32_t dword = EepromRead16(address);
  //shift read word up
  dword = dword << 16;
  // read lower word from EEPROM and OR it into double word
  dword = dword | EepromRead16(address + 2);
  return dword;
}

//write word to EEPROM
void EepromWrite16(uint16_t address, uint16_t value) {
  EEPROM.write(address, highByte(value));
  EEPROM.write(address + 1, lowByte(value));
}

//write double word to EEPROM
void EepromWrite32(uint16_t address, uint32_t value) {
  //truncate upper part and write lower part into EEPROM
  EepromWrite16(address + 2, word(value));
  //shift upper part down
  value = value >> 16;
  //truncate and write
  EepromWrite16(address, word(value));
}

//
//arduino timer routines adjusted for a 1ms interrupt (vice 1.024ms)
//

//returns current microsecond [us] count
uint32_t MyMicros(void) {
  //assumptions here: arduino 168/328 @ 16MHz
  uint32_t m;
  uint8_t t;

  //following not required because we only call this from inside an ISR
  m = my_timer0_millis;
  t = TCNT0;
  if ((TIFR0 & _BV(TOV0)) && (t < 249))
    m++;
  return ((m * 250) + t) * (64 / clockCyclesPerMicrosecond());
}

//
//interrupts
//

//our timer0 interrupt handler replaces arduino handler
ISR(TIMER0_COMPA_vect) {
  //incremented every 1ms
  my_timer0_millis++;
}

//this interrupt is called when the ir detector senses activity (which could be sunlight, etc.)
ISR(PCINT0_vect) {

  if (PINB & (1 << PORTB0)) {
    //high edge change, so calculate ir pulse length
    pulse_us = MyMicros() - start_edge;
    start_edge = 0;

    //look for a pulse time that matches an aim beacon
    if (pulse_us > AIM_MIN_PULSE_US && pulse_us < AIM_MAX_PULSE_US)
      tokens++;
    else
      return;   //unrecognized ir pulse

    //wait until specified number of pulses (or "tokens")
    if (tokens >= TOKEN_TARGET) {
      //a valid beacon was tripped...
      tokens = 0;
      //capture now
      lap_time = my_timer0_millis;
      //kickoff?
      if (lap == (first_lap - 1)) {
        //start lap #1
        lap++;
        //start timer now...
        my_timer0_millis = 0;
        TCNT0 = 0;
        return;
      }

      //ignore unwanted (sector?) beacon
      if (my_timer0_millis < ignore_detector_duration)
        return;

      //increment lap count
      lap++;
      //reset lap timer because we are starting a new lap
      my_timer0_millis = 0;
      TCNT0 = 0;
      return;

    } else
      return;

  } else
    //low edge change, so save time the ir pulse starts
    start_edge = MyMicros();
}

//
// 7-segment display routines
//

//convert uint32_t to array of digits
void ExtractDigits(uint32_t number) {
  uint8_t i;

  i = 0;
  while (number > 0) {
    //pull individual digits from 'number' and stuff into 'd' array
    d[i++] = (int)number % 10;
    number /= 10;
  }
  while (i <= 4) //display needs 4 chars
    d[i++] = 0;  //fill with leading '0's
}

//output number to digit 0, 1, 2 or 3
void _Display(uint8_t number, uint8_t digit) {
  //turn on corresponding digit
  if (invert_status)
    PORTC &= (uint8_t)~(1 << (3 - digit)); //inverted = backwards
  else
    PORTC &= (uint8_t)~(1 << digit);     //right side up
  if (number < 0 || number > 9)
    PORTD = 0b01111110;                  //landing here is an error
  else
    PORTD = *(segment + number + ((invert_status) ? 10 : 0));
}

void DisplayNumber(uint8_t index, uint32_t number) {
  uint8_t i;

  ExtractDigits(number);         //convert UL to byte array
  for (i = 0; i < 4; i++) {      //total time per loop ~5.5ms@16MHz
    _Display(d[index - i], i);   //lightup number
    //this delay controls individual segment on-time/brightness, longer = brighter
    _delay_us(45);               //maximum delay = 768/16 = 48us, hence...
    _delay_us(45);               //we do this twice
    //clear display
    PORTC = PORTC | 0b0011111;   //all digits off
    PORTD = 0;                   //all segments off
  }
  //this delay controls all segments off-time/brightness, shorter = brighter
  _delay_ms(5);
}

//
//button handlers
//

//sequence through lap times stored in laps[] array
void StepThruLaps(uint16_t max_lap) {
  uint16_t  i, button;

  //start at the beginning
  i = 0;

  while (1) {
    DisplayNumber(4, laps[i]);
    if ( !(PINB & (1 << PORTB1)) ) {
      button++;
      if (button > (DELAY_COUNTER * 2)) {
        button = 0;
        i++;
        if (i > max_lap)
          i = 0;
      }
    } else //button released
      break;
  }
}

uint8_t CheckButton(void) {
  static uint8_t button;
  uint8_t  max_lap;

  //debounce button first
  if ( !(PINB & (1 << PORTB1)) ) {
    button++;
    if (button > DEBOUNCE_COUNTER) {
      button = 0;                   //reset debounce counter
      StepThruLaps(laps_completed); //sequence thru stored laps
      return RESET;                 //return & reset timer
    }
  }
  return OK;                        //return & no reset
}

//
//power-up utiliy routines
//

void EraseMemory(void) {
  uint16_t i;
  uint8_t button;
  uint8_t erase_status, button_down;

  erase_status = FALSE;
  button_down = FALSE;
  button = 0;

  //debounce
  do {
    button++;
    if (button > DEBOUNCE_COUNTER) {
      button_down = TRUE;
      break;
    }
  } while ( !(PINB & (1 << PORTB1)) );

  if (button_down == FALSE)
    return; //button released

  do {
    for (i = 0; i < 500; i++) {
      //abort erase?
      erase_status = FALSE;
      if ( (PINB & (1 << PORTB1)) ) {
        button_down = FALSE;
        break;
      }
      DisplayNumber(3, 8888UL);
      _delay_ms(2);
    }

    for (i = 0; i < 500; i++) {
      //confirm erase?
      erase_status = TRUE;
      if ( (PINB & (1 << PORTB1)) ) {
        button_down = FALSE;
        break;
      }
      DisplayNumber(3, 0000UL);
      _delay_ms(2);
    }
  } while (button_down);

  if (erase_status)
    EEPROM.write(LAP_ADDRESS, 0);
}

void IncrementIgnore(void) {
  uint8_t button;

  button = 0;
  while (1) {
    //display ignore_detector_duration
    DisplayNumber(3, ignore_detector_duration / 1000);
    if ( !(PINB & (1 << PORTB1)) ) {
      button++;
      if (button > DELAY_COUNTER) {
        button = 0;
        ignore_detector_duration += IGNORE_STEP;
        //limit?
        if (ignore_detector_duration > MAX_IGNORE)
          ignore_detector_duration = MIN_IGNORE;
        //store duration in eeprom
        EepromWrite32((uint16_t)EEPROM_IGNORE_ADDRESS, ignore_detector_duration);
      }
    } else  //button released
      break;
  } //while
}

//access only during initial 5-second startup period
void CheckSetIgnore(void) {
  static uint8_t button;

  //debounce button first
  if ( !(PINB & (1 << PORTB1)) ) { //same as: if(digitalRead(BUTTON) == 0)
    button++;
    if (button > DEBOUNCE_COUNTER) {
      button = 0;                    //reset debounce counter
      IncrementIgnore();         //increment time
    }
  }
}

//load lap array from eeprom
void InitLaps(void) {
  uint16_t i;

  //init ir detector pulse counter
  start_edge = 0;
  pulse_us = 0;
  tokens = 0;
  //init lap stuff
  laps_completed = 0;
  lap = 0;
  first_lap = 1;
  //any stored laps?
  i = EEPROM.read(LAP_ADDRESS);
  if (i >= 1 && i < MAX_LAPS) {
    uint16_t j;

    for (j = 0; j < i; j++)
      laps[j] = EepromRead32( (uint16_t)(LAP_TIME_ADDRESS + (j * 4)) );
    laps_completed = i;
    lap = i;
    first_lap = i + 1;
  } else
    EEPROM.write(LAP_ADDRESS, (uint16_t)0);  //fix bad eeprom data
}

//extract a bunch of stuff from eeprom
void InitEepromData(void) {
  uint32_t temp;
  uint16_t i;

  //if button is down as power is turned on...
  if ( !(PINB & (1 << PORTB1)) )
    EraseMemory();

  //fetch invert status
  invert_status = (uint8_t)EEPROM.read(INVERT_STATUS_ADDRESS);
  //fetch startup time flag (flag==TRUE if reset within 5 seconds of powerup)
  if (EEPROM.read(STARTUP_FLAG_ADDRESS) == TRUE)
    invert_status ^= 0x01; //toggle display status
  //save status
  EEPROM.write(INVERT_STATUS_ADDRESS, (uint8_t)(invert_status & 0x01));
  //set startup flag
  EEPROM.write(STARTUP_FLAG_ADDRESS, (uint8_t)TRUE);

  InitLaps();
  //get ignore detector time from eeprom memory
  temp = EepromRead32( (uint16_t)EEPROM_IGNORE_ADDRESS );
  //validation
  if ((temp % IGNORE_STEP) != 0)
    temp = IGNORE_DEFAULT;
  if (temp < MIN_IGNORE || temp > MAX_IGNORE)
    ignore_detector_duration = IGNORE_DEFAULT;
  else
    ignore_detector_duration = temp;
  //save
  EepromWrite32((uint16_t)EEPROM_IGNORE_ADDRESS, ignore_detector_duration);

  //show software version during a brief ~5 seconds at power up
  for (i = 0; i < 725; i++) {
    if (laps_completed != 0)
      DisplayNumber(3, (uint32_t)laps_completed);
    else
      DisplayNumber(3, (uint32_t)VERSION);
    CheckSetIgnore();
    _delay_ms(1);
  }
  //clear startup flag
  EEPROM.write(STARTUP_FLAG_ADDRESS, (uint8_t)FALSE);
}

void setup(void) {
  //configure complete ports
  DDRC = 0b0111111;             //set data direction for port C (digit 1, 2, 3, 4 & DP)
  PORTC = 0b0011111;            //initialize all digits off
  DDRD = 0b11111110;            //set data direction for port D (segments A, B... G)
  PORTD = 0b00000000;           //initialize all digits off
  //configure individual pins
  DDRB |= (1 << PORTB1);        //set BUTTON pin as input
  PORTB |= (1 << PORTB1);       //turn pullup resistor on BUTTON pin

  //initialize pointer, determines which segments to turn on for each digit
  segment = &segment_array[0];
  //lenght of time display freezes showing previous lap time
  freeze_display_duration = FREEZE_DISPLAY_DEFAULT;
  //check for inverting display
  InitEepromData();

  //turn off interrupts
  cli();

  //replace arduino timer0 code with our timer
  my_timer0_millis = 0;
  //16Mhz/64 prescale/250 counts = 16000000/64/250 = 1000us (1ms)
  TCCR0A = 0;
  TCCR0A |= (1 << WGM01);          //CTC mode, top=OCR0A, TOV0 set @ max, update immediate
  TCCR0B = 0;
  TCCR0B |= (1 << CS01) | (1 << CS00); //Fcpu/64
  TIMSK0 |= (1 << OCIE0A);         //enable CTC A interrupt
  OCR0A = 249;                     //249 results in a 250 count rollover
  TCNT0 = 0;

  //setup digital pin #8 pin-change interrupt (ir detector)
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  //enable interrupts
  sei();
}

//display nothing until start/finish line is tripped
void Wait(void) {
  while (lap == (first_lap - 1)) {
    DisplayNumber(3, 0UL);
    _delay_ms(2);
    CheckButton();
  }
}

//
//main loop
//
void loop(void) {
  Wait();                                 //display zeros until crossing start/finish line
  while (1) {                             //endless loop

    if (lap > MAX_LAPS)                   //limit the amount of laps we save
      cli();                              //stop timer

    if (CheckButton() == RESET) {         //button pressed?
      InitLaps();                         //start over
      Wait();
    }

    if ((lap > first_lap) && (my_timer0_millis < freeze_display_duration)) {
      DisplayNumber(4, lap_time);         //temporary freeze previous lap time on display

      if (laps_completed != (lap - 1)) {  //has lap been stored?
        laps_completed = lap - 1;
        //store total number of laps in eeprom
        EepromWrite32((uint16_t)(LAP_TIME_ADDRESS + (laps_completed - 1) * 4), lap_time);
        EEPROM.write(LAP_ADDRESS, (uint16_t)laps_completed);
        laps[laps_completed - 1] = lap_time;
      }
    } else
      DisplayNumber(4, my_timer0_millis); //show ticking timer

  } //while
}   //loop
