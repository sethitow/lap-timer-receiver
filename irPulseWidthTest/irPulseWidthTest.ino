/*
  IR Capture
  Copyright 2012, all rights reserved.
  James M. Eli
  1/14/2012
 
  project parts:
    (1) arduino 16MHz
    (1) 38KHz IR receiver
    (1) breadboard
    (3) wires
 
  IR - Arduino
  SIG - D2
  GND - GND
  VCC - VCC
*/
 
//definitions
#define IN_PIN 2
#define MAX_CAPTURE 100
 
//globals
uint32_t now, start, capture[MAX_CAPTURE];
volatile bool flag_complete;
uint8_t i;

//interrupt fires on ir event (rise/fall)
void IRInterrupt(void) {
  now = micros();
  capture[i++] = now - start;
  start = now;
  if (i >= MAX_CAPTURE) {
    detachInterrupt(IN_PIN);
    flag_complete = true;
  }
}
 
void setup(void) {
  flag_complete = false;
  start = 0;
  now = 0;
  i = 0;
  Serial.begin(9600);
  Serial.println("Ready to attach.");
  attachInterrupt(digitalPinToInterrupt(IN_PIN), IRInterrupt, CHANGE);
  Serial.println("Ready to capture.");
}
 
void loop(void) {
  while (1) {
    if (flag_complete) {
      for (i=0; i<MAX_CAPTURE; i++) {        
        Serial.println(capture[i]);
        flag_complete = false;
      }
    }
  }
}
