#include "config.h"

volatile bool pulseCompleted;
volatile bool isRisingEdge;
volatile int startEdge;
volatile int pulseTime;

int longPulses;
int shortPulses;


void IRInterrupt(void) {

  isRisingEdge = digitalRead(IN_PIN);

  if(isRisingEdge){
    startEdge = micros();
  }
  else {
    pulseTime = micros() - startEdge;
    pulseCompleted = true;
  }
    
}

void setup() {
  Serial.begin(9600);
  Serial.println("Ready...");

  pulseCompleted = true;
  isRisingEdge = false;
  pulseTime = 0;
  longPulses = 0;
  shortPulses = 0;

  pinMode(IN_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(IN_PIN), IRInterrupt, CHANGE);

}

void loop() {
  
  if (pulseCompleted){
    pulseCompleted = false;
    if (pulseTime > SHORT_MIN_PULSE && pulseTime < SHORT_MAX_PULSE) shortPulses++;
    else if (pulseTime > LONG_MIN_PULSE && pulseTime < LONG_MAX_PULSE) longPulses++;
    
  }

  if (longPulses >= TOKEN_TARGET) {
    int baseID = round(shortPulses/longPulses);
    shortPulses = 0;
    longPulses = 0;
    Serial.print("Beacon "); Serial.print(baseID); Serial.println(" Detected");
    delay(IGNORE_AFTER_LAP_DURATION);
  }
  else{
    //Serial.println("No Beacon Was Detected, Oh No");
  }

}
