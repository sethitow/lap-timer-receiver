#include "config.h"

volatile bool pulseCompleted;
volatile bool isRisingEdge;
volatile int startEdge;
volatile int pulseTime;

int longPulses;
int shortPulses;


void IRInterrupt(void) {

  isRisingEdge = digitalRead(IN_PIN);

  if (isRisingEdge) {
    startEdge = micros();
  }
  else {
    pulseTime = micros() - startEdge;
    pulseCompleted = true;
  }

}

void setup() {

#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  Serial.println("Ready...");
#endif


  pulseCompleted = true;
  isRisingEdge = false;
  pulseTime = 0;
  longPulses = 0;
  shortPulses = 0;

  pinMode(IN_PIN, INPUT);
  pinMode(OUT_PIN, OUTPUT);

  digitalWrite(OUT_PIN, HIGH);
  digitalWrite(OUT_PIN, LOW);

//the following sketchyness is done to maintain best practice while using an ATMega based board

#ifdef ATTINY_BUILD_TARGET
  attachInterrupt(IN_PIN, IRInterrupt, CHANGE);
#endif

#ifdef ARDUINO_UNO_BUILD_TARGET
  attachInterrupt(digitalPinToInterrupt(IN_PIN), IRInterrupt, CHANGE);
#endif
}

void loop() {

  if (pulseCompleted) {
    pulseCompleted = false;
    if (pulseTime > SHORT_MIN_PULSE && pulseTime < SHORT_MAX_PULSE) shortPulses++;
    else if (pulseTime > LONG_MIN_PULSE && pulseTime < LONG_MAX_PULSE) longPulses++;

  }

  if (longPulses >= TOKEN_TARGET) {
    int baseID = round(shortPulses / longPulses);
    shortPulses = 0;
    longPulses = 0;

#ifdef SERIAL_DEBUG
    Serial.print("Beacon "); Serial.print(baseID); Serial.println(" Detected");
#endif

    digitalWrite(OUT_PIN, HIGH);

    delay(IGNORE_AFTER_LAP_DURATION);

    digitalWrite(OUT_PIN, LOW);
  }
  else {
    //Serial.println("No Beacon Was Detected, Oh No");
  }

}
