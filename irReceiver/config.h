//Uncomment the desired build target
//#define ATTINY_BUILD_TARGET
#define ARDUINO_UNO_BUILD_TARGET

#define SERIAL_DEBUG //Uncomment to enable serial debugging



#ifdef ARDUINO_UNO_BUILD_TARGET
#define IN_PIN 2 // The digital pin to which the IR receiver is attached. Must support ISR
#define OUT_PIN 3 // The digital pin to output signal
#endif

#ifdef ATTINY_BUILD_TARGET
#define IN_PIN 4 // The digital pin to which the IR receiver is attached. Must support ISR
#define OUT_PIN 3 // The digital pin to output signal
#endif

#define LONG_PULSE 5900 //length of the demodulated long pulse in microseconds
#define SHORT_PULSE 1190 // length of the demodulated short pulse in microseconds

#define TOLERANCE              100

#define TOKEN_TARGET 3

#define IGNORE_AFTER_LAP_DURATION 1000


//Don't change anything below here
#define LONG_MAX_PULSE       (LONG_PULSE + TOLERANCE)
#define LONG_MIN_PULSE       (LONG_PULSE - TOLERANCE)
#define SHORT_MAX_PULSE       (SHORT_PULSE + TOLERANCE)
#define SHORT_MIN_PULSE       (SHORT_PULSE - TOLERANCE)
