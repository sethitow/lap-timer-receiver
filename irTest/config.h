#define IN_PIN 2 // The digital pin to which the IR receiver is attached. Must support ISR

#define LONG_PULSE 5900 //length of the demodulated long pulse in microseconds
#define SHORT_PULSE 1190 // length of the demodulated short pulse in microseconds

#define TOLERANCE              100

#define TOKEN_TARGET 3

#define IGNORE_AFTER_LAP_DURATION 5000


//Don't change anything below here
#define LONG_MAX_PULSE       (LONG_PULSE + TOLERANCE)
#define LONG_MIN_PULSE       (LONG_PULSE - TOLERANCE)
#define SHORT_MAX_PULSE       (SHORT_PULSE + TOLERANCE)
#define SHORT_MIN_PULSE       (SHORT_PULSE - TOLERANCE)
