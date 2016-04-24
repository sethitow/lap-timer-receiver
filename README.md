# Infrared Lap Timer Receiver
An IR receiver for AIM lap timing beacons

##Hardware
The code is written to run on an AVR microcontroller. This was chosen over PIC32 because I already knew how to use interrupts in AVR-C and I didn't want to learn MP-Lab

##Files
###irPulseWidthTest
Measures the width of IR pulses for testing, diagnostics, and analysis

###irReceiver
most functional code

###lapTimerReceiverWithLCD
referance code mooched from https://ucexperiment.wordpress.com/2012/02/18/arduino-ir-lap-timer/

## About the AIM IR Beacon

The AIM IR beacon emits a series of infrared pusles modulated over a 38khz carrier frequency. The beacon emmits pulses of two widths:
* 5900 microseconds -- long pulse
* 1190 microseconds -- short pulse
The pulses are emitted in the following sequence:
1. long plulse
2. short pulse
3. short pulse

This sequency repeats indefintiley from start up to power off. 

## About the Receiver
The reciever detects the long and short pulses. It waits for 3 long pulses (called "tokens" in code) to determine if the ir signal is comming from a beacon or interfearence.

The number of short pululses received between long pulses is convereted to the Base ID, which will hopefull allow for more multiple stations around the track in the future. 