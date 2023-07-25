#include <Arduino.h>
/* C:\Users\Andrzej\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6\cores\arduino\Arduino.h */
#include "mega2560.h"
#include "myString.h"

void setupATmegaPorts(int Daybreak) {
  // port A as output (0-3 address lines; 4 - CSADC; 5 - Elevator direction; 6 - Arm direction; 7 - Arm enable)
  DDRA = 0xFF;
  PORTA = 0xF0;  // address lines -> LOW; signals -> HIGH
  if (Daybreak == 1150) {
    digitalWrite(pArmEn, HIGH); // stop arm rotation
    Serial3.println("Setting ATmega2560 ports for Daybreak" + String(Daybreak));
    
    // port B as output (0 - Elevator enable) or input (1 - Elevator direction, 2 - Deck OK, 3 - Has elevator)
    DDRB = 0x01;
    digitalWrite(pElevEn, HIGH); // stop elevator

    // port C as output (0 - alpha/beta, 3 - Platter enable) or input (1 - Platter home, 2 - Platter OK, 4 - Arm OK, 5 - Arm in, 6 - Arm home, 7 - Arm out)
    DDRC = 0x09;
    digitalWrite(pPlatEn, HIGH); // stop platter rotation

    // port G as input (0 - Deck low, 1 - Deck mid, 2 - Deck high)
    DDRG = 0x00;
  } else if (Daybreak == 0) {
    Serial3.println("Setting ATmega2560 ports for Daybrino");

    // port C as output (& gate signals high to close them)
    DDRC = 0xFF;
    PORTC = 0x03; // close gates
  }
  // port F as input/output (data lines)
  DDRF = 0x00;  // initialize as input
  PORTF = 0xFF;  // switch pull-up resistors on
  // port D (0-3) as output for CS signals
  DDRD = DDRD | 0x0F;
  PORTD = PORTD | 0x0F;
  // port L: PL0, PL3, PL4, PL5, PL6, PL7 as output normal high, PL1 as input Input Capture Pin rising edge ICP5, PL2 as input to TCNT7 photon counter
  DDRL = 0xF9;
  PORTL = 0xF9;
  // switch a builtin LED off
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void adcInit(void) {
  //16MHz/128 = 125 kHz the ADC reference clock
  // ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
  //16MHz/16 = 1000 kHz the ADC reference clock at this freq there is ~16 us per sample
  Serial3.println("Setting ATmega2560 ADC's clock freq to 1 MHz and reference voltage to 5 V");
  ADCSRA &= 0xF8;            // clear ADSP bits
  ADCSRA |= (1<<ADPS2);      // set ADSP2, 1 bit
  ADMUX |= (1<<REFS0);       // Set Voltage reference to Avcc (5v)
  ADCSRA |= (1<<ADEN);       // Turn on ADC
  ADCSRA |= (1<<ADSC);       // Do an initial conversion
}

void setupTimer3(void) {
  // initialize Timer3 to generate 1 Hz interrupts
  // https://oscarliang.com/arduino-timer-and-interrupt-tutorial/
  noInterrupts(); // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 3036; // preload timer with 65536 - 16MHz/256/1Hz
  TCCR3B |= (1 << CS12); // 256 prescaler
  TIMSK3 |= (1 << TOIE3); // enable timer overflow interrupt
  interrupts(); // enable all interrupts
}

/// @brief interrupt service routine that wraps a user defined function supplied by attachInterrupt
/// @param  interrupt vector
ISR(TIMER3_OVF_vect) {
  // https://oscarliang.com/arduino-timer-and-interrupt-tutorial/
  TCNT3 = 3036; // preload timer
  Serial3.println(" status: " + statusToStr(millis()/1000.0 + 0.5));
}
