// regex ^(bool|int|void)((?!-- ok).)*$ finds function definitions not ending with '-- ok'

// COM7 - Serial3 pair does not work for debugging; must use Serial0 for it
// uncomment line below for debugging
// #include "avr8-stub.h"
#include <Globals.h>
#include "mega2560.h"
#include "mega2560setup.h"
#include "myString.h"
#include "Daybreak1150cmds.h"

/*  begin of test routines  */
unsigned long x1, x2, x3, x4, cOVF;

void stopTimer3(void);
void startTimer3(void);
void ADCandDACtest(void);
void testCounter(long unsigned int dTime);
void setRampClock(unsigned int rcTime);
/*  end of test routines  */

void writeRampDiv(unsigned int rampDiv);  // (rampDiv) RAMP-DIV C! -- writes an initial unsigned char value to the ramp clock counter (CSRCK/)

// definitions of functions

void setup() {
  // COM7 - Serial3 pair doesn't work for debugging

  // initialize GDB stub -- uncomment line below for debugging
  // debug_init();

  // Daybreak 1150 reader set-up
  adcInit();  // inits ATmega2560 ADC's
  setupATmegaPorts(1150);  // inits ATmega2560 ports for communication with 1150 hardware
  setUp();  // setup Daybrino

  Serial.begin(115200, SERIAL_8N1);
  // Serial3.begin(9600, SERIAL_7N2);  // may use faster protocol
  Serial3.begin(115200, SERIAL_8N1);
  delay(200);
  // Serial0 is for programming/debugging or connects to any terminal (connections reboot ATmega2560)
  Serial.print("New session started"); Serial.write(0x0D); Serial.write(0x0A);
  Serial.print("  Is FLConsole? "); Serial.print(isFLConsole ? "yes" : "no"); Serial.write(0x0D); Serial.write(0x0A);
  // Serial3 connects PC with running FLConsole (connecting doesn't reboot ATmega2560)
  Serial3.print("New session started"); Serial3.write(0x0D); 
  Serial3.print("  Is FLConsole? "); Serial3.print(isFLConsole ? "yes" : "no"); Serial3.write(0x0D); 
  setupCounter();
  cOVF = 0;
  startCounter();
}

// use special command 'NF ' to switch Daybrino output to Arduino IDE terminal 

void loop() {
  // TODO: increase FLASH-COUNTER and set/reset FLASH accordingly
  // CHECK-ALL  - check fault conditions

  if (RampSeg != rseg_OSL) {
    if (dataReady) {
      // if (isFLConsole) 
      sendData();
    }
  } else {
    OSLsendIfDataWaiting();  // OSL-SEND-IF-DATA-WAITING
  }

  extractCommand();
  if (commandReady) {
    if ((Command.length() > 1) && isNumericChar(Command[1])) Command = Command.substring(0, 1) + " " + Command.substring(1);
    else if (Command.length() == 1) Command.concat(" ");
    // Serial3.println("extracted command: '" + Command + "'");
    commandReady = false;
    processCmd(Command);
  }
  
  // DISP-STATUS
  rampTest(RampSeg);
}


/*  begin of definitions of test routines  */
void stopTimer3(void) {  // not Daybreak -- ok
  noInterrupts();           // disable all interrupts
  TIMSK3 &= ~(1 << TOIE3);  // disable timer overflow interrupt
  interrupts();             // enable all interrupts
}

void startTimer3(void) {  // not Daybreak -- ok
  noInterrupts();          // disable all interrupts
  TCNT3 = 3036;            // preload timer for 1 s period
  TIMSK3 |= (1 << TOIE3);  // enable timer overflow interrupt
  interrupts();            // enable all interrupts
}

void ADCandDACtest(void) {  // not Daybreak -- ok
  Serial3.println();
  int* results;
  results = get8ADC();
  for (int i = 0; i < 8; i++) {
    Serial3.print("ADC#" + String(i) + ":  " + String(getADC(i)));
    Serial3.println("; ADC[" + String(i) + "]: " + String(results[i]));
  }
  
  Serial3.println("\nwriting 0 (0x000) to DAC1 & DAC2");
  writeDAC(1, 0);
  writeDAC(2, 0);
  Serial3.println();

  results = get8ADC();
  for (int i = 0; i < 8; i++) {
    Serial3.print("ADC#" + String(i) + ":  " + String(getADC(i)));
    Serial3.println("; ADC[" + String(i) + "]: " + String(results[i]));
  }
  delay(3000);
  Serial3.println("\nwriting 4095 (0xFFF) to DAC1 & DAC2");
  writeDAC(1, 4095);
  writeDAC(2, 4095);
  Serial3.println();

  results = get8ADC();
  for (int i = 0; i < 8; i++) {
    Serial3.print("ADC#" + String(i) + ":  " + String(getADC(i)));
    Serial3.println("; ADC[" + String(i) + "]: " + String(results[i]));
  }
}

void testCounter(long unsigned int dTime) {  // not Daybreak -- ok
  dTime = min(1000, dTime);
  Serial3.println("\nstarting counter test");
  // unsigned long timerTime;
  cOVF = 0;
  timerTime = millis();
  TCNT5 = 0;
  delay(dTime);
  PORTL |= 0x02;
  PORTL &= 0xFD;  // flash ICP5
  timerTime = millis() - timerTime;
  // x3 = TCNT5; <-- does it interfere with an interrupt service routine?
  delay(100);
  x4 = TCNT5;
  // Serial3.print("; direct read " + String(x3 + cOVF*65536));
  Serial3.print("; direct delayed read " + String(x4 + cOVF*65536));
  Serial3.print("; #pulses captured " + String(x1 + cOVF*65536) + " read in " + String(x2 + cOVF*65536));
  Serial3.println("; timerTime " + String(timerTime));
}

void setRampClock(unsigned int rcTime) {  // has no counterpart in 1100 firmware
  // nr of msecs between overflows / interrupts
  // rcTime = numTicks * 255 + lastMSEC;
  periodRampClock = rcTime;
  numTicks = periodRampClock / 255;
  lastMSEC = periodRampClock % 255;
  if (numTicks == 0) {
    setDataByte(255 - lastMSEC);  // or 256? what is the condition for an overflow RCO
  } else {
    setDataByte(0);
  }
  if (lastMSEC > 0) numTicks++;
  digitalWrite(pCSRCK, LOW);  // reset CSRCK (PD2 = pCSRCK) to move initial value to ramp counter input register
  digitalWrite(pCSRCK, LOW);
  digitalWrite(pCSRCK, HIGH);
  Ticks = 0;
}

ISR(TIMER5_OVF_vect) {  // not Daybreak -- ok
  // input capture interrupt service routine
  cOVF++;
}
/*  end of definitions of test routines  */

void writeRampDiv(unsigned int rampDiv) {  // (rampDiv) RAMP-DIV C! -- writes an initial unsigned char value to the ramp clock counter (CSRCK/)
  setDataByte(lowByte(rampDiv));
  digitalWrite(pCSRCK, LOW);  // reset CSRCK (PD2 = pCSRCK) to move initial value to ramp counter input register
  digitalWrite(pCSRCK, LOW);
  digitalWrite(pCSRCK, HIGH);
}
