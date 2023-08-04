#include <Arduino.h>
/* C:\Users\Andrzej\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6\cores\arduino\Arduino.h */
#include "mega2560.h"
#include "myString.h"
#include "Daybreak1150cmds.h"

// regex ^(bool|int|void)((?!-- ok).)*$ finds function definitions not ending with '-- ok'

int RampSeg = 0;
int Sample = 0;
bool changeDisp = false;
unsigned long Time = 0;
unsigned long timerTime = 0;
word Errors = 0;
// isBusy is set/reset in initHome and should also be set/reset around each of @-_ commands
bool isBusy = false;  // -- ok
bool dataReady;
bool isFLConsole = true;
unsigned long Photons;
unsigned int Ticks, numTicks, lastMSEC, periodRampClock;
// OSL ramp related
int oslInc, oslRamp;
bool oslOn, oslDisp;
int nr10msec, nr100msec, nr1sec, nr10sec;
unsigned int divTicks10msec, divTicks100msec, divTicks1sec, divTicks10sec;  // RAMP-DIV IN HIGH BYTE, NUMTICKS LOW
int TBindex, TBpoints, startTBindex;
// word DAC2;
byte Disp[4];  // DSP
byte Segs[10] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6};  // SEGS

String rampseg[11] = {"rseg_Idle", "rseg_Preheat", "rseg_PhHold", "rseg_PhCool", "rseg_StagePh", "rseg_StHold", "rseg_Ramp", "rseg_EndHold", "rseg_CoolDwn", "n.a.", "rseg_OSL"};

void setDataByte(byte data) {
  DDRF = 0xFF;  // switch port F to output
  PORTF = data;
}

byte getDataByte(void) {
  DDRF = 0x00;  // switch port F to input
  PORTF = 0xFF;  // and pull-up
  return(PINF);
}

void setAddress(byte address) {
  // address is low nibble of port A (PA0-3)
  address &= 0x0F;  // clear high nibble to be safe
  PORTA &= 0xF0;    // clear address bits PA0-3
  PORTA |= address;  // set address lines
}

byte getADC(byte num) {  // GET-ADC ( N -- VALUE ) // num - ADC number 0-7
  // port A low nibble as output (A0-3) and CSADC/ (A4) in setupATmegaPorts()
  byte data;
  if (num < 8) {
    setAddress(num);
    digitalWrite(pCSADC, LOW); 
    delayMicroseconds(3);  // wait for conversion
    data = getDataByte();
    digitalWrite(pCSADC, HIGH);
  }
  return(data);
}

int* get8ADC() {  // return values from all ADCs in a static array 
  static int adc[8];
  for (byte i = 0; i < 8; i++) {
    setAddress(i);
    digitalWrite(pCSADC, LOW); 
    delayMicroseconds(3);
    adc[i] = getDataByte();
    digitalWrite(pCSADC, HIGH);
  }
  return(adc);  // returns pointer to a static array 
  // how to use it
  // int* results;
  // results = get8ADC();
  // e.g. Serial3.println("ADC[" + String(i) + "]: " + String(results[i]));
  // each value is accessed as results[i] or *(results + i)
}

void writeDAC(byte n, word val) {  // PUT-DAC | PUT-DAC2  // n = 1 - DACA (RAMP); n = 2 - DACB (OSL?)
  byte lo, hi;
  val = min(val, 4095);  // 12-bit DAC
  if (n < 2) n = 1;
  if (n > 1) n = 2;
  lo = lowByte(val);
  hi = highByte(val);
  setDataByte(lo);
  if (n == 1) {
    setAddress(0x00);  // set address to DAC1 LSB
  } else {
    setAddress(0x02);  // or to DAC2 LSB
  }
  digitalWrite(pCSDAC1, LOW);  // reset CSDAC1 & WR (PD0 = pCSDAC1) and write val to input register
  digitalWrite(pCSDAC1, LOW);
  digitalWrite(pCSDAC1, HIGH);
  setDataByte(hi);
  if (n == 1) {
    setAddress(0x01);  // set address to DAC1 MSB
  } else {
    setAddress(0x03);  // or to DAC2 MSB
  }
  digitalWrite(pCSDAC1, LOW);  // reset CSDAC1 & WR (PD0 = pCSDAC1) and write val to input register
  digitalWrite(pCSDAC1, LOW);
  digitalWrite(pCSDAC1, HIGH);

  digitalWrite(pCSDAC2, LOW);  // reset CSDAC2 (PD1 = pCSDAC2) to move value to internal DAC register and start conversion
  digitalWrite(pCSDAC2, LOW);
  digitalWrite(pCSDAC2, HIGH);
  // DDRF = 0xFF;  // switch port F back to input
  // PORTF = 0xFF;  // and pull-up
}

void write2DAC(word val1, word val2) {  // PUT-DAC & PUT-DAC2  // write 2 values to 1 - DACA (RAMP) & 2 - DACB (OSL?)
  val1 = min(val1, 4095);  // 12-bit DAC
  val2 = min(val2, 4095);  // 12-bit DAC
  setDataByte(lowByte(val1));  // low val1 byte to D0-7
  setAddress(0x00);  // set address 0x00 to DAC1 LSB
  digitalWrite(pCSDAC1, LOW);  // reset CSDAC1 & WR (PD0 = pCSDAC1) and write val to input register
  digitalWrite(pCSDAC1, HIGH);
  setDataByte(highByte(val1));  // high val1 byte to D0-7
  setAddress(0x01);  // change address 0x01 to MSB
  digitalWrite(pCSDAC1, LOW);  // reset CSDAC1 & WR (PD0 = pCSDAC1) and write val to input register
  digitalWrite(pCSDAC1, LOW);
  digitalWrite(pCSDAC1, HIGH);

  setDataByte(lowByte(val2));  // low val2 byte to D0-7
  setAddress(0x02);  // change address 0x02 to DAC2 LSB
  digitalWrite(pCSDAC1, LOW);  // reset CSDAC1 & WR (PD0 = pCSDAC1) and write val to input register
  digitalWrite(pCSDAC1, LOW);
  digitalWrite(pCSDAC1, HIGH);
  setDataByte(highByte(val2));  // high val2 byte to D0-7
  setAddress(0x03);  // change address 0x03 to MSB
  digitalWrite(pCSDAC1, LOW);  // reset CSDAC1 & WR (PD0 = pCSDAC1) and write val to input register
  digitalWrite(pCSDAC1, LOW);
  digitalWrite(pCSDAC1, HIGH);

  digitalWrite(pCSDAC2, LOW);  // reset CSDAC2 (PD1 = pCSDAC2) to move values to internal DAC registers and start conversions
  digitalWrite(pCSDAC2, LOW);
  digitalWrite(pCSDAC2, HIGH);
  // DDRF = 0xFF;  // switch port F back to input
  // PORTF = 0xFF;  // and pull-up
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

void setupCounter(void) {  // has no counterpart in 1100 firmware - R65F12 internal counter B is setup in SET-UP
  // TCNT5 counter: PL2 counter T5 (counter input), PL1 counter ICP5 (input capture pin)
  // setup counter TCNT5
  DDRL &= ~((1 << DDB1) | (1 << DDB2)); // set pins PL1, PL2 as input
  TCCR5A = 0x00; // set timer/counter control register A to 0
  TCCR5B = (1 << ICES5) | (1 << CS52) | (1 << CS51) | (0 << CS50); // set timer/counter control register B
                                                                   // input: falling edge, external clock
                                                                   // ICP: rising edge from U11/5
  TIMSK5 |= (1 << ICIE5) | (0 <<  TOIE5); // enable input capture interrupt, disable overflow interrupt
}

void readCounter(void) {  // READ-COUNTER
  // there is a patch fixing 256 overcount problem 
  Ticks = 0;
  digitalWrite(pCSCNT, LOW);
  digitalWrite(pCSCNT, LOW);
  digitalWrite(pCSCNT, LOW);
  Photons = getDataByte();
  digitalWrite(pCSCNT, HIGH);
  // x2 = TCNT5;
  // x4 = ICR5;
  // x1 = Photons;
  if (Photons == 0xFF) Photons += (unsigned long) (TCNT5 - 1)*0x100;  // checked OK
  else Photons += (unsigned long) TCNT5*0x100;  // checked OK
  digitalWrite(clrPC, LOW);
  digitalWrite(clrPC, LOW);  // flash port PL0 to clear Photon Counter
  digitalWrite(clrPC, LOW);
  digitalWrite(clrPC, HIGH);
  TCNT5 = 0;  // clear TCNT5 
  dataReady = true;
}

void startCounter(void) {  // START-COUNTER
  PORTL = PORTL & 0xFE;  // flash port PL0 to clear Photon Counter
  PORTL = PORTL & 0xFE;
  PORTL = PORTL & 0xFE;
  PORTL = PORTL | 0x01;
  TCNT5 = 0;  // clear TCNT5 
}

ISR(TIMER5_CAPT_vect) {  // the main interrupt server that calls specialised servers
  // input capture interrupt service routine
  // if (Ticks == numTicks) {
  //   // x3 = millis();
  //   // read photons
  //   readCounter();
  //   // and set Ramp Clock
  //   setRampClock(periodRampClock);
  //   Time++;
  // } else if (numTicks - Ticks == 1) {
  //   // set lastMSEC to Ramp Clock
  //   setDataByte(255 - lastMSEC);  // or 256? what is the condition for an overflow RCO
  // digitalWrite(pCSRCK, LOW);  // reset CSRCK (PD2 = pCSRCK) to move initial value to ramp counter input register
  // digitalWrite(pCSRCK, LOW);
  // digitalWrite(pCSRCK, HIGH);
  // }
  // Ticks++;
  // Serial.println("ISR  Ticks=" + String(Ticks) + "  oslOn=" + String(oslOn ? "Y" : "N"));
  if (oslOn) {
    OSLServer();
  }
  else RampServer();
}

void setError(byte err_src) {  // SET-XXX-ERR -- ok
  Errors |= (1 << err_src);
}

void resetErrors(void) {  // RESET-ERRORS -- ok
  Errors = 0;
}

void TFault(void) { /*  // TFAULT
  // TODO:
  bitClear(Errors, T_err);
  if ((digitalRead(pTCfault) = LOW) || ((4*getTemp() > RampEnd4) && (digitalRead(pOvenEn) == LOW) && !isSetPt)) {
    setError(T_err);
    OvenDisp = Flash;
  } else {
    OvenDisp = (digitalRead(pOvenEn) == LOW);
  } */
}

void HVFault(void) { /*  // HVFAULT
  // TODO:
  if ((digitalRead(pShutOpen) == HIGH) && (digitalRead(pHVen) == LOW)) {
    setError(HV_err);
    HVdisp = Flash;
  } else {
    HVdisp = (digitalRead(pHVen) == LOW);
  } */
}

void IrradFault(void) { /*  // IRRADFAULT
  // TODO:
  if (elevTimeOut) {
    setError(Irrad_err);
  }
  if (elevTimeOut) {
    IrradDisp = Flash;
  } else {
    IrradDisp = (digitalRead(pIrrad) == HIGH);
  } */
}

void RampFault(void) { // RAMPFAULT
  if ((Ramp > (getADC(0)*125.0)/8.0) || ((Ramp > RampEnd4) && (!isSetPt))) {
    setError(Ramp_err);
  }
}

void ChangeFault(void) { /*  // CHANGEFAULT
  // TODO:
  if (((Errors & 0x01F0) > 0x0) || !((digitalRead(pPlatHom) == LOW) && (digitalRead(pArmHom) == LOW) && (isPlatterPos()))) {
    changeDisp = Flash;
  } else {
    changeDisp = false;
  } */
}

void CheckVacuum(void) { /*  // CHECK-VACUUM
  // TODO:
  if (digitalRead(pVacBleed) == LOW) {
    if (getADC(3) > 7) {
      VacOff();
      MainOn();
    }
  } */
}

void CheckIrradTime(void) { /*  // CHECK-IRRADTIME
  // TODO:
  if (digitalRead(pIrrEn) == HIGH) {
    if (Time + 1 > IrradTime) {
      irradStop();
      startTimer();
    }
  } */
}

void CheckCalTime(void) { /*  // CHECK-CALTIME -- not used
  // TODO:
  if (digitalRead(pCalEn) == HIGH) {
    if (Time + 1 > CalTime) {
      calOff();
      startTimer();
    }
  } */
}

void CheckAll(void) {  // CHECK-ALL
  // TODO: complete coding of fault checks
  TFault();
  HVFault();
  IrradFault();
  RampFault();
  ChangeFault();
  CheckVacuum();
  CheckIrradTime();
  // CheckCalTime();
}

void startTimer(void) {  // START-TIMER
  // initialises main board ramp (clock) timer: 250 counts, 4 TICKS -- 1 sec
  Ticks = 0;
  numTicks = 4;
  writeRampDiv(0x05);  // 5 = 255 - 250; 0x05 = ~0xFA
  Time = 0;
}

void sendHead(void) {  // SEND-HEAD  sends number of photons (3 bytes) and point number (1 byte or 2 bytes for 2200)
  String s, statusStr;

  dataReady = false;   // clears 'dataReady'
  s = String(Photons, HEX);
  while (s.length() < 6) s = "0" + s;
  statusStr = s;
  if (PointNo == -1) s = "FF";
  else s = String(PointNo, HEX);
  while (s.length() < 2) s = "0" + s;  // or 2 byte PointNo for 2200
  statusStr.concat(s);
  Serial3.print(statusStr);
}

void sendVHead(int point) {  // SEND-V-HEAD  sends Curve[point] and point
  String s, statusStr;

  s = String(Curve[point], HEX);
  while (s.length() < 6) s = "0" + s;
  statusStr = s;
  String(point, HEX);
  while (s.length() < 2) s = "0" + s;  // or 2 byte point number for 2200
  statusStr.concat(s);
  Serial3.print(statusStr);
}

void sendRest(void) {  // SEND-REST
  String s, statusStr;
  byte statusB;
  int* result;

  s = String(Sample, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr = s;
  if ((RampSeg == rseg_Ramp) && (Time == 0)) {  // modified to realise stage preheat functionality
    s = String(rseg_StHold, HEX);  
  } else {
    s = String(RampSeg, HEX);
  }
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  s = String(Errors, HEX);
  while (s.length() < 4) s = "0" + s;
  statusStr.concat(s);
  result = get8ADC();
  for (byte i = 0; i < 8; i++) {
    s = String(result[i], HEX);
    while (s.length() < 2) s = "0" + s;
    statusStr.concat(s);
  }
  statusB = (digitalRead(pDeckOK) << 0) | 
            (digitalRead(pDeckLo) << 1) | 
            (digitalRead(pArmOut) << 2) | 
            (digitalRead(pArmHom) << 3) | 
            (digitalRead(pArmIn) << 4) | 
            (digitalRead(pArmOK) << 5) | 
            (digitalRead(pPlatHom) << 6) | 
            (digitalRead(pPlatOK) << 7);
  s = String(statusB, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  statusB = (digitalRead(pElevEn) << 0) | 
            (1 << 1) |  // (digitalRead(pCalib) << 1) | 
            (1 << 2) |  // (digitalRead(pIrrad) << 2) | 
            (1 << 3) |  // (digitalRead(pVacMain) << 3) | 
            (1 << 4) |  // (digitalRead(pVacBleed) << 4) | 
            (digitalRead(pPurge) << 5) | 
            (digitalRead(pCool) << 6) | 
            (digitalRead(pOvenEn) << 7);
  s = String(statusB, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  statusB = (1 << 0) |  // (digitalRead(pHas770) << 0) | 
            (1 << 1) |  // (digitalRead(pTCfault) << 1) | 
            (1 << 2) |  // (digitalRead(pHVintlk) << 2) | 
            (digitalRead(pHVen) << 3) | 
            (digitalRead(pHasElev) << 4) | 
            ((isBusy ? 0 : 1) << 5) | 
            (1 << 6) |  // (digitalRead(p770OK) << 6) | 
            (digitalRead(pElevDir) << 7);
  // if (isBusy) statusB |= (0 << 5); else statusB |= (1 << 5);
  s = String(statusB, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  statusB = (1 << 0) |  // (digitalRead(pIRen) << 0) | 
            (1 << 1) |  // (digitalRead(pOSLen) << 1) | 
            (digitalRead(pDeckMi) << 2) | 
            (digitalRead(pDeckHi) << 3) | 
            (1 << 4) |  // (digitalRead(pContr770) << 4) | 
            (digitalRead(pArmEn) << 5) | 
            (digitalRead(pArmDir) << 6) | 
            (digitalRead(pPlatEn) << 7);
  s = String(statusB, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  s = String(Time, HEX);
  while (s.length() < 4) s = "0" + s;
  statusStr.concat(s);
  statusStr.toUpperCase();
  if (!isFLConsole) {
    s = " ramp " + String(Ramp, HEX) + " millis " + String(millis());
    statusStr.concat(s);
  }
  Serial3.print(statusStr);
  Serial3.write(0x0D);  // sends CARRIAGE RETURN  0x0D
  if (!isFLConsole) Serial3.write(0x0A);  // sends to IDE an  additional NEW LINE  0x0A
}

void send_Data(void) {  // SEND-DATA
  if (Serial3.availableForWrite() > 45) {
    sendHead();
    sendRest();
  }
}

void sendStatus(unsigned long int aCounts) {  // -- ok
  /* data structure 
   1 -  6  ffffff   - photon count
   7 - 24  ffffffffffffffffff - photon counts
   7 -  8  dd       - data point number
   9 - 10  ss       - sample number             (0 - 60)
  11 - 12  rr       - ramp segment/stage number (0 - 10)
  13 - 16  eeee     - error code
  17       Tm       - Tmax
           Tt       - T
           Te       - T error
           Vg       - Vacuum gauge
           Vc       - Vacuum gauge current
           Rv       - Ramp voltage
           Lt       - Lifetime duty factor
     - 32  HV       - High voltage
  33 - 40  ssssssss - 32 status bits
  41 - 44  tttt     - time since start
  */
  String s, statusStr;
  byte statusB;
  int* result;

  dataReady = false;
  s = String(aCounts, HEX);
  while (s.length() < 6) s = "0" + s;
  statusStr = s;
  if (PointNo == -1) s = "FF";
  else s = String(PointNo, HEX);
  while (s.length() < 2) s = "0" + s;  // or 2 byte PointNo for 2200
  statusStr.concat(s);
  s = String(Sample, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  if ((RampSeg == rseg_Ramp) && (Time == 0)) {  // modified to realise stage preheat functionality
    s = String(rseg_StHold, HEX);  
  } else {
    s = String(RampSeg, HEX);
  }
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  s = String(Errors, HEX);
  while (s.length() < 4) s = "0" + s;
  statusStr.concat(s);
  result = get8ADC();
  for (byte i = 0; i < 8; i++) {
    s = String(result[i], HEX);
    while (s.length() < 2) s = "0" + s;
    statusStr.concat(s);
  }
  statusB = (digitalRead(pDeckOK) << 0) | 
            (digitalRead(pDeckLo) << 1) | 
            (digitalRead(pArmOut) << 2) | 
            (digitalRead(pArmHom) << 3) | 
            (digitalRead(pArmIn) << 4) | 
            (digitalRead(pArmOK) << 5) | 
            (digitalRead(pPlatHom) << 6) | 
            (digitalRead(pPlatOK) << 7);
  s = String(statusB, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  statusB = (digitalRead(pElevEn) << 0) | 
            (1 << 1) |  // (digitalRead(pCalib) << 1) | 
            (1 << 2) |  // (digitalRead(pIrrad) << 2) | 
            (1 << 3) |  // (digitalRead(pVacMain) << 3) | 
            (1 << 4) |  // (digitalRead(pVacBleed) << 4) | 
            (digitalRead(pPurge) << 5) | 
            (digitalRead(pCool) << 6) | 
            (digitalRead(pOvenEn) << 7);
  s = String(statusB, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  statusB = (1 << 0) |  // (digitalRead(pHas770) << 0) | 
            (1 << 1) |  // (digitalRead(pTCfault) << 1) | 
            (1 << 2) |  // (digitalRead(pHVintlk) << 2) | 
            (digitalRead(pHVen) << 3) | 
            (digitalRead(pHasElev) << 4) | 
            ((isBusy ? 0 : 1) << 5) | 
            (1 << 6) |  // (digitalRead(p770OK) << 6) | 
            (digitalRead(pElevDir) << 7);
  // if (isBusy) statusB |= (0 << 5); else statusB |= (1 << 5);
  s = String(statusB, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  statusB = (1 << 0) |  // (digitalRead(pIRen) << 0) | 
            (1 << 1) |  // (digitalRead(pOSLen) << 1) | 
            (digitalRead(pDeckMi) << 2) | 
            (digitalRead(pDeckHi) << 3) | 
            (1 << 4) |  // (digitalRead(pContr770) << 4) | 
            (digitalRead(pArmEn) << 5) | 
            (digitalRead(pArmDir) << 6) | 
            (digitalRead(pPlatEn) << 7);
  s = String(statusB, HEX);
  while (s.length() < 2) s = "0" + s;
  statusStr.concat(s);
  s = String(Time, HEX);
  while (s.length() < 4) s = "0" + s;
  statusStr.concat(s);
  statusStr.toUpperCase();
  if (!isFLConsole) {
    s = " ramp " + String(Ramp, HEX) + " millis " + String(millis());
    statusStr.concat(s);
  }
  Serial3.print(statusStr);
  Serial3.write(0x0D);  // sends CARRIAGE RETURN  0x0D
  if (!isFLConsole) Serial3.write(0x0A);  // sends to IDE an  additional NEW LINE  0x0A
}

void testStatus(void) {  //  for tests only
  int* result;
  /*
  sample#
  busy
  relevant sensors (binary)
  errors (binary)
  */
  dataReady = false;
  String s = String(Photons);
  while (s.length() < 6) s = "0" + s;
  Serial3.print("photons " + s);
  Serial3.print(" sample " + byteToHexStr(Sample));
  Serial3.print(" rampSeg " + String(RampSeg));
  result = get8ADC();
  Serial3.print(" Vramp " + String(result[5]));
  Serial3.print(" T " + String(result[1]));
  Serial3.print(" Terr " + String(result[2]));
  Serial3.print(" HV " + String(result[7]));

  Serial3.print(" busy " + String(isBusy));

  Serial3.print(" errors " + wordToHexStr(Errors));
  Serial3.print(" time " + String(Time));
  if (Errors > 0) Serial3.print(" ERROR!");
  Serial3.println();
}

// 1100/1150 DISPLAY CODE

void Busy(void) {  // BUSY -- ok
  isBusy = true;
}

void unBusy(void) {  // UN-BUSY -- ok
  isBusy = false;
}

void ByteToDisp(byte Acc) {  // SEND-BYTE
  // TODO: complete coding - define signals pDspData, pDspClk
  // for (size_t i = 0; i < 8; i++) {
  //   digitalWrite(pDspData, bitRead(Acc, i));
  //   digitalWrite(pDspClk, HIGH);
  //   digitalWrite(pDspClk, HIGH);
  //   digitalWrite(pDspClk, HIGH);
  //   digitalWrite(pDspClk, LOW);
  // }
}

void StatusToDisp(void) {  // SEND-STATUS
  for (size_t i = 0; i < 4; i++) {
    ByteToDisp(Disp[i]);
  }
}

void NumberToDisp(void) {  // NUMBER2DSP
  Disp[2] = Segs[Sample % 0x0A];
  Disp[3] = Segs[Sample / 0x0A];
}

void FillDisp(void) {  // FILL-DSP
  // TODO: complete coding - define signals pVacBleed, pVacMain
  Disp[0] = 0;
  Disp[1] = 0;
  bitWrite(Disp[0], 4, (oslDisp ? 1 : 0));
  bitWrite(Disp[0], 5, (ElevDisp ? 1 : 0));
  bitWrite(Disp[0], 6, (rampOn ? 1 : 0));
  bitWrite(Disp[0], 7, ((digitalRead(pCalEn) == LOW) ? 1 : 0));
  bitWrite(Disp[1], 0, ((digitalRead(pCool) == LOW) ? 1 : 0));
  // bitWrite(Disp[1], 1, ((digitalRead(pVacBleed) == LOW) ? 1 : 0));
  bitWrite(Disp[1], 2, (IrradDisp ? 1 : 0));
  bitWrite(Disp[1], 3, (changeDisp ? 1 : 0));
  bitWrite(Disp[1], 4, (OvenDisp ? 1 : 0));
  bitWrite(Disp[1], 5, (HVdisp ? 1 : 0));
  bitWrite(Disp[1], 6, ((digitalRead(pPurge) == LOW) ? 1 : 0));
  // bitWrite(Disp[1], 7, ((digitalRead(pVacMain) == LOW) ? 1 : 0));
}

void DispStatus(void) {  // DISP-STATUS
  FillDisp();
  NumberToDisp();
  StatusToDisp();
}

void Chg(void) {  // CHG -- ok
  changeDisp = true;
  // DONE: DISP_STATUS();
  DispStatus();
}

void unChg(void) {  // UN-CHG -- ok
  changeDisp = false;
  // DONE: DISP_STATUS();
  DispStatus();
}

// 1100/1150 MOTION CODE

bool isArmPos(void) {  // IS-ARM-POS? -- ok
  return (digitalRead(pArmOK) == LOW);
}

bool isArmIn(void) {  // IS-ARM-IN? -- ok
  return (digitalRead(pArmIn) == LOW);
}

bool isArmOut(void) {  // IS-ARM-OUT? -- ok
  return (digitalRead(pArmOut) == LOW);
}

bool isArmHome(void) {  // IS-ARM-HOME? -- ok
  return (digitalRead(pArmHom) == LOW);
}

bool isPlatterPos(void) {  // IS-PLATTER-POS? -- ok
  return (digitalRead(pPlatOK) == LOW);
}

bool isPlatterHome(void) {  // IS-PLATTER-HOME? -- ok
  return (digitalRead(pPlatHom) == LOW);
}

bool isArmDest(int dest_pos) {  // IS-DEST? -- ok
  if (dest_pos == 0) return (isArmOut());
  if (dest_pos == 1) return (isArmHome());
  if (dest_pos == 2) return (isArmIn());
  return (false);
}

void PlatterWait(void) {  // PLATTER-WAIT -- ok
  while (digitalRead(pPlatOK) == LOW)
    ;  // waits until moved enough to be not sensed
}

void ArmWait(void) {  // ARM-WAIT -- ok
  while (digitalRead(pArmOK) == LOW)
    ;  // waits until moved enough to be not sensed
}

void setArmDirIn(void) {  // ARMDIR-IN -- ok
  digitalWrite(pArmDir, HIGH);
}

void setArmDirOut(void) {  // ARMDIR-OUT -- ok
  digitalWrite(pArmDir, LOW);
}

void startArm(void) {  // ARM-START -- ok
  digitalWrite(pArmEn, LOW);
}

void stopArm(void) {  // ARM-STOP -- ok
  digitalWrite(pArmEn, HIGH);
}

void startPlatter(void) {  // PLAT-START -- ok
  digitalWrite(pPlatEn, LOW);
}

void stopPlatter(void) {  // PLAT-STOP -- ok
  digitalWrite(pPlatEn, HIGH);
}

bool isNotLost(int pos) {  // NOT-LOST? -- not used -- ok
  static bool result = (isArmDest(pos) && isArmPos());
  if (!result) setError(Pos_err);
  return (result);
}

bool isOKtoMove(void) {  // OK-TO-MOVE? -- ok
  if ((RampSeg == 0) && isArmPos() && isPlatterPos()) return (true);
  else setError(Pos_err);
  return (false);
}

bool goArmIn(void) {  // ARM-IN? -- ok
  // -- modified
  unsigned long timeout = 5000;
  while (!isArmPos() && (millis() - timerTime < timeout)) ;
  stopArm();
  if (!isArmPos()) return (false);
  if (isArmIn()) return (true);
  startArm();
  ArmWait();
  while (!isArmPos() && (millis() - timerTime < timeout)) ;
  stopArm();
  return (isArmPos() && isArmIn());
}

bool goArmOut(void) {  // ARM-OUT? -- ok
  // static bool result = false;
  // -- modified
  unsigned long timeout = 5000;
  while (!isArmPos() && (millis() - timerTime < timeout)) ;
  stopArm();
  if (!isArmPos()) return (false);
  if (isArmOut()) return (true);
  startArm();
  ArmWait();
  while (!isArmPos() && (millis() - timerTime < timeout)) ;
  stopArm();
  return (isArmPos() && isArmOut());
}

bool goArmHome(void) {  // ARM-HOME? -- ok
  static bool result = false;
  unsigned long timeout = 5000;
  do {
    result = isArmPos() && isArmHome();
  } while (!result && (millis() - timerTime < timeout));
  stopArm();
  return (result);
}

bool goArmTo(int pos) {  // ARM? -- ok
  if (pos == 0) return (goArmOut());
  if (pos == 1) return (goArmHome());
  if (pos == 2) return (goArmIn());
  return (false);
}

bool ArmMove(int dest, int dir) {  // ARM-MOVE -- works for (1, 0|1) only
  static bool result;
  changeDisp = true;
  timerTime = millis();
  if (dir > 0) setArmDirIn();
  else setArmDirOut();
  startArm();
  ArmWait();
  goArmTo(dest);
  result = isArmDest(dest);
  if (result) changeDisp = false;
  else setError(Stuck_err);
  return (result);
}

bool Rotate1(bool check) {  // ROTATE1  ( CHECK?-- OK? ) -- ok
  unsigned long timeout = 5000;
  bool goOn;
  timerTime = millis();
  changeDisp = true;
  goOn = !check;
  if (check) goOn = isOKtoMove() && isArmHome();  // && ELEVATOR-DOWN?
  if (goOn) {
    timeout = 6000;
    startPlatter();
    PlatterWait();
    while (!isPlatterPos() && (millis() - timerTime < timeout))
      ;  // wait until next position is reached or too much time = stuck
    stopPlatter();
    if (isPlatterPos()) {
      Sample = (Sample + 1) % 20;
      send_Data();
      changeDisp = false;
    } else {
      setError(Pos_err);
      return (false);
    }
  } else {
    setError(Pos_err);
    return (false);
  }
  if (isPlatterHome() && (Sample != 0)) {
    Sample = 0;
    setError(Sync_err);
    return (false);
  }
  return (true);
}

bool RotateTo(int pos) {  // ROTATETO ( N--OK? ) -- ok
  bool done;
  if (pos == 0) done = isPlatterHome();
  else done = (pos == Sample);
  if (done) return (true);
  else {
    do 
      Rotate1(true);
    while ((pos != Sample) && (Errors == 0));
    if ((Errors == 0) && (pos == 0) && isPlatterHome()) return (true);
    if ((Errors == 0) && (pos == Sample)) return (true);
    return(false);
  }
}

bool ArmHome(void) {  // ARM-HOME -- ok
  if (isOKtoMove()) {
    if (!isArmHome()) {
      if (isArmIn()) {
        return (ArmMove(1, 0));
      } else {
        if (isArmOut()) {
          return (ArmMove(1, 1));
        } else {
          setError(Pos_err);
          return (false);
        }
      }
    } else {
      return (true);
    }
  } else {
    return (false);
  }
}

bool ArmIn(void) {  // ARM-IN -- ok
  bool result;
  if (isOKtoMove()) {
    if (!isArmIn()) {
      result = ArmToPos(1);
      if (!result) return(false);
      if (isArmIn()) return(true); 
      else result = ArmToPos(1);
      return(result && isArmIn());
      // return (ArmMove(2, 1)); -- modified
    } else {
      return (true);
    }
  } else {
    return (false);
  }
}

bool ArmOut(void) {  // ARM-OUT -- ok
  bool result;
  if (isOKtoMove()) {
    if (!isArmOut()) {
      result = ArmToPos(0);
      if (!result) return(false);
      if (isArmOut()) return(true); 
      else result = ArmToPos(0);
      return(result && isArmOut());
      // return (ArmMove(0, 0)); -- modified
    } else {
      return (true);
    }
  } else {
    return (false);
  }
}

bool sampleToPlate(void) {  // SAMPLE-TO-PLATE -- ok
  bool result = ArmHome();
  send_Data();
  if (result) {
    result = result && ArmOut();
    send_Data();
    if (result) {
      result = result && ArmHome();
      send_Data();
    }
  }
  return (result);
}

bool sampleToPlatter(void) {  // SAMPLE-TO-PLATTER -- ok
  bool result = ArmHome();
  send_Data();
  if (result) {
    result = result && ArmIn();
    send_Data();
    if (result) {
      result = result && ArmHome();
      send_Data();
    }
  }
  return (result);
}

void sampleBack(void) {  // SAMPLE-BACK
  sampleToPlatter();
}

bool isDeckOK(void) {
  static bool result;
  result = digitalRead(pDeckOK) == LOW;
  return(result);
}

bool isDeckLow(void) {
  static bool result;
  result = digitalRead(pDeckLo) == LOW;
  return(result);
}

bool isDeckMid(void) {
  static bool result;
  result = digitalRead(pDeckMi) == LOW;
  return(result);
}

bool isDeckHigh(void) {
  static bool result;
  result = digitalRead(pDeckHi) == LOW;
  return(result);
}

bool isDeckAt(int pos) {
  switch (pos) {
    case 0: return(isDeckLow()); break;
    case 1: return(isDeckMid()); break;
    case 2: return(isDeckHigh()); break;
    default: return(false); break;
  }
}

void goDeckDown(void) {
  unsigned long timeout = 25000;
  digitalWrite(pElevDir, HIGH);  // dir down
  timerTime = millis();
  digitalWrite(pElevEn, LOW);  // start moving
  while (digitalRead(pDeckOK) == LOW) ;  // wait until OK pos not sensed
  do {
    if (digitalRead(pDeckOK) == LOW) {
      if (digitalRead(pDeckLo) == LOW) {
        timerTime -= timeout + 50;
      }
    }
  } while ((millis() - timerTime) < timeout);
  digitalWrite(pElevEn, HIGH);  // stop moving
  if (!((digitalRead(pDeckOK) == LOW) && (digitalRead(pDeckLo) == LOW))) setError(Deck_err);
}

void goDeckMid(int dir) {  // up -> dir=1, down -> dir=0
  unsigned long timeout = 8000;
  if (dir == 1) digitalWrite(pElevDir, LOW);  // dir up
  else digitalWrite(pElevDir, HIGH);  // dir down
  timerTime = millis();
  digitalWrite(pElevEn, LOW);  // start moving
  while (digitalRead(pDeckOK) == LOW) ;  // wait until OK pos not sensed
  do {
    if (digitalRead(pDeckOK) == LOW) {
      if (digitalRead(pDeckMi) == LOW) {
        timerTime -= timeout + 50;
      }
    }
  } while ((millis() - timerTime) < timeout);
  digitalWrite(pElevEn, HIGH);  // stop moving
  if (!((digitalRead(pDeckOK) == LOW) && (digitalRead(pDeckMi) == LOW))) setError(Deck_err);
}

void goDeckHigh(void) {
  unsigned long timeout = 25000;
  digitalWrite(pElevDir, LOW);  // dir up
  timerTime = millis();
  digitalWrite(pElevEn, LOW);  // start moving
  while (digitalRead(pDeckOK) == LOW) ;  // wait until OK pos not sensed
  do {
    if (digitalRead(pDeckOK) == LOW) {
      if (digitalRead(pDeckHi) == LOW) {
        timerTime -= timeout + 50;
      }
    }
  } while ((millis() - timerTime) < timeout);
  digitalWrite(pElevEn, HIGH);  // stop moving
  if (!((digitalRead(pDeckOK) == LOW) && (digitalRead(pDeckHi) == LOW))) setError(Deck_err);
}

void Home(void) {  // HOME -- not used -- ok
  Chg();
  if (sampleToPlatter()) RotateTo(0);
  unChg();
}

bool Advance1(void) {  // ADVANCE1 -- ok
  bool result;
  Chg();
  result = sampleToPlatter();
  if (result) {
    result = Rotate1(true);
    if (result) {
      result = result && sampleToPlate();
    }
  }
  unChg();
  return (result);
}

bool AdvanceTo(int pos) {  // ADVANCETO -- ok
  bool result;
  Chg();
  result = sampleToPlatter();
  if (result) {
    result = RotateTo(pos);
    if (result) {
      delay(100);
      result = result && sampleToPlate();
    }
  }
  unChg();
  return (result);
}

// : ADVANCE READ1 19 MIN ADVANCETO DROP ; ( in A cmd only )
// : JUMP READ1 19 MIN CHG ROTATETO DROP UN-CHG ; ( in J cmd only )

void tillArmPos(void) {  // TILL-ARM-POS -- ok
  unsigned long timeout = 5000;
  timerTime = millis();
  startArm();
  while (digitalRead(pArmOK) == LOW)
    ;
  while ((digitalRead(pArmOK) == HIGH) && (millis() - timerTime < timeout))
    ;
  stopArm();
}

bool ArmToPos(int dir) {  // ARM-TO-POS -- ok
  static bool result;
  if (dir > 0) {
    setArmDirIn();
  } else {
    setArmDirOut();
  }
  tillArmPos();  // start-timer(); inside
  result = isArmPos();
  if (!result) setError(Pos_err);
  return (result);
}

int whereArm(void) {  // WHERE-ARM? -- ok
  static int result = -1;  // nowhere
  if (isArmOut()) result = 0;
  if (isArmHome()) result = 1;
  if (isArmIn()) result = 2;
  return (result);
}

bool ArmRecover(void) {  // ARM-RECOVER called in INIT-HOME (probably not used due to sensors range) -- ok
  static bool result;
  result = ArmToPos(1);  // home position
  if (result) {
    if (whereArm() == -1) {  // nowhere found
      result = ArmToPos(0);
      if (result) {
        result = whereArm() > -1;
      }
    } else {
      result = true;
    }
  }
  return (result);
}

bool initArm(void) {  // INIT-ARM -- ok
  static bool result;
  static int pos;
  pos = whereArm();
  result = isArmPos() || (pos > -1);
  if (result) {
    if (pos > -1) {
      if (!isArmPos()) {
        if (pos == 0) {
          ArmMove(1, 1);
        } else {
          if (pos == 1) {
            ArmMove(2, 1);
          } else {
            if (pos == 2) {
              ArmMove(1, 0);
            }
          }
        }
      } else {
        result = true;
      }
    } else {
      result = false;
    }
  } else {
    result = ArmRecover();
  }
  if (result) {
    result = sampleToPlatter();
  }
  return (result);
}

bool RotateHome(void) {  // ROTATE-HOME -- ok
  static bool result;
  result = isPlatterHome() && isPlatterPos();
  if (!result) {
    do {
      result = Rotate1(false);
    } while (result && (!isPlatterHome()));
  }
  result = isPlatterHome() && isPlatterPos();
  return (result);
}

void initHome(void) {  // INIT-HOME -- ok
  Busy();
  Chg();
  // ELEVATOR-DOWN
  if (isPlatterPos()) {
    if (initArm()) {
      RotateHome();
    }
  } else {
    toPlatPos();
    if (RotateHome()) {
      initArm();
    }
  }
  unBusy();
  unChg();
}

// OSL ramp related
void setTB(unsigned int nT, unsigned int rD) {  // SET-TB - command 'T22 x y '
  numTicks = nT;
  writeRampDiv(rD);
}

void setTBtable(void) {  // SET-TB-TABLE - sets default channel widths and number
  nr10msec = 0x00;
  nr100msec = 0x00;
  nr1sec = 0x64;  // 100
  nr10sec = 0x00;
  startTBindex = 0x02;
  divTicks10msec = 0x01F5;   // rampDiv - 1, numTicks - 245 -> 10
  divTicks100msec = 0x019B;   // rampDiv - 1, numTicks - 155 -> 100
  divTicks1sec = 0x0405;   // rampDiv - 4, numTicks - 5 -> 250
  divTicks10sec = 0x2805;   // rampDiv - 40, numTicks - 5 -> 250
}

void getTBtable(unsigned int X1, unsigned int Y1, unsigned int X2, unsigned int Y2, \
                unsigned int X3, unsigned int Y3, unsigned int X4, unsigned int Y4) {  
  // GET-TB-TABLE - command 'Bx1 y1 x2 y2 x3 y4 x4 y4 '
  // TODO: is it correct to subtract 0x100 - Y ? or 0xFF - Y
  divTicks10msec = X1*0x100 + (0x100 - Y1);
  divTicks100msec = X2*0x100 + (0x100 - Y2);
  divTicks1sec = X3*0x100 + (0x100 - Y3);
  divTicks10sec = X4*0x100 + (0x100 - Y4);
}

void getOSLinfo(unsigned int x0, unsigned int x1, unsigned int x2, unsigned int x3) {  // GET-OSL-INFO
  startTBindex = -1;
  nr10msec = x0;
  if ((x0 > 0) && (startTBindex < 0)) startTBindex = 0;
  nr100msec = x1;
  if ((x1 > 0) && (startTBindex < 0)) startTBindex = 1;
  nr1sec = x2;
  if ((x2 > 0) && (startTBindex < 0)) startTBindex = 2;
  nr10sec = x3;
  if ((x3 > 0) && (startTBindex < 0)) startTBindex = 3;
}

void oslLEDsOn(void) {  // OSL-LEDS-ON PG0 IRenable
  // digitalWrite(pIRen, LOW);
}

void oslLEDsOff(void) {  // OSL-LEDS-OFF PG0 IRenable
  // digitalWrite(pIRen, HIGH);
}

void constCurrent(void) {  // CONST-CURRENT PG1 OSLEnable on
  // digitalWrite(pOSLen, LOW);
}

void servoCurrent(void) {  // SERVO-CURRENT PG1 OSLEnable off
  // digitalWrite(pOSLen, HIGH);
}

void doLEDs(unsigned int power, unsigned int mode) {  // DO-LEDS  N-command
  writeDAC(2, power);
  if (power == 0) {
    servoCurrent();
    // writeExtension(0x1FEE, 0);
    oslLEDsOff();
    oslDisp = false;
  } else {
    if (mode == 2) {
      // writeExtension(0x1FEE, 1);
      constCurrent();  // EXT SHUTTER  AND GREEN
    } else {
      servoCurrent();
      oslLEDsOn();
    }
    oslDisp = true;
    startTimer();
  }
}

void firstPt(unsigned int fstPt) {  // ST# 
  if (fstPt == 0) {
    PointNo = -1;
    lastSent = -1;
    CurvePt = -1;
  } else {
    PointNo = 0;
    lastSent = 0;
    CurvePt = 0;
  }
}

void doOSL(unsigned int power, unsigned int mode, unsigned int fstPt) {  // DO-OSL  U-command
  oslRamp = power;
  writeDAC(2, power);
  if (mode > 3) {  // mode = 0-3--> DC, 4-7--> LM
    oslInc = oslRamp;
  } else {
    oslInc = 0;
  }
  servoCurrent();  // OSL off
  if (power > 0) {  // then start OSL
    if (mode == 6) {
      constCurrent();  // OSL on
      // writeExtension(0x1FEE, 1);
    } else {
      oslLEDsOn();  // IRSL on
    }
    delay(5);
    TBindex = startTBindex;
    Time = 0;
    oslDisp = true;
    firstPt(fstPt);
    RampSeg = 0x0A;
    setupTB();
    startCounter();
    oslOn = true;
  } else {
    servoCurrent();
    // writeExtension(0x1FEE, 0);
    oslOn = false;
    oslDisp = false;
  }
  Serial.println("In doOSL()");
  Serial.println("  TBindex: " + String(TBindex));
  Serial.println("    oslOn: " + String(oslOn ? "yes" : "no"));
  Serial.println("   10ms: " + String(nr10msec) + "  " + String(highByte(divTicks10msec)) + "  " + String(lowByte(divTicks10msec)));
  Serial.println("  100ms: " + String(nr100msec) + "  " + String(highByte(divTicks100msec)) + "  " + String(lowByte(divTicks100msec)));
  Serial.println("     1s: " + String(nr1sec) + "  " + String(highByte(divTicks1sec)) + "  " + String(lowByte(divTicks1sec)));
  Serial.println("    10s: " + String(nr10sec) + "  " + String(highByte(divTicks10sec)) + "  " + String(lowByte(divTicks10sec)));
  Serial.println("  point: " + String(PointNo) + " last " + String(lastSent));
}

void OSLsendIfDataWaiting(void) {  // OSL-SEND-IF-DATA-WAITING
  // Serial.println("OSLsendIfDataWaiting point: " + String(PointNo) + "  last: " + String(lastSent));
  if ((RampSeg = 0x0A) && (PointNo > -1)) {
    if (lastSent < PointNo) {
      if (Serial3.availableForWrite() > 45) {
        lastSent++;
        // send data
        sendVHead(lastSent);
        sendRest();
      }
    } else {
      if (!oslOn) RampSeg = 0;  // all sent
    }
  }
}

void setUp(void) {  // SET-UP
  // what is necessary
  digitalWrite(pCSADC, HIGH);  // set all CS's high
  digitalWrite(pCSDAC1, HIGH);
  digitalWrite(pCSDAC2, HIGH);
  digitalWrite(pCSRCK, HIGH);
  digitalWrite(pCSCNT, HIGH);
  CoolTemp = 0x32;  // 50 degC
  RampEnd4 = 0x7D0;  // DAC=2000 - 500 degC
  EndPt4 = 0x7D0;  // DAC=2000 - 500 degC
  write2DAC(0, 0);  // setTemp(0);
  PointNo = -1;  // PT#
  lastSent = -1;  // LAST-SENT
  CurvePt = -1;  // 412 CONSTANT CURVE ( 3 BYTES/POINT UP TO 1E80, OR ACCORDING TO #DETS )
  // detsNr = 1;  // #DETS
  // bytesPerPoint = 3;  // #BYTES/PT
  numTicks = 4;  // NUMTICKS
  dSpace4 = 0x14;  // dDAC=20 - 5 degC
  rateCnt = 0x0E7;  // RATECNT 231 -- ok
  rampRate = 0x0A;  // RAMPRATE 10 degC/s
  writeRampDiv(0x05);  // 250 counts - 1 s intervals
  resetErrors();  // Errors = 0;
  AllOff();  // ALL-OFF
  // MaxPt = 0x7D0;  // 2000
  MaxPt = sizeof(Curve)/sizeof(Curve[0]);
  // startCounter();  // START-COUNTER
  initHome();  // INIT-HOME
  resetErrors();
  setTBtable();  // SET-TB-TABLE
}

// 1150 mechanical test code

void inToPos(void) {  // IN-TO-POS -- ok
  ArmToPos(1);
}

void outToPos(void) {  // OUT-TO-POS -- ok
  ArmToPos(0);
}

void toPlatPos(void) {  // TO-PLAT-POS -- ok
  sendStatus(Photons);
  startPlatter();
  while (isPlatterPos())
    ;
  while (!isPlatterPos())
    ;
  stopPlatter();
  sendStatus(Photons);
}

void Rot_1(void) {  // ROT-1 -- ok
  sendStatus(Photons);
  Rotate1(true);
  sendStatus(Photons);
}

void Ain(void) {  // AIN -- ok
  sendStatus(Photons);
  ArmIn();
  sendStatus(Photons);
}

void Aout(void) {  // AOUT -- ok
  sendStatus(Photons);
  ArmOut();
  sendStatus(Photons);
}

void Ahome(void) {  // AHOME -- ok
  sendStatus(Photons);
  ArmHome();
  sendStatus(Photons);
}

void doTestCommand(String cmdS) {  // DO-TEST
  int tNr = getArgument(cmdS, 1).toInt();
  unsigned int a1, a2;
  String cmd = getArgument(cmdS, 0);
  String arg;
  switch (tNr) {
    case 0:  // T0 -- arm inward to sensor
      inToPos();  // -- ok
      if (Errors > 0) sendStatus(Photons);
      break;
    case 1:  // T1 -- arm outward to sensor
      outToPos();  // -- ok
      if (Errors > 0) sendStatus(Photons);
      break;
    case 2:  // T2 -- rotate platter to next position sensor
      toPlatPos();  // -- ok
      if (Errors > 0) sendStatus(Photons);
      break;
    case 3:  // T3 -- elevator down absolute
      Serial3.println("T3 is not applicable");
      break;
    case 4:  // T4 -- rotate one - sample++
      Rot_1();  // -- ok
      if (Errors > 0) sendStatus(Photons);
      break;
    case 5:  // T5 -- arm to in position
      Ain();  // after modification -- ok
      if (Errors > 0) sendStatus(Photons);
      break;
    case 6:  // T6 -- arm to home position
      Ahome();  // -- ok
      if (Errors > 0) sendStatus(Photons);
      break;
    case 7:  // T7 -- arm to out position
      Aout();  // after modification -- ok
      if (Errors > 0) sendStatus(Photons);
      break;
    case 11:  // home platter ( 0-19)
      Serial3.println("T11 is not implemented yet");
      break;
    case 12:  // mid platter (20-39)
      Serial3.println("T12 is not implemented yet");
      break;
    case 13:  // high platter (40-59)
      Serial3.println("T13 is not implemented yet");
      break;
    case 17:  // platters down to pos
      break;
    case 18:  // platters up to pos
      break;
    case 20:  // arg = 0 irradiator over irradiator port, 1 irradiator back
      arg = getArgument(cmdS, 2);
      Serial3.println("T20 is not implemented yet");
      break;
    case 22:  // arg1 number of ticks, arg2 number of msecs per tick
      arg = getArgument(cmdS, 2);
      a1 = arg.toInt();
      arg = getArgument(cmdS, 3);
      a2 = arg.toInt();
      setTB(a1, a2);
      break;
    default:
      Serial3.println("Test command " + cmdS + " is not defined here.");
      break;
  }
}

void doCommand(String cmdS) {  // DO-COMMAND
  int argNr = countArgs(cmdS);
  char cmd = getArgument(cmdS, 0).charAt(0);
  String arg;
  int a1, a2, a3, a4, b1, b2, b3, b4;
  bool errSyntax, errNotDef;
  errNotDef = false;
  errSyntax = false;
  // isBusy should also be set/reset around each of @-_ commands
  Busy();
  switch (cmd) {
    case '@':
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        setTemp(arg.toInt());
      } else errSyntax = true;
      break;
    case 'A':  // ADVANCE
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        AdvanceTo(arg.toInt());
        if (Errors > 0) sendStatus(Photons);
      } else errSyntax = true;
      break;
    case 'B':  // GET-TB-TABLE
      if (argNr == 8) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        arg = getArgument(cmdS, 2);
        b1 = arg.toInt();
        arg = getArgument(cmdS, 3);
        a2 = arg.toInt();
        arg = getArgument(cmdS, 4);
        b2 = arg.toInt();
        arg = getArgument(cmdS, 5);
        a3 = arg.toInt();
        arg = getArgument(cmdS, 6);
        b3 = arg.toInt();
        arg = getArgument(cmdS, 7);
        a4 = arg.toInt();
        arg = getArgument(cmdS, 8);
        b4 = arg.toInt();
        getTBtable(a1, b1, a2, b2, a3, b3, a4, b4);
      } else errSyntax = true;
      break;
    case 'C':  // COOL
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        argNr = arg.toInt();
        switch (argNr) {
          case 0:
            CoolOff();
            break;
          case 1:
            CoolOn();
            break;
          default: 
            Serial3.println("Command " + cmdS + " is not defined.");
            break;
        }
      } else errSyntax = true;
      break;
    case 'D':  // GET-SPACE
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        getSpace(a1);
      } else errSyntax = true;
      break;
    case 'E':  // GET-ENDPOINT
      if (argNr == 2) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        arg = getArgument(cmdS, 2);
        a2 = arg.toInt();
        getEndPoint(a1, a2);
      } else errSyntax = true;
      break;
    case 'F':  // 4FILTER
      errNotDef = true;
      break;
    case 'G':  // DO-RAMP
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        doRamp(a1);
      } else errSyntax = true;
      break;
    case 'H':  // DO-HV
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        doHV(a1);
      } else errSyntax = true;
      break;
    case 'I':  // DO-IRRAD
      errNotDef = true;
      break;
    case 'J':  // JUMP
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        RotateTo(arg.toInt());
        if (Errors > 0) sendStatus(Photons);
      } else errSyntax = true;
      break;
    case 'K':  // DO-CAL
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        doCal(arg.toInt());
      } else errSyntax = true;
      break;
    case 'L':  // GET-COOL
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        getCool(arg.toInt());
      } else errSyntax = true;
      break;
    case 'M':  // SAMPLE-BACK
      if (argNr == 0) sampleBack();
      else errSyntax = true;
      break;
    case 'N':  // DO-LEDS
      if (argNr == 2) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        arg = getArgument(cmdS, 2);
        a2 = arg.toInt();
        doLEDs(a1, a2);
      } else errSyntax = true;
      break;
    case 'O':  // DO-OVEN
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        argNr = arg.toInt();
        switch (argNr) {
          case 0:
            OvenOff();
            break;
          case 1:
            OvenOn();
            break;
          default: 
            Serial3.println("Command " + cmdS + " is not defined.");
            break;
        }
      } else errSyntax = true;
      break;
    case 'P':  // DO-PURGE
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        doPurge(a1);
      } else errSyntax = true;
      break;
    case 'Q':  // DO-QUERY
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        doQuery(a1);
      } else errSyntax = true;
      break;
    case 'R':  // GET-RAMPRATE
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        getRampRate(a1);
      }
      else errSyntax = true;
      break;
    case 'S':  // GET-STAGE
      if (argNr == 2) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        arg = getArgument(cmdS, 2);
        a2 = arg.toInt();
        getStage(a1, a2);
      }
      else errSyntax = true;
      break;
    case 'T':  // TEST
      doTestCommand(cmdS);
      break;
    case 'U':  // DO-OSL
      if (argNr == 3) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        arg = getArgument(cmdS, 2);
        a2 = arg.toInt();
        arg = getArgument(cmdS, 3);
        a3 = arg.toInt();
        doOSL(a1, a2, a3);
      } else errSyntax = true;
      break;
    case 'V':  // DO-VACUUM
      errNotDef = true;
      break;
    case 'W':  // GET-PREHEAT
      if (argNr == 2) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        arg = getArgument(cmdS, 2);
        a2 = arg.toInt();
        getPreheat(a1, a2);
      }
      else errSyntax = true;
      break;
    case 'X':  // GET-OSL-INFO
      if (argNr == 4) {
        arg = getArgument(cmdS, 1);
        a1 = arg.toInt();
        arg = getArgument(cmdS, 2);
        a2 = arg.toInt();
        arg = getArgument(cmdS, 3);
        a3 = arg.toInt();
        arg = getArgument(cmdS, 4);
        a4 = arg.toInt();
        getOSLinfo(a1, a2, a3, a4);
      } else errSyntax = true;
      break;
    case 'Y':  // SET-DAC2
      if (argNr == 1) {
        arg = getArgument(cmdS, 1);
        setDAC2(arg.toInt());
      } else errSyntax = true;
      break;
    case 'Z':  // SET-UP
      if (argNr == 0) {
        setUp();
      } else errSyntax = true;
      break;
    case '[':  // 862-CONTROL
      errNotDef = true;
      break;
    case '\\':  // NOP
      errNotDef = true;
      break;
    case ']':  // NOP
      errNotDef = true;
      break;
    case '^':  // SAMPLE-BACK
      if (argNr == 0) sampleBack();
      else errSyntax = true;
      break;
    case '_':  // RESET-ERRORS
      if (argNr == 0) {
        resetErrors();
      } else errSyntax = true;
      break;
    default:
      errSyntax = true;
      break;
  }
  unBusy();
  if (errSyntax) Serial3.println("Syntax error. Command " + cmdS + " is not defined.");
  if (errNotDef) Serial3.println("Command " + cmdS + " is not defined yet.");
}

void processCmd(String cmdS) {  // wrap around doCommand() && special commands
  int argNr = countArgs(cmdS);
  static byte n;
  static word val;
  String cmd, arg;
  bool olc, tlc;
  olc = ((cmdS.length() == 1) && (cmdS.charAt(0) >= '@') && (cmdS.charAt(0) <= '_')) || ((cmdS.charAt(0) >= '@') && (cmdS.charAt(0) <= '_') && (((cmdS.charAt(1) >= '0') && (cmdS.charAt(1) <= '9')) || cmdS.charAt(1) == ' '));
  tlc = (cmdS.length() > 1) && (cmdS.charAt(0) >= 'A') && (cmdS.charAt(0) <= 'Z') && (cmdS.charAt(1) >= 'A') && (cmdS.charAt(1) <= 'Z');
  if (olc) {
    doCommand(cmdS);
  } else if (tlc) { // a two-letter-command for a special test purpose
    cmd = cmdS.substring(0, 2);
    if (cmd == "AD") {  // read ADC -- AD n
      if (argNr >= 1) {
        arg = getArgument(cmdS, argNr);
        Serial3.println("ADC#" + arg + ": " + String(getADC(arg.toInt())));
      } else {
        for (int i = 0; i < 8; i++) {
          Serial3.println("ADC#" + String(i) + ": " + String(getADC(i)));
        }
      }
      sendStatus(Photons);
    } else if ((cmd == "DA") && (argNr >= 2)) {  // write DAC -- DA n val
      arg = getArgument(cmdS, 1);
      n = arg.toInt();
      arg = getArgument(cmdS, 2);
      val = arg.toInt();
      writeDAC(n, val);
      Info();
      sendStatus(Photons);
    } else if (cmd == "HO") {  // Home() -> sample to platter, rotate to #0
      Home();
      sendStatus(Photons);
    } else if (cmd == "IA") {  // initArm()
      initArm();
      sendStatus(Photons);
    } else if (cmd == "IH") {  // initHome()
      initHome();
      sendStatus(Photons);
    } else if (cmd == "RH") {  // RotateHome()
      RotateHome();
      sendStatus(Photons);
    } else if (cmd == "IP") {  // isPlatterPos()
      if (isPlatterPos()) Serial3.println("yes");
      else Serial3.println("no");
    } else if (cmd == "DD") {  // goDeckDown()
      // if (!isDeckLow()) {
      //   if (RotateTo(0)) {
      //     if (ArmOut()) {
      //       goDeckDown();
      //       // if (!isDeckLow()) {
      //       //   goDeckDown();
      //       // }
      //     }
      //   }
      // }
      // if (isDeckLow() && isDeckOK()) {
      //   Serial3.println("Command " + cmdS + " successful");
      // } else Serial3.println("Command " + cmdS + " unsuccessful");
      Serial3.println("Do not use command " + cmdS);
      sendStatus(Photons);
    } else if (cmd == "DM") {  // read pin
      // if (!isDeckMid()) {
      //   if (RotateTo(0)) {
      //     if (ArmOut()) {
      //       if (isDeckHigh) goDeckMid(1);
      //       else if (isDeckLow()) goDeckMid(0);
      //     }
      //   }
      // }
      // if (isDeckMid() && isDeckOK()) {
      //   Serial3.println("Command " + cmdS + " successful");
      // } else Serial3.println("Command " + cmdS + " unsuccessful");
      Serial3.println("Do not use command " + cmdS);
      sendStatus(Photons);
    } else if (cmd == "DH") {  // read pin
      // goDeckHigh();
      // if (!isDeckHigh()) {
      //   if (RotateTo(0)) {
      //     if (ArmOut()) {
      //       goDeckHigh();
      //       // if (!isDeckHigh()) {
      //       //   goDeckHigh();
      //       // }
      //     }
      //   }
      // }
      // if (isDeckHigh() && isDeckOK()) {
      //   Serial3.println("Command " + cmdS + " successful");
      // } else Serial3.println("Command " + cmdS + " unsuccessful");
      Serial3.println("Do not use command " + cmdS);
      sendStatus(Photons);
    } else if (cmd == "SS") {  // send status string
      sendStatus(Photons);
    } else if (cmd == "SP") {  // sample to plate (heater)
      Busy();
      sampleToPlate();
      unBusy();
    } else if (cmd == "IN") {  // send status info
      Info();
    } else if (cmd == "RC") {  // set ramp clock ticks/period
      arg = cmdS.substring(2);
      argNr = arg.toInt();
      setRampClock(argNr);
    } else if (cmd == "NF") {  // not FLConsole
        isFLConsole = false;
        Serial3.println("switching to Arduino IDE terminal");
    } else if (cmd == "TC") {  // test counter TCNT5
      arg = cmdS.substring(2);
      val = arg.toInt();
      testCounter(val);
    } else {
      Serial3.println("Special command " + cmdS + " is not defined here.");
    }
  } else {  // who knows?
    argNr = countArgs(cmdS);
    cmd = getArgument(cmdS, 0);
    Serial3.println("Command " + cmd + " with " + String(argNr) + " argument(s) is not defined here.");
    Serial3.println("command:  " + getArgument(cmdS, 0));
    for (int i = 1; i <= argNr; i++) {
      Serial3.println("argument: " + String(i) + "  " + getArgument(cmdS, i));
    }
  }
}

// not Daybreak

void Info(void) {  // -- ok
  Serial3.println("ffffffddssrreeeeTmTtTeVgVcRvLtHVsssssssstttt");
}

void goPlatterToPos(void) {  // not Daybreak -- ok
  startPlatter();
  while (isPlatterPos())
    ;  // delay(200);
  while (!isPlatterPos())
    ;
  stopPlatter();
  Sample = (Sample + 1) % 20;
  if (isPlatterPos() && isPlatterHome()) Sample = 0;
}

void goPlatterHome(void) {  // not Daybreak -- ok
  // full rotation takes ca. 30 s, ~1.5 s per position
  stopTimer3();
  startPlatter();
  while (isPlatterPos())
    ;
  while (!(isPlatterHome() && isPlatterPos()))
    ;
  stopPlatter();
  startTimer3();
}

void initPlatter(void) {  // not Daybreak -- ok
  if (!(isPlatterHome() && isPlatterPos())) {
    goPlatterHome();
  }
  Sample = 0;
}
// END