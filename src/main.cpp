// regex ^(bool|int|void)((?!-- ok).)*$ finds function definitions not ending with '-- ok'

// COM7 - Serial3 pair does not work for debugging; must use Serial0 for it
// #include "avr8-stub.h"
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
/*  end of test routines  */

// unsigned long Curve[1020];  // CURVE
// int CurvePt = -1;  // CURVEPTR  -- here index to current point in Curve[]
// int MaxPt;  // MAXPT ( # OF PTS IN GLOWCURVE FOR STORAGE )
// unsigned int rampRate, Ramp, rateCnt, dSpace4, EndPt4, RampEnd4, PhTemp4, StageTemp4, CoolTemp, HoldTime, PhTime, StageTime, calTime; 
// int PointNo, lastSent;
// bool Purging, HVdisp, OvenDisp, ElevDisp, IrradDisp, /*OSLon,*/ rampFlag, rampOn, isSetPt;

void CoolOn(void);
void CoolOff(void);
void PurgeOn(void);
void PurgeOff(void);
bool checkPurge(void);
void purgeIf(bool doPurge);
void OvenOn(void);
void OvenOff(void);
void BleedOn(void);
void MainOn(void);
void VacOff(void);
void calON(void);
void calOFF(void);
void HVon(int pmtNo);
void HVoff(void);
void AllOff(void);

// void sendData(int photons);

void writeRampDiv(unsigned int rampDiv);  // (rampDiv) RAMP-DIV C! -- writes an initial byte value to the ramp clock counter (CSRCK/)

// Specialised interrupt servers
void storePoint(void);
void incRampComp(void);
void RampServer(void);
void setup10msec(void);
void setup100msec(void);
void setup1sec(void);
void setup10sec(void);
void incOSLramp(void);
void nextPoint(void);
void OSLServer(void);

// heating & cooling
void getSpace(int space1);
void doCal(int cT);
void doQuery(int pointNo);
void doHV(int pmtNo);
void doPurge(int state);

void setTemp(int temp);
void setDAC2(int val);
void getCool(int temp);
void getEndPoint(int temp, int time);
void getPreheat(int temp, int time);
void getStage(int temp, int time);
void getRampRate(int rate);
void startRamp(int eT4, int rS, int sT4 = 0);
void doRamp(int rampType);
unsigned int getTemp(void);

// DONE: check the timinig of TL sequence with FLConsole in full dump mode
void rampTest(int rS);

// definitions of functions

void setup() {
  // COM7 - Serial3 pair doesn't work for debugging
  // initialize GDB stub
  // debug_init();

  // Daybreak 1150 reader set-up
  adcInit();
  setupATmegaPorts(1150);
  setUp();

  // Serial3.begin(9600, SERIAL_7N2);
  Serial.begin(115200, SERIAL_8N1);
  Serial3.begin(115200, SERIAL_8N1);
  delay(200);
  Serial.print("New session started"); Serial.write(0x0D); Serial.write(0x0A);
  Serial.print("  Is FLConsole? "); Serial.print(isFLConsole ? "yes" : "no"); Serial.write(0x0D); Serial.write(0x0A);
  Serial3.print("New session started"); Serial3.write(0x0D); 
  Serial3.print("  Is FLConsole? "); Serial3.print(isFLConsole ? "yes" : "no"); Serial3.write(0x0D); 
  // delay(5000);
  setupCounter();
  cOVF = 0;
  startCounter();
}

// use special command 'NF ' to switch Daybrino output to Arduino IDE terminal 

void loop() {
  // increase FLASH-COUNTER and set/reset FLAsH accordingly
  // CHECK-ALL  - check fault conditions

  if (RampSeg != rseg_OSL) {
    if (dataReady) {
      // if (isFLConsole) 
      send_Data();
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


/*  begin of test routines  */
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

ISR(TIMER5_OVF_vect) {  // not Daybreak -- ok
  // input capture interrupt service routine
  cOVF++;
}
/*  end of test routines  */


void CoolOn(void) {  // COOL-ON
  digitalWrite(pCool, LOW);
}

void CoolOff(void) {  // COOL-OFF
  digitalWrite(pCool, HIGH);
}

void PurgeOn(void) {  // PURGE-ON
  digitalWrite(pPurge, LOW);
}

void PurgeOff(void) {  // PURGE-OFF
  digitalWrite(pPurge, HIGH);
}

bool checkPurge(void) {  // CHECK-PURGE
  // check state of controlling pin and set Purging T/F
  return (digitalRead(pPurge) == LOW);
}

void purgeIf(bool doPurge) {  // PURGE-IF
  if (doPurge) PurgeOn();
}

void OvenOn(void) {  // OVEN-ON
  OvenDisp = true;
  digitalWrite(pOvenEn, LOW);
}

void OvenOff(void) {  // OVEN-OFF
  OvenDisp = false;
  digitalWrite(pOvenEn, HIGH);
}

void BleedOn(void) {  // BLEED-ON
  // TODO:  VacuumBleed
}

void MainOn(void) {  // MAIN-ON
  // TODO:  VacuumMain
}

void VacOff(void) {  // VAC-OFF
  // TODO:  VacuumBleed & VacuumMain off
}

void calON(void) {  // CAL-ON
  digitalWrite(pCalEn, LOW);
}

void calOFF(void) {  // CAL-OFF
  digitalWrite(pCalEn, HIGH);
}

void HVon(int pmtNo) {  // HV-ON
  if (pmtNo == 1) {
    HVdisp = true;
    digitalWrite(pHVen, LOW);
  }
  // TODO:  if there are more HVPS's then ...
}

void HVoff(void) {  // HV-OFF
    HVdisp = false;
    digitalWrite(pHVen, HIGH);
    // TODO:  and all other HV power supplies
}

void AllOff(void) {  // ALL-OFF
  digitalWrite(pElevEn, HIGH);
  digitalWrite(pCalEn, LOW);
  // TODO:
  // digitalWrite(pIrrad, LOW);
  // digitalWrite(pVacMain, HIGH);
  // digitalWrite(pVacBleed, HIGH);
  digitalWrite(pPurge, HIGH);
  digitalWrite(pCool, HIGH);
  digitalWrite(pOvenEn, HIGH);
  Purging = false;
  digitalWrite(pHVen, HIGH);
  digitalWrite(pArmEn, HIGH);
  digitalWrite(pArmDir, HIGH);
  digitalWrite(pPlatEn, HIGH);
}

// void sendData(int photons) {  // SEND-DATA, SEND-HEAD, SEND-REST
//   sendStatus(photons);
// }

void writeRampDiv(unsigned int rampDiv) {  // (rampDiv) RAMP-DIV C! -- writes an initial byte value to the ramp clock counter (CSRCK/)
  setDataByte(lowByte(rampDiv));
  digitalWrite(pCSRCK, LOW);  // reset CSRCK (PD2 = pCSRCK) to move initial value to ramp counter input register
  digitalWrite(pCSRCK, LOW);
  digitalWrite(pCSRCK, HIGH);
}

// Specialised interrupt servers
void storePoint(void) {  // STORE-PT
  Curve[++CurvePt] = Photons;
}

void incRampComp(void) {  // INC-RAMP-COMP
  Ramp++;  // ramp value for DAC (4 -> 1 deg C)
  if (Ramp > EndPt4) {
    Ticks = 0;
    rampOn = false;
    Time = 0;
  } else {
    writeDAC(1, Ramp);
  }
}

void RampServer(void) {  // RAMP-SERVER
  // TODO: reset WatchDog - this is a hardware DS1232 MicroMonitor Chip (U27)
  Ticks++;
  if (rampOn) incRampComp();
  if (Ticks == numTicks) {
    Time++;
    if (rampFlag) {
      writeRampDiv(rateCnt);
      rampFlag = false;
    }
    readCounter();
    if (rampOn) {
      PointNo++;
      storePoint();
    }
  }
}

void incOSLramp(void) {  // INC-OSL-RAMP
  if (oslInc > 0) {
    oslRamp += oslInc;
    writeDAC(2, oslRamp);
  }
}

void setup10msec(void) {  // SETUP10MSEC
  numTicks = highByte(divTicks10msec);
  writeRampDiv(lowByte(divTicks10msec));
  TBpoints = nr10msec;
  Ticks = 0;
}

void setup100msec(void) {  // SETUP100MSEC
  numTicks = highByte(divTicks100msec);
  writeRampDiv(lowByte(divTicks100msec));
  TBpoints = nr100msec;
  Ticks = 0;
}

void setup1sec(void) {  // SETUP1SEC
  numTicks = highByte(divTicks1sec);
  writeRampDiv(lowByte(divTicks1sec));
  TBpoints = nr1sec;
  Ticks = 0;
}

void setup10sec(void) {  // SETUP10SEC
  numTicks = highByte(divTicks10sec);
  writeRampDiv(lowByte(divTicks10sec));
  TBpoints = nr10sec;
  Ticks = 0;
}

void setupTB(void) {  // SETUPTB-F
  switch (TBindex) {
    case 0: setup10msec(); break;
    case 1: setup100msec(); break;
    case 2: setup1sec(); break;
    case 3: setup10sec(); break;
    // DONE: check the meaning of line below
    default: TBpoints = 1; break;  // nextPoint() decreases it to 0 and eventually stops OSL and sets Ticks/numTicks/RampDiv to 1000 msec
  }
}

void nextPoint(void) {  // NEXT-POINT
  TBpoints--;
  if (TBpoints == 0) {  // go to next TB
    do {
      TBindex++;
      setupTB();
    } while (TBpoints == 0);
    if (TBindex == 4) {  // OSL ramp is finished
      oslOn = false;
      // digitalWrite(pOSLen, HIGH);
      // digitalWrite(pIRen, HIGH);
      // writeExtension(0x1FEE, 0);  // external shutter off
      Time = 0;
      oslDisp = false;
      numTicks = 4;
      writeRampDiv(0x05);
      Ticks = 0;
    }
  }
}

void OSLServer(void) {  // OSL-SERVER
  // TODO: reset WatchDog - this is a hardware DS1232 MicroMonitor Chip (U27)
  Ticks++;
  // Serial.println("Ticks: " + String(Ticks) + "; numTicks: " + String(numTicks));
  if (Ticks == numTicks) {
    Time++;
    PointNo++;
    readCounter();
    storePoint();
    Serial.println("time: " + String(Time) + "; point: " + String(PointNo) + "; count: " + String(Photons));
    incOSLramp();
    nextPoint();
  }
}

// heating & cooling
void getSpace(int space1) {  // GET-SPACE
  dSpace4 = 4*min(max(space1, 1), 20);
}

void doCal(int cT) {  // DO-CAL
  if (cT == 0) {
    calOFF();
  } else if (cT > 0) {
    calTime = cT;
    timerTime = millis();
    calON();
  }
}

void doQuery(int pointNo) {  // DO-QUERY
  // DONE: write the code here
  pointNo = min(pointNo, PointNo);
  sendStatus(Curve[pointNo]);
}

void doHV(int pmtNo) {  // DO-HV
  if (pmtNo == 0) {  // all off
    HVoff();
  } else {
    HVon(pmtNo);
  }
}

void doPurge(int state) {  // DO-PURGE
  Purging = (state == 1);
  if (Purging) PurgeOn();
  else PurgeOff();
}

void setTemp(int temp) {  // SET-POINT
  Ramp = 4*temp;
  writeDAC(1, Ramp);
  isSetPt = true;
}

void setDAC2(int val) {  // SET-DAC2
  val = min(4095, max(0, val));
  writeDAC(2, val);
}

void getCool(int temp) {  // GET-COOL
  CoolTemp = temp;
}

void getEndPoint(int temp, int time) {  // GET-ENDPOINT
  // TODO: in FORTH code temp is incremented by 1 - why? Is it related to the condition ending the ramp?
  RampEnd4 = 4*max(min(temp, 700), 0);
  HoldTime = max(time, 0);
}

void getPreheat(int temp, int time) {  // GET-PREHEAT
  // TODO: in FORTH code temp is incremented by 1 - why?
  PhTemp4 = 4*max(min(temp, 700), 0);
  PhTime = max(time, 0);
}

void getStage(int temp, int time) {  // GET-STAGE
  // TODO: in FORTH code temp is incremented by 1 - why?
  StageTemp4 = 4*max(min(temp, 700), 0);
  StageTime = max(time, 0);
}

void getRampRate(int rate) {  // GET-RAMPRATE
  rampRate = max(min(rate, 25), 1);
  rateCnt = 255 - int(250.0/rampRate + 0.5);
  // effective ramp rate is 250.0/int(250.0/rampRate + 0.5) -- the difference is within +/-4.2 %
}

void startRamp(int eT4, int rS, int sT4) {  // START-RAMP
  // Serial3.println("startRamp(temp=" + String(eT4/4) + ", rampSeg=" + rampseg[rS] + ")");
  Ticks = 0;
  numTicks = dSpace4;
  RampSeg = rS;
  EndPt4 = eT4;
  writeRampDiv(rateCnt);
  Ramp = sT4;
  OvenOn();
  rampOn = true;
  Time = 0;
  startCounter();
}

void doRamp(int rampType) {  // DO-RAMP
  if (rampType == 0) {  // stop ramp
    rampOn = false;
    RampSeg = 0;
    Ramp = 0;
    writeDAC(1, Ramp);
    OvenOff();
    startTimer();
  } else {
    isSetPt = false;
    Purging = checkPurge();
    PointNo = -1;
    CurvePt = -1;
    if (PhTime > 0) {
      startRamp(PhTemp4, rseg_Preheat);  // START-PREHEAT   1
    } else {
      if (StageTime > 0) {
        startRamp(StageTemp4, rseg_StagePh);  // START-AFTER-PH    4
      } else {
        startRamp(RampEnd4, rseg_Ramp);  // START-AFTER-STAGE    6
      }
    }
  }
}

unsigned int getTemp(void) {  // GET-TEMP
  return getADC(1)*4;
}

void rampTest(int rS) {  // RAMPTEST
  switch (rS) {
    case rseg_Idle:     break;          //  SEG0TEST
    case rseg_Preheat:                  //  SEG1TEST, SEG4TEST, SEG6TEST
    case rseg_StagePh: 
    case rseg_Ramp:     if (!rampOn) {
                          startTimer();
                          RampSeg = rS + 1;
                          // Serial3.println("finished " + rampseg[rS] + " changing to " + rampseg[RampSeg]);
                        }
                        break;
    case rseg_PhHold:   if (PhTime - 1 < Time) {  //  SEG2TEST
                          PurgeOff();
                          CoolOn();
                          writeDAC(1, 0);
                          Ramp = 0;
                          OvenOff();
                          Time = 0;
                          RampSeg = rS + 1;
                          // Serial3.println("finished " + rampseg[rS] + " changing to " + rampseg[RampSeg]);
                        }
                        break;
    case rseg_PhCool:   if (getTemp() < CoolTemp) {  //  SEG3TEST
                          CoolOff();
                          purgeIf(Purging);
                          PointNo = -1;
                          if (StageTime == 0) {
                            startRamp(RampEnd4, rseg_Ramp);  // START-AFTER-STAGE
                          } else {
                            startRamp(StageTemp4, rseg_StagePh);  // START-AFTER-PH
                          }
                        }
                        break;
    case rseg_StHold:   if(StageTime - 1 < Time) {  //  SEG5TEST
                          startRamp(RampEnd4, rseg_Ramp, StageTemp4);  // START-AFTER-STAGE  modified to realise stage preheat functionality
                        }
                        break;
    case rseg_EndHold:  if (HoldTime < Time) {  //  SEG7TEST
                          rampOn = false;
                          Time = 0;
                          writeDAC(1, 0);
                          Ramp = 0;
                          OvenOff();
                          writeRampDiv(5);
                          RampSeg = rS + 1;
                          PurgeOff();
                          CoolOn();
                          // Serial3.println("finished " + rampseg[rS] + " changing to " + rampseg[RampSeg]);
                        }
                        break;
    case rseg_CoolDwn:  if (getTemp() < CoolTemp) {  //  SEG8TEST
                          CoolOff();
                          purgeIf(Purging);
                          RampSeg = rseg_Idle;
                          startTimer();
                          // Serial3.println("finished " + rampseg[rS] + " changing to " + rampseg[RampSeg]);
                        }
                        break;
    default:            break;
  }
}
