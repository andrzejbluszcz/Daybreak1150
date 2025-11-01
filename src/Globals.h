#include <Arduino.h>
// declaration of global variables for Daybrino 1150

// Errors handling -- err_src - not Daybreak
const unsigned char T_err = 0;
const unsigned char HV_err = 1;
const unsigned char Ramp_err = 2;
const unsigned char Irrad_err = 3;
const unsigned char Elev_err = 4;
const unsigned char Pos_err = 5;
const unsigned char Stuck_err = 6;
const unsigned char Sync_err = 7;
const unsigned char Arm_err = 8;
const unsigned char Ser_err = 9;
const unsigned char Buf_err = 10;
const unsigned char Deck_err = 11;
const unsigned char X862_err = 12;

// Generalized ramp segment numbers
const int rseg_Idle = 0;
const int rseg_Preheat = 1;
const int rseg_PhHold = 2;
const int rseg_PhCool = 3;
const int rseg_StagePh = 4;
const int rseg_StHold = 5;
const int rseg_Ramp = 6;
const int rseg_EndHold = 7;
const int rseg_CoolDwn = 8;
const int rseg_OSL = 10;

extern unsigned long Curve[1020];
extern unsigned long Time, timerTime, Photons;
extern int CurvePt, MaxPt, RampSeg, Sample;
extern unsigned int Errors;
extern unsigned int Ticks, numTicks, lastMSEC, periodRampClock;
extern unsigned int rampRate, Ramp, rateCnt, dSpace4, EndPt4, RampEnd4, PhTemp4, StageTemp4, CoolTemp, HoldTime, PhTime, StageTime, calTime; 
extern int PointNo, lastSent;
extern bool Purging, rampFlag, rampOn, isSetPt;
extern bool isBusy, dataReady, isFLConsole;

// OSL ramp related
extern unsigned int divTicks10msec, divTicks100msec, divTicks1sec, divTicks10sec;  // RAMP-DIV IN HIGH BYTE, NUMTICKS LOW
extern int oslInc, oslRamp;
extern int nr10msec, nr100msec, nr1sec, nr10sec;
extern int TBindex, TBpoints, startTBindex;
extern bool oslOn;

// related to reader status display
extern unsigned char Disp[4];  // DSP
extern unsigned char Segs[10];  // SEGS
extern bool HVdisp, OvenDisp, ElevDisp, IrradDisp, oslDisp, changeDisp;

// not Daybreak
extern String rampseg[11];

