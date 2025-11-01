#include <Arduino.h>
// global variables for Daybrino 1150

// from main.cpp
unsigned long Curve[1020];  // CURVE
int CurvePt = -1;  // CURVEPTR  -- here index to current point in Curve[]
int MaxPt;  // MAXPT ( # OF PTS IN GLOWCURVE FOR STORAGE )
unsigned int rampRate, Ramp, rateCnt, dSpace4, EndPt4, RampEnd4, PhTemp4, StageTemp4, CoolTemp, HoldTime, PhTime, StageTime, calTime; 
int PointNo, lastSent;
bool Purging, HVdisp, OvenDisp, ElevDisp, IrradDisp, /*OSLon,*/ rampFlag, rampOn, isSetPt;

// from Daybreak1150cmds.cpp
int RampSeg = 0;
int Sample = 0;
bool changeDisp = false;
unsigned long Time = 0;
unsigned long timerTime = 0;
unsigned int Errors = 0;
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

unsigned char Disp[4];  // DSP
unsigned char Segs[10] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6};  // SEGS

String rampseg[11] = {"rseg_Idle", "rseg_Preheat", "rseg_PhHold", "rseg_PhCool", "rseg_StagePh", "rseg_StHold", "rseg_Ramp", "rseg_EndHold", "rseg_CoolDwn", "n.a.", "rseg_OSL"};
