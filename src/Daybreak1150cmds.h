#ifndef Daybreak1150cmds
#define Daybreak1150cmds

extern int Sample, RampSeg;
extern bool changeDisp;
extern unsigned long Time;
extern unsigned long timerTime;
extern word Errors;
// isBusy is set/reset in initHome and should also be set/reset around each of @-_ commands
extern bool isBusy;
extern bool dataReady;
extern bool isFLConsole;
extern unsigned long Photons;
extern unsigned int Ticks, numTicks, lastMSEC, periodRampClock;  // to set Ramp Clock period in msecs
// extern unsigned long Curve[1024];  // CURVE
extern int CurvePt;  // CURVEPTR  -- here index to current point in Curve[]
extern int MaxPt;  // MAXPT ( # OF PTS IN GLOWCURVE FOR STORAGE )
extern unsigned int rampRate, Ramp, rateCnt, dSpace4, EndPt4, RampEnd4, PhTemp4, StageTemp4, CoolTemp, HoldTime, PhTime, StageTime, calTime; 
extern int PointNo, lastSent;
extern bool Purging, HVdisp, OvenDisp, OSLon, rampFlag, rampOn, isSetPt;

// Errors handling -- err_src
const byte T_err = 0;
const byte HV_err = 1;
const byte Ramp_err = 2;
const byte Irrad_err = 3;
const byte Elev_err = 4;
const byte Pos_err = 5;
const byte Stuck_err = 6;
const byte Sync_err = 7;
const byte Arm_err = 8;
const byte Ser_err = 9;
const byte Buf_err = 10;
const byte Deck_err = 11;
const byte X862_err = 12;

// generalized ramp stage numbers
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

extern String rampseg[11];

void setDataByte(byte data);
byte getDataByte(void);
void setAddress(byte address);
byte getADC(byte num);
int* get8ADC();
void writeDAC(byte n, word val);
void write2DAC(word val1, word val2);
void setRampClock(unsigned int rcTime);  // has no counterpart in 1100 firmware
extern void writeRampDiv(unsigned int rampDiv);  // firmware equivalent to setRampClock
void setupCounter(void);  // has no counterpart in 1100 firmware - R65F12 internal counter B is setup in SET-UP
void readCounter(void);
void startCounter(void);

void setError(byte err_src);
void resetErrors(void);
void startTimer(void);

void sendStatus(unsigned long int aCounts);
void testStatus(void);
extern void sendData(void);

// 1100/1150 MOTION CODE
void Busy(void);
void unBusy(void);
void Chg(void);
void unChg(void);
bool isArmPos(void);
bool isArmIn(void);
bool isArmOut(void);
bool isArmHome(void);
bool isPlatterPos(void);
bool isPlatterHome(void);
bool isArmDest(int dest_pos);
void PlatterWait(void);
void ArmWait(void);
void setArmDirIn(void);
void setArmDirOut(void);
void startArm(void);
void stopArm(void);
void startPlatter(void);
void stopPlatter(void);
bool isNotLost(int pos);
bool isOKtoMove(void);
bool goArmIn(void);
bool goArmOut(void);
bool goArmHome(void);
bool goArmTo(int pos);
bool ArmMove(int dest, int dir);
bool Rotate1(bool check);
bool RotateTo(int pos);
bool ArmHome(void);
bool ArmIn(void);
bool ArmOut(void);
bool sampleToPlate(void);
bool sampleToPlatter(void);
bool isDeckOK(void);
bool isDeckLow(void);
bool isDeckMid(void);
bool isDeckHigh(void);
bool isDeckAt(int pos);
void goDeckDown(void);
void goDeckMid(int dir);
void goDeckHigh(void);
void Home(void);
bool Advance1(void);
bool AdvanceTo(int pos);
void tillArmPos(void);
bool ArmToPos(int dir);
int whereArm(void);
bool ArmRecover(void);
bool initArm(void);
bool RotateHome(void);
void initHome(void);
void setUp(void);
// mechanics tests 1150 (T-commands)
void inToPos(void);
void outToPos(void);
void toPlatPos(void);
void Rot_1(void);
void Ain(void);
void Aout(void);
void Ahome(void);
void doTestCommand(String cmdS);
void doCommand(String cmdS);
void processCmd(String cmdS);  // with non-Daybreak special commands

extern void CoolOn(void);
extern void CoolOff(void);
extern void OvenOn(void);
extern void OvenOff(void);
extern void AllOff(void);
extern void getSpace(int space1);
extern void setTemp(int temp);
extern void getCool(int temp);
extern void getEndPoint(int temp, int time);
extern void getStage(int temp, int time);
extern void getPreheat(int temp, int time);
extern void getRampRate(int rate);

extern void doPurge(int state);
extern void doHV(int pmtNo);
extern void doQuery(int pointNo);
extern void doCal(int cT);
extern void doRamp(int rampType);
extern unsigned int getTemp(void);

extern void RampServer(void);

// not Daybreak
extern void stopTimer3(void);
extern void startTimer3(void);
extern void setRampClock(unsigned int rcTime);
extern void testCounter(long unsigned int dTime);
void Info(void);
void goPlatterToPos(void);
void goPlatterHome(void);
void initPlatter(void);

#endif