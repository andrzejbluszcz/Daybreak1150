#ifndef Daybreak1150cmds
#define Daybreak1150cmds

// Routines 

void setDataByte(byte data);
byte getDataByte(void);
void setAddress(byte address);

extern void setRampClock(unsigned int rcTime);  // has no counterpart in 1100 firmware
extern void writeRampDiv(unsigned int rampDiv);  // firmware equivalent to setRampClock
void setupCounter(void);  // has no counterpart in 1100 firmware - R65F12 internal counter B is setup in SET-UP
void writeDAC(byte n, word val);
void write2DAC(word val1, word val2);
int* get8ADC();

void CoolOn(void);
void CoolOff(void);
void PurgeOn(void);
void PurgeOff(void);
bool checkPurge(void);
void purgeIf(bool do_purge);
void HVon(int pmtNo);
void HVoff(void);
void IrradOn(void);
void IrradOff(void);
void CalOn(void);
void CalOff(void);
void OvenOn(void);
void OvenOff(void);
void oslLEDsOn(void);
void oslLEDsOff(void);
void constCurrent(void);
void servoCurrent(void);
void BleedOn(void);
void MainOn(void);
void VacOff(void);
void AllOff(void);

byte getADC(byte num);

void sendHead(void);
void sendVHead(int point);
void sendRest(void);
void sendData(void);
void sendStatus(unsigned long int aCounts);  // not used
void testStatus(void);  // for tests
// extern void sendData(int pointNo);  // redefined
void OSLsendIfDataWaiting(void);

// Interrupt servers
// SERIAL-SERVER CHECK-OVERRUN - not used here
void incRampComp(void);
void readCounter(void);
void startCounter(void);
void storePoint(void);
void setup10msec(void);
void setup100msec(void);
void setup1sec(void);
void setup10sec(void);
void setupTB(void);
void incOSLramp(void);
void nextPoint(void);
void OSLServer(void);
void RampServer(void);
// ISR(TIMER5_CAPT_vect);  // ATmega2560 processor firmware

// 1100/1150 DISPLAY CODE
void NumberToDisp(void);
void FillDisp(void);
void ByteToDisp(byte Acc);
void StatusToDisp(void);
void DispStatus(void);

void startTimer(void);
unsigned int getTemp(void);
void startRamp(int eT4, int rS, int sT4 = 0);
void rampTest(int rS);

// Error bit ops
void setError(byte err_src);
void resetErrors(void);

// Check conditions
void CheckVacuum(void);
void TFault(void);
void HVFault(void);
void RampFault(void);
void IrradFault(void);

void ChangeFault(void);
void CheckIrradTime(void);
void CheckCalTime(void);
void CheckAll(void);

// Command words
void setTemp(int temp);
// Cool not used COOL
void getSpace(int space1);
void getEndPoint(int temp, int time);
void doRamp(int rampType);
void doHV(int pmtNo);
void doQuery(int pointNo);
void doCal(int cT);
void getCool(int temp);
// doOven not used DO-OVEN
void doPurge(int state);
// TODO: doVacuum  DO-VACUUM
void getPreheat(int temp, int time);
void getStage(int temp, int time);
void getRampRate(int rate);
void getOSLinfo(unsigned int x0, unsigned int x1, unsigned int x2, unsigned int x3); 
void setDAC2(int val);
void doLEDs(unsigned int power, unsigned int mode);
void firstPt(unsigned int fstPt);
void doOSL(unsigned int power, unsigned int mode, unsigned int fstPt);



// 1100/1150 MOTION CODE
void Busy(void);
void unBusy(void);
bool hasElevator(void);
void Chg(void);
void unChg(void);
bool isElevatorDown(void);
bool isElevatorPos1(void);
bool isElevatorPos2(void);
bool isArmPos(void);
bool isArmIn(void);
bool isArmOut(void);
bool isArmHome(void);
bool isPlatterPos(void);
bool isPlatterHome(void);
bool isArmDest(int dest_pos);
bool isElevDest(int dest_pos);
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

// OSL ramp related
void setTBtable(void);
void getTBtable(unsigned int X1, unsigned int Y1, unsigned int X2, unsigned int Y2, \
                unsigned int X3, unsigned int Y3, unsigned int X4, unsigned int Y4);
void setTB(unsigned int nT, unsigned int rD);

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