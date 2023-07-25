// Arduino MEGA2560 programming port at 115200
//
// There is no DAC on this board!
// port F (ADC's) is used as digital I/O data D0-7 lines to read from and write to various IC's
// there are 44 digital input/outputs

#ifndef mega2560
#define mega2560

const unsigned int D2 = 2;   // PE4 PWM0 8 bit resolution / INT0
const unsigned int D3 = 3;   // PE5 PWM1 8 bit resolution / INT1
const unsigned int D4 = 4;   // PG5 PWM2 8 bit resolution
const unsigned int D5 = 5;   // PE3 PWM3 8 bit resolution
const unsigned int D6 = 6;   // PH3 PWM4 8 bit resolution
const unsigned int D7 = 7;   // PH4 PWM5 8 bit resolution
const unsigned int D8 = 8;   // PH5 PWM6 8 bit resolution
const unsigned int D9 = 9;   // PH6 PWM7 8 bit resolution
const unsigned int D10 = 10; // PB4 PWM8 8 bit resolution
const unsigned int D11 = 11; // PB5 PWM9 8 bit resolution
const unsigned int D12 = 12; // PB6 PWM10 8 bit resolution
const unsigned int D13 = 13; // PB7 PWM11 8 bit resolution; builtin LED
const unsigned int D14 = 14; // PJ1 / TX3
const unsigned int D15 = 15; // PJ0 / RX3
const unsigned int D16 = 16; // PH1 / TX2
const unsigned int D17 = 17; // PH0 / RX2
const unsigned int D18 = 18; // PD3 / TX1 / INT5
const unsigned int pCSCNT = 18; // PD3
const unsigned int D19 = 19; // PD2 / RX1 / INT4
const unsigned int pCSRCK = 19; // PD2
const unsigned int D20 = 20; // PD1 / SDA / INT3
const unsigned int pCSDAC2 = 20; // PD1
const unsigned int D21 = 21; // PD0 / SCL / INT2
const unsigned int pCSDAC1 = 21; // PD0
const unsigned int D22 = 22; // PA0
const unsigned int pA0 = 22; // PA0
const unsigned int D23 = 23; // PA1
const unsigned int pA1 = 23; // PA1
const unsigned int D24 = 24; // PA2
const unsigned int pA2 = 24; // PA2
const unsigned int D25 = 25; // PA3
const unsigned int pA3 = 25; // PA3
const unsigned int D26 = 26; // PA4
const unsigned int pCSADC = 26; // PA4
const unsigned int D27 = 27; // PA5
const unsigned int pElevDir = 27; // PA5
const unsigned int D28 = 28; // PA6
const unsigned int pArmDir = 28; // PA6
const unsigned int D29 = 29; // PA7
const unsigned int pArmEn = 29; // PA7
const unsigned int D30 = 30; // PC7
const unsigned int pArmOut = 30; // PC7
const unsigned int D31 = 31; // PC6
const unsigned int pArmHom = 31; // PC6
const unsigned int D32 = 32; // PC5
const unsigned int pArmIn = 32; // PC5
const unsigned int D33 = 33; // PC4
const unsigned int pArmOK = 33; // PC4
const unsigned int D34 = 34; // PC3
const unsigned int pPlatEn = 34; // PC3
const unsigned int D35 = 35; // PC2
const unsigned int pPlatOK = 35; // PC2
const unsigned int D36 = 36; // PC1
const unsigned int pPlatHom = 36; // PC1
const unsigned int D37 = 37; // PC0
const unsigned int pAlfBet = 37; // PC0
const unsigned int D38 = 38; // PD7
const unsigned int D39 = 39; // PG2
const unsigned int pDeckHi = 39; // PG2
const unsigned int D40 = 40; // PG1
const unsigned int pDeckMi = 40; // PG1
const unsigned int D41 = 41; // PG0
const unsigned int pDeckLo = 41; // PG0
const unsigned int D42 = 42; // PL7
const unsigned int pOvenEn = 42; // PL7
const unsigned int D43 = 43; // PL6
const unsigned int pCool = 43; // PL6
const unsigned int D44 = 44; // PL5
const unsigned int pPurge = 44; // PL5
const unsigned int D45 = 45; // PL4
const unsigned int pHVen = 45; // PL4
const unsigned int D46 = 46; // PL3
const unsigned int pCalEn = 46; // PL3
const unsigned int D47 = 47; // PL2 - TCNT5 - input
const unsigned int D48 = 48; // PL1 - ICP5 - input (rising edge instead of R65F12 PA0)
const unsigned int D49 = 49; // PL0
const unsigned int clrPC = 49; // PL0 - clear Photon Counter - output (low instead of R65F12 PA3)
const unsigned int D50 = 50; // PB3
const unsigned int pHasElev = 50; // PB3
const unsigned int D51 = 51; // PB2
const unsigned int pDeckOK = 51; // PB2
const unsigned int D52 = 52; // PB1
const unsigned int D53 = 53; // PB0
const unsigned int pElevEn = 53; // PB0
const unsigned int offsetADC = 54; // there are 8 ADConverters PF0-PF7 >> ADC0-ADC7 (PK0-PK7 are used as digital I/O data lines)
const unsigned int offsetPWM =  2; // there are 12 PWM0..PWM11 converters on digital pins D2..D13

#endif