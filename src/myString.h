#ifndef myString
#define myString

#include <Arduino.h>
/* in C:\Users\Andrzej\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6\cores\arduino\Arduino.h */

extern String Command;
extern bool commandReady;

int countArgs(String cmdS);

String byteToBinStr(byte value, byte len = 8);
String byteToHexStr(byte value, byte len = 2);
String wordToHexStr(word value, byte len = 4);
String getArgument(String cmdS, int argN);
String ByteToHex(byte val);
String WordToHex(word val);
String LongToHex(unsigned long val);
String PhotonsToHex(unsigned long val);
String statusToStr(word timeStamp);

bool isNumericChar(char c);
bool isCommandName(char c);
bool isCommandSuffix(char c);
bool isCommandDelim(char c);
bool isCommandChar(char c);
void extractCommand(void);

#endif