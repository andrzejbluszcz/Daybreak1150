#include <Arduino.h>
/* C:\Users\Andrzej\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6\cores\arduino\Arduino.h */

String Command = "";
bool commandReady = false;

const unsigned int MAX_MESSAGE_LENGTH = 65;

int countArgs(String cmdS) { // argument is anything after command separated with white space
  int i = 0;
  int k = 0;
  bool wasSep = false;
  cmdS.trim();
  while (cmdS.charAt(i) > 0) {
    if ( wasSep && !isSpace(cmdS.charAt(i)) ) {
      k++;
      wasSep = false;
    } else if ( !wasSep && isSpace(cmdS.charAt(i)) ) {
      wasSep = true;
    }
    i++;
  }
  return(k);
}

String byteToBinStr(unsigned char value, unsigned char len = 8) {
  String numS = String(value, BIN);
  while (numS.length() < len) numS = "0" + numS;
  return("0b" + numS);
}

String byteToHexStr(unsigned char value, unsigned char len = 2) {
  String numS = String(value, HEX);
  numS.toUpperCase();
  while (numS.length() < len) numS = "0" + numS;
  return("0x" + numS);
}

String wordToHexStr(unsigned int value, unsigned char len = 4) {
  String numS = String(value, HEX);
  numS.toUpperCase();
  while (numS.length() < len) numS = "0" + numS;
  return("0x" + numS);
} 

String ByteToHex(unsigned char val) {  // BYTETOHEX
  String result = String(val, HEX);
  if (result.length() < 2) result = "0" + result;
  return(result);
}

String WordToHex(unsigned int val) {
  String result = String(val, HEX);
  while (result.length() < 4) result = "0" + result;
  return(result);
}

String LongToHex(unsigned long val) {
  String result = String(val, HEX);
  while (result.length() < 8) result = "0" + result;
  return(result);
}

String PhotonsToHex(unsigned long val) {
  String result = String(val, HEX);
  while (result.length() < 6) result = "0" + result;
  return(result);
}

String statusToStr(unsigned int timeStamp) {
  // code A6-7, B0-3, C0-7, G0-2, timeStamp, status as a string of hex digits 
  const char hexDigit[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  static char result[13];
  static unsigned char statusByte = PINA & 0xC0;
  result[0] = hexDigit[statusByte >> 4];
  result[1] = hexDigit[statusByte & 0x0F];
  statusByte = PINB & 0x0F;
  result[2] = hexDigit[statusByte >> 4];
  result[3] = hexDigit[statusByte & 0x0F];
  statusByte = PINC & 0xFF;
  result[4] = hexDigit[statusByte >> 4];
  result[5] = hexDigit[statusByte & 0x0F];
  statusByte = PING & 0x07;
  result[6] = hexDigit[statusByte >> 4];
  result[7] = hexDigit[statusByte & 0x0F];
  // time
  result[8] = hexDigit[(timeStamp >> 12) & 0x0F];
  result[9] = hexDigit[(timeStamp >>  8) & 0x0F];
  result[10] = hexDigit[(timeStamp >>  4) & 0x0F];
  result[11] = hexDigit[timeStamp & 0x0F];
  result[12] = '\0';
  return(result);
}

String getArgument(String cmdS, int nArg) {
  String result = "";
  int i = 0;
  int k = 0;
  bool wasSep = false;
  cmdS.trim();
  while (cmdS.charAt(i) > 0) {
    if ( wasSep && !isSpace(cmdS.charAt(i)) ) {
      k++;
      wasSep = false;
    } else if ( !wasSep && isSpace(cmdS.charAt(i)) ) {
      wasSep = true;
    }
    if ( k == nArg ) {
      k = i;
      break;
    }
    i++;
  }
  result = cmdS.substring(k);
  k = 0;
  while ( !isSpace(result.charAt(k)) && (result.charAt(k) > 0) ) k++;
  return(result.substring(0, k));
}

bool isNumericChar(char c) {
  return ((c >= '0') && (c <= '9'));
}

bool isCommandName(char c) {
  return ((c >= '@') && (c <= '_'));
}

bool isCommandSuffix(char c) {  // command suffix is a second letter in a special command name
  return ((c >= 'A') && (c <= 'Z'));
}

bool isCommandDelim(char c) {
  return (c == ' ');
}

bool isCommandChar(char c) {
  return isCommandName(c) || isNumericChar(c) || isCommandDelim(c);
}

int commandDelims(char c) {  // the function returns a number of delimiters necessary to complete the command
  if ((c == 'B') || (c == 'M') || (c == 'Z') || (c == '_')) return 1; 
  else if ((c == '@') || (c == 'A') || (c == 'C') || (c == 'D') || (c == 'F') || (c == 'G') || (c == 'H') || (c == 'J') || (c == 'K') || (c == 'L') || (c == 'O') || (c == 'P') || (c == 'Q') || (c == 'R') || (c == 'T') || (c == 'V') || (c == 'Y')) return 1; 
  else if ((c == 'E') || (c == 'I') || (c == 'N') || (c == 'S') || (c == 'W') || (c == '[')) return 2;
  else if ((c == 'U') || (c == '^')) return 3;
  else if ((c == 'X')) return 4;
  // else if ((c == 'B')) return 8;  // for 2200
  else return -1;
}

int specialDelims(String cmd) {  // the function returns a number of delimiters necessary to complete the special (two-letter) command
  // Serial3.println(("specialDelims received cmd: '" + cmd + "'"));
  if ((cmd == "HO") || (cmd == "IA") || (cmd == "IH") || (cmd == "RH") || (cmd == "IP") || (cmd == "SS") || (cmd == "SP") || (cmd == "IN") || (cmd == "NF")) return 1; 
  else if ((cmd == "AD") || (cmd == "RC") || (cmd == "TC")) return 2;
  else if ((cmd == "DA")) return 3;
  else return -1;
}

void extractCommand(void) {
  static char message_str[MAX_MESSAGE_LENGTH];
  static unsigned int message_pos = 0;
  static int argCount = 0;
  static char cmdChar = 0x00;
  static bool isTplus = false;
  char rxChar;

  if (Serial3.available() > 0) {
    rxChar = Serial3.read();
    if (cmdChar == 0x00) {  // waiting for a new command - ignoring everything except '@'-'_'
      if (isCommandName(rxChar)) {
        cmdChar = rxChar;
        argCount = commandDelims(cmdChar);
        message_str[message_pos] = cmdChar;
        message_pos++;
        message_str[message_pos] = 0x00;
      }
    } else {  // waiting for argument or delimiter
      if (isCommandDelim(rxChar)) {
        message_str[message_pos] = rxChar;
        message_pos++;
        message_str[message_pos] = 0x00;
        argCount--;
        if (argCount <= 0) {  // a command is complete unless it is T19-T24 command or a special command
          if ((String(message_str) == "T19 ") && !isTplus) {  // needs special treatment
            isTplus = true;
            argCount = 2;
          } else if ((String(message_str) == "T20 ") && !isTplus) {
              isTplus = true;
              argCount = 1;
          } else if ((String(message_str) == "T21 ") && !isTplus) {
              isTplus = true;
              argCount = 1;
          } else if ((String(message_str) == "T22 ") && !isTplus) {
              isTplus = true;
              argCount = 2;
          } else if ((String(message_str) == "T23 ") && !isTplus) {
              isTplus = true;
              argCount = 1;
          } else if ((String(message_str) == "T24 ") && !isTplus) {
            isTplus = true;
            argCount = 2;
          } else {  // complete
            commandReady = true;
            Command = String(message_str);
            cmdChar = 0x00;
            isTplus = false;
            message_pos = 0;
            message_str[message_pos] = 0x00;
          }
        }
      }
      if (isNumericChar(rxChar)) {  // a numeric argument
        message_str[message_pos] = rxChar;
        message_pos++;
        message_str[message_pos] = 0x00;
      }
      if (isCommandSuffix(rxChar)) {
        message_str[message_pos] = rxChar;
        message_pos++;
        message_str[message_pos] = 0x00;
        argCount = specialDelims(String(message_str));
        // Serial3.println("   returned number of delimiters " + String(argCount));
      }
    }
  }
  // Serial3.println(String(message_str));
}