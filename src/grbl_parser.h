#ifndef GRBL_PARSER_H
#define GRBL_PARSER_H

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

// Define states for the GRBL parser state machine
#define GET_GRBL_STATUS_CLOSED 0
#define GET_GRBL_STATUS_START 1
#define GET_GRBL_STATUS_WPOS_HEADER 2
#define GET_GRBL_STATUS_WPOS_DATA 3
#define GET_GRBL_STATUS_HEADER 4
#define GET_GRBL_STATUS_F_DATA 5
#define GET_GRBL_STATUS_WCO_DATA 6
#define GET_GRBL_STATUS_ALARM 7
#define GET_GRBL_STATUS_BF_DATA 8
#define GET_GRBL_STATUS_MESSAGE 9
#define GET_GRBL_STATUS_OV_DATA 10
#define GET_GRBL_STATUS_PN_DATA 11
#define GET_GRBL_STATUS_SETTINGS 12
#define STR_GRBL_BUF_MAX_SIZE 80

class Grbl {
  public:
    Grbl(); // Constructor
    bool isGrblHal = false;
    bool parseData(char c);
    bool alarmed;
    int alarmNumber;
    bool errored;
    int errorNumber;
    bool paused;
    bool homing = false;

    float wcoXYZA[4];
    float wposXYZA[4];
    float mposXYZA[4];

    float feedSpindle[2] ;
    float bufferAvailable[2] ;
    float overwritePercent[3] ;

    int jogSpeed = 100;
    double jogDistanceDisplayed = 1000;
    double jogDistanceToSendMM  = 1000;
    bool incrementalJog = false;

    char machineStatus[13];
    char pinStatus[8];
    boolean newGrblStatusReceived = false ;
    unsigned long grblHeartbeat;
    volatile boolean waitOk ;
    volatile boolean waitStatus ;

    int cntOk = 0 ;
    char grblLastMessage[STR_GRBL_BUF_MAX_SIZE] ;
    float probeResult[4] ;
    int probeResultIdx = 0;
    bool probeResultSuccess = false;
    bool grblLastMessageChanged;

    // Grbl Settings
    char grblSettingsLine[STR_GRBL_BUF_MAX_SIZE] ;
    bool grblSoftLimitsEnabled = false;
    bool grblHardLimitsEnabled = false;
    bool grblHomingEnabled = false;
    float grblHomingPulloff = 0;
    int grblMinRpm = 0;
    int grblMaxRpm = 1000;
    bool grblLaserMode = false;
    float grblMaxFeedate[3];
    float grblMaxTravel[3];
    float grblAcceleration[3];
    float grblStepsPerMM[3];
    int grblStepPulse = 10;
    int grblStepIdleDelay = 255;
    bool grblStepPortInvert[3];
    bool grblDirectionInvert[3];
    bool grblStepEnableInvert = true;
    bool grblLimitInvertPin = false;
    bool grblProbeInvertPin = false;
    int grblStatusReportMask = 0;
    float grblJunctionDeviation = 0.010;
    float grblArcTolerance = 0.002;
    bool grblReportInches = 0;
    bool grblHomingDirInver[3];
    float grblHomingFeed = 0;
    float grblHomingSeek = 0;
    int grblHomingDebounce = 0;

  private:
    void handleLastNumericField(void);
    void convertDecToBin(int Dec, boolean Bin[]);
    uint8_t getGrblPosState = GET_GRBL_STATUS_CLOSED ;
    char strGrblBuf[STR_GRBL_BUF_MAX_SIZE] ;
    uint8_t strGrblIdx ;
    uint8_t wposOrMpos ;
    uint8_t wposIdx = 0 ;
    uint8_t wcoIdx = 0 ;
    uint8_t fsIdx = 0 ;
    uint8_t bfIdx = 0 ;
    uint8_t ovIdx = 0 ;
    uint8_t lastC = 0;
};

// Constructor
Grbl::Grbl() {
    // Initialization, if any, goes here.
    // Set default values to avoid garbage on startup
    machineStatus[0] = '\0';
    pinStatus[0] = '\0';
    grblLastMessage[0] = '\0';
    grblSettingsLine[0] = '\0';
    for (int i = 0; i < 4; i++) {
        wcoXYZA[i] = 0.0;
        wposXYZA[i] = 0.0;
        mposXYZA[i] = 0.0;
        probeResult[i] = 0.0;
    }
}

// Helper function to convert string to float
float stof_grbl(const char* s){
  float rez = 0, fact = 1;
  if (*s == '-'){
    s++;
    fact = -1;
  };
  for (int point_seen = 0; *s; s++){
    if (*s == '.'){
      point_seen = 1;
      continue;
    };
    int d = *s - '0';
    if (d >= 0 && d <= 9){
      if (point_seen) fact /= 10.0f;
      rez = rez * 10.0f + (float)d;
    };
  };
  return rez * fact;
};

void Grbl::convertDecToBin(int Dec, boolean Bin[]) {
  for(int i = 7 ; i >= 0 ; i--) {
    if(pow(2, i)<=Dec) {
      Dec = Dec - pow(2, i);
      Bin[8-(i+1)] = 1;
    } else {
        Bin[8-(i+1)] = 0; // Ensure it's reset
    }
  }
}

void Grbl::handleLastNumericField(void) { // decode last numeric field
  float temp = atof (&strGrblBuf[0]) ;
  if (getGrblPosState == GET_GRBL_STATUS_WPOS_DATA && wposIdx < 4) {
      if (wposOrMpos == 'W') { wposXYZA[wposIdx] = temp; mposXYZA[wposIdx] = wposXYZA[wposIdx] + wcoXYZA[wposIdx]; }
      else { mposXYZA[wposIdx] = temp; wposXYZA[wposIdx] = mposXYZA[wposIdx] - wcoXYZA[wposIdx]; }
      wposIdx++; strGrblIdx = 0;
  } else if (getGrblPosState == GET_GRBL_STATUS_F_DATA && fsIdx < 2) {
      feedSpindle[fsIdx++] = temp; strGrblIdx = 0;
  } else if (getGrblPosState == GET_GRBL_STATUS_BF_DATA && bfIdx < 2) {
      bufferAvailable[bfIdx++] = temp; strGrblIdx = 0;
  } else if (getGrblPosState == GET_GRBL_STATUS_WCO_DATA && wcoIdx < 4) {
      wcoXYZA[wcoIdx] = temp;
      if (wposOrMpos == 'W') { mposXYZA[wcoIdx] = wposXYZA[wcoIdx] + wcoXYZA[wcoIdx]; }
      else { wposXYZA[wcoIdx] = mposXYZA[wcoIdx] - wcoXYZA[wcoIdx]; }
      wcoIdx++; strGrblIdx = 0;
  } else if (getGrblPosState == GET_GRBL_STATUS_OV_DATA && ovIdx < 3) {
      overwritePercent[ovIdx++] = temp; strGrblIdx = 0;
  }
}

// The main parsing function
bool Grbl::parseData(char c) {
  bool handleAlarm = false;
  switch (c) {
    case 'k' : if ( lastC == 'o' ) { waitOk = false; cntOk++; getGrblPosState = GET_GRBL_STATUS_CLOSED; break; } // fallthrough intended if not 'ok'
    case '\r' :
      if( getGrblPosState == GET_GRBL_STATUS_CLOSED ) {
        if (  strGrblBuf[0] == 'e' && strGrblBuf[1] == 'r' ) { errorNumber = atoi( &strGrblBuf[6]); errored = true; handleAlarm = true; memccpy( machineStatus , "Alarm" , '\0', 13); homing = false; }
        else if  ( strGrblBuf[0] == 'A' && strGrblBuf[1] == 'L' )  { alarmNumber = atoi( &strGrblBuf[6]); alarmed = true; handleAlarm = true; memccpy( machineStatus , "Alarm" , '\0', 13); homing = false; }
      }
      if( getGrblPosState == GET_GRBL_STATUS_SETTINGS ) { strGrblBuf[strGrblIdx] = 0; memccpy( grblSettingsLine , strGrblBuf , '\0', STR_GRBL_BUF_MAX_SIZE - 1); }
      getGrblPosState = GET_GRBL_STATUS_CLOSED; strGrblIdx = 0; strGrblBuf[strGrblIdx] = 0;
      break;
    case '<' : getGrblPosState = GET_GRBL_STATUS_START; strGrblIdx = 0; strGrblBuf[strGrblIdx] = 0; wposIdx = 0; grblHeartbeat = millis(); break;
    case '>' : handleLastNumericField(); if( getGrblPosState == GET_GRBL_STATUS_PN_DATA ) { memccpy( pinStatus , strGrblBuf , '\0', 8); } getGrblPosState = GET_GRBL_STATUS_CLOSED; strGrblIdx = 0; strGrblBuf[strGrblIdx] = 0; newGrblStatusReceived = true; break;
    case '|' :
       if ( getGrblPosState == GET_GRBL_STATUS_START ) { getGrblPosState = GET_GRBL_STATUS_WPOS_HEADER; memccpy( machineStatus , strGrblBuf , '\0', 13); strGrblIdx = 0; strGrblBuf[strGrblIdx] = 0; }
       else if (  getGrblPosState == GET_GRBL_STATUS_MESSAGE ) { if ( strGrblIdx < (STR_GRBL_BUF_MAX_SIZE - 1)) { strGrblBuf[strGrblIdx++] = c; strGrblBuf[strGrblIdx] = 0; } }
       else { handleLastNumericField(); getGrblPosState = GET_GRBL_STATUS_HEADER; strGrblIdx = 0; }
      break;
    case ':' :
      if ( getGrblPosState == GET_GRBL_STATUS_WPOS_HEADER ) { getGrblPosState = GET_GRBL_STATUS_WPOS_DATA; wposOrMpos = strGrblBuf[0]; strGrblIdx = 0; strGrblBuf[strGrblIdx] = 0; wposIdx = 0; }
      else if ( (getGrblPosState == GET_GRBL_STATUS_START || getGrblPosState == GET_GRBL_STATUS_CLOSED || getGrblPosState == GET_GRBL_STATUS_MESSAGE ) && strGrblIdx < (STR_GRBL_BUF_MAX_SIZE - 1) ) { strGrblBuf[strGrblIdx++] = c; strGrblBuf[strGrblIdx] = 0; }
      else if ( getGrblPosState == GET_GRBL_STATUS_HEADER ) {
        memccpy( pinStatus , "       " , '\0', 8);
        if ( strGrblBuf[0] == 'F' ) { getGrblPosState = GET_GRBL_STATUS_F_DATA; strGrblIdx = 0; fsIdx = 0; }
        else if ( strGrblBuf[0] == 'W' ) { getGrblPosState = GET_GRBL_STATUS_WCO_DATA; strGrblIdx = 0; wcoIdx = 0; }
        else if ( strGrblBuf[0] == 'B' ) { getGrblPosState = GET_GRBL_STATUS_BF_DATA; strGrblIdx = 0; bfIdx = 0; }
        else if ( strGrblBuf[0] == 'O' ) { getGrblPosState = GET_GRBL_STATUS_OV_DATA; strGrblIdx = 0; ovIdx = 0; }
        else if ( strGrblBuf[0] == 'P' ) { getGrblPosState = GET_GRBL_STATUS_PN_DATA; strGrblIdx = 0; }
      }
      break;
    case ',' : if ( getGrblPosState != GET_GRBL_STATUS_MESSAGE ) { handleLastNumericField(); } else if ( strGrblIdx < (STR_GRBL_BUF_MAX_SIZE - 1) ) { strGrblBuf[strGrblIdx++] = c; strGrblBuf[strGrblIdx] = 0; } break;
    case '[' : if( getGrblPosState == GET_GRBL_STATUS_CLOSED ) { getGrblPosState = GET_GRBL_STATUS_MESSAGE; strGrblIdx = 0; strGrblBuf[strGrblIdx] = '['; strGrblIdx++; } break;
    case '$' : if( getGrblPosState == GET_GRBL_STATUS_CLOSED ) { getGrblPosState = GET_GRBL_STATUS_SETTINGS; strGrblIdx = 0; strGrblBuf[strGrblIdx] = '$'; strGrblIdx++; } break;
    case ' ' : if( ( getGrblPosState == GET_GRBL_STATUS_MESSAGE ) && strGrblIdx < (STR_GRBL_BUF_MAX_SIZE - 1) ){ strGrblBuf[strGrblIdx++] = ' '; } break;
    case ']' : if( getGrblPosState == GET_GRBL_STATUS_MESSAGE ) { getGrblPosState = GET_GRBL_STATUS_CLOSED; strGrblBuf[strGrblIdx++] = ']'; strGrblBuf[strGrblIdx] = 0; memccpy( grblLastMessage , strGrblBuf , '\0', STR_GRBL_BUF_MAX_SIZE - 1); strGrblIdx = 0; strGrblBuf[strGrblIdx] = 0; grblLastMessageChanged = true; } break;
    default :
      if (strGrblIdx < (STR_GRBL_BUF_MAX_SIZE - 1)) {
        if ((c == '-' || (c >= '0' && c <= '9') || c == '.' || c == '=') && (getGrblPosState == GET_GRBL_STATUS_WPOS_DATA || getGrblPosState == GET_GRBL_STATUS_F_DATA || getGrblPosState == GET_GRBL_STATUS_BF_DATA || getGrblPosState == GET_GRBL_STATUS_WCO_DATA || getGrblPosState == GET_GRBL_STATUS_START || getGrblPosState == GET_GRBL_STATUS_CLOSED || getGrblPosState == GET_GRBL_STATUS_MESSAGE || getGrblPosState == GET_GRBL_STATUS_OV_DATA || getGrblPosState == GET_GRBL_STATUS_SETTINGS)) {
          strGrblBuf[strGrblIdx++] = c; strGrblBuf[strGrblIdx] = 0;
        } else if (((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) && (getGrblPosState == GET_GRBL_STATUS_START || getGrblPosState == GET_GRBL_STATUS_HEADER || getGrblPosState == GET_GRBL_STATUS_CLOSED || getGrblPosState == GET_GRBL_STATUS_MESSAGE || getGrblPosState == GET_GRBL_STATUS_WPOS_HEADER || getGrblPosState == GET_GRBL_STATUS_PN_DATA || getGrblPosState == GET_GRBL_STATUS_SETTINGS)) {
          strGrblBuf[strGrblIdx++] = c; strGrblBuf[strGrblIdx] = 0;
        }
      }
  }
  lastC = c;

  if (strstr (grblLastMessage, "PRB")) {
    char tempMsg[STR_GRBL_BUF_MAX_SIZE];
    strcpy(tempMsg, grblLastMessage);
    char *ptr = strtok(tempMsg, ":,");
    probeResultIdx = 0;
    while(ptr != NULL) {
      ptr = strtok(NULL, ":,");
      if (ptr != NULL && probeResultIdx < 4) {
          if (probeResultIdx < 3) {
              probeResult[probeResultIdx] = stof_grbl(ptr);
          } else {
              probeResultSuccess = (bool)atoi(ptr);
          }
          probeResultIdx++;
      }
    }
  }

  if (getGrblPosState == GET_GRBL_STATUS_CLOSED && strstr(grblSettingsLine, "=")) {
    char tempLine[STR_GRBL_BUF_MAX_SIZE];
    strcpy(tempLine, grblSettingsLine);
    char* setting = strtok(tempLine, "=");
    char* value = strtok(NULL, "=");
    if (setting && value) {
      int settingNum = atoi(setting + 1); // Skip '$'
      switch(settingNum) {
        case 0: grblStepPulse = atoi(value); break;
        case 1: grblStepIdleDelay = atoi(value); break;
        case 2: { int temp = atoi(value); boolean Bin[]={0,0,0,0,0,0,0,0}; convertDecToBin(temp, Bin); grblStepPortInvert[0]=Bin[7]; grblStepPortInvert[1]=Bin[6]; grblStepPortInvert[2]=Bin[5]; } break;
        case 3: { int temp = atoi(value); boolean Bin[]={0,0,0,0,0,0,0,0}; convertDecToBin(temp, Bin); grblDirectionInvert[0]=Bin[7]; grblDirectionInvert[1]=Bin[6]; grblDirectionInvert[2]=Bin[5]; } break;
        case 4: grblStepEnableInvert = (bool)atoi(value); break;
        case 5: grblLimitInvertPin = (bool)atoi(value); break;
        case 6: grblProbeInvertPin = (bool)atoi(value); break;
        case 10: grblStatusReportMask = atoi(value); isGrblHal = (grblStatusReportMask > 3); break;
        case 11: grblJunctionDeviation = atof(value); break;
        case 12: grblArcTolerance = atof(value); break;
        case 13: grblReportInches = (bool)atoi(value); break;
        case 20: grblSoftLimitsEnabled = (bool)atoi(value); break;
        case 21: grblHardLimitsEnabled = (bool)atoi(value); break;
        case 22: grblHomingEnabled = (bool)atoi(value); break;
        case 23: { int temp = atoi(value); boolean Bin[]={0,0,0,0,0,0,0,0}; convertDecToBin(temp, Bin); grblHomingDirInver[0]=Bin[7]; grblHomingDirInver[1]=Bin[6]; grblHomingDirInver[2]=Bin[5]; } break;
        case 24: grblHomingFeed = atof(value); break;
        case 25: grblHomingSeek = atof(value); break;
        case 26: grblHomingDebounce = atoi(value); break;
        case 27: grblHomingPulloff = atof(value); break;
        case 30: grblMaxRpm = atoi(value); break;
        case 31: grblMinRpm = atoi(value); break;
        case 32: grblLaserMode = (bool)atoi(value); break;
        case 100: grblStepsPerMM[0] = atof(value); break;
        case 101: grblStepsPerMM[1] = atof(value); break;
        case 102: grblStepsPerMM[2] = atof(value); break;
        case 110: grblMaxFeedate[0] = atof(value); break;
        case 111: grblMaxFeedate[1] = atof(value); break;
        case 112: grblMaxFeedate[2] = atof(value); break;
        case 120: grblAcceleration[0] = atof(value); break;
        case 121: grblAcceleration[1] = atof(value); break;
        case 122: grblAcceleration[2] = atof(value); break;
        case 130: grblMaxTravel[0] = atof(value); break;
        case 131: grblMaxTravel[1] = atof(value); break;
        case 132: grblMaxTravel[2] = atof(value); break;
      }
    }
    grblSettingsLine[0] = '\0'; // Clear after parsing
  }

  if (!handleAlarm && newGrblStatusReceived) {
    if (strcmp(machineStatus, "Idle") == 0) { alarmed = false; paused = false; homing = false; }
    else if (strcmp(machineStatus, "Run") == 0) { alarmed = false; paused = false; homing = false; }
    else if (strncmp(machineStatus, "Hold", 4) == 0) { alarmed = false; paused = true; homing = false; if(strcmp(machineStatus, "Hold:0") != 0) {handleAlarm = true;} }
    else if (strcmp(machineStatus, "Jog") == 0) { alarmed = false; paused = false; homing = false; }
    else if (strcmp(machineStatus, "Alarm") == 0) { alarmed = true; paused = false; homing = false; handleAlarm = true; }
    else if (strncmp(machineStatus, "Door", 4) == 0) { alarmed = false; paused = true; homing = false; handleAlarm = true; }
    else if (strcmp(machineStatus, "Check") == 0) { alarmed = false; paused = false; homing = false; }
    else if (strcmp(machineStatus, "Home") == 0) { alarmed = false; paused = false; homing = true; }
    else if (strcmp(machineStatus, "Sleep") == 0) { alarmed = false; paused = false; homing = false; }
  }
  return handleAlarm;
}

#endif // GRBL_PARSER_H
