/*******************************************************************************
Created with PROGRAMINO IDE for Arduino
Libraries   :
Author      :
Date        :
Description : User-Body
*******************************************************************************/
#ifndef HEX_3DOF_USER
#define HEX_3DOF_USER

#define OPT_TERMINAL_MONITOR
#ifdef OPT_TERMINAL_MONITOR             // turning off terminal monitor will turn these off as well...
    #define OPT_FIND_SERVO_OFFSETS      // Only useful if terminal monitor is enabled
#endif

#define DEFINE_HEX_GLOBALS

#define OPT_GPPLAYER

// "Smooth division" factor for the smooth control function, a value of 3 to 5 is most suitable
#define SmDiv                   4

#define OPT_SINGLELEG

#define WALK_HEIGHT_NORMAL      40
#define WALK_HEIGHT_DOUBLE      100

#define SPEED_MIN               200
#define SPEED_MAX               25
#define SPEED_INIT              100

// Also define default BalanceDelay
#ifndef BALANCE_DELAY
#define BALANCE_DELAY 100
#endif

// Other values than 6 can be used, testing...CAUTION!! At your own risk ;)
#define BalanceDivFactor        6

// DEBUG
#define DBGSerial               Serial
#define DebugBaudrate           38400

/*******************************************************************************
Pin Numbers
*******************************************************************************/
#define LIVE_LED        22
#define USER_LED        69
#define JUMPER_J7       4
#define SOUND_PIN       46
#define UB_ADC          A3
#define PS2_DAT         28
#define PS2_CMD         30
#define PS2_SEL         32
#define PS2_CLK         31
#define SA0             A0
#define SA1             A1
#define SA2             A2
#define SA3             A4
#define SA4             A5
#define SA5             A6
#define IN1             24
#define OUT1            23

/*******************************************************************************
PIN & REVERSE
Servo pin defines and inverse function
*******************************************************************************/
// --> LEFT
#define cLFCoxaPin      36   // S15
#define cLFFemurPin     37   // S16
#define cLFTibiaPin     40   // S17
#define cLFCoxaInv      1
#define cLFFemurInv     1
#define cLFTibiaInv     1

#define cLMCoxaPin      33   // S12
#define cLMFemurPin     34   // S13
#define cLMTibiaPin     35   // S14
#define cLMCoxaInv      1
#define cLMFemurInv     1
#define cLMTibiaInv     1

#define cLRCoxaPin      25   // S9
#define cLRFemurPin     26   // S10
#define cLRTibiaPin     27   // S11
#define cLRCoxaInv      1
#define cLRFemurInv     1
#define cLRTibiaInv     1

// --> RIGHT
#define cRFCoxaPin      6    // S6
#define cRFFemurPin     7    // S7
#define cRFTibiaPin     8    // S8
#define cRFCoxaInv      0
#define cRFFemurInv     0
#define cRFTibiaInv     0

#define cRMCoxaPin      3    // S3
#define cRMFemurPin     17   // S4
#define cRMTibiaPin     16   // S5
#define cRMCoxaInv      0
#define cRMFemurInv     0
#define cRMTibiaInv     0

#define cRRCoxaPin      12   // S0
#define cRRFemurPin     5    // S1
#define cRRTibiaPin     2    // S2
#define cRRCoxaInv      0
#define cRRFemurInv     0
#define cRRTibiaInv     0

/*******************************************************************************
MIN-MAX ANGLES
*******************************************************************************/
#define cCoxaMin        -300        // Mechanical limits of the legs
#define cCoxaMax        300
#define cFemurMin       -1050
#define cFemurMax       750
#define cTibiaMin       -530
#define cTibiaMax       900

// --> LEFT
#define cLFCoxaMin1     cCoxaMin    // Mechanical limits of the Left Front Leg
#define cLFCoxaMax1     cCoxaMax
#define cLFFemurMin1    cFemurMin
#define cLFFemurMax1    cFemurMax
#define cLFTibiaMin1    cTibiaMin
#define cLFTibiaMax1    cTibiaMax

#define cLMCoxaMin1     cCoxaMin    // Mechanical limits of the Left Middle Leg
#define cLMCoxaMax1     cCoxaMax
#define cLMFemurMin1    cFemurMin
#define cLMFemurMax1    cFemurMax
#define cLMTibiaMin1    cTibiaMin
#define cLMTibiaMax1    cTibiaMax

#define cLRCoxaMin1     cCoxaMin    // Mechanical limits of the Left Rear Leg
#define cLRCoxaMax1     cCoxaMax
#define cLRFemurMin1    cFemurMin
#define cLRFemurMax1    cFemurMax
#define cLRTibiaMin1    cTibiaMin
#define cLRTibiaMax1    cTibiaMax

// --> RIGHT
#define cRFCoxaMin1     cCoxaMin    // Mechanical limits of the Right Front Leg
#define cRFCoxaMax1     cCoxaMax
#define cRFFemurMin1    cFemurMin
#define cRFFemurMax1    cFemurMax
#define cRFTibiaMin1    cTibiaMin
#define cRFTibiaMax1    cTibiaMax

#define cRMCoxaMin1     cCoxaMin    // Mechanical limits of the Right Middle Leg
#define cRMCoxaMax1     cCoxaMax
#define cRMFemurMin1    cFemurMin
#define cRMFemurMax1    cFemurMax
#define cRMTibiaMin1    cTibiaMin
#define cRMTibiaMax1    cTibiaMax

#define cRRCoxaMin1     cCoxaMin    // Mechanical limits of the Right Rear Leg
#define cRRCoxaMax1     cCoxaMax
#define cRRFemurMin1    cFemurMin
#define cRRFemurMax1    cFemurMax
#define cRRTibiaMin1    cTibiaMin
#define cRRTibiaMax1    cTibiaMax

/*******************************************************************************
LEG DIMENSIONS
Universal dimensions for each leg in mm
*******************************************************************************/
#define cXXCoxaLength     51
#define cXXFemurLength    75
#define cXXTibiaLength    117

// --> LEFT
#define cLFCoxaLength     cXXCoxaLength     // Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength

#define cLMCoxaLength     cXXCoxaLength     // Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength

#define cLRCoxaLength     cXXCoxaLength     // Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength

// --> RIGHT
#define cRFCoxaLength     cXXCoxaLength     // Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength

#define cRMCoxaLength     cXXCoxaLength     // Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength

#define cRRCoxaLength     cXXCoxaLength     // Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength

/*******************************************************************************
BODY DIMENSIONS
45° setup
*******************************************************************************/
#define cRRCoxaAngle1   -450    // Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0      // Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    450    // Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -450   // Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0      // Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    450    // Default Coxa setup angle, decimals = 1

#define cRROffsetX      -40     // Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      100     // Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX      -55     // Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ      0       // Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX      -40     // Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -100    // Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      40      // Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      100     // Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX      55      // Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ      0       // Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX      40      // Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -100    // Distance Z from center of the body to the Left Front coxa

/*******************************************************************************
START POSITIONS FEET
********************************************************************************/
#define cHexInitXZ      110    // 110 distance coax servo to tibia end (ground contact)
#define cHexInitXZCos   cos(45) * cHexInitXZ
#define cHexInitXZSin   sin(45) * cHexInitXZ
#define cHexInitY       35     // Center femur servo after start!

/*******************************************************************************
INT & GOALS
Lets try some multi leg positions depending on height settings
*******************************************************************************/
#define CNT_HEX_INITS       3
#define MAX_BODY_Y          135             // max body height
#ifdef DEFINE_HEX_GLOBALS
    const byte g_abHexIntXZ[] PROGMEM = {cHexInitXZ, 125, 125};
    const byte g_abHexMaxBodyY[] PROGMEM = {20, 50, MAX_BODY_Y};
#else
    extern const byte g_abHexIntXZ[] PROGMEM;
    extern const byte g_abHexMaxBodyY[] PROGMEM;
#endif

// --> LEFT
#define cLFInitPosX     cHexInitXZCos           // Start positions of the Left Front leg
#define cLFInitPosY     cHexInitY
#define cLFInitPosZ     -cHexInitXZSin

#define cLMInitPosX     cHexInitXZ              // Start positions of the Left Middle leg
#define cLMInitPosY     cHexInitY
#define cLMInitPosZ     0

#define cLRInitPosX     cHexInitXZCos           // Start positions of the Left Rear leg
#define cLRInitPosY     cHexInitY
#define cLRInitPosZ     cHexInitXZSin

// --> RIGHT
#define cRFInitPosX     cHexInitXZCos           // Start positions of the Right Front leg
#define cRFInitPosY     cHexInitY
#define cRFInitPosZ     -cHexInitXZSin

#define cRMInitPosX     cHexInitXZ              // Start positions of the Right Middle leg
#define cRMInitPosY     cHexInitY
#define cRMInitPosZ     0

#define cRRInitPosX     cHexInitXZCos           // Start positions of the Right Rear leg
#define cRRInitPosY     cHexInitY
#define cRRInitPosZ     cHexInitXZSin

#endif

