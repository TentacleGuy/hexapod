# 1 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino"
# 1 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino"
/*******************************************************************************

Created with PROGRAMINO IDE for Arduino

Project     : C-Control Hexapod "RoboBug" Firmware

Author      : UlliS

Version     : 1.0

Date        : 20.12.2017

********************************************************************************



[Basic Firmware is written by]

Project Lynxmotion Phoenix

Description: Phoenix software

Programmer: Jeroen Janssen [aka Xan]

Kurt Eckhardt(KurtE) converted to C and Arduino

Kåre Halvorsen aka Zenta - Makes everything work correctly!

Ulli Sommer - clean up, bug fixes, serial protocoll, extend and costumize for C-Control Hexapod



https://github.com/KurtE/Arduino_Phoenix_Parts



*******************************************************************************/
# 21 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino"
/*******************************************************************************

Basic Files

*******************************************************************************/
# 25 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino"
# 26 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2


# 29 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2
# 30 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2
# 31 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2
# 32 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2

/*******************************************************************************

ROBOT-MECHANIC

*******************************************************************************/
# 36 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino"
# 37 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2
//#include "HEX_CNC_3DOF_V2.h"

/*******************************************************************************

Header Files (2)

*******************************************************************************/
# 42 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino"
# 43 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2
# 44 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2
# 45 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2

/*******************************************************************************

CONTROLLER

*******************************************************************************/
# 49 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino"
// SERIAL USER-BOARD & PS2-CONTROLLER
# 51 "C:\\Users\\CTC\\Desktop\\Ulli Misc\\-- Projekte\\Jr. Ulli\\Library-Demos\\C-Control-Hexapod-V1.0\\examples\\Locomotion\\Motion-Firmware\\Motion-Firmware.ino" 2
