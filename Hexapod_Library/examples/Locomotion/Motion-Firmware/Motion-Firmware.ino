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

/*******************************************************************************
Basic Files
*******************************************************************************/
#if ARDUINO>99
    #include <Arduino.h>
#else
#endif
#include <Wire.h>
#include <EEPROM.h>
#include "I2CEEProm.h"
#include "ServoEx.h"

/*******************************************************************************
ROBOT-MECHANIC
*******************************************************************************/
#include "HEX_3DOF_USER.h"
//#include "HEX_CNC_3DOF_V2.h"

/*******************************************************************************
Header Files (2)
*******************************************************************************/
#include "Phoenix.h"
#include "phoenix_driver_ServoEx.h"
#include "Phoenix_Code.h"

/*******************************************************************************
CONTROLLER
*******************************************************************************/
// SERIAL USER-BOARD & PS2-CONTROLLER
#include "Data_Input.h"





