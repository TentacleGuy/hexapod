/*******************************************************************************
Created with PROGRAMINO IDE for Arduino
Libraries   : Arduino.h, PS2X_lib.h
Author      : UlliS
Date:       : 20.17.2017
Description : Hexapod Gamepad and User-Board communication
*******************************************************************************/
#ifndef __DATA_INPUT_H__
#define __DATA_INPUT_H__

#if ARDUINO>99
    #include <Arduino.h> // Arduino 1.0
#else
    #include <Wprogram.h> // Arduino 0022
#endif

// PS2 CONTROLLER
#define PS2_CONTROLLER
#ifdef PS2_CONTROLLER
    #include "PS2X_lib.h"
#endif

// COMMAND UART-INTERFACE
// Select only one COMMAND Interface (FTDI or User-Board)

//#define FTDI_USB
#ifdef FTDI_USB                 // FTDI USB PRG-M
    #define SERIAL_BAUD         38400
    #define SERIAL_CMD          Serial
#endif

#define USER_BOARD
#ifdef USER_BOARD               // User-Board
    #define SERIAL_BAUD         38400
    #define SERIAL_CMD          Serial1
#endif

// SERIAL PROTOCOLL [RX]
#define CMD_SYNC0               33
#define CMD_SYNC1               42
#define CMD_TERM_BYTE           255
int cmd_crc = 0;
int cmd     = 0;
int cmd_param[4];

// SEND DATA [TX]
int cmd_status_byte = 0;
int cmd_tx          = 0;
int cmd_tx_param[6];

// CMD REGISTER [WRITE-VALUES]
#define CMD_REG_POWER           5
#define CMD_REG_SPEED           10
#define CMD_REG_GAINT_MODE      15
#define CMD_REG_BALANCE_MODE    20
#define CMD_REG_BODY_HEIGHT     25
//#define CMD_REG_WALK_MODE       30
#define CMD_REG_TRANSLATE       35
#define CMD_REG_WALK            40
#define CMD_REG_ROTATE          45
#define CMD_REG_DOUBLE_HEIGHT   50
#define CMD_REG_DOUBLE_LENGTH   55
#define CMD_REG_SINGLE_LEG_POS  60
#define CMD_REG_SOUND           65
#define CMD_REG_OUT1            70
#define CMD_REG_STATUS_LED      75

// CMD-REGISTER [READ-VALUES]
#define CMD_REG_SA_LEG          100
#define CMD_REG_AKKU            105
#define CMD_REG_PS2_ACTIVE      110
#define CMD_REG_IS_WALKING      115
#define CMD_REG_IS_POWER_ON     120
#define CMD_REG_READ_PS2_VALUES 125
#define CMD_REG_IN1             130

// CMD-REGISTER [HW-RESET]
#define CMD_REG_RESET           255

// CMD STATUS FEEDBACK
#define STATUS_ACK_OK           64
#define STATUS_ERR_TERM         1
#define STATUS_ERR_STATE        2
#define STATUS_ERR_CRC          3
#define STATUS_ERR_CMD          255

// MOVE MODES
#define WALKMODE                0
#define TRANSLATEMODE           1
#define ROTATEMODE              2
#define SINGLELEGMODE           3

#define BALANCEMODE_ON          1
#define BALANCEMODE_OFF         0

// PS2-CONTROLLER
#define cTravelDeadZone         4      // The deadzone for the analog input from the remote

// SERIAL CMD BEEP
//#define SERIAL_CMD_BEEP

/*******************************************************************************
Global - Local to this file only...
*******************************************************************************/
#ifdef PS2_CONTROLLER
    PS2X ps2x; // Create PS2-Controller Class
#endif

// Define an instance of the Input Controller...
InputController g_InputController; // Our Input controller

static short g_BodyYOffset;
static short g_BodyYShift;
static byte ControlMode;
static bool DoubleHeightOn;
static bool DoubleTravelOn;
static bool WalkMethod;

// Some external or forward function references
extern void InputControllerTurnRobotOff(void);

// Some external or forward function references
boolean CheckForSerialData(void);
boolean HandleCmd(void);
boolean CheckRxCrc(void);
void SendData(byte _status, byte _cmd, byte _data0, byte _data1, byte _data2, byte _data3, byte _data4, byte _data5);
boolean ConfigBluetooth(void);

byte lx       = 128;
byte ly       = 128;
byte lz       = 128;
byte ly_shift = 128;
byte sl_nr    = 0; // Leg nr.

boolean fAdjustLegPositions = false;
boolean PS2_Active          = false;

/*******************************************************************************
This function is called by the main code to tell us when it is about to
do a lot of bit-bang outputs and it would like us to minimize any interrupts
that we do while it is active...
*******************************************************************************/
void InputController::AllowControllerInterrupts(boolean fAllow)
{
    // We don't need to do anything...
}

/*******************************************************************************
INIT
*******************************************************************************/
void InputController::Init(void)
{
    SERIAL_CMD.begin(SERIAL_BAUD);
    while(!SERIAL_CMD)
    {
        ; // Wait for serial port to connect. Needed for native USB port only
    }

    // PS2-CONTROLLER
    #ifdef PS2_CONTROLLER
        if(digitalRead(JUMPER_J7))
        {
            int error;
            PS2_Active = false;
            error      = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT); // Setup gamepad (clock, command, attention, data) pins

            #ifdef DBGSerial
                DBGSerial.print("PS2 Init: ");
                DBGSerial.println(error, DEC);
            #endif
        }
    #endif

    lx       = 128;
    ly       = 128;
    lz       = 128;
    ly_shift = 128;
    sl_nr    = 0; // Leg nr.

    g_BodyYOffset       = 0;
    g_BodyYShift        = 0;
    fAdjustLegPositions = false;

    ControlMode    = WALKMODE;
    DoubleHeightOn = false;
    DoubleTravelOn = false;
    WalkMethod     = false; // Walkomode 1

    g_InControlState.fRobotOn     = false;
    g_InControlState.SpeedControl = SPEED_INIT;
    g_InControlState.BalanceMode  = false;

    // OUTPUT Pins
    pinMode(USER_LED,OUTPUT);
    digitalWrite(USER_LED,LOW);

    pinMode(OUT1,OUTPUT);
    digitalWrite(OUT1,LOW);

    // INPUT Pins
    pinMode(IN1,INPUT);
    digitalWrite(IN1,HIGH);

    pinMode(JUMPER_J7, INPUT);
    digitalWrite(JUMPER_J7,HIGH);

    // Send data (Init finished)
    SERIAL_CMD.write(64);
    SERIAL_CMD.write(CMD_TERM_BYTE);
}

/*******************************************************************************
READ SERIAL DATA
*******************************************************************************/
boolean CheckForSerialData(void)
{
    // Recieved data frame from User-Board
    // [SYNC0],[SYNC1],[CRC],[CMD],[PARAM0],[PARAM1],[PARAM2],[PARAM3],[TERMINATION_BYTE]

    enum DATA_STATE
    {
        WAIT_FOR_SYNC_0,
        WAIT_FOR_SYNC_1,
        GET_CRC,
        GET_CMD,
        GET_PARAM0,
        GET_PARAM1,
        GET_PARAM2,
        GET_PARAM3,
        GET_TERM_CHAR,
    };

    // first state
    static int SerialState = WAIT_FOR_SYNC_0;
    byte _temp = 0;

    // read data
    while(SERIAL_CMD.available() > 0)
    {
        byte cTemp = SERIAL_CMD.read();
        switch(SerialState)
        {
          case WAIT_FOR_SYNC_0:
            // Looking for first sync byte
            if(cTemp == CMD_SYNC0)
            {
                SerialState++; // First sync byte found, go to next state
            }
            break;

          case WAIT_FOR_SYNC_1:
            // Looking for second sync byte
            if( cTemp == CMD_SYNC1 ) {
                SerialState++; // Second sync byte found
            }
            else
            {
                SerialState = WAIT_FOR_SYNC_0; // Failed sync
                while(SERIAL_CMD.available()>0)_temp = SERIAL_CMD.read();
                SERIAL_CMD.flush();
            }
            break;

          case GET_CRC:
            // Get crc byte
            cmd_crc = cTemp;
            SerialState++;
            break;

          case GET_CMD:
            // Get CMD byte
            cmd = cTemp;
            SerialState++;
            break;

          case GET_PARAM0:
            // Get first Parameter
            cmd_param[0] = cTemp;
            SerialState++;
            break;

          case GET_PARAM1:
            cmd_param[1] = cTemp;
            SerialState++;
            break;

          case GET_PARAM2:
            cmd_param[2] = cTemp;
            SerialState++;
            break;

          case GET_PARAM3:
            cmd_param[3] = cTemp;
            SerialState++;
            break;

          case GET_TERM_CHAR:
            // Check for terminator char
            if(cTemp == CMD_TERM_BYTE)
            {
                // Got the full command!
                SerialState = WAIT_FOR_SYNC_0;  // Ready to get the next command
                return true;                    // Done!
            }
            else
            {
                // Error
                SERIAL_CMD.write(STATUS_ERR_TERM);
                SerialState = WAIT_FOR_SYNC_0;
                while(SERIAL_CMD.available()>0)_temp = SERIAL_CMD.read(); // Clear Buffer
                SERIAL_CMD.flush();
            }
            break;

          default:
            // Error
            SERIAL_CMD.write(STATUS_ERR_STATE);
            SerialState = WAIT_FOR_SYNC_0;      // Abort and get next command
            while(SERIAL_CMD.available()>0)_temp = SERIAL_CMD.read(); // Clear Buffer
            SERIAL_CMD.flush();
        }
    }

    return false;

}

/*******************************************************************************
HANDLE SERIAL CMD DATA
*******************************************************************************/
boolean HandleCmd(void)
{
    boolean state = false;

    /*******************************************
    Is PS2-Controller active -> Exit here!
    *******************************************/
    if(PS2_Active)
    {
        cmd_tx = CMD_REG_PS2_ACTIVE;
        cmd_tx_param[0] = true;

        #ifdef SERIAL_CMD_BEEP
            MSound(1, 50, 1000);
        #endif
        return true;
    }

    /*******************************************
    Process CMD-Commands
    *******************************************/
    switch(cmd)
    {
        /*******************************************
        Power (on/off)
        *******************************************/
      case CMD_REG_POWER:
        if(cmd_param[0]==true)
        {
            // On
            g_InControlState.fRobotOn = 1;
            fAdjustLegPositions       = true;
        }
        else if(cmd_param[0]==false)
        {
            // Off
            InputControllerTurnRobotOff();
        }
        state = true;
        break;

        /*******************************************
        Balance mode (on/off)
        *******************************************/
      case CMD_REG_BALANCE_MODE:
        if(cmd_param[0]==BALANCEMODE_ON)
        {
            // On
            g_InControlState.BalanceMode = true;

        }
        else if(cmd_param[0]==BALANCEMODE_OFF)
        {
            // Off
            g_InControlState.BalanceMode = false;
        }
        state = true;
        break;

        /*******************************************
        Set body height (mm)
        *******************************************/
      case CMD_REG_BODY_HEIGHT:
        if(g_InControlState.fRobotOn)
        {
            // Limit height
            if(g_BodyYOffset > MAX_BODY_Y)
            {
                g_BodyYOffset = MAX_BODY_Y;
            }
            else
            {
                // Height
                g_BodyYOffset       = cmd_param[0];
                fAdjustLegPositions = true;
            }
        }
        state = true;
        break;

        /*******************************************
        Set movement speed
        *******************************************/
      case CMD_REG_SPEED:
        byte speed;
        speed = (cmd_param[0]*2)+5;
        if(speed >= SPEED_MIN)speed = SPEED_MIN;
        if(speed <= SPEED_MAX)speed = SPEED_MAX;
        g_InControlState.SpeedControl = speed;
        state                         = true;
        break;

        /*******************************************
        Walking
        *******************************************/
      case CMD_REG_WALK:
        if(g_InControlState.fRobotOn)
        {
            lx = cmd_param[0]; // left/right
            ly = cmd_param[1]; // forward/backward
            lz = cmd_param[2]; // turn -> yaw

            ControlMode  = WALKMODE;
            g_BodyYShift = 0;

            if(WalkMethod) // Walk Methode
            {
                g_InControlState.TravelLength.z = (lz-128); // Right Stick Up/Down
            }
            else
            {
                g_InControlState.TravelLength.x = -(lx - 128);
                g_InControlState.TravelLength.z = (ly - 128);
            }

            //Double travel length
            if(!DoubleTravelOn)
            {
                g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
                g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
            }

            g_InControlState.TravelLength.y = -(lz - 128)/4; // Right Stick Left/Right

        }
        state = true;
        break;

        /*******************************************
        Translate mode
        *******************************************/
      case CMD_REG_TRANSLATE:
        if(g_InControlState.fRobotOn)
        {
            lx       = cmd_param[0];
            ly       = cmd_param[1];
            lz       = cmd_param[2];
            ly_shift = cmd_param[3];

            ControlMode  = TRANSLATEMODE;
            g_BodyYShift = 0;

            g_InControlState.BodyPos.x  = (lx - 128)/2;
            g_InControlState.BodyPos.z  = -(lz - 128)/3;
            g_InControlState.BodyRot1.y = (ly - 128)*2;
            g_BodyYShift                = (-(ly_shift - 128) /2);

        }
        state = true;
        break;

        /*******************************************
        Rotate mode
        *******************************************/
      case CMD_REG_ROTATE:
        if(g_InControlState.fRobotOn)
        {
            lx       = cmd_param[0];
            ly       = cmd_param[1];
            lz       = cmd_param[2];
            ly_shift = cmd_param[3];

            ControlMode  = ROTATEMODE;
            g_BodyYShift = 0;

            g_InControlState.BodyRot1.x = (lz - 128);
            g_InControlState.BodyRot1.y = (ly - 128)*2;
            g_InControlState.BodyRot1.z = (lx - 128);
            g_BodyYShift                = (-(ly_shift - 128) /2);
        }
        state = true;
        break;

        /*******************************************
        Select gaint
        *******************************************/
      case CMD_REG_GAINT_MODE:
        if(abs(g_InControlState.TravelLength.x)<cTravelDeadZone
            && abs(g_InControlState.TravelLength.z)<cTravelDeadZone
        && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone)
        {
            byte gaint = cmd_param[0];
            if(gaint<=5)
            {
                g_InControlState.GaitType = gaint;
            }
            else
            {
                g_InControlState.GaitType = 0;
            }
            GaitSelect();
        }
        state = true;
        break;

        /*******************************************
        Set double leg lift height
        *******************************************/
      case CMD_REG_DOUBLE_HEIGHT:
        if(cmd_param[0]==0)g_InControlState.LegLiftHeight = WALK_HEIGHT_NORMAL;
        else if(cmd_param[0]==1)g_InControlState.LegLiftHeight = WALK_HEIGHT_DOUBLE;
        else if(cmd_param[0]==2)g_InControlState.LegLiftHeight = cmd_param[1] /2; // Custom height!!!
        state = true;
        break;

        /*******************************************
        Set double travel length
        *******************************************/
      case CMD_REG_DOUBLE_LENGTH:
        if(cmd_param[0]==0)DoubleTravelOn=0;
        else if(cmd_param[0]==1)DoubleTravelOn=1;
        state = true;
        break;

        /*******************************************
        Single leg position
        *******************************************/
      case CMD_REG_SINGLE_LEG_POS:
        if(g_InControlState.fRobotOn)
        {
            sl_nr = cmd_param[0]; // Leg nr.
            lx    = cmd_param[1];
            ly    = cmd_param[2];
            lz    = cmd_param[3];

            ControlMode = SINGLELEGMODE;

            g_InControlState.SelectedLeg = sl_nr;
            g_InControlState.SLLeg.x     = (lx - 128);
            g_InControlState.SLLeg.y     = (lz - 128);
            g_InControlState.SLLeg.z     = (ly - 128);
            g_InControlState.fSLHold     = false;
        }
        state = true;
        break;

        /*******************************************
        Sound function
        *******************************************/
      case CMD_REG_SOUND:
        MSound(1,cmd_param[0],cmd_param[1]*10);
        state = true;
        break;

        /*******************************************
        Set USER-LED
        *******************************************/
      case CMD_REG_STATUS_LED:
        if(cmd_param[0]==true)
        {
            digitalWrite(USER_LED,HIGH);
        }
        else if(cmd_param[0]==false)
        {
            digitalWrite(USER_LED,LOW);
        }
        state = true;
        break;

        /*******************************************
        Read IN1
        *******************************************/
      case CMD_REG_IN1:
        cmd_tx = CMD_REG_IN1;
        cmd_tx_param[0] = digitalRead(IN1);
        state = true;
        break;

        /*******************************************
        Set OUT1
        *******************************************/
      case CMD_REG_OUT1:
        if(cmd_param[0]==true)
        {
            digitalWrite(OUT1,HIGH);
        }
        else if(cmd_param[0]==false)
        {
            digitalWrite(OUT1,LOW);
        }
        state = true;
        break;

        /*******************************************
        Read SA0 to SA5 (analog pins)
        *******************************************/
      case CMD_REG_SA_LEG:
        int raw;
        if(cmd_param[0]==0)
        {
            raw = analogRead(SA0);
        }
        else if(cmd_param[0]==1)
        {
            raw = analogRead(SA1);
        }
        else if(cmd_param[0]==2)
        {
            raw = analogRead(SA2);
        }
        else if(cmd_param[0]==3)
        {
            raw = analogRead(SA3);
        }
        else if(cmd_param[0]==4)
        {
            raw = analogRead(SA4);
        }
        else if(cmd_param[0]==5)
        {
            raw = analogRead(SA5);
        }
        cmd_tx = CMD_REG_SA_LEG;
        cmd_tx_param[0] = highByte(raw);
        cmd_tx_param[1] = lowByte(raw);
        state = true;
        break;

        /*******************************************
        Read akku voltage
        *******************************************/
      case CMD_REG_AKKU:
        int voltage_raw;
        voltage_raw = analogRead(UB_ADC);
        cmd_tx      = CMD_REG_AKKU;
        cmd_tx_param[0] = highByte(voltage_raw);
        cmd_tx_param[1] = lowByte(voltage_raw);
        state = true;
        break;

        /*******************************************
        Check is PS2-Controller active
        *******************************************/
      case CMD_REG_PS2_ACTIVE:
        cmd_tx = CMD_REG_PS2_ACTIVE;
        cmd_tx_param[0] = PS2_Active;
        state = true;
        break;

        /*******************************************
        Check if the robot is walking
        *******************************************/
      case CMD_REG_IS_WALKING:
        cmd_tx = CMD_REG_IS_WALKING;
        cmd_tx_param[0] = fWalking;
        state = true;
        break;

        /*******************************************
        Check if the robot power is on
        *******************************************/
      case CMD_REG_IS_POWER_ON:
        cmd_tx = CMD_REG_IS_POWER_ON;
        cmd_tx_param[0] = g_InControlState.fRobotOn;
        state = true;
        break;

        /*******************************************
        Send PS2-Controller values
        *******************************************/
      case CMD_REG_READ_PS2_VALUES:
        #ifdef PS2_CONTROLLER
            ps2x.read_gamepad();
            // Wish the library had a valid way to verify that the read_gamepad succeeded... Will hack for now
            if ((ps2x.Analog(1) & 0xf0) == 0x70)
            {
                byte h_button = 0;
                byte l_button = 0;
                cmd_tx        = CMD_REG_READ_PS2_VALUES;
                cmd_tx_param[0] = highByte(ps2x.ButtonDataByte());
                cmd_tx_param[1] = lowByte(ps2x.ButtonDataByte());
                cmd_tx_param[2] = ps2x.Analog(PSS_LX);
                cmd_tx_param[3] = ps2x.Analog(PSS_LY);
                cmd_tx_param[4] = ps2x.Analog(PSS_RX);
                cmd_tx_param[5] = ps2x.Analog(PSS_RY);
            }
        #endif
        state = true;
        break;

        /*******************************************
        Full system reset
        *******************************************/
      case CMD_REG_RESET:
        if(cmd_param[0]==100,cmd_param[1]==100,cmd_param[2]==100)
        {
            MSound(3, 250, 2000, 250, 1000, 200, 500);
            // jump to the start of the program
            asm volatile ( "jmp 0");
        }
        state = true;
        break;

      default:
        // error
        state = false;
        break;
    }

    /*******************************************
    Is the robot is on then do it :-)
    *******************************************/
    if(g_InControlState.fRobotOn)
    {
        // Calculate walking time delay
        g_InControlState.InputTimeDelay = 128 - max(max(abs(lx - 128), abs(ly - 128)), abs(lz - 128));

        // Calculate g_InControlState.BodyPos.y
        g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);

        if(fAdjustLegPositions)
        {
            // Put main workings into main program file
            AdjustLegPositionsToBodyHeight();
        }
    }

    /*******************************************
    Debug CMD-RX beep
    *******************************************/
    #ifdef SERIAL_CMD_BEEP
        if((cmd != CMD_REG_SOUND) && (cmd != CMD_REG_POWER))
        {
            if(state)MSound(1, 20, 2500);
            else MSound(1, 50, 1000);
        }
    #endif

    return state;
}

/*******************************************************************************
CALCULATE RX-CRC
*******************************************************************************/
boolean CheckRxCrc(void)
{
    byte crc = CMD_SYNC0 ^ CMD_SYNC1 ^ cmd ^ cmd_param[0] ^ cmd_param[1] ^ cmd_param[2] ^ cmd_param[3];
    if(crc == cmd_crc)return true;
    return false;
}

/*******************************************************************************
SEND Data to User-Board
*******************************************************************************/
void SendData(byte _status, byte _cmd, byte _data0, byte _data1, byte _data2, byte _data3, byte _data4, byte _data5)
{
    // Calc CRC
    byte _crc = CMD_SYNC0 ^ CMD_SYNC1 ^ _status ^_cmd ^ _data0 ^ _data1 ^ _data2 ^ _data3 ^ _data4 ^ _data5;

    SERIAL_CMD.write(CMD_SYNC0);
    SERIAL_CMD.write(CMD_SYNC1);
    SERIAL_CMD.write(_crc);

    SERIAL_CMD.write(_status);
    SERIAL_CMD.write(_cmd);
    SERIAL_CMD.write(_data0);
    SERIAL_CMD.write(_data1);
    SERIAL_CMD.write(_data2);
    SERIAL_CMD.write(_data3);
    SERIAL_CMD.write(_data4);
    SERIAL_CMD.write(_data5);

    SERIAL_CMD.write(CMD_TERM_BYTE);
}

/*******************************************************************************
ROBOT SHUT DOWN
Code used couple of places so save a little room...
*******************************************************************************/
void InputControllerTurnRobotOff(void)
{
    // Turn the robot off
    g_InControlState.BodyPos.x      = 0;
    g_InControlState.BodyPos.y      = 0;
    g_InControlState.BodyPos.z      = 0;
    g_InControlState.BodyRot1.x     = 0;
    g_InControlState.BodyRot1.y     = 0;
    g_InControlState.BodyRot1.z     = 0;
    g_InControlState.TravelLength.x = 0;
    g_InControlState.TravelLength.z = 0;
    g_InControlState.TravelLength.y = 0;
    g_BodyYOffset                   = 0;
    g_BodyYShift                    = 0;
    #ifdef OPT_SINGLELEG
        g_InControlState.SelectedLeg = 255;
    #endif
    g_InControlState.fRobotOn = 0;
    AdjustLegPositionsToBodyHeight();
}

/*******************************************************************************
MAIN
*******************************************************************************/
void InputController::ControlInput(void)
{

    /*************************************************************************
    Read serial data...
    **************************************************************************/
    if(CheckForSerialData())
    {
        // Send Commands to User-Board
        cmd_tx = 0;
        cmd_tx_param[0] = 0;
        cmd_tx_param[1] = 0;
        cmd_tx_param[2] = 0;
        cmd_tx_param[3] = 0;
        cmd_tx_param[4] = 0;
        cmd_tx_param[5] = 0;

        // CRC ok?
        if(CheckRxCrc())
        {
            // Process the data
            if(HandleCmd())
            {
                cmd_status_byte = STATUS_ACK_OK;
            }
            else
            {
                cmd_status_byte = STATUS_ERR_CMD;
            }
        }
        else
        {
            cmd_status_byte = STATUS_ERR_CRC;
        }

        // Send data to User-Board (status byte + commands...)
        SendData(cmd_status_byte, cmd_tx, cmd_tx_param[0], cmd_tx_param[1], cmd_tx_param[2], cmd_tx_param[3], cmd_tx_param[4], cmd_tx_param[5]);

    }

    /*************************************************************************
    Read and process the PS2-Controller
    *************************************************************************/
    #ifdef PS2_CONTROLLER

        // Jumper J7 is set?
        // Locomotion-Controller PS2-Controller mode -> Jumper Pin is open
        if(digitalRead(JUMPER_J7))
        {
            // Then try to receive a packet of information from the PS2.
            ps2x.read_gamepad();

            // Wish the library had a valid way to verify that the read_gamepad succeeded... Will hack for now
            if ((ps2x.Analog(1) & 0xf0) == 0x70)
            {
                #ifdef DBGSerial
                    #ifdef DEBUG_PS2_INPUT
                        if (g_fDebugOutput) {
                            DBGSerial.print("PS2 Input: ");
                            DBGSerial.print(ps2x.ButtonDataByte(), HEX);
                            DBGSerial.print(":");
                            DBGSerial.print(ps2x.Analog(PSS_LX), DEC);
                            DBGSerial.print(" ");
                            DBGSerial.print(ps2x.Analog(PSS_LY), DEC);
                            DBGSerial.print(" ");
                            DBGSerial.print(ps2x.Analog(PSS_RX), DEC);
                            DBGSerial.print(" ");
                            DBGSerial.println(ps2x.Analog(PSS_RY), DEC);
                        }
                    #endif
                #endif

                /*******************************************
                [START BUTTON]
                Start button activate the PS2-Controller
                *******************************************/
                if(ps2x.ButtonPressed(PSB_START))
                {
                    if(!PS2_Active)
                    {
                        // PS2 is activate
                        digitalWrite(USER_LED,HIGH);
                        MSound(2, 100, 2500, 50, 4000);
                        PS2_Active = true;
                    }
                    else
                    {
                        // PS2 is deactivate
                        digitalWrite(USER_LED,LOW);
                        MSound(1, 100, 2500);
                        PS2_Active = false;
                    }
                }

                /*******************************************
                Now use the PS2 comannds
                Suppress serial commands from the User-Board
                *******************************************/
                if(PS2_Active)
                {
                    fAdjustLegPositions = false;

                    /*******************************************
                    [CROSS BUTTON]
                    Turn the robot on or off
                    *******************************************/
                    if(ps2x.ButtonPressed(PSB_CROSS))
                    {
                        if(g_InControlState.fRobotOn)
                        {
                            // Off
                            InputControllerTurnRobotOff();
                        }
                        else
                        {
                            // On
                            g_InControlState.fRobotOn = 1;
                            fAdjustLegPositions       = true;
                        }
                    }

                    /*******************************************
                    [CIRCLE BUTTON]
                    Reset the robot (Full system reset)
                    *******************************************/
                    if(ps2x.ButtonPressed(PSB_CIRCLE))
                    {
                        MSound(3, 250, 2000, 250, 1000, 200, 500);
                        asm volatile ("  jmp 0");
                    }

                    /*******************************************
                    Is the robot on we can move it :-)
                    *******************************************/
                    if(g_InControlState.fRobotOn)
                    {
                        /*******************************************
                        [L1 BUTTON]
                        Translate mode (on/off)
                        *******************************************/
                        if(ps2x.ButtonPressed(PSB_L1))
                        {
                            if(ControlMode == TRANSLATEMODE)
                            {
                                ControlMode = WALKMODE;
                                MSound( 2, 100, 2000, 50, 4000);
                            }
                            else
                            {
                                ControlMode = TRANSLATEMODE;
                                MSound(1, 50, 2000);
                            }
                        }

                        /*******************************************
                        [L2 BUTTON]
                        Rotate mode (on/off)
                        *******************************************/
                        if(ps2x.ButtonPressed(PSB_L2))
                        {
                            if(ControlMode == ROTATEMODE)
                            {
                                ControlMode = WALKMODE;
                                MSound( 2, 100, 2000, 50, 4000);
                            }
                            else
                            {
                                ControlMode = ROTATEMODE;
                                MSound(1, 50, 2000);
                            }
                        }

                        /*******************************************
                        [SQUARE BUTTON]
                        Balance mode on/off
                        *******************************************/
                        if(ps2x.ButtonPressed(PSB_SQUARE))
                        {
                            g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
                            if(g_InControlState.BalanceMode)
                            {
                                MSound(1, 50, 2000);
                            }
                            else
                            {
                                MSound( 2, 100, 2000, 50, 4000);
                            }
                        }

                        /*******************************************
                        [TRIANGLE BUTTON]
                        Stand up or sit down
                        *******************************************/
                        if(ps2x.ButtonPressed(PSB_TRIANGLE))
                        {
                            if(g_BodyYOffset>0)
                            {
                                MSound( 2, 100, 2000, 50, 4000);
                                g_BodyYOffset = 0;
                            }
                            else
                            {
                                MSound(1, 50, 2000);
                                g_BodyYOffset       = 35;
                                fAdjustLegPositions = true;
                            }
                        }

                        /*******************************************
                        [D-UP BUTTON]
                        Body offset up
                        *******************************************/
                        if(ps2x.ButtonPressed(PSB_PAD_UP))
                        {
                            g_BodyYOffset += 10;

                            // And see if the legs should adjust...
                            fAdjustLegPositions = true;
                            if (g_BodyYOffset > MAX_BODY_Y)
                            {
                                g_BodyYOffset = MAX_BODY_Y;
                                MSound(1, 150, 1500); // Max. reached
                            }
                            else
                            {
                                MSound( 1, 50, 2000);
                            }
                        }

                        /*******************************************
                        [D-DOWN BUTTON]
                        Body offset down
                        *******************************************/
                        if(ps2x.ButtonPressed(PSB_PAD_DOWN) && g_BodyYOffset)
                        {
                            if (g_BodyYOffset > 10)
                            {
                                g_BodyYOffset -= 10;
                                MSound( 1, 50, 2000);
                            }
                            else
                            {
                                g_BodyYOffset = 0;      // Constrain don't go less than zero.
                                MSound( 1, 150, 1500);  // Min. reached
                            }

                            // And see if the legs should adjust...
                            fAdjustLegPositions = true;
                        }

                        /*******************************************
                        [D-RIGHT BUTTON]
                        Speed plus
                        *******************************************/
                        if(ps2x.ButtonPressed(PSB_PAD_RIGHT))
                        {
                            if(g_InControlState.SpeedControl >= SPEED_MAX)
                            {
                                g_InControlState.SpeedControl -= 25;
                                if(g_InControlState.SpeedControl == SPEED_INIT) // Center reached
                                    MSound( 1, 100, 3000);
                                else MSound( 1, 50, 2000);
                            }
                            else
                            {
                                MSound( 1, 150, 1500); // Max. reached
                            }
                        }

                        /*******************************************
                        [D-LEFT BUTTON]
                        Speed minus
                        *******************************************/
                        if(ps2x.ButtonPressed(PSB_PAD_LEFT))
                        {
                            if(g_InControlState.SpeedControl <= SPEED_MIN)
                            {
                                g_InControlState.SpeedControl += 25;
                                if(g_InControlState.SpeedControl == SPEED_INIT) // Center reached
                                    MSound( 1, 100, 3000);
                                else MSound( 1, 50, 2000);
                            }
                            else
                            {
                                MSound( 1, 150, 1500); // Min. reached
                            }
                        }

                        /*******************************************
                        [LEFT ANALOG]
                        Read the left analog joystick
                        *******************************************/
                        lx = ps2x.Analog(PSS_LX);
                        ly = ps2x.Analog(PSS_LY);

                        /*******************************************
                        Walkmode is on...
                        *******************************************/
                        if(ControlMode == WALKMODE)
                        {
                            /*******************************************
                            [SELECT BUTTON]
                            Switch gaint
                            *******************************************/
                            if(ps2x.ButtonPressed(PSB_SELECT)
                                && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
                            && abs(g_InControlState.TravelLength.z)<cTravelDeadZone
                            && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone)
                            {
                                // Go to the next gait...
                                g_InControlState.GaitType = g_InControlState.GaitType + 1;
                                // Make sure we did not exceed number of gaits...
                                if(g_InControlState.GaitType<NUM_GAITS)
                                {
                                    MSound( 1, 50, 2000);
                                }
                                else
                                {
                                    MSound(2, 100, 2000, 50, 4000);
                                    g_InControlState.GaitType = 0;
                                }
                                GaitSelect();
                            }

                            /*******************************************
                            [R1 BUTTON]
                            Double leg height
                            *******************************************/
                            if(ps2x.ButtonPressed(PSB_R1))
                            {
                                DoubleHeightOn = !DoubleHeightOn;
                                if(DoubleHeightOn)
                                {
                                    //g_InControlState.LegLiftHeight = 80;
                                    g_InControlState.LegLiftHeight = WALK_HEIGHT_DOUBLE;
                                    MSound(1, 50, 2000);
                                }
                                else
                                {
                                    //g_InControlState.LegLiftHeight = 50;
                                    g_InControlState.LegLiftHeight = WALK_HEIGHT_NORMAL;
                                    MSound( 2, 100, 2000, 50, 4000);
                                }
                            }

                            /*******************************************
                            [R2 BUTTON]
                            Double leg lenght
                            *******************************************/
                            if(ps2x.ButtonPressed(PSB_R2))
                            {
                                DoubleTravelOn = !DoubleTravelOn;
                                if(DoubleTravelOn)
                                {
                                    MSound(1, 50, 2000);
                                }
                                else
                                {
                                    MSound( 2, 100, 2000, 50, 4000);
                                }
                            }

                            /*******************************************
                            Walking calculations
                            *******************************************/
                            if(WalkMethod)  // Walk Methode
                            {
                                g_InControlState.TravelLength.z = (ps2x.Analog(PSS_RY)-128);
                            }
                            else
                            {
                                g_InControlState.TravelLength.x = -(lx - 128);
                                g_InControlState.TravelLength.z = (ly - 128);
                            }

                            // Double travel length is active
                            if(!DoubleTravelOn)
                            {
                                g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
                                g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
                            }

                            g_InControlState.TravelLength.y = -(ps2x.Analog(PSS_RX) - 128)/4; // Right Stick Left/Right
                        }

                        // Translate functions is active
                        g_BodyYShift = 0;
                        if(ControlMode == TRANSLATEMODE)
                        {
                            g_InControlState.BodyPos.x  = (lx - 128)/2;
                            g_InControlState.BodyPos.z  = -(ly - 128)/3;
                            g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128)*2;
                            g_BodyYShift                = (-(ps2x.Analog(PSS_RY) - 128)/2);
                        }

                        // Rotate functions is active
                        if(ControlMode == ROTATEMODE)
                        {
                            g_InControlState.BodyRot1.x = (ly - 128);
                            g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128)*2;
                            g_InControlState.BodyRot1.z = (lx - 128);
                            g_BodyYShift                = (-(ps2x.Analog(PSS_RY) - 128)/2);
                        }

                        // Calculate walking time delay
                        g_InControlState.InputTimeDelay = 128 - max(max(abs(lx - 128), abs(ly - 128)), abs(ps2x.Analog(PSS_RX) - 128));

                    }

                    // Calculate g_InControlState.BodyPos.y
                    g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);

                    if (fAdjustLegPositions)
                    {
                        AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
                    }

                }

            }
        }
        else
        {
            // PS2-Controller deactivated
            digitalWrite(USER_LED,LOW);
            PS2_Active = false;
        }
    #endif
}
#endif
