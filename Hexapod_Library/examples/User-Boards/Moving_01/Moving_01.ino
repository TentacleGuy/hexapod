/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Moving_01.ino
Libraries   : SoftwareSerial.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- The sample show the basic movements w/o the hexapod library
- Send data from the User-Board to Locomotion-Controller

******************************************************************************/
#include <SoftwareSerial.h>

// Arduino or NodeMCU (select one)
//#define ARDUINO
#define NODEMCU


// UART
#define SERIAL_CMD_BAUD         38400       // Locomotion-Controller to User-Board

#define CMD_SYNC0               33          // Sync byte 1
#define CMD_SYNC1               42          // Sync byte 2
#define CMD_TERM_BYTE           255         // Termination byte

// CMD REGISTER [WRITE-VALUES]
#define CMD_REG_POWER           5
#define CMD_REG_SPEED           10
#define CMD_REG_GAINT_MODE      15
#define CMD_REG_BALANCE_MODE    20
#define CMD_REG_BODY_HEIGHT     25
#define CMD_REG_TRANSLATE       35
#define CMD_REG_WALK            40
#define CMD_REG_ROTATE          45
#define CMD_REG_DOUBLE_HEIGHT   50
#define CMD_REG_DOUBLE_LENGTH   55
#define CMD_REG_SINGLE_LEG_POS  60
#define CMD_REG_SOUND           65
#define CMD_REG_OUT1            70
#define CMD_REG_STATUS_LED      75

// CMD-REGISTER [HW-RESET]
#define CMD_REG_RESET           255

// MOVE MODES
#define WALKMODE                0
#define TRANSLATEMODE           1
#define ROTATEMODE              2
#define SINGLELEGMODE           3

#define BALANCEMODE_ON          1
#define BALANCEMODE_OFF         0

// GAIN MODES
enum GAIN_MODES
{
    TRIPOD_6,
    TRIPOD_8,
    TRIPPLE_12,
    TRIPPLE_16,
    RIPPLE_12,
    WAVE_24,
};

// USER-BOARD PINS (ARDUINO UNO or compatible)
#ifdef ARDUINO
    #define T1                      A2
    #define RXD_U                   4
    #define TXD_U                   2
#endif

// USER-BOARD PINS (NodeMCU)
#ifdef NODEMCU
    #define T1                      10
    #define RXD_U                   0
    #define TXD_U                   2
#endif

// Software Serial (Locomotion-Controler <--> User-Board)
SoftwareSerial SERIAL_CMD(RXD_U, TXD_U); // RX, TX Arduino UNO

/******************************************************************************
UART SEND- and READ-COMMANDS
******************************************************************************/
void SendData(byte _cmd, byte _data1, byte _data2, byte _data3, byte _data4)
{
    // calc CRC
    byte _crc = CMD_SYNC0 ^ CMD_SYNC1 ^ _cmd ^ _data1 ^ _data2 ^ _data3 ^ _data4;
    
    // send bytes
    SERIAL_CMD.write(CMD_SYNC0);
    SERIAL_CMD.write(CMD_SYNC1);
    SERIAL_CMD.write(_crc);
    
    SERIAL_CMD.write(_cmd);
    SERIAL_CMD.write(_data1);
    SERIAL_CMD.write(_data2);
    SERIAL_CMD.write(_data3);
    SERIAL_CMD.write(_data4);
    SERIAL_CMD.write(CMD_TERM_BYTE);
}

/******************************************************************************
INIT
******************************************************************************/
void setup() 
{
    // switche T1
    pinMode(T1,INPUT);
    digitalWrite(T1,HIGH);
    
    // set the data rate for the SoftwareSerial port (User-Board to Locomotion-Controller)
    SERIAL_CMD.begin(SERIAL_CMD_BAUD);
    
    // reset the Locomotion-Controller
    SendData(CMD_REG_RESET, 100, 100, 100, 0);
    delay(250);
    SendData(CMD_REG_RESET, 100, 100, 100, 0);
    
    // wait for Locomotion-Controller boot-up
    delay(1500);
}

/******************************************************************************
MAIN
******************************************************************************/
void loop() 
{
    // press T1 for start!
    if(!digitalRead(T1))
    {
        delay(50);
        if(!digitalRead(T1))
        {
            // PLAY a TONE
            SendData(CMD_REG_SOUND, 50, 150, 0, 0);
            
            //-----------------------------------------
            // INIT
            //-----------------------------------------
            
            // STOP
            SendData(CMD_REG_WALK, 128, 128, 128, 0);
            
            // SPEED = 100
            SendData(CMD_REG_SPEED, 100, 0, 0, 0);
            
            // BODY HEIGHT = 0
            SendData(CMD_REG_BODY_HEIGHT, 0, 0, 0, 0);
            
            // GAINT MODE = TRIPOD_8
            SendData(CMD_REG_GAINT_MODE, TRIPOD_8, 0, 0, 0);
            
            // POWER OFF
            SendData(CMD_REG_POWER, 0, 0, 0, 0);
            delay(3000);
            
            //-----------------------------------------
            // DEMO
            //-----------------------------------------
            
            // POWER ON
            SendData(CMD_REG_POWER, 1, 0, 0, 0);
            delay(2000);
            
            // BODY HEIGHT = 60
            SendData(CMD_REG_BODY_HEIGHT, 60, 0, 0, 0);
            // give the robot time to set the servo
            delay(3000);
            
            // WALK FWD = 0 -> FULL
            SendData(CMD_REG_WALK, 128, 0, 128, 0);
            // do this for 5 sec.
            delay(5000);
            
            // STOP
            SendData(CMD_REG_WALK, 128, 128, 128, 0);
            // do this for 2 sec.
            delay(2000);
            
            // WALK LEFT
            SendData(CMD_REG_WALK, 50, 128, 128, 0);
            // do this for 5 sec.
            delay(5000);
            
            // STOP
            SendData(CMD_REG_WALK, 128, 128, 128, 0);
            // do this for 2 sec.
            delay(2000);
            
            // WALK RIGHT
            SendData(CMD_REG_WALK, 200, 128, 128, 0);
            // do this for 10 sec.
            delay(10000);
            
            // STOP
            SendData(CMD_REG_WALK, 128, 128, 128, 0);
            delay(2000);
            
            // TURN LEFT
            SendData(CMD_REG_WALK, 128, 128, 50, 0);
            // do this for 5 sec.
            delay(5000);
            
            // STOP
            SendData(CMD_REG_WALK, 128, 128, 128, 0);
            // do this for 2 sec.
            delay(2000);
            
            // TURN RIGHT
            SendData(CMD_REG_WALK, 128, 128, 200, 0);
            // do this for 10 sec.
            delay(10000);
            
            // STOP
            SendData(CMD_REG_WALK, 128, 128, 128, 0);
            // do this for 3 sec.
            delay(3000);
            
            // BODY HEIGHT
            SendData(CMD_REG_BODY_HEIGHT, 0, 0, 0, 0);
            // give the robot time to set the servo
            delay(3000);
            
            // POWER OFF
            SendData(CMD_REG_POWER, 0, 0, 0, 0);
            
        } 
    }
}


