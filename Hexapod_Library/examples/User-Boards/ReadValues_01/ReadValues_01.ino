/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : ReadValues.ino
Libraries   : SoftwareSerial.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- The sample show the basic readings from the Locomotion-Controller
- Only send data from the Locomotion-Controller to the User-Board

******************************************************************************/
#include <SoftwareSerial.h>

// UART
#define SERIAL_CMD_BAUD         38400       // Locomotion-Controller to User-Board
#define SERIAL_STD_BAUD         115200      // USB Connector (Arduino-UNO/NodeMCU)

#define CMD_SYNC0               33          // Sync byte 1
#define CMD_SYNC1               42          // Sync byte 2
#define CMD_TERM_BYTE           255         // Termination byte

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

// USER-BOARD PINS (ARDUINO UNO)
#define T1                      A2
#define RXD_U                   4
#define TXD_U                   2

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
    // switches T1
    pinMode(T1,INPUT);
    digitalWrite(T1,HIGH);
    
    // open serial communications and wait for port to open:
    Serial.begin(SERIAL_STD_BAUD);
    while(!Serial);  // wait for serial port to connect. Needed for native USB port only
    
    // set the data rate for the SoftwareSerial port (User-Board to Locomotion-Controller)
    SERIAL_CMD.begin(SERIAL_CMD_BAUD);
    
    // reset the Locomotion-Controller
    SendData(CMD_REG_RESET, 100, 100, 100, 0);
    delay(250);
    SendData(CMD_REG_RESET, 100, 100, 100, 0);
    
    // wait for Locomotion-Controller Boot-Up
    delay(1500);
    
    // print a hello world over the USB connection
    Serial.println("> Hello here is the C-Control Hexapod!"); 
}

/******************************************************************************
MAIN
******************************************************************************/
void loop() 
{

    // press button T1 for start!
    if(!digitalRead(T1)) 
    {
        delay(50);
        if(!digitalRead(T1)) 
        {
            //----------------------------
            // incomming bytes
            //----------------------------
            // [SYNC0],[SYNC1],[CRC],[STATUS_BYTE],[CMD],[PARAM0],[PARAM1],[PARAM2],[PARAM3],[PARAM4],[PARAM5],[TERMINATION_BYTE]
            
            // SYNC0            = 33
            // SYNC1            = 42
            // CRC              = checksum
            // STATUS_BYTE      = 64
            // CMD              = 105
            // PARAM1           = (ADC) hight byte
            // PARAM2           = (ADC) low byte
            // PARAM3           = 0
            // PARAM4           = 0
            // PARAM5           = 0
            // TERMINATION_BYTE = 255
            
            //----------------------------
            // STATUS BYTE INFO
            //----------------------------
            // STATUS ACK OK    = 64
            // STATUS ERR TERM  = 1
            // STATUS ERR STATE = 2
            // STATUS ERR CRC   = 3
            // STATUS ERR CMD   = 255
            
            // send request 
            SendData(CMD_REG_AKKU, 0, 0, 0, 0);
            
            // read the values
            while(SERIAL_CMD.available() > 0)
            {
                // show the incoming bytes in the terminal
                byte cTemp = SERIAL_CMD.read();
                Serial.println(cTemp);
            }
            
            // wait for T1 release
            while(!digitalRead(T1))
            {
                delay(50)
            }
            
        }
    }
}



