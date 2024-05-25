/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : ReadGampead.ino
Libraries   : SoftwareSerial.h, Hexapod_Lib.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- Read the PS2-Controller values from Locomotion-Controller
- Jumper J7 deactivate the PS2-Controller on the Locomotion-Controller

******************************************************************************/

#define ARDUINO
//#define NODEMCU

#include <Hexapod_Lib.h>

/******************************************************************************
INIT
******************************************************************************/
void setup() 
{
    // high-Z for the audio output
    pinMode(PA_PIN,INPUT);
    digitalWrite(PA_PIN,LOW);
    
    // switches T1 and T2
    #ifdef ARDUINO
        pinMode(T1,INPUT);
        pinMode(T2,INPUT);
        //digitalWrite(T1,HIGH); // use internal pull-up resistor 
        //digitalWrite(T2,HIGH); // use internal pull-up resistor 
    #endif   
    
    #ifdef NODEMCU
        pinMode(T1,INPUT);
        //digitalWrite(T1,HIGH); // use internal pull-up resistor 
    #endif
    
    // open serial communications and wait for port to open:
    Serial.begin(SERIAL_STD_BAUD);
    while(!Serial) 
    {
        ;  // wait for serial port to connect. Needed for native USB port only 
    }
    
    // set the data rate for the SoftwareSerial port (User-Board to Locomotion-Controller)
    SERIAL_CMD.begin(SERIAL_CMD_BAUD);
    
    // reset the Locomotion-Controller
    ROBOT_RESET();
    delay(250);
    ROBOT_RESET();
    delay(150);
    ROBOT_RESET();
    
    // wait for Boot-Up
    delay(1500);
    ROBOT_INIT();
    
    // print a hello world over the USB connection
    Serial.println("> Hello here is the C-Control Hexapod!");
}

/******************************************************************************
MAIN
******************************************************************************/
void loop() 
{
    // main loop
    while(1) 
    {
        #ifdef NODEMCU
            delay(10); // only for NodeMCU to supress the watchdog reset
        #endif
        
        //-----------------------------------------
        // press T1 for start
        // read data from Locomotion-Controller
        //-----------------------------------------
        if(!digitalRead(T1)) // debounce T1
        {
            delay(50);
            if(!digitalRead(T1)) 
            {
                MSound(1, 100, 1000);
                
                // read PS2-Controller values
                // the robot must not be started with the PS2-Controller, otherwise the values are zero!
                // jumper J7 deactivate the PS2-Controller on the Locomotion-Controller!
                while(1) 
                {
                    // read the values from Locomotion-Controller
                    ReadPs2Values();
                    Serial.print("> Button: ");
                    Serial.println(ps2x_ButtonData);
                    Serial.print("> LX: ");
                    Serial.println(ps2x_Analog_PSS_LX);
                    Serial.print("> LY: ");
                    Serial.println(ps2x_Analog_PSS_LY);
                    Serial.print("> RX: ");
                    Serial.println(ps2x_Analog_PSS_RX);
                    Serial.print("> RY: ");
                    Serial.println(ps2x_Analog_PSS_RY);
                    
                    // exit loop with "select button"
                    if(ps2x_ButtonData == 1){goto _exit;}
                    
                    delay(50);
                }                
            }
        }        
    }
    
_exit:
    // exit point
    Serial.println("> PROGRAM EXIT");
    MSound(3, 100, 1000, 50, 2000, 100, 1000);
    
}
