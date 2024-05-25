/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Moving_05.ino
Libraries   : SoftwareSerial.h, Hexapod_Lib.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- The sample show the basic movements with the hexapod library
- Main focus is the ROBOT_SINGLE_LEG() function

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
        
        // start Demo
        if(!digitalRead(T1))
        {
            delay(50);
            if(!digitalRead(T1))
            {
                MSound(1, 100, 1000);
                
                ROBOT_INIT();                               // init
                ROBOT_PWR_ON();                             // power on
                ROBOT_HEIGHT(50);                           // set hight to 50
                ROBOT_SPEED(100);                           // set speed to 100 (middle)
                delay(2000);                                // wait 2 sec.
                
                //------------------------------
                // ROBOT_SINGLE_LEG
                // idx, x, y, z
                // 0 = RR
                // 1 = RM
                // 2 = RF
                // 3 = LR
                // 4 = LM
                // 5 = LF
                //------------------------------
                ROBOT_TRANSLATE_MODE(128,128,255,128);      // 0 to 128 body tilt backward (up)
                ROBOT_HEIGHT(60);                           // set the body hight
                ROBOT_SPEED(5);                             // set speed (5 is fast)
                
                for(int i = 0; i < 5; i++)                  // swing the front left leg
                {                    
                    ROBOT_SINGLE_LEG(5, 160, 60, 60);
                    delay(250);
                    ROBOT_SINGLE_LEG(5, 160, 0, 0);
                    delay(250);     
                }
                
                for(int i = 0; i < 5; i++)                  // swing the front right leg
                {
                    ROBOT_SINGLE_LEG(2, 160, 60, 60);
                    delay(250);
                    ROBOT_SINGLE_LEG(2, 160, 0, 0);
                    delay(250);
                }
                
                ROBOT_SINGLE_LEG(5, 128, 128, 128);
                delay(1000);
                ROBOT_TRANSLATE_MODE(128,128,128,128);      // 0 to 128 body tilt backward (up)
                
                ROBOT_STOP();                               // stop
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_PWR_OFF();                            // power off
                
                delay(1500);                                // wait 1,5 sec.
                
                for(int i = 0; i < 3; i++)                  // end beeping
                {
                    MSound(3, 100, 1000, 50, 2000, 100, 3500);
                    delay(100);
                }
                
            }
        }
    }
}
