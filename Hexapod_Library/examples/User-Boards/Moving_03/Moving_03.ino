/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Moving_03.ino
Libraries   : SoftwareSerial.h, Hexapod_Lib.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- The sample show the basic movements with the hexapod library
- Main focus is the ROBOT_MOVE() function

******************************************************************************/

// Arduino or NodeMCU (select one)
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
            delay(10);      // only for NodeMCU to supress the watchdog reset
        #endif
        
        int _hight = 80;    // init robot hight
        
        //-----------------------------------------
        // press T1 for start
        //-----------------------------------------
        if(!digitalRead(T1)) 
        {
            delay(50);
            if(!digitalRead(T1))
            {
                MSound(1, 100, 1000);
                
                ROBOT_INIT();                   // reset etc.
                ROBOT_PWR_ON();                 // power on
                ROBOT_HEIGHT(_hight);           // init hight
                ROBOT_SPEED(100);               // init speed (value 10 is fast and value 200 is very slow)
                delay(2000);
                
                //------------------------------
                // lateral, turn, move
                //------------------------------
                ROBOT_MOVE(128,128,128);        // stop
                delay(2000);
                
                ROBOT_MOVE(50,128,128);         // go left side with value 50
                delay(8000);                    // do this for 8 sec.
                
                ROBOT_MOVE(128,128,128);        // stop
                delay(1000);                    // do this for 1 sec.
                
                ROBOT_MOVE(255,128,128);        // go right side wiht value 255 -> is fast
                delay(5000);                    // do this for 5 sec.
                
                ROBOT_MOVE(128,128,128);        // stop
                delay(1000);                    // do this for 1 sec.
                
                ROBOT_MOVE(128,200,128);        // turn right with value 200
                delay(5000);                    // do this for 5 sec.
                
                ROBOT_MOVE(128,0,128);          // turn left with value 0 -> is fast
                delay(5000);                    // do this for 5 sec.
                
                ROBOT_MOVE(128,128,0);          // move forwared wiht value 0 -> is fast
                delay(5000);                    // do this for 5 sec.
                
                ROBOT_MOVE(128,128,128);        // stop
                delay(1000);                    // do this for 1 sec.
                
                ROBOT_MOVE(128,128,200);        // move backward with value 200
                delay(5000);                    // do this for 5 sec.
                
                ROBOT_SPEED(20);                // set new speed
                ROBOT_MOVE(128,255,128);        // turn right with value 255 (fast)
                delay(3000);                    // do this for 3 sec.
                
                ROBOT_STOP();                   // stop
                delay(2500);                    // do this for 2,5 sec.
                                
                ROBOT_HEIGHT(0);                // sit down
                delay(2000);                
                
                ROBOT_PWR_OFF();                // power off
                delay(1500);                
                
                for(int i = 0; i < 3; i++)      // end beeping
                {
                    MSound(3, 100, 1000, 50, 2000, 100, 3500);
                    delay(100);
                }
                
            }
        }
    }
}

