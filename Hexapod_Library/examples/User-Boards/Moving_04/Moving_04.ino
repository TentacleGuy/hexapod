/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Moving_04.ino
Libraries   : SoftwareSerial.h, Hexapod_Lib.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- The sample show the basic movements with the hexapod library
- Main focus is the ROBOT_ROTATE_MODE() and ROBOT_TRANSLATE_MODE() function

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
        
        // start demo
        if(!digitalRead(T1))
        {
            delay(50);
            if(!digitalRead(T1))
            {
                MSound(1, 100, 1000);
                
                ROBOT_INIT();
                ROBOT_PWR_ON();
                ROBOT_HEIGHT(50);
                ROBOT_SPEED(100);
                delay(2000);
                
                //------------------------------
                // ROBOT_ROTATE_MODE
                // x, y, z, BodyYShift
                //------------------------------
                ROBOT_ROTATE_MODE(128,128,128,128);         // center -> body normal
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_ROTATE_MODE(0,128,128,128);           // 0 to 128 body tilt to the right
                delay(2000);                                // do this for 2 sec.
                ROBOT_ROTATE_MODE(255,128,128,128);         // 128 to 255 body tilt to the left
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_ROTATE_MODE(128,0,128,128);           // 0 to 128 body turn to left 
                delay(2000);                                // do this for 2 sec.
                ROBOT_ROTATE_MODE(128,255,128,128);         // 128 to 255 body turn to right 
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_ROTATE_MODE(128,128,0,128);           // 0 to 128 body tilt backward (up)
                delay(2000);                                // do this for 2 sec.
                ROBOT_ROTATE_MODE(128,128,255,128);         // 128 to 255 body tilt forward (down)
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_ROTATE_MODE(128,128,128,0);           // body offset max (up)
                delay(2000);                                // do this for 2 sec.
                ROBOT_ROTATE_MODE(128,128,128,255);         // body offset min (down)
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_STOP();                               // stop
                MSound(3, 100, 1000, 50, 2000, 100, 3500);  // beeping
                delay(2000);
                
                
                //------------------------------
                // ROBOT_TRANSLATE_MODE
                // x, y, z, BodyYShift
                //------------------------------
                ROBOT_TRANSLATE_MODE(128,128,128,128);      // center -> body normal
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_TRANSLATE_MODE(0,128,128,128);        // 0 to 128 body tilt to the left
                delay(2000);                                // do this for 2 sec.
                ROBOT_TRANSLATE_MODE(255,128,128,128);      // 128 to 255 body tilt to the right
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_TRANSLATE_MODE(128,0,128,128);        // 0 to 128 body turn to left 
                delay(2000);                                // do this for 2 sec.
                ROBOT_TRANSLATE_MODE(128,255,128,128);      // 128 to 255 body turn to right 
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_TRANSLATE_MODE(128,128,0,128);        // 0 to 128 body tilt backward (up)
                delay(2000);                                // do this for 2 sec.
                ROBOT_TRANSLATE_MODE(128,128,255,128);      // 128 to 255 body tilt forward (down)
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_TRANSLATE_MODE(128,128,128,0);        // body offset max (up)
                delay(2000);                                // do this for 2 sec.
                ROBOT_TRANSLATE_MODE(128,128,128,255);      // body offset min (down)
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_STOP();                               // stop
                delay(2000);                                // do this for 2 sec.
                
                ROBOT_PWR_OFF();                            // power off
                
                delay(1500);
                
                for(int i = 0; i < 3; i++)                  // end beeping
                {
                    MSound(3, 100, 1000, 50, 2000, 100, 3500);
                    delay(100);
                }
                
            }
        }    
    }
}

