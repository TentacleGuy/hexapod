/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Moving_02.ino
Libraries   : SoftwareSerial.h, Hexapod_Lib.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- The sample show the basic movements with the hexapod library

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
            delay(10); // only for NodeMCU with Lib 2.3 to supress the watchdog reset
        #endif
        
        int _hight = 60;
        
        // start demo
        if(!digitalRead(T1))
        {
            delay(50);
            if(!digitalRead(T1))
            {                
                MSound(1, 100, 1000);                   // beep
                
                // Init
                ROBOT_STOP();                           // stop
                ROBOT_SPEED(100);                       // set speed to 100
                ROBOT_HEIGHT(0);                        // sit down
                ROBOT_GAINT_MODE(TRIPOD_6);             // tripod 6 gaint mode
                ROBOT_BALANCE_MODE_OFF();               // balance mode off
                ROBOT_PWR_ON();                         // power on
                                
                ROBOT_SPEED(20);                        // stand up
                for(int i = 0; i < _hight; i += 2)
                {
                    ROBOT_HEIGHT(i);
                    delay(25);
                }
                
                ROBOT_SPEED(50);                        // set speed to 50
                delay(1000);
                
                ROBOT_WALK_FWD();                       // walk forward
                delay(8000);                            // do this for 8 sec.
                
                ROBOT_STOP();                           // stop
                delay(5000);                            // do this for 5 sec.
                
                ROBOT_GAINT_MODE(TRIPOD_8);             // tripod 8 gaint
                                
                ROBOT_WALK_BWD();                       // walk backward
                delay(5000);                            // do this for 5 sec.
                
                ROBOT_STOP();                           // stop
                delay(3000);                            // do this for 3 sec.
                
                ROBOT_SPEED(20);                        // sit down
                for(int i = _hight; i > 2; i -= 2)
                {
                    ROBOT_HEIGHT(i);
                    delay(25);
                }
               
                ROBOT_SPEED(100);                       // set speed back to 100
                                
                ROBOT_PWR_OFF();                        // power off
                
            }
        }
    }
}

