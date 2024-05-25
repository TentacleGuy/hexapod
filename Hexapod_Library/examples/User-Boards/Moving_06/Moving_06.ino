/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Moving_06.ino
Libraries   : SoftwareSerial.h, Hexapod_Lib.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- The sample show the basic movements with the hexapod library
- Main focus is the the move functions and the status readings

- Start the program with T1
- Interrupt the program with the PS2-Gamepad (press the start button)
- Use the terminal to see the process

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
                
                //------------------------------
                // MOVE DEMO
                //------------------------------
                Serial.println("> START DEMO...");
                ROBOT_INIT();                               // init
                Serial.println("> ROBOT INIT");
                delay(1000);                                // wait 1 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                Serial.print("> Is Power On: ");
                Serial.println(ReadIsPowerOn());            // is power on?
                
                ROBOT_PWR_ON();                             // power on
                Serial.println("> ROBOT ON");
                delay(1000);                                // wait 1 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                Serial.print("> Is Power On: ");
                Serial.println(ReadIsPowerOn());            // is power on?
                
                ROBOT_HEIGHT(50);                           // set hight to 50
                Serial.println("> SET ROBOT HEIGHT");
                delay(2500);                                // give the servo time to work
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                
                ROBOT_WALK_FWD();                           // walk forward
                Serial.println("> ROBOT FWD");
                delay(5000);                                // do this for 5 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                Serial.print("> Is Walking: ");
                Serial.println(ReadIsWalking());            // robot is walking?
                
                ROBOT_STOP();                               // stop
                Serial.println("> ROBOT STOP");
                delay(3000);                                // do this for 3 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                Serial.print("> Is Walking: ");
                Serial.println(ReadIsWalking());            // robot is walking?
                
                ROBOT_HEIGHT(30);                           // set height to 30 (low)
                Serial.println("> SET NEW ROBOT HEIGHT");
                delay(2000);                                
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                
                ROBOT_SPEED(25);                            // set speed to 25 (fast)
                Serial.println("> SET NEW ROBOT SPEED");
                delay(500);
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                
                ROBOT_WALK_FWD();                           // walk forward
                Serial.println("> ROBOT FWD");                          
                delay(5000);                                // do this for 5 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                
                ROBOT_STOP();                               // stop
                Serial.println("> ROBOT STOP");
                delay(3000);                                // do this for 3 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                
                ROBOT_TURN_RIGHT();                         // turn right
                Serial.println("> ROBOT TURN RIGHT");
                delay(15000);                               // do this for 15 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                
                ROBOT_STOP();                               // stop
                Serial.println("> ROBOT STOP");
                delay(3000);                                // wait 3 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                
                ROBOT_WALK_FWD();                           // walk forward
                Serial.println("> ROBOT FWD");
                delay(5000);                                // do this for 5 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                
                ROBOT_STOP();                               // stop
                Serial.println("> ROBOT STOP");
                delay(3000);                                // wait 3 sec.
                if(ReadPs2Status()){goto _exit;}            // if the PS2-Gamepad active? yes goto exit point
                
                ROBOT_PWR_OFF();                            // power off
                Serial.println("> ROBOT OFF");
                Serial.println("> END DEMO...");
                
                delay(1500);                                // end beeping
                for(int i = 0; i < 3; i++)
                {
                    MSound(3, 100, 1000, 50, 2000, 100, 3500);
                    delay(100);
                }
                
                Serial.print("> Is Power On: ");
                Serial.println(ReadIsPowerOn());
                
            }
        }
    }
    
_exit:
    // exit point
    Serial.println("> PS2-Controller active -> PROGRAM EXIT");
    MSound(3, 100, 1000, 50, 2000, 100, 1000);
    
}

