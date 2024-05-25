/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : ReadValues_02.ino
Libraries   : SoftwareSerial.h, Hexapod_Lib.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- The exaple show the basic reading from the Locomotion-Controller
- The exaple use the hexapod library

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
                
                // read the akku voltage
                Serial.print("> Akku Voltage: ");
                Serial.println(ReadVoltage());
                
                // read the PS2-Controller status
                Serial.print("> PS2-Controller aktive: ");
                Serial.println(ReadPs2Status());
                
                // robot is walking? (0 = stopped, 1 = walking)
                Serial.print("> Is Walking: ");
                Serial.println(ReadIsWalking());
                
                // is robot power on? (0 = off, 1 = on)
                Serial.print("> Is Power On: ");
                Serial.println(ReadIsPowerOn());
                
                // read input IN1 (1 = open, 0 = pin is connected to ground)
                Serial.print("> IN1: ");
                Serial.println(ReadIN1());
                
                // read LEG-ADC SA0 to SA5 (values between 0 and 1023)
                Serial.print("> SA0: ");
                Serial.println(ReadLegAdc(0));
                
                Serial.print("> SA1: ");
                Serial.println(ReadLegAdc(1));
                
                Serial.print("> SA2: ");
                Serial.println(ReadLegAdc(2));
                
                Serial.print("> SA3: ");
                Serial.println(ReadLegAdc(3));
                
                Serial.print("> SA4: ");
                Serial.println(ReadLegAdc(4));
                
                Serial.print("> SA5: ");
                Serial.println(ReadLegAdc(5));
                
            }
        }
        
    }
}
