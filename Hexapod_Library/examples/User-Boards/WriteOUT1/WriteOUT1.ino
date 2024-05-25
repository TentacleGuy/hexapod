/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : WriteOUT1.ino
Libraries   : SoftwareSerial.h, Hexapod_Lib.h
Author      : UlliS
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

- The example set & reset the OUT1 output on the Locomotion-Controller

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
// LED flash variables
int ledState = LOW; // ledState used to set the LED
unsigned long previousMillis = 0; // will store last time LED was updated
const long interval = 500;  

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
        // send data to Locomotion-Controller
        //-----------------------------------------
        if(!digitalRead(T1)) // debounce T1
        {
            delay(50);
            if(!digitalRead(T1)) 
            {
                MSound(1, 100, 1000);
                
                // flash the LED on OUT1 w/o delay
                int cnt = 0;
                do 
                {
                    unsigned long currentMillis = millis();
                    if (currentMillis - previousMillis >= interval) 
                    {
                        // save the last time you blinked the LED
                        previousMillis = currentMillis;
                        
                        // if the LED is off turn it on and vice-versa:
                        if (ledState == LOW) 
                        {
                            ledState = HIGH;
                        } 
                        else 
                        {
                            ledState = LOW;
                        }
                        
                        // set the LED with the ledState of the variable:
                        WriteOUT1(ledState);
                        cnt++; 
                    }
                    
                }while(cnt<10);
                
                // end
                delay(1500);
                for(int i = 0; i < 3; i++)
                {
                    MSound(3, 100, 1000, 50, 2000, 100, 3500);
                    delay(100);
                }
            
            }
        }
    }
}
