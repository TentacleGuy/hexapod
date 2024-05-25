/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Switches.ino
Libraries   :
Author      : UlliS
Description : Read the switch T1 and T2
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

// 1. Upload the code to a Arduino-UNO board
// 2. Open the terminal and press T1 or T2 and show the results

******************************************************************************/

// Arduino or NodeMCU (select one)
#define ARDUINO
//#define NODEMCU

// USER-BOARD PINS (ARDUINO UNO or compatible)
#ifdef ARDUINO
    #define T1                      A2
    #define T2                      A3
#endif

// USER-BOARD PINS (NodeMCU)
#ifdef NODEMCU
    #define T1                      10
    #define T2                      A0
#endif


/******************************************************************************
SETUP
******************************************************************************/
void setup()
{

    Serial.begin(19200);
    while (!Serial) 
    {
        ; // wait for serial port to connect. Needed for Leonardo only
    }
    
    // switches as input
    #ifdef ARDUINO
        pinMode(T1,INPUT);
        pinMode(T2,INPUT);
        //digitalWrite(T1,HIGH);
        //digitalWrite(T2,HIGH);
    #endif
    
    #ifdef NODEMCU
        pinMode(T1,INPUT);
    #endif
    
}

/******************************************************************************
MAIN
******************************************************************************/
void loop()
{
    // show the results in the terminal
    
    Serial.print("T1: ");
    Serial.println(digitalRead(T1));
    
    #ifdef ARDUINO
        Serial.print("T2: ");
        Serial.println(digitalRead(T2));
    #else
        Serial.print("T2: ");
        Serial.println(analogRead(T2));
    #endif
    
    Serial.println();
    
    delay(500);
}
