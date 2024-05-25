/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Infrared.ino
Libraries   : IRremote.h
Author      : UlliS
Description : Infrared RC5 code receiver

[ INFO: This example is ONLY for ARDUINO UNO or compatible boards ]

******************************************************************************/

// use Timer2
#include <IRremote.h>

// RC5
#define RECEIVE_PIN 7U

int RC5_Code         = 0;
int RC5_Tog_bit      = 0;
int RC5_Tog_bit_prev = 0;
int RC5_Addr         = 0;
int RC5_Addr_prev    = 0;
int RC5_Command      = 0;

IRrecv irrecv(RECEIVE_PIN);
decode_results results;

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
    
    irrecv.enableIRIn(); // start the receiver
    
}

/******************************************************************************
MAIN
******************************************************************************/
void loop() 
{
    
    if (irrecv.decode(&results)) 
    {
        
        // receive the next value
        irrecv.resume();
        
        // toggelbit auswerten 1 oder 0
        RC5_Tog_bit = (results.value >> 11) & 0x3;
        if(RC5_Tog_bit!=0) 
        {
            RC5_Tog_bit = 1;
        } 
        else 
        {
            RC5_Tog_bit = 0;
        }
        
        RC5_Addr    = ((results.value >> 6)) & 0x1F;
        RC5_Command = results.value & 0x3F;
        
        Serial.print(RC5_Addr);
        Serial.print(",");
        Serial.print(RC5_Tog_bit);
        Serial.print(",");
        Serial.println(RC5_Command);
        
    }
    
}
