/******************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : LED.ino
Libraries   :
Author      : UlliS
Description : Blink the LIVE and USER LED

[ INFO: This example is ONLY for the Locomotion-Controller ]

******************************************************************/

#define USER_LED    69
#define LIVE_LED    22

/******************************************************************
SETUP
******************************************************************/
void setup() 
{
    pinMode(USER_LED,OUTPUT);
    pinMode(LIVE_LED,OUTPUT);
}

/******************************************************************
MAIN
******************************************************************/
void loop() 
{
    digitalWrite(USER_LED,HIGH);
    delay(250);
    digitalWrite(LIVE_LED,HIGH);
    delay(25);
    digitalWrite(USER_LED,LOW);
    delay(250);
    digitalWrite(LIVE_LED,LOW);
    delay(25);
}
