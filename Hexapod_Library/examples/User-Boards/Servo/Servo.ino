/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Servo.ino
Libraries   : Servo.h
Author      : UlliS
Description : Sweep the Servo on SU1, SU2 and SU3
******************************************************************************

[ INFO: This example is for ARDUINO UNO or compatible boards and NodeMCU ]

Original by BARRAGAN <http://barraganstudio.com>
This example code is in the public domain.
http://www.arduino.cc/en/Tutorial/Sweep

Modified 06 Nov 2017
by Ullis
******************************************************************************/
#include <Servo.h>

// Arduino or NodeMCU (select one)
#define ARDUINO
//#define NODEMCU


// create servo object to control a servo
Servo myservoSU1;
Servo myservoSU2;
Servo myservoSU3;

// USER-BOARD PINS (ARDUINO UNO or compatible)
#ifdef ARDUINO
    #define SU1                     6
    #define SU2                     5
    #define SU3                     9
#endif

// USER-BOARD PINS (NodeMCU)
#ifdef NODEMCU
    #define SU1                     12
    #define SU2                     13
    #define SU3                     15
#endif

// variable to store the servo position
int pos = 0;    

/******************************************************************************
SETUP
******************************************************************************/
void setup() 
{
    myservoSU1.attach(SU1);    // attaches the servo on pin 6 to the servo object
    myservoSU2.attach(SU2);    // attaches the servo on pin 5 to the servo object
    myservoSU3.attach(SU3);    // attaches the servo on pin 9 to the servo object
}

/******************************************************************************
MAIN
******************************************************************************/
void loop() 
{
    // goes from 0 degrees to 180 degrees
    for (pos = 0; pos <= 180; pos += 1) 
    {
        // in steps of 1 degree
        // tell servo to go to position in variable 'pos'
        myservoSU1.write(pos);
        myservoSU2.write(pos);
        myservoSU3.write(pos);
        // waits 15ms for the servo to reach the position
        delay(10);
    }
    
    // goes from 180 degrees to 0 degrees
    // tell servo to go to position in variable 'pos'
    for (pos = 180; pos >= 0; pos -= 1) 
    {
        myservoSU1.write(pos);
        myservoSU2.write(pos);
        myservoSU3.write(pos);
        // waits 15ms for the servo to reach the position
        delay(20);
    }
}

