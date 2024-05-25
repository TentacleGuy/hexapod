/******************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : ServoCenter.ino
Libraries   : Servo.h
Author      : UlliS
Description : Center all servos

[ INFO: This example is ONLY for the Locomotion-Controller ]

******************************************************************/
#include <Servo.h>

#define Servo_S0    12
#define Servo_S1    5
#define Servo_S2    2
#define Servo_S3    3
#define Servo_S4    17
#define Servo_S5    16
#define Servo_S6    6
#define Servo_S7    7
#define Servo_S8    8
#define Servo_S9    25
#define Servo_S10   26
#define Servo_S11   27
#define Servo_S12   33
#define Servo_S13   34
#define Servo_S14   35
#define Servo_S15   36
#define Servo_S16   37
#define Servo_S17   40

#define USER_LED    69
#define LIVE_LED    22

Servo myservoS0;
Servo myservoS1;
Servo myservoS2;
Servo myservoS3;
Servo myservoS4;
Servo myservoS5;
Servo myservoS6;
Servo myservoS7;
Servo myservoS8;
Servo myservoS9;
Servo myservoS10;
Servo myservoS11;
Servo myservoS12;
Servo myservoS13;
Servo myservoS14;
Servo myservoS15;
Servo myservoS16;
Servo myservoS17;

int pos    = 1500; // center position is 1,5ms -> 1500µs
int _delay = 100;  // delay between set the servo pos, to reduce the supply current

/******************************************************************
SETUP
******************************************************************/
void setup() 
{
    pinMode(LIVE_LED,OUTPUT);
    pinMode(USER_LED,OUTPUT);
    digitalWrite(LIVE_LED,LOW);
    digitalWrite(USER_LED,HIGH);
    
    myservoS0.attach(Servo_S0);
    myservoS0.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS1.attach(Servo_S1);
    myservoS1.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS2.attach(Servo_S2);
    myservoS2.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS3.attach(Servo_S3);
    myservoS3.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS4.attach(Servo_S4);
    myservoS4.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS5.attach(Servo_S5);
    myservoS5.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS6.attach(Servo_S6);
    myservoS6.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS7.attach(Servo_S7);
    myservoS7.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS8.attach(Servo_S8);
    myservoS8.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS9.attach(Servo_S9);
    myservoS9.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS10.attach(Servo_S10);
    myservoS10.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS11.attach(Servo_S11);
    myservoS11.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS12.attach(Servo_S12);
    myservoS12.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS13.attach(Servo_S13);
    myservoS13.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS14.attach(Servo_S14);
    myservoS14.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS15.attach(Servo_S15);
    myservoS15.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS16.attach(Servo_S16);
    myservoS16.writeMicroseconds(pos);
    delay(_delay);
    
    myservoS17.attach(Servo_S17);
    myservoS17.writeMicroseconds(pos);
    delay(_delay);
    
    digitalWrite(USER_LED,LOW);
}

/******************************************************************
MAIN
******************************************************************/
void loop() 
{
    digitalWrite(LIVE_LED,HIGH);
    delay(100);
    digitalWrite(LIVE_LED,LOW);
    delay(100);
}

