//Arduino Pin Connection
#define ENA 7
#define IN1 4
#define IN2 2
#define IN3 8
#define IN4 12
#define ENB 13
#define base 3
#define shoulder 5
#define elbow 6
#define forearm 9
#define wrist 10
#define endeffector 11

#define maxSpeed 60
#define minSpeed 0



#include <string.h>
#define IBUS_BUFFSIZE 32    
#define IBUS_MAXCHANNELS 10 // I am using only 10 channels because my TX (FlySky i6) supports max 10 channels
#include <Servo.h>
static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
static uint16_t rcValue[IBUS_MAXCHANNELS];

#include<SoftPWM.h>

static boolean rxFrameDone;

//signal from the Receiver
int ch_width_1; 
int ch_width_2;
int ch_width_3;
int ch_width_4;
int ch_width_5;
int ch_width_6;
int ch_width_7;
int ch_width_8;
int ch_width_9;
int ch_width_10;

//servo control
Servo armbase; //
Servo armshoulder;
Servo armwrist;
Servo armforearm;
Servo armendeffector;
Servo armelbow;

void setup()
{
    Serial.begin(115200);
    //Robotic Arm Servo Initialization
    // armbase.attach(base);
    // armshoulder.attach(shoulder);
    // armwrist.attach(wrist);
    // armforearm.attach(forearm);
    // armendeffector.attach(endeffector);
    // armelbow.attach(elbow);
    pinMode(ENA,OUTPUT);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);
    pinMode(ENB,OUTPUT);
    SoftPWMBegin();
    SoftPWMSet(ENA, 0);
    SoftPWMSet(ENB, 0);

}

void loop()
{
    readRx();
}

void readRx()
{
rxFrameDone = false;

if (Serial.available())
{
    uint8_t val = Serial.read();
    // Look for 0x2040 as start of packet
    if (ibusIndex == 0 && val != 0x20)
    {
    ibusIndex = 0;
    return;
    }
    if (ibusIndex == 1 && val != 0x40)
    {
    ibusIndex = 0;
    return;
    }

    if (ibusIndex == IBUS_BUFFSIZE)
    {
    ibusIndex = 0;
    int high=3;
    int low=2;
    for(int i=0;i<IBUS_MAXCHANNELS; i++)
    {
        rcValue[i] = (ibus[high] << 8) + ibus[low];
        high += 2;
        low += 2;
    }
    ch_width_1 = map(rcValue[0], 1000, 2000, 1000, 2000);
    // ch1.writeMicroseconds(ch_width_1);

    // Serial.print(ch_width_1);
    // Serial.print("     ");

   // ch_width_2 = map(rcValue[1], 1000, 2000, 1000, 2000);
   // ch2.writeMicroseconds(ch_width_2);

   // Serial.print(ch_width_2);
  //  Serial.print("     ");

    ch_width_3 = map(rcValue[2], 1000, 2000, 1000, 2000);
   // ch3.writeMicroseconds(ch_width_3);

    //Serial.print(ch_width_3);
   // Serial.print("     ");

   // ch_width_4 = map(rcValue[3], 1000, 2000, 1000, 2000);
   // ch4.writeMicroseconds(ch_width_4);

   // Serial.print(ch_width_4);
   // Serial.println("     ");
                            
    rxFrameDone = true;
    if(ch_width_3 >1550)
    {
        setSpeed(map(ch_width_3,1550,2000,minSpeed,maxSpeed));
        movement('F');
    }
    if(ch_width_3 < 1450)
    {
        setSpeed(abs(map(ch_width_3,1450,1000,minSpeed,maxSpeed)));
        movement('B');
    }
    if(ch_width_1 >1550)
    {
        setSpeed(map(ch_width_1,1550,2000,minSpeed,maxSpeed));
        movement('R');
    }
    if(ch_width_1 < 1450)
    {
        setSpeed(abs(map(ch_width_1,1450,1000,minSpeed,maxSpeed)));
        movement('L');
    }
    if(((ch_width_3>1400 && ch_width_3 <1600) && (ch_width_1>1400 && ch_width_1 <1600)))
    {
        setSpeed(0);
        movement('S');
    }
    return;
    }
    else
    {
    ibus[ibusIndex] = val;
    ibusIndex++;
    }
}
}


void movement(char command)
{
    if(command == 'F')
    {
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,HIGH);
    }
   if(command == 'B')
    {
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
    }
   if(command == 'L')
    {
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,HIGH);
    }
    if(command == 'R')
    {
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
    }
    if(command == 'S')
    {
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,LOW);
    }
}

void setSpeed(int motorSpeed)
{
    SoftPWMSetPercent(ENA, motorSpeed);
    SoftPWMSetPercent(ENB, motorSpeed);
    // analogWrite(ENA,motorSpeed);
    // analogWrite(ENB,motorSpeed);
}