/***************************************************************************/  
// Copyright 2019, Howard Bishop, Gnu Lesser General Public License as listed
//  below.
// Includes source from the Radio Shack Distance sensor, 
// the comments for which are also included below. 
// Function: Measure the distance to obstacles in front and print the distance
//        value to the serial terminal.The measured distance is from 
//        the range 0 to 400cm(157 inches).
//  Hardware: Ultrasonic Range sensor
//  Arduino IDE: Arduino-1.0
//  Author:  LG   
//  Date:    Jan 17,2013
//  Version: v1.0 modified by FrankieChu
//  by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
//
/*****************************************************************************/
#include "Arduino.h"
const byte POWERUP = 0; // power on in a known default state
const byte POST = 1;    // do a Power On Self-Test
const byte WAITING = 2; // Waiting is where we read distance
const byte EGR = 3;     // will flash EGR lamp
const byte CEL = 4;     // EGR Steady, Flash Check Engine Lamp
const byte QUIET = 5;   // Give a little dead time before restart

//                      PWR  POST  WAIT  EGR   CEL   QUIET
const long PERIOD[] = {1000, 3000,  100, 5000, 5000, 1000};
// let's say 1 is steady, 2 is blinking.  
// Low nibble is EGR, upper nibble is CEL
const byte LAMPS[] =  {0x00, 0x11, 0x00, 0x02, 0x21,  0x00};
const long LAMP_PERIOD = 500;
const int CEL_Pin = 13;
const int EGR_Pin = 12;
int CEL_Lamp = LOW;
int EGR_Lamp = LOW;
unsigned long prevLampTime = 0;
unsigned long stateDuration = 0;
unsigned long prevStateTime = 0;
byte lampCommand;
byte currentState;

class Ultrasonic
{
  public:
    Ultrasonic(int pin);
    void DistanceMeasure(void);
    long microsecondsToCentimeters(void);
    long microsecondsToInches(void);
  private:
    int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
        long duration;// the Pulse time received;
};
Ultrasonic::Ultrasonic(int pin)
{
  _pin = pin;
}
/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void)
{
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_pin,LOW);
  pinMode(_pin,INPUT);
  duration = pulseIn(_pin,HIGH);
}
/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void)
{
  return duration/29/2; 
}
/*The measured distance from the range 0 to 157 Inches*/
long Ultrasonic::microsecondsToInches(void)
{
  return duration/74/2; 
}

Ultrasonic ultrasonic(7);
void setup()
{
  pinMode(CEL_Pin, OUTPUT);
  pinMode(EGR_Pin, OUTPUT); 
  currentState = POWERUP;
  lampCommand = 0x00;
  Serial.begin(9600);
}
void loop()
{
  unsigned long currentMillis = millis(); 

  if (currentMillis - prevStateTime > PERIOD[currentState]) {
    prevStateTime = currentMillis;
    boolean stateComplete = false;

    switch (currentState) {
      case POWERUP:
         stateComplete = true;
         break;
      case POST:
         stateComplete = true;
         break;
      case WAITING:
           //long RangeInInches;
         long RangeInCentimeters;
         ultrasonic.DistanceMeasure();// get the current signal time;
         RangeInCentimeters = ultrasonic.microsecondsToCentimeters();
         Serial.print(RangeInCentimeters);//0~400cm
         Serial.println(" ");

         // remember to put a first order filter here to stabilize
         if (RangeInCentimeters < 10) {
            stateComplete = true;
         }
         break;      
      case EGR:
         stateComplete = true;
         break;
      case CEL:
         stateComplete = true;
         break;
      case QUIET:
         stateComplete = true;
         break;
    }
    if (stateComplete){
      if (currentState == QUIET) {
        currentState = WAITING;
      }
      else {
        currentState++;
      }
    }
    Serial.print("Current State: ");
    Serial.println(currentState);
  }
  if (currentMillis - prevLampTime > LAMP_PERIOD) {
    prevLampTime = currentMillis;
    if (LAMPS[currentState] & 0x20) {
       CEL_Lamp = (LAMPS[currentState] & 0x20 && CEL_Lamp == LOW)? HIGH : LOW;
    }
    else {
       CEL_Lamp = (LAMPS[currentState] & 0x10)? HIGH : LOW;
    }
    if (LAMPS[currentState] & 0x02) {
       EGR_Lamp = (LAMPS[currentState] & 0x02 && EGR_Lamp == LOW)? HIGH : LOW;
    }
    else {
      EGR_Lamp = (LAMPS[currentState] & 0x01)? HIGH : LOW;   
    }
    digitalWrite(CEL_Pin, CEL_Lamp);
    digitalWrite(EGR_Pin, EGR_Lamp);
  }

}
