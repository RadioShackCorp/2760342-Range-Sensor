/***************************************************************************/	
//	Function: Measure the distance to obstacles in front and print the distance
//			  value to the serial terminal.The measured distance is from 
//			  the range 0 to 400cm(157 inches).
//	Hardware: Ultrasonic Range sensor
//	Arduino IDE: Arduino-1.0
//	Author:	 LG		
//	Date: 	 Jan 17,2013
//	Version: v1.0 modified by FrankieChu
//	by www.seeedstudio.com
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
	Serial.begin(9600);
}
void loop()
{
	long RangeInInches;
	long RangeInCentimeters;
	ultrasonic.DistanceMeasure();// get the current signal time;
    RangeInInches = ultrasonic.microsecondsToInches();//convert the time to inches;
	RangeInCentimeters = ultrasonic.microsecondsToCentimeters();//convert the time to centimeters
	Serial.println("The distance to obstacles in front is: ");
	Serial.print(RangeInInches);//0~157 inches
	Serial.println(" inch");
	Serial.print(RangeInCentimeters);//0~400cm
	Serial.println(" cm");
	delay(100);
}
