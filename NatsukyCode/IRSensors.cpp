/* 
* IRSensors.cpp
*
* Created: 15/02/2016 1:06:51 p. m.
* Author: Michael Vargas
*/


#include "IRSensors.h"
#include "EEPROM.h"
#include "pinout32u4.h"
#include <stdlib.h>


// default constructor
IRSensors::IRSensors()
{
 _pins = 0;
_numSensors = 0;
_numSamples = 4;
} //IRSensors

void IRSensors::init(unsigned char* pins,unsigned char numSensors,unsigned char numSamples,unsigned char emitterPin)
{
	Analog::init();
	calibratedMinimum=0;
	calibratedMaximum=0;
	_numSensors = numSensors;
	_numSamples = numSamples;
	if (_pins == 0)
	{
		 _pins = (unsigned char*)malloc(sizeof(unsigned char)*_numSensors);
	}
	unsigned char i;
	for (i = 0; i < _numSensors; i++)
	{
	 _pins[i] = pins[i];
	}
	DDRF &= (~(1<<0) & ~(1<<1));
	DDRC &= (~(1<<7) & ~(1<<6));	
	Digital::setOutput(emitterPin,LOW);
	_emitterPin = emitterPin;
}

void IRSensors::read(unsigned int* sensor_values)
{
	if (_pins == 0)
	return;
	unsigned char i,j;
	// reset the values
	for(i = 0; i < _numSensors; i++) sensor_values[i] = 0;

	for (j = 0; j < _numSamples; j++)
	{
	 for (i = 0; i <_numSensors; i++)
	 {
		 if(i>= 0 && i<_numSensors) sensor_values[i] += Analog::read(_pins[i]);   
	 }
	}
	// get the rounded average of the readings for each sensor
	for (i = 0; i < _numSensors; i++){
	sensor_values[i] = (sensor_values[i] + (_numSamples >> 1)) / _numSamples;}
  }

void IRSensors::calibrate(){
	    int i;
	    unsigned int sensor_values[_numSensors];
	    unsigned int max_sensor_values[_numSensors];
	    unsigned int min_sensor_values[_numSensors];

	    // Allocate the arrays if necessary.
	    if(calibratedMaximum == 0)
	    {
		    calibratedMaximum = (unsigned int*)malloc(sizeof(unsigned int)*_numSensors);

		    // If the malloc failed, don't continue.
		    if(calibratedMaximum == 0)
		    return;

		    // Initialize the max and min calibrated values to values that
		    // will cause the first reading to update them.

		    for(i=0;i<_numSensors;i++)
		    calibratedMaximum[i] = 0;
	    }
	    if(calibratedMinimum == 0)
	    {
		    calibratedMinimum = (unsigned int*)malloc(sizeof(unsigned int)*_numSensors);

		    // If the malloc failed, don't continue.
		    if(calibratedMinimum == 0)
		    return;

		    for(i=0;i<_numSensors;i++)
		    calibratedMinimum[i] = 1023;
	    }

	    int j;
	    for(j=0;j<10;j++)
	    {
		    read(sensor_values);
		    for(i=0;i<_numSensors;i++)
		    {
			    // set the max we found THIS time
			    if(j == 0 || max_sensor_values[i] < sensor_values[i])
			    max_sensor_values[i] = sensor_values[i];

			    // set the min we found THIS time
			    if(j == 0 || min_sensor_values[i] > sensor_values[i])
			    min_sensor_values[i] = sensor_values[i];
		    }
	    }

	    // record the min and max calibration values
	    for(i=0;i<_numSensors;i++)
	    {
	        if(min_sensor_values[i] > calibratedMaximum[i])
		    calibratedMaximum[i] = min_sensor_values[i];
		    if(max_sensor_values[i] < calibratedMinimum[i])
		    calibratedMinimum[i] = max_sensor_values[i];
	    }
}

void IRSensors::readCalibrated(unsigned int* sensor_values)
{
	int i;
	read(sensor_values);
	   for(i=0;i<_numSensors;i++)
	   {
		 unsigned int denominator;
	     denominator = calibratedMaximum[i] - calibratedMinimum[i];
	     signed int x = 0;
	     if(denominator != 0)
	     x = (((signed long)sensor_values[i]) - calibratedMinimum[i]) * 1000 / denominator;
	     if(x < 0) x = 0;
	     else if(x > 1000)
	     x = 1000;
	     sensor_values[i] = x;
     }
}

int IRSensors::measureLine(unsigned int* sensor_values,unsigned int noise,bool white)
{
unsigned char i;
bool on_line = false;
unsigned long avg=0; // this is for the weighted total, which is long
unsigned int sum=0; // this is for the denominator which is <= 64000

	readCalibrated(sensor_values);

   for(i=0;i<(_numSensors+2);i++) {
	  unsigned int value =  (i >= 1 && i < 5) ? sensor_values[i-1] : 0;
     switch(i){
		case 0: value = (bitRead(PINF,1) ? 1000 : 0); break;
		case 5: value = (bitRead(PINC,7) ? 1000 : 0); break;
	 }
	if(white) value = 1000-value;
	
	if(value > 60) on_line = true;
	   
	if(value > noise) {
     //  if(value > 800) value=1000;
		   avg += (long)(value) * (i * 1000);
		   sum += value;
	   }
   }
   if(!on_line)
   {
	   // If it last read to the left of center, return 0.
	   if(_lastValue < (_numSensors+1)*1000)
	   return 0;

	   // If it last read to the right of center, return the max.
	   else
	   return (_numSensors+1)*1000;

   }

	_lastValue = avg/sum;
	
	return _lastValue;
}

void IRSensors::resetCalibration()
{
	unsigned char i;
	if(calibratedMinimum){
	for(i=0;i<_numSensors;i++)
	calibratedMinimum[i] = 1023;}
	if (calibratedMaximum)
	{
	for(i=0;i<_numSensors;i++)
	calibratedMaximum[i] = 0;
	}
}
void IRSensors::emittersEn(bool state){
	if(_emitterPin==255) return;
	Digital::setOutput(_emitterPin,state);
}
void IRSensors::saveCalibration()
{
	int i=0;
	if(calibratedMinimum){
		for (i=0;i<_numSensors;i++)
		{
			EEPROM.put(i*4,calibratedMaximum[i]);
			EEPROM.put((i*4)+2,calibratedMinimum[i]);
		}	
	}
}

void IRSensors::restoreCalibration()
{
	int i;
	if(calibratedMaximum == 0)
	{
		calibratedMaximum = (unsigned int*)malloc(sizeof(unsigned int)*_numSensors);

		// If the malloc failed, don't continue.
		if(calibratedMaximum == 0)
		return;

		// Initialize the max and min calibrated values to values that
		// will cause the first reading to update them.

		for(i=0;i<_numSensors;i++)
		calibratedMaximum[i] = 0;
	}
	if(calibratedMinimum == 0)
	{
		calibratedMinimum = (unsigned int*)malloc(sizeof(unsigned int)*_numSensors);

		// If the malloc failed, don't continue.
		if(calibratedMinimum == 0)
		return;

		for(i=0;i<_numSensors;i++)
		calibratedMinimum[i] = 1023;
	}

	for (i=0;i<_numSensors;i++)
	{
	EEPROM.get(i*4,calibratedMaximum[i]);
	EEPROM.get((i*4)+2,calibratedMinimum[i]);
	}
}

extern "C" void __cxa_pure_virtual() { while (1); }


// default destructor
IRSensors::~IRSensors()
{
	free(calibratedMinimum);
	free(_pins);
	free(calibratedMaximum);
} //~IRSensors
