/* 
* IRSensors.h
*
* Created: 15/02/2016 1:06:51 p. m.
* Author: Michael Vargas
*/


#ifndef __IRSENSORS_H__
#define __IRSENSORS_H__

#define NO_EMITTER_PIN 0xFF
#define EMITTERS_ON 1
#define  EMITTERS_OFF 0

class IRSensors
{
//variables
public:
protected:
private:

//functions
public:
	IRSensors();
	void init(unsigned char *pins,unsigned char numSensors,unsigned char numSamples,unsigned char emitterPin);
	void read(unsigned int* sensor_values);
	void calibrate();
	int measureLine(unsigned int* sensor_values,unsigned int noise = 60,bool white=false);
	void resetCalibration();
	void saveCalibration();
	void restoreCalibration();
	void emittersEn(bool state);
	unsigned int* calibratedMaximum;
	unsigned int* calibratedMinimum;
	~IRSensors();
protected:
private:
	void readCalibrated(unsigned int* sensor_values);
	IRSensors( const IRSensors &c );
	IRSensors& operator=( const IRSensors &c );
unsigned char* _pins;
unsigned char _numSensors;
unsigned char _numSamples;
int _lastValue;
unsigned char _emitterPin;
}; //IRSensors

#endif //__IRSENSORS_H__
