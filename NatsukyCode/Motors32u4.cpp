/* 
* Motors32u4.cpp
*
* Created: 12/02/2016 3:18:24 p. m.
* Author: Michael Vargas
*/


#include <avr/interrupt.h>
#include <math.h>
#include "Motors32u4.h"
/*
extern "C" void TIMER2_OVF_vect() __attribute__((naked, __INTR_ATTRS));
extern "C" void TIMER2_OVF_vect()
{
	

}*/

// default constructor
Motors32u4::Motors32u4(){
	initialize = false;
} //Motors32u4
void Motors32u4::init(unsigned char *pins[],prescalerTimer prescaler,bool modePWM)
{	cli();
	static bool timer0,timer1,timer3;
	initialize = false;
	TCCR0A = TCCR1A = TCCR3A = 0;
	TCCR0B = TCCR1B = TCCR3B = 0;
	for (unsigned char i=0;i<4;i++)
	{
		if(pins[i] == (unsigned char*)&OCR0A){
			TCCR0A |= (1<<COM0A0) | (1<<COM0A1);
			Digital::setOutput(IO_PB7,LOW);
			timer0=true;
		}
		if(pins[i] == (unsigned char*)&OCR0B){
			TCCR0A |= (1<<COM0B0) | (1<<COM0B1);
			Digital::setOutput(IO_PD0,LOW);
			timer0=true;
		}
		if(pins[i] == (unsigned char*)&OCR1A) {
			TCCR1A |= (1<<COM1A0) | (1<<COM1A1);
			Digital::setOutput(IO_PB5,LOW);
			timer1=true;
		}
		if(pins[i] == (unsigned char*)&OCR1B){
			TCCR1A |= (1<<COM1B0) | (1<<COM1B1);
			Digital::setOutput(IO_PB6,LOW);
			timer1=true;
		}
		if(pins[i] == (unsigned char*)&OCR1C){
			Digital::setOutput(IO_PB7,LOW);
			TCCR1A |= (1<<COM1C0) | (1<<COM1C1);
			timer1=true;
		}
		if(pins[i] == (unsigned char*)&OCR3A){
			Digital::setOutput(IO_PC6,LOW);
			TCCR3A |= (1<<COM3A0) | (1<<COM3A1);
			timer3 = true;
		}
	}
	if(!timer0 && !timer1 && !timer3) return; //Si no se detecta los registros de memoria apropiados
	if(timer0){
		if(modePWM) TCCR0A |= 0x03; //Configura el timer en modo PWM rapido
		else TCCR0A |= 0x01; //Configura el timer en modo fase correcta
		TCCR0B = prescaler; //Asigna el prescaler
	}
	if(timer1){
		TCCR1A |= (1<<WGM10);
		if(modePWM){TCCR1B |= (1<<WGM12);}
		
		TCCR1B |= prescaler;
	}
	if(timer3){
		TCCR3A |= (1<<WGM30);
		if(modePWM) TCCR3B |= (1<<WGM32);
		TCCR3B |= prescaler;
	}
	pinMotorLeftA = pins[0];
	pinMotorLeftB = pins[1];
	pinMotorRightA = pins[2];
	pinMotorRightB = pins[3];
	*pinMotorLeftA = *pinMotorLeftB = *pinMotorRightA = *pinMotorRightB = 0; //Inicializa los motores
	_modePWM = modePWM;
	_prescaler = prescaler;
	initialize = true;
	sei();
	}
void Motors32u4::speedLeft(int speed)
{
    bool reverse=false;

	if(speed < 0){
	reverse = true;
	speed=-speed;	
	}
	if(speed > 0xFF) speed = 0xFF;
	if(!reverse){
		*pinMotorLeftA = 0;
		*pinMotorLeftB = speed;
	}else{
		*pinMotorLeftB = 0;
	    *pinMotorLeftA = speed;
	}
}
void Motors32u4::speedRight(int speed)
{
	bool reverse=false;
	
	if(speed < 0){
		reverse = true;
		speed=-speed;
	}	
	if(speed > 0xFF) speed = 0xFF;
	if(!reverse){
		*pinMotorRightA = 0;
		*pinMotorRightB = speed;
		}else{
		*pinMotorRightB = 0;
		*pinMotorRightA = speed;
	}
}

void Motors32u4::setSpeeds(int left,int right)
{
speedLeft(left);
speedRight(right);
}
//Frenado tipo brake en el motor derecho ( deja que la llanta siga girando hasta que frene)
void Motors32u4::stopBrakeRight()
{
	*pinMotorRightA = *pinMotorRightB = 0XFF;
}
//Frenado tipo brake en el motor izquierdo ( deja que la llanta siga girando hasta que frene)
void Motors32u4::stopBrakeLeft()
{
	*pinMotorLeftA = *pinMotorLeftB = 0xFF;
}
//Frena toltamente el motor derecho
void Motors32u4::stopRight()
{
	*pinMotorRightA = *pinMotorRightB = 0;
}
//Frena totalmente el motor izquierdo
void Motors32u4::stopLeft()
{
	*pinMotorLeftA = *pinMotorLeftB = 0;
}
//Deja girando libremente ambos motores
void Motors32u4::stopBrakeMotors(){
	stopBrakeLeft();
	stopBrakeRight();
}

unsigned char Motors32u4::getFrequency()
{
unsigned char typePWM;
	if(_modePWM) typePWM=1;
	else typePWM=2;
		
return ((F_CPU / (_prescaler*256*typePWM))/1000);
}

//Frena ambos motores
void Motors32u4::stopMotors(){
	stopRight();
	stopLeft();
}

// default destructor
Motors32u4::~Motors32u4()
{
	
} //~Motors32u4
