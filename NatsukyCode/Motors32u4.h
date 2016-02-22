/* 
* Motors32u4.h
*
* Created: 12/02/2016 3:18:24 p. m.
* Author: Michael Vargas
*/


#ifndef __MOTORS32U4_H__
#define __MOTORS32U4_H__

#ifndef F_CPU
#define F_CPU 12000000UL
#endif


#include <avr/interrupt.h>

#define FAST_PWM 1
#define PHASE_CORRECT_PWM 0

#include "pinout32u4.h"


class Motors32u4
{
//variables
public:
protected:
private:

//functions
public:
     Motors32u4();
		/**
	 * \brief 
	 * Crea el objeto de la clase motores Configura el PWM segun los pines y asi asignar los respectivos registros OCRxA y OCRxB,
	 * como tambien configura el prescaler del timer para asi poder cambiar la frecuencia del PWM
	 * \param pins
	 * Poner los registros segun los pines que fueron conectados el puente-H, prescaler y modo PWM
	 * \return 0
	 */
	void init(unsigned char *pins[],prescalerTimer prescaler = PRESCALER_8,bool modePWM = FAST_PWM);
	/**
	 * \brief 
	 * Asigna la velocida al motor izquierdo
	 * \param speed
	 * Asigna la velocidad con valor velMaximo de 255
	 * \return void
	 */
	void speedLeft(int speed);
	/**
	 * \brief 
	 * Asigna la velocida al motor derecho
	 * \param speed
	 * Asigna la velocidad con valor velMaximo de 255
	 * \return void
	 */
	void speedRight(int speed);
		/**
	 * \brief 
	 * Asigna la velocidad de ambos motore
	 * \param speed
	 * Asigna la velocidad con valor velMaximo de 255
	 * \return void
	 */
    void setSpeeds(int left,int right);
	void stopBrakeRight();
	void stopBrakeLeft();
	void stopRight();
	void stopLeft();
	void stopMotors();
	void stopBrakeMotors();
	unsigned char getFrequency();
	~Motors32u4();
    private:
        volatile unsigned char* pinMotorLeftA;
        volatile unsigned char* pinMotorLeftB;
        volatile unsigned char* pinMotorRightA;
        volatile unsigned char* pinMotorRightB;
		prescalerTimer _prescaler;
        bool _modePWM;
		bool initialize;
		Motors32u4( const Motors32u4 &c );
		Motors32u4& operator=( const Motors32u4 &c );

}; //Motors32u4

#endif //__MOTORS32U4_H__
