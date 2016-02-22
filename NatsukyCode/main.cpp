/*
 * NatsukyCode.cpp
 *
 * Created: 12/02/2016 3:01:16 p. m.
 * Author : Michael Vargas
 */ 
#define F_CPU 12000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "IRSensors.h"
#include "UART32u4.h"
#include "Motors32u4.h"

#define NUM_SENSORS         4			//Numero de sensores usados
#define NUM_SAMPLES			4			//Numero de muestras por sensor
#define EMITTER_PIN			IO_PE6		//Controla el encendido y apagado de los sensores
#define REVERSE             35			//Velocidad de reversa para cuando se salga de las curvas
#define VEL_INITIAL         40			//Velocidad inicial
#define RANGEBRAKE ((NUM_SENSORS-1)*500)//Rango en el que se activa el freno
#define TICKS_PER_METER 72				//Numero de ticks para que el robot avance 1 metro
#define TICKS_PER_ROUND 8               //Numero de ticks por vuelta en una llanta o el numero de franjas negras

//Variables para interrupciones
volatile bool wakeUp = false, isRun = false;
volatile unsigned long ticksEncoderL,ticksEncoderR;
volatile unsigned long millis;
volatile unsigned char max=VEL_INITIAL;
char inputString[64];
volatile bool stringComplete = false;  //Cuando se termina de recibir los datos

//Otras variables
unsigned int volt;
int position;
bool isCalibrated = false;

//Vectores
unsigned int sensorValues[NUM_SENSORS];
unsigned char pinsSensors[] = {4,5,6,7}; //Pone los canales para realizar la lectura analogica
unsigned char* pinsMotors[] = {(unsigned  char*) &OCR0A,(unsigned  char*) &OCR0B,(unsigned char*) &OCR1A,(unsigned char*) &OCR1B};//Asigna las direcciones de memoria de los registros de salida PWM
	
//Crea los objetos de cada clase
IRSensors sensors;
Motors32u4 motors;

//Protipado de funciones
void pid_calc(int setPoint);
inline void setMotors2(int left, int right);
inline void sleep_Natsuky();
void printCalibration();
void serialEvent();

int main(void)
{
	sensors.init(pinsSensors,NUM_SENSORS,NUM_SAMPLES,EMITTER_PIN); //configura los sensores
	motors.init(pinsMotors,PRESCALER_1,FAST_PWM); //Configura los motores con un PWM de 23 Khz
	initUART(38400); //Inicia comuniacion serial a 38400 bits por segundo
	_delay_ms(500);
	//Activa a la resistencia pull_up para los pulsadores
	Digital::setInput(IO_PB4,PULL_UP_ENABLED);
	Digital::setInput(IO_PD6,PULL_UP_ENABLED);
	uart_println("Line Follower Natsuky");
	uart_print("Frequencia motores->");
	uart_print(motors.getFrequency());
	uart_println(" Khz");
    //Calcular voltaje de bateria
	DIDR1 |= (ADC8D);
	for (unsigned char k=0;k<NUM_SAMPLES;k++) volt += Analog::read(8);
	if(volt > 300)	volt/=NUM_SAMPLES;
	else volt = 0;
	//Envia la información del voltaje de la bateria
	uart_print("Voltaje de bateria->");
	float volt_battery = ((volt*3.3)/1023.0)/(39.0/139.0);
	uart_print(volt_battery,3);
	uart_println(" Volts");
	//En caso que la bateria esta descargada parpadean los LEDs
	Digital::setOutput(IO_PD7,LOW);
/*
	if(volt_battery <= 3.7){
	for (unsigned char k=0;k<5;k++)
	{
		Digital::setOutput(IO_PB4,TOGGLE);
		_delay_ms(500);
	}	
	}
	_delay_ms(250);*/
    //sleep_Natsuky();
	TIMSK1 = (1<<TOIE1); //Activa la interrupcion por desbordamiento para leer los datos recibidos en la uart 
	sei();
	while(true){
		if (stringComplete) {
			uart_print((char*)inputString);
			uart_println("OK!");
			// Limipiar entrada
			memset(inputString,0,CANTIDAD_ELEMENTOS(inputString));
			stringComplete = false;
		}
		static unsigned char i;
			if (!Digital::isInputHigh(IO_PB4)){
				Digital::setOutput(IO_PD7,HIGH);
				sensors.emittersEn(EMITTERS_ON);
				uart_println("Calibrando sensores...");
				for (i=0;i<80;i++)
				{
					Digital::setOutput(IO_PD7,TOGGLE);
					sensors.calibrate();
					_delay_ms(20);
				}
				Digital::setOutput(IO_PD7,LOW);
				Digital::setOutput(IO_PB0,LOW);
				sensors.emittersEn(EMITTERS_OFF);
				isCalibrated=true;
				printCalibration();
				sensors.saveCalibration(); //Guarda la calibración 
				}else{
				if(!Digital::isInputHigh(IO_PD6)){
					sensors.emittersEn(EMITTERS_ON);
					if(!isCalibrated){
						Digital::setOutput(IO_PD7,HIGH);
						Digital::setOutput(IO_PB0,LOW);
						sensors.restoreCalibration(); //Restaura la calibracion
						printCalibration();
						isCalibrated=true;
					}
					break;
					_delay_ms(250);
				}
			}

		}
	Digital::setOutput(IO_PB0,HIGH);
	Digital::setOutput(IO_PD7,LOW);
	uart_println("Ready...!");
	//Encender Sensores
	sensors.emittersEn(EMITTERS_ON);
 	_delay_ms(1000);
	 millis=0;
	 ticksEncoderL = ticksEncoderR = 0;
	//PCICR |= (1<<PCIE0); //Activa la interrupcion por cambio de estado para despertar al microcontrolador
	//PCMSK0 |= ((1<<PCINT4) | (1<<PCINT2)); //El robot se despierta con el pulsador izquierdo que esta en el pin B4
// 	motors.setSpeeds(80,80);
// 	_delay_ms(300);
	cli();
	//Configurar el timer en modo normal para que se desborde cada 10us
	TCCR3A = 0;
	TCCR3B |= 1;
	TIMSK3 |= (1<<TOIE3);
	TCNT3 = 65416;
	sei();
	isRun=true;
    for(;;) 
    {	
	//static unsigned int setPoint = 1500;
	
/*
	if (stringComplete) {
		uart_print((char*)inputString);
		uart_println("OK!");
		if(inputString[0] == '1'){
			isRun = !isRun;
		 uart_print("Encoder Derecho:");
		 uart_print(ticksEncoderR);
		 uart_print("\n");
		 uart_print("Encoder Izquierdo");
 		 uart_print(ticksEncoderL);	
		 uart_print("\n");
		}
		// Limipiar entrada
	    memset(inputString,0,CANTIDAD_ELEMENTOS(inputString));
		stringComplete = false;
	}*/
	
	position = sensors.measureLine(sensorValues);
	pid_calc(2500);
	//uart_println(position);
/*
	for (int i=0;i<NUM_SENSORS;i++)
	{
		uart_print(sensorValues[i]);
		uart_print("	");
	}
	uart_println(position);
    	_delay_ms(80);*/
	}
}
ISR(TIMER1_OVF_vect){
serialEvent();
}
/*
ISR(PCINT0_vect){

}*/
void serialEvent() {
	static unsigned char i;
//i=0;
	while (availableUART()) {
			
		// get the new byte:
		char inChar = (char)readUART();
		// add it to the inputString:
		inputString[i] = inChar;
		//uart_println(inputString[i]);
		i++;
		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\n') {
			stringComplete = true;
			i=0;
		}
	}
}
inline void sleep_Natsuky(){

	/*
	NOTA:
    * El robot consume 68 mA en modo Sleep
	* Los sensores consumen 92 - 100 mA, por defecto estan apagados para ahorar energia   
	*/
	 power_twi_disable();
	 power_usb_disable();
	 set_sleep_mode(SLEEP_MODE_PWR_SAVE);//Establecemos el modo de bajo consumo.
	 //Desavtivar los modulos que no se usan para ahorrar energia
	 sleep_enable();//inicializamos el modo bajo consumo
	 sleep_mode();//ponemos el modo bajo consumo.
}

void pid_calc(int setPoint)
{
static const float Kp=0.06;
static const float Kd=2.2;
static int proportional = 0;      // proporcional
static int derivative;
static int last_proportional=0;
  // Aquí no estamos interesados ??en los valores
  // individuales de cada sensor
  // El término proporcional debe ser 0 cuando estamos en línea
  
	proportional = ((int)position) - setPoint;
	if (proportional <= -RANGEBRAKE )
	{
	if (last_proportional <= -RANGEBRAKE) motors.speedLeft(-REVERSE);
	else 	motors.stopLeft();
  	motors.stopBrakeRight();
  	_delay_us(1250);
  	}
  	else if ( proportional >= RANGEBRAKE)
  	{
	if (last_proportional >= RANGEBRAKE) motors.speedRight(-REVERSE);
  	else 	motors.stopRight();
    motors.stopBrakeLeft();
	_delay_us(1250);
  	}
	derivative = proportional - last_proportional;
	//Recordando la última posición
	last_proportional = proportional;
    // Calcula la diferencia entre la potencia de los dos motores [ m1 - m2 ].
    // Si es un número positivo, el robot gira a la [ derecha ]
    // Si es un número negativo, el robot gira a la [ izquierda ]
    //  y la magnitud del número determina el ángulo de giro.
   int power_difference = ( proportional * Kp ) + ( derivative * Kd );
    // Si ocidad diferencial es mayor a la posible tanto positiva como negativa,
    // asignar la máxima permitida
    if ( power_difference > max ) power_difference = max;
    else if ( power_difference < -max ) power_difference = -max;
    // Asignar velocidad calculada en el poder diferencial de los motores
   ( power_difference < 0 ) ?   //setMotors2(max+power_difference, max)  : setMotors2(max, max-power_difference);
    motors.setSpeeds(max+power_difference, max)  : motors.setSpeeds(max, max-power_difference);
	

}
ISR(TIMER3_OVF_vect){
	TCNT3 = 65416;
	//static int trazado;
	//static bool recta;
	static unsigned int micros10;
	static bool Flag1,Flag2;
	static unsigned int ticksAuxR,ticksAuxL;
	//static unsigned long timeAcceleration;
	micros10++;
	if(micros10==100){
		micros10=0;
		millis++;
	}
	if(bitRead(PIND,5)){
	if(!Flag2){
	ticksEncoderR++;
	ticksAuxR++;
	Flag2 = true;
	}
	}else{
	Flag2=false;
	}
	if(bitRead(PINB,2)){
	if(!Flag1){
	ticksEncoderL++;
	ticksAuxL++;
	Flag1 = true;
	}
	}else{
	Flag1=false;
	}
	//Determinar trazado
	if (ticksAuxL > 8 || ticksAuxR > 8)
	{
	if(abs(ticksAuxL - ticksAuxR) <= 2 ){ 	//El robot esta en recta
			PORTD |= (1<<7); 
			PORTB |= (1<<0);
			if(millis >= 130){
				max+=2;
				millis=0;
			}
	}else{
		   millis = 0;
		   max = VEL_INITIAL;
		   PORTB &= ~(1<<0); //El robot esta en curva
		   PORTD &= ~(1<<7);
		}
	ticksAuxR = ticksAuxL = 0;
	}

	
}
/*
inline void setMotors2(int left, int right){
static float FACTOR_SPEED=0.175f; //Factor de velocidad para girar con una llanta hacia atras
	if(left==0 && right == max){
		motors.speedLeft(-(max*FACTOR_SPEED));motors.speedRight(max); //if(bitRead(PINF,1)) _delay_us(10);
		}else if(right==0 && left == max){
	    motors.speedLeft(max);motors.speedRight(-(max*FACTOR_SPEED)); //if(bitRead(PINC,7)) _delay_us(10);
		}else if (left > TOPE || right > TOPE){
		if(left>TOPE){motors.speedLeft(left);}
		if(right>TOPE){motors.speedRight(right);}
		}else{
		if(right<=TOPE){motors.speedRight(-15);}
		if(left<=TOPE){motors.speedLeft(-15);}
	}
}*/
void printCalibration()
{
	static unsigned char i;
	uart_println("Valores de calibracion:");
	for (i = 0;i < 4; i++)
	{
		uart_print(sensors.calibratedMaximum[i]);
		uart_print("	");
	}
	uart_println("	");
	for (i=0;i<4;i++)
	{
		uart_print(sensors.calibratedMinimum[i]);
		uart_print("	");
	}
	uart_println("	");
}