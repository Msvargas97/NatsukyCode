/*
* NatsukyCode.cpp
*
* Created: 12/02/2016 3:01:16 p. m.
* Author : Michael Vargas
*/
#define F_CPU 12000000UL //El robot corre con un cristal de 12Mz y a 3.3V
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "IRSensors.h"
#include "UART32u4.h"
#include "Motors32u4.h"

#define NUM_SENSORS         4			//Numero de sensores analogos para seguir linea los otros se usan como digitales
#define NUM_SAMPLES			4			//Numero de muestras por sensor
#define EMITTER_PIN			IO_PE6		//Controla el encendido y apagado de los sensores
#define VEL_INITIAL         80			//Velocidad inicial durante los primeros 300 ms para que el robot no se levante bruscamente
#define VEL_MAX				120			//Velocida maxima que alcanzara en la mayoria del recorrido
#define VEL_RECTAS			(VEL_MAX+30)//Velocidad que alcanzara en rectas
#define RANGEBRAKE ((NUM_SENSORS-1)*500)//Rango en el que se activa el freno
#define TICKS_PER_METER 72				//Numero de ticks para que el robot avance 1 metro
#define TICKS_PER_ROUND 8               //Numero de ticks por vuelta en una llanta o el numero de franjas negras
#define DELTA_STOP 200 //Rampa de desacceleracion para cuando se frene el robot usando el bluetooth
#define OFFSET_TEMP_SENSORS 9
#define TIME_START 1000
//Variables para interrupciones
volatile bool isRun = false;//Sirve para detener el robot
volatile unsigned long ticksEncoderL,ticksEncoderR; //Almacenan los ticks
volatile unsigned long millis;//Tiempo en millisegundos desde que el robot inicia el recorrido
volatile unsigned char max=VEL_INITIAL; //Asigna la velocidad inicial
char inputString[10]; //Buffer para almacenar los datos enviados por bluetooth
volatile bool stringComplete = false,enableBT = false,printPosition=false;;  //Cuando se termina de recibir los datos
volatile char option; //Determina la opcion escogida mediante los dos pulsadores

//Otras variables
unsigned int i;
unsigned int volt; //Guarda el valor en ADC del voltaje de la bateria
float volt_battery; //Almacena la tension de la bateria
int position; // Determina la posicion del robot respecto a la linea
volatile bool isCalibrated = false; //Variable para saber si se calibraron los sensores
//Vectores
unsigned int sensorValues[NUM_SENSORS]; //Vector para almacenar los valores de los sensores
unsigned char pinsSensors[] = {4,5,6,7}; //Pone los canales para realizar la lectura analogica
//Vector de punteros
unsigned char* pinsMotors[] = {(unsigned  char*) &OCR0A,(unsigned  char*) &OCR0B,(unsigned char*) &OCR1A,(unsigned char*) &OCR1B};//Asigna las direcciones de memoria de los registros de salida PWM
//Crea los objetos de cada clase
IRSensors sensors;
Motors32u4 motors;

//Protipado de funciones
void pid_calc(int setPoint); //Realiza los calculos del PID y mantiene el robot en linea, según el setPoint ingresado
inline void sleepNatsuky();//Funcion inline para poner al robot en modo reposo y ahorrar bateria
void printCalibration(); //Imprime los valores de calibracion
void serialEvent(); //Captura los datos enviados por bluetooth
void wakeNatsuky(); //Rutina que realiza despues de despertar al microcontrolador
void calibrationSensors();
void readBattery();
int GetTemp(void);
void restoreLastCalibration();


int main(void)
{
	int setPoint = 2500;
	//Activa a la resistencia pull_up para los pulsadores
	Digital::setInput(IO_PB4,PULL_UP_ENABLED);
	Digital::setInput(IO_PD6,PULL_UP_ENABLED);
	Analog::init();
	for (i=0;i<10;i++)
	{
		readBattery();
		_delay_ms(2);
	}
	isRun=false;
	Digital::setOutput(IO_PD7,LOW);
	Digital::setOutput(IO_PB0,HIGH);
	if(volt_battery <= 3.9){ //Si la bateria esta descargada enciende los LEDS
	Digital::setOutput(IO_PD7,HIGH);
	Digital::setOutput(IO_PB0,LOW);
	_delay_ms(2500);
	}
	sleepNatsuky(); //Duerme al microcontrolador y espera que este despierte por medio de una interrupcion externa
	// Y Espera una secuencia correcta del microcontrolador
	Digital::setOutput(IO_PD7,LOW);
	Digital::setOutput(IO_PB0,HIGH);
	cli();
	Digital::setInput(IO_PB4,PULL_UP_ENABLED);
	if(!enableBT){
	PCMSK0 |= (1<<PCINT4); //Use PCINT4 = PB4 pin 28.
	PCICR |= (1<<PCIE0);   //Activa las interrupcions por cambio de estado para frenar al robot con el pulsador	
	_delay_ms(TIME_START);
	}else{
	printPosition = false;
	PCMSK0 &= ~(1<<PCINT4);
	PCICR &= ~(1<<PCIE0);  //Desactiva las interrupcions por cambio de estado	
	}
	TCCR3A = 0;
	TCCR3B |= 1;
	TIMSK3 |= (1<<TOIE3);
	TCNT3 = 65416;
	ticksEncoderR = ticksEncoderL = 0;
	millis=0;
	sei();
	sensors.emittersEn(EMITTERS_ON);
	max = VEL_INITIAL;
	isRun = true;
	for(;;) //Inicio del bucle infinito
	{
		if(isRun && isCalibrated && option != 'A'){
			if(millis >= 300 && millis <= 310){
				max = VEL_MAX;
			}
			position = sensors.measureLine(sensorValues);
			pid_calc(setPoint);
		}else if (!isRun && option != 'A')
		{
			if(!enableBT) {
				motors.stopMotors(); //Apaga los motores
			}else{
			while(max > 0){
				sensors.emittersEn(EMITTERS_ON);
				static unsigned long delta;
				if ( (millis) - delta >= DELTA_STOP)
				{
					max-=10;
					delta=millis;
				}
				position = sensors.measureLine(sensorValues);
				pid_calc(setPoint);
			}
			TIMSK3 &= ~(1<<TOIE3);
			//max=0;
			sensors.emittersEn(EMITTERS_OFF);	//Apaga los sensores
		    motors.stopMotors(); // apaga los motores
			}
		}
		 if(enableBT==true && printPosition==true && isRun==false){
			 sensors.emittersEn(EMITTERS_ON);	
			 position = sensors.measureLine(sensorValues);
			 uart_print(bitRead(PINF,0));
			 uart_print("	");
			 uart_print(bitRead(PINF,1));
			 uart_print("	");
			for (i=0;i<NUM_SENSORS;i++){
				uart_print(sensorValues[i]);
				uart_print("	");
			}
			uart_print(bitRead(PINC,7));
			uart_print("	");
			uart_print(bitRead(PINC,6));
			uart_println(position);
			_delay_ms(500);
		}

		
	}
}
//################################## ESPACIO PARA FUNCIONES ########################################
void wakeNatsuky()
{
	power_adc_enable();
	sensors.init(pinsSensors,NUM_SENSORS,NUM_SAMPLES,EMITTER_PIN); //Configura los sensores
	motors.init(pinsMotors,PRESCALER_1,FAST_PWM); //Configura los motores con un PWM de 46 Khz
	sensors.emittersEn(EMITTERS_OFF);//Apaga los sensores mientras realiza las respectivas configuraciones
	Digital::setOutput(IO_PD7,LOW);
	Digital::setOutput(IO_PD5,HIGH);
	static int Temp;
	switch(option){
		case 'A'://Activa la UART para recibir informacion mediante el bluetooth
		Digital::setOutput(IO_PD7,HIGH);
		Digital::setOutput(IO_PD5,LOW);
		initUART(38400); //Inicia comuniacion serial a 38400 bits por segundo para el bluetooth
		uart_println("#############################");
		uart_println("Line Follower Natsuky"); //Mensaje de bienvenida
		uart_print("\nUltima fecha de compilacion: ");
		uart_println(__DATE__);
		uart_print("Hora:");
		uart_println(__TIME__);
		uart_println("\nPowered by: Michael Vargas\n");
		uart_print("Temperatura del Nucleo->");
		for (i=0;i<100;i++)
		{
		Temp = GetTemp();
		_delay_ms(10);
		}
		uart_print(Temp);
		uart_println(" C");
		if(Temp >= 38) uart_println("*Temperatura Alta!*");
		//Analog::init(); //Estable la configuracion por defecto
		Analog::init();
		readBattery();
		//Envia la información del voltaje de la bateria
		uart_print("Voltaje de bateria->");
		uart_print(volt_battery,3);
		uart_println(" V");
		uart_print("Frequencia motores->"); //Frecuencia de PWM de los motores
		uart_print(motors.getFrequency());
		uart_println(" Khz");
		uart_print("Vel. maxima->");
		uart_print((VEL_MAX*100)/255);
		uart_println("%");
		cli();
		TIMSK1 = (1<<TOIE1); //Activa la interrupcion por desbordamiento del Timer1 para leer los datos recibidos en la UART
		sei();
		enableBT = true;
		break;
		case 'B': restoreLastCalibration(); //Restaura los ultimos valores de calibracion y enseguida inicia la linea
				break;
		case 'C':
				Digital::setOutput(IO_PB0,HIGH); //Calibra los sensores
				Digital::setOutput(IO_PD7,HIGH);
				while (bitRead(PINB,4))
				{
				if(!bitRead(PIND,6) )	calibrationSensors(); //Calibra los sensores
				}
				break; 
	}

}
ISR(TIMER1_OVF_vect){
	serialEvent();
	if (stringComplete==true && enableBT==true) {
		static unsigned long ticksR,ticksL;
		uart_println("OK!");
		switch(inputString[0]){
			case '1':	calibrationSensors();	break; //Calibra sensores
			case '2':		Analog::init(); for(i=0;i<5;i++){readBattery(); _delay_ms(10);}
			 uart_print("Vbat:"); uart_print(volt_battery,3); uart_println(" V"); break; //Lee el voltaje de la bateria
			case '3':	uart_print("Vel. inicial:"); uart_println(VEL_INITIAL); //Inicia el recorrido
			uart_print("Vel. max:"); uart_println(VEL_MAX);
			uart_print("Vel. rectas:"); uart_println(VEL_RECTAS);
			uart_println("\nReady...!"); isRun=true; option = 0; sensors.emittersEn(EMITTERS_ON);
			enableBT = true;
			_delay_ms(TIME_START);
			break;
			case '4':	for (i=0;i<7;i++)
			{
				uart_print(GetTemp());
				uart_println(" C");
				_delay_ms(500);
			}
			break;
			case 'A':   
			TIMSK3 &= ~(1<<TOIE3);  //Desactiva las interrupciones
			ticksL = ticksEncoderL; //Obtiene los ticks
			ticksR = ticksEncoderR;
			TIMSK3 |= (1<<TOIE3);
			uart_print("*Ticks Right:");
			uart_print(ticksR);
			uart_print(" *Ticks Left:");
			uart_print(ticksL);
			uart_println(" ");
			break;
			case 'B':
				if(isRun){
					uart_println("*Frenando el robot...");
				}
				else {uart_println("*Activando el robot...");
					max = VEL_INITIAL;
					millis=0;
					TIMSK3 |= (1<<TOIE3);
					sensors.emittersEn(EMITTERS_ON);
					_delay_ms(100);
				}
					isRun = !isRun; //Parar el robot
			break;
			case 'C':
					if (!printPosition) uart_println("*Ativando printPosition...");
						else uart_println("*Desactivando printPosition...");
					printPosition = !printPosition;
			break;
			
		}
		//Limipiar entrada
		memset(inputString,0,CANTIDAD_ELEMENTOS(inputString));
		stringComplete = false;
	}
}
ISR(PCINT0_vect) //Despierta al microcontrolador del bajo consumo
{
	if (!isRun)
	{
		if(!bitRead(PINB,4) && isCalibrated){
			sensors.emittersEn(EMITTERS_ON);
			_delay_ms(200);
			isRun = true;
		}else if(!bitRead(PINB,4) && !bitRead(PIND,6)){
		option = 'A';	//Activa modulo Bluetooth HC-05 para que cuando no este, no genere errores y micro se reinicie
		enableBT = true;
		power_usart0_enable();
 		power_usart1_enable();
		PCMSK0 &= ~(1<<PCINT4);
		PCICR &= ~(1<<PCIE0);  //Desactiva las interrupcions por cambio de estado
		isCalibrated = false;
		}else if(!bitRead(PINB,4) && bitRead(PIND,6)){
		Digital::setOutput(IO_PB0,LOW);
		//isCalibrated = true;
		option='B';
		sleep_disable();
		PCMSK0 &= ~(1<<PCINT4);
		PCICR &= ~(1<<PCIE0);   //Desactiva las interrupcions por cambio de estado
		_delay_ms(1000);
	}
	if(bitRead(PINB,4) && !bitRead(PIND,6)){
		isCalibrated = false;
		option='C';
	}
	}else if (!bitRead(PINB,4))
	{
		isRun = false;
		sensors.emittersEn(EMITTERS_OFF);
		motors.stopBrakeMotors();
		_delay_ms(2500);
		motors.stopMotors();
// 		PCMSK0 &= ~(1<<PCINT4);
// 		PCICR &= ~(1<<PCIE0);   //Desactiva las interrupcions por cambio de estado
	}
}
int GetTemp(void)
{
 //ADC Multiplexer Selection Register
 ADMUX = 0;
 ADMUX |= (1 << REFS1);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin
 ADMUX |= (1 << REFS0);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin
 ADMUX &= ~(1 << MUX4);  //Temperature Sensor - 100111
 ADMUX &= ~(1 << MUX3);  //Temperature Sensor - 100111
 ADMUX |= (1 << MUX2);  //Temperature Sensor - 100111
 ADMUX |= (1 << MUX1);  //Temperature Sensor - 100111
 ADMUX |= (1 << MUX0);  //Temperature Sensor - 100111

 //ADC Control and Status Register A
 ADCSRA = 0;
 ADCSRA |= (1 << ADEN);  //Enable the ADC
 ADCSRA |= ((1 << ADPS1) | (1<<ADPS0));  //ADC Prescaler - 8 (12MHz -> 1.5MHz)
 //ADC Control and Status Register B
 ADCSRB = 0;
 ADCSRB |= (1 << MUX5);  //Temperature Sensor - 100111
 ADCSRA |= (1 << ADSC);  //Start temperature conversion
 while (bitRead(ADCSRA, ADSC));  //Wait for conversion to finish
 uint8_t low  = ADCL;
 uint8_t high = ADCH;
 int temperature = (high << 8) | low;  //Result is in kelvin
 temperature -= (273+OFFSET_TEMP_SENSORS);
 return temperature;
}
void readBattery()
{
	//Calcular voltaje de bateria
//	DDRD |= (1<<4);
//	Digital::setOutput(IO_PD4,LOW);
//	_delay_us(100);
	DDRD &= ~(1<<4);
	DIDR2 |= (1<<ADC8D); //Digital input disable, desactiva el pin digital D4 para poder usarlo como analogo
	volt=0;
	volt_battery = 0;
	unsigned char j;
	_delay_ms(100);
	for (j=0;j<NUM_SAMPLES;j++) 
	{
		volt += Analog::read(8); //Toma 4 muestras del ADC
		_delay_ms(10);
	}
	volt/=NUM_SAMPLES;
	volt_battery = ((volt*3.3)/1023.0)/0.2805755396f;
}
void serialEvent() {
	static unsigned char i;
	while (availableUART()) {
		char inChar = (char)readUART();
		inputString[i] = inChar;
		i++;
		if (inChar == '\n') {
			stringComplete = true;
			i=0;
		}
	}
}
inline void sleepNatsuky(){
	/*
	NOTA:
	* El robot consume 68 mA en modo Sleep
	* Los sensores consumen 92 - 100 mA, por defecto estan apagados para ahorar energia
	*/
	PCMSK0 |= (1<<PCINT4); //Use PCINT4 = PB4 pin 28.
	PCICR |= (1<<PCIE0);   //Activa las interrupcions por cambio de estado
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);//Establecemos el modo de bajo consumo.
	power_adc_disable();
	power_twi_disable();
	power_usb_disable();
	power_usart0_disable();
	power_usart1_disable();
	cli();
	sleep_enable();
	sei();
	sleep_cpu();
	wakeNatsuky(); //Despierta al micro
	sleep_disable();
	DDRD &= ~ (1<<5); //Establece los encoders como entradas
	DDRB &= ~(1<<2);
	DDRE |= (1<<6);
}
void pid_calc(int setPoint)
{
	static const float Kp = 0.0618435;
	static const float Kd = 2.2188435;
	static int derivative = 0,proportional = 0,last_proportional = 0,power_difference =0;
	static int vel_reverse = 0;

	proportional = ((int)position) - setPoint;
	derivative = proportional - last_proportional;
	power_difference = ( proportional * Kp ) + ( derivative * Kd );
	if ( power_difference > max ) power_difference = max;     // asignar la máxima permitida
	else if ( power_difference < -max ) power_difference = -max;
	// Asignar velocidad calculada en el poder diferencial de los motores
	(power_difference < 0 ) ? motors.setSpeeds(max+power_difference, max)  : motors.setSpeeds(max, max-power_difference);
	vel_reverse = 0;
	if (proportional <= (-2200) ){
		vel_reverse=(proportional <= -RANGEBRAKE ) ? proportional >> 6 : proportional>>7;
		motors.speedLeft(vel_reverse);
		} else if ( proportional >= (2200)){
		vel_reverse = (proportional >= RANGEBRAKE ) ? -(proportional >> 6) : -(proportional>>7);
		motors.speedRight(vel_reverse);
	}
	
	last_proportional = proportional; 	//Recordando la última posición
}
ISR(TIMER3_OVF_vect){
	
	TCNT3 = 65416; //vuelve a cargar el valor del timer para que se desborde cada 10us
	
	static unsigned char micros10;
	static bool Flag1,Flag2,recta;
	static unsigned char ticksAuxR,ticksAuxL;
	
	micros10++;
	if(micros10==100){
		micros10=0;
		millis++;
	}
/* // Sirve para probar el timer
	if (millis>=1000)
	{
		PORTD ^= (1<<7);
		PORTB ^= (1<<0);
		millis=0;
	}*/
	if (isRun)
	{
	if(bitRead(PIND,5)==true && *pinsMotors[2] == 0){
			if(!Flag2){
				ticksEncoderR++;
				ticksAuxR++;
				Flag2 = true;
			}
			}else{
			Flag2=false;
	}
	if(bitRead(PINB,2)==true && *pinsMotors[0] == 0){
			if(!Flag1){
				ticksEncoderL++;
				ticksAuxL++;
				Flag1 = true;
			}
			}else{
			Flag1=false;
	}
 //Determinar trazado.
 if(ticksAuxL > 25 || (ticksAuxL==0 && ticksAuxR > 25))
 {	static unsigned char trazado;
	 trazado = ticksAuxL - ticksAuxR;
	 
	 if(trazado > 7) //Giro derecha
	 {
		 recta = 0;
	 }
	 else if(trazado < -7) //Giro izquierda
	 {
		 recta = 0;
	 }
	 else //Recta
	 {
		 recta = 1;
	 }
	 ticksAuxR = ticksAuxL = 0;
 }
 if(recta){
	 PORTD |= (1<<7);
	PORTB |= (1<<0);
 }else{
	 PORTB &= ~(1<<0); //El robot esta en curva
	 PORTD &= ~(1<<7); 
 }
/*
	if (ticksAuxL > 10 || ticksAuxR > 10)
	{
		if(abs(ticksAuxL - ticksAuxR) <=2 ){ 	//El robot esta en recta
			PORTD |= (1<<7);
			PORTB |= (1<<0);
			}else{
			PORTB &= ~(1<<0); //El robot esta en curva
			PORTD &= ~(1<<7);
		}
		ticksAuxR = ticksAuxL = 0;
	}*/
	
	}
}
void restoreLastCalibration(){
	if(!isCalibrated){
		Digital::setOutput(IO_PD7,HIGH);
		Digital::setOutput(IO_PB0,LOW);
		sensors.restoreCalibration(); //Restaura la calibracion
		if(enableBT) printCalibration();
		isCalibrated=true;
	}
}
void calibrationSensors(){
	Digital::setOutput(IO_PD7,HIGH);
	Digital::setOutput(IO_PB0,HIGH);
	sensors.emittersEn(EMITTERS_ON);
	if(enableBT) uart_println("Calibrando sensores...");
	for (i=0;i<80;i++)
	{
		Digital::setOutput(IO_PD7,TOGGLE);
		sensors.calibrate();
		_delay_ms(15);
	}
	Digital::setOutput(IO_PD7,LOW);
	Digital::setOutput(IO_PB0,HIGH);
	sensors.emittersEn(EMITTERS_OFF);
	isCalibrated=true;
	if(enableBT) printCalibration();
	sensors.saveCalibration(); //Guarda la calibración en la EEPROM
}
void printCalibration()
{
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