/*
 * UART32U4.H
 * V1.0
 * Created: 31/01/2016 11:30:01 p. m.
 * Author : Michael Vargas
 */ 
#ifndef UART32u4_H
#define UART32u4_H



#if defined(__AVR_ATmega32U4__) || defined (__AVR_ATmega16u4__)

#pragma GCC diagnostic ignored "-Wwrite-strings" 

#ifndef F_CPU
#error "Error falta definir la frecuencia de operacion F_CPU"
#endif

#include <avr/io.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define BUFFER_SIZE 200
#define HEX 16
#define BIN 2
#define DEC 10
#define UBBRValue(x) (lrint((( F_CPU / ( x * 16UL))) - 1)) 
static char string[BUFFER_SIZE];

void initUART(uint32_t BAUD)
{
	
	UBRR1H = (unsigned char) (UBBRValue(BAUD) >> 8 );
	UBRR1L = (unsigned char) UBBRValue(BAUD);
	// Activar pines d RX y TX
	UCSR1B = ( 1 << RXEN1 ) | ( 1 << TXEN1 );	
    // 	Asignar formato ( 8data, 2stop )
	UCSR1C = ( 1 << USBS1 ) | ( 3 << UCSZ10 );
}
//Enviar caracteres
void writeUART(unsigned char data )
{
	while ( ! ( UCSR1A & ( 1 << UDRE1 ) ) );	
	// Envia los datos
	UDR1 = data;
}

// Lee los caracteres desde la UART
char readUART(void)
{
	while (!( UCSR1A & ( 1 << RXC1) ));
	return UDR1;
}
//Verifica si hay datos disponibles para leer
bool availableUART(void)
{
	if ( UCSR1A & ( 1 << RXC1) ) return true;
	else return false;
}

// Enviar un String sin salto de linea
void uart_print( char* data )
{
	while (*data)
	writeUART(*(data++));
}
void uart_print(unsigned long num,unsigned char radix=0)
{
	if(radix!=0) ultoa(num,string,radix);
	else if(radix==0 || radix==DEC)ultoa(num,string,10);
	
	PORTB ^= (1<<PORTB0);
	if(radix==HEX){
		uart_print("0x");
		}else if(radix==BIN){
		uart_print("0b");
	}
	uart_print(string);
	PORTB |= (1<<PORTB0);
	
}
//Enviar String con salto de linea
void uart_println(char* data)
{
	PORTB ^= (1<<PORTB0);
	uart_print(data);
	PORTB |= (1<<PORTB0);
	writeUART('\n');
}

void uart_print(double data_f,uint8_t decimales=0){
switch(decimales){
	case 3:
	sprintf(string,"%.3f",data_f);
	break;
	case 4:
	sprintf(string,"%.4f",data_f);
	break;
	case 5:
	sprintf(string,"%.5f",data_f);
	break;
	default:
	sprintf(string,"%.1f",data_f);
	break;
}
uart_print(string);
}
void uart_println(double data_f,uint8_t decimales=0){
	uart_print(data_f,decimales);
	writeUART('\n');
}
void uart_print(unsigned int num,unsigned char radix=0){
	if(radix!=0) itoa(num,string,radix);
	else if(radix==0 || radix==DEC)itoa(num,string,10);
	PORTB ^= (1<<PORTB0);
	if(radix==HEX){
		uart_print("0x");
		}else if(radix==BIN){
		uart_print("0b");
	}
	uart_print(string);
	PORTB |= (1<<PORTB0);
}
void uart_print(int num,unsigned char radix=0){
	if(radix!=0) itoa(num,string,radix);
	else if(radix==0 || radix==DEC)itoa(num,string,10);
	PORTB ^= (1<<PORTB0);
	if(radix==HEX){
		uart_print("0x");
		}else if(radix==BIN){
		uart_print("0b");
	}
	uart_print(string);
	PORTB |= (1<<PORTB0);
}
void uart_println(unsigned int num,int radix=0){
	uart_print(num,radix);
	writeUART('\n');
}
void uart_println(unsigned int &num,int radix=0){
	uart_print(num,radix);
	writeUART('\n');
}
void uart_println(int num,int radix=0){
	uart_print(num,radix);
	writeUART('\n');
}
#else
#error "Esta libreria solo es compatible con ATmega32u4"
#endif
#endif