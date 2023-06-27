/*xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
 * Programa    : EjmEco
 * Descripción : Programa que está a la espera de recibir un dato a través de un
 *             : enlace serial de parámetros 9600, 8, e, 2. Cada dato recibido
 *             : es retornado a su fuente.
 * Autor       : Luis Raymi
 * Hardware    : PA0: Rx
 *             : PA1: Tx
 * Fecha       : 23/05/2023
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx*/
// DECLARACION DE ARCHIVOS DE CABECERA
#include <stdint.h>
#include "tm4c123gh6pm.h"
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// DECLARACION DE PROTOTIPOS
void ConfigUART0( void );
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// DECLARACION DE CONSTANTES

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// DECLARACION VARIABLES GLOBALES

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// PROGRAMA PRINCIPAL
void main( void )
{
    //Declaración de variables
    uint8_t DatoRx;

    //Configuración de interfaz de E/S y periféricos
    ConfigUART0();

    //Establecer condición inicial

    //Lazo principal
    while( 1 )
    {
        while( UART0_FR_R & UART_FR_RXFE ) ;    // Espera mientras Buffer Rx esté vacío

        DatoRx = UART0_DR_R & 0xFF;             // Se lee el dato recibido

        while(UART0_FR_R & UART_FR_TXFF);       // Espera mientras Buffer Tx esté lleno

        UART0_DR_R = DatoRx;                    // Se escribe el dato para su transmisión
    }
}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// Función     : ConfigUART0
// Descripción : configura UART0 a 9600 bits por segundo, con una trama de 8 bits
//             : de dato, paridad par, 2 bits de parada y con FIFO de 16 bytes
// Entrada     : bps: velocidad en bits por segundo
// Salida      : ninguna
//------------------------------------------------------------------------------
void ConfigUART0( void )
{
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;               // habilitamos reloj para el UART0
    while( !(SYSCTL_PRUART_R & SYSCTL_PRUART_R0) ) ;    // espera que se active

    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;               // habilitamos reloj para GPIOA
    while( !(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0) );     // espera que se active

    UART0_CTL_R  &= ~0b000000000001;        // UARTEN=0, deshabilitamos el UART0

    UART0_IBRD_R &= ~0x0000FFFF;            // borra los 16 bits del UARTIBRD
    UART0_IBRD_R |=  104;                   // asigna valor calculado: 1000000/9600
    UART0_FBRD_R &= ~0x0000003F;            // borra los 6 bits del UARTFBRD
    UART0_FBRD_R |=  11;                    // asigna valor calculado: (1000000-104*9600)*64/9600 + 0.5

    UART0_LCRH_R &= ~0b11111111;            // borra valor actual de configuración
    UART0_LCRH_R |=  0b01111110;            // SPS=0,WLEN=11,FEN=1,STP2=1,EPS=1,PEN=1,BRK=0

    UART0_CTL_R  &= ~0b001110100001;        // borra valor actual
    UART0_CTL_R  |=  0b001100000001;        // RXE=1,TXE=1,LBE=0,HSE=0,UARTEN=1

    GPIO_PORTA_AMSEL_R &= ~0b00000011;      // desactiva modo analógico en PA0 y PA1

    GPIO_PORTA_AFSEL_R |=  0b00000011;      // activa funciones alternas en PA0 y PA1
    GPIO_PORTA_PCTL_R  &= ~0x000000FF;      // borra valor anterior
    GPIO_PORTA_PCTL_R  |=  0x00000011;      // asigna nuevo valor: UART0 a PA0 y PA1

    GPIO_PORTA_DEN_R   |=  0b00000011;      // activamos funciones digitales en PA0 y PA1
}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
