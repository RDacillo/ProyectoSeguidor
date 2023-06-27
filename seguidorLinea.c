#include <stdint.h>
#include "tm4c123gh6pm.h"


/*  PB6     PWMA    Timer0  A   (motor izquierdo)
    PB0     AIN2
    PD1     AIN1
    PD3     STBY
    PB4     BIN1
    PB1     BIN2
    PB7     PWMB    Timer0  B   (motor derecho)
*/

#define AIN2 0b00000001
#define AIN1 0b00000010
#define STBY 0b00001000
#define BIN1 0b00010000
#define BIN2 0b00000010

void movimientoBluetooth(uint8_t Terminal);
void configuraPWM(void);
void Config_Pines_Driver(void);
void configUART0(void);
uint8_t MostrarValorSensor(void);
void configPuertoA(void);
void configUART2(void);



void TxCar( uint8_t car );
uint8_t car;


void main(void){
    uint8_t Terminal;


    configuraPWM();
    configUART0();
    configUART2();
    configPuertoA();

    while(1){

        /*while( UART0_FR_R & UART_FR_RXFE ) ;    // Espera mientras Buffer Rx este vacio
        Terminal = UART0_DR_R & 0xFF;             // Se lee el dato recibido

        while(UART0_FR_R & UART_FR_TXFF);       // Espera mientras Buffer Tx este lleno
        UART0_DR_R = Terminal;                    // Se escribe el dato para su transmision (ECO)
*/

        while( UART2_FR_R & UART_FR_RXFE ) ;    // Espera mientras Buffer Rx este vacio
        Terminal = UART2_DR_R & 0xFF;             // Se lee el dato recibido
        /*
        while(UART2_FR_R & UART_FR_TXFF);       // Espera mientras Buffer Tx este lleno
        UART2_DR_R = Terminal;                    // Se escribe el dato para su transmision (ECO)
        */
        while(UART0_FR_R & UART_FR_TXFF);       // Espera mientras Buffer Tx este lleno
        UART0_DR_R = Terminal;                    // Se escribe el dato para su transmision (ECO)

        movimientoBluetooth(Terminal);

    }

}

void movimientoBluetooth(uint8_t Terminal){
    uint16_t compL;
    uint8_t compH;
    if(Terminal=='W'){  //avanza    AIN1 0, AIN2 1, STBY 1,
                                  //BIN1 0, BIN2 1
        GPIO_PORTD_DATA_R |= STBY;
        GPIO_PORTD_DATA_R &= ~AIN1;
        GPIO_PORTB_DATA_R |= AIN2|BIN2;
        GPIO_PORTB_DATA_R &= ~BIN1;

       //duty cycle a 75% en timer A y B, comparador= 99999=1869F

        compL = 0x1869F & 0x0FFFF;
        compH = (0x1869F & 0xFF000)>>16;
        TIMER0_TBMATCHR_R = (TIMER0_TBMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TBPMR_R = (TIMER0_TBPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos

        TIMER0_TAMATCHR_R = (TIMER0_TAMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TAPMR_R = (TIMER0_TAPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos

    }else if (Terminal =='D'){      //giro hacia la derecha

        // AIN1 = 0, AIN2 = 1 (Gira antihorario)
       // BIN1 = 0, BIN2 = 1 (gira antihorario)

        GPIO_PORTD_DATA_R |= STBY;
        GPIO_PORTD_DATA_R &= ~AIN1;
        GPIO_PORTB_DATA_R |= AIN2|BIN2;
        GPIO_PORTB_DATA_R &= ~BIN1;

        //duty cycle a 75% en timer B y 25% en A

        /*TIMER B:
         * T_alta=0.75*T_periodo=0.75*(1/120)
         * comparador=T_alta*16*10^6 -1
         * comparador=99999=1869F
         * */

        compL = 0x1869F & 0x0FFFF;
        compH = (0x1869F & 0xFF000)>>16;
        TIMER0_TBMATCHR_R = (TIMER0_TBMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TBPMR_R = (TIMER0_TBPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos

        /*TIMER A:
         * T_alta=0.25*T_periodo=0.25*(1/120)
         * comparador=T_alta*16*10^6 -1
         * comparador=33332=8234
         * */
        compL = 0x8234 & 0x0FFFF;
        compH = (0x8234 & 0xFF000)>>16;
        TIMER0_TAMATCHR_R = (TIMER0_TAMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TAPMR_R = (TIMER0_TAPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos

    }else if (Terminal =='S'){      //retrocede

        // AIN1 = 1, AIN2 = 0 (Gira horario)
         // BIN1 = 1, BIN2 = 0 (Gira horario)

        GPIO_PORTD_DATA_R |= STBY;
        GPIO_PORTD_DATA_R |= AIN1;
        GPIO_PORTB_DATA_R &= ~(AIN2|BIN2);
        GPIO_PORTB_DATA_R |= BIN1;

        //duty cycle a 75% en timer A y B, comparador= 1869F
        uint16_t compL;
        uint8_t compH;
        compL = 0x1869F & 0x0FFFF;
        compH = (0x1869F & 0xFF000)>>16;
        TIMER0_TBMATCHR_R = (TIMER0_TBMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TBPMR_R = (TIMER0_TBPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos

        TIMER0_TAMATCHR_R = (TIMER0_TAMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TAPMR_R = (TIMER0_TAPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos


    }else if (Terminal =='A'){      //giro hacia la izquierda

        // AIN1 = 0, AIN2 = 1 (Gira antihorario)
        // BIN1 = 0, BIN2 = 1 (gira antihorario)

        GPIO_PORTD_DATA_R |= STBY;
        GPIO_PORTD_DATA_R &= ~AIN1;
        GPIO_PORTB_DATA_R |= AIN2|BIN2;
        GPIO_PORTB_DATA_R &= ~BIN1;

        //duty cycle a 75% en timer A y 25% B

        /*TIMER B:
         * T_alta=0.25*T_periodo=0.25*(1/120)
         * comparador=T_alta*16*10^6 -1
         * comparador=33332=8234
         * */

        compL = 0x8234 & 0x0FFFF;
        compH = (0x8234 & 0xFF000)>>16;
        TIMER0_TBMATCHR_R = (TIMER0_TBMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TBPMR_R = (TIMER0_TBPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos

        /*TIMER A:
         * T_alta=0.75*T_periodo=0.75*(1/120)
         * comparador=T_alta*16*10^6 -1
         * comparador=99999=1869F
         * */
        compL = 0x1869F & 0x0FFFF;
        compH = (0x1869F & 0xFF000)>>16;
        TIMER0_TAMATCHR_R = (TIMER0_TAMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TAPMR_R = (TIMER0_TAPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos


    }else if(Terminal=='P'){

        //STBY=1, AIN1=0, AIN2=0, BIN1=0, BIN2=0
        GPIO_PORTD_DATA_R |= STBY;
        GPIO_PORTD_DATA_R &= ~AIN1;
        GPIO_PORTB_DATA_R &= ~AIN2;
        GPIO_PORTB_DATA_R &= ~BIN2;
        GPIO_PORTB_DATA_R &= ~BIN1;

        //duty cycle a 100% en timer A y B

        //TIMER B:
        // T_alta=1*T_periodo=1*(1/120)
        //  comparador=T_alta*16*10^6 -1
        //  comparador=133 332=208D4


        compL = 0x208D4 & 0x0FFFF;
        compH = (0x208D4 & 0xFF000)>>16;
        TIMER0_TBMATCHR_R = (TIMER0_TBMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TBPMR_R = (TIMER0_TBPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos

        //TIMER A:
         // T_alta=1*T_periodo=1*(1/120)
         // comparador=T_alta*16*10^6 -1
         // comparador=133 332=208D4

        compL = 0x208D4 & 0x0FFFF;
        compH = (0x208D4 & 0xFF000)>>16;
        TIMER0_TAMATCHR_R = (TIMER0_TAMATCHR_R & 0xFFFF0000) | compL;  //16 bits menos significativos
        TIMER0_TAPMR_R = (TIMER0_TAPMR_R & 0xFFFFFF00) | compH;      //8 bits mas significativos


    }else if (Terminal=='T'){

        MostrarValorSensor();
        while(UART0_FR_R & UART_FR_TXFF);       // Espera mientras Buffer Tx esté lleno
        UART0_DR_R = '\n';                    // Se escribe salto de l�nea
        UART0_DR_R = '\r';                    // Se escribe cursor al principio
    }else{
    }
}


void configuraPWM(void){

    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0; //habilitar reloj del timer 0
    while( (SYSCTL_PRTIMER_R & SYSCTL_PRTIMER_R0) == 0 ) {} //espera a que la senal de reloj se active

    TIMER0_CTL_R &= ~(TIMER_CTL_TBEN | TIMER_CTL_TAEN); //deshabilita el timer  B y A
    TIMER0_CFG_R &= ~(0x7); //modo de 2 contadores de 16 bits
    TIMER0_CFG_R |= 0x04;

    TIMER0_TBMR_R &= ~(0xFFF);    // contador B modo PWM
    TIMER0_TBMR_R |= 0x50A;

    TIMER0_TAMR_R = (TIMER0_TAMR_R &~0xFFF) + 0x50A; // contador A modo PWM

    TIMER0_CTL_R |= TIMER_CTL_TBPWML; //PWM B no invertido
    TIMER0_CTL_R |= TIMER_CTL_TAPWML; //PWM A no invertido

    //frecuencia es de 120Hz que son (1/120)/(1/16*10^6)=133 333.333 ciclos de reloj
    //133 333-1 = 208D5
    TIMER0_TBILR_R = 0x08D5; //16 bits menos significativos
    TIMER0_TBPR_R = 0x02; //8 bits mas significativos

    TIMER0_TAILR_R = 0x08D5; //16 bits menos significativos
    TIMER0_TAPR_R = 0x02; //8 bits mas significativos



    //el duty cycle lo inicializamos como 25%
    //0.25=T_in/T entonces T_in=0.25(1/120)= 2.08333*10^-3 segundos
    //Tiempo de configuracion: (T_in*16*10^6) -1 = 33332.333 = 33332 = 0x8234
    TIMER0_TBMATCHR_R = (TIMER0_TBMATCHR_R & 0xFFFF0000) | 0x8234;  //16 bits menos significativos
    TIMER0_TBPMR_R = (TIMER0_TBPMR_R & 0xFFFFFF00) | 0x00;      //8 bits mas significativos

    TIMER0_TAMATCHR_R = (TIMER0_TAMATCHR_R & 0xFFFF0000) | 0x8234;  //16 bits menos significativos
    TIMER0_TAPMR_R = (TIMER0_TAPMR_R & 0xFFFFFF00) | 0x00;      //8 bits mas significativos

    Config_Pines_Driver();

    TIMER0_CTL_R |= (TIMER_CTL_TAEN| TIMER_CTL_TBEN); //habilitamos nuevamente el timer0


}


void Config_Pines_Driver(void){

    //Pines asociados al puerto B:
    //PB0 : AIN2
    //PB1 : BIN2
    //PB4 : BIN1
    // Activamos la señal de reloj del puerto B
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB;
    // Esperamos a que realmente se active
    while( (SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1)==0) { }


    GPIO_PORTB_AMSEL_R &= ~0b00010011;  // Deshabilita el modo analógico
    GPIO_PORTB_AFSEL_R &= ~(0b00010011);  // Deshabilita función alternativa
    GPIO_PORTB_PCTL_R  &= ~(0x0000F00FF);  //Deshabilitamos las funciones alternas
    GPIO_PORTB_DIR_R |= 0b00010011;    // PB0,PB1,PB4 pin de salida
    GPIO_PORTB_DEN_R |= 0b00010011;       // Activar salida digital
    GPIO_PORTB_DATA_R  &= ~0b00010011;  //Inicializamos en un valor de acuerdo a cómo los motores deben estar en espera


    GPIO_PORTB_AMSEL_R &= ~0b11000000;  // Deshabilita el modo analógico
    GPIO_PORTB_AFSEL_R |= 0b11000000;  // Deshabilita función alternativa
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0x00FFFFFF) | 0x77000000;  //Deshabilitamos las funciones alternas
    GPIO_PORTB_DIR_R |= 0b11000000;    // PB6 y PB7 pin de salida
    GPIO_PORTB_DEN_R |= 0b11000000;       // Activar salida digital

    //Pines asociados al puerto D:
    //PD1 : AIN1
    //PD3 : STBY

    // Activamos la señal de reloj del puerto D
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;
    // Esperamos a que realmente se active
    while( (SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3)==0) { }

        GPIO_PORTD_AMSEL_R &= ~0b00001010;  //Deshabilitamos el modo analógico
        GPIO_PORTD_AFSEL_R &= ~0b00001010;  //Deshabilitamos las funciones alternas
        GPIO_PORTD_PCTL_R  &= ~(0x0000F0F0);  //Como GPIO
        GPIO_PORTD_DIR_R   |=  0b00001010;  // PD1,PD3 pin de salida
        GPIO_PORTD_DEN_R   |=  0b00001010;  //Activamos la salida digital
        GPIO_PORTD_DATA_R  &= ~0b00001010;  //Inicializamos en un valor de acuerdo a cómo los motores deben  estar en espera.
}


void configUART0(void){

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
        UART0_LCRH_R |=  0b01110000;            // SPS=0,WLEN=11,FEN=1,STP2=1,EPS=1,PEN=1,BRK=0

        UART0_CTL_R  &= ~0b001110100001;        // borra valor actual
        UART0_CTL_R  |=  0b001100000001;        // RXE=1,TXE=1,LBE=0,HSE=0,UARTEN=1

        GPIO_PORTA_AMSEL_R &= ~0b00000011;      // desactiva modo analógico en PA0 y PA1

        GPIO_PORTA_AFSEL_R |=  0b00000011;      // activa funciones alternas en PA0 y PA1
        GPIO_PORTA_PCTL_R  &= ~0x000000FF;      // borra valor anterior
        GPIO_PORTA_PCTL_R  |=  0x00000011;      // asigna nuevo valor: UART0 a PA0 y PA1

        GPIO_PORTA_DEN_R   |=  0b00000011;      // activamos funciones digitales en PA0 y PA1
}

void configUART2(void){ //9600, 8, N, 1

    //utilizaremos el UART2 del PD6, PD7
    SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART2;                   // habilitamos reloj para el UART2
    while( !(SYSCTL_PRUART_R & SYSCTL_PRUART_R2) ) ;        // espera que se active

    //SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;                 // habilitamos reloj para GPIOD
   // while( !(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) );         // espera que se active

        UART2_CTL_R  &= ~0b000000000001;            // UARTEN=0, deshabilitamos el UART2

        UART2_IBRD_R &= ~0x0000FFFF;                // borra los 16 bits del UARTIBRD
        UART2_IBRD_R |=  104;                       // asigna valor calculado: 1000000/9600
        UART2_FBRD_R &= ~0x0000003F;                // borra los 6 bits del UARTFBRD
        UART2_FBRD_R |=  11;                        // asigna valor calculado: (1000000-104*9600)*64/9600 + 0.5

        UART2_LCRH_R &= ~0b11111111;                // borra valor actual de configuracion
        UART2_LCRH_R |=  0b01110000;                // SPS=0 no importa,WLEN=11 8 bits de datos,FEN=1 con FIFO,STP2=0 1 bit de parada,
                                                    //EPS=0 no importa,PEN=0 sin paridad,BRK=0 si me interesa comunicarme

        UART2_CTL_R  &= ~0b001110100001;            // borra valor actual
        UART2_CTL_R  |=  0b001100000001;            // RXE=1,TXE=1,LBE=0,HSE=0,UARTEN=1

        GPIO_PORTD_AMSEL_R &= ~0b11000000;          // desactiva modo analogico en PD6, PD7

        GPIO_PORTD_AFSEL_R |=  0b11000000;          // activa funciones alternas en PD6, PD7
        GPIO_PORTD_PCTL_R  &= ~0xFF000000;          // borra valor anterior
        GPIO_PORTD_PCTL_R  |=  0x11000000;          // asigna nuevo valor: UART2 a PD6, PD7

        GPIO_PORTD_DEN_R   |=  0b11000000;          // activamos funciones digitales en PD6, PD7
}
uint8_t MostrarValorSensor(void){
    uint8_t sensor1;
    uint8_t sensor2;
    uint8_t salida;

    sensor1=(GPIO_PORTA_DATA_R & 0x40)>>6; //PA6

    sensor2=(GPIO_PORTA_DATA_R & 0x80)>>7; //PA7

    TxCar('\n');
    TxCar('\r');
    TxCar(sensor1+'0');
    TxCar(sensor2+'0');
    TxCar('\n');
    TxCar('\r');
    salida= (sensor1|sensor2);
    return salida;
}

void configPuertoA(){

    //se activa la señal de reloj del puerto A
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0));

    //configura PA6 y PA7 sensor
    GPIO_PORTA_DIR_R &= ~(0xC0);            // los pines PA6 y pA7 como entrada
    GPIO_PORTA_AFSEL_R &= ~(0xC0);          // se desactivan las funciones alternativas
    GPIO_PORTA_PCTL_R &= ~(0xFF000000);     // se trabaja con el GPIO y no con otras funciones alternas para PA6 y PA7
    GPIO_PORTA_AMSEL_R &= ~(0xC0);          // se desactivan las funciones analogicas para PA6 y PA7
    GPIO_PORTA_DEN_R |= 0xC0;               // se activan las funciones digitales para PA6 y PA7
}

void TxCar( uint8_t car ){
    while( UART0_FR_R & UART_FR_TXFF ) ;    // Espera mientras Buffer Tx est� lleno

    UART0_DR_R = car;                       // Se escribe el dato para su transmisi�n
}
