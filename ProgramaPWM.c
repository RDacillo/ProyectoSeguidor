#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "TivaES.h"

void ConfigTMR1A_PWM(void){
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
	while(!(SYSCTL_PRTIMER_R & SYSCTL_PRTIMER_R1));
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
	TIMER1_CFG_R &= ~(0x07);
	TIMER1_CFG_R |= 0x04;
	TIMER1_TAMR_R = (TIMER1_TAMR_R & 0xFFFFFFF0) | 0x0A;
	TIMER1_CTL_R |= TIMER_CTL_TAPWML;
	TIMER1_TAILR_R = (TIMER1_TAILR_R & 0xFFFF0000) | 0x70FF;
	TIMER1_TAPR_R = (TIMER1_TAPR_R & 0xFFFFFF00) | 0x02;
	TIMER1_TAMATCHR_R = (TIMER1_TAMATCHR_R & 0xFFFF0000) | 0x9C3F;
	TIMER1_TAPMR_R = (TIMER1_TAPMR_R & 0xFFFFFF00) | 0x00;
	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
	while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1));
	GPIO_PORTB_DIR_R |= 0x10;
	GPIO_PORTB_AFSEL_R |= 0x10;
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFF0FFFF) | 0x00070000;
	GPIO_PORTB_AMSEL_R &= ~(0x10);
	
	GPIO_PORTB_DEN_R |= 0x10;
	TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void ActualizaCicloTrabajo(uint8_t dutyCycle){
	uint32_t comparador;
	uint16_t compL;
	uint8_t compH;
	
	if((dutyCycle>=1)&&(dutyCyle<=99)){
		comparador = 1600*dutyCycle - 1;
		compL = comparador & 0x0000FFFF;
		compH = (comparador & 0x00FF0000)>>16;
		TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
		TIMER1_TAMATCHR_R = (TIMER1_TAMATCHR_R & 0xFFFF0000) | compL;
		TIMER1_TAPMR_R = (TIMER1_TAPMR_R & 0xFFFFFF00) | compH;
		TIMER1_CTL_R |= TIMER_CTL_TAEN;
	}
	if(dutyCycle == 100){
		// Queda pendiente
	}
	if(dutyCycle == 0){
		// Queda pendiente
	}
}

int main(void){
	uint8_t cilcoTrabajo;
	uint8_t antSW1, antSW2;
	TivaES_Inicializa();
	ConfigTMR1A_PWM();
	cicloTrabajo = 25;
	antSW1 = TivaES_LeePulsador(SW1);
	antSW2 = TivaES_LeePulsador(SW2);
	while(1){
		if(TivaES_SondeaPulsador(SW1, &antSW1)){
			if(cicloTrabajo<90){
				cicloTrabajo += 5;
				ActualizaCicloTrabajo(cicloTrabajo);
			}
		}
		if(TivaES_SondeaPulsador(SW2, &antSW2)){
			if(cicloTrabajo>10){
				cicloTrabajo -= 5;
				ActualizaCicloTrabajo(cicloTrabajo);
			}
		}
	}
}
