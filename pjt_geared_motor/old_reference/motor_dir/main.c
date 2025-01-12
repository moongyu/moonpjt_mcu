#include <p30f4012.h>
#include <stdio.h>
#include <pwm.h>

_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

/*
_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF & WDTPSA_64 & WDTPSB_4);
_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_20 & PWMxH_ACT_HI & PWMxL_ACT_HI);
_FGS(CODE_PROT_OFF);
*/

void motor_init(void);  
char adc_flag=0;
int i,j;

int main(void)
{
	TRISDbits.TRISD1 = 0;
	PORTD=0xffff;
	TRISE = 0x0000;
	TRISCbits.TRISC15 = 0;
	int pwm_value=0x00ff;

	motor_init();

	while(1)
	{	
		SetDCMCPWM(1,0x6f74,0);		// 0x6fff, 0x6f74
		for(i=0;i<10000;i++)
			for(j=0;j<400;j++);

		SetDCMCPWM(1,0,0);

		for(i=0;i<1000;i++)
			for(j=0;j<1000;j++);

		SetDCMCPWM(2,0x6f74,0);
		for(i=0;i<10000;i++)
			for(j=0;j<400;j++);

		SetDCMCPWM(2,0,0);

		for(i=0;i<1000;i++)
			for(j=0;j<1000;j++);
	}
}

void motor_init()
{
	unsigned int period;
	unsigned int sptime;

	unsigned int config1; 
	unsigned int config2;
	unsigned int config3;

	ConfigIntMCPWM(PWM_INT_DIS & PWM_INT_PR0 & PWM_FLTA_DIS_INT & PWM_FLTA_INT_PR0);

	period = 0x7fff; 
	sptime = 0x0000;

	config1 = PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 & PWM_IPCLK_SCALE1 & PWM_MOD_FREE;	
	config2 = PWM_MOD1_COMP & PWM_MOD2_COMP & PWM_PEN1L & PWM_PEN1H & PWM_PEN2L & PWM_PEN2H;
	config3 = PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN;
	
	OpenMCPWM(period, sptime, config1, config2, config3);
}
