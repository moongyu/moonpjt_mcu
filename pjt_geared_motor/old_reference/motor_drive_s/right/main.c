#include <p30f4012.h>
#include <stdio.h>
#include <pwm.h>

_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

#define FCY  16000000 
#define BAUDRATE 	9600        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1

void uart_init(void);
void motor_init(void);  
void delay_ms(unsigned int n);
void uart_put_string(unsigned char *uart_string);
void drive_mode_s(void);
void drive_mode_circle_fast(void);
void drive_mode_circle_slow(void);

char adc_flag=0;
int i,j;
unsigned char received_data=0xff;
int pwm_value_1=0x0000, pwm_value_2=0x0000;

void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF=0;
	received_data=U1RXREG;
}

void __attribute__((__interrupt__)) _U1TXInterrupt(void)
{
	IFS0bits.U1TXIF=0;
}

int main(void)
{
	TRISDbits.TRISD1 = 0;
	PORTD=0xffff;
	TRISE = 0x0000;
	TRISCbits.TRISC15 = 0;

	uart_init();
	motor_init();

//	printf("uart test ok!");

	SetDCMCPWM(1,0,0);
	SetDCMCPWM(2,0,0);

	while(1)
	{	
// right wheel
/*
		if(received_data==0x00) 		// forward
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x6fff;
			pwm_value_2=0x0000;
			received_data=0xff;
		}
		else if(received_data==0x02) 	// right
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x0000;
			pwm_value_2=0x6fff;

			SetDCMCPWM(1,pwm_value_1,0);
			SetDCMCPWM(2,pwm_value_2,0);
			delay_ms(50);
			pwm_value_1=0x0000;
			pwm_value_2=0x0000;

			received_data=0xff;
		}
		else if(received_data==0x03) 	// left
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x6fff;
			pwm_value_2=0x0000;

			SetDCMCPWM(1,pwm_value_1,0);
			SetDCMCPWM(2,pwm_value_2,0);
			delay_ms(50);
			pwm_value_1=0x0000;
			pwm_value_2=0x0000;

			received_data=0xff;
		}
		else if(received_data==0x01) 	// backward
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x0000;
			pwm_value_2=0x6fff;
			received_data=0xff;
		}
		else if(received_data==0x44) 	// stop
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x0000;
			pwm_value_2=0x0000;
			received_data=0xff;
		}
		else if(received_data==0x08) 	// drive_mode_s
		{
			drive_mode_s();
		}
		else 
		{
		}

			SetDCMCPWM(1,pwm_value_1,0);
			SetDCMCPWM(2,pwm_value_2,0);
*/

//	left wheel

		if(received_data==0x00) 		// forward
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x6fff;
			pwm_value_2=0x0000;
			received_data=0xff;
		}
		else if(received_data==0x02) 	// right
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x6fff;
			pwm_value_2=0x0000;

			SetDCMCPWM(1,pwm_value_1,0);
			SetDCMCPWM(2,pwm_value_2,0);
			delay_ms(50);
			pwm_value_1=0x0000;
			pwm_value_2=0x0000;

			received_data=0xff;
		}
		else if(received_data==0x03) 	// left
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x0000;
			pwm_value_2=0x6fff;

			SetDCMCPWM(1,pwm_value_1,0);
			SetDCMCPWM(2,pwm_value_2,0);
			delay_ms(50);
			pwm_value_1=0x0000;
			pwm_value_2=0x0000;

			received_data=0xff;
		}
		else if(received_data==0x01) 	// backward
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x0000;
			pwm_value_2=0x6fff;
			received_data=0xff;
		}
		else if(received_data==0x44) 	// stop
		{
			SetDCMCPWM(1,0,0);
			SetDCMCPWM(2,0,0);
			pwm_value_1=0x0000;
			pwm_value_2=0x0000;
			received_data=0xff;
		}
		else if(received_data==0x08) 	// drive_mode_s
		{
			drive_mode_s();
		}
		else if(received_data==0x11) 	// drive_mode_circle
		{
//			drive_mode_circle_fast();
			drive_mode_circle_slow();
		}
		else 
		{
		}

			SetDCMCPWM(1,pwm_value_1,0);
			SetDCMCPWM(2,pwm_value_2,0);

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

void uart_init(void)
{
	U1MODEbits.STSEL=0;
	U1MODEbits.PDSEL=0;
	U1MODEbits.ABAUD=0;

	U1BRG=BRGVAL;
	U1STAbits.URXISEL=0;
	U1STAbits.UTXISEL=1;

	U1MODEbits.ALTIO=1;

	U1MODEbits.UARTEN=1;
	U1STAbits.UTXEN=1;

	IFS0bits.U1TXIF=0;
	IFS0bits.U1RXIF=0;

	IEC0bits.U1TXIE=1;
	IEC0bits.U1RXIE=1;
}

void delay_ms(unsigned int n)
{
	unsigned int j;
	while(n--)
	{
		for(j=0; j<3197; j++)	// 128MHz : 3197
		{
			ClrWdt();
		}
	}
}

void drive_mode_s()
{
	int pwm_high=0x5fff, pwm_low=0x2fff, pwm_zero=0x0000;
// right
/*
	SetDCMCPWM(1,0,0);			// left_first
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_high,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(1000);

	SetDCMCPWM(1,0,0);			// forw
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_low,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(100);

	SetDCMCPWM(1,0,0);			// right 
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_low,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(2000);

	SetDCMCPWM(1,0,0);			// forw
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_low,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(100);

	SetDCMCPWM(1,0,0);			// left
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_high,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(2000);
*/

// left

	SetDCMCPWM(1,0,0);			// left_first
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_low,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(1000);

	SetDCMCPWM(1,0,0);			// forw
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_low,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(100);

	SetDCMCPWM(1,0,0);			// right 
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_high,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(2000);

	SetDCMCPWM(1,0,0);			// forw
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_low,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(100);

	SetDCMCPWM(1,0,0);			// left
	SetDCMCPWM(2,0,0);
	SetDCMCPWM(1,pwm_low,0);
	SetDCMCPWM(2,pwm_zero,0);	
	delay_ms(2000);

	received_data=0xff;
}

void drive_mode_circle_fast()
{
	int pwm_high_1=31000, pwm_low_1=11000, pwm_zero=0, pwm_high_2=23000, pwm_low_2=3000;
	SetDCMCPWM(1,0,0);			
	SetDCMCPWM(2,0,0);
	
	while(1){
	SetDCMCPWM(1,pwm_high_2,0);
	SetDCMCPWM(2,pwm_zero,0);	
	}
	received_data=0xff;
}

void drive_mode_circle_slow()
{
	int pwm_high_1=31000, pwm_low_1=11000, pwm_zero=0, pwm_high_2=23000, pwm_low_2=3000;
	SetDCMCPWM(1,0,0);			
	SetDCMCPWM(2,0,0);
	
	while(1){
	SetDCMCPWM(1,pwm_low_2,0);
	SetDCMCPWM(2,pwm_zero,0);	
	}
	received_data=0xff;
}

