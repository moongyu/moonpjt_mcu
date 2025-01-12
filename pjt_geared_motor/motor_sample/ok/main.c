//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//170812 PWM ON -> DELAY -> OFF -> DELAY �� �ݺ��ϴ� sample code�� ���� 
// forward, backword, right, left ����ġ�� ���� PWM CH1, CH2�� �߻��� 
// HW�߰��� AM-DC2-4C(L298*2EA)�� ����� 
// PORTB�� INPUT������ ��� ������ ������ Ȯ���ʿ� (�������� �����������)
// PORTB�� OUTPUT���� ��� �ȵ� �� ������ ������
// IO������ PORT C, D, E�� ����� �ڵ� 

//180218 HW�߰��� RF Module �߰� (TXM 8D423C) 
// Keyin ��ȣ ���� (Forward + Backword -> Forward + Right ) : RF Module���� �����ȵ�

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



#include <p30f4012.h>
#include <stdio.h>
#include <pwm.h>

_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);	// FRC�� PWM���� �� ������ ����ǳ� Ȯ�� �ʿ�, �����ϰ� EC_PLL8�� ����� 
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

/*
_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF & WDTPSA_64 & WDTPSB_4);
_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_20 & PWMxH_ACT_HI & PWMxL_ACT_HI);
_FGS(CODE_PROT_OFF);
*/

#define ACTIVE_LOW_ENALBE_1	LATEbits.LATE4	// PWM CH3�� Disable�� ������ �� ��� 
#define DIR_1				LATEbits.LATE5

#define ACTIVE_LOW_ENALBE_2	LATDbits.LATD0
#define DIR_2				LATDbits.LATD1

#define FORWORD				PORTCbits.RC15	// PORTB�� �����ÿ��� INPUT��� ���� �̷�, PORTC�� ������ 
#define BACK				PORTCbits.RC13
#define RIGHT				PORTCbits.RC14
#define LEFT				PORTEbits.RE8

#define STOP				(0x0000)
#define SPEED				(0x5fff)
#define SPEED_UP			(SPEED + 0x2000)

void motor_init(void);  
void io_init(void);

int i,j;
unsigned char system_on = 0;
unsigned int booting_delay_cnt = 0;


int main(void)
{

	TRISB = 0x003C;
	TRISC = 0xFFFF;
	TRISD = 0x0000;
	TRISE = 0x0100;

	io_init();
	motor_init();

	do
	{
		//if((FORWORD == 0) && (BACK == 0))	// FORWORD�� BACKWORD�� ���ÿ� ������ SYSTEM ON�� 
		if((FORWORD == 0) && (RIGHT == 0))	// FORWORD�� RIGHT�� ���ÿ� ������ SYSTEM ON�� 
		{
			booting_delay_cnt++;
			if(booting_delay_cnt >= 60000)
			{
				booting_delay_cnt = 0;
				system_on = 1;

				ACTIVE_LOW_ENALBE_1 = 0;
				ACTIVE_LOW_ENALBE_2 = 0;
			}
			else
			{
				system_on = 0;
			}
		}
		else
		{
			booting_delay_cnt = 0;
			system_on = 0;
		}
	}while(system_on == 0);
		
	while(system_on)
	{	
	/*
		// SAMPLE CODE
		// PWM ON -> DELAY -> OFF -> DELAY �� �ݺ�
		// CH1�� ����� 
		// �� �����ڵ忡���� PWM1 H���� ���ϴ� ��� ���� (PWM1 L �� �ȵ�, ������ High��)
		
		SetDCMCPWM(1,0x3f00,0);	// 0x7f00 ���� ������ ����� �߰��� ����� ���� �߻��� (0x3f00���� test ������)
		for(i=0;i<10000;i++)
			for(j=0;j<2000;j++);

		SetDCMCPWM(1,0x0000,0);
		for(i=0;i<10000;i++)
			for(j=0;j<1000;j++);
	*/

		// 10 : RIGHT
		// 01: LEFT
		// 00: FORWORD
		// 11: BACKWORD
		if(FORWORD == 0)
		{
			DIR_1 = 1;
			DIR_2 = 0;

			if(RIGHT == 0)
			{
				SetDCMCPWM(1,SPEED,0);
				SetDCMCPWM(2,SPEED_UP,0);
			}
			else if(LEFT == 0)
			{
				SetDCMCPWM(1,SPEED_UP,0);
				SetDCMCPWM(2,SPEED,0);
			}
			else
			{
				SetDCMCPWM(1,SPEED,0);
				SetDCMCPWM(2,SPEED,0);
			}
		}
		else if(BACK == 0)
		{
			DIR_1 = 0;	
			DIR_2 = 1;

			if(RIGHT == 0)
			{
				SetDCMCPWM(1,SPEED_UP,0);
				SetDCMCPWM(2,SPEED,0);
			}
			else if(LEFT == 0)
			{
				SetDCMCPWM(1,SPEED,0);
				SetDCMCPWM(2,SPEED_UP,0);
			}
			else
			{
				SetDCMCPWM(1,SPEED,0);
				SetDCMCPWM(2,SPEED,0);
			}
		}
		else if(RIGHT == 0)
		{
			DIR_1 = 1;
			DIR_2 = 1;
			
			SetDCMCPWM(1,SPEED,0);
			SetDCMCPWM(2,SPEED,0);
		}
		else if(LEFT == 0)
		{
			DIR_1 = 0;
			DIR_2 = 0;
			
			SetDCMCPWM(1,SPEED,0);
			SetDCMCPWM(2,SPEED,0);
		}
		else
		{
			SetDCMCPWM(1,STOP,0);
			SetDCMCPWM(2,STOP,0);
		}
		
	}
}

void motor_init()
{
	unsigned int period;
	unsigned int sptime;

	unsigned int config1; 
	unsigned int config2;
	unsigned int config3;

	ConfigIntMCPWM(PWM_INT_DIS & PWM_INT_PR0 & PWM_FLTA_DIS_INT & PWM_FLTA_INT_PR0);	// ���ͷ�Ʈ ������ 

	SetDCMCPWM(1,0x0000,0);	// CH1, Duty 0, update����(?)
	SetDCMCPWM(2,0x0000,0);	// CH2, Duty 0, update����(?)

	period = 0x7fff;	// �ֱⷹ������ �� 15bit 
	sptime = 0x0000;	// special event, �����ڵ忡���� ��� ���� 

	config1 = PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 & PWM_IPCLK_SCALE1 & PWM_MOD_FREE;	//  pwm enable, free running mode
	config2 = PWM_MOD1_COMP & PWM_MOD2_COMP & PWM_PEN1L & PWM_PEN1H & PWM_PEN2L & PWM_PEN2H & PWM_PDIS3L & PWM_PDIS3H;	// 1, 2 CH ��밡�� 
	config3 = PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN;
	
	OpenMCPWM(period, sptime, config1, config2, config3);
}

void io_init(void)
{
	FORWORD = 1;
	BACK = 1;
	RIGHT = 1;
	LEFT = 1;

	ACTIVE_LOW_ENALBE_1 = 1;
	DIR_1 = 1;

	ACTIVE_LOW_ENALBE_2 = 1;
	DIR_2 = 0;
}

