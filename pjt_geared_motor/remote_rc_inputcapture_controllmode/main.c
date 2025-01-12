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

// 180324 
// mz-10, GL-12 �ۼ��ű��� CH1, CH4���� ������ pluse�� Inputcapture �Ͽ� ������

// 190526 
// RB0 Pin�� sw�� �̿��� remote control method�� ������ (1: remote / 0 : manual; ����ġ ��Ʈ�ѷ��� �ٸ��� high ative��)
// manual mode�� Ȯ���� ��������̸� �̿��� 

// 190729 
// ����ġ ����� F+R, F+L Bug ����

// 190810 
// ADC ISR �߰� 
// ���� 8v �̸��� RE0 pin ���ؼ� high ��� (pi�� �̸� �޾� �ڵ� shutdown -h now ������)

// 190826
// Low speed input signal �߰� 
// RB03 �� high �Է½� pwm value�� ���� �������� ������ 

// ����
// PORTB �� �ʱ�ȭ�� analog input���� ���������� digital input���� ���� ���� (Programming mode��)
// ADPCF ��������� �� 


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



#include <p30f4012.h>
#include <stdio.h>
#include <pwm.h>
#include <incap.h>
#include <adc10.h>
#include "delay.h"



_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);	// FRC�� PWM���� �� ������ ����ǳ� Ȯ�� �ʿ�, �����ϰ� EC_PLL8�� ����� 
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

//Fcy=Fosc/4 (dsPIC30F)
// Fosc = 8M * 8 = 64Mhz (POST 1:1)
// Fcy = 64Mhz/4 = 16Mhz
#define XOSC				8000000
#define PLL					8
//#define PERSCALE			1	// POST	: no alter
#define TICK				((float)1/((XOSC * PLL)/4))

#define CONTROL_MODE_SW		(_RB0)

#define IN_SPEED			(_RB3)

#define ACTIVE_LOW_ENALBE_1	LATEbits.LATE4	// PWM CH3�� Disable�� ������ �� ��� 
#define DIR_1				LATEbits.LATE5

#define ACTIVE_LOW_ENALBE_2	LATDbits.LATD0
#define DIR_2				LATDbits.LATD1

#define BATLOW_ALRAM		(_RE0)

#define STOP				(0x0000)
#define SPEED_LOW			(0x1800)
#define SPEED_UP_LOW		(SPEED_LOW + 0x500)
#define SPEED				(0x3fff)
#define SPEED_UP			(SPEED + 0x2000)
#define SPEED_MAX			(0x7fff)
#define SPEED_MIN			(0x0000)	// remote controll ���� �νĽ� �߻��ϴ� pwm min��

// remote controll ���� �νĽ� �߻��ϴ� channel voltage �� �ֱ� 
#define CH_VOLT_MAX			(2000)	// [0.001] mili sec; 2.0ms 
#define CH_VOLT_REF			(1500)	// [0.001] mili sec; 1.5ms 
#define CH_VOLT_MIN			(1000)	// [0.001] mili sec; 1.0ms 
#define CH_VOLT_HYSERESIS	(70)	// [0.001] mili sec; 0.05ms 

#define ACTIVE_POLE			(0)

#define LIMIT_VOLT_ALRAM	(8.0)

void motor_init(void);  
void io_init(void);
void inputcapture_init(void); 
void adc_init(void); 

void RunDetectDir(void);
void RunMotor(void);

void ctl_method_init(void);
void get_dir(void);

void CalcBatteryVoltage(void);
void CtrlBatSwd(void);

void GetMotorSpeed(void);

int i,j;
unsigned char system_on = 0;
unsigned int booting_delay_cnt = 0;

unsigned char dir_ch4, dir_ch1=0;

float Coeff = 0;
unsigned int ch_pwm[2] = {0};

int Interrupt_Count = 0 , Int_flag;
unsigned int timer_first_edge0, timer_first_edge1;
unsigned int timer_second_edge0, timer_second_edge1;
unsigned int timer_pluse_width_raw0, timer_pluse_width_raw1;

float timer_pluse_width_sec0, timer_pluse_width_sec1;
unsigned int timer_pluse_width_ms0, timer_pluse_width_ms1;

unsigned char remote_control = 0;

unsigned char no_operation = 0, forward = 0, back = 0, right = 0,left = 0;

int adc_save_count=0;
int buf[2]={0}, adc_sum[2]={0};
long adc_average_sum[2]={0};

float f32batteryvoltage = 25.0;

unsigned char u8Motorspeed = 0;
unsigned int u16SpeedNormalVal = SPEED;
unsigned int u16SpeedUpVal = SPEED_UP;

///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
// input capture�Ͽ� ����� ms������ pluse width�� ���� 
// input : pluse
// output : ����, ms������ pluse width�� ���� 
void __attribute__((__interrupt__)) _IC7Interrupt(void)
{
	// controller-ch1 (forward, backward meaurement)
	
	//_RE0 = ~_RE0;

	if(_RB4 == 1)
	{
		ReadCapture7(&timer_first_edge0);
	}
	else if(_RB4 == 0)
	{
		ReadCapture7(&timer_second_edge0);
		//Int_flag = 1;
		
		timer_pluse_width_raw0 = timer_second_edge0 - timer_first_edge0;
		timer_pluse_width_sec0 = (float)timer_pluse_width_raw0 * TICK * 8;	// Timer Prescale = 8
		timer_pluse_width_ms0 = (unsigned int)(timer_pluse_width_sec0 * 1000000);
	}
	else
	{
	}

	IFS1bits.IC7IF = 0;

	// ���� ���� 
	if(timer_pluse_width_ms0 >= (CH_VOLT_REF + CH_VOLT_HYSERESIS))
	{
		dir_ch1 = forward;
	}
	else if(timer_pluse_width_ms0 < (CH_VOLT_REF - CH_VOLT_HYSERESIS))
	{
		dir_ch1 = back;
	}
	else
	{
		dir_ch1 = no_operation;
	}
}

// input capture�Ͽ� ����� ms������ pluse width�� ���� 
// input : pluse
// output : ����, ms������ pluse width�� ���� 
void __attribute__((__interrupt__)) _IC8Interrupt(void)
{
	// controller-ch4 (right, left meaurement)
	
	//_RE0 = ~_RE0;

	if(_RB5 == 1)
	{
		ReadCapture8(&timer_first_edge1);
	}
	else if(_RB5 == 0)
	{
		ReadCapture8(&timer_second_edge1);
		//Int_flag = 1;
		
		timer_pluse_width_raw1 = timer_second_edge1 - timer_first_edge1;
		timer_pluse_width_sec1 = (float)timer_pluse_width_raw1 * TICK * 8;	// Timer Prescale = 8
		timer_pluse_width_ms1 = (unsigned int)(timer_pluse_width_sec1 * 1000000);
	}
	else
	{
	}

	IFS1bits.IC8IF = 0;

	// ���� ���� 
	if(timer_pluse_width_ms1 >= CH_VOLT_REF + CH_VOLT_HYSERESIS)
	{
		dir_ch4 = left;
	}
	else if(timer_pluse_width_ms1 < CH_VOLT_REF - CH_VOLT_HYSERESIS)
	{
		dir_ch4 = right;
	}
	else
	{
		dir_ch4 = no_operation;
	}
}

void __attribute__ ((__interrupt__)) _ADCInterrupt(void)
{

	buf[0] = 0;//ReadADC10(0);
	buf[1] = ReadADC10(1);
	
	adc_save_count++;
	adc_sum[0] += buf[0];
	adc_sum[1] += buf[1];
	
	if(adc_save_count == 20)
	{
		adc_average_sum[0] = adc_sum[0]/20;
		adc_sum[0] = 0;

		adc_average_sum[1] = adc_sum[1]/20;
		adc_sum[1] = 0;
		
		adc_save_count = 0;

		CalcBatteryVoltage();
	}

	IFS0bits.ADIF=0;

	//_RD0 = 1;
}

int main(void)
{
	io_init();
	adc_init();
	motor_init();	

	if(remote_control == 1)
	{
		inputcapture_init();
	}

	Delay_ms(500);

	// Application Initialization
	Coeff = (float)((unsigned int)SPEED_MAX - SPEED_MIN) / ((unsigned int)CH_VOLT_MAX - CH_VOLT_REF);

	//3 remote control �� ����ó�� �õ� �ڵ尡 ����, manual mode�� ������ �õ��� (system_on)
	if(remote_control == 1)
	{
		do
		{
			//RunDetectDir();

			//if((forward == 0) && (back == 0))	// FORWARD�� BACKWORD�� ���ÿ� ������ SYSTEM ON�� 
			//if((forward == 0) && (right == 0))	// FORWARD�� RIGHT�� ���ÿ� ������ SYSTEM ON�� 
			//if((dir_ch1 == forward) && (dir_ch4 == right))	// FORWARD�� RIGHT�� ���ÿ� ������ SYSTEM ON�� 

			if(dir_ch1 == forward)	// forward ������ SYSTEM ON�� 
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
	}
	else
	{
		ACTIVE_LOW_ENALBE_1 = 0;
		ACTIVE_LOW_ENALBE_2 = 0;
		system_on = 1;
	}
	
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

		//RunDetectDir();

		CtrlBatSwd();

		GetMotorSpeed();

		RunMotor();
		
	}
}

void io_init(void)
{
	 //3 RORTB�� ADC �̿� ���� �ʼ�, Pin ���� �����ؾ��� 
	 // PORTB �� �ʱ�ȭ�� analog input���� ������, digital input���� ���� ���� (Programming mode��)
	_PCFG0 = 1;
	_PCFG3 = 1;
	_PCFG4 = 1;
	_PCFG5 = 1;
	
	TRISB = 0xFFFF;	// RB4, RB5 IC�� ��� , RB0 Control method sw�� ���, RB3 Input speed�� ���  
	TRISC = 0xFFFF;
	TRISD = 0x0000;
	TRISE = 0x0100;	// Output : RE0, 2 		Input : RE8

	if(CONTROL_MODE_SW == 1)
	{
		remote_control = 1;
	}
	else
	{
		remote_control = 0;
	}

	if(remote_control == 1)
	{
		// ���������� Marco ó�� �����Ͽ� ��� 
		no_operation = 0;
		forward = 1;
		back = 2;
		right = 3;
		left = 4;
	}
	else
	{
		// ���������� Port���� ���� �о�� ��� 
		//PORTCbits.RC15 = 0;
		//PORTCbits.RC13 = 0;
		//PORTCbits.RC14 = 0;
		//PORTEbits.RE8 = 0;

		get_dir();
	}

	GetMotorSpeed();
	
	ACTIVE_LOW_ENALBE_1 = 1;
	DIR_1 = 1;

	ACTIVE_LOW_ENALBE_2 = 1;
	DIR_2 = 0;

	BATLOW_ALRAM = 0;
}

void motor_init(void)
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
	//config2 = PWM_MOD1_COMP & PWM_MOD2_COMP & PWM_PEN1L & PWM_PEN1H & PWM_PEN2L & PWM_PEN2H & PWM_PDIS3L & PWM_PDIS3H;	// 1, 2 CH ��밡�� 
	config2 = PWM_MOD1_IND & PWM_MOD2_IND & PWM_PDIS1L & PWM_PEN1H & PWM_PDIS2L & PWM_PEN2H & PWM_PDIS3L & PWM_PDIS3H;	// 1, 2 CH ��밡��
	config3 = PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN;
	
	OpenMCPWM(period, sptime, config1, config2, config3);
}

void inputcapture_init(void)
{
/////////////////////////////// Inputcapture Initilization ///////////////////////////////
	// Enable IC7 Interrupt and Priority to '1' 
	ConfigIntCapture7(IC_INT_PRIOR_1 & IC_INT_ON);

	// Configure the InputCapture7 in stop in idle mode , Timer
	// 3 as source , interrupt on capture 1, I/C on every edge 
	OpenCapture7(IC_IDLE_STOP & IC_TIMER3_SRC &
	IC_INT_1CAPTURE & IC_EVERY_EDGE);	// IC_EVERY_RISE_EDGE


	// Enable IC8 Interrupt and Priority to '1' 
	ConfigIntCapture8(IC_INT_PRIOR_1 & IC_INT_ON);

	// Configure the InputCapture8 in stop in idle mode , Timer
	// 3 as source , interrupt on capture 1, I/C on every edge 
	OpenCapture8(IC_IDLE_STOP & IC_TIMER3_SRC &
	IC_INT_1CAPTURE & IC_EVERY_EDGE);	// IC_EVERY_RISE_EDGE

	T3CONbits.TON = 1;		// Timer 3 On 
	T3CONbits.TCKPS0 = 1;	//Prescale (1:8), Internal clcok (Default)
	T3CONbits.TCKPS1 = 0;
}

/*
void RunDetectDir(void)
{
	// forward, BACKWARD
	if(timer_pluse_width_ms0 <= CH_VOLT_REF - CH_VOLT_HYSERESIS)
	{
		dir_ch1 = forward;
	}
	else if(timer_pluse_width_ms0 > CH_VOLT_REF + CH_VOLT_HYSERESIS)
	{
		dir_ch1 = back;
	}
	else
	{
		dir_ch1 = no_operation;
		timer_pluse_width_ms0 = 0;
	}

	// right, left
	if(timer_pluse_width_ms1 <= CH_VOLT_REF - CH_VOLT_HYSERESIS)
	{
		dir_ch4 = left;
	}
	else if(timer_pluse_width_ms1 > CH_VOLT_REF + CH_VOLT_HYSERESIS)
	{
		dir_ch4 = right;
	}
	else
	{
		dir_ch4 = no_operation;
		timer_pluse_width_ms1 = 0;
	}
}
*/

void adc_init()
{
//*************************************** A/D Conversion Initilization **********************************************
SetChanADC10(ADC_CH0_POS_SAMPLEA_AN0 &		// ADC_CH0_POS_SAMPLEA_AN0, ADC_CHX_POS_SAMPLEA_AN0AN1AN2
			ADC_CH0_POS_SAMPLEA_AN1&
			ADC_CH0_POS_SAMPLEA_AN2&
			ADC_CH0_NEG_SAMPLEA_NVREF);

ConfigIntADC10(ADC_INT_ENABLE & 		// ENABLE, DISABLE
			ADC_INT_PRI_6); 			// 0~7

OpenADC10(	ADC_MODULE_ON & 				// ON, OFF
			ADC_IDLE_CONTINUE & 			// IDLE_STOP, IDLE_CONTINUE
			ADC_FORMAT_INTG &			// SIGN_FRACT, FRACT, SIGN_INT, INTG
			ADC_CLK_AUTO &				// AUTO, MPWM, TMR, INT0, MANUAL
			ADC_AUTO_SAMPLING_ON &		// ON, OFF
			ADC_SAMPLE_SIMULTANEOUS &		// SIMULTANEOUS, INDIVIDUAL
			ADC_SAMP_ON,					// ON, OFF 

			ADC_VREF_AVDD_AVSS &			// AVDD_AVSS, EXT_AVSS, AVDD_EXT, EXT_EXT
			ADC_SCAN_ON &					// ON, OFF
			ADC_CONVERT_CH0 &				// CH0, CH_0A, CH_0ABC
			ADC_SAMPLES_PER_INT_3 & 	// 1~16
			ADC_ALT_BUF_OFF &				// ON, OFF
			ADC_ALT_INPUT_OFF,			// ON, OFF

			ADC_SAMPLE_TIME_0 & 			// 0~31
			ADC_CONV_CLK_SYSTEM &			// INTERNAL_RC, SYSTEM
			ADC_CONV_CLK_16Tcy, 		// Tcy2, Tcy, 3Tcy2~32Tcy

			//ENABLE_AN0_ANA &				// ALL_ANA, ALL_DIG, AN0_ANA~AN15_ANA_ANA
			ENABLE_AN1_ANA,
			//ENABLE_AN2_ANA,

			SCAN_ALL);					// NONE, ALL, SKIP_SCAN_AN0~15

}

void RunMotor(void)
{
	long buf0,buf1;

	if(remote_control == 0)
	{
		//3 ���Ⱚ�� gpio ���� �о��, ����ġ ��Ʈ�ѷ��� low active, ��������� ��Ʈ�ѽ� high active 
		get_dir();
		
		// 10 : right
		// 01: left
		// 00: forward
		// 11: BACKWORD
		if(forward == ACTIVE_POLE)
		{
			DIR_1 = 1;
			DIR_2 = 0;
		
			if(right == ACTIVE_POLE)
			{
				SetDCMCPWM(1,u16SpeedUpVal,0);
				SetDCMCPWM(2,u16SpeedNormalVal,0);
			}
			else if(left == ACTIVE_POLE)
			{
				SetDCMCPWM(1,u16SpeedNormalVal,0);
				SetDCMCPWM(2,u16SpeedUpVal,0);
			}
			else
			{
				SetDCMCPWM(1,u16SpeedNormalVal,0);
				SetDCMCPWM(2,u16SpeedNormalVal,0);
			}
		}
		else if(back == ACTIVE_POLE)
		{
			DIR_1 = 0;	
			DIR_2 = 1;
		
			if(right == ACTIVE_POLE)
			{
				SetDCMCPWM(1,u16SpeedUpVal,0);
				SetDCMCPWM(2,u16SpeedNormalVal,0);
			}
			else if(left == ACTIVE_POLE)
			{
				SetDCMCPWM(1,u16SpeedNormalVal,0);
				SetDCMCPWM(2,u16SpeedUpVal,0);
			}
			else
			{
				SetDCMCPWM(1,u16SpeedNormalVal,0);
				SetDCMCPWM(2,u16SpeedNormalVal,0);
			}
		}
		else if(right == ACTIVE_POLE)
		{
			DIR_1 = 1;
			DIR_2 = 1;
			
			SetDCMCPWM(1,u16SpeedNormalVal,0);
			SetDCMCPWM(2,u16SpeedNormalVal,0);
		}
		else if(left == ACTIVE_POLE)
		{
			DIR_1 = 0;
			DIR_2 = 0;
			
			SetDCMCPWM(1,u16SpeedNormalVal,0);
			SetDCMCPWM(2,u16SpeedNormalVal,0);
		}
		else
		{
			SetDCMCPWM(1,STOP,0);
			SetDCMCPWM(2,STOP,0);
		}
	}	
	else
	{
		////////////////////////////////
		// input : pluse width, coeff
		// output : pwm value 
		buf0 = (long)(Coeff * timer_pluse_width_ms1 - 98301);	// remote ch4, r/l
		buf1 = (long)(Coeff * timer_pluse_width_ms0 - 98301);	// remote ch1, f/b

		if((buf0 >= 0x7fff) || (buf0 <= -32767))
		{
			ch_pwm[0] = 0x7fff;							// pwm max
		}
		else if(buf0 < 0)
		{
			ch_pwm[0] = (unsigned int)(buf0 * (-1));	// ���밪 ��ȯ
		}
		else
		{
			ch_pwm[0] = (unsigned int)buf0;				// remote ch4, r/l
		}
		
		if((buf1 >= 0x7fff) || (buf1 <= -32767))
		{
			ch_pwm[1] = 0x7fff;							// pwm max
		}
		else if(buf1 < 0)
		{
			ch_pwm[1] = (unsigned int)(buf1 * (-1));	// ���밪 ��ȯ
		}
		else
		{
			ch_pwm[1] = (unsigned int)buf1;				// remote ch1, f/b
		}
		////////////////////////////////


		// 10 : right
		// 01: left
		// 00: forward
		// 11: BACKWORD

		// ch_pwm[1] : ch1(f/b), ch_pwm[0] : ch4(r/l)
		if(dir_ch1 == forward)
		{
			DIR_1 = 1;	// left wheel
			DIR_2 = 0;	// right wheel

			if(dir_ch4 == right)
			{
				//SetDCMCPWM(1,ch_pwm[0],0);	// SetDCMCPWM1 : left wheel
				SetDCMCPWM(1,0x7f00,0);		//3 SetDCMCPWM1 : left wheel, "speed up /w fixed value"
				SetDCMCPWM(2,ch_pwm[1],0);	// SetDCMCPWM2 : right wheel
			}
			else if(dir_ch4 == left)
			{
				SetDCMCPWM(1,ch_pwm[1],0);
				//SetDCMCPWM(2,ch_pwm[0],0);
				SetDCMCPWM(2,0x7f00,0);
			}
			else
			{
				SetDCMCPWM(1,ch_pwm[1],0);
				SetDCMCPWM(2,ch_pwm[1],0);
			}
		}
		else if(dir_ch1 == back)
		{
			DIR_1 = 0;	
			DIR_2 = 1;

			if(dir_ch4 == right)
			{
				//SetDCMCPWM(1,ch_pwm[0],0);
				SetDCMCPWM(1,0x7f00,0);
				SetDCMCPWM(2,ch_pwm[1],0);
			}
			else if(dir_ch4 == left)
			{
				SetDCMCPWM(1,ch_pwm[1],0);
				//SetDCMCPWM(2,ch_pwm[0],0);
				SetDCMCPWM(2,0x7f00,0);
			}
			else
			{
				SetDCMCPWM(1,ch_pwm[1],0);
				SetDCMCPWM(2,ch_pwm[1],0);
			}
		}
		else if(dir_ch4 == right)
		{
			DIR_1 = 1;
			DIR_2 = 1;
			
			SetDCMCPWM(1,ch_pwm[0],0);
			SetDCMCPWM(2,ch_pwm[0],0);
		}
		else if(dir_ch4 == left)
		{
			DIR_1 = 0;
			DIR_2 = 0;
			
			SetDCMCPWM(1,ch_pwm[0],0);
			SetDCMCPWM(2,ch_pwm[0],0);
		}
		else
		{
			SetDCMCPWM(1,STOP,0);
			SetDCMCPWM(2,STOP,0);
		}
	}
}

void get_dir(void)
{
	forward = PORTCbits.RC15;
	back = PORTCbits.RC13;
	right = PORTCbits.RC14;
	left = PORTEbits.RE8;
}

void CalcBatteryVoltage(void)
{
	// R1 = 24k, R2 = 68K
	f32batteryvoltage = (((float)adc_average_sum[1] / 1024.0) * 5.0 ) / (24.0/(68.0+24.0));
}

void CtrlBatSwd(void)
{
	static unsigned int u16cnt = 0;
	
	if(f32batteryvoltage <= LIMIT_VOLT_ALRAM)
	{
		u16cnt++;
		
		if(u16cnt >= 10)
		{
			BATLOW_ALRAM = 1;	// �������Ǿ���, ���� ���� �� pi on �ؾ��� 
		}
	}
	else
	{
		u16cnt = 0;
		//BATLOW_ALRAM = 1;	// _debug
	}
}

void GetMotorSpeed(void)
{
	static unsigned char su8_speed_lowcnt = 0;
	static unsigned char su8_speed_upcnt = 0;
	
	if(IN_SPEED == 0)
	{
		su8_speed_lowcnt = 0;
		
		if(su8_speed_upcnt >= 2)
		{
			u8Motorspeed = 0;

			u16SpeedNormalVal = SPEED;
			u16SpeedUpVal = SPEED_UP;
		}
		else
		{
			su8_speed_upcnt++;
		}
	}
	else
	{
		su8_speed_upcnt = 0;

		if(su8_speed_lowcnt >= 2)
		{
			u8Motorspeed = 1;

			u16SpeedNormalVal = SPEED_LOW;
			u16SpeedUpVal = SPEED_UP_LOW;
		}
		else
		{
			su8_speed_lowcnt++;
		}
	}
}
