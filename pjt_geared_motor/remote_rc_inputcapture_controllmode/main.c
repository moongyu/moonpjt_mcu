//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//170812 PWM ON -> DELAY -> OFF -> DELAY 를 반복하는 sample code를 응용 
// forward, backword, right, left 스위치에 의해 PWM CH1, CH2를 발생함 
// HW추가로 AM-DC2-4C(L298*2EA)를 사용함 
// PORTB는 INPUT용으로 사용 가능한 것인지 확인필요 (그전에는 사용하지말것)
// PORTB가 OUTPUT으로 사용 안될 것 같으나 동작함
// IO용으로 PORT C, D, E를 사용한 코드 

//180218 HW추가로 RF Module 추가 (TXM 8D423C) 
// Keyin 신호 변경 (Forward + Backword -> Forward + Right ) : RF Module에서 지원안됨

// 180324 
// mz-10, GL-12 송수신기의 CH1, CH4에서 나오는 pluse를 Inputcapture 하여 제어함

// 190526 
// RB0 Pin의 sw를 이용해 remote control method를 가변함 (1: remote / 0 : manual; 스위치 컨트롤러와 다르게 high ative임)
// manual mode의 확인은 라즈베리파이를 이용함 

// 190729 
// 스위치 제어시 F+R, F+L Bug 수정

// 190810 
// ADC ISR 추가 
// 전원 8v 미만시 RE0 pin 통해서 high 출력 (pi가 이를 받아 자동 shutdown -h now 진행함)

// 190826
// Low speed input signal 추가 
// RB03 로 high 입력시 pwm value를 낮춰 저속으로 주행함 

// 주의
// PORTB 는 초기화시 analog input으로 설정됨으로 digital input으로 설정 변경 (Programming mode시)
// ADPCF 설정해줘야 함 


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



#include <p30f4012.h>
#include <stdio.h>
#include <pwm.h>
#include <incap.h>
#include <adc10.h>
#include "delay.h"



_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);	// FRC로 PWM동작 할 것으로 예상되나 확인 필요, 안전하게 EC_PLL8로 사용함 
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

#define ACTIVE_LOW_ENALBE_1	LATEbits.LATE4	// PWM CH3을 Disable로 설정한 후 사용 
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
#define SPEED_MIN			(0x0000)	// remote controll 방향 인식시 발생하는 pwm min값

// remote controll 방향 인식시 발생하는 channel voltage 의 주기 
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
// input capture하여 방향과 ms단위의 pluse width를 연산 
// input : pluse
// output : 방향, ms단위의 pluse width를 연산 
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

	// 방향 결정 
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

// input capture하여 방향과 ms단위의 pluse width를 연산 
// input : pluse
// output : 방향, ms단위의 pluse width를 연산 
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

	// 방향 결정 
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

	//3 remote control 시 기존처럼 시동 코드가 존재, manual mode시 무조건 시동됨 (system_on)
	if(remote_control == 1)
	{
		do
		{
			//RunDetectDir();

			//if((forward == 0) && (back == 0))	// FORWARD와 BACKWORD를 동시에 누르면 SYSTEM ON함 
			//if((forward == 0) && (right == 0))	// FORWARD와 RIGHT를 동시에 누르면 SYSTEM ON함 
			//if((dir_ch1 == forward) && (dir_ch4 == right))	// FORWARD와 RIGHT를 동시에 누르면 SYSTEM ON함 

			if(dir_ch1 == forward)	// forward 누르면 SYSTEM ON함 
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
		// PWM ON -> DELAY -> OFF -> DELAY 를 반복
		// CH1만 사용함 
		// 본 샘플코드에서는 PWM1 H에서 원하는 출력 나감 (PWM1 L 는 안됨, 무조건 High됨)
		
		SetDCMCPWM(1,0x3f00,0);	// 0x7f00 으로 설정시 디버거 중간에 끊기는 현상 발생함 (0x3f00으로 test 진행함)
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
	 //3 RORTB를 ADC 이외 사용시 필수, Pin 별로 셋팅해야함 
	 // PORTB 는 초기화시 analog input으로 설정됨, digital input으로 설정 변경 (Programming mode시)
	_PCFG0 = 1;
	_PCFG3 = 1;
	_PCFG4 = 1;
	_PCFG5 = 1;
	
	TRISB = 0xFFFF;	// RB4, RB5 IC로 사용 , RB0 Control method sw로 사용, RB3 Input speed로 사용  
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
		// 방향정보를 Marco 처럼 정의하여 사용 
		no_operation = 0;
		forward = 1;
		back = 2;
		right = 3;
		left = 4;
	}
	else
	{
		// 방향정보를 Port에서 값을 읽어와 사용 
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

	ConfigIntMCPWM(PWM_INT_DIS & PWM_INT_PR0 & PWM_FLTA_DIS_INT & PWM_FLTA_INT_PR0);	// 인터럽트 사용안함 

	SetDCMCPWM(1,0x0000,0);	// CH1, Duty 0, update안함(?)
	SetDCMCPWM(2,0x0000,0);	// CH2, Duty 0, update안함(?)

	period = 0x7fff;	// 주기레지스터 총 15bit 
	sptime = 0x0000;	// special event, 샘플코드에서는 사용 안함 

	config1 = PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 & PWM_IPCLK_SCALE1 & PWM_MOD_FREE;	//  pwm enable, free running mode
	//config2 = PWM_MOD1_COMP & PWM_MOD2_COMP & PWM_PEN1L & PWM_PEN1H & PWM_PEN2L & PWM_PEN2H & PWM_PDIS3L & PWM_PDIS3H;	// 1, 2 CH 사용가능 
	config2 = PWM_MOD1_IND & PWM_MOD2_IND & PWM_PDIS1L & PWM_PEN1H & PWM_PDIS2L & PWM_PEN2H & PWM_PDIS3L & PWM_PDIS3H;	// 1, 2 CH 사용가능
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
		//3 방향값을 gpio 통해 읽어옴, 스위치 컨트롤러시 low active, 라즈베리파이 컨트롤시 high active 
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
			ch_pwm[0] = (unsigned int)(buf0 * (-1));	// 절대값 변환
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
			ch_pwm[1] = (unsigned int)(buf1 * (-1));	// 절대값 변환
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
			BATLOW_ALRAM = 1;	// 해제조건없음, 전원 투입 후 pi on 해야함 
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
