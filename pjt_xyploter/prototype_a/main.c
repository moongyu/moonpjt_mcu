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

// 200222
// PORT제어 후 즉시 다른 PORT 제어하면 이 전 PORT동작안함, delay 필요 
// EX. _RD0=1;_RD1=1; 하면 _RD0=0으로 output
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



#include <p30f4012.h>
#include <stdio.h>
#include <pwm.h>
#include <incap.h>
#include <adc10.h>
#include <timer.h>
#include "delay.h"
#include "define.h"



_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);	// FRC로 PWM동작 할 것으로 예상되나 확인 필요, 안전하게 EC_PLL로 사용함 
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

//Fcy=Fosc/4 (dsPIC30F)
// Fosc = 8M * 8 = 64Mhz (POST 1:1)
// Fcy = 64Mhz/4 = 16Mhz
#define XOSC				8000000
#define PLL					8
//#define PERSCALE			1	// POST	: no alter
#define TICK				((float)1/((XOSC * PLL)/4))	// 1/16M = 0.0625us

#define READY		(0)
#define RUN			(1)


#define STOP				(0x0000)
#define SPEED_LOW_3			(0x0fff)
#define SPEED_LOW_2			(0x1fff)
#define SPEED_LOW_1			(0x2fff)
#define SPEED				(0x3fff)
#define SPEED_UP_1			(0x4fff)
#define SPEED_UP_2			(0x5fff)
#define SPEED_UP_3			(0x6fff)
#define SPEED_MAX			(0x7fff)
#define SPEED_MIN			(0x0000)	// remote controll 방향 인식시 발생하는 pwm min값

//#define STEP_MOTOR_PULSE_W	(70)	//170ma?	// 1/32step
//#define STEP_MOTOR_PULSE_W	(60)	// 158ma

//#define STEP_MOTOR_PULSE_W	(2300)	// 1800(?) - 2300	full step 	2300: 850us, 3.68ms

#define X_AISX_DIR_RIGHT	(0)
#define X_AISX_DIR_LEFT		(1)


#define STEP_MOTOR_PULSE_W	(500)
#define CRAINKING_DROP_TIME	(65)

#define IN_LEFE		(_RB3)
#define IN_RIGHT	(_RB2)
#define IN_UP		(_RB4)
#define IN_DOWN		(_RB5)
#define IN_CRAIN_GRAP	(_RB0)
#define IN_TEST		(_RB1)

#define IN_Z_AXIS_TACTSW		(_RC15)

#define DIR_X_AXIS_MOTOR		(_RD1)
#define ENABLE_X_AXIS_MOTOR		(_RD0)
#define X_AXIS_STEP_MOTOR_PWM	(_RE0)

#define DIR_Y_AXIS_MOTOR		(_RE8)

#define DIR_Z_AXIS_MOTOR		(_RE4)

#define OUT_CRAIN_PWM			(_RE1)	// _test

#define IN_TACTSW_X_AXIS_ORIGIN		(_RC13)
#define IN_TACTSW_Y_AXIS_ORIGIN		(_RC14)
#define IN_TACTSW_Y_AXIS_END		(_RE2)


enum
{
	INIT,
	CRAINKING_DOWN,
	GRAP,
	CRAINKING_UP,
	GO_HOME,
	DROP,
	FINISH
}eCrankStateDef;

void motor_init(void);  
void io_init(void);
void adc_init(void); 

void SensingSwitch_Xaxis(void);
void ControlMotorEnable_Xaxis(unsigned char);
void ControlMotor_Xaxis(void);

void ControlMotor_Yaxis(void);
void SensingSwitch_Yaxis(void);

void ControlMotor_Zaxis(void);
void SensingSwitch_Zaxis(void);

void GrapProgress(void);
void CheckCrankingDownTimer(void);

void SensingSwitch_Test(void);

void SensingTactsw(void);

int adc_save_count=0;
int buf[2]={0}, adc_sum[2]={0};
long adc_average_sum[2]={0};

unsigned char u8Motorspeed = 0;
unsigned int u16SpeedNormalVal = SPEED;
unsigned int u16SpeedUpVal = SPEED_UP_1;

unsigned char gucContolMotor_XaxisFlag = 0;
unsigned char gucContolMotor_YaxisFlag = 0;
unsigned char gucContolMotor_ZaxisFlag = 0;

unsigned char gucGrapState = INIT;	// _test 초기화 

unsigned char gucCrankingDownTimerFlag = 0;
unsigned char gucContolGrapFlag = 0, gucContolGrapDoneFlag = 0;

unsigned char guc_z_axis_tactsw = 0, guc_x_axis_tactsw = 0, guc_y_axis_tactsw = 0, guc_y_axis_end_tactsw = 0;

unsigned char gucContolMotor_GrapProcessXaxisFlag, gucContolMotor_GrapProcessYaxisFlag = 0;

unsigned int gunReal_cnt = 0;

///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
// input capture하여 방향과 ms단위의 pluse width를 연산 
// input : pluse
// output : 방향, ms단위의 pluse width를 연산 
void __attribute__((__interrupt__)) _IC7Interrupt(void)
{
}

void __attribute__((__interrupt__)) _IC8Interrupt(void)
{
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
	}

	IFS0bits.ADIF=0;

	//_RD0 = 1;
}

/////////////////////////////////////////////////////////////////////////
// 1ms Timer
void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	static unsigned int unBaseTime0 = 0;
	static unsigned int timer_10msec = 0;
	static unsigned int timer_100msec = 0;
	static unsigned int timer_1sec = 0;

	unsigned char crain_onoff_flag = 0;

	static unsigned char ucContolGrapTimer = 0;

	WriteTimer3(0);

	IFS0bits.T3IF=0;

	//_RD1 = !_RD1;

	SensingTactsw();

	unBaseTime0++;

	if(unBaseTime0 >= 10)
	{
		unBaseTime0 = 0;
		timer_10msec++;

		//_RE1 = !_RE1;
	}

	if(timer_10msec >= 10)
	{
		timer_10msec = 0;
		timer_100msec++;

		CheckCrankingDownTimer();
	}
	
	if(timer_100msec >= 10)
	{
		timer_100msec = 0;
		timer_1sec++;

		if(gucContolGrapFlag == SET)
		{
			ucContolGrapTimer++;
		}
	
	}

	ControlMotor_Xaxis();


	if(gucContolGrapFlag == SET)
	{
		crain_onoff_flag = unBaseTime0 % 5;
		if(crain_onoff_flag >= 3)
		{
			OUT_CRAIN_PWM = 0;
		}
		else
		{
			OUT_CRAIN_PWM = 1;
		}

		if(ucContolGrapTimer >= 3)
		{
			ucContolGrapTimer = 0;
			gucContolGrapDoneFlag = SET;
		}
		else
		{
			//gucContolGrapDoneFlag = CLEAR;
		}
		
	}
	else
	{
		crain_onoff_flag = 0;
		OUT_CRAIN_PWM = 0;

		ucContolGrapTimer = 0;
		gucContolGrapDoneFlag = CLEAR;
	}
	

}
/////////////////////////////////////////////////////////////////////////


int main(void)
{
	
	io_init();
	//adc_init();
	motor_init();
	timer_init();

	//SetDCMCPWM(1,u16SpeedNormalVal,0);
	//Delay_ms(1000);
	//SetDCMCPWM(1,STOP,0);

	ControlMotorEnable_Xaxis(CLEAR);

	SetDCMCPWM(2,STOP,0);	// _test
	SetDCMCPWM(3,STOP,0);

	while(1)
	{

		SensingSwitch_Xaxis();
	
		SensingSwitch_Yaxis();
		ControlMotor_Yaxis();

		//SensingSwitch_Zaxis();
		GrapProgress();
		ControlMotor_Zaxis();

		SensingSwitch_Test();


/*
		if(_RB2 == 1)
		{
			u16debouncecnt_RB3 = 0;

			_RD0 = 1;
			for(i=0;i<=10;i++);

			_RD1 = 1;

			if(u16debouncecnt_RB2 >= 100)
			{
				
				_RE1 = 1;
				for(i=0;i<=STEP_MOTOR_PULSE_W ;i++);
				_RE1 = 0;
				for(i=0;i<=STEP_MOTOR_PULSE_W;i++);

			}
			else
			{
				u16debouncecnt_RB2++;
			}
		}
		else if(_RB3 == 1)
		{
			u16debouncecnt_RB2 = 0;
_RD0 = 1;for(i=0;i<=10;i++);
			_RD1 = 0;

			if(u16debouncecnt_RB3 >= 100)
			{
				_RE1 = 1;
				for(i=0;i<=STEP_MOTOR_PULSE_W;i++);
				_RE1 = 0;
				for(i=0;i<=STEP_MOTOR_PULSE_W;i++);

			}
			else
			{
				u16debouncecnt_RB3++;
			}
		}
		else
		{
_RD0 = 0;for(i=0;i<=10;i++);
			u16debouncecnt_RB2 = 0;
			u16debouncecnt_RB3 = 0;
			_RE1 = 0;
		}
*/		
		/*if(_RB2 == 1)
		{
			u16debouncecnt_RB3 = 0;
			if(u16debouncecnt_RB2 >= 100)
			{
				_RD1 = 1;
				SetDCMCPWM(1,u16SpeedNormalVal,0);
			}
			else
			{
				u16debouncecnt_RB2++;
			}
		}
		else if(_RB3 == 1)
		{
			u16debouncecnt_RB2 = 0;
			if(u16debouncecnt_RB3 >= 100)
			{
				_RD1 = 0;
				SetDCMCPWM(1,u16SpeedNormalVal,0);
			}
			else
			{
				u16debouncecnt_RB3++;
			}
		}
		else
		{
			u16debouncecnt_RB2 = 0;
			u16debouncecnt_RB3 = 0;
			SetDCMCPWM(1,STOP,0);
		}*/
	}
}

void io_init(void)
{
	 //3 RORTB를 ADC 이외 사용시 필수, Pin 별로 셋팅해야함 
	 // PORTB 는 초기화시 analog input으로 설정됨, digital input으로 설정 변경 (Programming mode시)
	_PCFG0 = 1; // RB0
	_PCFG1 = 1; // RB1
	_PCFG2 = 1;	// RB2
	_PCFG3 = 1;	// RB3
	_PCFG4 = 1;	// RB4
	_PCFG5 = 1;	// RB5
	
	TRISB = 0xFFFF;
	TRISC = 0xFFFF;
	
	TRISD = 0x0000; 
	TRISE = 0x0004;	//RE2 : Input
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
	SetDCMCPWM(3,0x0000,0);	// CH3, Duty 0, update안함(?)

	period = 0x7fff;	// 주기레지스터 총 15bit 
	sptime = 0x0000;	// special event, 샘플코드에서는 사용 안함 

	config1 = PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 & PWM_IPCLK_SCALE1 & PWM_MOD_FREE;	//  pwm enable, free running mode
	config2 = PWM_MOD1_IND & PWM_MOD2_IND & PWM_MOD3_IND & PWM_PDIS1L & PWM_PDIS1H & PWM_PDIS2L & PWM_PEN2H & PWM_PDIS3L & PWM_PEN3H;	// 1, 2, 3 CH만 사용가능 (H: pwm, L: IO)	// PWM_PDIS1H
	config3 = PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN;
	
	OpenMCPWM(period, sptime, config1, config2, config3);

	//SetDCMCPWM(2,STOP,0);	// 초기 motor 정지 	// _test
}

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

//////////////////////////////////////////////////////////////////
void timer_init()
{
/*	ConfigIntTimer1(T1_INT_PRIOR_7 &			// 0~7 => 최소 1 이상 설정할 것	
				T1_INT_ON);					// ON, OFF

	WriteTimer1(0);

	OpenTimer1(	T1_ON &						// ON, OFF
				T1_IDLE_STOP &					// CON, STOP
				T1_GATE_OFF &					// ON, OFF
				T1_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128
				T1_SYNC_EXT_OFF &  			// ON, OFF (ON 필수)
				T1_SOURCE_INT,	         	// EXT, INT
				1600);						// Match_value		// 100us 주기
	*/
	ConfigIntTimer3(T3_INT_PRIOR_3 &			// 0~7 => 최소 1 이상 설정할 것		// 190227 우선순위 7 로 설정
				T3_INT_ON);					// ON, OFF

	WriteTimer3(0);

	OpenTimer3(	T3_ON &						// ON, OFF
				T3_IDLE_STOP &					// CON, STOP
				T3_GATE_OFF &					// ON, OFF
				T3_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128

				T3_SOURCE_INT,	         	// EXT, INT
				16000);						// Match_value		// 1.0ms 주기		
				// 1 TICK * N
				// (1 / (8M * PLL 8)) / 4 Fix ) * 16000 N = 1ms
				// 0.0625us * 16000 = 1ms

}
//////////////////////////////////////////////////////////////////

void SensingSwitch_Xaxis(void)
{
	static unsigned int u16debouncecnt_RB2 = 0;
	static unsigned int u16debouncecnt_RB3 = 0;
 
	if(IN_RIGHT == 1)
	{
		u16debouncecnt_RB3 = 0;

		//if(guc_x_axis_tactsw == CLEAR)
		//{	
			if(u16debouncecnt_RB2 >= 100)
			{
				ControlMotorEnable_Xaxis(SET);	// ControlMotorEnable_Xaxis 위치 이동 검토 필요 _check
				DIR_X_AXIS_MOTOR = X_AISX_DIR_RIGHT;	// 순서 주의 (port 제어 후 delay 필요)
				gucContolMotor_XaxisFlag = SET;
			}
			else
			{
				u16debouncecnt_RB2++;
			}
		//}
		//else
		//{
		//}
	}
	else if(IN_LEFE == 1)
	{
		u16debouncecnt_RB2 = 0;

		if(guc_x_axis_tactsw == CLEAR)
		{
			if(u16debouncecnt_RB3 >= 100)
			{
				ControlMotorEnable_Xaxis(SET);
				DIR_X_AXIS_MOTOR = X_AISX_DIR_LEFT;		// 순서 주의 (port 제어 후 delay 필요)
				gucContolMotor_XaxisFlag = SET;
			}
			else
			{
				u16debouncecnt_RB3++;
			}
		}
		else
		{
			ControlMotorEnable_Xaxis(CLEAR);
		}
	}
	else
	{
		//ControlMotorEnable_Xaxis(CLEAR);

		u16debouncecnt_RB2 = 0;
		u16debouncecnt_RB3 = 0;
		
		gucContolMotor_XaxisFlag = CLEAR;
	}
}

void ControlMotorEnable_Xaxis(unsigned char command)
{
	unsigned char i = 0;

	#if 0
	if(command == 1)
	{	
		_RD0 = 1;
		for(i=0;i<=10;i++);
	}
	else
	{
		_RD0 = 0;
		for(i=0;i<=10;i++);
	}
	#endif 

	if(command == 1)
	{	
		ENABLE_X_AXIS_MOTOR = 0;
		for(i=0;i<=10;i++);
	}
	else
	{
		ENABLE_X_AXIS_MOTOR = 1;
		for(i=0;i<=10;i++);
	}

}

void ControlMotor_Xaxis(void)
{
	static unsigned int unPluseCnt = 0;

	unPluseCnt++;

	if((gucContolMotor_XaxisFlag == SET) || (gucContolMotor_GrapProcessXaxisFlag == SET))
	{
		if(unPluseCnt == 1)
		{
			X_AXIS_STEP_MOTOR_PWM = 1;
		}
		else
		{
			unPluseCnt = 0;
			X_AXIS_STEP_MOTOR_PWM = 0;
		}
	}
	else
	{
		X_AXIS_STEP_MOTOR_PWM = 0;
		unPluseCnt = 0;

		ControlMotorEnable_Xaxis(CLEAR);
	}
	
}

////////////////////////////////////////////////////////////////////////
void SensingSwitch_Yaxis(void)
{
	static unsigned int u16debouncecnt_up = 0;
	static unsigned int u16debouncecnt_down = 0;

	if(IN_UP == 1)
	{
		u16debouncecnt_down = 0;

		if(guc_y_axis_end_tactsw == CLEAR)
		{		
			if(u16debouncecnt_up >= 30)
			{
				DIR_Y_AXIS_MOTOR = 1;
				gucContolMotor_YaxisFlag = SET;
			}
			else
			{
				u16debouncecnt_up++;
			}
		}
		else
		{
			gunReal_cnt = 0;
			SetDCMCPWM(2,gunReal_cnt,0);

			gucContolMotor_YaxisFlag = CLEAR;
		}
	}
	else if(IN_DOWN == 1)
	{
		u16debouncecnt_up = 0;

		if(guc_y_axis_tactsw == CLEAR)
		{
			if(u16debouncecnt_down >= 30)
			{			
				DIR_Y_AXIS_MOTOR = 0;

				gucContolMotor_YaxisFlag = SET;
			}
			else
			{
				u16debouncecnt_down++;
			}
		}
		else
		{
			gunReal_cnt = 0;
			SetDCMCPWM(2,gunReal_cnt,0);

			gucContolMotor_YaxisFlag = CLEAR;
		}
	}
	else
	{
		u16debouncecnt_up = 0;
		u16debouncecnt_down = 0;
		
		gucContolMotor_YaxisFlag = CLEAR;
	}
}


void ControlMotor_Yaxis(void)
{
	static unsigned int cnt = 0;
	//static unsigned int gunReal_cnt = 0;
	unsigned int i = 0;

	if((gucContolMotor_YaxisFlag == SET) || (gucContolMotor_GrapProcessYaxisFlag == SET))
	{
		//SetDCMCPWM(2,u16SpeedNormalVal,0);
		//SetDCMCPWM(2,u16SpeedUpVal,0);

		if(gunReal_cnt <= (SPEED - SPEED_LOW_2))
		{
			gunReal_cnt++;
		}

		/*if(gunReal_cnt <= (SPEED - SPEED_LOW_2))
		{
			cnt++;
			i = cnt % 2;
			if(i == 0)
			{
				gunReal_cnt++;
			}
		}*/
		
		SetDCMCPWM(2,(SPEED_LOW_2 + gunReal_cnt),0);
	}
	else
	{
		if(gunReal_cnt != 0)
		{
			gunReal_cnt--;
		}

		/*if(gunReal_cnt != STOP)
		{
			cnt--;
			i = cnt % 2;
			if(i == 0)
			{
				gunReal_cnt--;
			}
		}*/
		
		//SetDCMCPWM(2,STOP,0);
		SetDCMCPWM(2,gunReal_cnt,0);
	}
}

void SensingSwitch_Zaxis(void)
{
	static unsigned int u16debouncecnt_up = 0;
	static unsigned int u16debouncecnt_down = 0;

	if(IN_CRAIN_GRAP == 1)
	{
		u16debouncecnt_down = 0;

		//DIR_Z_AXIS_MOTOR = 0;

		if(u16debouncecnt_up >= 30)
		{
			//gucContolMotor_ZaxisFlag = SET;
			gucGrapState = CRAINKING_DOWN;
		}
		else
		{
			u16debouncecnt_up++;
		}
	}
	else if(0)//IN_DOWN == 1)
	{
/*		u16debouncecnt_up = 0;

		DIR_Y_AXIS_MOTOR = 0;

		if(u16debouncecnt_down >= 30)
		{
			gucContolMotor_YaxisFlag = SET;
		}
		else
		{
			u16debouncecnt_down++;
		}*/
	}
	else
	{
		u16debouncecnt_up = 0;
		u16debouncecnt_down = 0;
		
		//gucContolMotor_ZaxisFlag = CLEAR;
	}
}

void ControlMotor_Zaxis(void)
{
	if(gucContolMotor_ZaxisFlag == SET)
	{
		SetDCMCPWM(3,SPEED_LOW_2,0);
		//SetDCMCPWM(3,0xefff,0);

		//SetDCMCPWM(1,0X6FFF,0);
		
		//OUT_CRAIN_PWM = ON;
	}
	else
	{
		SetDCMCPWM(3,STOP,0);

		//SetDCMCPWM(1,STOP,0);

		//OUT_CRAIN_PWM = OFF;
	}
	
}


void GrapProgress(void)
{
	static unsigned char ucUnitState = READY;
	
	switch(gucGrapState)
	{
		case INIT:
			ucUnitState = READY;
			SensingSwitch_Zaxis();

			guc_z_axis_tactsw = CLEAR;
		break;

		case CRAINKING_DOWN:
			DIR_Z_AXIS_MOTOR = 1;

			if(ucUnitState == READY)
			{
				ucUnitState = RUN;
				gucContolMotor_ZaxisFlag = SET;
				gucCrankingDownTimerFlag = SET;
			}
			else
			{
				if(gucCrankingDownTimerFlag == CLEAR)
				{
					ucUnitState = READY;
					gucContolMotor_ZaxisFlag = CLEAR;
					gucGrapState = GRAP;
				}
			}
			
		break;

		case GRAP:
			gucContolGrapFlag = SET;
			if(gucContolGrapDoneFlag == SET)
			{
				gucGrapState = CRAINKING_UP;
			}
		break;

		case CRAINKING_UP:
			DIR_Z_AXIS_MOTOR = 0;

			if(ucUnitState == READY)
			{
				ucUnitState = RUN;
				gucContolMotor_ZaxisFlag = SET;
			}
			else
			{
				if(guc_z_axis_tactsw == SET)
				{
					ucUnitState = READY;
					gucContolMotor_ZaxisFlag = CLEAR;
					gucGrapState = GO_HOME;
				}
			}
		break;

		case GO_HOME:
			if(ucUnitState == READY)
			{
				ucUnitState = RUN;

				ControlMotorEnable_Xaxis(SET);
				DIR_X_AXIS_MOTOR = X_AISX_DIR_LEFT;
				Delay_us(2);

				DIR_Y_AXIS_MOTOR = 0;
				Delay_us(2);
			
				gucContolMotor_GrapProcessXaxisFlag = SET;
				gucContolMotor_GrapProcessYaxisFlag = SET;
			}
			else
			{
				if(guc_x_axis_tactsw == SET)
				{
					gucContolMotor_GrapProcessXaxisFlag = CLEAR;
					ControlMotorEnable_Xaxis(CLEAR);
				}
				if(guc_y_axis_tactsw == SET)
				{
					gucContolMotor_GrapProcessYaxisFlag = CLEAR;
					
					gunReal_cnt = 0;
					SetDCMCPWM(2,gunReal_cnt,0);
				}
				if((gucContolMotor_GrapProcessXaxisFlag == CLEAR) & (gucContolMotor_GrapProcessYaxisFlag == CLEAR))
				{
					ucUnitState = READY;
					gucGrapState = DROP;
				}
			}
		break;

		case DROP:
			gucContolGrapFlag = CLEAR;
			gucGrapState = INIT;
		break;

		case FINISH:
		break;

		default:
		break;
	}

}

void CheckCrankingDownTimer(void)
{
	static char ucCrankingDownTimer = 0;
	
	if(gucCrankingDownTimerFlag == SET)
	{
		ucCrankingDownTimer++;
		if(ucCrankingDownTimer >= CRAINKING_DROP_TIME)	// [100ms]
		{
			ucCrankingDownTimer = 0;
			gucCrankingDownTimerFlag = CLEAR;
		}
	}
}

void SensingSwitch_Test(void)
{
	static unsigned int u16debouncecnt = 0;
	static unsigned char ucState = READY;

	if(IN_TEST == 1)
	{
		DIR_Z_AXIS_MOTOR = 0;

		if(u16debouncecnt >= 30)
		{
			ucState == RUN;
			SetDCMCPWM(3,SPEED_LOW_2,0);
		}
		else
		{
			u16debouncecnt++;
		}
	}
	else
	{
		u16debouncecnt = 0;	
		if(ucState == RUN)
		{
			ucState = READY;
			SetDCMCPWM(3,STOP,0);
		}
		else
		{
		}
	}
}

void SensingTactsw(void)
{
	if(IN_Z_AXIS_TACTSW == ON)
	{
		guc_z_axis_tactsw = SET;
	}
	else
	{
		guc_z_axis_tactsw = CLEAR;
	}

	if(IN_TACTSW_X_AXIS_ORIGIN == ON)
	{
		guc_x_axis_tactsw = SET;

		// _test
		/*if(gucGrapState == GO_HOME)
		{
			gucContolMotor_GrapProcessXaxisFlag = CLEAR;
			ControlMotorEnable_Xaxis(CLEAR);
		}*/
	}
	else
	{
		guc_x_axis_tactsw = CLEAR;
	}

	if(IN_TACTSW_Y_AXIS_ORIGIN == ON)
	{
		guc_y_axis_tactsw = SET;
	}
	else
	{
		guc_y_axis_tactsw = CLEAR;
	}

	if(IN_TACTSW_Y_AXIS_END == ON)
	{
		guc_y_axis_end_tactsw = SET;
	}
	else
	{
		guc_y_axis_end_tactsw = CLEAR;
	}
}

