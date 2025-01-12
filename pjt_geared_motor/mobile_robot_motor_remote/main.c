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

// 180224 
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



#include <p30f4012.h>
#include <stdio.h>
#include <pwm.h>
#include <adc10.h>

_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);	// FRC로 PWM동작 할 것으로 예상되나 확인 필요, 안전하게 EC_PLL8로 사용함 
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

/*
_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF & WDTPSA_64 & WDTPSB_4);
_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_20 & PWMxH_ACT_HI & PWMxL_ACT_HI);
_FGS(CODE_PROT_OFF);
*/

#define ACTIVE_LOW_ENALBE_1	LATEbits.LATE4	// PWM CH3을 Disable로 설정한 후 사용 
#define DIR_1				LATEbits.LATE5

#define ACTIVE_LOW_ENALBE_2	LATDbits.LATD0
#define DIR_2				LATDbits.LATD1

//3 Control type 결정
#define REMOTE_CONTROL		(1)

#ifdef MANUAL_CONTROLL	// Switch type 사용시 
#define FORWARD				PORTCbits.RC15	// PORTB는 디버깅시에만 INPUT사용 가능 이력, PORTC로 변경함 
#define BACK				PORTCbits.RC13
#define RIGHT				PORTCbits.RC14
#define LEFT				PORTEbits.RE8
#elif REMOTE_CONTROL
#define NO_OPERATION		(0)
#define FORWARD				(1)
#define BACK				(2)
#define RIGHT				(3)
#define LEFT				(4)
#else
// "error"
#endif

#define STOP				(0x0000)
#define SPEED				(0x5fff)
#define SPEED_UP			(SPEED + 0x2000)
#define SPEED_MAX			(0x7fff)
#define SPEED_MIN			(0x3fff)	// remote controll 방향 인식시 발생하는 pwm min값

#define CH_VOLT_MAX			(440)	// remote controll 방향 인식시 발생하는 max voltage [mv]
#define CH_VOLT_REF			(335)	// min voltage [mv]
#define CH_VOLT_MIN			(230)	// min voltage [mv]
#define CH_VOLT_HYSERESIS	(30)


void motor_init(void);  
void io_init(void);
void adc_init(void); 

int i,j;
unsigned char system_on = 0;
unsigned int booting_delay_cnt = 0;

int adc_save_count=0;
int buf[2]={0}, adc_sum[2]={0};
long adc_average_sum[2]={0};

int adc_voltage_result[2]={0};
unsigned char dir_ch1, dir_ch2=0;

int Coeff = 0;
int ch_pwm[2] = {0};

void motor_init();
void io_init(void);
void adc_init();

void RunDetectDir(void);
void RunMotor(void);

///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

void __attribute__ ((__interrupt__)) _ADCInterrupt(void)
{

	buf[0] = ReadADC10(0);	// Remote controller CH1
	buf[1] = ReadADC10(1);	// Remote controller CH2
	
	adc_save_count++;
	adc_sum[0] += buf[0];
	adc_sum[1] += buf[1];
	
	if(adc_save_count == 10)
	{
		adc_average_sum[0] = adc_sum[0]/10;
		adc_sum[0] = 0;

		adc_average_sum[1] = adc_sum[1]/10;
		adc_sum[1] = 0;
		
		adc_save_count = 0;
	}
	
	IFS0bits.ADIF=0;
	
}

int main(void)
{

	io_init();
	motor_init();
	adc_init();

	// Application Initialization
	// Coeff = (0x7ffff-0x3fff) / (440mv - 230mv)
	//                = 80 (gain)
	Coeff = (int)((long)SPEED_MAX - SPEED_MIN) / ((long)CH_VOLT_MAX - CH_VOLT_MIN);
	
	do
	{
		RunDetectDir();
		
		//if((FORWARD == 0) && (BACK == 0))	// FORWARD와 BACKWORD를 동시에 누르면 SYSTEM ON함 
		//if((FORWARD == 0) && (RIGHT == 0))	// FORWARD와 RIGHT를 동시에 누르면 SYSTEM ON함 
		if((dir_ch2 == FORWARD) && (dir_ch1 == RIGHT))	// FORWARD와 RIGHT를 동시에 누르면 SYSTEM ON함 
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

		RunDetectDir();

		RunMotor();
		
	}
}

void motor_init()
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
	config2 = PWM_MOD1_COMP & PWM_MOD2_COMP & PWM_PEN1L & PWM_PEN1H & PWM_PEN2L & PWM_PEN2H & PWM_PDIS3L & PWM_PDIS3H;	// 1, 2 CH 사용가능 
	config3 = PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN;
	
	OpenMCPWM(period, sptime, config1, config2, config3);
}

void io_init(void)
{

	TRISB = 0x003C;
	TRISC = 0xFFFF;
	TRISD = 0x0000;
	TRISE = 0x0100;

	// MANUAL_CONTROLL 시에만 활성화 
	//FORWARD = 1;
	//BACK = 1;
	//RIGHT = 1;
	//LEFT = 1;

	ACTIVE_LOW_ENALBE_1 = 1;
	DIR_1 = 1;

	ACTIVE_LOW_ENALBE_2 = 1;
	DIR_2 = 0;
}

void adc_init()
{
//*************************************** A/D Conversion Initilization **********************************************
// ADC Channel 선택 
SetChanADC10(ADC_CH0_POS_SAMPLEA_AN0 &	// ADC_CH0_POS_SAMPLEA_AN0, ADC_CHX_POS_SAMPLEA_AN0AN1AN2
			ADC_CH0_POS_SAMPLEA_AN1 &
			//ADC_CH0_POS_SAMPLEA_AN2&
			ADC_CH0_NEG_SAMPLEA_NVREF);	// POS, NEG 같이 ADC해야 함, 설정없으면 ADC 안됨 

ConfigIntADC10(ADC_INT_ENABLE & 		// ENABLE, DISABLE
			ADC_INT_PRI_6); 			// 0~7

OpenADC10(	ADC_MODULE_ON & 			// ON, OFF
			ADC_IDLE_CONTINUE & 		// IDLE_STOP, IDLE_CONTINUE
			ADC_FORMAT_INTG &			// SIGN_FRACT, FRACT, SIGN_INT, INTG
			ADC_CLK_AUTO &				// AUTO, MPWM, TMR, INT0, MANUAL
			ADC_AUTO_SAMPLING_ON &		// ON, OFF
			ADC_SAMPLE_SIMULTANEOUS &	// SIMULTANEOUS, INDIVIDUAL
			ADC_SAMP_ON,				// ON, OFF 

			ADC_VREF_AVDD_AVSS &		// AVDD_AVSS, EXT_AVSS, AVDD_EXT, EXT_EXT	// Reference voltage 선택 
			ADC_SCAN_ON &				// ON, OFF
			ADC_CONVERT_CH0 &			// CH0, CH_0A, CH_0ABC
			//ADC_SAMPLES_PER_INT_3 & 	// 1~16		// 샘플링 채널 대상 선택 : AN 2 PIN 사용 
			ADC_SAMPLES_PER_INT_2 &
			ADC_ALT_BUF_OFF &			// ON, OFF
			ADC_ALT_INPUT_OFF,			// ON, OFF

			ADC_SAMPLE_TIME_0 & 		// 0~31		// Auto-sample time bits [SAMC]
			ADC_CONV_CLK_SYSTEM &		// INTERNAL_RC, SYSTEM	// ADC Clock [ADCS]
			ADC_CONV_CLK_16Tcy, 		// Tcy2, Tcy, 3Tcy2~32Tcy		// Conversion clock select bits

			ENABLE_AN0_ANA &			// ALL_ANA, ALL_DIG, AN0_ANA~AN15_ANA_ANA	// ADC Channal Input pin 선택 
			ENABLE_AN1_ANA,
			//ENABLE_AN2_ANA,*/		// AN 2 PIN 사용

			SCAN_ALL);					// NONE, ALL, SKIP_SCAN_AN0~15	// scan 범위 설정 

}

void RunDetectDir(void)
{
	// 기준전압 : 5V, ADC :10bit
	adc_voltage_result[0] = ((long)5000 * adc_average_sum[0])/1024;	// [mV] remote ch1 
	adc_voltage_result[1] = ((long)5000 * adc_average_sum[1])/1024;	// [mV] remote ch2

	// FORWARD, BACKWARD
	if(adc_voltage_result[1] <= CH_VOLT_REF - CH_VOLT_HYSERESIS)
	{
		dir_ch2 = FORWARD;
	}
	else if(adc_voltage_result[1] > CH_VOLT_REF + CH_VOLT_HYSERESIS)
	{
		dir_ch2 = BACK;
	}
	else
	{
		dir_ch2 = NO_OPERATION;
		adc_voltage_result[1] = 0;
	}

	// RIGHT, LEFT
	if(adc_voltage_result[0] <= CH_VOLT_REF - CH_VOLT_HYSERESIS)
	{
		dir_ch1 = LEFT;
	}
	else if(adc_voltage_result[0] > CH_VOLT_REF + CH_VOLT_HYSERESIS)
	{
		dir_ch1 = RIGHT;
	}
	else
	{
		dir_ch1 = NO_OPERATION;
		adc_voltage_result[0] = 0;
	}
}

void RunMotor(void)
{
/*	
	// 10 : RIGHT
	// 01: LEFT
	// 00: FORWARD
	// 11: BACKWORD
	if(FORWARD == 0)
	//if(dir_ch2 == FORWARD)
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
*/	

	ch_pwm[0] = adc_voltage_result[1]*Coeff;	// remote ch1
	ch_pwm[1] = adc_voltage_result[2]*Coeff;	// remote ch2

	if(ch_pwm[0] >= 0x7fff)
	{
		ch_pwm[0] = 0x7fff;
	}
	if(ch_pwm[1] >= 0x7fff)
	{
		ch_pwm[1] = 0x7fff;
	}
		
	// 10 : RIGHT
	// 01: LEFT
	// 00: FORWARD
	// 11: BACKWORD
	if(dir_ch2 == FORWARD)
	{
		DIR_1 = 1;
		DIR_2 = 0;

		if(dir_ch1 == RIGHT)
		{
			SetDCMCPWM(1,ch_pwm[1],0);	// pwm1 : right wheel
			SetDCMCPWM(2,ch_pwm[0],0);	// pwm2 : left wheel
		}
		else if(dir_ch1 == LEFT)
		{
			SetDCMCPWM(1,ch_pwm[0],0);
			SetDCMCPWM(2,ch_pwm[1],0);
		}
		else
		{
			SetDCMCPWM(1,ch_pwm[1],0);
			SetDCMCPWM(2,ch_pwm[1],0);
		}
	}
	else if(dir_ch2 == BACK)
	{
		DIR_1 = 0;	
		DIR_2 = 1;

		if(dir_ch1 == RIGHT)
		{
			SetDCMCPWM(1,ch_pwm[0],0);
			SetDCMCPWM(2,ch_pwm[1],0);
		}
		else if(dir_ch1 == LEFT)
		{
			SetDCMCPWM(1,ch_pwm[1],0);
			SetDCMCPWM(2,ch_pwm[0],0);
		}
		else
		{
			SetDCMCPWM(1,ch_pwm[0],0);
			SetDCMCPWM(2,ch_pwm[0],0);
		}
	}
	else if(dir_ch1 == RIGHT)
	{
		DIR_1 = 1;
		DIR_2 = 1;
		
		SetDCMCPWM(1,ch_pwm[0],0);
		SetDCMCPWM(2,ch_pwm[0],0);
	}
	else if(LEFT == 0)
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
