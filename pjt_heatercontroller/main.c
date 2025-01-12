//////////////////////////////////////////////////////////////////////////////////////

// adc buf size long으로 변경, 12bit adc 경우 overflow됨 

//////////////////////////////////////////////////////////////////////////////////////


#include <p30f3013.h>
#include <stdio.h>
#include <timer.h>
#include <adc12.h>

#include "delay.h"
#include "define.h"



// 사용안할 시 내부 osc로 설정됨, IDE>Configure 에서 확인가능 
_FOSC(CSW_FSCM_OFF & ECIO_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);      

//#define FCY  8000000 
//#define BAUDRATE 	19200
#define FCY  16000000        
#define BAUDRATE 	9600        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1           

//3 PORTB는 Output으로 사용시 PB(RB)가 아닌 LATB로 사용할 것 
//ex. #define LED_RED		_LATB3

#define OUT_LED_YELLOW_1	_LATB6
#define OUT_LED_YELLOW_2	_LATB7
#define OUT_LED_RED			_LATB8
#define OUT_LED_GREEN		_LATB9

#define OUT_BUZZER	_RD8
#define OUT_RLY		_RD9

///////////////////////////////
#define REF_R2	(10000)	// ohm

#define REF_X1	(5100)	// ohm
#define REF_X2	(10300)	// ohm
#define REF_Y1	(39.6)	// degree
#define REF_Y2	(25.0)	// degree
///////////////////////////////

#define CAL_TEMP_LOW	(2)	// 펌프 보호를 위해 MAX Target temp를 38도 로 제한함 


//////////////////////////////////////////////////////////////////
void io_init(void);
void timer_init(void);
void adc_init(void);

void SetTargetTemp(void);
void CalcTemp(void);
void CtrlHeaterRly(void);
void CheckHeating(void);
void CtrlHeating(void);
void CtrlReHeating(void);
unsigned int GetTargetTemp(void);
void CalcAlphaBeta(void);

//////////////////////////////////////////////////////////////////

enum
{
	LOW,
	MIDDLE,
	HIGH
}eTempDef;

enum
{
	INIT,
	HEATING,
	RE_HEATING,
	DONE
}eHeatingStateDef;


// Timer 기본변수
unsigned int gu16_timer_cnt, gu16_timer_1sec = 0;

// ADC 기본변수
int adc_save_count=0;
long buf[2]={0}, adc_sum[2]={0};
long adc_average_sum[2]={0};

unsigned int gu16temp_sw, gu16temp_sensor = 0;
unsigned char gu8TargetTemp = 0;

float ftemp_volt = 0;

float fR1_sensor, falpha, fbeta = 0;
unsigned int gu16TempFinal = 0;

unsigned char gu8SetAlramShort, gu8SetAlramLong, gu8SetAlramMid = 0;

unsigned char gu8HeaterStatus, gu8HeatingState = 0;


//////////////////////////////////////////////////////////////////

// 1ms Timer
void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	static unsigned int su16BuzzerCnt = 0;
	
	WriteTimer3(0);

	IFS0bits.T3IF=0;

	gu16_timer_cnt++;
		
	if(gu16_timer_cnt >= 1000)
	{
		gu16_timer_cnt = 0;

		gu16_timer_1sec++;
		
		OUT_LED_YELLOW_1 = ~OUT_LED_YELLOW_1;
		//OUT_BUZZER = ~OUT_BUZZER;
	}

	if(gu16_timer_1sec >= 1)
	{
		CtrlHeaterRly();
	}

	// buzzer operation
	if(gu8SetAlramLong == SET)
	{
		su16BuzzerCnt++;
		OUT_BUZZER = ON;

		if(su16BuzzerCnt >= 1000*5)
		{
			su16BuzzerCnt = 0;
			OUT_BUZZER = OFF;

			gu8SetAlramLong = CLEAR;
		}
	}
	else if(gu8SetAlramMid == SET)
	{
		su16BuzzerCnt++;
		OUT_BUZZER = ON;

		if(su16BuzzerCnt >= 1000*2)
		{
			su16BuzzerCnt = 0;
			OUT_BUZZER = OFF;

			gu8SetAlramMid = CLEAR;
		}
	}
	else if(gu8SetAlramShort == SET)
	{
		su16BuzzerCnt++;
		OUT_BUZZER = ON;

		if(su16BuzzerCnt >= 500)
		{
			su16BuzzerCnt = 0;
			OUT_BUZZER = OFF;

			gu8SetAlramShort = CLEAR;
		}
	}
	else
	{
		OUT_BUZZER = OFF;
	}

}

void __attribute__ ((__interrupt__)) _ADCInterrupt(void)
{

	buf[0] = ReadADC12(0);
	buf[1] = ReadADC12(1);
	
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

	gu16temp_sensor = (unsigned int)adc_average_sum[0];
	gu16temp_sw = (unsigned int)adc_average_sum[1];

	IFS0bits.ADIF=0;

	//_RD0 = 1;
}

int main(void)
{

	io_init();
	timer_init();
	adc_init();

	CalcAlphaBeta();
	
	while(1)
	{
		SetTargetTemp();

		CalcTemp();

		// CtrlHeaterRly() : 1초주기로 인터럽트에서 실행 
	}
}

//////////////////////////////////////////////////////////////////
void io_init(void)
{
	//3 방향설정  
	// LED
	_TRISB6 = 0;
	_TRISB7 = 0;
	_TRISB8 = 0;
	_TRISB9 = 0;

	// BUZZER
	_TRISD8 = 0;

	// RLY
	_TRISD9 = 0;

	// ADC
	_TRISB0 = 1;
	_TRISB1 = 1;


	//3 PORT 값 초기화
	OUT_BUZZER = 0;
	OUT_RLY = 0;
}

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
				8000);						// Match_value		// 1.0ms 주기		
				// (8M * PLL 4) / 4 Fix ) * 8000 N = 1ms
				// 0.125us * 8000 = 1ms

}

void adc_init()
{
//*************************************** A/D Conversion Initilization **********************************************
SetChanADC12(ADC_CH0_POS_SAMPLEA_AN0 &		// ADC_CH0_POS_SAMPLEA_AN0, ADC_CHX_POS_SAMPLEA_AN0AN1AN2
			ADC_CH0_POS_SAMPLEA_AN1&
			ADC_CH0_POS_SAMPLEA_AN2&
			ADC_CH0_NEG_SAMPLEA_NVREF);

ConfigIntADC12(ADC_INT_ENABLE & 		// ENABLE, DISABLE
			ADC_INT_PRI_6); 			// 0~7

OpenADC12(	ADC_MODULE_ON & 				// ON, OFF						// Module On/Off
			ADC_IDLE_CONTINUE & 			// IDLE_STOP, IDLE_CONTINUE		// Idle mode operation
			ADC_FORMAT_INTG &			// SIGN_FRACT, FRACT, SIGN_INT, INTG		// Result output format
			ADC_CLK_AUTO &				// AUTO, MPWM, TMR, INT0, MANUAL		// Conversion trigger source
			ADC_AUTO_SAMPLING_ON &		// ON, OFF							// Auto sampling select
			ADC_SAMP_ON,					// ON, OFF 						// Sample enable

			ADC_VREF_AVDD_AVSS &			// AVDD_AVSS, EXT_AVSS, AVDD_EXT, EXT_EXT	// Voltage Reference
			ADC_SCAN_ON &					// ON, OFF								// Scan selection
			ADC_SAMPLES_PER_INT_3 & 	// 1~16			// Number of samples between interrupts
			ADC_ALT_BUF_OFF &				// ON, OFF	// Buffer mode select
			ADC_ALT_INPUT_OFF,			// ON, OFF		// Alternate Input Sample mode select

			ADC_SAMPLE_TIME_0 & 			// 0~31					// Auto Sample Time bits
			ADC_CONV_CLK_SYSTEM &			// INTERNAL_RC, SYSTEM	// Conversion Clock Source select
			ADC_CONV_CLK_16Tcy, 		// Tcy2, Tcy, 3Tcy2~32Tcy		// Conversion clock select

			ENABLE_AN0_ANA &				// ALL_ANA, ALL_DIG, AN0_ANA~AN15_ANA_ANA
			ENABLE_AN1_ANA,
			//ENABLE_AN2_ANA,

			SCAN_ALL);					// NONE, ALL, SKIP_SCAN_AN0~15

}

void SetTargetTemp(void)
{
	unsigned char static su8TargetTempBuf = 0;
	
	if(gu16temp_sw >= 4096-1500)
	{
		OUT_LED_YELLOW_1 = CLEAR;
		OUT_LED_YELLOW_2 = CLEAR;
		OUT_LED_RED = SET;

		gu8TargetTemp = HIGH;
	}
	else if(gu16temp_sw >= 4096-3000)
	{
		OUT_LED_YELLOW_1 = CLEAR;
		OUT_LED_YELLOW_2 = SET;
		OUT_LED_RED = CLEAR;

		gu8TargetTemp = MIDDLE;
	}
	else
	{
		OUT_LED_YELLOW_1 = SET;
		OUT_LED_YELLOW_2 = CLEAR;
		OUT_LED_RED = CLEAR;

		gu8TargetTemp = LOW;
	}

	if(su8TargetTempBuf != gu8TargetTemp)
	{
		gu8SetAlramShort = SET;
		
		su8TargetTempBuf = gu8TargetTemp;
	}
}

void CalcTemp(void)
{
	ftemp_volt = ((float)gu16temp_sensor / 4096) * 5.0;
	if(ftemp_volt >= 4.99)
	{
		ftemp_volt = 4.99;
	}

	//fR1_sensor = (ftemp_volt * REF_R2 ) / (5.0 - ftemp_volt);	// ohm
	fR1_sensor = ((float)REF_R2 * (5.0 - ftemp_volt)) / ftemp_volt;	// ohm

	gu16TempFinal = (unsigned int)(falpha * fR1_sensor + fbeta);
}

void CtrlHeaterRly(void)
{
	switch(gu8HeatingState)
	{
		case INIT:
			CheckHeating();
		break;

		case HEATING:
			CtrlHeating();
		break;

		case RE_HEATING:
			CtrlReHeating();
		break;

		case DONE:
		break;

		default:
		break;
	}	
}

// target temp 이하가 되면 heating을 시작 
void CheckHeating(void)
{
	static unsigned char u8DebounceCnt = 0;
	
	if(gu16TempFinal <= GetTargetTemp())	// heating
	{
		if(u8DebounceCnt >= 5)
		{
			u8DebounceCnt = 0;
						
			gu8HeaterStatus = ON;
			gu8SetAlramMid = SET;
			gu8HeatingState = HEATING;
			
			// rly on
			OUT_RLY = SET;
			OUT_LED_GREEN = CLEAR;

		}
		else
		{
			u8DebounceCnt++;
		}
	}
	else
	{
		u8DebounceCnt = 0;
	
		// rly off
		OUT_RLY = CLEAR;
		OUT_LED_GREEN = SET;
	}
}

// heating 시작 이 후 target temp대비 3도 이상 가열되면 heating off, reheating 위한 준비모드로 진입 
void CtrlHeating(void)
{
	static unsigned char u8DebounceCnt = 0;
	
	if(gu16TempFinal >= GetTargetTemp() + 2)
	{
		if(u8DebounceCnt >= 5)
		{
			u8DebounceCnt = 0;
						
			gu8HeaterStatus = OFF;
			gu8HeatingState = RE_HEATING;
			
			// rly off
			OUT_RLY = CLEAR;
			OUT_LED_GREEN = SET;
			gu8SetAlramLong = SET;	// 알람 발생 
		}
		else
		{
			u8DebounceCnt++;
		}
	}
	else
	{
		u8DebounceCnt = 0;
	}
}

// 초기 gu8HeaterStatus = OFF 상태로 진입함, 현재 온도가 target temp 대비 5도 이하로 하강하면 재히팅 시작  
// 재히팅 이후 현재온도가 target temp 대비 3도 이상 가열되면 heater off, 재히팅 준비상태로 되돌아감 
// (반복)
void CtrlReHeating(void)
{
	static unsigned char u8DebounceCnt, u8RlyOffCnt = 0;

	if(gu8HeaterStatus == OFF)
	{
		if(gu16TempFinal + 5 <= GetTargetTemp())	// 현재 temp + 5도 이상부터 heating
		{
			if(u8DebounceCnt >= 5)
			{
				u8DebounceCnt = 0;
				gu8HeaterStatus = ON;
				gu8SetAlramMid = SET;

				// rly on
				OUT_RLY = SET;
				OUT_LED_GREEN = CLEAR;
			}
			else
			{
				u8DebounceCnt++;
			}
		}
		else
		{
			u8DebounceCnt = 0;
		}
	}
	else
	{
		if(gu16TempFinal >= GetTargetTemp() + 2)
		{
			if(u8RlyOffCnt >= 5)
			{
				u8RlyOffCnt = 0;				
				gu8HeaterStatus = OFF;
				
				// rly off
				OUT_RLY = CLEAR;
				OUT_LED_GREEN = SET;
				gu8SetAlramShort = SET;
			}
			else
			{
				u8RlyOffCnt++;
			}
		}
		else
		{
			u8RlyOffCnt = 0;
		}
	}
}


unsigned int GetTargetTemp(void)
{
	unsigned int u16RetVal = 0;
	
	switch(gu8TargetTemp)
	{
		case LOW:
			u16RetVal = 20;
		break;

		case MIDDLE:
			u16RetVal = 30;
		break;

		case HIGH:
			u16RetVal = 40 - CAL_TEMP_LOW;
		break;

		default:
		break;
			
	}
	
	return(u16RetVal);
}

void CalcAlphaBeta(void)
{
	falpha = (float)((float)(REF_Y2 - REF_Y1 ) / (REF_X2 - REF_X1));
	fbeta = (float)((-1.0) * falpha * REF_X1 + REF_Y1);
}
