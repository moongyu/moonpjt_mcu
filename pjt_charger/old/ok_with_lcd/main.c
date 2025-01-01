//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//180224 ADC AN01, AN02를 ADC하는 Basic code 
// 설정 채널이 모두 ADC완료되면 인터럽트 발생
// 20회 인터럽트 후 취득값 평균값 처리 

// 주요 레지스터
// SMPI : Sample/Convert Sequences Per Interrupt Selection bits
// CHPS : Selects Channels Utilized bits
// PCFG : Analog Input Pin Configuration Control bits
// CSSL : A/D Input Pin Scan Selection bits
// ADRC : A/D Conversion Clock Source bit
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



#include <p30f4012.h>
#include <stdio.h>
#include <adc10.h>
#include <timer.h>

#include "cLCD.h"

_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);	// 스펙상 FRC 사용시 ADRC 설정 변경  
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

/*
_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF & WDTPSA_64 & WDTPSB_4);
_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_20 & PWMxH_ACT_HI & PWMxL_ACT_HI);
_FGS(CODE_PROT_OFF);
*/

// 사용안함 CSSL로 ADC Port analog input설정만 해주면 됨 (TRISB 설정도 불필요)
#define PB02				_RB2	
#define PB03				_RB3	

//============================================================
#define GAIN				(1.012)	//(1.082)
#define OFFSET				(134.4)	//(207.5)

#define GAIN_BAT			(1.0967) //(1.1027)
#define OFFSET_BAT			(-82.14) //(-25.48)

#define RATIO_REG			(5.0)	//(1.0/(100.0/500.0));0
#define RATIO_REG_BAT		(1.0/(101.0/489.0))
#define RATIO_CONV			(0.005 * 100)

#define VREF_MV				(5000)
#define VREF_LSB			(1024)

#define CUTOFF_PACKVOL			(13000) //(13500)	//(13000)	//(12800)	// _dev2
#define OVP_TH				(15000) //(14500)	//(14000) //(13500)		// _dev2

#define CHECK_FCC_PERIOD		(60*10)	//(60*30)	// [sec]	// _dev
#define FCC_HOLD_TIME			(15)	// [sec]
//============================================================

#define ADC_CH_NUM				(3)
#define CAPACITY_SUM_UINT		(3600)	// 60 * 60 = 3600 [AH]
#define DARK_CURRENT			(0.01)

void adc_init(void);  
void io_init(void);
void timer_init(void);


//============================================================
void ControlFet(void);
void RecordingMinMax(void);
void CalcPackvol(void);

void ControlFetNormal(void);
//============================================================
unsigned char dec2ascii(unsigned char);
void report_val_conv(void);
unsigned char float2string100(float);
unsigned char float2string10(float);
unsigned char float2string1(float);
unsigned char float2string0_1(float);
void ClacWatt(void);

//============================================================	


int i,adc_save_count=0;
int buf[ADC_CH_NUM]={0}, adc_sum[ADC_CH_NUM]={0};
long adc_average_sum[ADC_CH_NUM]={0};

int test01=0;

//============================================================
unsigned int vref_mv, vref_lsb = 0;
unsigned int adc_done_flag = 0;

long cnt_on, cnt_off = 0;

float ratio_reg;

float adc_result_source_volt, packvol_convert = 0.0, packvol_convert_cali = 0.0;
float adc_result_bat, packvol_convert_bat= 0.0, packvol_convert_cali_bat = 0.0;
float adc_result_curr, packvol_convert_curr= 0.0, packvol_convert_cali_curr = 0.0;

float packvol_convert_min = 20000.0, packvol_convert_max = 0.0;
//============================================================

unsigned char ucTimerOVP, ucTimerHoldLed = 0;
unsigned int unFccHoldCnt, unFccTimer = 0;
unsigned char ucCheckFccFlag = 0;
//============================================================

char buf_char[16];
char buf_char_volt[5];
char buf_char_curr[4];
char buf_char_curr_dir[2];
char buf_char_watt_total[4];
char buf_char_watt_day[4];

float sum_watt_1day, sum_watt_total = 0.0;
unsigned int ClacWattCnt = 0;
unsigned char update_report_flag = 0;

//============================================================

void __attribute__ ((__interrupt__)) _ADCInterrupt(void)
{
	buf[0] = ReadADC10(0);
	buf[1] = ReadADC10(1);
	buf[2] = ReadADC10(2);
	
	adc_save_count++;
	adc_sum[0] += buf[0];
	adc_sum[1] += buf[1];
	adc_sum[2] += buf[2];
	
	if(adc_save_count == 10)
	{
		adc_average_sum[0] = adc_sum[0]/10;
		adc_sum[0] = 0;

		adc_average_sum[1] = adc_sum[1]/10;
		adc_sum[1] = 0;

		adc_average_sum[2] = adc_sum[2]/10;
		adc_sum[2] = 0;
		
		adc_save_count = 0;

		adc_done_flag = 1;
		//_RD1 = !_RD1;		400us마다 진입 
	}
	
	IFS0bits.ADIF=0;

	//_RD1 = 1;
}

void __attribute__((__interrupt__)) _T1Interrupt(void)
{
	WriteTimer1(0);

	//timer_value=ReadTimer1();	// watch로 확인안됨 
	//timer_value=TMR1;			// watch로 확인안됨 
	
	IFS0bits.T1IF=0;

	//_RD0 = 1;
}

void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	static unsigned int unBaseTime = 0;
	static unsigned int unBaseTimeFcc = 0;
	static unsigned int unBaseTimeHoldLed = 0;

	WriteTimer3(0);

	IFS0bits.T3IF=0;

	//_RD0 = 1;

	unBaseTime++;
	unBaseTimeFcc++;
	unBaseTimeHoldLed++;

	if(unBaseTime >= 500)
	{
		unBaseTime = 0;

		if(ucTimerOVP == 0)
		{
			ucTimerOVP = 1;
		}
		else
		{
			ucTimerOVP = 0;
		}
	}

	if(unBaseTimeHoldLed >= 250)
	{
		unBaseTimeHoldLed = 0;

		if(ucTimerHoldLed == 0)
		{
			ucTimerHoldLed = 1;
		}
		else
		{
			ucTimerHoldLed = 0;
		}

		update_report_flag = 1;
	}

	// fcc check 기능
	// 일정 전압 이상이면 fcc period만큼 fet off 후 재동작함
	if(unBaseTimeFcc >= 600)	// 약 1초 
	{
		unBaseTimeFcc = 0;
		unFccHoldCnt++;	// timer용 

		if(packvol_convert_cali_bat >= CUTOFF_PACKVOL+200)	// 오차 감안하여 더 진입 어려운 조건으로 설정
		//if(packvol_convert_cali_bat >= 13000)	// _dev2
		{
			unFccTimer++;
			
			if(unFccTimer >= CHECK_FCC_PERIOD)
			{
				ucCheckFccFlag = 1;
				unFccTimer = 0;	// CHECK_FCC_PERIOD 주기로 진입 가능, unFccHoldCnt TH보다 작아야 함  

				unFccHoldCnt = 0;	// reload
			}
		}
		else
		{
			unFccTimer = 0;
		}

		ClacWatt();
	}
}

float test_a = 9.1;
float test_b = 14.2;
float test_c = 0.3;
float test_d = 0.0;
int  test_e = 411;

int main(void)
{
	io_init();
	adc_init();
	timer_init();
	InitLCD();

	ratio_reg = 5.0;		//(1.0/(100.0/500.0));
	vref_mv = 5000;
	vref_lsb = 1024;

	HomeClearLCD();

	//Wrt_S_LCD("0123456789ABCDEF", 0, 0);	// Copy 시 주의할 것 (ex. blink)
	//Wrt_S_LCD("FEDCBA9876543210", 0, 1);

	//Wrt_S_LCD("BOOTING", 0, 0);

	Wrt_S_LCD("00.0[V] 0.0[A]-C", 0, 0);
	Wrt_S_LCD("0.0[KW] 000[W]/D", 0, 1);

	while(1)
	{	
		test01++;

		//Wait(1000);	// 1ms 소요 확인 
		//Wait(10000);	// 10ms 소요 확인 

		if(adc_done_flag == 1)
		{
			//_RD1 = !_RD1;		// ADC 10회 기준 약 500us 소요 (2Khz)

			adc_done_flag = 0;

			CalcPackvol();
			
			RecordingMinMax();
			
			ControlFet();

			report_val_conv();
		}

		if(update_report_flag == 1)
		{
			update_report_flag = 0;
		
			// 삭제 - 수행시간 : 200ms@Wait(10000);
			// 적용 - 수행시간 : 20ms@Wait(1000);
			Wrt_S_LCD(buf_char_volt, 0, 0);
			Wrt_S_LCD(buf_char_curr, 8, 0);
			Wrt_S_LCD(buf_char_curr_dir, 15, 0);
			Wrt_S_LCD(buf_char_watt_total, 0, 1);
			Wrt_S_LCD(buf_char_watt_day, 8, 1);
		}

	}

	closeTimer1();
	closeTimer3();
	
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

			ENABLE_AN0_ANA &				// ALL_ANA, ALL_DIG, AN0_ANA~AN15_ANA_ANA
			ENABLE_AN1_ANA &
			ENABLE_AN2_ANA,

			SCAN_ALL);					// NONE, ALL, SKIP_SCAN_AN0~15

}

void io_init(void)
{
	//3 방향설정  
	// test pin 설정 
	_TRISD0 = 0;
	_TRISD1 = 0;

	// LCD 4+3 PIN
	_TRISE0 = 0;
	_TRISE1 = 0;
	_TRISE2 = 0;
	_TRISE3 = 0;

	_TRISB3 = 0;
	_TRISB4 = 0;
	_TRISB5 = 0;

	//3 초기값 
	// test pin 
	_RD0 = 0;
	_RD1 = 0;
	
}

void timer_init()
{
	ConfigIntTimer1(T1_INT_PRIOR_3 &			// 0~7 => 최소 1 이상 설정할 것
				T1_INT_ON);					// ON, OFF

	WriteTimer1(0);

	OpenTimer1(	T1_ON &						// ON, OFF
				T1_IDLE_STOP &					// CON, STOP
				T1_GATE_OFF &					// ON, OFF
				T1_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128
				T1_SYNC_EXT_OFF &  			// ON, OFF (ON 필수)
				T1_SOURCE_INT,	         	// EXT, INT
				25000);						// Match_value		// 1.5ms 주기

	// Timer3 ; 주기 검증 안함, 인터럽트 발생만 확인  
	ConfigIntTimer3(T3_INT_PRIOR_3 &			// 0~7 => 최소 1 이상 설정할 것
				T3_INT_ON);					// ON, OFF

	WriteTimer3(0);

	OpenTimer3(	T3_ON &						// ON, OFF
				T3_IDLE_STOP &					// CON, STOP
				T3_GATE_OFF &					// ON, OFF
				T3_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128

				T3_SOURCE_INT,	         	// EXT, INT
				25000);						// Match_value		// 1.5ms 주기 				
}

void ControlFet(void)
{
	static unsigned long sulOVPCnt = 0;
	static unsigned long ulOVPHoldtime = 0;

	if(packvol_convert_cali_bat >= OVP_TH)
	{
		sulOVPCnt++;

		if(sulOVPCnt >= 15000)	// 약 4초 확인함 (육안) 15000: 6초(_dev) 
		{
			sulOVPCnt = 15000;

			ulOVPHoldtime = (unsigned long)10000*10;	// 40 초 추정 _dev
			
			_RD0 = 0;
			asm volatile("NOP");
			asm volatile("NOP");

			_RD1 = ucTimerOVP;
		}
	}
	else
	{	
		sulOVPCnt = 0;

		if(ulOVPHoldtime != 0)
		{
			ulOVPHoldtime--;
			
			_RD1 = ucTimerOVP;
		}
		else
		{
			ControlFetNormal();
		}
		
	}
}

void RecordingMinMax(void)
{
	if(packvol_convert_cali <= packvol_convert_min)
	{
		packvol_convert_min = packvol_convert_cali;
	}
	if(packvol_convert_cali >= packvol_convert_max)
	{
		packvol_convert_max = packvol_convert_cali;
	}
}

void CalcPackvol(void)
{
	adc_result_source_volt = ((float)adc_average_sum[0] / VREF_LSB) * VREF_MV ;
	adc_result_bat = ((float)adc_average_sum[1] / VREF_LSB) * VREF_MV ;
	adc_result_curr = ((float)adc_average_sum[2] / VREF_LSB) * VREF_MV ;

	
	packvol_convert = adc_result_source_volt * RATIO_REG;	// [mV]
	packvol_convert_bat = adc_result_bat * RATIO_REG_BAT;	// [mV]
	packvol_convert_curr = adc_result_curr / RATIO_CONV;	// [mV]

	packvol_convert_cali = packvol_convert * GAIN + OFFSET;	// [mV]
	packvol_convert_cali_bat = packvol_convert_bat * GAIN_BAT + OFFSET_BAT;	// [mV]
	packvol_convert_cali_curr = packvol_convert_curr * 1 + 0;	// [mV]

}

void ControlFetNormal(void)
{
	if(0)//ucCheckFccFlag == 1)	// _dev2 FccHold 기능 disable
	{
		_RD0 = 0;
		asm volatile("NOP");
		asm volatile("NOP");
		
		_RD1 = ucTimerHoldLed;
		asm volatile("NOP");
		asm volatile("NOP");

		if(unFccHoldCnt >= FCC_HOLD_TIME)
		{
			unFccHoldCnt = 0;
			ucCheckFccFlag = 0;
		}
	}
	else if(packvol_convert_cali_bat <= 1000)	// vbat adc 미연결시 fet 강제 off
	{
		cnt_off++;
			
		_RD0 = 0;
		asm volatile("NOP");
		asm volatile("NOP");
		
		_RD1 = 0;
		asm volatile("NOP");
		asm volatile("NOP");
	}
	else
	{
		//if(packvol_convert_cali >= CUTOFF_PACKVOL)
		if(packvol_convert_cali_bat >= CUTOFF_PACKVOL)	// _dev3
		{
			cnt_off++;
			
			_RD0 = 0;
			asm volatile("NOP");
			asm volatile("NOP");
			
			_RD1 = 0;
			asm volatile("NOP");
			asm volatile("NOP");
		}
		else
		{
			cnt_on++;
			
			_RD0 = 1;
			asm volatile("NOP");
			asm volatile("NOP");
			
			_RD1 = 1;
			asm volatile("NOP");
			asm volatile("NOP");
		}
	}
}

unsigned char dec2ascii(unsigned char hours)
{
	unsigned char low,high,tmp;
	
	high=hours/10;
	tmp=high*10;
	low=hours-tmp;
	tmp=high*16+low;

	return(tmp);
}

void report_val_conv(void)
{
	float temp_curr_disp = 0.0;
	
	temp_curr_disp = packvol_convert_cali_curr;

/*
	// PACK [00.0 V]
	buf_char[0] = float2string10(test_a);
	buf_char[1] = float2string1(test_a);
	buf_char[2] = '.';
	buf_char[3] = float2string0_1(test_a);
	buf_char[4] = ' ';

	// PACK [0.0 A]
	buf_char[5] = float2string1(test_b);
	buf_char[6] = '.';
	buf_char[7] = float2string0_1(test_b);
	buf_char[8] = ' ';

	// TOTAL POWER [0.0 KW]
	buf_char[9] = float2string1(test_c);
	buf_char[10] = '.';
	buf_char[11] = float2string0_1(test_c);
	buf_char[12] = ' ';

	// 1DAY POWER [000 W]
	buf_char[13] = float2string100(test_d);
	buf_char[14] = float2string10(test_d);
	buf_char[15] = float2string1(test_d);
*/

	// PACK [00.0 V]
	buf_char_volt[0] = float2string10(packvol_convert_cali);
	buf_char_volt[1] = float2string1(packvol_convert_cali);
	buf_char_volt[2] = '.';
	buf_char_volt[3] = float2string0_1(packvol_convert_cali);
	buf_char_volt[4] = '\0';

	// CH/DCH [-C/-D]
	// offset 3mv 가정 시, gain*저항*I = vout,
	// I = 3mv/(100*5m) = 6mA
	if(temp_curr_disp >= DARK_CURRENT)
	{	
		buf_char_curr_dir[0] = 'C';
	}
	else if(temp_curr_disp <= -DARK_CURRENT)
	{
		buf_char_curr_dir[0] = 'D';
		temp_curr_disp = 0.0;	// 충전이 아니면 전류 표기는 0
	}
	else
	{
		buf_char_curr_dir[0] = 'R';
		temp_curr_disp = 0.0;	// 충전이 아니면 전류 표기는 0
		
	}
	buf_char_curr_dir[1] = '\0';

	// PACK [0.0 A]
	buf_char_curr[0] = float2string1(temp_curr_disp);
	buf_char_curr[1] = '.';
	buf_char_curr[2] = float2string0_1(temp_curr_disp);
	buf_char_curr[3] = '\0';

	// TOTAL POWER [0.0 KW]
	buf_char_watt_total[0] = float2string1(sum_watt_total);
	buf_char_watt_total[1] = '.';
	buf_char_watt_total[2] = float2string0_1(sum_watt_total);
	buf_char_watt_total[3] = '\0';

	// 1DAY POWER [000 W]
	buf_char_watt_day[0] = float2string100(sum_watt_1day);
	buf_char_watt_day[1] = float2string10(sum_watt_1day);
	buf_char_watt_day[2] = float2string1(sum_watt_1day);
	buf_char_watt_day[3] = '\0';

}

unsigned char float2string100(float val)
{
	unsigned char retval;

	if((val >= 1000.0) || (val < 100.0))
	{
		retval = ' ';
	}
	else
	{
		retval = (unsigned char)(val / 100) + '0';
	}
	
	return(retval);
}

unsigned char float2string10(float val)
{
	unsigned char retval;
	unsigned int tempval;
	
	tempval = (unsigned int)val;

	if(tempval < 10)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= 100)
	{
		tempval = tempval % 100;
	}

	if(tempval >= 10)
	{
		tempval = tempval / 10;
	}
		
	retval = (unsigned char)tempval + '0';
	
	return(retval);
}

unsigned char float2string1(float val)
{
	unsigned char retval;
	unsigned int tempval;

	tempval = (unsigned int)val;

	if(tempval >= 100)
	{
		tempval = tempval % 100;
	}
	
	if(tempval >= 10)
	{
		tempval = tempval % 10;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char float2string0_1(float val)
{
	unsigned char retval;
	unsigned int tempval;

	val = val * 10;
	tempval = (unsigned int)val;

	if(tempval >= 1000)
	{
		tempval = tempval % 1000;
	}
	
	if(tempval >= 100)
	{
		tempval = tempval % 100;
	}

	if(tempval >= 10)
	{
		tempval = tempval % 10;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

void ClacWatt(void)
{	
	if(packvol_convert_cali_curr >= DARK_CURRENT)
	{
		sum_watt_1day += (packvol_convert * packvol_convert_cali_curr) / CAPACITY_SUM_UINT;
	}

	ClacWattCnt++;
	if(ClacWattCnt >= CAPACITY_SUM_UINT)
	{
		ClacWattCnt = 0;
		sum_watt_1day = 0;

		sum_watt_total += (sum_watt_1day/1000);
		if(sum_watt_total >= 9.99)
		{
			sum_watt_total = 0.0;
		}
	}
	
}
