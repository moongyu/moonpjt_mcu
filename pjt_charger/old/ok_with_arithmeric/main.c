//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// 180920 Charger
// ���� Vbat���� ũ�� PowerFET on, ������ off �Ͽ� PWM ������ ��
// ��Ʈ���� 10m�� �� �帣�� Current ���� IC�� Vout�� �̿��� ���� ���� 
// LCD�� Vpack, Current, Total power, 1 Day Power�� ǥ���� 
// ADC 3CH ��� (0CH: Vpack, 1CH: Vbat, 2CH: Current Vout) 
// Timer 3 ��� - 1ms

// 180920 ��Ģ���� �˻�� �߰� 
// +-*/ ���� �� ������� ��밪�� �ٸ��� Buzzer�� 1�� �ֱ�� �︲
// SW�� ����Ƚ���� Error Ƚ���� LCD ���� Ȯ�� ���� 
// ����Ƚ���� (2^32)^99 ǥ�� ���� 
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



#include <p30f4012.h>
#include <stdio.h>
#include <adc10.h>
#include <timer.h>

#include "cLCD.h"

_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);	// ����� FRC ���� ADRC ���� ����  
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

/*
_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF & WDTPSA_64 & WDTPSB_4);
_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_20 & PWMxH_ACT_HI & PWMxL_ACT_HI);
_FGS(CODE_PROT_OFF);
*/

// ������ CSSL�� ADC Port analog input������ ���ָ� �� (TRISB ������ ���ʿ�)
#define PB02				_RB2	
#define PB03				_RB3	

#define MODE_KEY			_RE4
#define OUT_BUZZER			_RE5
//============================================================
#define GAIN				(1.012)	//(1.082)
#define OFFSET				(134.4)	//(207.5)

#define GAIN_BAT			(1.0967) //(1.1027)
#define OFFSET_BAT			(-82.14) //(-25.48)

#define GAIN_CURR			(0.95)
#define OFFSET_CURR			(0.0)

#define RATIO_REG			(5.0)	//(1.0/(100.0/500.0));0
#define RATIO_REG_BAT		(1.0/(101.0/489.0))
#define RATIO_CONV			(0.01 * 100)	// 0.01 : 10moh, 100 : gain 100v/v

#define VREF_MV				(5000)
#define VREF_LSB			(1024)

#define CUTOFF_PACKVOL		(13000) //(13500)	//(13000)	//(12800)	// _dev2
#define OVP_TH				(15000) //(14500)	//(14000) //(13500)		// _dev2

#define CHECK_FCC_PERIOD	(60*10)	//(60*30)	// [sec]	// _dev
#define FCC_HOLD_TIME		(15)	// [sec]
//============================================================

#define ADC_CH_NUM				(3)
#define CAPACITY_SUM_UINT		(3600)	// 60 * 60 = 3600 [AH]
#define DARK_CURRENT			(10.0)	// [mA]

#define ADC_SAMPLE_RATE			(10)
//============================================================
#define ARITHMETIC_RESULT_ADD			(255)
#define ARITHMETIC_RESULT_DEC			(85)
#define ARITHMETIC_RESULT_MUX			(14450)
#define ARITHMETIC_RESULT_DIV			(2)

#define DECIMAL_4BYTE			(4294967295)	// 2^32 = 4294967296
#define MONITOR_POWER		(0)
#define MONITOR_ARITHMETIC		(1)


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
void arithmetic_operation(void);
void report_arithmetic(void);
void dec2ascii_normal(unsigned long, unsigned char);
void check_key(void);
void output_buzzer(void);
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
unsigned char update_fet_cont_flag = 0;

float buf_ave_curr[10] = {0};
float curr_ave = 0.0;

//============================================================

unsigned long result_arithmetic = 0;
unsigned long cnt_high, cnt_low = 0;
unsigned int val0=85, val1 = 170;
unsigned char dec_val_buf[11]={0};
unsigned char disp_mode = MONITOR_POWER;

unsigned char pwm_period = 0;

void __attribute__ ((__interrupt__)) _ADCInterrupt(void)
{
	buf[0] = ReadADC10(0);
	buf[1] = ReadADC10(1);
	buf[2] = ReadADC10(2);
	
	adc_save_count++;
	adc_sum[0] += buf[0];
	adc_sum[1] += buf[1];
	adc_sum[2] += buf[2];
	
	if(adc_save_count == ADC_SAMPLE_RATE)
	{
		adc_average_sum[0] = adc_sum[0]/ADC_SAMPLE_RATE;
		adc_sum[0] = 0;

		adc_average_sum[1] = adc_sum[1]/ADC_SAMPLE_RATE;
		adc_sum[1] = 0;

		adc_average_sum[2] = adc_sum[2]/ADC_SAMPLE_RATE;
		adc_sum[2] = 0;
		
		adc_save_count = 0;

		adc_done_flag = 1;
		//_RD1 = !_RD1;		400us���� ���� 
	}
	
	IFS0bits.ADIF=0;

	//_RD1 = 1;
}

// 1ms Timer
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
	WriteTimer1(0);

	//timer_value=ReadTimer1();	// watch�� Ȯ�ξȵ� 
	//timer_value=TMR1;			// watch�� Ȯ�ξȵ� 
	
	IFS0bits.T1IF=0;

	//_RD1 = !_RD1;
}

// 1ms Timer
void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	static unsigned int unBaseTime0 = 0;
	static unsigned int unBaseTime = 0;
	static unsigned int unBaseTimeFcc = 0;
	static unsigned int unBaseTimeHoldLed = 0;

	WriteTimer3(0);

	IFS0bits.T3IF=0;

	//_RD1 = !_RD1;

	unBaseTime0++;
	
	unBaseTime++;
	unBaseTimeFcc++;
	unBaseTimeHoldLed++;

	if(unBaseTime0 >= 50)
	{
		unBaseTime0 = 0;
		update_fet_cont_flag = 1;
	}

	if(unBaseTime >= 1000)
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

	if(unBaseTimeHoldLed >= 500)
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

	// fcc check ���
	// ���� ���� �̻��̸� fcc period��ŭ fet off �� �絿����
	if(unBaseTimeFcc >= 1000)
	{
		unBaseTimeFcc = 0;
		unFccHoldCnt++;	// timer�� 

		pwm_period = ~pwm_period;

		if(packvol_convert_cali_bat >= CUTOFF_PACKVOL+200)	// ���� �����Ͽ� �� ���� ����� �������� ����
		//if(packvol_convert_cali_bat >= 13000)	// _dev2
		{
			unFccTimer++;
			
			if(unFccTimer >= CHECK_FCC_PERIOD)
			{
				ucCheckFccFlag = 1;
				unFccTimer = 0;	// CHECK_FCC_PERIOD �ֱ�� ���� ����, unFccHoldCnt TH���� �۾ƾ� ��  

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

	//Wrt_S_LCD("0123456789ABCDEF", 0, 0);	// Copy �� ������ �� (ex. blink)
	//Wrt_S_LCD("FEDCBA9876543210", 0, 1);

	//Wrt_S_LCD("BOOTING", 0, 0);

	Wrt_S_LCD("00.0[V] 0.0[A]-C", 0, 0);
	Wrt_S_LCD("0.0[KW] 000[W]/D", 0, 1);

	while(1)
	{	
		test01++;

		//Wait(1000);	// 1ms �ҿ� Ȯ�� 
		//Wait(10000);	// 10ms �ҿ� Ȯ�� 
		
		arithmetic_operation();
		output_buzzer();
		
		if(adc_done_flag == 1)
		{
			//_RD1 = !_RD1;		// ADC 10ȸ ���� �� 500us �ҿ� (2Khz)

			adc_done_flag = 0;

			CalcPackvol();
			
			RecordingMinMax();	// _debug
			
			ControlFet();

			report_val_conv();
		}

		// ��� ���� 
		if(update_fet_cont_flag == 1)
		{
			update_fet_cont_flag = 0;
			//ControlFet();
		}

		check_key();
		
		if(update_report_flag == 1)
		{
			update_report_flag = 0;

			if(disp_mode == MONITOR_ARITHMETIC)
			{
				report_arithmetic();
			}
			else
			{
				// ���� - ����ð� : 200ms@Wait(10000);
				// ���� - ����ð� : 20ms@Wait(1000);
				Wrt_S_LCD(buf_char_volt, 0, 0);
				Wrt_S_LCD(buf_char_curr, 8, 0);
				Wrt_S_LCD(buf_char_curr_dir, 15, 0);
				Wrt_S_LCD(buf_char_watt_total, 0, 1);
				Wrt_S_LCD(buf_char_watt_day, 8, 1);
			}
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
	//3 ���⼳��  
	// CONTROL POWER FET
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

	// KEY
	_TRISE4 = 1;

	// BUZZER
	_TRISE5 = 0;

	
	//3 �ʱⰪ 
	_RD0 = 0;
	_RD1 = 0;

	_RE5 = 0;
}

void timer_init()
{
	ConfigIntTimer1(T1_INT_PRIOR_3 &			// 0~7 => �ּ� 1 �̻� ������ ��
				T1_INT_ON);					// ON, OFF

	WriteTimer1(0);

	OpenTimer1(	T1_ON &						// ON, OFF
				T1_IDLE_STOP &					// CON, STOP
				T1_GATE_OFF &					// ON, OFF
				T1_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128
				T1_SYNC_EXT_OFF &  			// ON, OFF (ON �ʼ�)
				T1_SOURCE_INT,	         	// EXT, INT
				16000);						// Match_value		// 1.0ms �ֱ�

	// Timer3 ; �ֱ� ���� ����, ���ͷ�Ʈ �߻��� Ȯ��  
	ConfigIntTimer3(T3_INT_PRIOR_3 &			// 0~7 => �ּ� 1 �̻� ������ ��
				T3_INT_ON);					// ON, OFF

	WriteTimer3(0);

	OpenTimer3(	T3_ON &						// ON, OFF
				T3_IDLE_STOP &					// CON, STOP
				T3_GATE_OFF &					// ON, OFF
				T3_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128

				T3_SOURCE_INT,	         	// EXT, INT
				16000);						// Match_value		// 1.0ms �ֱ�				
}

void ControlFet(void)
{
	static unsigned long sulOVPCnt = 0;
	static unsigned long ulOVPHoldtime = 0;

	// OVP_TH �̻� debouncing time ������ OVP ����, FET ���� Off
	// ���� Release �Ǿ ulOVPHoldtime ���� Off ���� �� FET ���� ���� 
	if(packvol_convert_cali_bat >= OVP_TH)
	{
		sulOVPCnt++;

		if(sulOVPCnt >= 15000)	// �� 4�� Ȯ���� (����) 15000: 6��(_dev) 
		{
			sulOVPCnt = 15000;

			ulOVPHoldtime = (unsigned long)10000*10;	// 40 �� ���� _dev
			
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
			// ������¿����� FET ���� 
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
	packvol_convert_curr = adc_result_curr / RATIO_CONV;	// [mA]

	packvol_convert_cali = packvol_convert * GAIN + OFFSET;	// [mV]
	packvol_convert_cali_bat = packvol_convert_bat * GAIN_BAT + OFFSET_BAT;		// [mV]
	packvol_convert_cali_curr = packvol_convert_curr * GAIN_CURR + OFFSET_CURR;	// [mA]

}

void ControlFetNormal(void)
{
	if(0)//ucCheckFccFlag == 1)	// _dev2 FccHold ��� disable (FCC ���� �� FET�� �����ð� ���� OFF �� �ٽ� ���� �� )
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
	else if(packvol_convert_cali_bat <= 1000)	// vbat adc �̿���� fet ���� off
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
		if(packvol_convert_cali_bat >= CUTOFF_PACKVOL)
		{
			cnt_off++;
			
			_RD0 = 0;
			asm volatile("NOP");	// RD0, 1 ���� ���� ���� ���� Delay ���� 
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
	unsigned char i;
	float temp_curr_disp = 0.0;

	// LCD ǥ�� ����ȭ ���� ��� ó�� (Buffer 10��)
	temp_curr_disp = packvol_convert_cali_curr;	// [mA]

	buf_ave_curr[9] = buf_ave_curr[8];
	buf_ave_curr[8] = buf_ave_curr[7];
	buf_ave_curr[7] = buf_ave_curr[6];
	buf_ave_curr[6] = buf_ave_curr[5];
	buf_ave_curr[5] = buf_ave_curr[4];
	buf_ave_curr[4] = buf_ave_curr[3];
	buf_ave_curr[3] = buf_ave_curr[2];
	buf_ave_curr[2] = buf_ave_curr[1];
	buf_ave_curr[1] = buf_ave_curr[0];
	buf_ave_curr[0] = temp_curr_disp;

	for(i=0; i<= 9; i++)
	{
		temp_curr_disp += buf_ave_curr[i];
	}
	temp_curr_disp = temp_curr_disp / 10.0;
	curr_ave = temp_curr_disp;

	// PACK [00.0 V]
	buf_char_volt[0] = float2string10(packvol_convert_cali*0.001);
	buf_char_volt[1] = float2string1(packvol_convert_cali*0.001);
	buf_char_volt[2] = '.';
	buf_char_volt[3] = float2string0_1(packvol_convert_cali*0.001);
	buf_char_volt[4] = '\0';

	// CH/DCH [-C/-D]
	// DARK_CURRENT ���� �߰��Ͽ� CH/DCH ���� 
	if(packvol_convert_cali_curr >= DARK_CURRENT)
	{	
		buf_char_curr_dir[0] = 'C';
	}
	else if(packvol_convert_cali_curr <= -DARK_CURRENT)
	{
		buf_char_curr_dir[0] = 'D';
		curr_ave = 0.0;	// ������ �ƴϸ� ���� ǥ��� 0
	}
	else
	{
		buf_char_curr_dir[0] = 'R';
		curr_ave = 0.0;	// ������ �ƴϸ� ���� ǥ��� 0
		
	}
	buf_char_curr_dir[1] = '\0';

	// PACK [0.0 A]
	// LCD���� ������ DARK_CURRENT ���� ��� ������ ǥ���� (����, Rest�� ���� 0 mA) - PACK [A] �� ���������� ���� �뵵��  
	buf_char_curr[0] = float2string1(curr_ave*0.001);
	buf_char_curr[1] = '.';
	buf_char_curr[2] = float2string0_1(curr_ave*0.001);
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
		sum_watt_1day += ((packvol_convert_cali*0.001) * (curr_ave * 0.001)) / CAPACITY_SUM_UINT;
	}

	ClacWattCnt++;
	if(ClacWattCnt >= CAPACITY_SUM_UINT*24)
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

void arithmetic_operation(void)
{
	if(cnt_low >= DECIMAL_4BYTE)
	{
		cnt_low = 0;
		cnt_high++;
	}
	else
	{		
		cnt_low++;
	}

	if(((val0 + val1) != ARITHMETIC_RESULT_ADD)
		| ((val1 - val0) != ARITHMETIC_RESULT_DEC)
		| ((val0 * val1) != ARITHMETIC_RESULT_MUX)
		| ((val1 / val0) != ARITHMETIC_RESULT_DIV))
	{
		result_arithmetic++;

		if(result_arithmetic >= 99)
		{
			result_arithmetic = 99;
		}
	}

}

void report_arithmetic(void)
{
	// _Debug
	//result_arithmetic = 2;
	//cnt_high = 11;
	//cnt_low = 4155544433;
	
	dec2ascii_normal(result_arithmetic, 2);	// result_arithmetic ���� 2�ڸ����θ� ǥ���� ex) 2 -> 02
	Wrt_S_LCD(dec_val_buf, 4, 0);

	dec2ascii_normal(cnt_high, 2);
	Wrt_S_LCD(dec_val_buf, 14, 0);

	dec2ascii_normal(cnt_low, 10);
	Wrt_S_LCD(dec_val_buf, 7, 1);
	
}

void dec2ascii_normal(unsigned long tempval, unsigned char pos_number)
{
	dec_val_buf[0] = (unsigned char)(tempval / 1000000000)+'0';
	tempval = tempval % 1000000000;
	dec_val_buf[1] = (unsigned char)(tempval / 100000000)+'0';
	tempval = tempval % 100000000;
	dec_val_buf[2] = (unsigned char)(tempval / 10000000)+'0';
	tempval = tempval % 10000000;
	dec_val_buf[3] = (unsigned char)(tempval / 1000000)+'0';
	tempval = tempval % 1000000;
	dec_val_buf[4] = (unsigned char)(tempval / 100000)+'0';
	tempval = tempval % 100000;
	dec_val_buf[5] = (unsigned char)(tempval / 10000)+'0';
	tempval = tempval % 10000;
	dec_val_buf[6] = (unsigned char)(tempval / 1000)+'0';
	tempval = tempval % 1000;
	dec_val_buf[7] = (unsigned char)(tempval / 100)+'0';
	tempval = tempval % 100;
	dec_val_buf[8] = (unsigned char)(tempval / 10)+'0';
	tempval = tempval % 10;
	dec_val_buf[9] = (unsigned char)tempval+'0';

	switch(pos_number)
	{
		case 1:
			dec_val_buf[0] = dec_val_buf[9];
			dec_val_buf[1] = '\0';
			break;

		case 2:
			dec_val_buf[0] = dec_val_buf[8];
			dec_val_buf[1] = dec_val_buf[9]; 
			dec_val_buf[2] = '\0';
			break;

		case 3:
			dec_val_buf[0] = dec_val_buf[7];
			dec_val_buf[1] = dec_val_buf[8]; 
			dec_val_buf[2] = dec_val_buf[9]; 
			dec_val_buf[3] = '\0';
			break;
			
		case 4: 	
			dec_val_buf[4] = '\0';
			break;
		
		case 5: 	
			dec_val_buf[5] = '\0';
			break;
		
		case 6: 	
			dec_val_buf[6] = '\0';
			break;
			
		case 7: 	
			dec_val_buf[7] = '\0';
			break;
		
		case 8: 	
			dec_val_buf[8] = '\0';
			break;
		
		case 9:
			dec_val_buf[0] = dec_val_buf[1];
			dec_val_buf[1] = dec_val_buf[2]; 
			dec_val_buf[2] = dec_val_buf[3];
			dec_val_buf[3] = dec_val_buf[4];
			dec_val_buf[4] = dec_val_buf[5]; 
			dec_val_buf[5] = dec_val_buf[6];
			dec_val_buf[6] = dec_val_buf[7];
			dec_val_buf[7] = dec_val_buf[8]; 
			dec_val_buf[8] = dec_val_buf[9];
			dec_val_buf[9] = '\0';
			break;

		case 10:
			dec_val_buf[10] = '\0';
			break;

		default:
			break;
			
	}
	
}

// ����ġ�� ���� �ø��� display mode�� ����� 
void check_key(void)
{
	static unsigned int cnt_key = 0;

	if(MODE_KEY == 0)
	{
		if(cnt_key >= 300)
		{
			cnt_key = 0;
			
			if(disp_mode == MONITOR_POWER)
			{
				disp_mode = MONITOR_ARITHMETIC;

				HomeClearLCD();
				Wrt_S_LCD("ERR:   HI_CNT:   ", 0, 0);
				Wrt_S_LCD("LO_CNT:          ", 0, 1);
			}
			else
			{
				disp_mode = MONITOR_POWER;
				
				HomeClearLCD();
				Wrt_S_LCD("00.0[V] 0.0[A]-C", 0, 0);
				Wrt_S_LCD("0.0[KW] 000[W]/D", 0, 1);
			}
		}
		else
		{
			cnt_key++;
		}
	}
	else
	{
		cnt_key = 0;
	}

}

void output_buzzer(void)
{
	if(result_arithmetic != 0)
	{
		OUT_BUZZER = pwm_period;
	}
	else
	{
		OUT_BUZZER = 0;
	}
}
