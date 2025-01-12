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

void adc_init(void);  
void io_init(void);

int i,adc_save_count=0;
int buf[2]={0}, adc_sum[2]={0};
long adc_average_sum[2]={0};

int test01=0;

void __attribute__ ((__interrupt__)) _ADCInterrupt(void)
{

	buf[0] = ReadADC10(0);
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

	_RD0 = 1;
}

int main(void)
{
	io_init();
	adc_init();

	while(1)
	{	
		test01++;
	}
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
	//TRISB = 0xFFFF;	

	_TRISD0 = 0;	// test pin 설정 
	_RD0 = 0;		// test pin 
}
