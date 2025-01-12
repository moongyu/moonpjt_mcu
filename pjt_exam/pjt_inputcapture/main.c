//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//180302 IC1으로 Inputcapture sample code (or IC8)
// 1.5ms high pulse를 잡아 [sec] 단위로 측정

// 주요 레지스터
// T3CON : Type C Timer Base Register (Prescale 8 사용, 주의)
// IC1CON : Input Capture Control Register (or  IC8CON)

// Fosc, Fcy 정의
// PLL POST에 의해 Prescale 가능 (1:1사용)
// Fosc는 PLL post단의 clock으로 생각됨 
// Timer Prescale 가능 (Fcy 에서 Divide함)

// 주의 
// Capture시 overflow로 오측정되지않게 조심할 것 
// Fosc, Fcy, POST, Timer Prescale 주의할 것 
// 사용할 PIN의 Input 설정과 ICn 변경에 의한 레지스터 변경 주의할 것 

// 주의
// PORTB 는 초기화시 analog input으로 설정됨으로 digital input으로 설정 변경 (Programming mode시)
// ADPCF 설정해줘야 함 

// 이상한 점 
// TMRn=0 or ReadTimern 동작하는 것 같으나 Watch확인안됨 

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



#include <p30f4012.h>
#include <stdio.h>
#include <incap.h>
#include <timer.h>


_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);	// 스펙상 FRC 사용시 ADRC 설정 변경  
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

void inputcapture_init(void);  
void io_init(void);

int test01=0;
int Interrupt_Count = 0 , Int_flag;
unsigned int timer_first_edge;
unsigned int timer_second_edge;
unsigned int timer_pluse_width_raw;

float timer_pluse_width_sec;

unsigned int a;

//3 RD0 PIN으로 IC1 사용시 
/*
void __attribute__((__interrupt__)) _IC1Interrupt(void)
{
	_RE0 = ~_RE0;

	if(_RD0 == 1)
	{
		ReadCapture1(&timer_first_edge);
	}
	else if(_RD0 == 0)
	{
		ReadCapture1(&timer_second_edge);
		Int_flag = 1;
		
		timer_pluse_width_raw = timer_second_edge - timer_first_edge;
		timer_pluse_width_sec = (float)timer_pluse_width_raw * TICK * 8;	// Timer Prescale = 8
	}
	else
	{
	}

	IFS0bits.IC1IF = 0;
}

int main(void)
{
	io_init();
	inputcapture_init();

	while(1)
	{	
		test01++;
	}
}

void inputcapture_init(void)
{
/////////////////////////////// Inputcapture Initilization ///////////////////////////////
	// Enable IC1 Interrupt and Priority to '1' 
	ConfigIntCapture1(IC_INT_PRIOR_1 & IC_INT_ON);

	// Configure the InputCapture1 in stop in idle mode , Timer
	// 3 as source , interrupt on capture 1, I/C on every edge 
	OpenCapture1(IC_IDLE_STOP & IC_TIMER3_SRC &
	IC_INT_1CAPTURE & IC_EVERY_EDGE);	// IC_EVERY_RISE_EDGE

	T3CONbits.TON = 1;		// Timer 3 On 
	T3CONbits.TCKPS0 = 1;	//Prescale (1:8), Internal clcok (Default)
	T3CONbits.TCKPS1 = 0;
}

void io_init(void)
{
	TRISD = 0xFFFF;	//IC1 사용 (RD0), TRIS 설정만 하면 됨
	TRISE = 0x0000;	// TEST PIN RE0 용도 
}
*/





//3 RB5 Pin을 이용해 IC8 사용시 
void __attribute__((__interrupt__)) _IC8Interrupt(void)
{
	_RE0 = ~_RE0;

	if(_RB5 == 1)
	{
		ReadCapture8(&timer_first_edge);
	}
	else if(_RB5 == 0)
	{
		ReadCapture8(&timer_second_edge);
		Int_flag = 1;
		
		timer_pluse_width_raw = timer_second_edge - timer_first_edge;
		timer_pluse_width_sec = (float)timer_pluse_width_raw * TICK * 8;	// Timer Prescale = 8
	}
	else
	{
	}

	IFS1bits.IC8IF = 0;

	_RD0 = 1;	// test pin 확인 
}

int main(void)
{
	io_init();
	inputcapture_init();

	while(1)
	{	
		test01++;
	}
}

void inputcapture_init(void)
{
/////////////////////////////// Inputcapture Initilization ///////////////////////////////
	// Enable IC1 Interrupt and Priority to '1' 
	ConfigIntCapture8(IC_INT_PRIOR_1 & IC_INT_ON);

	// Configure the InputCapture1 in stop in idle mode , Timer
	// 3 as source , interrupt on capture 1, I/C on every edge 
	OpenCapture8(IC_IDLE_STOP & IC_TIMER3_SRC &
	IC_INT_1CAPTURE & IC_EVERY_EDGE);	// IC_EVERY_RISE_EDGE

	T3CONbits.TON = 1;		// Timer 3 On 
	T3CONbits.TCKPS0 = 1;	//Prescale (1:8), Internal clcok (Default)
	T3CONbits.TCKPS1 = 0;
}

void io_init(void)
{
	_PCFG5 = 1;	// RORTB를 ADC 이외 사용시 필수 

	_TRISB5 = 1;	//IC8 사용 (RB5), TRIS 설정만 하면 됨
	TRISE = 0x0000;	// TEST PIN RE0 용도 

	_TRISD0 = 0;
	_RD0 = 0;
	
}
