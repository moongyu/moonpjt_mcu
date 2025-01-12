//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//180302 Timer1로 1.5ms pluse 만들기 
// RE Test pin으로 파형관찰 

// 주요 레지스터

// Fosc, Fcy 정의
// PLL POST에 의해 Prescale 가능 (1:1사용)
// Fosc는 PLL post단의 clock으로 생각됨 
// Timer Prescale 가능 (Fcy 에서 Divide함)

// 주의 
// Fosc, Fcy, POST, Timer Prescale 주의할 것 

// 이상한 점 
// TMRn=0 or ReadTimern 동작하는 것 같으나 Watch확인안됨 


// 180926 1ms timer 만들기 
// Fosc = Fcy / 4
//         = (8Mhz * PLL 8) / 4
//         = 16Mhz (=> 0.0625us)
// OpenTimer3()에서 16000 설정시 0.0625us* 16000 = 1ms 가능함 
// PJT Charger에서 적용 중 
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


#include <p30f4012.h>
#include <stdio.h>
#include <timer.h>

_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

void io_init(void);
void timer_init(void);

unsigned int timer_value=0;

void __attribute__((__interrupt__)) _T1Interrupt(void)
{

	PORTE^=0xffff;
	WriteTimer1(0);

	//timer_value=ReadTimer1();	// watch로 확인안됨 
	//timer_value=TMR1;			// watch로 확인안됨 
	
	IFS0bits.T1IF=0;

	_RD0 = 1;
}

void __attribute__((__interrupt__)) _T3Interrupt(void)
{

	PORTE^=0xffff;
	WriteTimer3(0);

	IFS0bits.T3IF=0;

	//_RD0 = 1;
}

int main()
{
	io_init();
	timer_init();

	while(1)
	{
		// 정상동작하나 timer_value가  watch로 확인안됨  
/*		timer_value=ReadTimer1();
		if(timer_value >= 12000)
		{
			PORTE^=0xffff;
			timer_value = 0;
			WriteTimer1(0);
		}
		else
		{
		}
*/
		// test 경험 존재 추정 
		//timer_value=TMR1;
		//printf("%d\r\n",timer_value);
		//if(TMR1==30000) printf("%d\r\n",TMR1);
	}

	closeTimer1();
	closeTimer3();
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

void io_init(void)
{
	TRISE = 0x0000;
	PORTE=0x0000;

	_TRISD0 = 0;
	_RD0 = 0;
}
