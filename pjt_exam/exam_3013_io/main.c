///////////////////////////////////////////////////////////// 
//180324
// PORTB를 Input으로 PORTD LED ON/OFF 하기

// 주의
// PORTB 는 초기화시 analog input으로 설정됨으로 digital input으로 설정 변경 (Programming mode시)
// ADPCF 설정해줘야 함 

///////////////////////////////////////////////////////////// 
#include <p30f3013.h>
#include <stdio.h>


// 사용안할 시 내부 osc로 설정됨, IDE>Configure 에서 확인가능 
_FOSC(CSW_FSCM_OFF & ECIO_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     


void Initial_IO_Ports(void);
void dDelay(int del);

int main(void)
{
	Initial_IO_Ports();
	while(1)
	{
		//if(_RB5 == 0)
		if(_RB0 == 0)
		{
			_RD8 = 0;
		}
		else
		{
			_RD8 = 1;
		}
	}
}

void Initial_IO_Ports(void)
{
	//3 PORTB 는 초기화시 analog input으로 설정됨, digital input으로 설정 변경 (Programming mode시)
	_PCFG0 = 1;	

	// IO Input/Output
	_TRISD8 = 0;

	_TRISB0 = 1;
	_TRISB5 = 1;

	// IO Init value
	_RD8 = 0;

}

void dDelay(int del)
{
	while(del--);
}
