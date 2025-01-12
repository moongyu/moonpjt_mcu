///////////////////////////////////////////////////////////// 
//180324
// PORTB�� Input���� PORTD LED ON/OFF �ϱ�

// ����
// PORTB �� �ʱ�ȭ�� analog input���� ���������� digital input���� ���� ���� (Programming mode��)
// ADPCF ��������� �� 

///////////////////////////////////////////////////////////// 
#include <p30f3013.h>
#include <stdio.h>


// ������ �� ���� osc�� ������, IDE>Configure ���� Ȯ�ΰ��� 
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
	//3 PORTB �� �ʱ�ȭ�� analog input���� ������, digital input���� ���� ���� (Programming mode��)
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
