#include <p30f4012.h>
#include <stdio.h>
#include "cLCD.h"
/*
_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     
*/

unsigned char s=0x59;
void DelayMS(unsigned int n);

int main(void)
{
	TRISE=0x0000;
	TRISB=0x0000;
	TRISD=0x0000;

	InitLCD();

	while(1)
	{
		Wrt_S_LCD("TEST", 0, 0);
		DelayMS(100);
		Wrt_Int_LCD(s, 0, 1);
		DelayMS(100);
	}

}

void DelayMS(unsigned int n)
{
	unsigned int j;
	while(n--)
	{
		for(j=0; j<3197; j++)	// 128MHz : 3197
		{
			ClrWdt();
		}
	}
}
