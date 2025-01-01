#include <p30f4012.h>
#include <stdio.h>
#include "ds1302.h"
/*
_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     
*/
#define FCY  16000000 
#define BAUDRATE 	9600        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1

void uart_init(void);
void DelayMS(unsigned int n);
unsigned char received_data=0xff;
struct Tds1302_Time Ds1302_Time;

void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF=0;
	received_data=U1RXREG;
}

void __attribute__((__interrupt__)) _U1TXInterrupt(void)
{
	IFS0bits.U1TXIF=0;
}

void GetTime(void)
{
	unsigned char am, md, hr;
	Ds1302_Time.second=(Ds1302_ReadByte(dSecond)&0x7f);
	Ds1302_Time.minute=(Ds1302_ReadByte(dMinute)&0x7f);
	am=hr=md=Ds1302_ReadByte(dHour);
	Ds1302_Time.md=((md>>7)&0x01);
	Ds1302_Time.am=((am>>5)&0x01);
	Ds1302_Time.hour=(hr&0x3f);
	
}

int main(void)
{
	TRISE=0x0000;
	TRISB=0x0000;
	TRISD=0x0000;
	ADPCFG=0xffff;

	uart_init();
	Ds1302_Init();
	Ds1302_ResetTime();

//	printf("uart test ok!");
unsigned char s,t;
	while(1)
	{
		GetTime();
		printf("HH:%x MM:%x SS:%x\r\n",Ds1302_Time.hour,Ds1302_Time.minute,Ds1302_Time.second);
		DelayMS(100);

	}

}
void uart_init(void)
{
	U1MODEbits.STSEL=0;		// 1 stop bit
	U1MODEbits.PDSEL=0;		// 8bit, no parity
	U1MODEbits.ABAUD=0;		// input to capture module from 'UxRX pin'

	U1BRG=BRGVAL;
	U1STAbits.URXISEL=0;	// interrup on TxBUF becoming empty
	U1STAbits.UTXISEL=1;	// when a character is received

	U1MODEbits.ALTIO=1;		// using UxATX and UxARX I/O pins

	U1MODEbits.UARTEN=1;	// UART enable
	U1STAbits.UTXEN=1;		// TX enable

	IFS0bits.U1TXIF=0;
	IFS0bits.U1RXIF=0;

	IEC0bits.U1TXIE=1;
	IEC0bits.U1RXIE=1;

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
