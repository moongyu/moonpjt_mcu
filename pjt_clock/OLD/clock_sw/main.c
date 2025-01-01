#include <p30f4012.h>
#include <stdio.h>
#include "ds1302.h"
#include "cLCD.h"

/*
_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     
*/
#define FCY  16000000 
#define BAUDRATE 	9600        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1

void Initial_INT(void);
void uart_init(void);
void DelayMS(unsigned int n);
unsigned char received_data=0xff;
struct Tds1302_Time Ds1302_Time;
unsigned char s,t;

void __attribute__((__interrupt__)) _INT1Interrupt(void)
{
	IFS1bits.INT1IF=0;
//	PORTDbits.RD1=!PORTDbits.RD1;
	unsigned char lsb,msb,buf,inc;
	
	buf=Ds1302_Time.minute;
	lsb=buf&0x0f;
	msb=(buf>>4)&0x0f;

	if(lsb>9)
	{
		msb=msb+1;
		lsb=0;
	}
	else lsb=lsb+1;

	inc=msb*10+lsb;
	if(inc>59) 
	{
		Ds1302_Time.minute=00;
		inc=0;
	}	
	else{}

	Ds1302_SetMinutes(inc);	
	Ds1302_SetSec(00);
}

void __attribute__((__interrupt__)) _INT2Interrupt(void)
{
	IFS1bits.INT2IF=0;
	unsigned char lsb,msb,buf,inc;

	buf=Ds1302_Time.hour;
	lsb=buf&0x0f;
	msb=(buf>>4)&0x01;

	if(lsb>9)
	{
		msb=msb+1;
		lsb=0;
	}
	else lsb=lsb+1;

	inc=msb*10+lsb;

	if(inc>12) 
	{
		inc=1;
		Ds1302_SetHour(inc,Ds1302_Time.am,Ds1302_Time.md);
	}	
	else Ds1302_SetHour(inc,Ds1302_Time.am,Ds1302_Time.md);
	
}

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
	TRISD=0x0003;	
	ADPCFG=0xffff;

	uart_init();
	Initial_INT();
	InitLCD();
	Ds1302_Init();
	Ds1302_ResetTime();
	Ds1302_SetTime();

//	printf("uart test ok!");

	HomeClearLCD();
	Wrt_S_LCD("HH MM SS", 0, 0);
	Wrt_S_LCD("  :  :  ", 0, 1);

	while(1)
	{	
		GetTime();
//		printf("HH:%x MM:%x SS:%x\r\n",Ds1302_Time.hour,Ds1302_Time.minute,Ds1302_Time.second);
//		printf("AM/PM:%x HH:%x MM:%x SS:%x\r\n",Ds1302_Time.am,Ds1302_Time.hour,Ds1302_Time.minute,Ds1302_Time.second);
//		printf("%d\r\n",Ds1302_Time.am);
		Wrt_Int_LCD(Ds1302_Time.hour, 0, 1);
		Wrt_Int_LCD(Ds1302_Time.minute, 3, 1);
		Wrt_Int_LCD(Ds1302_Time.second, 6, 1);

		DelayMS(10);

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

void Initial_INT(void)
{
	// INT1
	IEC1bits.INT1IE=1;	// interrupt enable control R1
	IPC4bits.INT1IP=3;	// interrupt priority control R4
	IFS1bits.INT1IF=0;	// interrupt flag status R1
	INTCON2bits.INT1EP=0; //1:negative edge, 0: positive edge

	// INT2
	IEC1bits.INT2IE=1;	// interrupt enable control R1
	IPC5bits.INT2IP=4;	// interrupt priority control R4
	IFS1bits.INT2IF=0;	// interrupt flag status R1
	INTCON2bits.INT1EP=0; //1:negative edge, 0: positive edge
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
