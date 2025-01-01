#include "ds1302.h"
#include <p30f4012.h>
#include <stdio.h>

void dDelay(int del)
{
	while(del--);
}

unsigned char Ds1302_InByte(void)
{
	unsigned char c, d;
	char i;
	c=0x00;
	for(i=0;i<8;i++)
	{			
		TRISDbits.TRISD0=1;
		if(PORTDbits.RD0==1) c=c|0x80;
		else c=c&0x7f;
		c>>=1;
		TRISDbits.TRISD0=0;

		PORTEbits.RE0=1;		// clk
		dDelay(50);
		PORTEbits.RE0=0;		// clk
		PORTDbits.RD0=1;		//io
	}
//	printf("%x\r\n",c);
	return c;
}

void Ds1302_Start(void)
{
	PORTEbits.RE0=0;		// clk
	PORTEbits.RE1=0;		// rst
	PORTEbits.RE1=1;		// rst
	dDelay(50);
}

void Ds1302_Stop(void)
{
	dDelay(50);
	PORTEbits.RE0=0;
	PORTEbits.RE1=0;
	PORTEbits.RE1=1;
}

void Ds1302_OutByte(unsigned char command)
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		PORTDbits.RD0=0;	//io
		if(command & 0x01) PORTDbits.RD0=1;
		else PORTDbits.RD0=0;
		command>>=1;

		PORTEbits.RE0=0;	//clk
		dDelay(50);
		PORTEbits.RE0=1;

	}
}

void Ds1302_WriteByte(unsigned char command, unsigned char data)
{
	Ds1302_Start();
	Ds1302_OutByte(command & 0xfe);
	Ds1302_OutByte(data);
	Ds1302_Stop();
}

unsigned char Ds1302_ReadByte(unsigned char command)
{
	unsigned char i;
	Ds1302_Start();
	Ds1302_OutByte(command | 0x01);
	i=Ds1302_InByte();
	Ds1302_Stop();

	return i;
}

void Ds1302_Init(void)
{
	unsigned char i;
	Ds1302_WriteByte(dControl,0x00);
	Ds1302_WriteByte(dCharger,0xa0);
	i=Ds1302_ReadByte(dSecond);
	Ds1302_WriteByte(dSecond,0x7f&i);
}

void Ds1302_ResetTime(void)
{
	Ds1302_SetSec(00);
	Ds1302_SetMinutes(00);
	Ds1302_SetHour(12,0,1);
}

void Ds1302_SetSec(unsigned char secs)
{
	unsigned char low,high,tmp;
	high=secs/10;
	tmp=high*10;
	low=secs-tmp;
	Ds1302_WriteByte(0x80,high*16+low);
}

void Ds1302_SetMinutes(unsigned char mins)
{
	unsigned char low,high,tmp;
	high=mins/10;
	tmp=high*10;
	low=mins-tmp;
	Ds1302_WriteByte(0x82,high*16+low);
}

void Ds1302_SetHour(unsigned char hours, unsigned char am, unsigned char md)
{
	unsigned char low,high,tmp;
	high=hours/10;
	tmp=high*10;
	low=hours-tmp;
	tmp=high*16+low;
	tmp=am*32+tmp;
	if(md==0) tmp=0x80+tmp;
	Ds1302_WriteByte(0x84,tmp);
}

unsigned char Ds1302_GetSec(void)
{
	unsigned char i;
	Ds1302_Start();
	Ds1302_OutByte(0x81 | 0x01);
	i=Ds1302_InByte();
	Ds1302_Stop();

	return i;
}
