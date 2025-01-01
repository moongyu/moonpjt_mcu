#include "ds1302.h"
#include <p30f4012.h>
#include <stdio.h>

#define DATA_IN TRISBbits.TRISB4=1;
#define DATA_OUT TRISBbits.TRISB4=0;  
#define DATA LATBbits.LATB4
#define CLK  LATBbits.LATB3
#define RST  LATBbits.LATB5

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
		DATA_IN;
		if(PORTBbits.RB4==1) c=c|0x80;
		else c=c&0x7f;
		c>>=1;
		DATA_OUT;

		CLK=1;		
		dDelay(50);
		CLK=0;		
		DATA=1;		
	}
//	printf("%x\r\n",c);
	return c;
}

void Ds1302_Start(void)
{
	CLK=0;		
	RST=0;		
	RST=1;		
	dDelay(50);
}

void Ds1302_Stop(void)
{
	dDelay(50);
	CLK=0;
	RST=0;
	RST=1;
}

void Ds1302_OutByte(unsigned char command)
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		DATA=0;	
		if(command & 0x01) DATA=1;
		else DATA=0;
		command>>=1;

		CLK=0;	
		dDelay(50);
		CLK=1;

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
	Ds1302_WriteByte(dCharger,0xa5);	// d1=1EA, r1=2k
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
//	tmp=am*32+tmp;
//	if(md==0) tmp=0x80+tmp;
	Ds1302_WriteByte(0x84,tmp);
}
// 시도 실패  
void Ds1302_SetHour_hour(unsigned char hours)
{
	unsigned char low,high,tmp;
	high=hours/10;
	tmp=high*10;
	low=hours-tmp;
	tmp=high*16+low;
	Ds1302_WriteByte(0x84,tmp);
}

void Ds1302_SetTime(void)
{
	Ds1302_Time.second=00;
	Ds1302_Time.minute=00;
	Ds1302_Time.hour=00;
	Ds1302_Time.am=0;
	Ds1302_Time.md=0;

	Ds1302_SetSec(Ds1302_Time.second);
	Ds1302_SetMinutes(Ds1302_Time.minute);
	Ds1302_SetHour(Ds1302_Time.hour,Ds1302_Time.am,Ds1302_Time.md);
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
