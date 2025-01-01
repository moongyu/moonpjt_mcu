#ifndef __DS1302_H
#define __DS1302_H


typedef struct Tds1302_Time
{
	unsigned char second;
	unsigned char minute;
	unsigned char hour;
	unsigned char date;
	unsigned char month;
	unsigned char year;
	unsigned char am;
	unsigned char md;
	long alltime;
}ds1302_Time;
extern struct Tds1302_Time Ds1302_Time;




#define dControl 0x8e
#define dCharger 0x90
#define dBurst	 0xbe
#define dSecond  0x80
#define dMinute  0x82
#define dHour    0x84
#define dDate	 0x86
#define dMonth   0x88
#define dYear	 0x8c

void Ds1302_Start(void);
void Ds1302_Init(void);
void Ds1302_Stop(void);

unsigned char Ds1302_InByte(void);
void Ds1302_OutByte(unsigned char command);
unsigned char Ds1302_ReadByte(unsigned char command);
void Ds1302_WriteByte(unsigned char command, unsigned char data);
void Ds1302_ResetTime(void);
void Ds1302_SetSec(unsigned char secs);
void Ds1302_SetMinutes(unsigned char mins);
void Ds1302_SetHour(unsigned char Hours, unsigned char am, unsigned char md);
unsigned char Ds1302_GetSec(void);

#endif
