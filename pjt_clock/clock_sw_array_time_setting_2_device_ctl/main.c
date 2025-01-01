#include <p30f4012.h>
#include <stdio.h>
#include "ds1302.h"
#include "cLCD.h"

// 170219 
// 저전력 위해 FRC Mode 로 변경
// sw 1개 추가하여 port E 04를 읽음 
// time mode, onoff setting mode 추가 

// 171118
// SYS MODE를 TIME/USER MODE로 분리 
// USER MODE에서 LED TIME, FAN TIME을 각각 셋팅 가능하도록 수정 
// port E 05를 OUTPUT PIN설정하여 FAN ON/OFF를 추가
// 주위 
// LCD CMD ControlCV시 칸수가 잘못 복사 가능성 존재. 주의할 것 (칸수 확인할 것)
// main() 바로 다음 line은 BR 매번 걸리는 현상 존재. 이유 불분명 

// (1) 8M EX Mode (OSC, Not crystal) : ~90mA@5V
// (2) Internal FRC : ~30mA@5V
//_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FOSC(CSW_FSCM_OFF & FRC);

_FWDT(WDT_OFF);

_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

#define FCY  16000000 
#define BAUDRATE 	9600        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1

#define SET_MODE_STATE_INIT (0)
#define SET_MODE_STATE_CH 	(1)
#define SET_MODE_STATE_TIME (2)

#define SET_MODE_NOT_DEF	(0)
#define SET_MODE_CH1		(1)
#define SET_MODE_CH2 		(2)

void Initial_INT(void);
void uart_init(void);
void DelayMS(unsigned int n);
unsigned char received_data=0xff;
struct Tds1302_Time Ds1302_Time;
unsigned char s,t;

unsigned char guc_sys_mode = 0;
unsigned char guc_set_time_on = 1;	// default on: 1시, off: 2시 
unsigned char guc_set_time_off = 2;

unsigned int gun_swtich_cnt = 0;
unsigned int gun_test_cnt = 0;

unsigned char guc_set_mode_number = 1;
unsigned char guc_enter_key_F = 0;
unsigned char guc_setting_mode = SET_MODE_CH1;

unsigned char guc_set_fan_on = 1;
unsigned char guc_set_fan_off = 2;
unsigned char guc_set_mode_state = SET_MODE_STATE_INIT;

unsigned char dec2ascii(unsigned char);
	
void __attribute__((__interrupt__)) _INT1Interrupt(void)	// minute setting, off counter, enter key
{
	IFS1bits.INT1IF=0;
//	PORTDbits.RD1=!PORTDbits.RD1;
	unsigned char lsb,msb,buf,inc;

	if(guc_sys_mode == 0)
	{
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
	else
	{
		if(guc_set_mode_state == SET_MODE_STATE_CH)
		{
			guc_enter_key_F = 1;
		}
		else if(guc_set_mode_state == SET_MODE_STATE_TIME)
		{
			if(guc_set_mode_number == 1)
			{
				guc_set_time_off++;
			
				if(guc_set_time_off >= 24)
				{
					guc_set_time_off = 0;
				}
			}
			else if(guc_set_mode_number == 2)
			{
				guc_set_fan_off++;
			
				if(guc_set_fan_off >= 24)
				{
					guc_set_fan_off = 0;
				}
			}
		}
		else
		{
			guc_enter_key_F = 0;
		}

	}
}

void __attribute__((__interrupt__)) _INT2Interrupt(void)	// hour, on counter, setting mode counter
{
	IFS1bits.INT2IF=0;
	unsigned char lsb,msb,buf,inc;

	if(guc_sys_mode == 0)
	{
		buf=Ds1302_Time.hour;
		lsb=buf&0x0f;
		msb=(buf>>4)&0x03;

		if(lsb>9)
		{
			msb=msb+1;
			lsb=0;
		}
		else lsb=lsb+1;

		inc=msb*10+lsb;

		if(inc>23) inc=0;
		else {}

		Ds1302_SetHour(inc,Ds1302_Time.am,Ds1302_Time.md);
	}
	else
	{
		if(guc_set_mode_state == SET_MODE_STATE_CH)
		{
			guc_set_mode_number++;
			if(guc_set_mode_number >= 3)
			{
				guc_set_mode_number = 1;
			}
		}
		else if(guc_set_mode_state == SET_MODE_STATE_TIME)
		{
			if(guc_set_mode_number == 1)
			{
				guc_set_time_on++;
			
				if(guc_set_time_on >= 24)
				{
					guc_set_time_on = 0;
				}
			}
			else if(guc_set_mode_number == 2)
			{
				guc_set_fan_on++;
			
				if(guc_set_fan_on >= 24)
				{
					guc_set_fan_on = 0;
				}
			}
		}
		else
		{
		}
	}

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
	TRISE=0x0010;

	TRISB=0x0000;
	TRISD=0x0003;	
	ADPCFG=0xffff;

	uart_init();
	Initial_INT();
	InitLCD();
	Ds1302_Init();
//	Ds1302_ResetTime();	// backup energy
//	Ds1302_SetTime();

//	printf("uart test ok!");

	HomeClearLCD();
	Wrt_S_LCD("HH MM SS", 0, 0);
	Wrt_S_LCD("  :  :  ", 0, 1);

	LATEbits.LATE8 = 0;	// for LED Driver
	LATEbits.LATE5 = 0; // for FAN Driver

	while(1)
	{	
		gun_test_cnt++;	// test용 cnt, ICD2 debugger 실시간 모니터링 안됨 

		///////////////////////////////// SYS_MODE CHANGE /////////////////////////////////

		if(PORTEbits.RE4 == 1)	// port 를 input 으로 읽어야 함 
		{
			gun_swtich_cnt++;

			if(gun_swtich_cnt >= 3) // internal FC mode 에서는 속도가 매우 느림, 3회만으로도 2초 소요  
			{
				gun_swtich_cnt = 0;

				HomeClearLCD();
				guc_set_mode_state = SET_MODE_STATE_INIT;
				
				if(guc_sys_mode == 0)
				{
					guc_sys_mode = 1;	// on/off setting mode
					
					Wrt_S_LCD("SELECT MODE:  ", 0, 0);
					Wrt_S_LCD("THEN, ENTER KEY", 0, 1);

				} 
				else
				{
					guc_sys_mode = 0;	// Time mode

					Wrt_S_LCD("HH MM SS", 0, 0);
					Wrt_S_LCD("  :  :  ", 0, 1);

				}
			}
		}
		else
		{
			gun_swtich_cnt = 0;
		}

		///////////////////////////////// SYS_MODE DOING /////////////////////////////////
		//3 TIME MODE
		if(guc_sys_mode == 0)
		{
			GetTime();

			Wrt_Int_LCD(Ds1302_Time.hour, 0, 1);
			Wrt_Int_LCD(Ds1302_Time.minute, 3, 1);
			Wrt_Int_LCD(Ds1302_Time.second, 6, 1);

			DelayMS(1);

			if(Ds1302_Time.hour == dec2ascii(guc_set_time_on))
			{
				LATEbits.LATE8=1;
			}
			else if(Ds1302_Time.hour == dec2ascii(guc_set_time_off)) 
			{
				LATEbits.LATE8=0;
			}
			else 
			{
			}

			if(Ds1302_Time.hour == dec2ascii(guc_set_fan_on))
			{
				LATEbits.LATE5=1;
			}
			else if(Ds1302_Time.hour == dec2ascii(guc_set_fan_off)) 
			{
				LATEbits.LATE5=0;
			}
			else 
			{
			}
			
		}
		//3 USER MODE
		else if(guc_sys_mode == 1)
		{
			switch(guc_set_mode_state)
			{
				case SET_MODE_STATE_INIT:
					guc_set_mode_state = SET_MODE_STATE_CH;
					Wrt_Int_LCD(dec2ascii(guc_set_mode_number), 12, 0);	//LCD위한 변환표에 의해 dec. 값 변경 필요 ; 20시=>32, 19시=>25,18시=>24에 해당.
			
				break;

				case SET_MODE_STATE_CH:
					Wrt_Int_LCD(dec2ascii(guc_set_mode_number), 12, 0);
					if(guc_enter_key_F == 1)
					{
						guc_enter_key_F = 0;

						if(guc_set_mode_number == 1)
						{
							guc_setting_mode = SET_MODE_CH1;
							HomeClearLCD();
							
							Wrt_S_LCD("  ON    OFF(CH1)", 0, 0);
							Wrt_S_LCD("  :00    :00", 0, 1);
	
							guc_set_mode_state = SET_MODE_STATE_TIME;
						}
						else if(guc_set_mode_number == 2)
						{
							guc_setting_mode = SET_MODE_CH2;
							HomeClearLCD();
							
							Wrt_S_LCD("  ON    OFF(CH2)", 0, 0);
							Wrt_S_LCD("  :00    :00", 0, 1);
	
							guc_set_mode_state = SET_MODE_STATE_TIME;
						}
						else
						{
							guc_setting_mode = SET_MODE_NOT_DEF;
							Wrt_S_LCD("NOT DEFINED MODE", 0, 0);
						}
					}

				break;

				case SET_MODE_STATE_TIME:
					if(guc_setting_mode == SET_MODE_CH1)
					{
						Wrt_Int_LCD(dec2ascii(guc_set_time_on), 0, 1);	//LCD위한 변환표에 의해 dec. 값 변경 필요 ; 20시=>32, 19시=>25,18시=>24에 해당.
						Wrt_Int_LCD(dec2ascii(guc_set_time_off), 7, 1);
					}
					else if(guc_setting_mode == SET_MODE_CH2)
					{
						Wrt_Int_LCD(dec2ascii(guc_set_fan_on), 0, 1);	//LCD위한 변환표에 의해 dec. 값 변경 필요 ; 20시=>32, 19시=>25,18시=>24에 해당.
						Wrt_Int_LCD(dec2ascii(guc_set_fan_off), 7, 1);
					}
					else
					{
					}
			
				break;

				default:
				break;
					
			}

		}
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
		for(j=0; j<319; j++)	// 128MHz : 3197
		{
			ClrWdt();
		}
	}
}

unsigned char dec2ascii(unsigned char hours)
{
	unsigned char low,high,tmp;
	
	high=hours/10;
	tmp=high*10;
	low=hours-tmp;
	tmp=high*16+low;

	return(tmp);
}


