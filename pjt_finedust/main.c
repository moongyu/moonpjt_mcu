//////////////////////////////////////////////////////////////////////////////////////
//180324 공기청정기 
// TMS5003T 로부터 Uart Rx 로 PM1, 2.5, 10, 온습도 정보를 받아옴 
// 미세먼지 기준치별 Green, Yellow, Red LED 알림 
// 1초 주기로 MCU 정상동작을 알리는 Indicator LED 동작 
// Fan제어 위한 SW1 : 강제 On/Off, SW2 : Yellow, Red 시에만 DC Fan On

// OC 구현하여 동작하나 실제 기능으로는 사용하지 않음 (Fan에서 pwm 지원 불가)
//////////////////////////////////////////////////////////////////////////////////////


#include <p30f3013.h>
#include <stdio.h>
#include <uart.h>
#include <timer.h>
#include <outcompare.h>

#include "cLCD.h"


// 사용안할 시 내부 osc로 설정됨, IDE>Configure 에서 확인가능 
_FOSC(CSW_FSCM_OFF & ECIO_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);      

//#define FCY  8000000 
//#define BAUDRATE 	19200
#define FCY  16000000        
#define BAUDRATE 	9600        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1           

#define UART_ENABLE

//3 PORTB는 Output으로 사용시 PB(RB)가 아닌 LATB로 사용할 것 
#define LED_RED		_LATB3
#define LED_YELLOW	_LATB4
#define LED_GREEN	_LATB5

#define LED_INDICATOR			_RD8
#define LED_INDICATOR_STATUS	_LATD8

#define CONTROL_FAN	_RF4
#define IN_SW1		_LATF5
#define IN_SW2		_LATF6


// PM2.5 ->  좋음 : 0~15, 보통 : ~35, 안좋음 ~75, 매우 안좋음 : 76~ 
// PM10 -> 좋음 : 0~30, 보통 : ~80, 안좋음 ~150, 매우 안좋음 : 151~ 
#define PM2_5_GOOD			(15)
#define PM2_5_MID			(35)
#define PM2_5_BAD			(75)

#define PM10_GOOD			(30)
#define PM10_MID			(80)
#define PM10_BAD			(150)

#define FAN_OP_DELAY		(60)	// _debug (5)
//////////////////////////////////////////////////////////////////
void io_init(void);
void uart_init(void);
void timer_init(void);
void oc_init(void);

unsigned char float2string100(float);
unsigned char float2string10(float);
unsigned char float2string1(float);

void control_led(void);
void check_sw(void);
void control_fan(void);
//////////////////////////////////////////////////////////////////
enum
{
	PM_GOOD,
	PM_MID,
	PM_BAD,
	PM_ERROR
}ePM_STATUS;

enum
{
	INIT,
	NONAUTOMODE,
	AUTOMODE
}eSW_STATUS;

unsigned int gu16_timer_cnt = 0;
unsigned char received_data = 0;

unsigned char buf[32];
unsigned char *Receivedddata = buf;

unsigned char buf_result[32];

unsigned char receive_result_flag = 0;
unsigned int rawdata_pm1_0, rawdata_pm2_5, rawdata_pm10, rawdata_pm1_0_atmos, rawdata_pm2_5_atmos, rawdata_pm10_atmos = 0;
unsigned int rawdata_temp, rawdata_humi = 0;

unsigned char rx_receive_cnt, rx_length = 0;

unsigned int testa = 0;

char buf_char_pm1_0[4];
char buf_char_pm2_5[4];
char buf_char_pm10[4];
char buf_char_temp[3];
char buf_char_humi[3];
char buf_char_indi[2];

unsigned int disp_pm1_0_atmos, disp_pm2_5_atmos, disp_pm10_atmos, disp_temp, disp_humi = 0;

unsigned int update_indicator = 0;

unsigned char control_fan_flag, fan_automode_flag = INIT;
unsigned char status_pm = 0;
//////////////////////////////////////////////////////////////////
void __attribute__ ((__interrupt__)) _U1TXInterrupt(void)  //uart
{
	IFS0bits.U1TXIF=0;
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	unsigned char i = 0;
	
	IFS0bits.U1RXIF=0;
	//received_data=U1RXREG;	// ReadUART1(); 효과임으로 중복 사용 불가 

	while( DataRdyUART1())
	{
		buf[rx_receive_cnt] = ReadUART1();
	}

	if(rx_receive_cnt >= 31)
	{
		rx_receive_cnt = 0;
		rx_length = buf[3];

		if((rx_length == 28)
			&& (buf[0] == 0x42) 
			&& (buf[1] == 0x4d)
			&& (buf[29] == 0x00))	// data 무결정 체크 
		{					
			for(i = 0; i<=31; i++)
			{
				buf_result[i] = buf[i];
			}
			receive_result_flag = 1;
		}

		for(i = 0; i<=31; i++)
		{
			buf[i] = 0;
		}
	}
	else
	{
		rx_receive_cnt++;
	}
}

// 1ms Timer
void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	static int automode_fan_on_cnt, automode_fan_off_cnt = 0;
	
	WriteTimer3(0);

	IFS0bits.T3IF=0;

	gu16_timer_cnt++;
			
#ifdef UART_ENABLE
			//printf("%d\n",gu16_timer_cnt);
#endif

	if(gu16_timer_cnt >= 1000)
	{
		gu16_timer_cnt = 0;
		if(LED_INDICATOR_STATUS == 0)	//3 Port의 상태 확인 위해서 LATD 읽음 
		{
			LED_INDICATOR = 1;
		}
		else
		{
			LED_INDICATOR = 0;
		}
		
		if(fan_automode_flag == AUTOMODE)
		{
			if((status_pm == PM_MID) || (status_pm == PM_BAD))
			{
				automode_fan_off_cnt = 0;
				
				if(automode_fan_on_cnt >= FAN_OP_DELAY)	// 60초 동안 지속시 fan on
				{
					control_fan_flag = 1;
				}
				else
				{
					automode_fan_on_cnt++;
				}
			}
			else
			{
				automode_fan_on_cnt = 0;
				
				if(control_fan_flag == 1)	// 60초 동안 지속시 fan off
				{	
					if(automode_fan_off_cnt >= FAN_OP_DELAY)
					{
						control_fan_flag = 0;
					}
					else
					{
						automode_fan_off_cnt++;
					}
				}
				else
				{
					automode_fan_off_cnt = 0;
				}
			}
		}
		else
		{
			automode_fan_on_cnt = 0;
			automode_fan_off_cnt = 0;
		}
	}

}

void __attribute__((__interrupt__)) _OC1Interrupt(void)
{
	// ISR 안걸리는 현상 확인필요 (TBD)
	
	IFS0bits.OC1IF = 0;
	//OC1R = 4000;		// OC1IF 후 Reload하라고 guide 존재하나 없어도 동작함
}

int main(void)
{
	//static unsigned int received_cnt =0 ;
	
	io_init();
	InitLCD();
#ifdef UART_ENABLE
	uart_init();
#endif
	timer_init();
	//oc_init();	// no use
	
	HomeClearLCD();

#ifdef UART_ENABLE
	printf("UART TEST Okay!\r\n");
#endif

	//_RD8 = 1;
	_RD9 = 1;
   //_RF4 = 1;	// _debug

           //("0123456789012345
	Wrt_S_LCD("[ug/m3]:   ,   ", 0, 0);
	Wrt_S_LCD("   ,   deg   %  ", 0, 1);

	while(1)
	{	
		check_sw();

		control_fan();

		if(receive_result_flag == 1)
		{
			receive_result_flag = 0;

			if(update_indicator >= 9)
			{
				update_indicator = 0;
			}
			else
			{
				update_indicator++;
			}
		
			rawdata_pm1_0 = buf_result[4]<<8;
			rawdata_pm1_0 |= buf_result[5];

			rawdata_pm2_5 = buf_result[6]<<8;
			rawdata_pm2_5 |= buf_result[7];

			rawdata_pm10 = buf_result[8]<<8;
			rawdata_pm10 |= buf_result[9];

			rawdata_pm1_0_atmos = buf_result[10]<<8;
			rawdata_pm1_0_atmos |= buf_result[11];

			rawdata_pm2_5_atmos = buf_result[12]<<8;
			rawdata_pm2_5_atmos |= buf_result[13];

			rawdata_pm10_atmos = buf_result[14]<<8;
			rawdata_pm10_atmos |= buf_result[15];

			rawdata_temp = buf_result[24]<<8;
			rawdata_temp |= buf_result[25];
			rawdata_temp /= 10;

			rawdata_humi = buf_result[26]<<8;
			rawdata_humi |= buf_result[27];
			rawdata_humi /= 10;

			if(rawdata_pm1_0_atmos >= 999)
			{
				disp_pm1_0_atmos = 999;
			}
			if(rawdata_pm2_5_atmos >= 999)
			{
				disp_pm2_5_atmos = 999;
			}
			if(rawdata_pm10_atmos >= 999)
			{
				disp_pm10_atmos = 999;
			}
			if(rawdata_temp >= 99)
			{
				disp_temp = 99;
			}
			if(rawdata_humi >= 99)
			{
				disp_humi = 99;
			}

			// _debug
			//rawdata_pm1_0_atmos = 999;
			//rawdata_pm2_5_atmos = 999;
			//rawdata_pm10_atmos = 999;

			// 1행 : PM2.5 PM10
			// 2행 : PM1 TEMP HUMI INDICATOR
			buf_char_pm1_0[0] = float2string100(rawdata_pm1_0_atmos);
			buf_char_pm1_0[1] = float2string10(rawdata_pm1_0_atmos);
			buf_char_pm1_0[2] = float2string1(rawdata_pm1_0_atmos);
			buf_char_pm1_0[3] = '\0';
			Wrt_S_LCD(buf_char_pm1_0, 0, 1);

			buf_char_pm2_5[0] = float2string100(rawdata_pm2_5_atmos);
			buf_char_pm2_5[1] = float2string10(rawdata_pm2_5_atmos);
			buf_char_pm2_5[2] = float2string1(rawdata_pm2_5_atmos);
			buf_char_pm2_5[3] = '\0';
			Wrt_S_LCD(buf_char_pm2_5, 8, 0);

			buf_char_pm10[0] = float2string100(rawdata_pm10_atmos);
			buf_char_pm10[1] = float2string10(rawdata_pm10_atmos);
			buf_char_pm10[2] = float2string1(rawdata_pm10_atmos);
			buf_char_pm10[3] = '\0';
			Wrt_S_LCD(buf_char_pm10, 12, 0);

			buf_char_temp[0] = float2string10(rawdata_temp);
			buf_char_temp[1] = float2string1(rawdata_temp);
			buf_char_temp[2] = '\0';
			Wrt_S_LCD(buf_char_temp, 5, 1);

			buf_char_humi[0] = float2string10(rawdata_humi);
			buf_char_humi[1] = float2string1(rawdata_humi);
			buf_char_humi[2] = '\0';
			Wrt_S_LCD(buf_char_humi, 11, 1);

			buf_char_indi[0] = float2string1(update_indicator);
			buf_char_indi[1] = '\0';
			Wrt_S_LCD(buf_char_indi, 15, 1);	

			control_led();

		}

	}
}

//////////////////////////////////////////////////////////////////
void io_init(void)
{
	//3 방향설정  
	_TRISD8 = 0;
	_TRISD9 = 0;
	
	//3 초기값 
	_RD8 = 0;
	_RD9 = 0;

	// LCD 4+3 PIN
	_TRISB6 = 0;
	_TRISB7 = 0;
	_TRISB8 = 0;
	_TRISB9 = 0;

	_TRISB0 = 0;
	_TRISB1 = 0;
	_TRISB2 = 0;

	// uart cable에 의한 mcu 오동작 여부 확인 필요 
	_TRISC13 = 0;
	_TRISC14 = 1;

	// LED 3EA
	_TRISB3 = 0;
	_TRISB4 = 0;
	_TRISB5 = 0;

	// FAN
	_TRISF4 = 0;

	// SW
	_TRISF5 = 1;
	_TRISF6 = 1;

}

void uart_init(void)
{
	
	//*************************************** RS232 Initilization ************************************************
    ConfigIntUART1(UART_RX_INT_EN &				// EN, DIS
				UART_RX_INT_PR6 &				// 0 ~ 7
				UART_TX_INT_EN &				// EN, DIS
				UART_TX_INT_PR3);				// 0 ~ 7


    OpenUART1(	UART_EN &					// EN, DIS
				UART_IDLE_CON &				// CON, STOP
				UART_ALTRX_ALTTX &			// ALTRX_ALTTX, RX_TX
				UART_EN_WAKE &				// EN_WAKE, DIS_WAKE
				UART_DIS_LOOPBACK &			// EN_LOOPBACK, DIS_LOOPBACK
				UART_EN_ABAUD &				// EN_ABAUD, DIS_ABAUD
				UART_NO_PAR_8BIT &			// NO_PAR_9BIT, NO_PAR_8BIT, ODD_PAR_8BIT, EVEN_PAR_8BIT
				UART_1STOPBIT,				// 2STOPBITS, 1STOPBIT

				UART_INT_TX &					// INT_BUF_EMPTY, INT_TX
				UART_TX_PIN_NORMAL &			// NORMAIL, LOW
				UART_TX_ENABLE &				// ENABLE, DISABLE
				UART_INT_RX_CHAR &			// RX_BUF_FUL, RX_3_4_FUL, RX_CHAR
				UART_ADR_DETECT_DIS &			// EN, DIS
				UART_RX_OVERRUN_CLEAR,			// CLEAR
   		 		BRGVAL);							// 16=115200BPS, 34=57600BPS, 51=38400, 103=19200, ./(16(UxBRG+1))
	//***********************************************************************************************
}
void timer_init()
{
/*	ConfigIntTimer1(T1_INT_PRIOR_7 &			// 0~7 => 최소 1 이상 설정할 것	
				T1_INT_ON);					// ON, OFF

	WriteTimer1(0);

	OpenTimer1(	T1_ON &						// ON, OFF
				T1_IDLE_STOP &					// CON, STOP
				T1_GATE_OFF &					// ON, OFF
				T1_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128
				T1_SYNC_EXT_OFF &  			// ON, OFF (ON 필수)
				T1_SOURCE_INT,	         	// EXT, INT
				1600);						// Match_value		// 100us 주기
	*/
	ConfigIntTimer3(T3_INT_PRIOR_3 &			// 0~7 => 최소 1 이상 설정할 것		// 190227 우선순위 7 로 설정
				T3_INT_ON);					// ON, OFF

	WriteTimer3(0);

	OpenTimer3(	T3_ON &						// ON, OFF
				T3_IDLE_STOP &					// CON, STOP
				T3_GATE_OFF &					// ON, OFF
				T3_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128

				T3_SOURCE_INT,	         	// EXT, INT
				8000);						// Match_value		// 1.0ms 주기		
				// (8M * PLL 4) / 4 Fix ) * 8000 N = 1ms
				// 0.125us * 8000 = 1ms

}

void oc_init(void)
{
//2 256ms 주기 중 50% duty의 pwm 발생

	//OC1R = 4000;	// 사전 초기화 guide 존재하나 없어도 동작함

	ConfigIntOC1(OC_INT_OFF & OC_INT_PRIOR_5);	// ISR 수행안함 

	OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC &
			OC_PWM_FAULT_PIN_DISABLE,	// OC_PWM_FAULT_PIN_DISABLE: pwm방식 (OC1R<-OC1SR시 OC High, overflow시 OC Low)
			4000, 4000);	// OCxRS(Secondary Register) , OCxR (Main Register) ; pwm duty cycle에 해당 

	ConfigIntTimer2(T2_INT_PRIOR_4 &			// 0~7 => 최소 1 이상 설정할 것
				T2_INT_OFF);					// ON, OFF

	WriteTimer2(0);

	OpenTimer2(	T2_ON &						// ON, OFF
				T2_IDLE_STOP &					// CON, STOP
				T2_GATE_OFF &					// ON, OFF
				T2_PS_1_256 &				 	// 1_1, 1_8, 1_64, 1_128

				T2_SOURCE_INT,	         	// EXT, INT
				8000);	// pwm 전체 주기에 해당 
				// (8M * PLL 4) / 4 Fix ) * 8000 N = 1ms
				// 0.125us * 8000 = 1ms
				// 1ms*256 = 256ms 주기 
}



///////////////////////////////////////////////////////////////
unsigned char float2string100(float val)
{
	unsigned char retval;

	if((val >= 1000.0) || (val < 100.0))
	{
		retval = ' ';
	}
	else
	{
		retval = (unsigned char)(val / 100) + '0';
	}
	
	return(retval);
}

unsigned char float2string10(float val)
{
	unsigned char retval;
	unsigned int tempval;
	
	tempval = (unsigned int)val;

	if(tempval < 10)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= 100)
	{
		tempval = tempval % 100;
	}

	if(tempval >= 10)
	{
		tempval = tempval / 10;
	}
		
	retval = (unsigned char)tempval + '0';
	
	return(retval);
}

unsigned char float2string1(float val)
{
	unsigned char retval;
	unsigned int tempval;

	tempval = (unsigned int)val;

	if(tempval >= 100)
	{
		tempval = tempval % 100;
	}
	
	if(tempval >= 10)
	{
		tempval = tempval % 10;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

void control_led(void)
{
	if((rawdata_pm2_5_atmos <= PM2_5_GOOD) && (rawdata_pm10_atmos <= PM10_GOOD))
	{
		LED_RED = 0;
		LED_YELLOW = 0;
		LED_GREEN = 1;
		status_pm = PM_GOOD;	// _debug
	}
	else if((rawdata_pm2_5_atmos <= PM2_5_MID) && (rawdata_pm10_atmos <= PM10_MID))
	{
		LED_RED = 0;
		LED_YELLOW = 1;
		LED_GREEN = 0;
		status_pm = PM_MID;
	}
	else if((rawdata_pm2_5_atmos <= PM2_5_BAD) && (rawdata_pm10_atmos <= PM10_BAD))
	{
		LED_RED = 1;
		LED_YELLOW = 0;
		LED_GREEN = 0;
		status_pm = PM_BAD;
	}
	else
	{
		LED_RED = 1;
		LED_YELLOW = 1;
		LED_GREEN = 1;
		status_pm = PM_ERROR;
	}
}

void check_sw(void)
{
	static int key_cnt_sw1 = 0;
	static int key_cnt_sw2 = 0;
	
	if(IN_SW1 == 0)
	{
		key_cnt_sw2 = 0;
	
		if(key_cnt_sw1 >= 100)
		{
			fan_automode_flag = NONAUTOMODE;	// SW1 눌려지면 NONAUTOMODE로 SW1 상태에 의해 FAN 강제 제어 
			control_fan_flag = 1;
		}
		else
		{	
			key_cnt_sw1++;
		}
	}
	else if(IN_SW2 == 0)// && (fan_automode_flag != NONAUTOMODE))
	{
		key_cnt_sw1 = 0;
		
		if(key_cnt_sw2 >= 100)	// SW2 눌려지면 PM값을 ISR에서 보고 60초 누적시 FAN 제어 결정 
		{
			fan_automode_flag = AUTOMODE;
			// control_fan_flag 는 ISR에서 automode를 보고 결정 
		}
		else
		{	
			key_cnt_sw2++;
		}
	}
	else
	{
		key_cnt_sw1 = 0;
		key_cnt_sw2 = 0;
		fan_automode_flag = INIT;
		control_fan_flag = 0;
	}
}

void control_fan(void)
{
	if(control_fan_flag == 1)
	{
		CONTROL_FAN = 1;
	}
	else
	{
		CONTROL_FAN = 0;
	}
}