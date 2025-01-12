//////////////////////////////////////////////////////////////////////////////////////
//211230 수명 가속기 (accelerate life tester)
// lsd는 on, hsd는 2초 ON, 2초 OFF (EX) 무한 반복 
// fail safe 발생시 hsd, lsd off, system enable sw 다시 on 시 cycle cnt는 초기화, hsd만 제어 (실제 전류는 안흐름)
// sw1 : system enable & cycle cnt clear, hsd 재기동 
// sw2 : RSV
// PORTF는 SW INPUT 사용시 RF를 사용해야 함 (LATF X) - 3013
// PORTB는 INPUT 사용시 LATB?
// lsd는 초기 on 
// indicaton led 1개 toggle
//////////////////////////////////////////////////////////////////////////////////////


#include <p30f3013.h>
#include <stdio.h>
#include <uart.h>
#include <timer.h>
#include <outcompare.h>

#include "cLCD.h"
#include "Delay.h"


// 사용안할 시 내부 osc로 설정됨, IDE>Configure 에서 확인가능 
_FOSC(CSW_FSCM_OFF & ECIO_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);      

#define FCY  8000000 
#define BAUDRATE 	19200
//#define FCY  16000000        
//#define BAUDRATE 	9600        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1           

//#define UART_ENABLE

//3 PORTB는 Output으로 사용시 PB(RB)가 아닌 LATB로 사용할 것 
#define LED_INDICATOR			_RD8
#define LED_INDICATOR_STATUS	_LATD8


// OUTPORT
#define OUT_RSV_B3	_LATB3	// buzzer 사용 
#define OUT_RSV_B4	_LATB4	// lsd 사용 
#define OUT_RSV_B5	_LATB5

#define OUT_F4_RLY_CTRL	_RF4

// INPORT
//#define IN_SW1		_LATF5	// system enable && count clear
//#define IN_SW2		_LATF6	// load status sensor
#define IN_SW1		_RF5	// system enable && count clear
#define IN_SW2		_RF6	// load status sensor



#define NUM_10_BILI		(1000000000)
#define NUM_1_BILI		(100000000)
#define NUM_1000_MILI	(10000000)
#define NUM_100_MILI	(1000000)
#define NUM_10_MILI		(100000)
#define NUM_1_MILI		(10000)
#define NUM_1000		(1000)
#define NUM_100			(100)
#define NUM_10			(10)
#define NUM_1			(1)

#define NUM_2_64BIT		(4294967295)

#define BASE_1SEC		(2000)

#define RLY_ON_TIME			(4000)	// 2SEC
#define RLY_OFF_TIME		(RLY_ON_TIME * 2)
#define RLY_FAIL_SENSING	(RLY_ON_TIME*6)	// 8SEC (= 3 cycle)







//////////////////////////////////////////////////////////////////
void io_init(void);
void uart_init(void);
void timer_init(void);
void oc_init(void);

unsigned char float2string10_Bili(unsigned long);
unsigned char float2string1_Bili(unsigned long);
unsigned char float2string1000_Mili(unsigned long);
unsigned char float2string100_Mili(unsigned long);
unsigned char float2string10_Mili(unsigned long);
unsigned char float2string1_Mili(unsigned long);
unsigned char float2string1000(unsigned long);
unsigned char float2string100(unsigned long);
unsigned char float2string10(unsigned long);
unsigned char float2string1(unsigned long);

void check_sw_app(void);
void check_sw_isr(void);
void ctrl_output_buzzer(void);

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
unsigned int gu16_timer_cnt_app = 0;

unsigned char received_data = 0;

unsigned char buf[32];
unsigned char *Receivedddata = buf;

unsigned char buf_result[32];

char buf_char_lcd_doc01[11];
unsigned long gu32_lcd_debug = 0;

//////////////////////////////////////////////////////////////////

unsigned int gu16_rly_ctrl_on = 0;
unsigned long gu32_rly_ctrl_cyclecnt = 0;

unsigned char gu8_system_enable = 0;
unsigned char gu8_fault_detect = 0;
unsigned long gu32_buzzer_timer = 0;

unsigned int gu16_sw_beep01 = 0;

//////////////////////////////////////////////////////////////////
void __attribute__ ((__interrupt__)) _U1TXInterrupt(void)  //uart
{
	IFS0bits.U1TXIF=0;
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
#if 0
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
#endif
}

// 500us Timer
void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	static int automode_fan_on_cnt, automode_fan_off_cnt = 0;
	
	WriteTimer3(0);

	IFS0bits.T3IF=0;

	gu16_timer_cnt++;

	_RD9 = ~_RD9;

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
	}

	if(gu8_system_enable == 1)
	{
		gu16_timer_cnt_app++;

		if(gu16_timer_cnt_app >= RLY_ON_TIME)
		{
			gu16_rly_ctrl_on = 1;
			OUT_F4_RLY_CTRL = 1;	// timing 오차 최소화 위해 ISR에서 직접 제어 
			
			if(gu16_timer_cnt_app >= RLY_OFF_TIME)
			{
				gu16_timer_cnt_app = 0;
				gu16_rly_ctrl_on = 0;
				OUT_F4_RLY_CTRL = 0;

				gu32_rly_ctrl_cyclecnt++;
				if(gu32_rly_ctrl_cyclecnt >= NUM_10_BILI*4)
				{
					gu32_rly_ctrl_cyclecnt = NUM_10_BILI*4 - 1;
				}
			}
		}

		check_sw_isr();
	}

	gu32_buzzer_timer++;
	if(gu32_buzzer_timer >= (long)BASE_1SEC*60)
	{
		gu32_buzzer_timer = 0;
	}

	check_sw_app();
	if(gu16_sw_beep01 != 0)
	{
		gu16_sw_beep01--;
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
	//_RD9 = 1;
   //_RF4 = 1;	// _debug

           //"1234567890123456"
	Wrt_S_LCD("ACLERT LIFE TEST", 0, 0);
	//Wrt_S_LCD("1234567890123456", 0, 1);

	OUT_RSV_B4 = 1;

	while(1)
	{	
		ctrl_output_buzzer();

		// COUNT [0000000000 ~ 3999999999]
		//gu32_lcd_debug = 4294967295;
		//gu32_lcd_debug = 3294967295;
		//gu32_lcd_debug = 294967295;
		//gu32_lcd_debug = 900967205;
		gu32_lcd_debug = 0;
		if(gu32_lcd_debug >= NUM_10_BILI*4)
		{
			gu32_lcd_debug = NUM_10_BILI*4 - 1;
		}
		buf_char_lcd_doc01[0] = float2string10_Bili(gu32_rly_ctrl_cyclecnt);	// gu32_rly_ctrl_cyclecnt upper limit는 isr에서 수행 
		buf_char_lcd_doc01[1] = float2string1_Bili(gu32_rly_ctrl_cyclecnt);
		buf_char_lcd_doc01[2] = float2string1000_Mili(gu32_rly_ctrl_cyclecnt);
		buf_char_lcd_doc01[3] = float2string100_Mili(gu32_rly_ctrl_cyclecnt);
		buf_char_lcd_doc01[4] = float2string10_Mili(gu32_rly_ctrl_cyclecnt);
		buf_char_lcd_doc01[5] = float2string1_Mili(gu32_rly_ctrl_cyclecnt);
		buf_char_lcd_doc01[6] = float2string1000(gu32_rly_ctrl_cyclecnt);
		buf_char_lcd_doc01[7] = float2string100(gu32_rly_ctrl_cyclecnt);
		buf_char_lcd_doc01[8] = float2string10(gu32_rly_ctrl_cyclecnt);
		buf_char_lcd_doc01[9] = float2string1(gu32_rly_ctrl_cyclecnt); 
		buf_char_lcd_doc01[10] = '\0';

		Wrt_S_LCD(buf_char_lcd_doc01, 6, 1);

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

	// OUT
	_TRISB3 = 0;
	_TRISB4 = 0;
	_TRISB5 = 0;

	_TRISF4 = 0;

	// INPUT SW
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
				8000);						// Match_value	
				// (8M * PLL 4) / 4 Fix ) * 8000 N = 1ms
				// 0.125us * 8000 = 1ms

				// (8M * PLL 8) / 4 Fix ) * 8000 N = 500us
				// 0.0625us * 8000 = 500us [적용]

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
unsigned char float2string10_Bili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	// input 범위 : 40억 미만 
	if((tempval >= NUM_10_BILI * 4) || (tempval < NUM_10_BILI))
	{
		tempval = ' ';
		return(tempval);
	}
	else
	{
		tempval = tempval / NUM_10_BILI;
	}

	retval = (unsigned char)tempval + '0';
	
	return(retval);
}

unsigned char float2string1_Bili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;
	
	if(tempval < NUM_1_BILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval / NUM_1_BILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char float2string1000_Mili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_1000_MILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval / NUM_1000_MILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char float2string100_Mili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_100_MILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}

	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval / NUM_100_MILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char float2string10_Mili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_10_MILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}

	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval / NUM_10_MILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char float2string1_Mili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_1_MILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}

	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval / NUM_1_MILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char float2string1000(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_1000)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}
	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval % NUM_1_MILI;
	}
	
	if(tempval >= NUM_1000)
	{
		tempval = tempval / NUM_1000;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char float2string100(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_100)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}
	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval % NUM_1_MILI;
	}
	if(tempval >= NUM_1000)
	{
		tempval = tempval % NUM_1000;
	}
	
	if(tempval >= NUM_100)
	{
		tempval = tempval / NUM_100;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char float2string10(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_10)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}
	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval % NUM_1_MILI;
	}
	if(tempval >= NUM_1000)
	{
		tempval = tempval % NUM_1000;
	}
	if(tempval >= NUM_100)
	{
		tempval = tempval % NUM_100;
	}
	
	if(tempval >= NUM_10)
	{
		tempval = tempval / NUM_10;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char float2string1(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;
	
	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}
	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval % NUM_1_MILI;
	}
	if(tempval >= NUM_1000)
	{
		tempval = tempval % NUM_1000;
	}
	if(tempval >= NUM_100)
	{
		tempval = tempval % NUM_100;
	}
	if(tempval >= NUM_10)
	{
		tempval = tempval % NUM_10;
	}
	
	if(tempval >= NUM_1)
	{
		tempval = tempval / NUM_1;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

void check_sw_app(void)
{
	static int key_cnt_sw1 = 0;
	static int key_cnt_sw2 = 0;

	if(IN_SW1 == 0)	// fail safe mode 진입 후 복귀 방지 위해 default = 1 이어야 함 
	{
		if(key_cnt_sw1 >= 0) // cap 2.2uF 적용 & isr에서 바른 주기로 감지함 -> sw filter 불필요 
		{
			key_cnt_sw1 = 0;
			gu8_system_enable = 1; 
			gu32_rly_ctrl_cyclecnt = 0;

			gu16_sw_beep01 = 100;

		}
		else
		{	
			key_cnt_sw1++;
		}
	}
	else
	{
		key_cnt_sw1 = 0;
	}
}

void check_sw_isr(void)
{
	static int key_cnt_sw1 = 0;
	static int key_cnt_sw2 = 0;
	static int key_cnt_sw3 = 0;

	// fail safe 발생시 수행안함 

	if(IN_SW2 == 0)
	{
		key_cnt_sw3 = 0;
		if(key_cnt_sw2 >= RLY_FAIL_SENSING)
		{
			key_cnt_sw2 = 0;
			
			// fail safe mode
			gu8_system_enable = 0;
			gu8_fault_detect = 1;

			OUT_F4_RLY_CTRL = 0;
			OUT_RSV_B4 = 0;	// lsd off

			gu16_sw_beep01 = 1000;
		}
		else
		{	
			key_cnt_sw2++;
		}
	}
	else
	{
		key_cnt_sw2 = 0;
		
		if(key_cnt_sw3 >= RLY_FAIL_SENSING)
		{
			key_cnt_sw3 = 0;

			// fail safe mode
			gu8_system_enable = 0;
			gu8_fault_detect = 1;

			OUT_F4_RLY_CTRL = 0;
			OUT_RSV_B4 = 0;	// lsd off

			gu16_sw_beep01 = 1000;
		}
		else
		{	
			key_cnt_sw3++;
		}
	}
	
}

void ctrl_output_buzzer(void)
{
	// 1초 : ON, 59초 : OFF
	if( ((gu8_fault_detect == 1) && (gu32_buzzer_timer >= (long)BASE_1SEC*59)) 
		|| (gu16_sw_beep01 != 0))
	//if( (gu32_buzzer_timer >= (long)BASE_1SEC*59) || (gu16_sw_beep01 != 0))
	{
		OUT_RSV_B3 = 1;
	}
	else
	{
		OUT_RSV_B3 = 0;
	}

	#if 0
	if(gu16_sw_beep01 != 0)
	{
		OUT_RSV_B3 = 1;
	}
	else
	{
		OUT_RSV_B3 = 0;
	}
	#endif
}

