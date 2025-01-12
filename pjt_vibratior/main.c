//////////////////////////////////////////////////////////////////////////////////////
// 210502 진동시험기
// - OC 250us 주기 pwm발생
// - Timer3 debugging용 TP 500ms 주기 확인
// SD2C Step motor driver를 이용해 왕복운동 함 
// uart protocol, tx만 사용 
// uart tx 제어시 운전 중 값 변경하면 모터 멈춤 현상 발생, 드라이버의 uart 제어 변경하면 재현됨으로 추정  

//////////////////////////////////////////////////////////////////////////////////////


#include <p30f3013.h>
#include <stdio.h>
#include <uart.h>
#include <timer.h>
#include <outcompare.h>

#include "cLCD.h"

#include "delay.h"


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
#define OUT_EN		_LATB6
#define OUT_DIR		_LATB7
#define OUT_PULSE	_LATB8

#define LED_INDICATOR			_RD8
#define LED_INDICATOR_STATUS	_LATD8

//////////////////////////////////////////////////////////////////
void io_init(void);
void uart_init(void);
void timer_init(void);
void oc_init(void);

//////////////////////////////////////////////////////////////////

/* SYSTEM VAR START */
unsigned int gu16_timer_cnt = 0;
unsigned char received_data = 0;

unsigned char buf[32];
unsigned char *Receivedddata = buf;

unsigned char buf_result[32];

unsigned char receive_result_flag = 0;

unsigned char rx_receive_cnt, rx_length = 0;
/* SYSTEM VAR END */

unsigned int gu16_motor_timer_cnt = 0;
unsigned char gu8_change_dir = 0;
unsigned char gu8_state = 0;

//////////////////////////////////////////////////////////////////
void __attribute__ ((__interrupt__)) _U1TXInterrupt(void)  //uart
{
	IFS0bits.U1TXIF=0;
}

// RX로 처리 데이터 없음으로 주석 처리 
void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	unsigned char i = 0;
	
	IFS0bits.U1RXIF=0;
	//received_data=U1RXREG;	// ReadUART1(); 효과임으로 중복 사용 불가 
/*
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
	}*/
}

// 500us Timer
void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	static int automode_fan_on_cnt, automode_fan_off_cnt = 0;
	
	WriteTimer3(0);

	IFS0bits.T3IF=0;

	gu16_timer_cnt++;
						
#ifdef UART_ENABLE
		//printf("%d\n",gu16_timer_cnt);
#endif

	if(gu16_timer_cnt >= 1000)	// 500ms 마다 toggling
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

	gu16_motor_timer_cnt++;

	if(gu16_motor_timer_cnt >= (1000*1))
	{
		gu16_motor_timer_cnt = 0;
		if(gu8_change_dir == 0)
		{
			gu8_change_dir = 1;
			gu8_state = 1;
		}
		else
		{
			gu8_change_dir = 0;
			gu8_state = 2;
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

#ifdef UART_ENABLE
	uart_init();
#endif

	timer_init();
	//oc_init();
	

#ifdef UART_ENABLE
	//printf("UART TEST Okay!\r\n");
#endif
	Delay_ms(10);

	LED_INDICATOR = 1;

	// UART TX 확인용, SD2C 드라이버 메뉴얼 참고  
	//printf("O");
	//Delay_ms(10);	
	//printf(CMD_STEPMOTOR_RUN);
	//Delay_ms(10);
	//printf(CMD_SET_SPEED);
	//Delay_ms(10);
	//printf("%d\n", CMD_SET_SPEED_VAL);
	
	WriteUART1(0x4F);
	Delay_ms(1);	
	WriteUART1(0x47);
	Delay_ms(11);

	WriteUART1(0x53);
	Delay_ms(1);
	WriteUART1(0xCC);
	Delay_ms(1);

	while(1)
	{	
		// 주기적으로 왕복하도록 UART TX 해줌 
		switch(gu8_state)
		{
			case 0:
			break;

			case 1:
			case 2:
				gu8_state = 0;
				if(gu8_change_dir == 0)
				{
					WriteUART1(0x66);
					Delay_us(10);
					WriteUART1(0x3C);	// CCW (DEFAULT)
					Delay_us(10);
					WriteUART1(0x71);

				}
				else
				{
					WriteUART1(0x66);
					Delay_us(10);
					WriteUART1(0x3E);	// CW
					Delay_us(10);
					WriteUART1(0x71);
				}
			break;

			default:
			break;
		}
	}
}

//////////////////////////////////////////////////////////////////
void io_init(void)
{
	//3 방향설정  
	_TRISD8 = 0;
	
	//3 초기값 
	_RD8 = 0;

	// Application PIN
	_TRISB6 = 0;
	_TRISB7 = 0;
	_TRISB8 = 0;

	// uart cable에 의한 mcu 오동작 여부 확인 필요 
	_TRISC13 = 0;
	_TRISC14 = 1;

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
				// (8M * PLL 8) / 4 Fix ) * 8000 N
				// 0.0625us * 8000 = 500us

}

void oc_init(void)
{
//2 250us 주기 50% duty의 pwm 발생

//OC1R = 4000;	// 사전 초기화 guide 존재하나 없어도 동작함

	ConfigIntOC1(OC_INT_OFF & OC_INT_PRIOR_5);	// ISR 수행안함 

	OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC &
			OC_PWM_FAULT_PIN_DISABLE,	// OC_PWM_FAULT_PIN_DISABLE: pwm방식 (OC1R<-OC1SR시 OC High, overflow시 OC Low)
			2000, 2000);	// OCxRS(Secondary Register) , OCxR (Main Register) ; pwm duty cycle에 해당 

	ConfigIntTimer2(T2_INT_PRIOR_4 &			// 0~7 => 최소 1 이상 설정할 것
				T2_INT_OFF);					// ON, OFF

	WriteTimer2(0);

	OpenTimer2(	T2_ON &						// ON, OFF
				T2_IDLE_STOP &					// CON, STOP
				T2_GATE_OFF &					// ON, OFF
				T2_PS_1_1 &					// 1_1, 1_8, 1_64, 1_128

				T2_SOURCE_INT,	         	// EXT, INT
				4000);	// pwm 전체 주기에 해당 	
				// (8M * PLL 8) / 4 Fix ) * 4000 N
				// 0.0625us * 4000 = 250us (4000 변경시 OCxRS, OCxR on/off 주기 함게 변경 )
				// 0.0625us * 12000 = -> 1.2khz
				// 0.0625us * 16000 = 1ms -> 1khz
				// 0.0625us * 20000 = -> 800hz

}



///////////////////////////////////////////////////////////////

