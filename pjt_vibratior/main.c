//////////////////////////////////////////////////////////////////////////////////////
// 210502 ���������
// - OC 250us �ֱ� pwm�߻�
// - Timer3 debugging�� TP 500ms �ֱ� Ȯ��
// SD2C Step motor driver�� �̿��� �պ�� �� 
// uart protocol, tx�� ��� 
// uart tx ����� ���� �� �� �����ϸ� ���� ���� ���� �߻�, ����̹��� uart ���� �����ϸ� ���������� ����  

//////////////////////////////////////////////////////////////////////////////////////


#include <p30f3013.h>
#include <stdio.h>
#include <uart.h>
#include <timer.h>
#include <outcompare.h>

#include "cLCD.h"

#include "delay.h"


// ������ �� ���� osc�� ������, IDE>Configure ���� Ȯ�ΰ��� 
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

//3 PORTB�� Output���� ���� PB(RB)�� �ƴ� LATB�� ����� �� 
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

// RX�� ó�� ������ �������� �ּ� ó�� 
void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	unsigned char i = 0;
	
	IFS0bits.U1RXIF=0;
	//received_data=U1RXREG;	// ReadUART1(); ȿ�������� �ߺ� ��� �Ұ� 
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
			&& (buf[29] == 0x00))	// data ������ üũ 
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

	if(gu16_timer_cnt >= 1000)	// 500ms ���� toggling
	{
		gu16_timer_cnt = 0;

		if(LED_INDICATOR_STATUS == 0)	//3 Port�� ���� Ȯ�� ���ؼ� LATD ���� 
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
	// ISR �Ȱɸ��� ���� Ȯ���ʿ� (TBD)
	
	IFS0bits.OC1IF = 0;
	//OC1R = 4000;		// OC1IF �� Reload�϶�� guide �����ϳ� ��� ������
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

	// UART TX Ȯ�ο�, SD2C ����̹� �޴��� ����  
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
		// �ֱ������� �պ��ϵ��� UART TX ���� 
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
	//3 ���⼳��  
	_TRISD8 = 0;
	
	//3 �ʱⰪ 
	_RD8 = 0;

	// Application PIN
	_TRISB6 = 0;
	_TRISB7 = 0;
	_TRISB8 = 0;

	// uart cable�� ���� mcu ������ ���� Ȯ�� �ʿ� 
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
/*	ConfigIntTimer1(T1_INT_PRIOR_7 &			// 0~7 => �ּ� 1 �̻� ������ ��	
				T1_INT_ON);					// ON, OFF

	WriteTimer1(0);

	OpenTimer1(	T1_ON &						// ON, OFF
				T1_IDLE_STOP &					// CON, STOP
				T1_GATE_OFF &					// ON, OFF
				T1_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128
				T1_SYNC_EXT_OFF &  			// ON, OFF (ON �ʼ�)
				T1_SOURCE_INT,	         	// EXT, INT
				1600);						// Match_value		// 100us �ֱ�
	*/
	
	ConfigIntTimer3(T3_INT_PRIOR_3 &			// 0~7 => �ּ� 1 �̻� ������ ��		// 190227 �켱���� 7 �� ����
				T3_INT_ON);					// ON, OFF

	WriteTimer3(0);

	OpenTimer3(	T3_ON &						// ON, OFF
				T3_IDLE_STOP &					// CON, STOP
				T3_GATE_OFF &					// ON, OFF
				T3_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128

				T3_SOURCE_INT,	         	// EXT, INT
				8000);						// Match_value		// 1.0ms �ֱ�		
				// (8M * PLL 8) / 4 Fix ) * 8000 N
				// 0.0625us * 8000 = 500us

}

void oc_init(void)
{
//2 250us �ֱ� 50% duty�� pwm �߻�

//OC1R = 4000;	// ���� �ʱ�ȭ guide �����ϳ� ��� ������

	ConfigIntOC1(OC_INT_OFF & OC_INT_PRIOR_5);	// ISR ������� 

	OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC &
			OC_PWM_FAULT_PIN_DISABLE,	// OC_PWM_FAULT_PIN_DISABLE: pwm��� (OC1R<-OC1SR�� OC High, overflow�� OC Low)
			2000, 2000);	// OCxRS(Secondary Register) , OCxR (Main Register) ; pwm duty cycle�� �ش� 

	ConfigIntTimer2(T2_INT_PRIOR_4 &			// 0~7 => �ּ� 1 �̻� ������ ��
				T2_INT_OFF);					// ON, OFF

	WriteTimer2(0);

	OpenTimer2(	T2_ON &						// ON, OFF
				T2_IDLE_STOP &					// CON, STOP
				T2_GATE_OFF &					// ON, OFF
				T2_PS_1_1 &					// 1_1, 1_8, 1_64, 1_128

				T2_SOURCE_INT,	         	// EXT, INT
				4000);	// pwm ��ü �ֱ⿡ �ش� 	
				// (8M * PLL 8) / 4 Fix ) * 4000 N
				// 0.0625us * 4000 = 250us (4000 ����� OCxRS, OCxR on/off �ֱ� �԰� ���� )
				// 0.0625us * 12000 = -> 1.2khz
				// 0.0625us * 16000 = 1ms -> 1khz
				// 0.0625us * 20000 = -> 800hz

}



///////////////////////////////////////////////////////////////

