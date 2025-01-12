//////////////////////////////////////////////////////////////////////////////////////
//180324
// UART Test

// PORT8 : output으로 설정 후 상태판단으로 RD 사용 X -> 이 경우 _LATD8 로 사용 
// Uart init 후 UART Cable 삭제하면 동작 안함 
//////////////////////////////////////////////////////////////////////////////////////


#include <p30f3013.h>
#include <stdio.h>
#include <uart.h>
#include <timer.h>


// 사용안할 시 내부 osc로 설정됨, IDE>Configure 에서 확인가능 
_FOSC(CSW_FSCM_OFF & ECIO_PLL4);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);      

#define FCY  8000000 
#define BAUDRATE 	19200        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1           

enum{TEMP, HUMI};

#define UART_ENABLE


//////////////////////////////////////////////////////////////////
void io_init(void);
void uart_init(void);
void timer_init(void);

//////////////////////////////////////////////////////////////////
unsigned int gu16_timer_cnt = 0;
unsigned char received_data = 0;

unsigned char buf[80];
unsigned char *Receivedddata = buf;

//////////////////////////////////////////////////////////////////
void __attribute__ ((__interrupt__)) _U1TXInterrupt(void)  //uart
{
	IFS0bits.U1TXIF=0;
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF=0;
	//received_data=U1RXREG;	// ReadUART1(); 효과임으로 중복 사용 불가 

	while( DataRdyUART1())
	{
	( *(Receivedddata)++) = ReadUART1();
	}

}

// 1ms Timer
void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	
	WriteTimer3(0);

	IFS0bits.T3IF=0;

	gu16_timer_cnt++;
			
#ifdef UART_ENABLE
			//printf("%d\n",gu16_timer_cnt);
#endif

	if(gu16_timer_cnt >= 1000)
	{
		gu16_timer_cnt = 0;
		if(_LATD8 == 0)
		{
			_RD8 = 1;
		}
		else
		{
			_RD8 = 0;
		}
	}
}

int main(void)
{

	io_init();

#ifdef UART_ENABLE
	uart_init();
#endif

	timer_init();
	

#ifdef UART_ENABLE
	printf("UART TEST Okay!\r\n");
#endif

	//_RD8 = 1;
	_RD9 = 1;

	while(1)
	{
	
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
				// (8M * PLL 4) / 4 Fix ) * 8000 N = 1ms
				// 0.125us * 8000 = 1ms

}
//////////////////////////////////////////////////////////////////
