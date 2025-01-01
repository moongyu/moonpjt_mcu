//////////////////////////////////////////////////////////////////////////////////////
// eeprom 기능 test
//////////////////////////////////////////////////////////////////////////////////////


#include <p30f3013.h>
#include <stdio.h>
#include <uart.h>
#include <timer.h>
#include <outcompare.h>

#include <p30fxxxx.h>
#include <libpic30.h>

#include "cLCD.h"
#include "Delay.h"


// 사용안할 시 내부 osc로 설정됨, IDE>Configure 에서 확인가능 
_FOSC(CSW_FSCM_OFF & ECIO_PLL8);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);      

// 38400bps 확인
#define FCY  8000000 
#define BAUDRATE 	19200

// 9600bps 확인 
#define FCY  16000000        
#define BAUDRATE 	9600        
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

//////////////////////////////////////////////////////////////////
void io_init(void);
void uart_init(void);
void timer_init(void);
void oc_init(void);

//////////////////////////////////////////////////////////////////


unsigned int gu16_timer_cnt = 0;
unsigned int gu16_timer_cnt_app = 0;

unsigned char received_data = 0;

unsigned char buf[32];
unsigned char *Receivedddata = buf;

unsigned char buf_result[32];

char buf_char_lcd_doc01[11];
unsigned long gu32_lcd_debug = 0;

//////////////////////////////////////////////////////////////////

/*Declare constants/coefficients/calibration data to be stored in DataEEPROM*/
// 32 byte 만큼 eeprom 공간에 data 즉시 저장 
// eeprom에 데이터 없으면 저장, al.
int _EEDATA(32) fooArrayInDataEE[] = {0,1,2,3,4,5,6,7,8,9,0xA,0xB,0xC,0xD,0xE,0xF};
int _EEDATA(32) fooArrayInDataEE_1[] = {0x10, 0x11};


/*Declare variables to be stored in RAM*/
int fooArray1inRAM[] = {0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF, 0xABCD, 0xBCDE,0xCDEF, 0xDEFA, 0x0000, 0x1111, 0x2222, 0x3333, 0x4444, 0x5555};

int fooArray2inRAM[16];

int test_eeprom_array[16];


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

}

void __attribute__((__interrupt__)) _OC1Interrupt(void)
{
	// ISR 안걸리는 현상 확인필요 (TBD)
	
	IFS0bits.OC1IF = 0;
	//OC1R = 4000;		// OC1IF 후 Reload하라고 guide 존재하나 없어도 동작함
}

unsigned int gu16_resetcause = 0;
unsigned char gu8_resetcause_monitor01, gu8_resetcause_monitor02 = 0;

int main(void)
{

	_prog_addressT EE_addr;  
	int temp = 0;    

	/* initialize a variable to represent the Data EEPROM address */ 
	// EE_addr 에 fooArrayInDataEE 주소값 가르킴 
	_init_prog_address(EE_addr, fooArrayInDataEE);        

	/*Copy array "fooArrayinDataEE" from DataEEPROM to "fooArray2inRAM" in RAM*/    
	// fooArray2inRAM 램변수로 읽어오기 
	_memcpy_p2d16(fooArray2inRAM, EE_addr, _EE_ROW);   

	// 이 전에 0x5a5a 로 저장 이력이 존재하면 [14]에 0x5a5a 로 confirm
	if(fooArray2inRAM[0] == 0x5a5a)
	{
		fooArray1inRAM[14] = 0x5a5a;
	}
	else
	{
		// 그렇지않으면 최초 부팅임으로 [14]에 기존 램 변수값 저장 	
	}

	/*Erase a row in Data EEPROM at array "fooArrayinDataEE" */    
	// eeprom 지우기 
	_erase_eedata(EE_addr, _EE_ROW);    
	_wait_eedata();   

	/*Write a row to Data EEPROM from array "fooArray1inRAM" */ 
	// fooArray1inRAM 램변수의 값을 eeprom 저장 
	_write_eedata_row(EE_addr, fooArray1inRAM);    
	_wait_eedata();  


	/****************************************************/
	_memcpy_p2d16(test_eeprom_array, EE_addr, _EE_ROW);
	_wait_eedata();  

	// 값 변경 후 저장 
	fooArray1inRAM[0] = 0x5a5a;
	fooArray1inRAM[1] = 0x2;
	fooArray1inRAM[2] = 0x3;
	fooArray1inRAM[3] = 0xAAAA;
	fooArray1inRAM[4] = 0xff;
	fooArray1inRAM[5] = 0x0100;
	fooArray1inRAM[15] = 0x5678;

	_erase_eedata(EE_addr, _EE_ROW);    // wirte 전에 반드시 erase 필요 
	_wait_eedata(); 

	_write_eedata_row(EE_addr, fooArray1inRAM);    
	_wait_eedata();

	// 잘 저장되언는지 확인 
	_memcpy_p2d16(test_eeprom_array, EE_addr, _EE_ROW);
	_wait_eedata();  
	
	while(1); /* Place a breakpoint here, run code and refresh the DataEEPROM window in MPLAB IDE */

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

