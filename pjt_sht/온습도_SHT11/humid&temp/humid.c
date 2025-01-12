#include <p30f4012.h>
#include "lcd.h"
#include <stdio.h>
#include <math.h>
#include <uart.h>
//#include <adc10.h>
#include <float.h>


_FOSC(CSW_FSCM_OFF & EC_PLL8);
_FWDT(WDT_OFF);      
//_FBORPOR(MCLR_EN & PWRT_OFF);   
//_FGS(CODE_PROT_OFF);    

#define FCY  16000000 
#define BAUDRATE 	19200        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1           

typedef union
{
	unsigned int i;
	float f;
}value;

enum{TEMP, HUMI};

//#define DATA LATBbits.LATB5
//#define SCK LATDbits.LATD0

#define DATA PORTBbits.RB5
#define SCK PORTDbits.RD0


#define noACK 0
#define ACK 1

#define STATUS_REG_W 0x06
#define STATUS_REG_R 0x07
#define MEASURE_TEMP 0x03
#define MEASURE_HUMI 0x05
#define RESET 0x1e

void __attribute__ ((__interrupt__)) _U1TXInterrupt(void)  //uart
{
	IFS0bits.U1TXIF=0;
}



char s_write_byte(unsigned char value)
{
	TRISBbits.TRISB5=0;   //data
	unsigned char i, error = 0;
	for(i=0x80;i>0;i/=2)
	{
		if(i & value) DATA=1;
		else DATA=0;
		SCK=1;
		Delay_us(15);
		SCK=0;
		//Delay_us(5);
	}
	DATA=1;
	TRISBbits.TRISB5=1;   //data
	SCK=1;
	error=DATA;
	SCK=0;
	return error;
}

char s_read_byte(unsigned char ack)
{
	unsigned char i, val=0;
	DATA=1;
	TRISBbits.TRISB5=1;  //data
	for(i=0x80 ; i>0 ; i/=2)
	{
		SCK=1;
		if(DATA) val=(val | i);
		SCK = 0;
	}
	TRISBbits.TRISB5=0;   //data
	DATA=!ack;
	SCK=1;
	Delay_us(15);
	SCK=0;
	DATA=1;
	return val;
}

void s_transstart(void)
{
	TRISBbits.TRISB5=0;   //data
	DATA=1; SCK=0;
	Delay_us(5);
	SCK=1;
	Delay_us(5);
	DATA=0;
	Delay_us(5);
	SCK=0;
	Delay_us(15);   //
	SCK=1;
	Delay_us(5);
	DATA=1;
	Delay_us(5);
	SCK=0;
	TRISBbits.TRISB5=1;   //data
}

void s_connectionreset(void)
{
	TRISBbits.TRISB5=0;   //data
	unsigned char i;
	DATA=1; SCK=0;
	for(i=0; i<9;i++)
	{
		SCK=1;
		SCK=0;
	}
	s_transstart();
//	TRISBbits.TRISB5=1;   //data
}

char s_softreset(void)
{
	unsigned char error=0;
	s_connectionreset();
	error+=s_write_byte(RESET);
	return error;
}

char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum)
{
	unsigned char error=0;
	s_transstart();
	error=s_write_byte(STATUS_REG_R);
	*p_value=s_read_byte(ACK);
	*p_checksum=s_read_byte(noACK);
	return error;
}

char s_write_statusreg(unsigned char *p_value)
{
	unsigned char error=0;
	s_transstart();
	error+=s_write_byte(STATUS_REG_W);
	error+=s_write_byte(*p_value);
	return error;
}

char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode)
{
	unsigned error=0;
	//unsigned long i;
	unsigned int i;
	s_transstart();
	switch(mode)
	{
		case TEMP: 
		error+=s_write_byte(MEASURE_TEMP); 
		break;

		case HUMI:
		error+=s_write_byte(MEASURE_HUMI);
		break;

		default: break;
	}
	TRISBbits.TRISB5=1;   //data
	for(i=0; i<65535;i++)
	//Delay_us(10);
	//for(i=0; i<500000;i++)
	if(DATA==0) break;
	
	if(DATA) error+=1;
	*(p_value) = s_read_byte(ACK);
	*(p_value+1)=s_read_byte(ACK);
	*p_checksum=s_read_byte(noACK);
	return error;
}

/*
void init_uart()
{
	SCON = 0x52;
	TMOD = 0x20;
	TCON = 0x69;
	TH1 = 0xfd;
}
*/

void calc_sthll(float *p_humidity, float *p_temperature)
{
	const float C1=-4.0;
	const float C2=0.0405;
	const float C3=-0.0000028;
	const float T1=0.01;
	const float T2=0.00008;

	float rh=*p_humidity;
	float t=*p_temperature;
	float rh_lin;
	float rh_true;
	float t_C;

	t_C=t*0.01-40;
	rh_lin=C3*rh*rh + C2*rh +C1;
	rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;
	if(rh_true>100) rh_true=100;
	if(rh_true<0.1) rh_true=0.1;
	
	*p_temperature=t_C;
	*p_humidity=rh_true;
}

float calc_dewpoint(float h, float t)
{
	float k,dew_point;
	k=(log10(h)-2)/0.4343+(17.62*t)/(243.12+t);
	dew_point = 243.12*k/(17.62-k);
	return dew_point;
}


int main(void)
{
	//*************************************** RS232 Initilization ******************************************************

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

	TRISE = 0x0000;
	//TRISB = 0x0000; 
	TRISBbits.TRISB5=1;   //data
	//TRISD = 0x0000; 
	TRISDbits.TRISD0=0;     //sck

	value humi_val, temp_val;
	float dew_point;
	unsigned char error, checksum;
	unsigned int i;
	
	//Init_Lcd();

	s_connectionreset();
	//Lcd_String(0,0,"temp&humid");
	
    //Delay_ms(500);
	//Delay_ms(50);
	while(1)
	{
		//Lcd_String(0,0,"temp&humid");
		error=0;
		error+=s_measure((unsigned char*) &humi_val.i,&checksum,HUMI);
		error+=s_measure((unsigned char*) &temp_val.i,&checksum,TEMP);
		if(error!=0) s_connectionreset();
		else
		{
			humi_val.f=(float)humi_val.i;
			temp_val.f=(float)temp_val.i;
			calc_sthll(&humi_val.f,&temp_val.f);
			dew_point=calc_dewpoint(humi_val.f,temp_val.f);

			printf("temp:%5.1fC humi:%5.1f%% dew point:%5.1fC \r\n",temp_val.f,humi_val.f,dew_point);
			//Lcd_String(0,0,"temp&humid");
			//Lcd_Float(0,1,10*temp_val.f); 
		}
		for(i=0;i<40000;i++);    // 과열방지!!!
		Delay_ms(100);
	}
}




