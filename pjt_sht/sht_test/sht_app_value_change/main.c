//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////온습도 센서 어플리케이션
///////////////////////////////////////////////DATA 08' 5.3
///////////////////////////////////////////////BY YANG TAE-KYUNG
//////////////////////////////////////////////////////////////////////////////////////


#include <p30f4012.h>
#include "lcd.h"
#include <stdio.h>
#include <math.h>
#include <uart.h>

_FOSC(CSW_FSCM_OFF & EC_PLL4);
_FWDT(WDT_OFF);      

#define FCY  8000000 
#define BAUDRATE 	19200        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1           

enum{TEMP, HUMI};

#define DATA PORTDbits.RD0
#define SCK PORTBbits.RB5

#define DATA_IN TRISDbits.TRISD0=1;   
#define DATA_OUT TRISDbits.TRISD0=0;   

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

/////////////////////////////////s_write_byte()//////////////////////////////////
char s_write_byte(unsigned char value)
{
	unsigned char i, error = 0;
	for(i=0x80;i>0;i/=2)
	{
		if(i & value) DATA=1;
		else DATA=0;
		SCK=1;
		Nop();Nop();Nop();
		SCK=0;
	}
	DATA=1;
	
	DATA_IN;
	SCK=1;
	error=DATA;
	SCK=0;
	DATA_OUT;

	if(error==1) printf("s_write_byte()_error\r\n");
	return error;
}
////////////////////////////////////s_read_byte()////////////////////////////////////
char s_read_byte(unsigned char ack)
{
	unsigned char i, val=0;
	DATA=1;
	
	DATA_IN;
	for(i=0x80 ; i>0 ; i/=2)
	{
		SCK=1;
		if(DATA) val=(val | i);
		SCK = 0;
	}
	DATA_OUT;

	DATA=!ack;
	SCK=1;
	Nop();Nop();Nop();
	SCK=0;
	DATA=1;

//	printf("val=%d\r\n",val);
	return val;
}

void s_transstart(void)
{
	DATA=1; SCK=0;
	Nop();
	SCK=1;
	Nop();
	DATA=0;
	Nop();
	SCK=0;
	Nop();
	SCK=1;
	Nop();
	DATA=1;
	Nop();
	SCK=0;
}

void s_connectionreset(void)
{
	unsigned char i;
	DATA=1; SCK=0;
	for(i=0; i<9;i++)
	{
		SCK=1;
		SCK=0;
	}
	s_transstart();
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

/////////////////////////////////////////s_measure()////////////////////////////////////
char s_measure(unsigned char *p_value_MSB, unsigned char *p_value_LSB, unsigned char *p_checksum, unsigned char mode)
{
	unsigned error=0;
	unsigned int i=0;
	unsigned long i_l=0;

	s_transstart();

	switch(mode)
	{
		case TEMP:error+=s_write_byte(MEASURE_TEMP); break;
		case HUMI:error+=s_write_byte(MEASURE_HUMI); break;
		default: break;
	}

	DATA_IN;
	while(1)
	{
		i_l++;	
		if(DATA==0) 
		{
			// i_l=160000 정도 counting 후 break함 
//			printf("okay! measure cnt: %ld\r\n",i_l); 
			break;
		}
		
		// 190216 line을 길게하면 while탈출불가, reset으로 방어로직 
		if(i_l >= 500000)
		{
			s_softreset();
			break;
		}
	}
	if(DATA) 
	{
		printf("s_measure()_error\r\n"); 
		error+=1;
	}
	DATA_OUT;

	*(p_value_MSB) = s_read_byte(ACK);
	*(p_value_LSB)=s_read_byte(ACK);
	*p_checksum=s_read_byte(noACK);

//	printf("*p_value_MSB=%d\r\n",*p_value_MSB);
//	printf("*p_value_LSB=%d\r\n",*p_value_LSB);
	
	return error;
}

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

	TRISBbits.TRISB5=0;		//SCK
	TRISDbits.TRISD0=0;     //DATA

	unsigned char temp_val_MSB, temp_val_LSB, humi_val_MSB, humi_val_LSB;
	float dew_point, buffer_temp_f, buffer_humi_f;
	unsigned char error, checksum, test_char;
	unsigned int i, test_int, cnt=0, buffer_temp, buffer_humi;
	
	for(i=0;i<10000;i++);
	
/*
	while(1){
		printf("UART TEST Okay!\r\n");
	}
*/
	s_connectionreset();

	while(1)
	{
		error=0;
		error+=s_measure(&humi_val_MSB,&humi_val_LSB,&checksum,HUMI);
		error+=s_measure(&temp_val_MSB,&temp_val_LSB,&checksum,TEMP);

		if(error!=0) {printf("main()_error\r\n"); s_connectionreset();}
		else
		{
//			printf("temp_val_decimal_MSB:%d\r\n",temp_val_MSB);
//			printf("temp_val_decimal_LSB:%d\r\n",temp_val_LSB);

			buffer_temp=temp_val_MSB;
			buffer_temp=(buffer_temp<<8);
			buffer_temp=buffer_temp|temp_val_LSB;
//			printf("buffer_temp:%d\r\n",buffer_temp);

			buffer_humi=humi_val_MSB;
			buffer_humi=(buffer_humi<<8);
			buffer_humi=buffer_humi|humi_val_LSB;
//			printf("buffer_humi:%d\r\n",buffer_humi);

			buffer_humi_f=(float)buffer_humi;
			buffer_temp_f=(float)buffer_temp;
			calc_sthll(&buffer_humi_f,&buffer_temp_f);
			dew_point=calc_dewpoint(buffer_humi_f,buffer_temp_f);

			printf("temp:%5.1fC humi:%5.1f%% dew point:%5.1fC \r\n",buffer_temp_f,buffer_humi_f,dew_point);
		}
		for(i=0;i<60000;i++); 
		for(i=0;i<60000;i++);
		for(i=0;i<60000;i++);
	}
}




