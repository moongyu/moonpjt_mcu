//////////////////////////////////////////////////////////////////////////////////////
// 2019.02.19
// 달걀 부화기 pjt

// 동작 
// 1. 온도에 의해 램프 제어 
// 2. 습도에 의해 가습기 제어 (sw방식)
// 3. 시간에 의해 DC-FAN 제어
// 4. 온도 고장, 오측정시 Alram LED ON
// 5. 습도 고장, 오측정시, 자연습도 기준 이상시 Alram LED Toggle (온도 고장 우선순위가 더 높음)

// 주의 
// 1. EC_PLL4 변경시 sht sensor 통신 불량
// 2. RE1 ON/OFF 제어안됨 
//////////////////////////////////////////////////////////////////////////////////////


#include <p30f4012.h>
#include "lcd.h"
#include <stdio.h>
#include <math.h>
#include <uart.h>
#include <timer.h>


_FOSC(CSW_FSCM_OFF & EC_PLL4);
//_FOSC(CSW_FSCM_OFF & EC_PLL8);


_FWDT(WDT_OFF);      

#define FCY  8000000 
#define BAUDRATE 	19200        
#define BRGVAL   	((FCY/BAUDRATE)/16)-1           

enum{TEMP, HUMI};

#define DATA PORTDbits.RD0
#define SCK PORTBbits.RB5

#define DATA_IN TRISDbits.TRISD0=1;   
#define DATA_OUT TRISDbits.TRISD0=0;   

#define OUT_LED			_RE0
#define OUT_ULTRA		_RE2
#define OUT_FAN			_RE3
#define OUT_ALRAM		_RE4

#define noACK 0
#define ACK 1

#define STATUS_REG_W 0x06
#define STATUS_REG_R 0x07
#define MEASURE_TEMP 0x03
#define MEASURE_HUMI 0x05
#define RESET 0x1e

#define LED_ON_TEMP		(37-2)	// 37도 기준 
#define LED_OFF_TEMP	(37+2)	
#define ULTRA_ON_HUMI	(50-10)	// 50% 기준 
#define ULTRA_OFF_HUMI	(50+10)

#define ULTRA_FORCE_OFF	(70)
#define ULTRA_FORCE_ON	(20)

#define FAN_ON_TIMER	(1800)//(3600)	// 1 hour 주기 
#define FAN_ON_TIME		(150)//(300)	// on time 5분

#define ULTRA_CNT		(200)	// [ms]	// 너무 길게 눌러도 동작 안됨, 사람의 손으로 누르는 속도 반영 
#define ULTRA_CNT_REST	(1000)	// [ms]

#define INIT		(0)
#define START		(1)
#define ON			(2)
#define TOGGLE_ON	(3)
#define TOGGLE_OFF	(4)
#define OFF			(5)






//////////////////////////////////////////////////////////////////
void io_init(void);
void timer_init(void);

void control_led(void);
void control_ultra(void);
void control_fan(void);

void check_abnormal(void);

//////////////////////////////////////////////////////////////////

float f_temp_result, f_humi_result = 0.0;
unsigned char control_led_flag, control_ultra_flag = 0;
unsigned int timer_fan_on = 0;

unsigned int ultra_force_control_active_cnt = 0;

unsigned char ultra_cnt = 0;

void __attribute__ ((__interrupt__)) _U1TXInterrupt(void)  //uart
{
	IFS0bits.U1TXIF=0;
}

// 1ms Timer
void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	static unsigned int unBaseTime0 = 0;
	static unsigned int timer_1sec = 0;
	static unsigned int timer_fan_cnt = 0;

	WriteTimer3(0);

	IFS0bits.T3IF=0;

	//_RD1 = !_RD1;

	unBaseTime0++;
	
	if(unBaseTime0 >= 1000)	// base time 1sec -> 2sec 로 변경 
	{
		unBaseTime0 = 0;
		timer_1sec++;
	}

	if(ultra_cnt != 0)
	{
		ultra_cnt--;
	}
	else
	{
		ultra_cnt = 0;
	}
	

	if(timer_1sec >= 2)
	{
		timer_1sec = 0;

		check_temp();
		check_humi();

		if(timer_fan_on == 0)
		{
			timer_fan_cnt++;
			if(timer_fan_cnt >= FAN_ON_TIMER)		// base time 1sec -> 2sec 로 변경
			{
				timer_fan_cnt = 0;
				timer_fan_on = FAN_ON_TIME;		// base time 1sec -> 2sec 로 변경 
			}
		}
		else
		{
			timer_fan_on--;	// underflow 불가능 
		}

		if(ultra_force_control_active_cnt <= 10)	// 20 sec
		{
			if(ultra_force_control_active_cnt == 0)
			{
				ultra_force_control_active_cnt = 0;
			}
			else
			{
				ultra_force_control_active_cnt--;
			}
		}
	}


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

	io_init();

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

	timer_init();

	TRISBbits.TRISB5=0;		//SCK
	TRISDbits.TRISD0=0;     //DATA

	unsigned char temp_val_MSB, temp_val_LSB, humi_val_MSB, humi_val_LSB;
	float dew_point, buffer_temp_f, buffer_humi_f;
	unsigned char error, checksum, test_char;
	unsigned int i, test_int, cnt=0, buffer_temp, buffer_humi;

	float total_sum_tmp[2] = {0.0};
	unsigned char sum_cnt = 0;
	
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
		for(i=0;i<60000;i++); 	// UART Display 속도 지연 
		//for(i=0;i<60000;i++);
		//for(i=0;i<60000;i++);

		//f_temp_result = buffer_temp_f;
		//f_humi_result = buffer_humi_f;

		total_sum_tmp[0] += buffer_temp_f;
		total_sum_tmp[1] += buffer_humi_f;
		sum_cnt++;
		
		if(sum_cnt >= 5)
		{		
			f_temp_result = total_sum_tmp[0]/5;	// 5회 평균 처리 
			f_humi_result = total_sum_tmp[1]/5;
			
			sum_cnt = 0;
			total_sum_tmp[0] = 0;
			total_sum_tmp[1] = 0;
		}
		
		control_led();
		control_ultra();
		control_fan();

		check_abnormal();
			
	}
}

//////////////////////////////////////////////////////////////////
void io_init(void)
{
	//3 방향설정  
	// OUT LED, ULTRA, FAN
	_TRISE0 = 0;
	_TRISE1 = 0;
	_TRISE2 = 0;
	_TRISE3 = 0;
	_TRISE4 = 0;
	
	//3 초기값 
	_RE0 = 0;
	_RE1 = 0;
	_RE2 = 0;
	_RE3 = 0;
	_RE4 = 0;
}

void timer_init()
{
	/*
	// not use
	ConfigIntTimer1(T1_INT_PRIOR_3 &			// 0~7 => 최소 1 이상 설정할 것
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
	
	// Timer3 ; 주기 검증 안함, 인터럽트 발생만 확인  
	ConfigIntTimer3(T3_INT_PRIOR_3 &			// 0~7 => 최소 1 이상 설정할 것
				T3_INT_ON);					// ON, OFF

	WriteTimer3(0);

	OpenTimer3(	T3_ON &						// ON, OFF
				T3_IDLE_STOP &					// CON, STOP
				T3_GATE_OFF &					// ON, OFF
				T3_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128

				T3_SOURCE_INT,	         	// EXT, INT
				8000);						// Match_value		// 1.0ms 주기				
}
//////////////////////////////////////////////////////////////////


void check_temp(void)
{
	if(control_led_flag == 0)
	{
		if(f_temp_result <= LED_ON_TEMP)
		{
			control_led_flag = 1;
		}
	}
	else
	{
		if(f_temp_result >= LED_OFF_TEMP)
		{
			control_led_flag = 0;
		}
	}
}

void control_led(void)
{
	if(control_led_flag == 1)
	{
		OUT_LED = 1;
	}
	else
	{
		OUT_LED = 0;
	}
}

void check_humi(void)
{
	if(control_ultra_flag == 0)
	{
		if(f_humi_result <= ULTRA_ON_HUMI)
		{
			control_ultra_flag = 1;
		}
	}
	else
	{
		if(f_humi_result >= ULTRA_OFF_HUMI)
		{
			control_ultra_flag = 0;
		}
	}
}
/*
void control_ultra(void)
{
	static unsigned char ultra_state = INIT;
	unsigned int i;
	
	if((control_ultra_flag == 1) && (ultra_state == INIT))
	{
		OUT_ULTRA = 1;
		for(i=0; i<60000; i++);
		for(i=0; i<60000; i++);
		for(i=0; i<10000; i++);
		OUT_ULTRA = 0;

		ultra_state = START;
	}
	else if((control_ultra_flag == 0) && (ultra_state == START))
	{
		//if(ultra_state == START)
		//{
			OUT_ULTRA = 1;
			for(i=0; i<60000; i++);
			for(i=0; i<60000; i++);
			for(i=0; i<10000; i++);
			OUT_ULTRA = 0;

			for(i=0; i<60000; i++);
			for(i=0; i<60000; i++);
			for(i=0; i<10000; i++);

			OUT_ULTRA = 1;
			for(i=0; i<60000; i++);
			for(i=0; i<60000; i++);
			for(i=0; i<10000; i++);
			OUT_ULTRA = 0;

			ultra_state = INIT;
		//}
	}
	else if((control_ultra_flag == 1) && (ultra_state == START) && (f_humi_result <= ULTRA_FORCE_ON))
	{
		OUT_ULTRA = 1;
		for(i=0; i<60000; i++);
		for(i=0; i<60000; i++);
		for(i=0; i<10000; i++);
		OUT_ULTRA = 0;
	}
	else if((f_humi_result >= ULTRA_FORCE_OFF) && (ultra_force_control_active_cnt == 0))
	{
		OUT_ULTRA = 1;
		for(i=0; i<60000; i++);
		for(i=0; i<60000; i++);
		for(i=0; i<10000; i++);
		OUT_ULTRA = 0;

		ultra_state = INIT;

		ultra_force_control_active_cnt = 10;	// 20 sec reload
	}
	else
	{
		OUT_ULTRA = 0;
	}
}*/

void control_ultra(void)
{
	static unsigned char ultra_state = INIT;
	unsigned int i;
	
	if((control_ultra_flag == 1) && (ultra_state == INIT))
	{
		ultra_state = START;
		OUT_ULTRA = 1;
		ultra_cnt = ULTRA_CNT;
	}
	else if((control_ultra_flag == 1) && (ultra_state == START) && (ultra_cnt == 0) )
	{
		OUT_ULTRA = 0;
		ultra_state = ON;
	}
	else if((control_ultra_flag == 0) && (ultra_state == ON))
	{
		OUT_ULTRA = 1;
		ultra_cnt = ULTRA_CNT;
		ultra_state = TOGGLE_ON;
	}
	else if((control_ultra_flag == 0) && (ultra_state == TOGGLE_ON) && (ultra_cnt == 0) )
	{
		OUT_ULTRA = 0;
		ultra_state = TOGGLE_OFF;
		ultra_cnt = ULTRA_CNT_REST;
	}
	else if((control_ultra_flag == 0) && (ultra_state == TOGGLE_OFF) && (ultra_cnt == 0) )
	{
		OUT_ULTRA = 1;
		ultra_cnt = ULTRA_CNT;
		ultra_state = OFF;
	}
	else if((control_ultra_flag == 0) && (ultra_state == OFF) && (ultra_cnt == 0) )
	{
		OUT_ULTRA = 0;
		ultra_state = INIT;
	}
	else
	{
		OUT_ULTRA = 0;
	}
}


void control_fan(void)
{
	if(timer_fan_on != 0)
	{
		OUT_FAN = 1;
	}
	else
	{
		OUT_FAN = 0;
	}
}

void check_abnormal(void)
{
	unsigned int i;

	if((f_temp_result >= 60)
		|| (f_temp_result <= 20))
	{
		OUT_ALRAM = 1; 
	}
	else if((f_humi_result >= 80)
			|| (f_humi_result <= 30))
	{
		OUT_ALRAM = 1; 
		for(i=0; i<60000; i++);
		for(i=0; i<60000; i++);
		for(i=0; i<60000; i++);
		OUT_ALRAM = 0;
		for(i=0; i<60000; i++);
		for(i=0; i<60000; i++);
		for(i=0; i<60000; i++);

	}
	else
	{
		OUT_ALRAM = 0;
	}
}

