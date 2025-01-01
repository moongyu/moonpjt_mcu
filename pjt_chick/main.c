//////////////////////////////////////////////////////////////////////////////////////
// 2019.02.19
// 달걀 부화기 pjt

// 동작 
// 1. 온도에 의해 램프 제어 
// 2. 습도에 의해 가습기 제어 (sw방식)
// 3. 시간에 의해 DC-FAN 제어
// 4. 온도 고장, 오측정시 Alram LED Toggle
// 5. 습도 고장, 오측정시, 자연습도 기준 이상시 Alram LED Toggle (온도 고장 우선순위가 더 높음)

// 190221 수위센서 이용한 자동급수기능 추가 
// 6. 수위 낮음 10초 지속시 급수밸드 on
// 7. 수위 차면 자동 급수밸브 off, 밸브열고 60초 내 수위 안차면 강제 차단, 알람 점등 (해제는 mcu reset 이상) 

// 190400 초음파가습기 강제 off 기능 추가 
// 가습기 제어 동작실패시 limit이상의 습도 도달하면 강제로 가습기 전원 off

// 190601 220V AC Duct 추가 
// 주기적으로 duct를 동작시켜 환기

// 주의 
// 1. EC_PLL4 변경시 sht sensor 통신 불량
// 2. RE1 ON/OFF 제어안됨 
//3 3. [매우 주의] Uart 연결없이 init 하면 디버거 go 해도 바로 stop됨, Release 동일

//3 190304 ultra 제어 디버깅 후 uart 동작은 확인 안한 상태 
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

//#define UART_ENABLE
#define CHICK_CARE

#define DATA PORTDbits.RD0
#define SCK PORTBbits.RB5

#define DATA_IN TRISDbits.TRISD0=1;   
#define DATA_OUT TRISDbits.TRISD0=0;   

#define OUT_LED			_RE0
#define OUT_ULTRA		_RE2
#define OUT_FAN			_RE3
#define OUT_ALRAM		_RE4
#define OUT_WATER		_RE5
#define OUT_ROTATE_MOTOR	_RE8

#define OUT_AC_DUCT		_RE1

#define IN_WATERLEVEL	_RB0
#define IN_SW_HUMI		_RB1

#define OUT_HUMI_POWER		_RD1

#define noACK 0
#define ACK 1

#define STATUS_REG_W 0x06
#define STATUS_REG_R 0x07
#define MEASURE_TEMP 0x03
#define MEASURE_HUMI 0x05
#define RESET 0x1e

// 사용시 float 처리 필수
// 실제 -> 측정 ; 오차 +1~+2도 존재함 
// 37.2도 -> 39도; 2도 더 높음 
// 36.2도 -> 37.2도; 1도 더 높음 
#define TEMP_OFFSET	(2.0)
#ifdef CHICK_CARE
	// step 1
	//#define LED_ON_TEMP		(36.0)
	//#define LED_OFF_TEMP		(38.0)
	
	// step 2
	//#define LED_ON_TEMP		(36.0-(TEMP_OFFSET*1))
	//#define LED_OFF_TEMP		(38.0-(TEMP_OFFSET*1))

	// step 3
	//#define LED_ON_TEMP		(36.0-(TEMP_OFFSET*2))
	//#define LED_OFF_TEMP		(38.0-(TEMP_OFFSET*2))

	// step 4
	#define LED_ON_TEMP		(36.0-(TEMP_OFFSET*3))
	#define LED_OFF_TEMP		(38.0-(TEMP_OFFSET*3))
	
#else
	#define LED_ON_TEMP		(37.0+2.5)//(37.0+2.5)	//(37-1)	// 37도 기준 
	#define LED_OFF_TEMP		(38.5+2.0)	//(37+2)
#endif

#define ULTRA_ON_HUMI_HIGH	(70-5)	// 70% 설정시 ultra 제어 안됨 현상 발생  	
#define ULTRA_OFF_HUMI_HIGH	(70+0)	
#define ULTRA_ON_HUMI_MID	(55-5)	// 55% 기준 
#define ULTRA_OFF_HUMI_MID	(55+5)

#define ALRAM_TEMP_HIGH	(LED_OFF_TEMP+1.5)	//(41.0)	// LED_OFF_TEMP 의 앞 온도 기준에서 더하기 
#define ALRAM_TEMP_LOW	(34.0)
//#define ALRAM_HUMI_HIGH	(ULTRA_OFF_HUMI_HIGH+10)//(75.0)	// 190406 humi select 기능으로 인해 변수로 대체 
#define ALRAM_HUMI_LOW	(35.0)

// 반주기설정시 
// 1시간 주기(1800)에 10분(300) on설정
// 10분 * 60초 / 2 = 300 ; 2는 timer 반주기 설정 의함 
#define FAN_ON_TIMER_DCFAN	(1800)	// 30분 주기 
#define ROTATE_ON_TIMER		(3600)	// 1 hour 주기 
#define DUCT_ON_TIMER		(60*10)	// 10분 주기 

#ifdef CHICK_CARE
	#define FAN_ON_TIME		(120)
#else
	#define FAN_ON_TIME		(60)	//1분 	//(600)	// on time 10분 // 저온 방지 시간설정
#endif

#define DUCT_ON_TIME			(60)	// 1분 동작 

#define FAN_ON_TIMER_RMOTOR	(ROTATE_ON_TIMER*6)// 6 hour 주기 
#define FAN_ON_TIME_RMOTOR	(8)	// on time 8초

#define ULTRA_CNT		(200)	// [ms]	// 너무 길게 눌러도 동작 안됨, 사람의 손으로 누르는 속도 반영, 100 ~ 300ms 정상동작 확인,  (10ms, 300ms 실패) 
#define ULTRA_CNT_REST	(1000)	// [ms]

#define WATER_FORCE_OFF	(20)	// [sec]
#define WATER_OFF_TIME	(2)	// [sec]	// 시간 동안 value on하여 충분히 급수함, 결국 이 시간동안의 물만큼 사용하게 됨, WATER_FORCE_OFF 보다 반드시 작아야 함  

#define INIT		(0)
#define START		(1)
#define ON			(2)
#define TOGGLE_ON	(3)
#define TOGGLE_OFF	(4)
#define OFF			(5)

#define BOOT_DELAY_DONE_CNT		(5)





//////////////////////////////////////////////////////////////////
void io_init(void);
void uart_init(void);
void timer_init(void);

void control_led(void);
void control_ultra(void);
void control_fan(void);
void control_rotatemotor(void);

void check_abnormal(void);

// 190221 수위센서 이용한 자동급수기능 추가 
void check_water_level(void);
void control_waterlevel(void);

void debug_ultra_con(void);

void check_temp(void);
void check_humi(void);

void check_sw_humi(void);

void control_ac_duct(void);

//////////////////////////////////////////////////////////////////

float f_temp_result, f_humi_result = 0.0;
unsigned char control_led_flag, control_ultra_flag = 0;
unsigned int timer_fan_on, timer_rotate_motor_on, timer_duct_on = 0;

unsigned long ultra_cnt = 0;

unsigned int waterlevel_on_cnt, waterlevel_off_cnt, waterlevel_ontime = 0;
unsigned char control_waterlevel_flag, waterlevel_forced_off = 0;

unsigned int booting_delay_time = 0;

unsigned char ultra_state = INIT;
unsigned char sensor_err = 0;

unsigned long test01 = 0;

float f_humi_level_on, f_humi_level_off, f_alram_humi_high = 0.0;

unsigned char debug_humi_fail = 0;	// 190400 초음파가습기 강제 off 기능 추가 

unsigned char guc_ac_duct_f = 0;

void __attribute__ ((__interrupt__)) _U1TXInterrupt(void)  //uart
{
	IFS0bits.U1TXIF=0;
}

//unsigned int timer_rotate_motor_cnt = 0;	// _debug
// 1ms Timer
void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	static unsigned int unBaseTime0 = 0;
	static unsigned int timer_1sec = 0;
	static unsigned int timer_fan_cnt = 0;
	static unsigned int timer_rotate_motor_cnt = 0;
	static unsigned int sun_timer_duct_cnt = 0;

	WriteTimer3(0);

	IFS0bits.T3IF=0;

	//_RD1 = !_RD1;

	// _debug
	if(ultra_state == START)
	{
		test01++;
	}

/////////////////////////////////////////////////////////////////////////
	unBaseTime0++;
	
	if(unBaseTime0 >= 1000)
	{
		unBaseTime0 = 0;
		timer_1sec++;

		if(booting_delay_time >= BOOT_DELAY_DONE_CNT)
		{
			check_water_level();
		}
		else
		{
			booting_delay_time++;
		}
	}

	if(ultra_cnt != 0)
	{
		ultra_cnt--;
	}
	else
	{
		ultra_cnt = 0;
	}
	
	if(timer_1sec >= 1)
	{
		timer_1sec = 0;

		//3 DC Fan
		if(timer_fan_on == 0)
		{
			timer_fan_cnt++;
			if(timer_fan_cnt >= FAN_ON_TIMER_DCFAN)
			{
				timer_fan_cnt = 0;
				timer_fan_on = FAN_ON_TIME;
			}
		}
		else
		{
			timer_fan_on--;	// underflow 불가능 
		}

		//3 Ac rotate motor
		if(timer_rotate_motor_on == 0)
		{
			timer_rotate_motor_cnt++;	// 단위 주위
			if(timer_rotate_motor_cnt >= FAN_ON_TIMER_RMOTOR)
			{
				timer_rotate_motor_cnt = 0;
				timer_rotate_motor_on = FAN_ON_TIME_RMOTOR;
			}
		}
		else
		{
			timer_rotate_motor_on--;	// underflow 불가능 
		}

		//3 AC duct 
		if(timer_duct_on == 0)
		{
			sun_timer_duct_cnt++;
			if(sun_timer_duct_cnt >= DUCT_ON_TIMER)	// 65535 넘으면 안됨  
			{
				sun_timer_duct_cnt = 0;
				timer_duct_on = DUCT_ON_TIME;
			}
		}
		else
		{
			timer_duct_on--;	// underflow 불가능 
		}

	}

	control_ultra();	// control flag 가 booting_delay_time 이후 set 가능함 의해 boot delay 불필요 


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

#ifdef UART_ENABLE
	if(error==1) printf("s_write_byte()_error\r\n");
#endif

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

#ifdef UART_ENABLE
		printf("s_measure()_error\r\n");
#endif

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

#ifdef UART_ENABLE
	uart_init();
#endif

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
#ifdef UART_ENABLE
	while(1){
		printf("UART TEST Okay!\r\n");
	}
#endif
*/
	// _debug
	//OUT_ROTATE_MOTOR = 1;	

	s_connectionreset();

	check_sw_humi();

	while(1)
	{
		error=0;
		error+=s_measure(&humi_val_MSB,&humi_val_LSB,&checksum,HUMI);
		error+=s_measure(&temp_val_MSB,&temp_val_LSB,&checksum,TEMP);

		if(error!=0) 
		{
#ifdef UART_ENABLE
			printf("main()_error\r\n"); 
#endif
			s_connectionreset();
		}
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
			calc_sthll(&buffer_humi_f,&buffer_temp_f);	// /w dew_p(), 8ms 소요 al.
			dew_point=calc_dewpoint(buffer_humi_f,buffer_temp_f);

#ifdef UART_ENABLE
			//printf("temp:%5.1fC humi:%5.1f%% dew point:%5.1fC \r\n",buffer_temp_f,buffer_humi_f,dew_point);	// % 가 한칸 공백 기능 
			printf("temp:%5.1f humi:%5.1f%% %d %d %d %d\r\n",
				buffer_temp_f,buffer_humi_f, control_led_flag, control_ultra_flag, timer_fan_on, control_waterlevel_flag);
#endif

		}
		//for(i=0;i<60000;i++); 	// UART Display 속도 지연 
		//for(i=0;i<60000;i++);
		//for(i=0;i<60000;i++);

		//f_temp_result = buffer_temp_f;
		//f_humi_result = buffer_humi_f;

		total_sum_tmp[0] += buffer_temp_f;
		total_sum_tmp[1] += buffer_humi_f;
		sum_cnt++;
		
		if(sum_cnt >= 5)
		{		
			// _debug
			f_temp_result = total_sum_tmp[0]/5;	// 5회 평균 처리 
			f_humi_result = total_sum_tmp[1]/5;

			sum_cnt = 0;
			total_sum_tmp[0] = 0;
			total_sum_tmp[1] = 0;
		}

		// 센서 오류시 fan 제외한 모든 부하 강제 off
		// _debug 0.0 으로 처리되는 debuging 시 주석 여부 판단  
		if((f_temp_result == 0.0) || ( f_humi_result == 0.0))
		{
			sensor_err = 1;
			control_led_flag = 0;
			control_ultra_flag = 0;
			control_waterlevel_flag = 0;
		}
		else
		{
			sensor_err = 0;
		}

		if(booting_delay_time >= BOOT_DELAY_DONE_CNT)
		{
			check_temp();	// _work main loop 로 이동 
			check_humi();

			control_led();
			//control_ultra(); // 190304 main loop 지연으로 제어시점 틀어짐, ISR안으로 이동 
			control_fan();
			control_waterlevel();

			control_rotatemotor();	// 190317 220v 전란용 모터제어 추가 

			control_ac_duct();		// 190601 220V AC Duct 추가 
	
			check_abnormal();
		}
			
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
	_TRISE5 = 0;
	_TRISE8 = 0;
	
	_TRISD1 = 0;	// 190400 초음파가습기 강제 off 기능 추가 

	//3 초기값 
	_RE0 = 0;
	_RE1 = 0;
	_RE2 = 0;
	_RE3 = 0;
	_RE4 = 0;
	_RE5 = 0;
	_RE8 = 0;

	_RD1 = 1;	// 190400 초음파가습기 강제 off 기능 추가 

	//3 PORTB 는 초기화시 analog input으로 설정됨, digital input으로 설정 변경 (Programming mode시)
	_PCFG0 = 1;
	_TRISB0 = 1;

	_PCFG1 = 1;
	_TRISB1 = 1;
		
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


void check_temp(void)
{
	if(control_led_flag == 0)
	{
		if(f_temp_result <= (float)LED_ON_TEMP)
		{
			control_led_flag = 1;
		}
	}
	else
	{
		if(f_temp_result >= (float)LED_OFF_TEMP)
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
		if(f_humi_result <= f_humi_level_on)
		{
			control_ultra_flag = 1;
		}
	}
	else
	{
		if(f_humi_result >= f_humi_level_off)
		{
			control_ultra_flag = 0;
		}
	}
}

void control_ultra(void)
{
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
		// 조건없으면 OUT_ULTRA 유지
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
	
	if((waterlevel_forced_off == 1) || (sensor_err == 1))
	{
		OUT_ALRAM = 1; 
	}
	else if((f_temp_result >= (float)ALRAM_TEMP_HIGH) || (f_temp_result <= (float)ALRAM_TEMP_LOW))
	{
		OUT_ALRAM = 1; 
		for(i=0; i<20000; i++);
		OUT_ALRAM = 0;
		for(i=0; i<20000; i++);
	}
	else if((f_humi_result >= f_alram_humi_high)
			|| (f_humi_result <= (float)ALRAM_HUMI_LOW))
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

	if(f_humi_result >= f_alram_humi_high)
	{
		debug_humi_fail = 1;
		OUT_HUMI_POWER = 0;	// 190400 초음파가습기 강제 off 기능 추가 

		ultra_state = INIT;
		ultra_cnt = 0;
	}
	else
	{
		debug_humi_fail = 0;
		OUT_HUMI_POWER = 1;
	}
}

// main loop 지연이 수초 대이면 ISR 로 이동 필요, ms 로 관리 필요 
void check_water_level(void)
{
	// 수위 높음 : 저항 무한대 => 0v
	// 수위 낮음 : 저항 수ohm => 5v

	if((IN_WATERLEVEL == 1) && (control_waterlevel_flag == 0))
	{
		waterlevel_on_cnt++;
		if(waterlevel_on_cnt >= 10)
		{
			control_waterlevel_flag = 1;
			waterlevel_on_cnt = 0;
		}
	}
	else
	{
		waterlevel_on_cnt = 0;
	}

	if(control_waterlevel_flag == 1)
	{
		waterlevel_ontime++;
		
		if(IN_WATERLEVEL == 0)
		{
			waterlevel_off_cnt++;

			if(waterlevel_off_cnt >= WATER_OFF_TIME)	// 시간 동안 value on하여 충분히 급수함, 결국 이 시간동안의 물만큼 사용하게 됨, WATER_FORCE_OFF 보다 반드시 작아야 함  
			{
				OUT_WATER = 0;

				control_waterlevel_flag = 0;
				waterlevel_ontime = 0;
				waterlevel_off_cnt = 0;
			}
		}
		else if(waterlevel_ontime >= WATER_FORCE_OFF)
		{
			waterlevel_ontime = WATER_FORCE_OFF;
			control_waterlevel_flag = 0;
			waterlevel_forced_off = 1;
			waterlevel_off_cnt = 0;
		}
		else
		{
			waterlevel_off_cnt = 0;
		}
	}
	else
	{
		waterlevel_ontime = 0;
	}
}

void control_waterlevel(void)
{
	if((waterlevel_forced_off == 1) || (control_waterlevel_flag == 0))
	{
		OUT_WATER = 0;
	}
	else if(control_waterlevel_flag == 1)
	{
		OUT_WATER = 1;
	}
	else
	{
		OUT_WATER = 0;
	}
	
}

void control_rotatemotor(void)
{
	if(timer_rotate_motor_on != 0)
	{
		OUT_ROTATE_MOTOR = 1;
	}
	else
	{
		OUT_ROTATE_MOTOR = 0;
	}
}

void check_sw_humi(void)
{
	if(IN_SW_HUMI == 0)
	{
		f_humi_level_on = (float)ULTRA_ON_HUMI_MID;
		f_humi_level_off = (float)ULTRA_OFF_HUMI_MID;

		f_alram_humi_high = ULTRA_OFF_HUMI_MID+10.0;
	}
	else
	{
		f_humi_level_on = (float)ULTRA_ON_HUMI_HIGH;
		f_humi_level_off = (float)ULTRA_OFF_HUMI_HIGH;

		f_alram_humi_high = ULTRA_OFF_HUMI_HIGH+10.0;
	}
}

// 190601 220V AC Duct 추가
void control_ac_duct(void)
{
	if(timer_duct_on != 0)
	{
		OUT_AC_DUCT = 1;
	}
	else
	{
		OUT_AC_DUCT = 0;
	}
}