//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//170812 PWM ON -> DELAY -> OFF -> DELAY �� �ݺ��ϴ� sample code�� ���� 
// forward, backword, right, left ����ġ�� ���� PWM CH1, CH2�� �߻��� 
// HW�߰��� AM-DC2-4C(L298*2EA)�� ����� 
// PORTB�� INPUT������ ��� ������ ������ Ȯ���ʿ� (�������� �����������)
// PORTB�� OUTPUT���� ��� �ȵ� �� ������ ������
// IO������ PORT C, D, E�� ����� �ڵ� 

//180218 HW�߰��� RF Module �߰� (TXM 8D423C) 
// Keyin ��ȣ ���� (Forward + Backword -> Forward + Right ) : RF Module���� �����ȵ�

// 180324 
// mz-10, GL-12 �ۼ��ű��� CH1, CH4���� ������ pluse�� Inputcapture �Ͽ� ������

// ����
// PORTB �� �ʱ�ȭ�� analog input���� ���������� digital input���� ���� ���� (Programming mode��)
// ADPCF ��������� �� 


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



#include <p30f4012.h>
#include <stdio.h>
#include <pwm.h>
#include <incap.h>


_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);	// FRC�� PWM���� �� ������ ����ǳ� Ȯ�� �ʿ�, �����ϰ� EC_PLL8�� ����� 
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

//Fcy=Fosc/4 (dsPIC30F)
// Fosc = 8M * 8 = 64Mhz (POST 1:1)
// Fcy = 64Mhz/4 = 16Mhz
#define XOSC				8000000
#define PLL					8
//#define PERSCALE			1	// POST	: no alter
#define TICK				((float)1/((XOSC * PLL)/4))


#define ACTIVE_LOW_ENALBE_1	LATEbits.LATE4	// PWM CH3�� Disable�� ������ �� ��� 
#define DIR_1				LATEbits.LATE5

#define ACTIVE_LOW_ENALBE_2	LATDbits.LATD0
#define DIR_2				LATDbits.LATD1

//3 Control type ����
#define REMOTE_CONTROL		(1)

#ifdef MANUAL_CONTROLL	// Switch type ���� 
#define FORWARD				PORTCbits.RC15	// PORTB�� �����ÿ��� INPUT��� ���� �̷�, PORTC�� ������ 
#define BACK				PORTCbits.RC13
#define RIGHT				PORTCbits.RC14
#define LEFT				PORTEbits.RE8
#elif REMOTE_CONTROL
#define NO_OPERATION		(0)
#define FORWARD				(1)
#define BACK				(2)
#define RIGHT				(3)
#define LEFT				(4)
#else
// "error"
#endif

#define STOP				(0x0000)
#define SPEED				(0x5fff)
#define SPEED_UP			(SPEED + 0x2000)
#define SPEED_MAX			(0x7fff)
#define SPEED_MIN			(0x0000)	// remote controll ���� �νĽ� �߻��ϴ� pwm min��

// remote controll ���� �νĽ� �߻��ϴ� channel voltage
#define CH_VOLT_MAX			(2000)	// [0.001] mili sec; 2.0ms 
#define CH_VOLT_REF			(1500)	// [0.001] mili sec; 1.5ms 
#define CH_VOLT_MIN			(1000)	// [0.001] mili sec; 1.0ms 
#define CH_VOLT_HYSERESIS	(70)	// [0.001] mili sec; 0.05ms 


void motor_init(void);  
void io_init(void);

int i,j;
unsigned char system_on = 0;
unsigned int booting_delay_cnt = 0;

unsigned char dir_ch4, dir_ch1=0;

float Coeff = 0;
unsigned int ch_pwm[2] = {0};

int Interrupt_Count = 0 , Int_flag;
unsigned int timer_first_edge0, timer_first_edge1;
unsigned int timer_second_edge0, timer_second_edge1;
unsigned int timer_pluse_width_raw0, timer_pluse_width_raw1;

float timer_pluse_width_sec0, timer_pluse_width_sec1;
unsigned int timer_pluse_width_ms0, timer_pluse_width_ms1;

void motor_init();
void io_init(void);
void inputcapture_init(void); 


void RunDetectDir(void);
void RunMotor(void);

///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
// input capture�Ͽ� ����� ms������ pluse width�� ���� 
// input : pluse
// output : ����, ms������ pluse width�� ���� 
void __attribute__((__interrupt__)) _IC7Interrupt(void)
{
	// controller-ch1 (forward, backward meaurement)
	
	//_RE0 = ~_RE0;

	if(_RB4 == 1)
	{
		ReadCapture7(&timer_first_edge0);
	}
	else if(_RB4 == 0)
	{
		ReadCapture7(&timer_second_edge0);
		//Int_flag = 1;
		
		timer_pluse_width_raw0 = timer_second_edge0 - timer_first_edge0;
		timer_pluse_width_sec0 = (float)timer_pluse_width_raw0 * TICK * 8;	// Timer Prescale = 8
		timer_pluse_width_ms0 = (unsigned int)(timer_pluse_width_sec0 * 1000000);
	}
	else
	{
	}

	IFS1bits.IC7IF = 0;

	// ���� ���� 
	if(timer_pluse_width_ms0 >= (CH_VOLT_REF + CH_VOLT_HYSERESIS))
	{
		dir_ch1 = FORWARD;
	}
	else if(timer_pluse_width_ms0 < (CH_VOLT_REF - CH_VOLT_HYSERESIS))
	{
		dir_ch1 = BACK;
	}
	else
	{
		dir_ch1 = NO_OPERATION;
	}
}

// input capture�Ͽ� ����� ms������ pluse width�� ���� 
// input : pluse
// output : ����, ms������ pluse width�� ���� 
void __attribute__((__interrupt__)) _IC8Interrupt(void)
{
	// controller-ch4 (right, left meaurement)
	
	//_RE0 = ~_RE0;

	if(_RB5 == 1)
	{
		ReadCapture8(&timer_first_edge1);
	}
	else if(_RB5 == 0)
	{
		ReadCapture8(&timer_second_edge1);
		//Int_flag = 1;
		
		timer_pluse_width_raw1 = timer_second_edge1 - timer_first_edge1;
		timer_pluse_width_sec1 = (float)timer_pluse_width_raw1 * TICK * 8;	// Timer Prescale = 8
		timer_pluse_width_ms1 = (unsigned int)(timer_pluse_width_sec1 * 1000000);
	}
	else
	{
	}

	IFS1bits.IC8IF = 0;

	// ���� ���� 
	if(timer_pluse_width_ms1 >= CH_VOLT_REF + CH_VOLT_HYSERESIS)
	{
		dir_ch4 = LEFT;
	}
	else if(timer_pluse_width_ms1 < CH_VOLT_REF - CH_VOLT_HYSERESIS)
	{
		dir_ch4 = RIGHT;
	}
	else
	{
		dir_ch4 = NO_OPERATION;
	}
}

int main(void)
{

	io_init();
	motor_init();
	inputcapture_init();

	// Application Initialization
	Coeff = (float)((unsigned int)SPEED_MAX - SPEED_MIN) / ((unsigned int)CH_VOLT_MAX - CH_VOLT_REF);

	do
	{
		//RunDetectDir();

		//if((FORWARD == 0) && (BACK == 0))	// FORWARD�� BACKWORD�� ���ÿ� ������ SYSTEM ON�� 
		//if((FORWARD == 0) && (RIGHT == 0))	// FORWARD�� RIGHT�� ���ÿ� ������ SYSTEM ON�� 
		//if((dir_ch1 == FORWARD) && (dir_ch4 == RIGHT))	// FORWARD�� RIGHT�� ���ÿ� ������ SYSTEM ON�� 
		if(dir_ch1 == FORWARD)	// FORWARD ������ SYSTEM ON�� 
		{
			booting_delay_cnt++;
			if(booting_delay_cnt >= 60000)
			{
				booting_delay_cnt = 0;
				system_on = 1;

				ACTIVE_LOW_ENALBE_1 = 0;
				ACTIVE_LOW_ENALBE_2 = 0;
			}
			else
			{
				system_on = 0;
			}
		}
		else
		{
			booting_delay_cnt = 0;
			system_on = 0;
		}
	}while(system_on == 0);
		
	while(system_on)
	{	
	/*
		// SAMPLE CODE
		// PWM ON -> DELAY -> OFF -> DELAY �� �ݺ�
		// CH1�� ����� 
		// �� �����ڵ忡���� PWM1 H���� ���ϴ� ��� ���� (PWM1 L �� �ȵ�, ������ High��)
		
		SetDCMCPWM(1,0x3f00,0);	// 0x7f00 ���� ������ ����� �߰��� ����� ���� �߻��� (0x3f00���� test ������)
		for(i=0;i<10000;i++)
			for(j=0;j<2000;j++);

		SetDCMCPWM(1,0x0000,0);
		for(i=0;i<10000;i++)
			for(j=0;j<1000;j++);
	*/

		//RunDetectDir();

		RunMotor();
		
	}
}

void io_init(void)
{
	_PCFG4 = 1; // RORTB�� ADC �̿� ���� �ʼ�
	_PCFG5 = 1; // RORTB�� ADC �̿� ���� �ʼ� 

	TRISB = 0xFFFF;	// RB4, RB5 IC�� ��� 
	TRISC = 0xFFFF;
	TRISD = 0x0000;
	TRISE = 0x0100;

	// MANUAL_CONTROLL �ÿ��� Ȱ��ȭ 
	//FORWARD = 1;
	//BACK = 1;
	//RIGHT = 1;
	//LEFT = 1;

	ACTIVE_LOW_ENALBE_1 = 1;
	DIR_1 = 1;

	ACTIVE_LOW_ENALBE_2 = 1;
	DIR_2 = 0;
}

void motor_init(void)
{
	unsigned int period;
	unsigned int sptime;

	unsigned int config1; 
	unsigned int config2;
	unsigned int config3;

	ConfigIntMCPWM(PWM_INT_DIS & PWM_INT_PR0 & PWM_FLTA_DIS_INT & PWM_FLTA_INT_PR0);	// ���ͷ�Ʈ ������ 

	SetDCMCPWM(1,0x0000,0);	// CH1, Duty 0, update����(?)
	SetDCMCPWM(2,0x0000,0);	// CH2, Duty 0, update����(?)

	period = 0x7fff;	// �ֱⷹ������ �� 15bit 
	sptime = 0x0000;	// special event, �����ڵ忡���� ��� ���� 

	config1 = PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 & PWM_IPCLK_SCALE1 & PWM_MOD_FREE;	//  pwm enable, free running mode
	config2 = PWM_MOD1_COMP & PWM_MOD2_COMP & PWM_PEN1L & PWM_PEN1H & PWM_PEN2L & PWM_PEN2H & PWM_PDIS3L & PWM_PDIS3H;	// 1, 2 CH ��밡�� 
	config3 = PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN;
	
	OpenMCPWM(period, sptime, config1, config2, config3);
}

void inputcapture_init(void)
{
/////////////////////////////// Inputcapture Initilization ///////////////////////////////
	// Enable IC7 Interrupt and Priority to '1' 
	ConfigIntCapture7(IC_INT_PRIOR_1 & IC_INT_ON);

	// Configure the InputCapture7 in stop in idle mode , Timer
	// 3 as source , interrupt on capture 1, I/C on every edge 
	OpenCapture7(IC_IDLE_STOP & IC_TIMER3_SRC &
	IC_INT_1CAPTURE & IC_EVERY_EDGE);	// IC_EVERY_RISE_EDGE


	// Enable IC8 Interrupt and Priority to '1' 
	ConfigIntCapture8(IC_INT_PRIOR_1 & IC_INT_ON);

	// Configure the InputCapture8 in stop in idle mode , Timer
	// 3 as source , interrupt on capture 1, I/C on every edge 
	OpenCapture8(IC_IDLE_STOP & IC_TIMER3_SRC &
	IC_INT_1CAPTURE & IC_EVERY_EDGE);	// IC_EVERY_RISE_EDGE

	T3CONbits.TON = 1;		// Timer 3 On 
	T3CONbits.TCKPS0 = 1;	//Prescale (1:8), Internal clcok (Default)
	T3CONbits.TCKPS1 = 0;
}

/*
void RunDetectDir(void)
{
	// FORWARD, BACKWARD
	if(timer_pluse_width_ms0 <= CH_VOLT_REF - CH_VOLT_HYSERESIS)
	{
		dir_ch1 = FORWARD;
	}
	else if(timer_pluse_width_ms0 > CH_VOLT_REF + CH_VOLT_HYSERESIS)
	{
		dir_ch1 = BACK;
	}
	else
	{
		dir_ch1 = NO_OPERATION;
		timer_pluse_width_ms0 = 0;
	}

	// RIGHT, LEFT
	if(timer_pluse_width_ms1 <= CH_VOLT_REF - CH_VOLT_HYSERESIS)
	{
		dir_ch4 = LEFT;
	}
	else if(timer_pluse_width_ms1 > CH_VOLT_REF + CH_VOLT_HYSERESIS)
	{
		dir_ch4 = RIGHT;
	}
	else
	{
		dir_ch4 = NO_OPERATION;
		timer_pluse_width_ms1 = 0;
	}
}
*/

void RunMotor(void)
{
	long buf0,buf1;
/*	
	// 10 : RIGHT
	// 01: LEFT
	// 00: FORWARD
	// 11: BACKWORD
	if(FORWARD == 0)
	//if(dir_ch1 == FORWARD)
	{
		DIR_1 = 1;
		DIR_2 = 0;
	
		if(RIGHT == 0)
		{
			SetDCMCPWM(1,SPEED,0);
			SetDCMCPWM(2,SPEED_UP,0);
		}
		else if(LEFT == 0)
		{
			SetDCMCPWM(1,SPEED_UP,0);
			SetDCMCPWM(2,SPEED,0);
		}
		else
		{
			SetDCMCPWM(1,SPEED,0);
			SetDCMCPWM(2,SPEED,0);
		}
	}
	else if(BACK == 0)
	{
		DIR_1 = 0;	
		DIR_2 = 1;
	
		if(RIGHT == 0)
		{
			SetDCMCPWM(1,SPEED_UP,0);
			SetDCMCPWM(2,SPEED,0);
		}
		else if(LEFT == 0)
		{
			SetDCMCPWM(1,SPEED,0);
			SetDCMCPWM(2,SPEED_UP,0);
		}
		else
		{
			SetDCMCPWM(1,SPEED,0);
			SetDCMCPWM(2,SPEED,0);
		}
	}
	else if(RIGHT == 0)
	{
		DIR_1 = 1;
		DIR_2 = 1;
		
		SetDCMCPWM(1,SPEED,0);
		SetDCMCPWM(2,SPEED,0);
	}
	else if(LEFT == 0)
	{
		DIR_1 = 0;
		DIR_2 = 0;
		
		SetDCMCPWM(1,SPEED,0);
		SetDCMCPWM(2,SPEED,0);
	}
	else
	{
		SetDCMCPWM(1,STOP,0);
		SetDCMCPWM(2,STOP,0);
	}
*/	

	////////////////////////////////
	// input : pluse width, coeff
	// output : pwm value 
	buf0 = (long)(Coeff * timer_pluse_width_ms1 - 98301);	// remote ch4, r/l
	buf1 = (long)(Coeff * timer_pluse_width_ms0 - 98301);	// remote ch1, f/b

	if((buf0 >= 0x7fff) || (buf0 <= -32767))
	{
		ch_pwm[0] = 0x7fff;							// pwm max
	}
	else if(buf0 < 0)
	{
		ch_pwm[0] = (unsigned int)(buf0 * (-1));	// ���밪 ��ȯ
	}
	else
	{
		ch_pwm[0] = (unsigned int)buf0;				// remote ch4, r/l
	}
	
	if((buf1 >= 0x7fff) || (buf1 <= -32767))
	{
		ch_pwm[1] = 0x7fff;							// pwm max
	}
	else if(buf1 < 0)
	{
		ch_pwm[1] = (unsigned int)(buf1 * (-1));	// ���밪 ��ȯ
	}
	else
	{
		ch_pwm[1] = (unsigned int)buf1;				// remote ch1, f/b
	}
	////////////////////////////////


	// 10 : RIGHT
	// 01: LEFT
	// 00: FORWARD
	// 11: BACKWORD

	// ch_pwm[1] : ch1(f/b), ch_pwm[0] : ch4(r/l)
	if(dir_ch1 == FORWARD)
	{
		DIR_1 = 1;
		DIR_2 = 0;

		if(dir_ch4 == RIGHT)
		{
			//SetDCMCPWM(1,ch_pwm[0],0);	// SetDCMCPWM1 : left wheel
			SetDCMCPWM(1,0x7f00,0);	// SetDCMCPWM1 : left wheel
			SetDCMCPWM(2,ch_pwm[1],0);	// SetDCMCPWM2 : right wheel
		}
		else if(dir_ch4 == LEFT)
		{
			SetDCMCPWM(1,ch_pwm[1],0);
			//SetDCMCPWM(2,ch_pwm[0],0);
			SetDCMCPWM(2,0x7f00,0);
		}
		else
		{
			SetDCMCPWM(1,ch_pwm[1],0);
			SetDCMCPWM(2,ch_pwm[1],0);
		}
	}
	else if(dir_ch1 == BACK)
	{
		DIR_1 = 0;	
		DIR_2 = 1;

		if(dir_ch4 == RIGHT)
		{
			//SetDCMCPWM(1,ch_pwm[0],0);
			SetDCMCPWM(1,0x7f00,0);
			SetDCMCPWM(2,ch_pwm[1],0);
		}
		else if(dir_ch4 == LEFT)
		{
			SetDCMCPWM(1,ch_pwm[1],0);
			//SetDCMCPWM(2,ch_pwm[0],0);
			SetDCMCPWM(2,0x7f00,0);
		}
		else
		{
			SetDCMCPWM(1,ch_pwm[1],0);
			SetDCMCPWM(2,ch_pwm[1],0);
		}
	}
	else if(dir_ch4 == RIGHT)
	{
		DIR_1 = 1;
		DIR_2 = 1;
		
		SetDCMCPWM(1,ch_pwm[0],0);
		SetDCMCPWM(2,ch_pwm[0],0);
	}
	else if(dir_ch4 == LEFT)
	{
		DIR_1 = 0;
		DIR_2 = 0;
		
		SetDCMCPWM(1,ch_pwm[0],0);
		SetDCMCPWM(2,ch_pwm[0],0);
	}
	else
	{
		SetDCMCPWM(1,STOP,0);
		SetDCMCPWM(2,STOP,0);
	}
}

