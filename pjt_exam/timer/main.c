//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//180302 Timer1�� 1.5ms pluse ����� 
// RE Test pin���� �������� 

// �ֿ� ��������

// Fosc, Fcy ����
// PLL POST�� ���� Prescale ���� (1:1���)
// Fosc�� PLL post���� clock���� ������ 
// Timer Prescale ���� (Fcy ���� Divide��)

// ���� 
// Fosc, Fcy, POST, Timer Prescale ������ �� 

// �̻��� �� 
// TMRn=0 or ReadTimern �����ϴ� �� ������ WatchȮ�ξȵ� 


// 180926 1ms timer ����� 
// Fosc = Fcy / 4
//         = (8Mhz * PLL 8) / 4
//         = 16Mhz (=> 0.0625us)
// OpenTimer3()���� 16000 ������ 0.0625us* 16000 = 1ms ������ 
// PJT Charger���� ���� �� 
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


#include <p30f4012.h>
#include <stdio.h>
#include <timer.h>

_FOSC(CSW_FSCM_OFF & EC_PLL8);
//_FOSC(CSW_FSCM_OFF & FRC);
_FWDT(WDT_OFF);                 
_FBORPOR(MCLR_EN & PWRT_OFF);   
_FGS(CODE_PROT_OFF);     

void io_init(void);
void timer_init(void);

unsigned int timer_value=0;

void __attribute__((__interrupt__)) _T1Interrupt(void)
{

	PORTE^=0xffff;
	WriteTimer1(0);

	//timer_value=ReadTimer1();	// watch�� Ȯ�ξȵ� 
	//timer_value=TMR1;			// watch�� Ȯ�ξȵ� 
	
	IFS0bits.T1IF=0;

	_RD0 = 1;
}

void __attribute__((__interrupt__)) _T3Interrupt(void)
{

	PORTE^=0xffff;
	WriteTimer3(0);

	IFS0bits.T3IF=0;

	//_RD0 = 1;
}

int main()
{
	io_init();
	timer_init();

	while(1)
	{
		// �������ϳ� timer_value��  watch�� Ȯ�ξȵ�  
/*		timer_value=ReadTimer1();
		if(timer_value >= 12000)
		{
			PORTE^=0xffff;
			timer_value = 0;
			WriteTimer1(0);
		}
		else
		{
		}
*/
		// test ���� ���� ���� 
		//timer_value=TMR1;
		//printf("%d\r\n",timer_value);
		//if(TMR1==30000) printf("%d\r\n",TMR1);
	}

	closeTimer1();
	closeTimer3();
}

void timer_init()
{
	ConfigIntTimer1(T1_INT_PRIOR_3 &			// 0~7 => �ּ� 1 �̻� ������ ��
				T1_INT_ON);					// ON, OFF

	WriteTimer1(0);

	OpenTimer1(	T1_ON &						// ON, OFF
				T1_IDLE_STOP &					// CON, STOP
				T1_GATE_OFF &					// ON, OFF
				T1_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128
				T1_SYNC_EXT_OFF &  			// ON, OFF (ON �ʼ�)
				T1_SOURCE_INT,	         	// EXT, INT
				25000);						// Match_value		// 1.5ms �ֱ�

	// Timer3 ; �ֱ� ���� ����, ���ͷ�Ʈ �߻��� Ȯ��  
	ConfigIntTimer3(T3_INT_PRIOR_3 &			// 0~7 => �ּ� 1 �̻� ������ ��
				T3_INT_ON);					// ON, OFF

	WriteTimer3(0);

	OpenTimer3(	T3_ON &						// ON, OFF
				T3_IDLE_STOP &					// CON, STOP
				T3_GATE_OFF &					// ON, OFF
				T3_PS_1_1&				 	// 1_1, 1_8, 1_64, 1_128

				T3_SOURCE_INT,	         	// EXT, INT
				25000);						// Match_value		// 1.5ms �ֱ� 				
}

void io_init(void)
{
	TRISE = 0x0000;
	PORTE=0x0000;

	_TRISD0 = 0;
	_RD0 = 0;
}
