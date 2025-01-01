#ifndef _LCD_H
#define _LCD_H
#include "delay.h"
//      D4 - PORTE0
//      D5 - PORTE1
//      D6 - PORTE2
//      D7 - PORTE3
//      RS - PORTE4
//      R/W - PORTE5
//      EN - PORTE8
//#define P1 LATBbits.LATB0

#define RS	LATEbits.LATE4	//Lcd RS	PORTE4
#define RW	LATEbits.LATE5	//Lcd RW	PORTE5
#define E	LATBbits.LATB4	//Lcd E		PORTC15
// Lcd ����� �� ��Ʈ�� �����Ѵ�.
#define C_DISP_CURSOR  0x10
#define DISP_ON        0x04
#define CURSOR_ON      0x0F
#define CURSOR_OFF		0x0C
#define BLINK_ON       0x04

#define ID_AC_INC        0x02   
#define S_DISP_SHIFT_ON  0x01

#define SC_DISP_SHIFT  0x08
#define RL_RIGHT_SHIFT 0x04

#define LCD_LEFT_SHIFT   C_DISP_CURSOR|SC_DISP_SHIFT 
#define LCD_RIGHT_SHIFT  C_DISP_CURSOR|SC_DISP_SHIFT|RL_RIGHT_SHIFT 



//LCD ���� �Լ�
void Init_Lcd(void);
void Lcd_Instruction(unsigned char);//LCD ��ɾ� ����
void Lcd_Data(unsigned char);//LCD ������ ����
void Lcd_Clear(void);	//LCD ȭ�� Ŭ����
void Lcd_Position(char x, char y);//LCD ��� ��ġ x,y
void Lcd_String(char, char, unsigned char *);// LCD�� x,y��ġ�� ���� ���
void Lcd_Char(char, char,char, unsigned char);
//LCD ���  ���� �Լ�	   
void Lcd_Instruction(unsigned char Instruction)
{
	Delay_us(30);
	LATB = ((Instruction>>4)&0x0f); //���� ����Ʈ�� ��ɾ� ����
	RS =0;	//��ɾ� ���� ���
	RW=0;
	E=1;
	E=0;
	LATB = (Instruction&0x0f);	//���� ����Ʈ ��ɾ� ����
	RS =0;
	RW=0;
	E=1;
	E=0;
	Delay_us(30);	
}

//������ ���� �Լ�
void Lcd_Data(unsigned char Data)
{
	Delay_us(30);
	LATB = ((Data>>4)&0x0f); //���� ����Ʈ ������ ����
	RS =1;	//������ ���� ���
	RW=0;
	E=1;
	E=0;
	LATB = (Data&0x0f);	//���� ����Ʈ ������ ����
	RS =1;
	RW=0;
	E=1;
	E=0;
	Delay_us(30);
}

//LCD �ʱ�ȭ
void Init_Lcd(void)
{
	Delay_ms(30);
	E=0;
	
	Lcd_Instruction(0x20); //4bit�� ���� (���� ��� ��)
	Delay_us(50);
	Lcd_Instruction(0x20);
	Delay_us(50);
	Lcd_Instruction(0x20);
	
	//Function Set (4bit,2-Line,Font 5*11)
	Lcd_Instruction(0x28);
	Delay_us(50);
	
	//Display On/Off Control ( Display ON, Cursor OFF, blinking of cursor OFF)
	Lcd_Instruction(0x0c);//
	Delay_us(50);

	Lcd_Instruction(0x01);//LCD Clear
	Delay_us(50);

// Entry Mode Set , ���������� ����, ȭ���� ����Ʈ ����
	Lcd_Instruction(0x06);	

	Delay_ms(50);
}

void Lcd_Clear(void)
{
	 Lcd_Instruction(0x01);
}
void Lcd_String(char x, char y,unsigned char *text)
{
	Lcd_Position(x,y);
	while(*text)
	{
		Lcd_Data(*(text++));
	}
}
void Lcd_Position(char x, char y)
{
	unsigned char Position;
	if(y>1)y=1;
	if(x>15)x=15;
	Position= (y==1)?x+0xc0:x+0x80;
	Lcd_Instruction(Position);
}
void Lcd_Char(char x, char y, char type,unsigned char text)
{
	Lcd_Position(x,y);
	if(type == 0)
		Lcd_Data(text);
	else if(type == 1)
	{
		Lcd_Data((text/10)+0x30);	
		Lcd_Data((text%10)+0x30);
	}
}
// LCD_data((number >> 4) + '0');
//  LCD_data((number & 0x0F) + '0');
#endif
