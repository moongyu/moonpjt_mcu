#ifndef _DELAY_H
#define _DELAY_H

//시간 지연 함수
void Delay_us(unsigned char);
void Delay_ms(unsigned int);

//uSecond Delay
void Delay_us(unsigned char time_us)
{
	register unsigned char i;
	for(i=0; i<time_us; i++)
	{
		asm volatile("NOP");
		asm volatile("NOP");
		asm volatile("NOP");
		asm volatile("NOP");
		asm volatile("NOP");
		asm volatile("NOP");
		asm volatile("NOP");
		asm volatile("NOP");
	}
}

//mSecond Delay
void Delay_ms(unsigned int time_ms)		/* time delay for ms */
{ register unsigned int i;

  for(i = 0; i < time_ms; i++)
    { Delay_us(250);
      Delay_us(250);
      Delay_us(250);
      Delay_us(250);
    }
}
#endif
