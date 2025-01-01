//////////////////////////////////////////////////////////////////////////////////////
// 220826 lcd 표기 
//////////////////////////////////////////////////////////////////////////////////////

#include "disp_lcd_convertvalue.h"


#define NUM_10_BILI		(1000000000)
#define NUM_1_BILI		(100000000)
#define NUM_1000_MILI	(10000000)
#define NUM_100_MILI	(1000000)
#define NUM_10_MILI		(100000)
#define NUM_1_MILI		(10000)
#define NUM_1000		(1000)
#define NUM_100			(100)
#define NUM_10			(10)
#define NUM_1			(1)

///////////////////////////////////////////////////////////////
unsigned char Integer2string10_Bili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	// input 범위 : 40억 미만 
	if((tempval >= NUM_10_BILI * 4) || (tempval < NUM_10_BILI))
	{
		tempval = ' ';
		return(tempval);
	}
	else
	{
		tempval = tempval / NUM_10_BILI;
	}

	retval = (unsigned char)tempval + '0';
	
	return(retval);
}

unsigned char Integer2string1_Bili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;
	
	if(tempval < NUM_1_BILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval / NUM_1_BILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char Integer2string1000_Mili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_1000_MILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval / NUM_1000_MILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char Integer2string100_Mili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_100_MILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}

	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval / NUM_100_MILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char Integer2string10_Mili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_10_MILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}

	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval / NUM_10_MILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char Integer2string1_Mili(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_1_MILI)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}

	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval / NUM_1_MILI;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char Integer2string1000(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_1000)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}
	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval % NUM_1_MILI;
	}
	
	if(tempval >= NUM_1000)
	{
		tempval = tempval / NUM_1000;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char Integer2string100(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_100)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}
	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval % NUM_1_MILI;
	}
	if(tempval >= NUM_1000)
	{
		tempval = tempval % NUM_1000;
	}
	
	if(tempval >= NUM_100)
	{
		tempval = tempval / NUM_100;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char Integer2string10(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;

	if(tempval < NUM_10)
	{
		tempval = ' ';
		return(tempval);
	}

	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}
	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval % NUM_1_MILI;
	}
	if(tempval >= NUM_1000)
	{
		tempval = tempval % NUM_1000;
	}
	if(tempval >= NUM_100)
	{
		tempval = tempval % NUM_100;
	}
	
	if(tempval >= NUM_10)
	{
		tempval = tempval / NUM_10;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}

unsigned char Integer2string1(unsigned long val)
{
	unsigned char retval;
	unsigned long tempval;

	tempval = val;
	
	if(tempval >= NUM_10_BILI)
	{
		tempval = tempval % NUM_10_BILI;
	}
	if(tempval >= NUM_1_BILI)
	{
		tempval = tempval % NUM_1_BILI;
	}
	if(tempval >= NUM_1000_MILI)
	{
		tempval = tempval % NUM_1000_MILI;
	}
	if(tempval >= NUM_100_MILI)
	{
		tempval = tempval % NUM_100_MILI;
	}
	if(tempval >= NUM_10_MILI)
	{
		tempval = tempval % NUM_10_MILI;
	}
	if(tempval >= NUM_1_MILI)
	{
		tempval = tempval % NUM_1_MILI;
	}
	if(tempval >= NUM_1000)
	{
		tempval = tempval % NUM_1000;
	}
	if(tempval >= NUM_100)
	{
		tempval = tempval % NUM_100;
	}
	if(tempval >= NUM_10)
	{
		tempval = tempval % NUM_10;
	}
	
	if(tempval >= NUM_1)
	{
		tempval = tempval / NUM_1;
	}
	else
	{
		tempval = 0;
	}
	
	retval = (unsigned char)tempval + '0';

	return(retval);
}


