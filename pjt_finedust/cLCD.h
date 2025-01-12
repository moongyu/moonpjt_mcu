#include "cbin2dec.h"
#include <p30f3013.h>

////////////////////////////////
// LCD 3PIN 설정변경 

#define MASK_ZERO_RB6_9_ZERO	(0xfc3f)
#define SHFIF_RB6_9				(6)

/*
#define dRS   LATBbits.LATB0
#define dE    LATBbits.LATB2
#define dR_W  LATBbits.LATB1
//#define dBusy PORTDbits.RD3

#define dRS   LATBbits.LATB3
#define dE    LATBbits.LATB5
#define dR_W  LATBbits.LATB4
*/

#define dRS   LATBbits.LATB0
#define dE    LATBbits.LATB2
#define dR_W  LATBbits.LATB1

////////////////////////////////

#define BYTE unsigned char
#define WORD unsigned long

void Int2Ascii(int value, int *pstring);
void Wait(WORD count);

//---------------------------------------------------------------------
int iInt_Ascii[4];
//---------------------------------------------------------------------
void WrtLCDByte(BYTE bValue)
{
    BYTE bTemp;
    bTemp = bValue;

    // write ms nibble to port
    //LATE = (LATE & 0xFFF0) | (bTemp >> 4 & 0x000F); 
    LATB = (LATB & MASK_ZERO_RB6_9_ZERO) | ((bTemp >> 4 & 0x000F)<<SHFIF_RB6_9); 

    // make PORTE output 
    //TRISE = TRISE & 0xFFF0;	// 불필요 

    // clear R/!W for write
    dR_W = 0;
    
    // toggle enable for > 1 usec
    dE = 1;
    Wait(5);
    dE = 0;

    Wait(5);

    // write ls nibble to port
    //LATE = (LATE & 0xFFF0) | (bValue & 0x000F); 
    LATB = (LATB & MASK_ZERO_RB6_9_ZERO) | ((bValue & 0x000F)<<SHFIF_RB6_9); 

    // clear R/!W for write
    dR_W = 0;
    
    
    // toggle enable for > 1 usec
    dE = 1;
    Wait(5);
    dE = 0;

    //make PORTE input;
    //TRISE =  TRISE | 0x000F;	// 불필요 
                        
}
//---------------------------------------------------------------------
void WrtLCDInst(BYTE bValue)
{
          // Wait 1milsec for not busy
          Wait(1000);

          // clear RS for Instruction
          dRS = 0;
          WrtLCDByte(bValue);
}
//---------------------------------------------------------------------
void WrtLCDData(BYTE bValue)
{
     // Wait 1 milsec for not busy
     Wait(1000);      

     // set RS for Data
     dRS = 1;
     WrtLCDByte(bValue);
}
//---------------------------------------------------------------------
void WrtChrNext(BYTE bNumber)
{
	WrtLCDData (bNumber);
} 
//---------------------------------------------------------------------
void ChrPos (BYTE bChrPosC, BYTE bChrPosR)
{
	WrtLCDInst (0x80 | bChrPosC | (bChrPosR << 6));
}
//---------------------------------------------------------------------
void Wrt_Int_LCD( int iNumber, BYTE bChrPosC, BYTE bChrPosR)
{

	int *iP;
	int i;
	int buf;

	iP= &iInt_Ascii[0];

	Int2Ascii(iNumber,iP);			//Convert Int in w0 to 4 digit Ascii @ Int_Ascii

	ChrPos(bChrPosC, bChrPosR);
	for (i=0; i<4; i=i + 1)
	{
		buf=*iP++;
		if(i>1) WrtChrNext(buf);
	}	
}
//---------------------------------------------------------------------
void Wrt_S_LCD(char s[], BYTE bChrPosC, BYTE bChrPosR)
{

	BYTE i = 0;
	
	ChrPos(bChrPosC, bChrPosR);
	while (s[i] != '\0')
	{
		WrtChrNext(s[i]);
        i++;
	}	
}
//---------------------------------------------------------------------
void Wrt_Signed_Int_LCD(int iNumber , BYTE bChrPosC, BYTE bChrPosR)
{    
    if (iNumber < 0)
    {
      iNumber = -iNumber;
    }
      bin2dec(iNumber); 
  		     
  	  WrtChrNext(thousands);           
      WrtChrNext(hundreds);
      WrtChrNext(tens);
      WrtChrNext(ones);
}            
//---------------------------------------------------------------------
void InitLCD(void)
{
   Wait(0xFFFF);
   Wait(0xFFFF);
   Wait(0xFFFF);
   Wait(0xFFFF);	

   // write ms nibble to port
   //LATE = (LATE & 0xFFF0) | (3); 
   LATB = (LATB & MASK_ZERO_RB6_9_ZERO) | ((3)<<SHFIF_RB6_9) ; 
   

   // make PORTE output 
   //TRISE = TRISE & 0xFFF0;
   TRISB = TRISB & MASK_ZERO_RB6_9_ZERO;

   // clear R/!W for write
   dR_W = 0;
    
   // toggle enable for > 1 usec
   dE = 1;
   Wait(5);
   dE = 0;

    Wait(0xFFFF);
    
   
    // write ls nibble to port
    //LATE = (LATE & 0xFFF0) | (3);
    LATB = (LATB & MASK_ZERO_RB6_9_ZERO) | ((3)<<SHFIF_RB6_9) ; 
    
    
    // toggle enable for > 1 usec
    dE = 1;
    Wait(5);
    dE = 0;

    // write ms nibble to port
    //LATE = (LATE & 0xFFF0) | (3); 
    LATB = (LATB & MASK_ZERO_RB6_9_ZERO) | ((3)<<SHFIF_RB6_9) ; 

    // make PORTD output 
    //TRISE = TRISE & 0xFFF0;		// 불필요 

    // clear R/!W for write
    dR_W = 0;
    
    // *** Added for LCD timing
    Wait(5);

    // toggle enable for > 1 usec
    dE = 1;
    Wait(5);
    dE = 0;

    Wait(100);

    // write ls nibble to port
    //LATE = (LATE & 0xFFF0) | (2); 
    LATB = (LATB & MASK_ZERO_RB6_9_ZERO) | ((2)<<SHFIF_RB6_9) ; 
    
    // toggle enable for > 1 usec
    dE = 1;
    Wait(5);
    dE = 0;

                    
    WrtLCDInst(0x28);   // Function set: 4 bit data, L1652                               
    WrtLCDInst(0x06);   // Entry Mode: Inc 1, no shift
    WrtLCDInst(0x0C);   // Display ON, Cursor off, no blink
    WrtLCDInst(0x01);   // Clear Display
    WrtLCDInst(0x80);   // Set Cursor to top left

}	
//---------------------------------------------------------------------
void HomeLCD(void)
{
	WrtLCDInst (0x02); // Home
}
//---------------------------------------------------------------------
void HomeClearLCD(void)
{
	WrtLCDInst (0x02);  // Home
    WrtLCDInst (0x01);  // Clear Display
}
//---------------------------------------------------------------------
// total cycles = (5 x count) + 3 , 135.64nsec/cycle, 1msec=7372, ffff = 8.8msec 
void Wait(WORD count)
{
    while(count--);
}
//---------------------------------------------------------------------
