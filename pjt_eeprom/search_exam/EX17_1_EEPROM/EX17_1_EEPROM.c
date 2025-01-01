/********************************************************************
*
*  Filename:   EX17_1_EEPROM.C 
*  Purpose:    Example to show how to read and write the EEPROM.
*              Also shown the means to read data using
*              Prgram Space Visibility
*
*********************************************************************/

#define __dsPIC30F4011__

/* header files */
#include 	<p30f4011.h>
#include 	"eeprom.h"     /* defines eeprom macros */
#include 	<uart.h>     /* uart peripheral library header file */
#include	<adc10.h>				// 將adc10函式的?型宣告檔?含入

#define		FCY 	7372800 * 2		// ?為使用頻率為將?部 7.3728 MHz * 8 的模式 , 每??令週期需 4 ? clock
									// 所以 FCY =  (7.3728 * 8 / 4 ) MHz = 7372800* 2

//---------------------------------------------------------------------------
// 請參考此處有關 Configuration bits 於?式中直接宣告的方式
// ?詳細的參數列表請參考 p30F4011.h 
//---------------------------------------------------------------------------
   	_FOSC(CSW_FSCM_OFF & XT_PLL8);   //	XT with 8xPLL oscillator, Failsafe clock off
   	_FWDT(WDT_OFF);                  //	Watchdog timer disabled
  	_FBORPOR(PBOR_OFF & MCLR_EN);    //	Brown-out reset disabled, MCLR reset enabled
   	_FGS(CODE_PROT_OFF);             //	Code protect disabled



/* globals */
char s0[] = "\r\n**********************************************************\r\n";
char s1[] = "\nEX 17 - EEPROM\r\n";
char s2[] = "Switch Options\r\n";
char s3[] = "\t[SW1] Read EEPROM\r\n";   
char s4[] = "\t[SW2] Program EEPROM by word\r\n";   
char s5[] = "\t[SW3] Program EEPROM by row\r\n";
char s6[] = "\t[SW4] Erase EEPROM\r\n";
char s7[] = "\tPush button to program the EEPROM\r\n";

char sWord[] = "\n[SW2] - EEPROM Program by word set\r\n";
char sRow[] = "\n[SW3] - EEPROM Program by row set\r\n";
char sErase[] = "\n[SW4] - EEPROM Erased\r\n";

unsigned config1, config2;
unsigned programming_mode = WORD_MODE;
unsigned new_word, new_row[NUM_EE_WORDS_PER_ROW];
unsigned num_new_words=0;
unsigned row_offset_address;
unsigned word_offset_address;

/* volatile data - used by ISRs */
unsigned char SW;

/* externs */
extern unsigned int _EEDATA(2) word_buffer[256];
extern unsigned int _EEDATA(2) row_buffer[256];


/* funtion prototypes */
void 	Init_ADC(void) ;
void 	Init_UART(void);
void	Show_ADC(void) ;
void	sitoa(unsigned char, unsigned char *TXdata ) ;
unsigned char	AN_Key(unsigned int) ;


/* source code */
/*************************************************************************
*                                                                         
* Function:  main() 
* Purpose:   This is the main executive for LAB2 of the DHO824 class.
*
* Argument:  none
* Returns:   none
*
**************************************************************************/
int main (void) {

	unsigned char dummy ;

   /* initialize addresses for programming */
   row_offset_address = (unsigned) &row_buffer;
   word_offset_address = (unsigned) &word_buffer;
   
   /* erase the EEPROM */
   EraseEE ();
   
   /* setup the ADC switches */
   Init_ADC();

   /* setup the UART for i/o */
   Init_UART();

   /* dump the introductory message */
   putsUART1 ((int*)s0);
   putsUART1 ((int*)s1);
   putsUART1 ((int*)s7);
   putsUART1 ((int*)s2);
   putsUART1 ((int*)s3);
   putsUART1 ((int*)s4);
   putsUART1 ((int*)s5);
   putsUART1 ((int*)s6);
   putsUART1 ((int*)s0);

   while (1)
   {

     Show_ADC();

     switch(SW) {
	 case('1') : 
         /* read the programmed locations */
         ReadEE();
         break ;
         
	 case('2') : 
         /* set the word programming mode */
         /* start programming at 0x7FF000 */
         programming_mode = WORD_MODE;
         putsUART1 ((int*)sWord);
         break;
         
	 case('3') : 
         /* set the row programming mode */
         /* start programming at 0x7FF800 */
         programming_mode = ROW_MODE;
         putsUART1 ((int*)sRow);
         break;
      
	 case('4') : 
         /* erase the entire Data EEPROM */
         EraseEE();
         putsUART1 ((int*)sErase);
         /* reset the programming parameters */
         row_offset_address = (unsigned) &row_buffer;
         word_offset_address = (unsigned) &word_buffer;
         num_new_words = 0;
         break;
     }
   
     if ( (SW == '2') || (SW == '3') ){
     	dummy = '0' ;
     	do  {
			 putsUART1((int *)"\r\nSelect VR Value to be saved to EEPROM (1/2) : ");
         	while(!DataRdyUART1()); 
         	dummy = ReadUART1();
         	WriteUART1(dummy);
    		putsUART1((int *)"\r\n");
     	} while( (dummy < '1') || (dummy > '2') );

     	if (dummy == '1') new_word = ADCBUF1;
     	else new_word = ADCBUF0;

     	if (programming_mode == WORD_MODE) {
     		/* program the new word */
     		ProgramEEWord (new_word);
     	}
     	else {
     		/* we add the new word to our row to program */
			for (num_new_words = 0;num_new_words < NUM_EE_WORDS_PER_ROW;num_new_words++) {
     		new_row[num_new_words] = new_word;
			}

        	ProgramEERow ();
     	}
	}
   } // end of while(1)
        
}  /* end main() */
    

/***********************************************/
// Subroutine to initialize UART module

void	Init_UART(void)
{
	/* Holds the value of baud register */
	unsigned int baudvalue;
	/* Holds the value of uart config reg */
	unsigned int U1MODEvalue;
	/* Holds the information regarding uart
	TX & RX interrupt modes */
	unsigned int U1STAvalue;
	/* Turn off UART1module */
	CloseUART1();
	/* Configure uart1 receive and transmit interrupt */
	ConfigIntUART1(UART_RX_INT_DIS & UART_RX_INT_PR6 &
	UART_TX_INT_DIS & UART_TX_INT_PR2);
	/* Setup the Buad Rate Generator */
	baudvalue = 95;			//UxBRG = ( (FCY/Desired Baud Rate)/16) – 1
							//UxBRG = ( (7372800*2/9600)/16-1) = 95
	/* Configure UART1 module to transmit 8 bit data with one stopbit.
	Also Enable loopback mode */
	U1MODEvalue = UART_EN & UART_IDLE_CON &
				UART_DIS_WAKE & UART_DIS_LOOPBACK &
				UART_DIS_ABAUD & UART_NO_PAR_8BIT &
				UART_1STOPBIT;
	U1STAvalue = UART_INT_TX_BUF_EMPTY &
				UART_TX_ENABLE & UART_INT_RX_CHAR &
				UART_ADR_DETECT_DIS &
				UART_RX_OVERRUN_CLEAR;
	OpenUART1(U1MODEvalue, U1STAvalue, baudvalue);

	return;

}

/***********************************************/
// Subroutine to initialize ADC module

void	Init_ADC(void)
{

	unsigned int Channel, PinConfig, Scanselect, Adcon3_reg, Adcon2_reg, Adcon1_reg;

	ADCON1bits.ADON = 0; /* turn off ADC */

	PinConfig = ENABLE_AN0_ANA &		// Select port pins as analog inputs ADPCFG<15:0>
				ENABLE_AN6_ANA &
				ENABLE_AN8_ANA ;
	Adcon1_reg = ADC_MODULE_ON &		// Turn on A/D module (ADON)
		ADC_IDLE_STOP &					// ADC turned off during idle (ADSIDL)
		ADC_FORMAT_INTG &				// Output in integer format (FORM)
		ADC_CLK_AUTO &					//*Conversion trigger automatically (SSRC)
		ADC_SAMPLE_INDIVIDUAL &			//*Sample channels individually (SIMSAM)
		ADC_AUTO_SAMPLING_ON;			//*Sample trigger automatically (ASAM)
	Adcon2_reg = ADC_VREF_AVDD_AVSS &	// Voltage reference : +AVdd, -AVss (VCFG)
		ADC_SCAN_ON &					//*Scan on (CSCNA)
		ADC_ALT_BUF_OFF &				// Use fixed buffer (BUFM)
		ADC_ALT_INPUT_OFF &				// Does not alternate between MUX A & MUX B (ALTS)
		ADC_CONVERT_CH0 &				//*Convert only channel 0 (CHPS)
		ADC_SAMPLES_PER_INT_3;			//*3 samples between interrupt (SMPI)
	Adcon3_reg = ADC_SAMPLE_TIME_16 &	// Auto-Sample time (SAMC)
		ADC_CONV_CLK_SYSTEM &			// Use system clock (ADRC)
		ADC_CONV_CLK_4Tcy;				// Conversion clock = 4 Tcy (ADCS)
										// ADCS = 2*(154ns)/(1/Fcy)-1 = 3.5416
										// TAD = (ADCS+1)/(2*Fcy) = 169.54ns
	Scanselect = SKIP_SCAN_AN1 &SKIP_SCAN_AN2  &	//*ADC scan channel 0,1,6(ADCSSL)
				SKIP_SCAN_AN3 & SKIP_SCAN_AN4  & 
				SKIP_SCAN_AN5 & SKIP_SCAN_AN7  & 
				SKIP_SCAN_AN9  & SKIP_SCAN_AN10 &
				SKIP_SCAN_AN11 & SKIP_SCAN_AN12 &
				SKIP_SCAN_AN13 & SKIP_SCAN_AN14 &
				SKIP_SCAN_AN15;
	OpenADC10(Adcon1_reg, Adcon2_reg, Adcon3_reg, PinConfig, Scanselect);

	Channel = ADC_CH0_POS_SAMPLEA_AN0 &	//*CH0 Pos. : AN0, AN6, AN8, 
			ADC_CH0_POS_SAMPLEA_AN6 &	// Neg. : Nominal Vref- Defined in ADCON2
			ADC_CH0_POS_SAMPLEA_AN8 &
			ADC_CH0_NEG_SAMPLEA_NVREF ;	// (ADCHS)
	SetChanADC10(Channel);

	ConfigIntADC10(ADC_INT_DISABLE);	// Disable ADC interrupt

}

/***********************************************/
// Subroutine to show ADC on LCD

void	Show_ADC(void)
{
	unsigned int ADCValue;
	
	IFS0bits.ADIF = 0;
	ADCON1bits.ASAM = 1;			//*start sampling & conversion ...
	while(!IFS0bits.ADIF) ;
	ADCON1bits.ASAM = 0;			//*stop sampling & conversion ...

//	ADCValue[0] = (ReadADC10(0)>>2) ;
//	ADCValue[1] = (ReadADC10(1)>>2) ;
	ADCValue = ReadADC10(2) ;

	SW = AN_Key( ADCValue );				// 將類比轉換按鍵結果以數字顯?
}

/***********************************************/
// Subroutine to ANALOG KEYS on LCD

unsigned char	AN_Key(unsigned int ADCValue)
{

	unsigned char SW;
	ADCValue >>=6;
		
	if (ADCValue == 0)	(SW = '1') ;
	else 
	{	if(ADCValue <= 8) (SW = '2') ;
		else 
		{	if(ADCValue <= 10) (SW = '3') ;
			else 
			{	if(ADCValue <= 12) (SW = '4') ;
				else (SW = ' ') ;
			}
		}
	}

	return SW;
}
