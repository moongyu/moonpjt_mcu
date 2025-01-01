
/********************************************************************
*	FCY for Delay
********************************************************************/

/* To use __delay_us() or __delay_ms(), FCY has to be set. */
/* The value of FCY is clock value which is set by user.   */
/* Fcy = Fosc / 4 */
#define FCY	(5000000 / 4)


/********************************************************************
*	Header Files
********************************************************************/

#include <xc.h>					// Mandatory header file

//#include "p30F3010.h"
#include "libpic30.h"		// for __delay_us())

#include <stdio.h>
#include <string.h>


/********************************************************************
*	Macro Definition
********************************************************************/

#define __TEST_EEPROM_BLOCK_OPERATION__

#define BAUDRATE						9600


/********************************************************************
*	Global/Static Function
********************************************************************/

static void DRV_UART_Init(void);

static void EEPROM_EraseWord(unsigned long addr);
static void EEPROM_EraseBlock(unsigned long addr);

static void EEPROM_WriteWord(unsigned long addr, unsigned short value);
static void EEPROM_WriteBlock(unsigned long addr, unsigned short *pbuf);

static void EEPROM_ReadWord(unsigned long addr, unsigned short *pvalue);



/********************************************************************/
/* printf() in stdio.h needs putch() function.                      */
/********************************************************************/

void putch(unsigned char data)
{
    U1TXREG = data;
    while(U1STAbits.TRMT == 0);
}



/********************************************************************
* Function		: main
* Description	: Main application to test
* Argument		: None
* Return		: None
********************************************************************/

int main(void)
{
	unsigned long eeprom_addr;
	unsigned short value, rd_value;
	unsigned short buffer[16] = { 0xAC00, 0xAC01, 0xAC02, 0xAC03,0xAC04, 0xAC05, 0xAC06, 0xAC07,
								  0xAC08, 0xAC09, 0xAC0A, 0xAC0B,0xAC0C, 0xAC0D, 0xAC0E, 0xAC0F };
	unsigned short count;

	PORTEbits.RE0 = 0;
	TRISEbits.TRISE0 = 0;

	/* Initialize a UART 1. */
	DRV_UART_Init();

	printf("[XC-16 tool chain]\n");
	printf("sizeof(char) = %d byte\n", sizeof(char));
	printf("sizeof(short) = %d bytes\n", sizeof(short));
	printf("sizeof(int) = %d bytes\n", sizeof(int));
	printf("sizeof(long) = %d bytes\n\n", sizeof(long));

	/* Test to write one word */
	eeprom_addr = 0x7FFD10;
	value = 0x1234;
	EEPROM_WriteWord(eeprom_addr, value);

	/* Test to write one word */
	eeprom_addr = 0x7FFD12;
	value = 0x5678;
	EEPROM_WriteWord(eeprom_addr, value);

	/* Test to read one word */
	eeprom_addr = 0x7FFD10;
	EEPROM_ReadWord(eeprom_addr, &rd_value);
	printf("[R] I-EEPROM address [0x%lX]: rd_value = 0x%X\n", eeprom_addr, rd_value);

	/* Test to read one word */
	eeprom_addr = 0x7FFD12;
	EEPROM_ReadWord(eeprom_addr, &rd_value);
	printf("[R] I-EEPROM address [0x%lX]: rd_value = 0x%X\n", eeprom_addr, rd_value);

	/* Test to erase one word */
	eeprom_addr = 0x7FFD10;
	EEPROM_EraseWord(eeprom_addr);
	EEPROM_ReadWord(eeprom_addr, &rd_value);
	printf("[E] I-EEPROM address [0x%lX]: rd_value = 0x%X\n", eeprom_addr, rd_value);

	/* Test to erase one word */
	eeprom_addr = 0x7FFD12;
	EEPROM_EraseWord(eeprom_addr);
	EEPROM_ReadWord(eeprom_addr, &rd_value);
	printf("[E] I-EEPROM address [0x%lX]: rd_value = 0x%X\n", eeprom_addr, rd_value);

	/* Test to write multiple words */
	eeprom_addr = 0x7FFD20;

#ifdef __TEST_EEPROM_BLOCK_OPERATION__
	EEPROM_EraseBlock(eeprom_addr);
#else
	for(count = 0; count < 16; count++)
	{
		EEPROM_EraseWord(eeprom_addr);
		eeprom_addr += 2;
	}
#endif

	/* Check erased area */
	eeprom_addr = 0x7FFD20;

	for(count = 0; count < 16; count++)
	{
		EEPROM_ReadWord(eeprom_addr, &rd_value);
		printf("[R] I-EEPROM address [0x%lX]: rd_value = 0x%X\n", eeprom_addr, rd_value);
		eeprom_addr += 2;
	}

	/* Test to write multiple words (16 words) */
	eeprom_addr = 0x7FFD20;

#ifdef __TEST_EEPROM_BLOCK_OPERATION__
	EEPROM_WriteBlock(eeprom_addr, buffer);
#else
	for(count = 0; count < 16; count++)
	{
		EEPROM_WriteWord(eeprom_addr, buffer[count]);
		eeprom_addr += 2;
	}
#endif
	
	/* Test to read 16 words */
	eeprom_addr = 0x7FFD20;

	for(count = 0; count < 16; count++)
	{
		EEPROM_ReadWord(eeprom_addr, &rd_value);
		printf("[R] I-EEPROM address [0x%lX]: rd_value = 0x%X\n", eeprom_addr, rd_value);
		eeprom_addr += 2;
	}

	while(1)
	{
		printf("Hello\n");

		LATEbits.LATE0 = 0;
		__delay_us(500000);

		LATEbits.LATE0 = 1;
		__delay_us(500000);
	}

    while(1);
}



static void EEPROM_EraseWord(unsigned long addr)
{ 
	unsigned short upper_addr, lower_addr;

	upper_addr = (addr >> 16) & 0xFF;
	lower_addr = (addr & 0xFFFF);

	NVMADRU = upper_addr;
	NVMADR = lower_addr;
	NVMCON = 0x4044;     // 16bit word

	asm volatile(" DISI #5" );

	NVMKEY = 0x55;
	NVMKEY = 0xAA;
	NVMCONbits.WR = 1; 
	while (NVMCONbits.WR == 1); 

	IFS0bits.NVMIF =0;
	NVMCONbits.WREN = 0; 
}



static void EEPROM_EraseBlock(unsigned long addr)
{ 
	unsigned short upper_addr, lower_addr;

	upper_addr = (addr >> 16) & 0xFF;
	lower_addr = (addr & 0xFFFF);

	NVMADRU = upper_addr;
	NVMADR = lower_addr;
	NVMCON = 0x4045;     // 16bit word

	asm volatile(" DISI #5" );

	NVMKEY = 0x55;
	NVMKEY = 0xAA;
	NVMCONbits.WR = 1; 
	while (NVMCONbits.WR == 1); 

	IFS0bits.NVMIF =0;
	NVMCONbits.WREN = 0; 
}



static void EEPROM_WriteWord(unsigned long addr, unsigned short value)
{ 
	unsigned short upper_addr, lower_addr;

	upper_addr = (addr >> 16) & 0xFF;
	lower_addr = (addr & 0xFFFF);

	IFS0bits.NVMIF = 0;
	NVMCONbits.WREN = 0; 

	//NVMADRU = upper_addr;
	//NVMADR = lower_addr;

	TBLPAG = upper_addr;
	WREG0 = lower_addr;
	WREG2 = value;
	asm volatile("TBLWTL W2,[W0]"); 

	NVMCON = 0x4004; 
	asm volatile(" DISI #5" );

	NVMKEY = 0x55;
	NVMKEY = 0xAA;
	NVMCONbits.WR = 1; 
	while (NVMCONbits.WR == 1); 

	IFS0bits.NVMIF = 0;
	NVMCONbits.WREN = 0; 
}



static void EEPROM_WriteBlock(unsigned long addr, unsigned short *pbuf)
{ 
	unsigned short upper_addr, lower_addr;
	unsigned short value;
	unsigned char count;

	upper_addr = (addr >> 16) & 0xFF;
	lower_addr = (addr & 0xFFFF);

	IFS0bits.NVMIF = 0;
	NVMCONbits.WREN = 0; 

	TBLPAG = upper_addr;

	for(count = 0; count < 16; count++)
	{
		/* Indirect access is not applied in WREG */
		value = *pbuf;

		WREG0 = lower_addr;
		WREG2 = value;

		asm volatile("TBLWTL W2,[W0]");
	
		lower_addr += 2;
		pbuf++;
	}

	NVMCON = 0x4005;

	asm volatile(" DISI #5" );

	NVMKEY = 0x55;
	NVMKEY = 0xAA;
	NVMCONbits.WR = 1; 
	while (NVMCONbits.WR == 1); 

	IFS0bits.NVMIF = 0;
	NVMCONbits.WREN = 0; 
}



static void EEPROM_ReadWord(unsigned long addr, unsigned short *pdata) 
{ 
	TBLPAG = (addr >> 16) & 0xFF;
	WREG0 = (addr & 0xFFFF);

	asm volatile(" TBLRDL [W0],W4"); 
	*pdata = WREG4;
}



static void DRV_UART_Init(void)
{
	U1MODEbits.UARTEN = 1;		// 0 = UARTx is enabled.
	U1MODEbits.USIDL = 0;		// 0 = Continue module operation in Idle mode
	U1MODEbits.ALTIO = 0;
	U1MODEbits.WAKE = 0;		// 0 = No wake-up is enabled
	U1MODEbits.LPBACK = 0;		// 0 = Loopback mode is disabled
	U1MODEbits.ABAUD = 0;		// 0 = Baud rate measurement is disabled or completed

	U1MODEbits.PDSEL = 0;		// b00 = 00 = 8-bit data, no parity
	U1MODEbits.STSEL = 0;		// 0 = One Stop bit

	/*-------------------------------------------------------------------*/
	/* UxSTA: UARTx STATUS AND CONTROL REGISTER                          */
	/* [Description]                                                     */
	/* bit[15] UTXISEL: Transmission Interrupt Mode Selection bits */
	/* bit[11] UTXBRK: Transmit Break bit                                */
	/* bit[10] UTXEN: Transmit Enable bit                                */
	/* bit[9] UTXBF: Transmit Buffer Full Status bit (read-only). 1 = FULL */
	/* bit[7] URXISEL: Receive Interrupt Mode Selection bits            */
	/* bit[5] ADDEN: Address Character Detect bit (bit 8 of received data = 1) */
	/* [Property]                                                        */
	/*     Mandatory register to operate Rx                              */
	/*-------------------------------------------------------------------*/
	U1STAbits.UTXISEL = 0;
	U1STAbits.UTXBRK = 0;		// 0 = Sync Break transmission is disabled or completed
	U1STAbits.UTXEN = 1;		// 1 = Transmit is enabled, UxTX pin controlled by UARTx
	U1STAbits.URXISEL = 0;		// 
	U1STAbits.ADDEN = 0;		// 0 = Address Detect mode is disabled

	/*-------------------------------------------------------------------*/
	/* Baud rate Calculation                                             */
	/* [Equation]                                                        */
	/* i) BRGH = 0                                                       */
	/* BuadRate = FCY / (16 * (UxBRG + 1))                               */
	/* UxBRG = FCY / (16 * BuadRate) - 1 = ((FCY / 16) / BuadRate) - 1   */
	/*-------------------------------------------------------------------*/
	/* Set baud rate to 9600 bps  */
	U1BRG = ((FCY / 16) / BAUDRATE) - 1;

	/* Enable UART. */
	U1MODEbits.UARTEN = 1;		// 1 = UARTx is enabled; all UARTx pins are controlled by UARTx as defined by UEN

	/* Wait at least 105 microseconds (1/9600) before sending first char */
	__delay_us(1000000 / BAUDRATE);
}

/* End of File */
