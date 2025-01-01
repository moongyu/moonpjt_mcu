/**********************************************************************
* ?2005 Microchip Technology Inc.
*
* FileName:        DataEEPROM.h
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC30Fxxxx
* Compiler:        MPLAB?C30 v1.32.00 or higher
* IDE:             MPLAB?IDE v7.20.01 or later
* Dev. Board Used: dsPICDEM 1.1 Development Board
* Hardware Dependencies: None
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Inc. (“Microchip? licenses this software to you
* solely for use with Microchip dsPIC?digital signal controller
* products. The software is owned by Microchip and is protected under
* applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED “AS IS.? MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
* BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
* DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
* PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
* BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author                 Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EB/HV                  11/02/05  First release of source file
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
*
**********************************************************************/

#include "p30fxxxx.h"
#include "DataEEPROM.h"

_FOSC(CSW_FSCM_OFF & XT_PLL8); /* Set up for XTxPLL8 mode since */
                                /* we will be tuning the FRC in this example */
_FWDT(WDT_OFF);                 /* Turn off the Watch-Dog Timer.  */
_FBORPOR(MCLR_EN & PWRT_OFF);   /* Enable MCLR reset pin and turn off the power-up timers. */
_FGS(CODE_PROT_OFF);            /* Disable Code Protection */

/*Declare constants/coefficients/calibration data to be stored in DataEEPROM*/
int _EEDATA(32) fooArrayInDataEE[] = {0,1,2,3,4,5,6,7,8,9,0xA,0xB,0xC,0xD,0xE,0xF};

/*Declare variables to be stored in RAM*/
//int fooArray1inRAM[] = {0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF, 0xABCD, 0xBCDE,
//                       0xCDEF, 0xDEFA, 0x0000, 0x1111, 0x2222, 0x3333, 0x4444, 0x5555};

int fooArray1inRAM[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

int fooArray2inRAM[16];


int main(void)
{
	int temp = 0;

	int i;

	/*Read array named "fooArrayinDataEE" from DataEEPROM and place the result into*/
	/*array in RAM named, "fooArray2inRAM" */
	//temp = ReadEE(__builtin_tblpage(&fooArrayInDataEE[0]),__builtin_tbloffset(&fooArrayInDataEE[0]),&fooArray2inRAM[0], ROW);

	/*Erase 16 words (1 row in dsPIC30F DataEEPROM) in Data EEPROM from array named "fooArrayinDataEE" */
	temp = EraseEE(0x007f, 0xf000, ROW);
temp = EraseEE(0x007f, 0xf010, ROW);
temp = EraseEE(0x007f, 0xf020, ROW);
temp = EraseEE(0x007f, 0xf030, ROW);


	/*Write 16 words (1 row in dsPIC30F DataEEPROM) to Data EEPROM from array named "fooArray1inRAM" */
	/*to array named "fooArrayinDataEE" */
	//temp = WriteEE(&fooArray1inRAM[0],__builtin_tblpage(&fooArrayInDataEE[0]),__builtin_tbloffset(&fooArrayInDataEE[0]), ROW);


	//temp = ReadEE(__builtin_tblpage(&fooArrayInDataEE[0]),__builtin_tbloffset(&fooArrayInDataEE[0]),&fooArray2inRAM[0], ROW);
//	
	for(i=0;i<64;i++)
	{	
		temp = WriteEE(&i,0x007F, 0xF000 +i, WORD);
	}


	while(1); /* Place a breakpoint here, run code and refresh the DataEEPROM window in MPLAB IDE */
}



