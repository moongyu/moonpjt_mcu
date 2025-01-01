/**********************************************************************
* © 2005 Microchip Technology Inc.
*
* FileName:        DataEEPROM.h
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC30Fxxxx
* Compiler:        MPLAB® C30 v3.01 or higher
* IDE:             MPLAB® IDE v7.52 or later
* Dev. Board Used: dsPICDEM 1.1 Development Board
* Hardware Dependencies: None
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip,s 
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP,S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author                 Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EB/HV                  11/02/05  First release of source file
* GM                     10/19/07  Revised to use new library functions
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
*
**********************************************************************/

#include <p30fxxxx.h>
#include <libpic30.h>

_FOSC(CSW_FSCM_OFF & XT_PLL8); /* Set up for XTxPLL8 mode since */
                                /* we will be tuning the FRC in this example */
_FWDT(WDT_OFF);                 /* Turn off the Watch-Dog Timer.  */
_FBORPOR(MCLR_EN & PWRT_OFF);   /* Enable MCLR reset pin and turn off the power-up timers. */
_FGS(CODE_PROT_OFF);            /* Disable Code Protection */

/*Declare constants/coefficients/calibration data to be stored in DataEEPROM*/
int _EEDATA(32) fooArrayInDataEE[] = {0,1,2,3,4,5,6,7,8,9,0xA,0xB,0xC,0xD,0xE,0xF};

/*Declare variables to be stored in RAM*/
int fooArray1inRAM[] = {0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF, 0xABCD, 0xBCDE,
                       0xCDEF, 0xDEFA, 0x0000, 0x1111, 0x2222, 0x3333, 0x4444, 0x5555};

int fooArray2inRAM[16];

int main(void);
void _ISR _DefaultInterrupt(void);


int main(void)
{
    _prog_addressT EE_addr;
    int temp = 0;

    /* initialize a variable to represent the Data EEPROM address */
    _init_prog_address(EE_addr, fooArrayInDataEE);
    
    /*Copy array "fooArrayinDataEE" from DataEEPROM to "fooArray2inRAM" in RAM*/
    _memcpy_p2d16(fooArray2inRAM, EE_addr, _EE_ROW);

    /*Erase a row in Data EEPROM at array "fooArrayinDataEE" */
    _erase_eedata(EE_addr, _EE_ROW);
    _wait_eedata();

    /*Write a row to Data EEPROM from array "fooArray1inRAM" */
    _write_eedata_row(EE_addr, fooArray1inRAM);
    _wait_eedata();

    while(1); /* Place a breakpoint here, run code and refresh the DataEEPROM window in MPLAB IDE */
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void)
{
    while(1) ClrWdt()
}

