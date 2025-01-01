/*********************************************************************
*                                                                    *
*                       Software License Agreement                   *
*                                                                    *
*   The software supplied herewith by Microchip Technology           *
*   Incorporated (the "Company") for its dsPIC controller            *
*   is intended and supplied to you, the Company's customer,         *
*   for use solely and exclusively on Microchip dsPIC                *
*   products. The software is owned by the Company and/or its        *
*   supplier, and is protected under applicable copyright laws. All  *
*   rights are reserved. Any use in violation of the foregoing       *
*   restrictions may subject the user to criminal sanctions under    *
*   applicable laws, as well as to civil liability for the breach of *
*   the terms and conditions of this license.                        *
*                                                                    *
*   THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO           *
*   WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,    *
*   BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND    *
*   FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE     *
*   COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,  *
*   INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.  *
*                                                                    *
*********************************************************************/
/********************************************************************
*
*  Filename:   LAB2.H 
*  Purpose:    This file is the header file for LAB2.
*
*********************************************************************/

#ifndef LAB2_H
#define LAB2_H


/* System and UART Defines */
#define BAUD_RATE       96       /* in hundreds */
#define CLOCK           73728    /* in hundreds */
#define PLL_MULTIPLY    8
#define FCY             (CLOCK*PLL_MULTIPLY)/4
#define BRG             (FCY/(16*BAUD_RATE))-1
#define TIMER_PRESCALE  256
#define TIMER_MSEC      (.1*FCY/TIMER_PRESCALE)  /* since FCY in hundreds! */

/* programming defines */
#define NUM_EE_WORDS_PER_ROW  16
#define DATA_EEPROM_TBLPAG    0x7F     
#define DATA_EEPROM_START     0xFD00   /* lower 16-bits of address for dsPIC30F4011 1K */
#define WORD_MODE             0x1
#define ROW_MODE              0x2

/* NVMCON values for Data EEPROM operations */
#define EE_PRG_ROW     0x4005 /* program one data EEPROM row */
#define EE_PRG_WORD    0x4004 /* program one data EEPROM word */
#define EE_ERS_ROW     0x4045 /* erase one data EEPROM row */
#define EE_ERS_WORD    0x4044 /* erase one data EEPROM word */
#define EE_ERS_ALL     0x4046 /* erase all data EEPROM */

/* logic defines */
#define FALSE 0x0
#define TRUE 0x1

/* programming macros */

#define UNLOCK_NVM_AND_PROGRAM \
    asm ("mov     #0x55, w7"); \
    asm ("mov     w7, NVMKEY"); \
    asm ("mov     #0xaa, w7"); \
    asm ("mov     w7, NVMKEY"); \
    asm ("bset    NVMCON, #0xF"); \
    asm ("nop"); \
    asm ("nop");

#define LOAD_DATA_WRITE_LATCHES \
    asm ("repeat   #15             ; perform 16 writes"); \
    asm ("tblwtl.w [w6++], [w7++]  ; write one 16-bit word at a time");          

#endif    
