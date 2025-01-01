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
*  Filename:   EEPROM.C
*  Purpose:    This file contains Data EEPROM erase/program/read
*              functions.
*
*********************************************************************/

/* header files */
#include "eeprom.h"          /* defines eeprom macros */
#include <uart.h>          /* uart peripheral library header file */
#include <p30f4011.h>      /* processor defines */

/* define the Data EEPROM constants */
/* these will be re-programmed during program execution */
const unsigned int _EEDATA(2) word_buffer[256];
const unsigned int _EEDATA(2) row_buffer[256];

/* globals - NOTE: these are located in RAM */
char sCRLF[] = "\r\n";
char sTab[] = "\t";
char sWordRead[]  = "\r\nReading Word Array:\r\n";
char sRowRead[] = "\r\nReading Row Array:\r\n";
char sEmpty[] = "\tEMPTY!\r\n";

/* externs */
extern unsigned new_row[];
extern unsigned row_offset_address;
extern unsigned word_offset_address;

/* funtion prototypes */
void EraseEE(void);
void ProgramEEWord (unsigned word);
void ProgramEERow (void);
void ReadEE(void);


/* source code */
/*************************************************************************
*                                                                         
*  Function:  EraseEE()
*  Arguments: none
*  Returns:   void
* 
*  This function erases the entire EEPROM using the erase segment
*  operation.
*
**************************************************************************/
void EraseEE (void)   {
    
   /* set the NVMCON for erasing all Data EEPROM */
   NVMCON = EE_ERS_ALL;
    
   /* set the NVMADRU/NVMADR */
   NVMADRU = 0x7F;   // TBPLAG for EEPROM
   NVMADR = 0xFC00;  // any EEPROM address okay
   
   /* start the erase */
   UNLOCK_NVM_AND_PROGRAM;

   /* wait until the erase completes */
   while (!NVMCONbits.WR);
        
   }  /* end EraseEE() */


/*************************************************************************
*                                                                         
*  Function:  ProgramEEWord()
*  Arguments: word, the data value to program
*  Returns:   void
*
*  This function programs the specified word to the Data EEPROM.
*  The target location is the present "word_offset_address".
*
**************************************************************************/
void ProgramEEWord (unsigned word)   {

    /* set the NVMCON for this programming */
    NVMCON = EE_PRG_WORD;

    /* set the TBLPAG register... fixed for all Data EEPROM */
    TBLPAG = DATA_EEPROM_TBLPAG;

    /* set the source register W6 to the desired word to write */
    WREG6 = word;

    /* set the destination pointer W7 for programming */
    WREG7 = (unsigned) word_offset_address;
   
    /* load the one write latch! */
    asm ("tblwtl.w w6, [w7]  ; write the 16-bit word");
   
    /* now do the programming operation */
    UNLOCK_NVM_AND_PROGRAM;
                     
    /* wait until the programming completes */
    while (NVMCONbits.WR);

    /* point to the next location of Data EEPROM */
    word_offset_address += 2;
   
}  /* end ProgramEEWord() */


/*************************************************************************
*                                                                         
*  Function:  ProgramEERow()
*  Arguments: none
*  Returns:   void
*
*  This function programs one row of the Data EEPROM.
*  The data programmed is from the new_row[] array.
*  The target row in the Data EEPROM is the present "row_offset_address".
*
**************************************************************************/
void ProgramEERow (void)   {

    /* set the NVMCON for this programming */
    NVMCON = EE_PRG_ROW;

    /* set the TBLPAG register... fixed for all Data EEPROM */
    TBLPAG = DATA_EEPROM_TBLPAG;

    /* set the source register W6 to the desired word to write */
    WREG6 = (unsigned) &new_row;

    /* set the destination pointer W7 for programming */
    WREG7 = (unsigned) row_offset_address;

    /* load the 16 write latches */
    LOAD_DATA_WRITE_LATCHES;   
   
    /* now do the programming operation */
    UNLOCK_NVM_AND_PROGRAM;
                     
    /* wait until the erase completes, then verify WR is still clear */
    while (NVMCONbits.WR);
    
    /* set the row_offset_address to point to the next row */
    row_offset_address += 2*NUM_EE_WORDS_PER_ROW;
   
}  /* end ProgramEERow() */


/*************************************************************************
*                                                                         
*  Function:  ReadEE()
*  Arguments: none
*  Returns:   void
*
*  This function reads the two halves of the Data EEPROM and outputs
*  the data to the UART.  Erased locations are not written to the UART. 
*
**************************************************************************/
void ReadEE (void)
{
   int p, page_shadow;
   unsigned i, u;

   /* save the PSVPAG */   
   page_shadow = PSVPAG;   

   /* set the PSVPAG for Data EEPROM */
   CORCONbits.PSV = 1;
   PSVPAG = 0xFF;

   /* print a message */
   putsUART1 ((int*) sWordRead);

   /* read the first half - which was programmed by word */
   u = 0;
   
   if ( (word_buffer[u] == 0xFFFF) )
   {
      /* Data EEPROM half is empty! */
      putsUART1 ((int*) sEmpty);      
   }
   else
   {   
      /* print a tab */
      putsUART1 ((int*) sTab);

      while (word_buffer[u] != 0xFFFF) 
      {
         while (BusyUART1());
         WriteUART1 (word_buffer[u]);
         u += 1;
         
         /* after 16 words, print a CR-LF */
         if (!(u%16)) {
            /* print a CR-LF and tab */
            putsUART1 ((int*) sCRLF);
            putsUART1 ((int*) sTab);
            }
      }  /* end while () */
   }  /* end else */
   
   /* read the last half - which was programmed by row */
   /* print a message */
   putsUART1 ((int*) sRowRead);
   u = 0;
   if ( (row_buffer[u] == 0xFFFF) )
   {
      /* Data EEPROM half is empty! */
      putsUART1 ((int*) sEmpty);      
   }
   else
   {   
      while (row_buffer[u] != 0xFFFF)
      {
         /* print a tab */
         putsUART1 ((int*) sTab);
   
         /* we have a programmed row - dump it to UART */
         for (i=0; i<16; i++)
         {
            while (BusyUART1());
            WriteUART1 (row_buffer[u+i]);
        
         }  /* end for (i) */
      
         /* add a CR-LF */
         putsUART1 ((int*) sCRLF);
      
         /* bump the row */
         u += 16;
            
      }  /* end while () */
   }  /* end else */

   /* restore the PSVPAG */
   PSVPAG = page_shadow;
   
}  /* end ReadEE() */

