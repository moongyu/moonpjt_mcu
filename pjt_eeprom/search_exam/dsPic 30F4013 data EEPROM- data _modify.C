#include "p30fxxxx.h"
//-------------------------------------------------------------------------------
// p30f2010 의 ee data write 부분을 포인터로 하면 잘됩니다.
 //아래부분이 잘 안됩니다

int _EEDATA(32) InDataEE[] = {2,11,22,33,44,55,66,77,88,99,};

main()
{

   int debug;

    erase_word_eeprom( &InDataEE[0]);                    // 동작 잘 됩니다.
    write_word_eeprom(&InDataEE[0],1234);               // 동작이 잘 됩니다. 
    debug = read_eeprom( &InDataEE[0] );                 // 단독으로 하면 동작 잘됨니다.

	while(1);
}
 int read_eeprom(int *addr) 

{ 
   TBLPAG = 0x7F; 
    WREG0 = (unsigned int) addr; 
   asm volatile(" TBLRDL [W0],W4"); 
    return WREG4; 
}


void erase_word_eeprom( unsigned int *addr) 
{ 
    NVMADRU = 0x7F; 
    NVMADR = (unsigned int)addr; 
    NVMCON = 0x4044;     // 16bit word

    asm volatile(" DISI #5" );
    NVMKEY = 0x55;
    NVMKEY = 0xAA;
    NVMCONbits.WR = 1; 
    while (NVMCONbits.WR == 1); 
    IFS0bits.NVMIF =0;
    NVMCONbits.WREN = 0; 
}

void write_word_eeprom(unsigned int *addr, unsigned valude) 
{ 
 
 unsigned int * Temp;
 unsigned int Temp2;
 
 Temp = addr;
 Temp2 = valude; 
  
 NVMADRU = 0x7F; 
 NVMADR = (unsigned int)addr; 
 NVMCON = 0x4044;     // 16bit word

 asm volatile(" DISI #5" );
 NVMKEY = 0x55;
 NVMKEY = 0xAA;
 NVMCONbits.WR = 1; 
 while (NVMCONbits.WR == 1); 
 IFS0bits.NVMIF =0;
 NVMCONbits.WREN = 0; 

 TBLPAG = 0x7f;
 WREG3 = (unsigned int) Temp; 
 WREG4 = Temp2;
 asm volatile(" TBLWTL W4,[W3]"); 
 NVMCON = 0x4004; 
 
 asm volatile(" DISI #7" );
 NVMKEY = 0x55;
 NVMKEY = 0xAA;
 NVMCONbits.WR = 1; 

 while (NVMCONbits.WR == 1); 
 
 IFS0bits.NVMIF =0;
 NVMCONbits.WREN = 0; 
}

//--------------------------------------------------------------------------------



//BUILD FAILED 발생합니다 다시한번 검토를 바랍니다 |
 
//--------------------------------------------------------------------------------
// p30f2010 사용하여  컴파일 Error가 발생하여 안되는 군요. 고수님 다시한번 부탁드립니다.
 // BUILD FAILED 부분은 아래에 있습니다

//-----------------------------------
//--------------------------------
//-----------------------------------------------------------
// 0번지에 190을 기록함

// offset = 0;
// data =190;
//write_word_eeprom(offset,data); 

//-------------------------------------------------------
// 0 번지를 읽음 --> 당연히 Temp는 190 되어야 함.

//Temp   = read_eeprom( 0 );   // 0 번지를 읽음

//--------------------------------------------------------------
// 13번지에 2555을 기록함

// offset = 13;
// data =2555;
//write_word_eeprom(offset,data); 

//-----------------------------------------------------------------------------
/*

// p30f2010 의 ee data write 부분이 안되는 군요. 부탁드립니다.
//addr주면 잘 안됩니다 

#include <p30fxxxx.h> 

int _EEDATA(32) InDataEE[] = {44};
#define EEDATA_BASE_ADDR 0x7FFC00
 #define offset = 13
 #define data =190
main()

{

              
  	erase_word_eeprom(offset);
 	write_word_eeprom(offset,data);
 	read_eeprom (offset)

while(1)
}

  
int read_eeprom( int offset) 
{ 
 TBLPAG = 0x7F; 
 WREG3 = (unsigned int)(offset + EEDATA_BASE_ADDR); 
 asm volatile(" TBLRDL [W3],W4"); 
 return WREG4; 
}

void erase_word_eeprom( unsigned int offset) 
{ 
 NVMADRU = 0x7F; 
 NVMADR = (unsigned int)( offset + EEDATA_BASE_ADDR);
 NVMCON = 0x4044;     // 

 asm volatile(" DISI #5" );
 NVMKEY = 0x55;
 NVMKEY = 0xAA;
 NVMCONbits.WR = 1; 
 while (NVMCONbits.WR == 1); 
 IFS0bits.NVMIF =0;
 NVMCONbits.WREN = 0; 
}

//---------------------------------------------------------
// 1. erase address 
// 2. write data 
// 3. 07f fc00  --> start address
//---------------------------------------------------------

write_word_eeprom(unsigned int offset, int data) 
{ 
 unsigned int Temp;
 int Temp2;
 
 Temp = offset*2;
 Temp2 = data; 
  
 NVMADRU = 0x7F; 
 NVMADR = (unsigned int)( Temp + EEDATA_BASE_ADDR); 
 NVMCON = 0x4044;     //

 asm volatile(" DISI #5" );
 NVMKEY = 0x55;
 NVMKEY = 0xAA;
 NVMCONbits.WR = 1; 
 while (NVMCONbits.WR == 1); 
 IFS0bits.NVMIF =0;
 NVMCONbits.WREN = 0; 

 TBLPAG = 0x7f;
 WREG1 = (unsigned int)( Temp + EEDATA_BASE_ADDR ); 
 WREG4 = Temp2;
 asm volatile(" TBLWTL W4,[W1]"); 
 NVMCON = 0x4004; 
 
 asm volatile(" DISI #5" );
 NVMKEY = 0x55;
 NVMKEY = 0xAA;
 NVMCONbits.WR = 1; 

 while (NVMCONbits.WR == 1); 
 
 IFS0bits.NVMIF =0;
 NVMCONbits.WREN = 0; 
}

*/
