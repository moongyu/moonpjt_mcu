# MPLAB IDE generated this makefile for use with GNU make.
# Project: cmg.mcp
# Date: Wed Apr 29 21:46:44 2020

AS = xc16-as.exe
CC = xc16-gcc.exe
LD = xc16-ld.exe
AR = xc16-ar.exe
HX = xc16-bin2hex.exe
RM = rm

cmg.hex : cmg.cof
	$(HX) "cmg.cof" -omf=coff

cmg.cof : main.o
	$(CC) -omf=coff -mcpu=30F3013 "main.o" "C:\Program Files (x86)\Microchip\xc16\v1.11\lib\dsPIC30F\libp30F3013-coff.a" -o"cmg.cof" -Wl,-L"C:\Program Files (x86)\Microchip\xc16\v1.11\lib",--script="C:\Program Files (x86)\Microchip\xc16\v1.11\support\dsPIC30F\gld\p30F3013.gld",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD2=1,--defsym=__ICD2RAM=1,-Map="cmg.map",--report-mem

main.o : define.h delay.h c:/program\ files\ (x86)/microchip/xc16/v1.11/support/dsPIC30F/h/p30F3013.h c:/program\ files\ (x86)/microchip/xc16/v1.11/support/dsPIC30F/h/p30Fxxxx.h c:/program\ files\ (x86)/microchip/xc16/v1.11/support/peripheral_30F_24H_33F/adc12.h c:/program\ files\ (x86)/microchip/xc16/v1.11/support/dsPIC30F/h/p30F3013.h c:/program\ files\ (x86)/microchip/xc16/v1.11/support/dsPIC30F/h/p30Fxxxx.h c:/program\ files\ (x86)/microchip/xc16/v1.11/support/peripheral_30F_24H_33F/timer.h c:/program\ files\ (x86)/microchip/xc16/v1.11/include/stdarg.h c:/program\ files\ (x86)/microchip/xc16/v1.11/include/stddef.h c:/program\ files\ (x86)/microchip/xc16/v1.11/include/stdio.h c:/program\ files\ (x86)/microchip/xc16/v1.11/support/dsPIC30F/h/p30f3013.h main.c
	$(CC) -omf=coff -mcpu=30F3013 -x c -c "main.c" -o"main.o" -I"C:\Program Files (x86)\Microchip\xc16\v1.11\inc" -D__DEBUG -D__MPLAB_DEBUGGER_ICD2=1 -g -Wall

clean : 
	$(RM) "main.o" "cmg.cof" "cmg.hex"

