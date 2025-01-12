;..............................................................................
;Subroutine: 
;           Input               : Binary value in W0
;                                 
;
;           Output              : Sequence of 4 ASCII characters 
;                                 Stored in 4 succesive memory locs.
;                                 pointed to by W1
;                                 [W1] contains least significant  character
;                                 [W1+8] contains most significant  character
;
;           Context Save/Restore: Subroutine saves and restores w2 - w7
;
;           Example             : w0 = "0x9E0F"
;                                 output:(lowest byte address) "0x39"
;                                        (higher byte address) "0x45"
;                                       (second to max address)"0x30"
;                                             (highest address)"0x46"
;..............................................................................
              
              .equ nibble, 4
              .equ nibble_mask, 0x000F
              .equ char_count, 4
              
              .section .text
              .global _Int2Ascii
_Int2Ascii:                 
              push.d w2             ;save off the scratch registers
              push.d w4
              push.d w6

              mov #0x30, w6         ;w6 used to store a constant to create ASCII
              mov #char_count, w5   ;w5 holds the character count
              mov #nibble_mask, w2  ;w2 holds a value to mask a nibble

;perform bin 2 ascii conversion one character at a time
next_char:        
              and w0, w2, w3
              lsr w0, #nibble, w0
              cp w3, #0x9
              bra le, betn_0_9
              add w3, #7, w3
betn_0_9:
              add w3, w6, w4
              push w4
              
              dec w5, w5
              bra nz, next_char

	      pop w0
	      mov w0, [w1++]

	      pop w0
	      mov w0, [w1++]

	      pop w0
	      mov w0, [w1++]

	      pop w0
	      mov w0, [w1++]

	      sub #8, w1	
              
              pop.d w6
              pop.d w4
              pop.d w2
        
              return
   
        
