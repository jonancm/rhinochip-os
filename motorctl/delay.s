/*
; © 2005  Microchip Technology Inc.
;
; Microchip Technology Inc. (“Microchip”) licenses this software to you
; solely for use with Microchip dsPIC® digital signal controller
; products.  The software is owned by Microchip and is protected under
; applicable copyright laws.  All rights reserved.
;
; SOFTWARE IS PROVIDED “AS IS.”  MICROCHIP EXPRESSLY DISCLAIMS ANY
; WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
; PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
; BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
; DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
; PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
; BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
; ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
;
;
;FILE DESCRIPTION:
; This is an MPLAB ASM30 Assembler source file for dsPIC30F processors.
; This file contains software delay routines.
;
;REVISION HISTORY:
;  $Log: Delay.s,v $
;  Revision 1.1.1.1  2005/06/06 09:16:46  VasukiH
;  First release of software
;
;
*/

;Symbol/Literal/Immediate Operand Definitions:
;".equ" directives are similar to "#define" pre-processor directives in C
.equ    NANOSEC125,  2          ;125ns approx = 2*TCY at 14.7 MIPS
.equ    MICROSEC, 8*NANOSEC125  ;8*125ns =1us
.equ    MILLISEC, 1000*MICROSEC ;1000*1us = 1000*8*125ns = 1ms

;Global Declarations for Functions and variables:
.global _Delay5us
.global _Delay5ms

;Code Sections:
;Code sections are named ".text"
.section .text


_Delay5us:
;Function Prototype for C:
; void Delay5us(int Count)
; Note: "_" prefixed to the function name is not used when calling the
;       function from a C file.
; Function Execution Time when Count=1 is 5 microseconds at 14.74 MIPS
;
        push    w1              ;Store w1 on to stack
        mov     #MICROSEC, w1   ;w1 = MICROSEC
        dec     w1, w1          ;w1 = w1 - 1
        bra     nz, $-2         ;If w1 != 0 then Branch to previous instruction
        dec     w0, w0          ;else w0 = w0 - 1
        bra     nz, $-8         ;If w0 != 0 then Branch back 4 instructions
        pop     w1              ;else restore w1 from stack
        return                  ;Return to calling routine

_Delay5ms:
;Function Prototype for C:
; void Delay5ms(int Count)
; Note: "_" prefixed to the function name is not used when calling the
;       function from a C file.
; Function Execution Time when Count=1 is 5 milliseconds at 14.74 MIPS
        push    w1              ;Store w1 on to stack
        mov     #MILLISEC, w1   ;w1 = MILLISEC
        dec     w1, w1          ;w1 = w1 - 1
        bra     nz, $-2         ;If w1 != 0 then Branch to previous instruction
        dec     w0, w0          ;else w0 = w0 - 1
        bra     nz, $-8         ;If w0 != 0 then Branch back 4 instructions
        pop     w1              ;else restore w1 from stack
        return                  ;Return to calling routine


.end                            ;End of File




