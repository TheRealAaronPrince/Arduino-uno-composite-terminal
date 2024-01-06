// pixelsEtc.S

#include "UnoVideoSerial.h"

#define Pin5 &PORTD, _BV(5)
#define Pin9 &PORTB, _BV(1)

#define PIN_SUPPRESS Pin5
#define PIN_SYNC Pin9

.equ   DDRD, 0x0A
.equ  PORTD, 0x0B
.equ UCSR0B, 0xC1
.equ   UDR0, 0xC6
.equ   SREG, 0x3F

#define UNSUPPRESS CBI  DDRD, 5
#define SUPPRESS   SBI  DDRD, 5

 .global pixelsEtc ; (*pScreenRam, *fontSlice, tcnt, minTCNT)
 ;                       R25  R24     R23 R22  R21 R20 R19 R18   

pixelsEtc:
	; interrupt may jitter because of instructions taking 1, 2, 3 clock cycles
	; (and maybe more if there are other interrupts)
	; this hack synchronizes the CPU with the tcnt value
	; only checking the low byte here because interrupt should always occur earlier than 240 counts (15 us)
	inc R18
	CP R20, R18 ; compare tcnt with minTCNT+1
	BRLO jitterFix1 ; extra cycle if tcnt was lower
jitterFix1:
	inc R18
	CP R20, R18 ; compare tcnt with minTCNT+2
	BRLO jitterFix2
jitterFix2:
	INC R18
	CP R20, R18 ; compare tcnt with minTCNT+3
	BRLO jitterFix3
jitterFix3:
	INC R18
	CP R20, R18 ; compare tcnt with minTCNT+4
	BRLO jitterFix4
jitterFix4:
	MOVW R26, R24 ; X = pScreenRam
	; when USART is awoken, it insists on clocking out at least one high pixel, no matter what we do.
	; so to suppress that, we use an IO pin, active low, to ground the video signal after the resistor.
	; once the USART has got going, we switch the IO pin back to input mode so it stops grounding the video signal.
	; interestingly, when the USART is put to sleep at the end of the row, then its output goes low.
	; this is true even when the last pixel sent was high.
	LDI R18, 0xFF ; for flipping bits RVS
	SUPPRESS
	LD R24, X+ ; get first character from screen ram
	CPI R24, 0x80 ; test RVS bit set
	IN R19, SREG ; save flags
	ANDI R24, 0x7F ; mask out RVS bit
	MOVW R30, R22 ; Z = fontSlice
	ADD R30, R24 ; add character offset to Z. fontSlice is 128-aligned, so no need to add the high bytes
	LPM R20, Z ; fetch first 8 pixels
	OUT SREG, R19 ; restore flags
	BRLO notRvs
	EOR R20, R18 ; flip RVS
notRvs:
	STS UDR0, R20 ; send them to the USART
	LDI R24, 0x08
	STS UCSR0B, R24 ; enable USART transmit
	UNSUPPRESS
	LDI R25, BYTES_PER_RASTER-1 ; loop counter
loop: ;                                          cycles
	LD R24, X+ ; get character from screen ram // +2 =  2
	CPI R24, 0x80 ; test RVS bit set           // +1 =  3
	BRLO normal ;                              // +1 =  4
	ANDI R24, 0x7F ; mask out RVS bit          // +1 =  5
	MOVW R30, R22 ; Z = fontSlice              // +1 =  6
	ADD R30, R24                               // +1 =  7
	LPM R20, Z ; fetch 8 pixels                // +3 = 10
	EOR R20, R18 ; flip RVS                    // +1 = 11
	STS UDR0, R20 ; send them to the USART     // +2 = 13
	DEC R25                                    // +1 = 14
	BRNE loop                                  // +2 = 16
	RJMP done
normal:                                      // +2 =  5
	MOVW R30, R22 ; Z = fontSlice              // +1 =  6
	ADD R30, R24                               // +1 =  7
	LPM R20, Z ; fetch 8 pixels                // +3 = 10
	STS UDR0, R20 ; send them to the USART     // +2 = 12
	NOP                                        // +1 = 13
	DEC R25                                    // +1 = 14
	BRNE loop                                  // +2 = 16
done:
	STS UCSR0B, R1 ; disable USART transmit
	RET
