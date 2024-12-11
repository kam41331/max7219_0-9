;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler 
; Version 4.4.0 #14620 (Linux)
;--------------------------------------------------------
	.module main
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _setup
	.globl _max7219_init
	.globl _max7219_send
	.globl _GPIO_WriteLow
	.globl _GPIO_WriteHigh
	.globl _GPIO_Init
	.globl _CLK_HSIPrescalerConfig
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; Stack segment in internal ram
;--------------------------------------------------------
	.area SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)

; default segment ordering for linker
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area CONST
	.area INITIALIZER
	.area CODE

;--------------------------------------------------------
; interrupt vector
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ; reset
	int _TRAP_IRQHandler ; trap
	int _TLI_IRQHandler ; int0
	int _AWU_IRQHandler ; int1
	int _CLK_IRQHandler ; int2
	int _EXTI_PORTA_IRQHandler ; int3
	int _EXTI_PORTB_IRQHandler ; int4
	int _EXTI_PORTC_IRQHandler ; int5
	int _EXTI_PORTD_IRQHandler ; int6
	int _EXTI_PORTE_IRQHandler ; int7
	int _CAN_RX_IRQHandler ; int8
	int _CAN_TX_IRQHandler ; int9
	int _SPI_IRQHandler ; int10
	int _TIM1_UPD_OVF_TRG_BRK_IRQHandler ; int11
	int _TIM1_CAP_COM_IRQHandler ; int12
	int _TIM2_UPD_OVF_BRK_IRQHandler ; int13
	int _TIM2_CAP_COM_IRQHandler ; int14
	int _TIM3_UPD_OVF_BRK_IRQHandler ; int15
	int _TIM3_CAP_COM_IRQHandler ; int16
	int _UART1_TX_IRQHandler ; int17
	int _UART1_RX_IRQHandler ; int18
	int _I2C_IRQHandler ; int19
	int _UART3_TX_IRQHandler ; int20
	int _UART3_RX_IRQHandler ; int21
	int _ADC2_IRQHandler ; int22
	int _TIM4_UPD_OVF_IRQHandler ; int23
	int _EEPROM_EEC_IRQHandler ; int24
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
	call	___sdcc_external_startup
	tnz	a
	jreq	__sdcc_init_data
	jp	__sdcc_program_startup
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	./src/main.c: 21: void max7219_send(uint8_t address, uint8_t data) {
; genLabel
;	-----------------------------------------
;	 function max7219_send
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 2 bytes.
_max7219_send:
	pushw	x
; genReceive
	ld	(0x01, sp), a
;	./src/main.c: 23: CS_LOW;
; genSend
	ld	a, #0x02
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteLow
;	./src/main.c: 25: mask = 0b10000000;
; genAssign
	ld	a, #0x80
	ld	(0x02, sp), a
;	./src/main.c: 26: CLK_LOW;
; genSend
	ld	a, #0x01
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteLow
;	./src/main.c: 28: while (mask) {
; genLabel
00104$:
; genIfx
	tnz	(0x02, sp)
	jrne	00157$
	jp	00106$
00157$:
;	./src/main.c: 29: if (mask & address) {
; genAnd
	ld	a, (0x02, sp)
	and	a, (0x01, sp)
; genIfx
	tnz	a
	jrne	00158$
	jp	00102$
00158$:
;	./src/main.c: 30: DATA_HIGH;
; genSend
	ld	a, #0x04
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteHigh
; genGoto
	jp	00103$
; genLabel
00102$:
;	./src/main.c: 32: DATA_LOW;
; genSend
	ld	a, #0x04
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteLow
; genLabel
00103$:
;	./src/main.c: 34: CLK_HIGH;
; genSend
	ld	a, #0x01
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteHigh
;	./src/main.c: 35: mask = mask >> 1;
; genRightShiftLiteral
	srl	(0x02, sp)
;	./src/main.c: 36: CLK_LOW;
; genSend
	ld	a, #0x01
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteLow
; genGoto
	jp	00104$
; genLabel
00106$:
;	./src/main.c: 39: mask = 0b10000000;
; genAssign
	ld	a, #0x80
	ld	(0x02, sp), a
;	./src/main.c: 41: while (mask) {
; genLabel
00110$:
; genIfx
	tnz	(0x02, sp)
	jrne	00159$
	jp	00112$
00159$:
;	./src/main.c: 42: if (mask & data) {
; genAnd
	ld	a, (0x02, sp)
	and	a, (0x05, sp)
; genIfx
	tnz	a
	jrne	00160$
	jp	00108$
00160$:
;	./src/main.c: 43: DATA_HIGH;
; genSend
	ld	a, #0x04
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteHigh
; genGoto
	jp	00109$
; genLabel
00108$:
;	./src/main.c: 45: DATA_LOW;
; genSend
	ld	a, #0x04
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteLow
; genLabel
00109$:
;	./src/main.c: 47: CLK_HIGH;
; genSend
	ld	a, #0x01
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteHigh
;	./src/main.c: 48: mask = mask >> 1;
; genRightShiftLiteral
	srl	(0x02, sp)
;	./src/main.c: 49: CLK_LOW;
; genSend
	ld	a, #0x01
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteLow
; genGoto
	jp	00110$
; genLabel
00112$:
;	./src/main.c: 52: CS_HIGH;
; genSend
	ld	a, #0x02
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_WriteHigh
; genLabel
00113$:
;	./src/main.c: 53: }
; genEndFunction
	popw	x
	popw	x
	pop	a
	jp	(x)
;	./src/main.c: 55: void max7219_init(void) { 
; genLabel
;	-----------------------------------------
;	 function max7219_init
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 0 bytes.
_max7219_init:
;	./src/main.c: 56: GPIO_Init(CS_GPIO, CS_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
; genIPush
	push	#0xc0
; genSend
	ld	a, #0x02
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_Init
;	./src/main.c: 57: GPIO_Init(CLK_GPIO, CLK_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
; genIPush
	push	#0xc0
; genSend
	ld	a, #0x01
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_Init
;	./src/main.c: 58: GPIO_Init(DATA_GPIO, DATA_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
; genIPush
	push	#0xc0
; genSend
	ld	a, #0x04
; genSend
	ldw	x, #0x5005
; genCall
	call	_GPIO_Init
;	./src/main.c: 61: max7219_send(DECODE_MODE, DECODE_ALL);        // dekodér
; genIPush
	push	#0xff
; genSend
	ld	a, #0x09
; genCall
	call	_max7219_send
;	./src/main.c: 62: max7219_send(SCAN_LIMIT, 0);                  // Kolik cifer zapneme(0=1cifra)
; genIPush
	push	#0x00
; genSend
	ld	a, #0x0b
; genCall
	call	_max7219_send
;	./src/main.c: 63: max7219_send(INTENSITY, 5);                   // Jas
; genIPush
	push	#0x05
; genSend
	ld	a, #0x0a
; genCall
	call	_max7219_send
;	./src/main.c: 64: max7219_send(DISPLAY_TEST, DISPLAY_TEST_OFF); // test displeje
; genIPush
	push	#0x00
; genSend
	ld	a, #0x0f
; genCall
	call	_max7219_send
;	./src/main.c: 65: max7219_send(SHUTDOWN, SHUTDOWN_ON);          // On/OFF
; genIPush
	push	#0x01
; genSend
	ld	a, #0x0c
; genCall
	call	_max7219_send
; genLabel
00101$:
;	./src/main.c: 66: }
; genEndFunction
	ret
;	./src/main.c: 68: void setup(void) {
; genLabel
;	-----------------------------------------
;	 function setup
;	-----------------------------------------
;	Register assignment is optimal.
;	Stack space usage: 0 bytes.
_setup:
;	./src/main.c: 69: CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // taktovat MCU na 16MHz
; genSend
	clr	a
; genCall
	call	_CLK_HSIPrescalerConfig
;	./src/main.c: 71: init_milis();
; genCall
	call	_init_milis
;	./src/main.c: 73: max7219_init();
; genCall
	jp	_max7219_init
; genLabel
00101$:
;	./src/main.c: 74: }
; genEndFunction
	ret
;	./src/main.c: 76: int main(void) {
; genLabel
;	-----------------------------------------
;	 function main
;	-----------------------------------------
;	Register assignment might be sub-optimal.
;	Stack space usage: 13 bytes.
_main:
	sub	sp, #13
;	./src/main.c: 78: setup();
; genCall
	call	_setup
;	./src/main.c: 80: max7219_send(DIGIT0, 0);
; genIPush
	push	#0x00
; genSend
	ld	a, #0x01
; genCall
	call	_max7219_send
;	./src/main.c: 83: uint32_t time = 0;
; genAssign
	clrw	x
	ldw	(0x03, sp), x
	ldw	(0x01, sp), x
;	./src/main.c: 84: uint8_t jednotky = 0;
; genAssign
	clr	(0x0d, sp)
;	./src/main.c: 87: while (1) {
; genLabel
00106$:
;	./src/main.c: 88: if ((milis() - time) > 500) {
; genCall
	call	_milis
	ldw	(0x0b, sp), x
; genCast
	ldw	y, (0x0b, sp)
	ldw	(0x07, sp), y
	ld	a, (0x07, sp)
	rlc	a
	clr	a
	sbc	a, #0x00
	ld	(0x06, sp), a
; genMinus
	ldw	x, (0x07, sp)
	subw	x, (0x03, sp)
	ldw	(0x0b, sp), x
	push	a
	ld	a, (0x07, sp)
	sbc	a, (0x03, sp)
	ld	(0x0b, sp), a
	pop	a
	sbc	a, (0x01, sp)
	ld	(0x09, sp), a
; genCmp
; genCmpTnz
	ldw	x, #0x01f4
	cpw	x, (0x0b, sp)
	clr	a
	sbc	a, (0x0a, sp)
	clr	a
	sbc	a, (0x09, sp)
	jrc	00131$
	jp	00106$
00131$:
; skipping generated iCode
;	./src/main.c: 89: time = milis();
; genCall
	call	_milis
; genCast
	ldw	(0x03, sp), x
	ld	a, (0x03, sp)
	rlc	a
	clr	a
	sbc	a, #0x00
	ld	(0x02, sp), a
	ld	(0x01, sp), a
;	./src/main.c: 90: max7219_send(DIGIT0, jednotky);
; genIPush
	ld	a, (0x0d, sp)
	push	a
; genSend
	ld	a, #0x01
; genCall
	call	_max7219_send
;	./src/main.c: 92: jednotky++;
; genPlus
	inc	(0x0d, sp)
;	./src/main.c: 93: if (jednotky > 9) {
; genCmp
; genCmpTnz
	ld	a, (0x0d, sp)
	cp	a, #0x09
	jrugt	00132$
	jp	00106$
00132$:
; skipping generated iCode
;	./src/main.c: 94: jednotky = 0;
; genAssign
	clr	(0x0d, sp)
; genGoto
	jp	00106$
; genLabel
00108$:
;	./src/main.c: 98: }
; genEndFunction
	addw	sp, #13
	ret
	.area CODE
	.area CONST
	.area INITIALIZER
	.area CABS (ABS)
