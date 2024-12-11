                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler 
                                      3 ; Version 4.4.0 #14620 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module main
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _main
                                     12 	.globl _setup
                                     13 	.globl _max7219_init
                                     14 	.globl _max7219_send
                                     15 	.globl _GPIO_WriteLow
                                     16 	.globl _GPIO_WriteHigh
                                     17 	.globl _GPIO_Init
                                     18 	.globl _CLK_HSIPrescalerConfig
                                     19 ;--------------------------------------------------------
                                     20 ; ram data
                                     21 ;--------------------------------------------------------
                                     22 	.area DATA
                                     23 ;--------------------------------------------------------
                                     24 ; ram data
                                     25 ;--------------------------------------------------------
                                     26 	.area INITIALIZED
                                     27 ;--------------------------------------------------------
                                     28 ; Stack segment in internal ram
                                     29 ;--------------------------------------------------------
                                     30 	.area SSEG
      0085F2                         31 __start__stack:
      0085F2                         32 	.ds	1
                                     33 
                                     34 ;--------------------------------------------------------
                                     35 ; absolute external ram data
                                     36 ;--------------------------------------------------------
                                     37 	.area DABS (ABS)
                                     38 
                                     39 ; default segment ordering for linker
                                     40 	.area HOME
                                     41 	.area GSINIT
                                     42 	.area GSFINAL
                                     43 	.area CONST
                                     44 	.area INITIALIZER
                                     45 	.area CODE
                                     46 
                                     47 ;--------------------------------------------------------
                                     48 ; interrupt vector
                                     49 ;--------------------------------------------------------
                                     50 	.area HOME
      008000                         51 __interrupt_vect:
      008000 82 00 80 6F             52 	int s_GSINIT ; reset
      008004 82 00 82 F8             53 	int _TRAP_IRQHandler ; trap
      008008 82 00 82 F9             54 	int _TLI_IRQHandler ; int0
      00800C 82 00 82 FA             55 	int _AWU_IRQHandler ; int1
      008010 82 00 82 FB             56 	int _CLK_IRQHandler ; int2
      008014 82 00 82 FC             57 	int _EXTI_PORTA_IRQHandler ; int3
      008018 82 00 82 FD             58 	int _EXTI_PORTB_IRQHandler ; int4
      00801C 82 00 82 FE             59 	int _EXTI_PORTC_IRQHandler ; int5
      008020 82 00 82 FF             60 	int _EXTI_PORTD_IRQHandler ; int6
      008024 82 00 83 00             61 	int _EXTI_PORTE_IRQHandler ; int7
      008028 82 00 83 01             62 	int _CAN_RX_IRQHandler ; int8
      00802C 82 00 83 02             63 	int _CAN_TX_IRQHandler ; int9
      008030 82 00 83 03             64 	int _SPI_IRQHandler ; int10
      008034 82 00 83 04             65 	int _TIM1_UPD_OVF_TRG_BRK_IRQHandler ; int11
      008038 82 00 83 05             66 	int _TIM1_CAP_COM_IRQHandler ; int12
      00803C 82 00 83 06             67 	int _TIM2_UPD_OVF_BRK_IRQHandler ; int13
      008040 82 00 83 07             68 	int _TIM2_CAP_COM_IRQHandler ; int14
      008044 82 00 83 08             69 	int _TIM3_UPD_OVF_BRK_IRQHandler ; int15
      008048 82 00 83 09             70 	int _TIM3_CAP_COM_IRQHandler ; int16
      00804C 82 00 83 0A             71 	int _UART1_TX_IRQHandler ; int17
      008050 82 00 83 0B             72 	int _UART1_RX_IRQHandler ; int18
      008054 82 00 83 0C             73 	int _I2C_IRQHandler ; int19
      008058 82 00 83 0D             74 	int _UART3_TX_IRQHandler ; int20
      00805C 82 00 83 0E             75 	int _UART3_RX_IRQHandler ; int21
      008060 82 00 83 0F             76 	int _ADC2_IRQHandler ; int22
      008064 82 00 83 10             77 	int _TIM4_UPD_OVF_IRQHandler ; int23
      008068 82 00 83 2A             78 	int _EEPROM_EEC_IRQHandler ; int24
                                     79 ;--------------------------------------------------------
                                     80 ; global & static initialisations
                                     81 ;--------------------------------------------------------
                                     82 	.area HOME
                                     83 	.area GSINIT
                                     84 	.area GSFINAL
                                     85 	.area GSINIT
      00806F CD 84 3C         [ 4]   86 	call	___sdcc_external_startup
      008072 4D               [ 1]   87 	tnz	a
      008073 27 03            [ 1]   88 	jreq	__sdcc_init_data
      008075 CC 80 6C         [ 2]   89 	jp	__sdcc_program_startup
      008078                         90 __sdcc_init_data:
                                     91 ; stm8_genXINIT() start
      008078 AE 00 00         [ 2]   92 	ldw x, #l_DATA
      00807B 27 07            [ 1]   93 	jreq	00002$
      00807D                         94 00001$:
      00807D 72 4F 00 00      [ 1]   95 	clr (s_DATA - 1, x)
      008081 5A               [ 2]   96 	decw x
      008082 26 F9            [ 1]   97 	jrne	00001$
      008084                         98 00002$:
      008084 AE 00 04         [ 2]   99 	ldw	x, #l_INITIALIZER
      008087 27 09            [ 1]  100 	jreq	00004$
      008089                        101 00003$:
      008089 D6 80 94         [ 1]  102 	ld	a, (s_INITIALIZER - 1, x)
      00808C D7 00 00         [ 1]  103 	ld	(s_INITIALIZED - 1, x), a
      00808F 5A               [ 2]  104 	decw	x
      008090 26 F7            [ 1]  105 	jrne	00003$
      008092                        106 00004$:
                                    107 ; stm8_genXINIT() end
                                    108 	.area GSFINAL
      008092 CC 80 6C         [ 2]  109 	jp	__sdcc_program_startup
                                    110 ;--------------------------------------------------------
                                    111 ; Home
                                    112 ;--------------------------------------------------------
                                    113 	.area HOME
                                    114 	.area HOME
      00806C                        115 __sdcc_program_startup:
      00806C CC 82 46         [ 2]  116 	jp	_main
                                    117 ;	return from main will return to caller
                                    118 ;--------------------------------------------------------
                                    119 ; code
                                    120 ;--------------------------------------------------------
                                    121 	.area CODE
                                    122 ;	./src/main.c: 21: void max7219_send(uint8_t address, uint8_t data) {
                                    123 ; genLabel
                                    124 ;	-----------------------------------------
                                    125 ;	 function max7219_send
                                    126 ;	-----------------------------------------
                                    127 ;	Register assignment is optimal.
                                    128 ;	Stack space usage: 2 bytes.
      008160                        129 _max7219_send:
      008160 89               [ 2]  130 	pushw	x
                                    131 ; genReceive
      008161 6B 01            [ 1]  132 	ld	(0x01, sp), a
                                    133 ;	./src/main.c: 23: CS_LOW;
                                    134 ; genSend
      008163 A6 02            [ 1]  135 	ld	a, #0x02
                                    136 ; genSend
      008165 AE 50 05         [ 2]  137 	ldw	x, #0x5005
                                    138 ; genCall
      008168 CD 84 30         [ 4]  139 	call	_GPIO_WriteLow
                                    140 ;	./src/main.c: 25: mask = 0b10000000;
                                    141 ; genAssign
      00816B A6 80            [ 1]  142 	ld	a, #0x80
      00816D 6B 02            [ 1]  143 	ld	(0x02, sp), a
                                    144 ;	./src/main.c: 26: CLK_LOW;
                                    145 ; genSend
      00816F A6 01            [ 1]  146 	ld	a, #0x01
                                    147 ; genSend
      008171 AE 50 05         [ 2]  148 	ldw	x, #0x5005
                                    149 ; genCall
      008174 CD 84 30         [ 4]  150 	call	_GPIO_WriteLow
                                    151 ;	./src/main.c: 28: while (mask) {
                                    152 ; genLabel
      008177                        153 00104$:
                                    154 ; genIfx
      008177 0D 02            [ 1]  155 	tnz	(0x02, sp)
      008179 26 03            [ 1]  156 	jrne	00157$
      00817B CC 81 B0         [ 2]  157 	jp	00106$
      00817E                        158 00157$:
                                    159 ;	./src/main.c: 29: if (mask & address) {
                                    160 ; genAnd
      00817E 7B 02            [ 1]  161 	ld	a, (0x02, sp)
      008180 14 01            [ 1]  162 	and	a, (0x01, sp)
                                    163 ; genIfx
      008182 4D               [ 1]  164 	tnz	a
      008183 26 03            [ 1]  165 	jrne	00158$
      008185 CC 81 93         [ 2]  166 	jp	00102$
      008188                        167 00158$:
                                    168 ;	./src/main.c: 30: DATA_HIGH;
                                    169 ; genSend
      008188 A6 04            [ 1]  170 	ld	a, #0x04
                                    171 ; genSend
      00818A AE 50 05         [ 2]  172 	ldw	x, #0x5005
                                    173 ; genCall
      00818D CD 85 5C         [ 4]  174 	call	_GPIO_WriteHigh
                                    175 ; genGoto
      008190 CC 81 9B         [ 2]  176 	jp	00103$
                                    177 ; genLabel
      008193                        178 00102$:
                                    179 ;	./src/main.c: 32: DATA_LOW;
                                    180 ; genSend
      008193 A6 04            [ 1]  181 	ld	a, #0x04
                                    182 ; genSend
      008195 AE 50 05         [ 2]  183 	ldw	x, #0x5005
                                    184 ; genCall
      008198 CD 84 30         [ 4]  185 	call	_GPIO_WriteLow
                                    186 ; genLabel
      00819B                        187 00103$:
                                    188 ;	./src/main.c: 34: CLK_HIGH;
                                    189 ; genSend
      00819B A6 01            [ 1]  190 	ld	a, #0x01
                                    191 ; genSend
      00819D AE 50 05         [ 2]  192 	ldw	x, #0x5005
                                    193 ; genCall
      0081A0 CD 85 5C         [ 4]  194 	call	_GPIO_WriteHigh
                                    195 ;	./src/main.c: 35: mask = mask >> 1;
                                    196 ; genRightShiftLiteral
      0081A3 04 02            [ 1]  197 	srl	(0x02, sp)
                                    198 ;	./src/main.c: 36: CLK_LOW;
                                    199 ; genSend
      0081A5 A6 01            [ 1]  200 	ld	a, #0x01
                                    201 ; genSend
      0081A7 AE 50 05         [ 2]  202 	ldw	x, #0x5005
                                    203 ; genCall
      0081AA CD 84 30         [ 4]  204 	call	_GPIO_WriteLow
                                    205 ; genGoto
      0081AD CC 81 77         [ 2]  206 	jp	00104$
                                    207 ; genLabel
      0081B0                        208 00106$:
                                    209 ;	./src/main.c: 39: mask = 0b10000000;
                                    210 ; genAssign
      0081B0 A6 80            [ 1]  211 	ld	a, #0x80
      0081B2 6B 02            [ 1]  212 	ld	(0x02, sp), a
                                    213 ;	./src/main.c: 41: while (mask) {
                                    214 ; genLabel
      0081B4                        215 00110$:
                                    216 ; genIfx
      0081B4 0D 02            [ 1]  217 	tnz	(0x02, sp)
      0081B6 26 03            [ 1]  218 	jrne	00159$
      0081B8 CC 81 ED         [ 2]  219 	jp	00112$
      0081BB                        220 00159$:
                                    221 ;	./src/main.c: 42: if (mask & data) {
                                    222 ; genAnd
      0081BB 7B 02            [ 1]  223 	ld	a, (0x02, sp)
      0081BD 14 05            [ 1]  224 	and	a, (0x05, sp)
                                    225 ; genIfx
      0081BF 4D               [ 1]  226 	tnz	a
      0081C0 26 03            [ 1]  227 	jrne	00160$
      0081C2 CC 81 D0         [ 2]  228 	jp	00108$
      0081C5                        229 00160$:
                                    230 ;	./src/main.c: 43: DATA_HIGH;
                                    231 ; genSend
      0081C5 A6 04            [ 1]  232 	ld	a, #0x04
                                    233 ; genSend
      0081C7 AE 50 05         [ 2]  234 	ldw	x, #0x5005
                                    235 ; genCall
      0081CA CD 85 5C         [ 4]  236 	call	_GPIO_WriteHigh
                                    237 ; genGoto
      0081CD CC 81 D8         [ 2]  238 	jp	00109$
                                    239 ; genLabel
      0081D0                        240 00108$:
                                    241 ;	./src/main.c: 45: DATA_LOW;
                                    242 ; genSend
      0081D0 A6 04            [ 1]  243 	ld	a, #0x04
                                    244 ; genSend
      0081D2 AE 50 05         [ 2]  245 	ldw	x, #0x5005
                                    246 ; genCall
      0081D5 CD 84 30         [ 4]  247 	call	_GPIO_WriteLow
                                    248 ; genLabel
      0081D8                        249 00109$:
                                    250 ;	./src/main.c: 47: CLK_HIGH;
                                    251 ; genSend
      0081D8 A6 01            [ 1]  252 	ld	a, #0x01
                                    253 ; genSend
      0081DA AE 50 05         [ 2]  254 	ldw	x, #0x5005
                                    255 ; genCall
      0081DD CD 85 5C         [ 4]  256 	call	_GPIO_WriteHigh
                                    257 ;	./src/main.c: 48: mask = mask >> 1;
                                    258 ; genRightShiftLiteral
      0081E0 04 02            [ 1]  259 	srl	(0x02, sp)
                                    260 ;	./src/main.c: 49: CLK_LOW;
                                    261 ; genSend
      0081E2 A6 01            [ 1]  262 	ld	a, #0x01
                                    263 ; genSend
      0081E4 AE 50 05         [ 2]  264 	ldw	x, #0x5005
                                    265 ; genCall
      0081E7 CD 84 30         [ 4]  266 	call	_GPIO_WriteLow
                                    267 ; genGoto
      0081EA CC 81 B4         [ 2]  268 	jp	00110$
                                    269 ; genLabel
      0081ED                        270 00112$:
                                    271 ;	./src/main.c: 52: CS_HIGH;
                                    272 ; genSend
      0081ED A6 02            [ 1]  273 	ld	a, #0x02
                                    274 ; genSend
      0081EF AE 50 05         [ 2]  275 	ldw	x, #0x5005
                                    276 ; genCall
      0081F2 CD 85 5C         [ 4]  277 	call	_GPIO_WriteHigh
                                    278 ; genLabel
      0081F5                        279 00113$:
                                    280 ;	./src/main.c: 53: }
                                    281 ; genEndFunction
      0081F5 85               [ 2]  282 	popw	x
      0081F6 85               [ 2]  283 	popw	x
      0081F7 84               [ 1]  284 	pop	a
      0081F8 FC               [ 2]  285 	jp	(x)
                                    286 ;	./src/main.c: 55: void max7219_init(void) { 
                                    287 ; genLabel
                                    288 ;	-----------------------------------------
                                    289 ;	 function max7219_init
                                    290 ;	-----------------------------------------
                                    291 ;	Register assignment is optimal.
                                    292 ;	Stack space usage: 0 bytes.
      0081F9                        293 _max7219_init:
                                    294 ;	./src/main.c: 56: GPIO_Init(CS_GPIO, CS_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
                                    295 ; genIPush
      0081F9 4B C0            [ 1]  296 	push	#0xc0
                                    297 ; genSend
      0081FB A6 02            [ 1]  298 	ld	a, #0x02
                                    299 ; genSend
      0081FD AE 50 05         [ 2]  300 	ldw	x, #0x5005
                                    301 ; genCall
      008200 CD 83 2B         [ 4]  302 	call	_GPIO_Init
                                    303 ;	./src/main.c: 57: GPIO_Init(CLK_GPIO, CLK_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
                                    304 ; genIPush
      008203 4B C0            [ 1]  305 	push	#0xc0
                                    306 ; genSend
      008205 A6 01            [ 1]  307 	ld	a, #0x01
                                    308 ; genSend
      008207 AE 50 05         [ 2]  309 	ldw	x, #0x5005
                                    310 ; genCall
      00820A CD 83 2B         [ 4]  311 	call	_GPIO_Init
                                    312 ;	./src/main.c: 58: GPIO_Init(DATA_GPIO, DATA_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
                                    313 ; genIPush
      00820D 4B C0            [ 1]  314 	push	#0xc0
                                    315 ; genSend
      00820F A6 04            [ 1]  316 	ld	a, #0x04
                                    317 ; genSend
      008211 AE 50 05         [ 2]  318 	ldw	x, #0x5005
                                    319 ; genCall
      008214 CD 83 2B         [ 4]  320 	call	_GPIO_Init
                                    321 ;	./src/main.c: 61: max7219_send(DECODE_MODE, DECODE_ALL);        // dekodér
                                    322 ; genIPush
      008217 4B FF            [ 1]  323 	push	#0xff
                                    324 ; genSend
      008219 A6 09            [ 1]  325 	ld	a, #0x09
                                    326 ; genCall
      00821B CD 81 60         [ 4]  327 	call	_max7219_send
                                    328 ;	./src/main.c: 62: max7219_send(SCAN_LIMIT, 0);                  // Kolik cifer zapneme(0=1cifra)
                                    329 ; genIPush
      00821E 4B 00            [ 1]  330 	push	#0x00
                                    331 ; genSend
      008220 A6 0B            [ 1]  332 	ld	a, #0x0b
                                    333 ; genCall
      008222 CD 81 60         [ 4]  334 	call	_max7219_send
                                    335 ;	./src/main.c: 63: max7219_send(INTENSITY, 5);                   // Jas
                                    336 ; genIPush
      008225 4B 05            [ 1]  337 	push	#0x05
                                    338 ; genSend
      008227 A6 0A            [ 1]  339 	ld	a, #0x0a
                                    340 ; genCall
      008229 CD 81 60         [ 4]  341 	call	_max7219_send
                                    342 ;	./src/main.c: 64: max7219_send(DISPLAY_TEST, DISPLAY_TEST_OFF); // test displeje
                                    343 ; genIPush
      00822C 4B 00            [ 1]  344 	push	#0x00
                                    345 ; genSend
      00822E A6 0F            [ 1]  346 	ld	a, #0x0f
                                    347 ; genCall
      008230 CD 81 60         [ 4]  348 	call	_max7219_send
                                    349 ;	./src/main.c: 65: max7219_send(SHUTDOWN, SHUTDOWN_ON);          // On/OFF
                                    350 ; genIPush
      008233 4B 01            [ 1]  351 	push	#0x01
                                    352 ; genSend
      008235 A6 0C            [ 1]  353 	ld	a, #0x0c
                                    354 ; genCall
      008237 CD 81 60         [ 4]  355 	call	_max7219_send
                                    356 ; genLabel
      00823A                        357 00101$:
                                    358 ;	./src/main.c: 66: }
                                    359 ; genEndFunction
      00823A 81               [ 4]  360 	ret
                                    361 ;	./src/main.c: 68: void setup(void) {
                                    362 ; genLabel
                                    363 ;	-----------------------------------------
                                    364 ;	 function setup
                                    365 ;	-----------------------------------------
                                    366 ;	Register assignment is optimal.
                                    367 ;	Stack space usage: 0 bytes.
      00823B                        368 _setup:
                                    369 ;	./src/main.c: 69: CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // taktovat MCU na 16MHz
                                    370 ; genSend
      00823B 4F               [ 1]  371 	clr	a
                                    372 ; genCall
      00823C CD 84 5A         [ 4]  373 	call	_CLK_HSIPrescalerConfig
                                    374 ;	./src/main.c: 71: init_milis();
                                    375 ; genCall
      00823F CD 82 D7         [ 4]  376 	call	_init_milis
                                    377 ;	./src/main.c: 73: max7219_init();
                                    378 ; genCall
      008242 CC 81 F9         [ 2]  379 	jp	_max7219_init
                                    380 ; genLabel
      008245                        381 00101$:
                                    382 ;	./src/main.c: 74: }
                                    383 ; genEndFunction
      008245 81               [ 4]  384 	ret
                                    385 ;	./src/main.c: 76: int main(void) {
                                    386 ; genLabel
                                    387 ;	-----------------------------------------
                                    388 ;	 function main
                                    389 ;	-----------------------------------------
                                    390 ;	Register assignment might be sub-optimal.
                                    391 ;	Stack space usage: 13 bytes.
      008246                        392 _main:
      008246 52 0D            [ 2]  393 	sub	sp, #13
                                    394 ;	./src/main.c: 78: setup();
                                    395 ; genCall
      008248 CD 82 3B         [ 4]  396 	call	_setup
                                    397 ;	./src/main.c: 80: max7219_send(DIGIT0, 0);
                                    398 ; genIPush
      00824B 4B 00            [ 1]  399 	push	#0x00
                                    400 ; genSend
      00824D A6 01            [ 1]  401 	ld	a, #0x01
                                    402 ; genCall
      00824F CD 81 60         [ 4]  403 	call	_max7219_send
                                    404 ;	./src/main.c: 83: uint32_t time = 0;
                                    405 ; genAssign
      008252 5F               [ 1]  406 	clrw	x
      008253 1F 03            [ 2]  407 	ldw	(0x03, sp), x
      008255 1F 01            [ 2]  408 	ldw	(0x01, sp), x
                                    409 ;	./src/main.c: 84: uint8_t jednotky = 0;
                                    410 ; genAssign
      008257 0F 0D            [ 1]  411 	clr	(0x0d, sp)
                                    412 ;	./src/main.c: 87: while (1) {
                                    413 ; genLabel
      008259                        414 00106$:
                                    415 ;	./src/main.c: 88: if ((milis() - time) > 500) {
                                    416 ; genCall
      008259 CD 82 B7         [ 4]  417 	call	_milis
      00825C 1F 0B            [ 2]  418 	ldw	(0x0b, sp), x
                                    419 ; genCast
      00825E 16 0B            [ 2]  420 	ldw	y, (0x0b, sp)
      008260 17 07            [ 2]  421 	ldw	(0x07, sp), y
      008262 7B 07            [ 1]  422 	ld	a, (0x07, sp)
      008264 49               [ 1]  423 	rlc	a
      008265 4F               [ 1]  424 	clr	a
      008266 A2 00            [ 1]  425 	sbc	a, #0x00
      008268 6B 06            [ 1]  426 	ld	(0x06, sp), a
                                    427 ; genMinus
      00826A 1E 07            [ 2]  428 	ldw	x, (0x07, sp)
      00826C 72 F0 03         [ 2]  429 	subw	x, (0x03, sp)
      00826F 1F 0B            [ 2]  430 	ldw	(0x0b, sp), x
      008271 88               [ 1]  431 	push	a
      008272 7B 07            [ 1]  432 	ld	a, (0x07, sp)
      008274 12 03            [ 1]  433 	sbc	a, (0x03, sp)
      008276 6B 0B            [ 1]  434 	ld	(0x0b, sp), a
      008278 84               [ 1]  435 	pop	a
      008279 12 01            [ 1]  436 	sbc	a, (0x01, sp)
      00827B 6B 09            [ 1]  437 	ld	(0x09, sp), a
                                    438 ; genCmp
                                    439 ; genCmpTnz
      00827D AE 01 F4         [ 2]  440 	ldw	x, #0x01f4
      008280 13 0B            [ 2]  441 	cpw	x, (0x0b, sp)
      008282 4F               [ 1]  442 	clr	a
      008283 12 0A            [ 1]  443 	sbc	a, (0x0a, sp)
      008285 4F               [ 1]  444 	clr	a
      008286 12 09            [ 1]  445 	sbc	a, (0x09, sp)
      008288 25 03            [ 1]  446 	jrc	00131$
      00828A CC 82 59         [ 2]  447 	jp	00106$
      00828D                        448 00131$:
                                    449 ; skipping generated iCode
                                    450 ;	./src/main.c: 89: time = milis();
                                    451 ; genCall
      00828D CD 82 B7         [ 4]  452 	call	_milis
                                    453 ; genCast
      008290 1F 03            [ 2]  454 	ldw	(0x03, sp), x
      008292 7B 03            [ 1]  455 	ld	a, (0x03, sp)
      008294 49               [ 1]  456 	rlc	a
      008295 4F               [ 1]  457 	clr	a
      008296 A2 00            [ 1]  458 	sbc	a, #0x00
      008298 6B 02            [ 1]  459 	ld	(0x02, sp), a
      00829A 6B 01            [ 1]  460 	ld	(0x01, sp), a
                                    461 ;	./src/main.c: 90: max7219_send(DIGIT0, jednotky);
                                    462 ; genIPush
      00829C 7B 0D            [ 1]  463 	ld	a, (0x0d, sp)
      00829E 88               [ 1]  464 	push	a
                                    465 ; genSend
      00829F A6 01            [ 1]  466 	ld	a, #0x01
                                    467 ; genCall
      0082A1 CD 81 60         [ 4]  468 	call	_max7219_send
                                    469 ;	./src/main.c: 92: jednotky++;
                                    470 ; genPlus
      0082A4 0C 0D            [ 1]  471 	inc	(0x0d, sp)
                                    472 ;	./src/main.c: 93: if (jednotky > 9) {
                                    473 ; genCmp
                                    474 ; genCmpTnz
      0082A6 7B 0D            [ 1]  475 	ld	a, (0x0d, sp)
      0082A8 A1 09            [ 1]  476 	cp	a, #0x09
      0082AA 22 03            [ 1]  477 	jrugt	00132$
      0082AC CC 82 59         [ 2]  478 	jp	00106$
      0082AF                        479 00132$:
                                    480 ; skipping generated iCode
                                    481 ;	./src/main.c: 94: jednotky = 0;
                                    482 ; genAssign
      0082AF 0F 0D            [ 1]  483 	clr	(0x0d, sp)
                                    484 ; genGoto
      0082B1 CC 82 59         [ 2]  485 	jp	00106$
                                    486 ; genLabel
      0082B4                        487 00108$:
                                    488 ;	./src/main.c: 98: }
                                    489 ; genEndFunction
      0082B4 5B 0D            [ 2]  490 	addw	sp, #13
      0082B6 81               [ 4]  491 	ret
                                    492 	.area CODE
                                    493 	.area CONST
                                    494 	.area INITIALIZER
                                    495 	.area CABS (ABS)
