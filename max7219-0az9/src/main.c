#include "max7219.h"
#include <main.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm8s.h>

#define CLK_GPIO GPIOB
#define CLK_PIN GPIO_PIN_0
#define DATA_GPIO GPIOB
#define CS_GPIO GPIOB
#define CS_PIN GPIO_PIN_1
#define DATA_PIN GPIO_PIN_2

#define CLK_HIGH GPIO_WriteHigh(CLK_GPIO, CLK_PIN)
#define CLK_LOW GPIO_WriteLow(CLK_GPIO, CLK_PIN)
#define DATA_HIGH GPIO_WriteHigh(DATA_GPIO, DATA_PIN)
#define DATA_LOW GPIO_WriteLow(DATA_GPIO, DATA_PIN)
#define CS_HIGH GPIO_WriteHigh(CS_GPIO, CS_PIN)
#define CS_LOW GPIO_WriteLow(CS_GPIO, CS_PIN)

void max7219_send(uint8_t address, uint8_t data) {
    uint8_t mask;
    CS_LOW;

    mask = 0b10000000;
    CLK_LOW;

    while (mask) {
        if (mask & address) {
            DATA_HIGH;
        } else {
            DATA_LOW;
        }
        CLK_HIGH;
        mask = mask >> 1;
        CLK_LOW;
    }

    mask = 0b10000000;

    while (mask) {
        if (mask & data) {
            DATA_HIGH;
        } else {
            DATA_LOW;
        }
        CLK_HIGH;
        mask = mask >> 1;
        CLK_LOW;
    }

    CS_HIGH;
}

void max7219_init(void) { 
    GPIO_Init(CS_GPIO, CS_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(CLK_GPIO, CLK_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
    GPIO_Init(DATA_GPIO, DATA_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);

    // základní parametry pro fci
    max7219_send(DECODE_MODE, DECODE_ALL);        // dekodér
    max7219_send(SCAN_LIMIT, 0);                  // Kolik cifer zapneme(0=1cifra)
    max7219_send(INTENSITY, 5);                   // Jas
    max7219_send(DISPLAY_TEST, DISPLAY_TEST_OFF); // test displeje
    max7219_send(SHUTDOWN, SHUTDOWN_ON);          // On/OFF
}

void setup(void) {
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // taktovat MCU na 16MHz

    init_milis();

    max7219_init();
}

int main(void) {

    setup();

    max7219_send(DIGIT0, 0);
   /*digit0 až 7*/

    uint32_t time = 0;
    uint8_t jednotky = 0;
   

    while (1) {
        if ((milis() - time) > 500) {
            time = milis();
            max7219_send(DIGIT0, jednotky);

            jednotky++;
            if (jednotky > 9) {
                jednotky = 0;
            }
        }
    }
}
/*-------------------------------  Assert -----------------------------------*/
#include "__assert__.h"