/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2010, Embedded Artists AB
 *   All rights reserved.
 *
 ******************************************************************************/


#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "rotary.h"
#include "led7seg.h"
#include "oled.h"
#include "rgb.h"

#define CHECKPOINT_ADC_CHANNEL ADC_CHANNEL_5
#define CHECKPOINT_ADC_PORT    1
#define CHECKPOINT_ADC_PIN     31
#define CHECKPOINT_ADC_FUNC    3

static uint8_t barPos = 2;

static void moveBar(uint8_t steps, uint8_t dir)
{
    uint16_t ledOn = 0;

    if (barPos == 0)
        ledOn = (1 << 0) | (3 << 14);
    else if (barPos == 1)
        ledOn = (3 << 0) | (1 << 15);
    else
        ledOn = 0x07 << (barPos-2);

    barPos += (dir*steps);
    barPos = (barPos % 16);

    pca9532_setLeds(ledOn, 0xffff);
}

static void drawOled(uint8_t joyState)
{
    static int wait = 0;
    static uint8_t currX = 48;
    static uint8_t currY = 32;
    static uint8_t lastX = 0;
    static uint8_t lastY = 0;

    if ((joyState & JOYSTICK_CENTER) != 0) {
        oled_clearScreen(OLED_COLOR_BLACK);
        return;
    }

    if (wait++ < 3)
        return;

    wait = 0;

    if ((joyState & JOYSTICK_UP) != 0 && currY > 0) {
        currY--;
    }

    if ((joyState & JOYSTICK_DOWN) != 0 && currY < OLED_DISPLAY_HEIGHT-1) {
        currY++;
    }

    if ((joyState & JOYSTICK_RIGHT) != 0 && currX < OLED_DISPLAY_WIDTH-1) {
        currX++;
    }

    if ((joyState & JOYSTICK_LEFT) != 0 && currX > 0) {
        currX--;
    }

    if (lastX != currX || lastY != currY) {
        oled_putPixel(currX, currY, OLED_COLOR_WHITE);
        lastX = currX;
        lastY = currY;
    }
}

static uint8_t targetTurns = 0;
static uint8_t currentTurns = 0;

static uint8_t trimToTurns(uint32_t trim)
{
    uint32_t turns = (trim * 10) / 4096;

    if (turns > 9) {
        turns = 9;
    }

    return (uint8_t)turns;
}

static void updateCheckpointDisplayAndLed(uint32_t trim, uint8_t rotaryDir)
{
    uint8_t newTarget = trimToTurns(trim);

    /* Default state for each cycle is LED off. */
    rgb_setLeds(0);

    if (newTarget == 0) {
        targetTurns = 0;
        currentTurns = 0;
        led7seg_setChar((uint8_t)('0'), FALSE);
        return;
    }

    if (newTarget != targetTurns) {
        targetTurns = newTarget;
        led7seg_setChar((uint8_t)('0' + targetTurns), FALSE);
    }

    if (rotaryDir == ROTARY_RIGHT) {
        if (currentTurns < 99) {
            currentTurns++;
        }
    }
    else if (rotaryDir == ROTARY_LEFT) {
        if (currentTurns > 0) {
            currentTurns--;
        }
    }

    if ((currentTurns == targetTurns) && (targetTurns > 0)) {
        rgb_setLeds(RGB_GREEN);
    }
}

static uint32_t readTrimFiltered(void)
{
    return readAdcAverage(CHECKPOINT_ADC_CHANNEL);
}

static uint8_t readRotaryEventNonBlocking(void)
{
    static uint8_t prevState = 0x03;
    uint8_t currState = (uint8_t)((GPIO_ReadValue(0) >> 24) & 0x03);
    uint8_t event = ROTARY_WAIT;

    /*
     * Non-blocking edge detection to avoid stalling the main loop.
     * On this board: 3->2 typically means right, 3->1 left.
     */
    if (prevState == 0x03) {
        if (currState == 0x02) {
            event = ROTARY_RIGHT;
        }
        else if (currState == 0x01) {
            event = ROTARY_LEFT;
        }
    }

    prevState = currState;
    return event;
}

static uint32_t readAdcAverage(uint8_t channel)
{
    uint32_t sum = 0;
    uint32_t i = 0;

    for (i = 0; i < 8; i++) {
        ADC_StartCmd(LPC_ADC,ADC_START_NOW);
        while (!(ADC_ChannelGetStatus(LPC_ADC,channel,ADC_DATA_DONE)));
        sum += ADC_ChannelGetData(LPC_ADC,channel);
    }

    return (sum / 8);
}

#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);




static uint32_t notes[] = {
        2272, // A - 440 Hz
        2024, // B - 494 Hz
        3816, // C - 262 Hz
        3401, // D - 294 Hz
        3030, // E - 330 Hz
        2865, // F - 349 Hz
        2551, // G - 392 Hz
        1136, // a - 880 Hz
        1012, // b - 988 Hz
        1912, // c - 523 Hz
        1703, // d - 587 Hz
        1517, // e - 659 Hz
        1432, // f - 698 Hz
        1275, // g - 784 Hz
};

static void playNote(uint32_t note, uint32_t durationMs) {

    uint32_t t = 0;

    if (note > 0) {

        while (t < (durationMs*1000)) {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            NOTE_PIN_LOW();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            t += note;
        }

    }
    else {
    	Timer0_Wait(durationMs);
        //delay32Ms(0, durationMs);
    }
}

static uint32_t getNote(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'G')
        return notes[ch - 'A'];

    if (ch >= 'a' && ch <= 'g')
        return notes[ch - 'a' + 7];

    return 0;
}

static uint32_t getDuration(uint8_t ch)
{
    if (ch < '0' || ch > '9')
        return 400;

    /* number of ms */

    return (ch - '0') * 200;
}

static uint32_t getPause(uint8_t ch)
{
    switch (ch) {
    case '+':
        return 0;
    case ',':
        return 5;
    case '.':
        return 20;
    case '_':
        return 30;
    default:
        return 5;
    }
}

static void playSong(uint8_t *song) {
    uint32_t note = 0;
    uint32_t dur  = 0;
    uint32_t pause = 0;

    /*
     * A song is a collection of tones where each tone is
     * a note, duration and pause, e.g.
     *
     * "E2,F4,"
     */

    while(*song != '\0') {
        note = getNote(*song++);
        if (*song == '\0')
            break;
        dur  = getDuration(*song++);
        if (*song == '\0')
            break;
        pause = getPause(*song++);

        playNote(note, dur);
        //delay32Ms(0, pause);
        Timer0_Wait(pause);

    }
}

static uint8_t * song = (uint8_t*)"C2.C2,D4,C4,F4,E8,";
        //(uint8_t*)"C2.C2,D4,C4,F4,E8,C2.C2,D4,C4,G4,F8,C2.C2,c4,A4,F4,E4,D4,A2.A2,H4,F4,G4,F8,";
        //"D4,B4,B4,A4,A4,G4,E4,D4.D2,E4,E4,A4,F4,D8.D4,d4,d4,c4,c4,B4,G4,E4.E2,F4,F4,A4,A4,G8,";



static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_adc(void)
{
	PINSEL_CFG_Type PinCfg;

	/*
	 * Init ADC pin connect
     * AD0.5 on P1.31 (7-segment checkpoint input)
	 */
    PinCfg.Funcnum = CHECKPOINT_ADC_FUNC;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
    PinCfg.Pinnum = CHECKPOINT_ADC_PIN;
    PinCfg.Portnum = CHECKPOINT_ADC_PORT;
	PINSEL_ConfigPin(&PinCfg);

	/* Configuration for ADC :
	 * 	Frequency at 0.2Mhz
     *  ADC selected channel, no Interrupt
	 */
	ADC_Init(LPC_ADC, 200000);
    ADC_IntConfig(LPC_ADC,CHECKPOINT_ADC_CHANNEL,DISABLE);
    ADC_ChannelCmd(LPC_ADC,CHECKPOINT_ADC_CHANNEL,ENABLE);

}


int main (void) {

    uint8_t rotaryState = 0;

    uint32_t trim = 0;


    /* Checkpoint mode: only rotary + 7-seg + trimpot + RGB */
    init_ssp();
    init_adc();

    rotary_init();
    led7seg_init();
    rgb_init();

    led7seg_setChar('0', FALSE);
    rgb_setLeds(0);

    while (1) {

        rotaryState = readRotaryEventNonBlocking();

        trim = readTrimFiltered();

        updateCheckpointDisplayAndLed(trim, rotaryState);

        Timer0_Wait(1);
    }


}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
