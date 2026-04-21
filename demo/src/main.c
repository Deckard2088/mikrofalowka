/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2010, Embedded Artists AB
 *   All rights reserved.
 *
 ******************************************************************************/


#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"

#include "led7seg.h"
#include "rgb.h"

#define CHECKPOINT_ADC_A_CHANNEL ADC_CHANNEL_0
#define CHECKPOINT_ADC_A_PORT    0
#define CHECKPOINT_ADC_A_PIN     23
#define CHECKPOINT_ADC_A_FUNC    1

#define CHECKPOINT_ADC_B_CHANNEL ADC_CHANNEL_5
#define CHECKPOINT_ADC_B_PORT    1
#define CHECKPOINT_ADC_B_PIN     31
#define CHECKPOINT_ADC_B_FUNC    3

static uint32_t readAdcAverage(uint8_t channel);

static uint8_t trimToTurns(uint32_t trim)
{
    uint32_t turns = (trim * 10) / 4096;

    if (turns > 9) {
        turns = 9;
    }

    return (uint8_t)turns;
}

static void updateCheckpointDisplayAndLed(uint32_t trim)
{
    uint8_t newTarget = trimToTurns(trim);

    /* Potentiometer directly drives the 7-segment digit. */
    led7seg_setChar((uint8_t)('0' + newTarget), FALSE);

    /* Default state for each cycle is LED off. */
    rgb_setLeds(0);

    if (newTarget > 0) {
        rgb_setLeds(RGB_GREEN);
    }
}

static uint32_t readTrimFiltered(void)
{
    static uint8_t initialized = 0;
    static uint8_t useB = 1;
    static uint32_t lastA = 0;
    static uint32_t lastB = 0;
    uint32_t trimA = readAdcAverage(CHECKPOINT_ADC_A_CHANNEL);
    uint32_t trimB = readAdcAverage(CHECKPOINT_ADC_B_CHANNEL);
    uint32_t deltaA = (trimA > lastA) ? (trimA - lastA) : (lastA - trimA);
    uint32_t deltaB = (trimB > lastB) ? (trimB - lastB) : (lastB - trimB);

    if (!initialized) {
        initialized = 1;
        lastA = trimA;
        lastB = trimB;
        return trimB;
    }

    if (deltaA > (deltaB + 20)) {
        useB = 0;
    }
    else if (deltaB > (deltaA + 20)) {
        useB = 1;
    }

    lastA = trimA;
    lastB = trimB;

    return useB ? trimB : trimA;
}

static void init_ssp(void)
{
    SSP_CFG_Type SSP_ConfigStruct;
    PINSEL_CFG_Type PinCfg;

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
    SSP_Init(LPC_SSP1, &SSP_ConfigStruct);
    SSP_Cmd(LPC_SSP1, ENABLE);
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
static void init_adc(void)
{
	PINSEL_CFG_Type PinCfg;

    /*
     * Init ADC pin connect
     * AD0.0 on P0.23 and AD0.5 on P1.31
     */
    PinCfg.Funcnum = CHECKPOINT_ADC_A_FUNC;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
    PinCfg.Pinnum = CHECKPOINT_ADC_A_PIN;
    PinCfg.Portnum = CHECKPOINT_ADC_A_PORT;
    PINSEL_ConfigPin(&PinCfg);

    PinCfg.Funcnum = CHECKPOINT_ADC_B_FUNC;
    PinCfg.Pinnum = CHECKPOINT_ADC_B_PIN;
    PinCfg.Portnum = CHECKPOINT_ADC_B_PORT;
	PINSEL_ConfigPin(&PinCfg);

	/* Configuration for ADC :
	 * 	Frequency at 0.2Mhz
     *  ADC channels 0 and 5, no Interrupt
	 */
	ADC_Init(LPC_ADC, 200000);
    ADC_IntConfig(LPC_ADC,CHECKPOINT_ADC_A_CHANNEL,DISABLE);
    ADC_IntConfig(LPC_ADC,CHECKPOINT_ADC_B_CHANNEL,DISABLE);
    ADC_ChannelCmd(LPC_ADC,CHECKPOINT_ADC_A_CHANNEL,ENABLE);
    ADC_ChannelCmd(LPC_ADC,CHECKPOINT_ADC_B_CHANNEL,ENABLE);

}


int main (void) {

    uint32_t trim = 0;

    init_ssp();
    init_adc();

    led7seg_init();
    rgb_init();

    led7seg_setChar('0', FALSE);
    rgb_setLeds(0);

    while (1) {
        trim = readTrimFiltered();

        updateCheckpointDisplayAndLed(trim);

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
