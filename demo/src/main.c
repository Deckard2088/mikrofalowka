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

#define CHECKPOINT_ADC_CHANNEL ADC_CHANNEL_5
#define CHECKPOINT_ADC_PORT    1
#define CHECKPOINT_ADC_PIN     31
#define CHECKPOINT_ADC_FUNC    3

static uint8_t targetTurns = 0;

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

    /* Default state for each cycle is LED off. */
    rgb_setLeds(0);

    if (newTarget == 0) {
        targetTurns = 0;
        led7seg_setChar((uint8_t)('0'), FALSE);
        return;
    }

    if (newTarget != targetTurns) {
        targetTurns = newTarget;
        led7seg_setChar((uint8_t)('0' + targetTurns), FALSE);
    }

    if (targetTurns > 0) {
        rgb_setLeds(RGB_GREEN);
    }
}

static uint32_t readTrimFiltered(void)
{
    return readAdcAverage(CHECKPOINT_ADC_CHANNEL);
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
     * AD0.5 on P1.31 (second potentiometer)
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
