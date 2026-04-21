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

#include "rotary.h"
#include "led7seg.h"
#include "rgb.h"
#include "light.h"
#include "oled.h"
#include "temp.h"
#include "acc.h"

#define AUTO_DECAY_INTERVAL_MS 5000
#define OLED_REFRESH_INTERVAL_MS 200
#define BUZZER_PIN_HIGH() GPIO_SetValue(0, 1<<26)
#define BUZZER_PIN_LOW()  GPIO_ClearValue(0, 1<<26)

static uint8_t ch7seg = '0';
static uint32_t msTicks = 0;
static uint8_t buf[12];

static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base)
{
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0;
    int tmpValue = value;

    if (pBuf == NULL || len < 2) {
        return;
    }

    if (base < 2 || base > 36) {
        return;
    }

    if (value < 0) {
        tmpValue = -tmpValue;
        value = -value;
        pBuf[pos++] = '-';
    }

    do {
        pos++;
        tmpValue /= (int)base;
    } while (tmpValue > 0);

    if ((uint32_t)pos > len) {
        return;
    }

    pBuf[pos] = '\0';

    do {
        pBuf[--pos] = (uint8_t)pAscii[value % (int)base];
        value /= (int)base;
    } while (value > 0);
}

void SysTick_Handler(void)
{
    msTicks++;
}

static uint32_t getTicks(void)
{
    return msTicks;
}

static uint8_t rotate7SegChar(uint8_t ch)
{
    switch (ch) {
    case '0':
        return '0';
    case '1':
        return '|';
    case '2':
        return '2';
    case '3':
        return 'E';
    case '4':
        return 'h';
    case '5':
        return '5';
    case '6':
        return '9';
    case '7':
        return 'L';
    case '8':
        return '8';
    case '9':
        return '6';
    default:
        return ch;
    }
}

static void refreshOutputs(void)
{
    led7seg_setChar(rotate7SegChar(ch7seg), FALSE);

    if (ch7seg == '0')
        rgb_setLeds(0);
    else
        rgb_setLeds(RGB_GREEN);
}

static uint8_t change7Seg(uint8_t rotaryDir)
{
    if (rotaryDir != ROTARY_WAIT) {

        if (rotaryDir == ROTARY_RIGHT) {
            ch7seg++;
        }
        else {
            ch7seg--;
        }

        if (ch7seg > '9')
            ch7seg = '0';
        else if (ch7seg < '0')
            ch7seg = '9';

        refreshOutputs();
        return 1;
    }

    return 0;
}

static void init_buzzer(void)
{
    /* Speaker amplifier control and buzzer output pin. */
    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27);
    GPIO_ClearValue(0, 1<<28);
    GPIO_ClearValue(2, 1<<13);
    BUZZER_PIN_LOW();
}

static void buzzerPlayTone(uint32_t periodUs, uint32_t durationMs)
{
    uint32_t elapsed = 0;

    while (elapsed < (durationMs * 1000)) {
        BUZZER_PIN_HIGH();
        Timer0_us_Wait(periodUs / 2);

        BUZZER_PIN_LOW();
        Timer0_us_Wait(periodUs / 2);

        elapsed += periodUs;
    }
}

static void buzzerZeroPulse(void)
{
    /* Short annoying triple beep when zero is reached. */
    buzzerPlayTone(700, 75);
    Timer0_Wait(25);
    buzzerPlayTone(430, 75);
    Timer0_Wait(25);
    buzzerPlayTone(900, 75);
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

static void init_i2c(void)
{
    PINSEL_CFG_Type PinCfg;

    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);

    I2C_Init(LPC_I2C2, 100000);
    I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_adc(void)
{
    PINSEL_CFG_Type PinCfg;

    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 23;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, 200000);
    ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, DISABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
}

static void init_oledSensorView(void)
{
    oled_clearScreen(OLED_COLOR_WHITE);
    oled_putString(1,1,  (uint8_t*)"Temp   : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,9,  (uint8_t*)"Light  : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,17, (uint8_t*)"Trimpot: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,25, (uint8_t*)"Acc x  : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,33, (uint8_t*)"Acc y  : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,41, (uint8_t*)"Acc z  : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

static void update_oledSensorView(int32_t t, uint32_t lux, uint32_t trim, int8_t x, int8_t y, int8_t z)
{
    intToString(t, buf, 10, 10);
    oled_fillRect((1+9*6),1, 80, 8, OLED_COLOR_WHITE);
    oled_putString((1+9*6),1, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

    intToString((int)lux, buf, 10, 10);
    oled_fillRect((1+9*6),9, 80, 16, OLED_COLOR_WHITE);
    oled_putString((1+9*6),9, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

    intToString((int)trim, buf, 10, 10);
    oled_fillRect((1+9*6),17, 80, 24, OLED_COLOR_WHITE);
    oled_putString((1+9*6),17, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

    intToString((int)x, buf, 10, 10);
    oled_fillRect((1+9*6),25, 80, 32, OLED_COLOR_WHITE);
    oled_putString((1+9*6),25, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

    intToString((int)y, buf, 10, 10);
    oled_fillRect((1+9*6),33, 80, 40, OLED_COLOR_WHITE);
    oled_putString((1+9*6),33, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

    intToString((int)z, buf, 10, 10);
    oled_fillRect((1+9*6),41, 80, 48, OLED_COLOR_WHITE);
    oled_putString((1+9*6),41, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

static void service_oledSensors(int32_t xoff, int32_t yoff, int32_t zoff)
{
    static uint32_t lastOledTick = 0;
    uint32_t now = getTicks();
    int8_t x = 0;
    int8_t y = 0;
    int8_t z = 0;
    int32_t t = 0;
    uint32_t lux = 0;
    uint32_t trim = 0;

    if ((now - lastOledTick) < OLED_REFRESH_INTERVAL_MS) {
        return;
    }
    lastOledTick = now;

    acc_read(&x, &y, &z);
    x = (int8_t)(x + xoff);
    y = (int8_t)(y + yoff);
    z = (int8_t)(z + zoff);

    t = temp_read();
    lux = light_read();

    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)));
    trim = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);

    update_oledSensorView(t, lux, trim, x, y, z);
}


int main (void) {

    uint8_t rotaryState = 0;
    uint32_t elapsedMs = 0;
    uint8_t lastCh7seg = '0';
    int32_t xoff = 0;
    int32_t yoff = 0;
    int32_t zoff = 0;
    int8_t x = 0;
    int8_t y = 0;
    int8_t z = 0;

    init_i2c();
    init_ssp();
    init_adc();

    rotary_init();
    led7seg_init();
    rgb_init();
    init_buzzer();

    oled_init();
    light_init();
    acc_init();
    temp_init(&getTicks);

    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1) {
        }
    }

    acc_read(&x, &y, &z);
    xoff = 0 - x;
    yoff = 0 - y;
    zoff = 64 - z;

    light_enable();
    light_setRange(LIGHT_RANGE_4000);
    init_oledSensorView();

    refreshOutputs();

    while (1) {
//dziala
        rotaryState = rotary_read();
        if (change7Seg(rotaryState)) {
            elapsedMs = 0;
        }

        if (++elapsedMs >= AUTO_DECAY_INTERVAL_MS) {
            elapsedMs = 0;

            if (ch7seg > '0') {
                ch7seg--;
                refreshOutputs();
            }
        }

        if (ch7seg == '0' && lastCh7seg != '0') {
            /* Zero reached: short beep only once. */
            buzzerZeroPulse();
            refreshOutputs();
        }

        /* OLED refresh is serviced independently from rotary/7-seg logic. */
        service_oledSensors(xoff, yoff, zoff);

        lastCh7seg = ch7seg;

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
