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
#include "lpc17xx_timer.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_adc.h"

#include "rotary.h"
#include "led7seg.h"
#include "rgb.h"
#include "oled.h"
#include "temp.h"
#include "light.h"

#define AUTO_DECAY_INTERVAL_MS 5000
#define BUZZER_PIN_HIGH() GPIO_SetValue(0, 1<<26)
#define BUZZER_PIN_LOW()  GPIO_ClearValue(0, 1<<26)

#define MOTOR_PORT  2
#define MOTOR_IN1   (1U << 0)
#define MOTOR_IN2   (1U << 1)

static uint8_t ch7seg = '0';
static uint8_t buf[10];
static uint32_t msTicks = 0;

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
    Timer0_Wait(25);
    buzzerPlayTone(700, 75);
    Timer0_Wait(25);
    buzzerPlayTone(430, 75);
    Timer0_Wait(25);
    buzzerPlayTone(900, 75);
    Timer0_Wait(25);
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

static void init_motor(void)
{
    PINSEL_CFG_Type pinCfg;

    pinCfg.Funcnum = 0;
    pinCfg.OpenDrain = 0;
    pinCfg.Pinmode = 0;
    pinCfg.Portnum = MOTOR_PORT;

    pinCfg.Pinnum = 0;
    PINSEL_ConfigPin(&pinCfg);
    pinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&pinCfg);

    GPIO_SetDir(MOTOR_PORT, MOTOR_IN1 | MOTOR_IN2, 1);
    GPIO_ClearValue(MOTOR_PORT, MOTOR_IN1 | MOTOR_IN2);

    /* Simple forward drive: IN1 high, IN2 low. */
    GPIO_SetValue(MOTOR_PORT, MOTOR_IN1);
    GPIO_ClearValue(MOTOR_PORT, MOTOR_IN2);
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

    /* Initialize I2C peripheral */
    I2C_Init(LPC_I2C2, 100000);

    /* Enable I2C operation */
    I2C_Cmd(LPC_I2C2, ENABLE);
}

static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base)
{
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0;
    int tmpValue = value;

    if (pBuf == NULL || len < 2)
        return;

    if (base < 2 || base > 36)
        return;

    if (value < 0)
    {
        tmpValue = -tmpValue;
        value = -value;
        pBuf[pos++] = '-';
    }

    do {
        pos++;
        tmpValue /= base;
    } while(tmpValue > 0);

    if (pos > len)
        return;

    pBuf[pos] = '\0';

    do {
        pBuf[--pos] = pAscii[value % base];
        value /= base;
    } while(value > 0);
}

static uint32_t getTicks(void)
{
    return msTicks;
}

void SysTick_Handler(void)
{
    msTicks++;
}

static void init_adc(void)
{
    PINSEL_CFG_Type PinCfg;

    /* Init ADC pin connect - AD0.0 on P0.23 */
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 23;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);

    /* Configuration for ADC */
    ADC_Init(LPC_ADC, 200000);
    ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, DISABLE);
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
}


int main (void) {

    uint8_t rotaryState = 0;
    uint32_t lastDecayTime = 0;
    uint32_t lastOledUpdate = 0;
    uint8_t lastCh7seg = '0';
    int8_t temp = 0;
    uint32_t lux = 0;
    uint32_t trim = 0;

    init_ssp();
    init_i2c();
    init_adc();

    rotary_init();
    led7seg_init();
    rgb_init();
    init_buzzer();
    init_motor();
    oled_init();
    light_init();
    temp_init(&getTicks);

    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);  /* Capture error */
    }

    light_enable();
    light_setRange(LIGHT_RANGE_4000);

    oled_clearScreen(OLED_COLOR_WHITE);
    lastDecayTime = msTicks;
    lastOledUpdate = msTicks;

    refreshOutputs();

    while (1) {
        /* Always start ADC conversion (non-blocking) */
        ADC_StartCmd(LPC_ADC, ADC_START_NOW);

        rotaryState = rotary_read();
        if (change7Seg(rotaryState)) {
            lastDecayTime = msTicks;  /* Reset decay timer on change */
        }

        /* Decay timer - decrease every 5 seconds */
        if ((msTicks - lastDecayTime) >= AUTO_DECAY_INTERVAL_MS) {
            lastDecayTime = msTicks;

            if (ch7seg > '0') {
                ch7seg--;
                refreshOutputs();
            }
        }

        if (ch7seg == '0' && lastCh7seg != '0') {
            /* Zero reached: short beep only once. */
            buzzerZeroPulse();
            refreshOutputs();
            oled_clearScreen(OLED_COLOR_WHITE);
        }

        lastCh7seg = ch7seg;

        /* Display sensors on OLED every 200ms only when ch7seg != '0' */
        if ((msTicks - lastOledUpdate) >= 200) {
            lastOledUpdate = msTicks;

            if (ch7seg != '0') {
                /* Read sensors */
                temp = temp_read();
                lux = light_read();

                /* Read potentiometer if conversion is ready (non-blocking) */
                if (ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)) {
                    trim = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
                }

                /* Display on OLED */
                oled_clearScreen(OLED_COLOR_WHITE);
                oled_putString(1, 5, (uint8_t*)"Temp[C]: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                intToString(temp, buf, 10, 10);
                oled_putString(60, 5, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

                oled_putString(1, 20, (uint8_t*)"Light: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                intToString(lux, buf, 10, 10);
                oled_putString(60, 20, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

                oled_putString(1, 35, (uint8_t*)"Trim: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                intToString(trim, buf, 10, 10);
                oled_putString(60, 35, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            }
        }

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