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
#include "lpc17xx_adc.h"

#include "rotary.h"
#include "led7seg.h"
#include "rgb.h"
#include "oled.h"
#include "temp.h"

#define AUTO_DECAY_INTERVAL_MS 5000
#define BUZZER_PIN_HIGH() GPIO_SetValue(0, 1<<26)
#define BUZZER_PIN_LOW()  GPIO_ClearValue(0, 1<<26)


#define MQ135_ADC_CHANNEL ADC_CHANNEL_0

#define HCSR04_TRIG_PORT 2
#define HCSR04_TRIG_PIN  10
#define HCSR04_ECHO_PORT 0
#define HCSR04_ECHO_PIN  4

#define DHT11_PORT 0
#define DHT11_PIN  12

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


static void init_sensor_gpio(void)
{
    PINSEL_CFG_Type pinCfg;

    pinCfg.Funcnum = 0;
    pinCfg.OpenDrain = 0;
    pinCfg.Pinmode = 0;

    pinCfg.Portnum = HCSR04_TRIG_PORT;
    pinCfg.Pinnum = HCSR04_TRIG_PIN;
    PINSEL_ConfigPin(&pinCfg);

    pinCfg.Portnum = HCSR04_ECHO_PORT;
    pinCfg.Pinnum = HCSR04_ECHO_PIN;
    PINSEL_ConfigPin(&pinCfg);

    pinCfg.Portnum = DHT11_PORT;
    pinCfg.Pinnum = DHT11_PIN;
    PINSEL_ConfigPin(&pinCfg);

    GPIO_SetDir(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN, 1);
    GPIO_SetDir(HCSR04_ECHO_PORT, 1U << HCSR04_ECHO_PIN, 0);

    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);

    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 0);
}

static int wait_for_level(uint8_t port, uint8_t pin, uint8_t level, uint32_t timeoutUs)
{
    uint32_t elapsed = 0;
    uint32_t mask = 1U << pin;

    while (((GPIO_ReadValue(port) & mask) != 0) != (level != 0)) {
        if (elapsed >= timeoutUs) {
            return -1;
        }
        Timer0_us_Wait(1);
        elapsed++;
    }

    return 0;
}

static int dht11_read(uint8_t* tempC, uint8_t* humidity)
{
    uint8_t data[5] = {0};
    uint32_t i = 0;
    uint32_t bitIndex = 0;

    if (humidity == NULL) {
        return -1;
    }

    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 1);
    GPIO_ClearValue(DHT11_PORT, 1U << DHT11_PIN);
    Timer0_Wait(20);

    GPIO_SetValue(DHT11_PORT, 1U << DHT11_PIN);
    Timer0_us_Wait(30);
    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 0);

    if (wait_for_level(DHT11_PORT, DHT11_PIN, 0, 100) != 0) {
        return -1;
    }
    if (wait_for_level(DHT11_PORT, DHT11_PIN, 1, 100) != 0) {
        return -1;
    }
    if (wait_for_level(DHT11_PORT, DHT11_PIN, 0, 100) != 0) {
        return -1;
    }

    for (i = 0; i < 40; i++) {
        if (wait_for_level(DHT11_PORT, DHT11_PIN, 1, 100) != 0) {
            return -1;
        }

        Timer0_us_Wait(40);
        bitIndex = i / 8;
        if ((GPIO_ReadValue(DHT11_PORT) & (1U << DHT11_PIN)) != 0) {
            data[bitIndex] |= (1U << (7 - (i % 8)));
        }

        if (wait_for_level(DHT11_PORT, DHT11_PIN, 0, 100) != 0) {
            return -1;
        }
    }

    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) {
        return -1;
    }

    *humidity = data[0];
    if (tempC != NULL) {
        *tempC = data[2];
    }

    return 0;
}

static uint32_t hcsr04_read_cm(void)
{
    uint32_t durationUs = 0;
    uint32_t maxWaitUs = 30000;

    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);
    Timer0_us_Wait(2);
    GPIO_SetValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);
    Timer0_us_Wait(10);
    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);

    if (wait_for_level(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN, 1, maxWaitUs) != 0) {
        return 0;
    }

    while ((GPIO_ReadValue(HCSR04_ECHO_PORT) & (1U << HCSR04_ECHO_PIN)) != 0) {
        if (durationUs >= maxWaitUs) {
            break;
        }
        Timer0_us_Wait(1);
        durationUs++;
    }

    return durationUs / 58U;
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


int main(void)
{
    uint8_t rotaryState = ROTARY_WAIT;
    uint32_t lastDecayTime = 0;
    uint32_t lastDhtTime = 0;

    uint8_t lastCh7seg = '0';

    uint8_t dhtHum = 0;
    int32_t temp10 = 0;
    uint32_t distanceCm = 0;
    uint32_t airRaw = 0;

    init_ssp();
    init_adc();

    rotary_init();
    led7seg_init();
    rgb_init();
    init_buzzer();
    oled_init();
    init_sensor_gpio();
    temp_init(&getTicks);

    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1);
    }

    oled_clearScreen(OLED_COLOR_WHITE);

    lastDecayTime = msTicks;
    lastDhtTime = msTicks;

    refreshOutputs();

    /* Dodatkowe zmienne sterujące przed pętlą while(1) */
        uint32_t lastTaskTime = 0;
        uint8_t programStep = 0;

        while (1)
        {
            /* =========================================================================
             * KROK INTERWENCYJNY: ROTARY (Enkoder musi być czytany non-stop, bez czekania!)
             * ========================================================================= */
            rotaryState = rotary_read();

            if (rotaryState == ROTARY_RIGHT)
            {
                if (ch7seg >= '9') ch7seg = '0';
                else ch7seg++;

                refreshOutputs();
                lastDecayTime = msTicks;
            }
            else if (rotaryState == ROTARY_LEFT)
            {
                if (ch7seg <= '0') ch7seg = '9';
                else ch7seg--;

                refreshOutputs();
                lastDecayTime = msTicks;
            }

            /* =========================================================================
             * AUTO DECAY (Niezmienione - co 5 sekund)
             * ========================================================================= */
            if ((msTicks - lastDecayTime) >= AUTO_DECAY_INTERVAL_MS)
            {
                lastDecayTime = msTicks;

                if (ch7seg > '0')
                {
                    ch7seg--;
                    refreshOutputs();
                }
            }

            /* =========================================================================
             * ZERO EVENT
             * ========================================================================= */
            if (ch7seg == '0' && lastCh7seg != '0')
            {
                buzzerZeroPulse();
            }

            lastCh7seg = ch7seg;


            /* =========================================================================
             * MASZYNA STANÓW (Time-Slicing)
             * Co 200 ms wykonuje się TYLKO JEDNA ciężka operacja, a nie wszystkie na raz!
             * ========================================================================= */
            if ((msTicks - lastTaskTime) >= 200)
            {
                lastTaskTime = msTicks;

                switch (programStep)
                {
                    case 0:
                        /* Krok 0: Odczyt MQ-135 (ADC) */
                        ADC_StartCmd(LPC_ADC, ADC_START_NOW);
                        while(!ADC_ChannelGetStatus(LPC_ADC, MQ135_ADC_CHANNEL, ADC_DATA_DONE));
                        airRaw = ADC_ChannelGetData(LPC_ADC, MQ135_ADC_CHANNEL);

                        programStep = 1; /* Przejdź do czytania temperatury w następnej iteracji */
                        break;

                    case 1:
                        /* Krok 1: Odczyt temperatury z czujnika na plytce */
                        temp10 = temp_read();

                        /* Odczyt DHT11 (max ok. 1 Hz) */
                        if ((msTicks - lastDhtTime) >= 1200) {
                            if (dht11_read(NULL, &dhtHum) == 0) {
                                lastDhtTime = msTicks;
                            }
                        }

                        programStep = 2; /* Przejdź do światła */
                        break;

                    case 2:
                        /* Krok 2: Odczyt HC-SR04 */
                        distanceCm = hcsr04_read_cm();

                        programStep = 3; /* Przejdź do aktualizacji tekstu na OLED */
                        break;

                    case 3:
                        /* Krok 3: Wysłanie danych tekstowych na ekran OLED */
                        {
                            int32_t tempAbs = temp10;

                            if (tempAbs < 0) {
                                tempAbs = -tempAbs;
                                oled_putString(1, 5,  (uint8_t*)"T:-", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                            }
                            else {
                                oled_putString(1, 5,  (uint8_t*)"T: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                            }

                            intToString(tempAbs / 10, buf, 10, 10);
                            oled_putString(20, 5, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                            oled_putString(38, 5,  (uint8_t*)".", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                            intToString(tempAbs % 10, buf, 10, 10);
                            oled_putString(44, 5, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                            oled_putString(52, 5,  (uint8_t*)"C H: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                            intToString(dhtHum, buf, 10, 10);
                            oled_putString(78, 5, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                            oled_putString(90, 5,  (uint8_t*)"% ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                        }

                        programStep = 4;
                        break;

                    case 4:
                        /* Krok 4: Kolejna część OLED */
                        oled_putString(1, 20, (uint8_t*)"U: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                        intToString(distanceCm, buf, 10, 10);
                        oled_putString(20, 20, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                        oled_putString(60, 20, (uint8_t*)"cm ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

                        programStep = 5;
                        break;

                    case 5:
                        /* Krok 5: Ostatnia część OLED */
                        oled_putString(1, 35, (uint8_t*)"A: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                        intToString(airRaw, buf, 10, 10);
                        oled_putString(20, 35, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                        oled_putString(60, 35, (uint8_t*)"   ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

                        programStep = 0; /* Powrót do początku układanki */
                        break;

                    default:
                        programStep = 0;
                        break;
                }
            }
        }
}


void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
