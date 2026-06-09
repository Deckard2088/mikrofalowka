/*****************************************************************************
 * A demo example using several of the peripherals on the base board
 *
 * Copyright(C) 2010, Embedded Artists AB
 * All rights reserved.
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_i2c.h"

#include "rotary.h"
#include "led7seg.h"
#include "rgb.h"
#include "oled.h"
#include "temp.h"
#include "light.h"

#define AUTO_DECAY_INTERVAL_MS 5000

#define BUZZER_PIN_HIGH() GPIO_SetValue(0, 1<<26)
#define BUZZER_PIN_LOW()  GPIO_ClearValue(0, 1<<26)

#define MQ135_DOUT_PORT 0
#define MQ135_DOUT_PIN  16

/* =========================================================================
 * KONFIGURACJA PINÓW HC-SR04
 * ========================================================================= */
#define HCSR04_TRIG_PORT 2
#define HCSR04_TRIG_PIN  1
#define HCSR04_ECHO_PORT 2
#define HCSR04_ECHO_PIN  0

/* =========================================================================
 * KONFIGURACJA PINU DHT11
 * ========================================================================= */
#define DHT11_PORT 0
#define DHT11_PIN  21  

/* =========================================================================
 * KONFIGURACJA PINU DLA SILNICZKA (PIO3_0 -> Port 3, Pin 0)
 * ========================================================================= */
#define MOTOR_PORT 3
#define MOTOR_PIN  0

static uint8_t ch7seg = '0';
static uint8_t buf[10];
static uint32_t msTicks = 0;

/* =========================================================================
 * FUNKCJE OPÓŹNIAJĄCE
 * ========================================================================= */
static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 30; 
    while(count--) {
        __NOP(); 
    }
}

static void delay_ms(uint32_t ms)
{
    while(ms--) {
        delay_us(1000);
    }
}

static uint8_t rotate7SegChar(uint8_t ch)
{
    switch (ch) {
    case '0': return '0';
    case '1': return '|';
    case '2': return '2';
    case '3': return 'E';
    case '4': return 'h';
    case '5': return '5';
    case '6': return '9';
    case '7': return 'L';
    case '8': return '8';
    case '9': return '6';
    default:  return ch;
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

static void init_i2c(void)
{
    PINSEL_CFG_Type PinCfg;

    // Konfiguracja SDA (P0.10)
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    
    // Konfiguracja SCL (P0.11) - POPRAWIONO BŁĄD PRZYPISANIA PORTU
    PinCfg.Pinnum = 11;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);

    I2C_Init(LPC_I2C2, 100000);
    I2C_Cmd(LPC_I2C2, ENABLE);
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

    pinCfg.Portnum = MQ135_DOUT_PORT;
    pinCfg.Pinnum = MQ135_DOUT_PIN;
    PINSEL_ConfigPin(&pinCfg);

    // Konfiguracja pinu silniczka PIO3_0
    pinCfg.Portnum = MOTOR_PORT;
    pinCfg.Pinnum = MOTOR_PIN;
    PINSEL_ConfigPin(&pinCfg);

    GPIO_SetDir(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN, 1); 
    GPIO_SetDir(HCSR04_ECHO_PORT, 1U << HCSR04_ECHO_PIN, 0); 

    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);

    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 0);
    GPIO_SetDir(MQ135_DOUT_PORT, 1U << MQ135_DOUT_PIN, 0);

    // Ustawienie pinu silniczka jako WYJŚCIE i wyłączenie go na start
    GPIO_SetDir(MOTOR_PORT, 1U << MOTOR_PIN, 1);
    GPIO_ClearValue(MOTOR_PORT, 1U << MOTOR_PIN);
}

static int wait_for_level(uint8_t port, uint8_t pin, uint8_t level, uint32_t timeoutUs)
{
    uint32_t elapsed = 0;
    uint32_t mask = 1U << pin;

    while (((GPIO_ReadValue(port) & mask) != 0) != (level != 0)) {
        if (elapsed >= timeoutUs) {
            return -1;
        }
        delay_us(1); 
        elapsed++;
    }
    return 0;
}

static uint32_t hcsr04_read_cm(void)
{
    uint32_t durationUs = 0;
    uint32_t timeoutLimit = 15000; 

    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);
    delay_us(2); 
    GPIO_SetValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);
    delay_us(10); 
    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);

    if (wait_for_level(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN, 1, 5000) != 0) {
        return 0;
    }

    while ((GPIO_ReadValue(HCSR04_ECHO_PORT) & (1U << HCSR04_ECHO_PIN)) != 0) {
        if (durationUs >= timeoutLimit) {
            return 0;
        }
        delay_us(1); 
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

int main(void)
{
    uint8_t rotaryState = ROTARY_WAIT;
    uint32_t lastDecayTime = 0;
    uint32_t lastDhtTime = 0;

    uint8_t lastCh7seg = '0';

    uint8_t dhtHum = 0;
    int32_t temp10 = 0;
    uint32_t distanceCm = 0;
    uint32_t airDigital = 0;
    uint32_t lux = 0;

    init_ssp();
    init_i2c();

    rotary_init();
    led7seg_init();
    rgb_init();
    init_buzzer();
    oled_init();
    init_sensor_gpio();
    temp_init(&getTicks);
    light_init();

    if (SysTick_Config(SystemCoreClock / 1000))
    {
        while (1);
    }

    oled_clearScreen(OLED_COLOR_WHITE);
    light_enable();
    light_setRange(LIGHT_RANGE_4000);

    lastDecayTime = msTicks;
    lastDhtTime = msTicks;

    refreshOutputs();

    uint32_t lastTaskTime = 0;
    uint8_t programStep = 0;

    while (1)
    {
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

         if ((msTicks - lastDecayTime) >= AUTO_DECAY_INTERVAL_MS)
        {
            lastDecayTime = msTicks;

            if (ch7seg > '0')
            {
                ch7seg--;
                refreshOutputs();
            }
        }

        if (ch7seg == '0' && lastCh7seg != '0')
        {
            oled_clearScreen(OLED_COLOR_WHITE); 
            buzzerZeroPulse();
        }

        lastCh7seg = ch7seg;

        /* =========================================================================
         * REAKCJA SILNICZKA NA STAN WYŚWIETLACZA 7-SEGMENTOWEGO
         * ========================================================================= */
        if (ch7seg == '0')
        {
            GPIO_ClearValue(MOTOR_PORT, 1U << MOTOR_PIN); // Silnik STOP
        }
        else
        {
            GPIO_SetValue(MOTOR_PORT, 1U << MOTOR_PIN);  // Silnik START
        }

        /* =========================================================================
         * MASZYNA STANÓW
         * ========================================================================= */
        if ((msTicks - lastTaskTime) >= 200)
        {
            lastTaskTime = msTicks;

            switch (programStep)
            {
                case 0:
                    airDigital = (GPIO_ReadValue(MQ135_DOUT_PORT) & (1U << MQ135_DOUT_PIN)) ? 1U : 0U;
                    programStep = 1; 
                    break;

                case 1:
                    temp10 = temp_read();

                    if ((msTicks - lastDhtTime) >= 1200) {
                        int8_t status = dht11_read(NULL, &dhtHum);
                        if (status == 0) {
                            lastDhtTime = msTicks;
                        } else {
                            dhtHum = 99; 
                        }
                    }
                    programStep = 2; 
                    break;

                case 2:
                    lux = light_read();
                    programStep = 3; 
                    break;

                case 3:
                    distanceCm = hcsr04_read_cm();
                    programStep = 4; 
                    break;

                case 4:
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
                    programStep = 5;
                    break;

                case 5:
                    oled_putString(1, 20, (uint8_t*)"U: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    intToString(distanceCm, buf, 10, 10);
                    oled_putString(20, 20, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    oled_putString(50, 20, (uint8_t*)"cm   ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    programStep = 6;
                    break;

                case 6:
                    oled_putString(1, 35, (uint8_t*)"A: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    intToString(airDigital, buf, 10, 10);
                    oled_putString(20, 35, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    oled_putString(60, 35, (uint8_t*)"   ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    programStep = 7;
                    break;

                case 7:
                    oled_putString(1, 50, (uint8_t*)"L: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    intToString(lux, buf, 10, 10);
                    oled_putString(20, 50, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    oled_putString(60, 50, (uint8_t*)"lx ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    programStep = 0; 
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
    while(1);
}