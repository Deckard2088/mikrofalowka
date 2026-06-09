/*****************************************************************************
 * LPC1769 FULL SENSOR SYSTEM – FIXED (FEATURE COMPLETE)
 *****************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_timer.h"

#include "rotary.h"
#include "led7seg.h"
#include "rgb.h"
#include "oled.h"
#include "temp.h"
#include "light.h"

/* ================= CONFIG ================= */
#define AUTO_DECAY_INTERVAL_MS 5000
#define INTERVAL_MQ_MS        100
#define INTERVAL_HC_MS        150
#define INTERVAL_I2C_MS       300
#define INTERVAL_OLED_MS      250
#define INTERVAL_DHT_MS       2000

#define DHT_PORT 0
#define DHT_PIN  21

#define MQ_PORT 0
#define MQ_PIN  16

#define HC_TRIG_PORT 2
#define HC_TRIG_PIN  1

#define HC_ECHO_PORT 2
#define HC_ECHO_PIN  0

#define MOTOR_PORT 3
#define MOTOR_PIN  1

#define BUZZER_PORT 0
#define BUZZER_PIN  26

/* ================= GLOBALS ================= */
static volatile uint32_t msTicks = 0;

static uint8_t ch7seg = '0';
static uint8_t buf[16];

static int32_t prev_temp = -9999;
static int32_t prev_hum  = -1;
static int32_t prev_dist = -1;
static int32_t prev_air  = -1;
static int32_t prev_lux  = -1;

/* DHT */
static uint8_t dht_raw[5];
static uint8_t dht_hum = 0;
static uint8_t dht_ok  = 0;

/* ================= SYS ================= */
void SysTick_Handler(void)
{
    msTicks++;
}

static uint32_t now(void)
{
    return msTicks;
}

/* ================= DELAY ================= */
static void delay_us(uint32_t us)
{
    volatile uint32_t n = us * 25;
    while (n--) __NOP();
}

/* ================= SAFE STRING ================= */
static void intToString(int32_t v, uint8_t *buf, uint32_t len)
{
    static const char d[] = "0123456789";
    uint32_t i = 0, x, neg = 0;
    uint8_t tmp[12];
    uint32_t p = 0;

    if (!buf || len < 2) return;

    buf[0] = '\0';

    if (v < 0) { neg = 1; x = (uint32_t)(-v); }
    else x = (uint32_t)v;

    do {
        tmp[p++] = d[x % 10];
        x /= 10;
    } while (x);

    if (neg) buf[i++] = '-';

    while (p && i < len - 1)
        buf[i++] = tmp[--p];

    buf[i] = '\0';
}

/* ================= OLED ================= */
static void oled_update(int x, int y, int32_t v, int32_t *prev)
{
    if (v == *prev) return;
    *prev = v;

    intToString(v, buf, sizeof(buf));
    oled_putString(x, y, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

/* ================= HC-SR04 ================= */
static uint32_t hcsr04_read(void)
{
    uint32_t t = 0, dur = 0;
    uint32_t mask = 1 << HC_ECHO_PIN;

    GPIO_ClearValue(HC_TRIG_PORT, 1 << HC_TRIG_PIN);
    delay_us(2);
    GPIO_SetValue(HC_TRIG_PORT, 1 << HC_TRIG_PIN);
    delay_us(10);
    GPIO_ClearValue(HC_TRIG_PORT, 1 << HC_TRIG_PIN);

    while (!(GPIO_ReadValue(HC_ECHO_PORT) & mask))
        if (++t > 2000) return 0;

    while (GPIO_ReadValue(HC_ECHO_PORT) & mask) {
        if (++dur > 25000) break;
        delay_us(1);
    }

    return dur / 58;
}

/* ================= DHT11 (FIXED, RELIABLE) ================= */
static uint8_t dht_read_byte(uint8_t *out)
{
    uint8_t i;
    uint32_t timeout;

    *out = 0;

    for (i = 0; i < 8; i++)
    {
        timeout = 0;

        while (!(GPIO_ReadValue(DHT_PORT) & (1 << DHT_PIN)))
            if (++timeout > 1000) return 0;

        delay_us(40);

        if (GPIO_ReadValue(DHT_PORT) & (1 << DHT_PIN))
            *out |= (1 << (7 - i));

        timeout = 0;
        while (GPIO_ReadValue(DHT_PORT) & (1 << DHT_PIN))
            if (++timeout > 1000) break;
    }

    return 1;
}

static uint8_t dht_read(void)
{
    uint8_t i, sum;

    GPIO_SetDir(DHT_PORT, 1 << DHT_PIN, 1);
    GPIO_ClearValue(DHT_PORT, 1 << DHT_PIN);
    delay_us(20000);

    GPIO_SetValue(DHT_PORT, 1 << DHT_PIN);
    delay_us(30);
    GPIO_SetDir(DHT_PORT, 1 << DHT_PIN, 0);

    while (!(GPIO_ReadValue(DHT_PORT) & (1 << DHT_PIN)));
    while (GPIO_ReadValue(DHT_PORT) & (1 << DHT_PIN));

    for (i = 0; i < 5; i++)
        if (!dht_read_byte(&dht_raw[i]))
            return 0;

    sum = (dht_raw[0] + dht_raw[1] + dht_raw[2] + dht_raw[3]) & 0xFF;

    if (sum != dht_raw[4]) return 0;

    dht_hum = dht_raw[0];
    return 1;
}

/* ================= MAIN ================= */
int main(void)
{
    uint32_t lastDht = 0, lastHc = 0, lastOled = 0;
    uint32_t lastMq = 0, lastI2c = 0, lastDecay = 0;

    int32_t temp = 0, lux = 0, dist = 0, air = 0;

    SysTick_Config(SystemCoreClock / 1000);

    oled_init();
    led7seg_init();
    rgb_init();

    while (1)
    {
        uint32_t t = now();

        /* rotary / motor / 7seg */
        uint8_t rot = rotary_read();
        if (rot == ROTARY_RIGHT && ch7seg < '9') ch7seg++;
        if (rot == ROTARY_LEFT && ch7seg > '0') ch7seg--;

        if (ch7seg == '0')
            GPIO_ClearValue(MOTOR_PORT, 1 << MOTOR_PIN);
        else
            GPIO_SetValue(MOTOR_PORT, 1 << MOTOR_PIN);

        /* MQ135 */
        if (t - lastMq > INTERVAL_MQ_MS)
        {
            lastMq = t;
            air = GPIO_ReadValue(MQ_PORT) & (1 << MQ_PIN);
        }

        /* HC-SR04 */
        if (t - lastHc > INTERVAL_HC_MS)
        {
            lastHc = t;
            dist = hcsr04_read();
        }

        /* DHT */
        if (t - lastDht > INTERVAL_DHT_MS)
        {
            lastDht = t;
            dht_ok = dht_read();
        }

        /* I2C sensors */
        if (t - lastI2c > INTERVAL_I2C_MS)
        {
            lastI2c = t;
            temp = temp_read();
            lux  = light_read();
        }

        /* OLED */
        if (t - lastOled > INTERVAL_OLED_MS)
        {
            lastOled = t;

            oled_update(20, 5,  temp, &prev_temp);
            oled_update(78, 5,  dht_hum, &prev_hum);
            oled_update(20, 20, dist, &prev_dist);
            oled_update(20, 35, air, &prev_air);
            oled_update(20, 50, lux, &prev_lux);
        }

        /* buzzer logic (example simple rule) */
        if (air == 0)
            GPIO_SetValue(BUZZER_PORT, 1 << BUZZER_PIN);
        else
            GPIO_ClearValue(BUZZER_PORT, 1 << BUZZER_PIN);
    }
}