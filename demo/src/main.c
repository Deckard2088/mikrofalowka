/*****************************************************************************
 * Demo z czujnikami – wersja nieblokująca dla LPC1769
 *
 * Zmiany względem oryginału:
 *  1. DHT11 przepisany jako maszyna stanów (bez delay_ms(30) w pętli głównej)
 *  2. Buforowane wartości OLED – odświeżamy tylko zmienione pola
 *  3. HC-SR04: timeout skrócony, nie blokuje dłużej niż ~6 ms
 *  4. Enkoder odczytywany priorytetowo na początku każdej iteracji
 *  5. Wszystkie opóźnienia > 1 ms zastąpione sprawdzaniem msTicks
 *
 * Copyright(C) 2010, Embedded Artists AB  (oryginał)
 * Modyfikacje: optymalizacja pod LPC1769 @ 120 MHz
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

/* =========================================================================
 * STAŁE CZASOWE (ms)
 * ========================================================================= */
#define AUTO_DECAY_INTERVAL_MS  5000
#define INTERVAL_MQ_MS           100
#define INTERVAL_HC_MS           150
#define INTERVAL_I2C_MS          300
#define INTERVAL_OLED_MS         250
#define INTERVAL_DHT_MS         2000

/* =========================================================================
 * PINY
 * ========================================================================= */
#define BUZZER_PIN_HIGH()   GPIO_SetValue(0, 1<<26)
#define BUZZER_PIN_LOW()    GPIO_ClearValue(0, 1<<26)

#define MQ135_DOUT_PORT     0
#define MQ135_DOUT_PIN      16

#define HCSR04_TRIG_PORT    2
#define HCSR04_TRIG_PIN     1
#define HCSR04_ECHO_PORT    2
#define HCSR04_ECHO_PIN     0

#define DHT11_PORT          0
#define DHT11_PIN           21

#define MOTOR_PORT          3
#define MOTOR_PIN           1

/* =========================================================================
 * MASZYNA STANÓW DHT11
 *   Zamiast blokującego delay_ms(30) używamy msTicks do odmierzania czasu.
 *   Każde wywołanie dht11_fsm_tick() wraca natychmiast i przesuwa stan tylko
 *   wtedy, gdy minął wymagany czas lub spełniony jest warunek GPIO.
 * ========================================================================= */
typedef enum {
    DHT_IDLE = 0,
    DHT_PULL_LOW,       /* trzymamy linię LOW przez 30 ms */
    DHT_PULL_HIGH,      /* puszczamy linię (wejście) */
    DHT_WAIT_RESP_LOW,  /* czekamy na LOW od sensora (ACK start) */
    DHT_WAIT_RESP_HIGH, /* czekamy na HIGH od sensora (ACK end)   */
    DHT_WAIT_BIT_START, /* czekamy na LOW kończący sygnał HIGH bitu */
    DHT_SAMPLE_BIT,     /* próbkujemy bit po ~40 µs od jego startu  */
    DHT_DONE
} dht_state_t;

static dht_state_t  dht_state       = DHT_IDLE;
static uint32_t     dht_state_ts    = 0;   /* msTicks przy wejściu w stan    */
static uint8_t      dht_raw[5]      = {0};
static uint8_t      dht_bit_idx     = 0;
static uint32_t     dht_bit_high_ts = 0;   /* msTicks gdy bit-HIGH się zaczął */

/* Wynikowe wartości (aktualizowane po poprawnym odczycie) */
static uint8_t  dht_hum     = 0;
static uint8_t  dht_ok      = 0;   /* 1 = ostatni odczyt poprawny */

/* =========================================================================
 * ZMIENNE GLOBALNE
 * ========================================================================= */
static uint8_t  ch7seg   = '0';
static uint8_t  buf[10];
static uint32_t msTicks  = 0;

/* Poprzednie wartości wyświetlone na OLED – odświeżamy tylko przy zmianie */
static int32_t  prev_temp10    = -9999;
static uint8_t  prev_dhtHum    = 255;
static uint32_t prev_dist      = 0xFFFFFFFF;
static uint32_t prev_air       = 0xFFFFFFFF;
static uint32_t prev_lux       = 0xFFFFFFFF;

/* =========================================================================
 * DELAY KRÓTKIE (tylko do inicjalizacji i bit-bang, gdzie < 1 ms)
 * ========================================================================= */
static void delay_us(uint32_t us)
{
    /* ~30 cykli na µs przy 120 MHz – używaj tylko tam gdzie MUSI być busy-wait */
    volatile uint32_t n = us * 30;
    while (n--) { __NOP(); }
}

/* =========================================================================
 * ROTARY / LED7SEG / RGB
 * ========================================================================= */
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
    rgb_setLeds(ch7seg == '0' ? 0 : RGB_GREEN);
}

/* =========================================================================
 * HC-SR04 – nieblokujący (max ~6 ms busy-wait, akceptowalne co 150 ms)
 * ========================================================================= */
static uint32_t hcsr04_read_cm(void)
{
    uint32_t dur = 0;
    uint32_t mask = 1U << HCSR04_ECHO_PIN;

    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);
    delay_us(2);
    GPIO_SetValue  (HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);
    delay_us(10);
    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);

    /* Czekaj na zbocze narastające (timeout 1500 µs) */
    uint32_t t = 0;
    while (!(GPIO_ReadValue(HCSR04_ECHO_PORT) & mask)) {
        if (++t > 1500) return 0;
        delay_us(1);
    }

    /* Mierz czas trwania pulsu ECHO (timeout 6000 µs ≈ 103 cm) */
    while (GPIO_ReadValue(HCSR04_ECHO_PORT) & mask) {
        if (dur >= 6000) return 0;
        delay_us(1);
        dur++;
    }
    return dur / 58U;
}

/* =========================================================================
 * DHT11 – MASZYNA STANÓW (tick wołany z pętli głównej)
 *
 *  Wywołaj dht11_fsm_trigger() co INTERVAL_DHT_MS, żeby uruchomić pomiar.
 *  Wywołuj dht11_fsm_tick() w każdej iteracji pętli głównej.
 *  Wyniki dostępne przez dht_hum (dht_ok == 1 gdy poprawne).
 * ========================================================================= */
void dht11_fsm_trigger(void)
{
    if (dht_state != DHT_IDLE) return;  /* poprzedni odczyt jeszcze trwa */

    /* Ustaw linię jako wyjście, pociągnij LOW */
    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 1);
    GPIO_ClearValue(DHT11_PORT, 1U << DHT11_PIN);

    dht_bit_idx = 0;
    dht_raw[0] = dht_raw[1] = dht_raw[2] = dht_raw[3] = dht_raw[4] = 0;
    dht_state    = DHT_PULL_LOW;
    dht_state_ts = msTicks;
}

void dht11_fsm_tick(void)
{
    uint32_t mask = 1U << DHT11_PIN;

    switch (dht_state) {

    case DHT_IDLE:
        break;

    case DHT_PULL_LOW:
        /* Trzymamy LOW przez 30 ms – sprawdzamy msTicks, nie blokujemy */
        if ((msTicks - dht_state_ts) >= 30) {
            /* Teraz daj HIGH na ~30 µs (busy-wait, krótkie) */
            GPIO_SetValue(DHT11_PORT, 1U << DHT11_PIN);
            delay_us(30);
            /* Przełącz na wejście */
            GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 0);
            dht_state = DHT_WAIT_RESP_LOW;
        }
        break;

    case DHT_WAIT_RESP_LOW:
        /* Czekaj aż sensor pociągnie LOW (ACK start) */
        if (!(GPIO_ReadValue(DHT11_PORT) & mask)) {
            dht_state    = DHT_WAIT_RESP_HIGH;
            dht_state_ts = msTicks;
        } else if ((msTicks - dht_state_ts) > 5) {
            /* Timeout – sensor nie odpowiedział */
            dht_state = DHT_IDLE;
        }
        break;

    case DHT_WAIT_RESP_HIGH:
        /* Czekaj aż sensor puści HIGH (ACK end) */
        if (GPIO_ReadValue(DHT11_PORT) & mask) {
            dht_state    = DHT_WAIT_BIT_START;
            dht_state_ts = msTicks;
        } else if ((msTicks - dht_state_ts) > 5) {
            dht_state = DHT_IDLE;
        }
        break;

    case DHT_WAIT_BIT_START:
        /* Czekaj na LOW – to początek nowego bitu (opada zbocze po HIGH) */
        if (!(GPIO_ReadValue(DHT11_PORT) & mask)) {
            /* LOW = bit-start. Teraz czekamy ~50 µs (LOW) a potem HIGH */
            delay_us(60);   /* przeskocz LOW + trochę HIGH bitu */
            /* Próbkuj teraz – jeśli nadal HIGH to bit=1, LOW to bit=0 */
            dht_state = DHT_SAMPLE_BIT;
        } else if ((msTicks - dht_state_ts) > 10) {
            dht_state = DHT_IDLE;
        }
        break;

    case DHT_SAMPLE_BIT:
        {
            uint8_t byteIdx = dht_bit_idx / 8;
            uint8_t bitPos  = 7 - (dht_bit_idx % 8);

            if (GPIO_ReadValue(DHT11_PORT) & mask) {
                dht_raw[byteIdx] |= (1U << bitPos);  /* bit = 1 */
            }
            /* Poczekaj na koniec HIGH tego bitu zanim przejdziemy dalej */
            uint32_t safety = 0;
            while ((GPIO_ReadValue(DHT11_PORT) & mask) && safety < 200) {
                delay_us(1);
                safety++;
            }

            dht_bit_idx++;
            if (dht_bit_idx >= 40) {
                dht_state = DHT_DONE;
            } else {
                dht_state    = DHT_WAIT_BIT_START;
                dht_state_ts = msTicks;
            }
        }
        break;

    case DHT_DONE:
        /* Weryfikacja sumy kontrolnej */
        if (((dht_raw[0] + dht_raw[1] + dht_raw[2] + dht_raw[3]) & 0xFF) == dht_raw[4]) {
            dht_hum = dht_raw[0];
            dht_ok  = 1;
        } else {
            dht_ok = 0;
        }
        /* Wróć linię na wejście i zresetuj stan */
        GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 0);
        dht_state = DHT_IDLE;
        break;
    }
}

/* =========================================================================
 * POMOCNICZA: int → string
 * ========================================================================= */
static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base)
{
    static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
    int pos = 0, tmpValue = value;
    if (!pBuf || len < 2 || base < 2 || base > 36) return;
    if (value < 0) { tmpValue = -tmpValue; value = -value; pBuf[pos++] = '-'; }
    do { pos++; tmpValue /= base; } while (tmpValue > 0);
    if (pos > (int)len) return;
    pBuf[pos] = '\0';
    do { pBuf[--pos] = pAscii[value % base]; value /= base; } while (value > 0);
}

/* =========================================================================
 * POMOCNICZA: wyświetl pole OLED tylko gdy wartość się zmieniła
 * ========================================================================= */
static void oled_update_field(uint8_t x, uint8_t y,
                              int value, int* prev,
                              const char* suffix)
{
    if (value == *prev) return;
    *prev = value;
    intToString(value, buf, 10, 10);
    oled_putString(x, y, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    if (suffix) {
        oled_putString(x + 18, y, (uint8_t*)suffix, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    }
}

/* =========================================================================
 * INIT
 * ========================================================================= */
static void init_buzzer(void)
{
    GPIO_SetDir(0, 1<<27, 1); GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1); GPIO_SetDir(0, 1<<26, 1);
    GPIO_ClearValue(0, 1<<27); GPIO_ClearValue(0, 1<<28);
    GPIO_ClearValue(2, 1<<13); BUZZER_PIN_LOW();
}

static void init_ssp(void)
{
    SSP_CFG_Type SSP_ConfigStruct;
    PINSEL_CFG_Type PinCfg;

    PinCfg.Funcnum = 2; PinCfg.OpenDrain = 0; PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0; PinCfg.Pinnum = 7; PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 8; PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 9; PINSEL_ConfigPin(&PinCfg);
    PinCfg.Funcnum = 0; PinCfg.Portnum = 2; PinCfg.Pinnum = 2;
    PINSEL_ConfigPin(&PinCfg);

    SSP_ConfigStructInit(&SSP_ConfigStruct);
    SSP_Init(LPC_SSP1, &SSP_ConfigStruct);
    SSP_Cmd(LPC_SSP1, ENABLE);
}

static void init_i2c(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2; PinCfg.Portnum = 0;
    PinCfg.Pinnum = 10; PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11; PINSEL_ConfigPin(&PinCfg);
    I2C_Init(LPC_I2C2, 100000);
    I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_sensor_gpio(void)
{
    PINSEL_CFG_Type pinCfg;
    pinCfg.Funcnum = 0; pinCfg.OpenDrain = 0; pinCfg.Pinmode = 0;

    pinCfg.Portnum = HCSR04_TRIG_PORT; pinCfg.Pinnum = HCSR04_TRIG_PIN;
    PINSEL_ConfigPin(&pinCfg);
    pinCfg.Portnum = HCSR04_ECHO_PORT; pinCfg.Pinnum = HCSR04_ECHO_PIN;
    PINSEL_ConfigPin(&pinCfg);
    pinCfg.Portnum = DHT11_PORT;        pinCfg.Pinnum = DHT11_PIN;
    PINSEL_ConfigPin(&pinCfg);
    pinCfg.Portnum = MQ135_DOUT_PORT;  pinCfg.Pinnum = MQ135_DOUT_PIN;
    PINSEL_ConfigPin(&pinCfg);
    pinCfg.Portnum = MOTOR_PORT;        pinCfg.Pinnum = MOTOR_PIN;
    PINSEL_ConfigPin(&pinCfg);

    GPIO_SetDir(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN, 1);
    GPIO_SetDir(HCSR04_ECHO_PORT, 1U << HCSR04_ECHO_PIN, 0);
    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);

    GPIO_SetDir(DHT11_PORT,       1U << DHT11_PIN,       0);
    GPIO_SetDir(MQ135_DOUT_PORT,  1U << MQ135_DOUT_PIN,  0);

    GPIO_SetDir(MOTOR_PORT, 1U << MOTOR_PIN, 1);
    GPIO_ClearValue(MOTOR_PORT, 1U << MOTOR_PIN);
}

/* =========================================================================
 * SysTick
 * ========================================================================= */
static uint32_t getTicks(void) { return msTicks; }

void SysTick_Handler(void) { msTicks++; }

/* =========================================================================
 * PĘTLA GŁÓWNA
 * ========================================================================= */
int main(void)
{
    uint8_t  lastCh7seg  = '0';
    uint32_t lastDecay   = 0;
    uint32_t lastMq      = 0;
    uint32_t lastHc      = 0;
    uint32_t lastI2c     = 0;
    uint32_t lastOled    = 0;
    uint32_t lastDht     = 0;

    int32_t  temp10     = 0;
    uint32_t distanceCm = 0;
    uint32_t airDigital = 0;
    uint32_t lux        = 0;

    /* ---- Inicjalizacja ---- */
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

    if (SysTick_Config(SystemCoreClock / 1000)) { while (1); }

    oled_clearScreen(OLED_COLOR_WHITE);
    light_enable();
    light_setRange(LIGHT_RANGE_4000);

    lastDecay = lastMq = lastHc = lastI2c = lastOled = lastDht = msTicks;
    refreshOutputs();

    /* Etykiety stałe na OLED (rysujemy raz) */
    oled_putString(1,  5,  (uint8_t*)"T:       C H:     %", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,  20, (uint8_t*)"U:         cm",       OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,  35, (uint8_t*)"A:",                  OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,  50, (uint8_t*)"L:         lx",       OLED_COLOR_BLACK, OLED_COLOR_WHITE);

    /* ================================================================
     * PĘTLA GŁÓWNA
     * Każda iteracja zajmuje < 1 ms (z wyjątkiem HC-SR04 co 150 ms,
     * gdzie busy-wait wynosi max ~6 ms – akceptowalne przy tym interwale).
     * ================================================================ */
    while (1)
    {
        /* === 1. ENKODER (priorytet) ================================ */
        uint8_t rot = rotary_read();
        if (rot == ROTARY_RIGHT) {
            if (ch7seg < '9') ch7seg++;
            else              ch7seg = '0';
            refreshOutputs();
            lastDecay = msTicks;
        } else if (rot == ROTARY_LEFT) {
            if (ch7seg > '0') ch7seg--;
            else              ch7seg = '9';
            refreshOutputs();
            lastDecay = msTicks;
        }

        /* === 2. AUTO-DECAY 7seg ==================================== */
        if ((msTicks - lastDecay) >= AUTO_DECAY_INTERVAL_MS) {
            lastDecay = msTicks;
            if (ch7seg > '0') { ch7seg--; refreshOutputs(); }
        }

        /* === 3. SILNICZEK ========================================== */
        if (ch7seg == '0') GPIO_ClearValue(MOTOR_PORT, 1U << MOTOR_PIN);
        else               GPIO_SetValue  (MOTOR_PORT, 1U << MOTOR_PIN);

        /* Wyczyść OLED jeśli wróciło do 0 */
        if (ch7seg == '0' && lastCh7seg != '0') {
            oled_clearScreen(OLED_COLOR_WHITE);
            /* Przywróć stałe etykiety */
            oled_putString(1,  5,  (uint8_t*)"T:       C H:     %", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,  20, (uint8_t*)"U:         cm",       OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,  35, (uint8_t*)"A:",                  OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,  50, (uint8_t*)"L:         lx",       OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            /* Wymusz odświeżenie przy następnej aktualizacji */
            prev_temp10 = -9999; prev_dhtHum = 255;
            prev_dist = 0xFFFFFFFF; prev_air = 0xFFFFFFFF; prev_lux = 0xFFFFFFFF;
        }
        lastCh7seg = ch7seg;

        /* === 4. DHT11 – tick maszyny stanów (wołamy zawsze) ======= */
        dht11_fsm_tick();

        /* Wyzwól nowy pomiar co INTERVAL_DHT_MS i tylko gdy stan IDLE */
        if ((msTicks - lastDht) >= INTERVAL_DHT_MS && dht_state == DHT_IDLE) {
            lastDht = msTicks;
            dht11_fsm_trigger();
        }

        /* === 5. MQ-135 – co 100 ms ================================ */
        if ((msTicks - lastMq) >= INTERVAL_MQ_MS) {
            lastMq = msTicks;
            airDigital = (GPIO_ReadValue(MQ135_DOUT_PORT) & (1U << MQ135_DOUT_PIN)) ? 1U : 0U;
        }

        /* === 6. HC-SR04 – co 150 ms =============================== */
        if ((msTicks - lastHc) >= INTERVAL_HC_MS) {
            lastHc = msTicks;
            distanceCm = hcsr04_read_cm();
        }

        /* === 7. I2C (temp + light) – co 300 ms ==================== */
        if ((msTicks - lastI2c) >= INTERVAL_I2C_MS) {
            lastI2c = msTicks;
            temp10  = temp_read();
            lux     = light_read();
        }

        /* === 8. OLED – co 250 ms, tylko zmienione pola ============ */
        if ((msTicks - lastOled) >= INTERVAL_OLED_MS) {
            lastOled = msTicks;

            /* Temperatura: "T: XX.X C" */
            int32_t tAbs = (temp10 < 0) ? -temp10 : temp10;
            if (temp10 != prev_temp10) {
                prev_temp10 = temp10;
                oled_putString(1, 5, temp10 < 0 ? (uint8_t*)"T:-" : (uint8_t*)"T: ",
                               OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                intToString(tAbs / 10, buf, 10, 10);
                oled_putString(20, 5, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                oled_putString(38, 5, (uint8_t*)".", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                intToString(tAbs % 10, buf, 10, 10);
                oled_putString(44, 5, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            }

            /* Wilgotność */
            if (dht_hum != prev_dhtHum) {
                prev_dhtHum = dht_hum;
                intToString(dht_hum, buf, 10, 10);
                oled_putString(78, 5, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            }

            /* Odległość */
            if (distanceCm != prev_dist) {
                prev_dist = distanceCm;
                intToString((int)distanceCm, buf, 10, 10);
                oled_putString(20, 20, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                oled_putString(50, 20, (uint8_t*)"   ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            }

            /* Jakość powietrza */
            if (airDigital != prev_air) {
                prev_air = airDigital;
                intToString((int)airDigital, buf, 10, 10);
                oled_putString(20, 35, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            }

            /* Natężenie światła */
            if (lux != prev_lux) {
                prev_lux = lux;
                intToString((int)lux, buf, 10, 10);
                oled_putString(20, 50, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                oled_putString(50, 50, (uint8_t*)"   ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            }
        }
    } /* while(1) */
}

void check_failed(uint8_t *file, uint32_t line) { while (1); }