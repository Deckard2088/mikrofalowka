/*****************************************************************************
MIKROFALÓWKA

Zespół: E-09
Termin zajęć: wtorek, 16:15
Skład zespołu:
-Kacper Skoczylas 254864 (lider)
-Dawid Wachecki 254890
-Witold Dudek 254742

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
#define OLED_CLEAR_INTERVAL_MS 3000

#define BUZZER_PIN_HIGH() GPIO_SetValue(0, 1<<26)
#define BUZZER_PIN_LOW()  GPIO_ClearValue(0, 1<<26)

/* =========================================================================
 * KONFIGURACJA STEROWANIA SILNIKIEM
 * ========================================================================= */
#define MOTOR_PORT      2
#define MOTOR_PIN        0   

#define MOTOR_SET_IDLE() GPIO_SetValue(MOTOR_PORT, 1U << MOTOR_PIN)    
#define MOTOR_SET_BUSY() GPIO_ClearValue(MOTOR_PORT, 1U << MOTOR_PIN)  

#define MQ135_DOUT_PORT 2
#define MQ135_DOUT_PIN  10

#define HCSR04_TRIG_PORT 2
#define HCSR04_TRIG_PIN  4
#define HCSR04_ECHO_PORT 2
#define HCSR04_ECHO_PIN  6

/* =========================================================================
 * KONFIGURACJA PINU DHT11
 * ========================================================================= */
#define DHT11_PORT 2
#define DHT11_PIN  8  

static uint8_t ch7seg = '0';
static uint8_t buf[10];
static volatile uint32_t msTicks = 0;

/* =========================================================================
 * FUNKCJE OPÓŹNIAJĄCE
 * ========================================================================= */
static void delay_us(uint32_t us)
{
    volatile uint32_t count = us * 3;
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

    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
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

    pinCfg.Portnum = MOTOR_PORT;
    pinCfg.Pinnum = MOTOR_PIN;
    PINSEL_ConfigPin(&pinCfg);
    GPIO_SetDir(MOTOR_PORT, 1U << MOTOR_PIN, 1);
    MOTOR_SET_IDLE(); 

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

    GPIO_SetDir(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN, 1);
    GPIO_SetDir(HCSR04_ECHO_PORT, 1U << HCSR04_ECHO_PIN, 0);

    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);

    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 0);
    GPIO_SetDir(MQ135_DOUT_PORT, 1U << MQ135_DOUT_PIN, 0);
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

/* =========================================================================
 * BEZPIECZNA FUNKCJA ODCZYTU DHT11 (Zabezpieczona przed zawieszaniem linii)
 * ========================================================================= */
static int dht11_read(uint8_t* tempC, uint8_t* humidity)
{
    uint8_t data[5] = {0};
    uint32_t i = 0;
    uint32_t bitIndex = 0;

    if (humidity == NULL) {
        return -1;
    }

    // Upewniamy się, że linia zaczyna od czystego stanu wysokiego
    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 1);
    GPIO_SetValue(DHT11_PORT, 1U << DHT11_PIN);
    delay_ms(20); 

    // 1. Wysłanie impulsu START (Stan niski przez 20ms)
    GPIO_ClearValue(DHT11_PORT, 1U << DHT11_PIN);
    delay_ms(20);

    // 2. Podniesienie linii i zwolnienie magistrali (tryb Input)
    GPIO_SetValue(DHT11_PORT, 1U << DHT11_PIN);
    delay_us(30);
    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 0);

    // Od tego punktu, w razie błędu, skaczemy do dht_error, by przywrócić pin
    if (wait_for_level(DHT11_PORT, DHT11_PIN, 0, 100) != 0) {
        goto dht_error;
    }
    if (wait_for_level(DHT11_PORT, DHT11_PIN, 1, 100) != 0) {
        goto dht_error;
    }
    if (wait_for_level(DHT11_PORT, DHT11_PIN, 0, 100) != 0) {
        goto dht_error;
    }

    // 3. Odczyt strumienia 40 bitów danych
    for (i = 0; i < 40; i++) {
        if (wait_for_level(DHT11_PORT, DHT11_PIN, 1, 100) != 0) {
            goto dht_error;
        }
        
        uint32_t high_time = 0;
        while ((GPIO_ReadValue(DHT11_PORT) & (1U << DHT11_PIN)) !=0 ) {
            high_time++;
            delay_us(1);
            
            if (high_time > 100){
                break;
            }
        }

        bitIndex = i / 8;
        if (high_time > 12) {
            data[bitIndex] |= (1U << (7 - (i % 8)));
        }

        if (wait_for_level(DHT11_PORT, DHT11_PIN, 0, 100) != 0) {
            goto dht_error;
        }
    }

    // Weryfikacja sumy kontrolnej
    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) {
        goto dht_error;
    }

    *humidity = data[0];
    if (tempC != NULL) {
        *tempC = data[2];
    }
    
    // Udany odczyt - ustawiamy pin jako bezpieczne wejście i kończymy
    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 0);
    return 0;

dht_error:
    // Awaryjne przywrócenie pinu jako wejście w przypadku jakiegokolwiek timeoutu
    GPIO_SetDir(DHT11_PORT, 1U << DHT11_PIN, 0);
    return -1;
}

static uint32_t hcsr04_read_cm(void)
{
    uint32_t echoTimeUs = 0;
    uint32_t maxWaitUs = 30000;
    uint32_t timeoutCounter = 0;
    
    uint32_t oldPR = LPC_TIM0->PR; 

    // 1. Wysłanie impulsu TRIG (10 us)
    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);
    delay_us(2);
    GPIO_SetValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);
    delay_us(10);
    GPIO_ClearValue(HCSR04_TRIG_PORT, 1U << HCSR04_TRIG_PIN);

    // 2. Oczekiwanie na ECHO = 1
    if (wait_for_level(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN, 1, maxWaitUs) != 0) {
        return 0; 
    }

    // 3. START POMIARU
    LPC_TIM0->TCR = 0x02; 
    LPC_TIM0->PR  = 24;   // Taktowanie stopera co 1 us
    LPC_TIM0->TCR = 0x01; 

    // 4. Czekamy na opadnięcie ECHO = 0
    while ((GPIO_ReadValue(HCSR04_ECHO_PORT) & (1U << HCSR04_ECHO_PIN)) != 0) {
        timeoutCounter++;
        if (timeoutCounter > 600000) { 
            break;
        }
    }

    // 5. STOP POMIARU
    echoTimeUs = LPC_TIM0->TC; 
    LPC_TIM0->TCR = 0x00;   
    LPC_TIM0->PR  = oldPR;  

    return echoTimeUs / 58U;
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

/* =========================================================================
 * GŁÓWNA FUNKCJA MAIN
 * ========================================================================= */
int main(void)
{
    //uint8_t rotaryState = ROTARY_WAIT;
    uint32_t lastDecayTime = 0;
    uint32_t lastDhtTime = 0;
    uint32_t lastLightUpdateTime = 0;

    uint8_t lastCh7seg = '0';

    uint8_t dhtHum = 0;
    int32_t temp10 = 0;
    uint32_t distanceCm = 0;
    uint32_t distanceStateProg = 1; 
    uint32_t airDigital = 0;
    //uint32_t lux = 0;

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
    lastLightUpdateTime = msTicks;

    refreshOutputs();

    uint32_t lastTaskTime = 0;
    uint8_t programStep = 0;

    while (1)
    {
        /* =========================================================================
         * ODCZYT I AKTUALIZACJA LUXÓW W PĘTLI GŁÓWNEJ
         * ========================================================================= */
        if ((msTicks - lastLightUpdateTime) >= 50) 
        {
            lastLightUpdateTime = msTicks;
            uint32_t lux = light_read();
            
            oled_putString(1, 50, (uint8_t*)"L: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            intToString(lux, buf, 10, 10);
            oled_putString(20, 50, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(60, 50, (uint8_t*)"lx ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
        }

        /* =========================================================================
         * KROK INTERWENCYJNY: ROTARY
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
         * AUTO DECAY
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
         * MASZYNA STANÓW (Time-Slicing) - Wykonuje się co 200 ms
         * ========================================================================= */
        if ((msTicks - lastTaskTime) >= 200)
        {
            lastTaskTime = msTicks;

            MOTOR_SET_BUSY();

            switch (programStep)
            {
                case 0:
                    /* Krok 0: Przejście do właściwych pomiarów */
                    programStep = 1;
                    break;

                case 1:
                    /* Krok 1: Odczyt temperatury bazowej i DHT11 */
                    temp10 = temp_read();

                    // Bezpieczny interwał odpytywania DHT11 (co 3.5 sekundy)
                    if ((msTicks - lastDhtTime) >= 3500) {
                        lastDhtTime = msTicks; // Timer odliczany zawsze (blokuje zawieszanie)
                        dht11_read(NULL, &dhtHum); // Przy błędzie zachowa stary odczyt
                    }
                    programStep = 2;
                    break;

                case 2:
                    /* Krok 2: Odczyt MQ-135 */
                    airDigital = (GPIO_ReadValue(MQ135_DOUT_PORT) & (1U << MQ135_DOUT_PIN)) ? 1U : 0U;
                    programStep = 3;
                    break;
                    
                case 3:
                    /* Krok 3: Pomiar odległości */
                    distanceCm = hcsr04_read_cm();
                    
                    if (distanceCm > 0 && distanceCm < 4) {
                        distanceStateProg = 1; 
                    } else {
                        distanceStateProg = 0; 
                    }
                    programStep = 4;
                    break;

                case 4:
                    /* Krok 4: Aktualizacja temperatury i wilgotności na OLED */
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
                    /* Krok 5: Wyświetlanie stanu odległości na OLED */
                    oled_putString(1, 20, (uint8_t*)"U: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    intToString(distanceStateProg, buf, 10, 10);
                    oled_putString(20, 20, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    oled_putString(40, 20, (uint8_t*)"       ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    
                    programStep = 6;
                    break;

                case 6:
                    /* Krok 6: Czujnik gazu na OLED */
                    oled_putString(1, 35, (uint8_t*)"A: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    intToString(airDigital, buf, 10, 10);
                    oled_putString(20, 35, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    oled_putString(60, 35, (uint8_t*)"   ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                    
                    programStep = 7;
                    break;

                case 7:
                    /* Krok 7: Zamknięcie pętli pomiarowej */
                    programStep = 0; 
                    MOTOR_SET_IDLE();
                    break;

                default:
                    programStep = 0;
                    MOTOR_SET_IDLE();
                    break;
            }
        }
    }
}

void check_failed(const uint8_t *file, uint32_t line)
{
    while(1);
}
