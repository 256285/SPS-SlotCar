#include <msp430.h>
#include <stdint.h>
#include "include/UART.h"
#include "include/I2C.h"
#include "include/LED.h"
#include "include/motor.h"
#include "include/main.h"
#include "include/L3GD20H.h"
#include "include/logging.h"
#include "include/track.h"

#define I2C_RX_BUFFER_SIZE       6

#define TURN_ON_THRESHOLD        4000
#define TURN_OFF_THRESHOLD       1200

#define SPEED_LEARN              15                 // původní PWM ~65 = odhad 1m/s
#define SPEED_RACE_STRAIGHT      30                 // původní 65 odpovídá 12,5
#define SPEED_RACE_TURN          8

#define BRAKE_LEAD_TICKS         300
#define TURN_ACCEL_PERCENT       80

#define GYRO_CAL_SAMPLES         32
#define GYRO_WARMUP_SAMPLES      20

#define I2C_READ_DELAY           16000
#define GYRO_SAMPLE_DELAY        160000





/* Functions */

void initClockTo16MHz(void);
void SensorReset(void);

static void readGyroscope(void);
static void calibrateYawOffset(void);
static void updateTurnState(void);
static void updateOutputs(void);


/* I2C */

uint8_t RX_buffer[I2C_RX_BUFFER_SIZE] = {0};
volatile uint8_t RXByteCtr = 0;
volatile uint8_t ReceiveIndex = 0;


/* Gyroscope */

int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;

int16_t yawOffset = 0;
int32_t yaw = 0;


/* Driving */

TurnState turnState = STRAIGHT;


/* Main */

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    initClockTo16MHz();
    _BIS_SR(GIE);

    SensorReset();


    /* I2C */

    P3DIR |= BIT0 | BIT3;
    P3OUT |= BIT0;
    P3OUT &= ~BIT3;

    __delay_cycles(1600000);

    I2C_init(L3GD20H_I2C_ADDR);


    /* Gyroscope */

    I2C_write_byte(CTRL1, BIT0 | BIT1 | BIT2 | BIT3);
    I2C_write_byte(CTRL2, 0);
    I2C_write_byte(CTRL4, BIT7);

    __delay_cycles(3200000);

    UART_init();


    /* Outputs */

    LED_init();
    RGB_LED_init();

    Motor_init();
    H_MODE1_OFF();
    Motor_setSpeed(0);


    /* Gyroscope calibration */

    calibrateYawOffset();


    /* Initialization finished */

    RGB_RED_OFF();

    __delay_cycles(32000000);


    /* Tracking */

    Track_reset();

    warmup = 0;

    Log_init();

    Motor_setSpeed(SPEED_LEARN);


    /* Main loop */

    while (1)
    {
        if (!Log_sampleReady())
            continue;


        readGyroscope();

        yaw = (int32_t)gyroZ - yawOffset;

        updateTurnState();


        /* if(Track_isDone())
        {
            Track_send();
            Log_send();
        } */


        if (warmup < WARMUP_SAMPLES)
        {
            warmup++;                       // auto se ještě rozjíždí
            RGB_RED_ON();
        }
        else
        {
            RGB_RED_OFF();

            Log_add((int16_t)yaw);          // loguj až po rozjezdu

            if (Track_update(turnState, yaw))
            {
                RGB_GREEN_ON();
            }

            if (Track_sigReady())
                RGB_BLUE_OFF();
            else
                RGB_BLUE_ON();

            if (Track_overflow())
                RGB_RED_ON();
        }


        updateOutputs();
    }


    // UART_send_int(turnState);
    // UART_send_string("\r\n");
}


/* Gyroscope read */

static void readGyroscope(void)
{
    ReceiveIndex = 0;

    I2C_read_byte(OUT_X_L, 6);

    __delay_cycles(I2C_READ_DELAY);

    while (RXByteCtr != 0);


    gyroX = (int16_t)(((uint16_t)RX_buffer[1] << 8)
                    | RX_buffer[0]);

    gyroY = (int16_t)(((uint16_t)RX_buffer[3] << 8)
                    | RX_buffer[2]);

    gyroZ = (int16_t)(((uint16_t)RX_buffer[5] << 8)
                    | RX_buffer[4]);
}


/* Gyroscope calibration */

static void calibrateYawOffset(void)
{
    uint8_t i;
    int32_t offsetSum = 0;


    /* Warmup */

    for (i = 0; i < GYRO_WARMUP_SAMPLES; i++)
    {
        readGyroscope();

        __delay_cycles(GYRO_SAMPLE_DELAY);
    }


    /* Offset */

    for (i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
        readGyroscope();

        offsetSum += gyroZ;

        __delay_cycles(GYRO_SAMPLE_DELAY);
    }

    yawOffset = (int16_t)(offsetSum / GYRO_CAL_SAMPLES);
}


/* Turn detection */

static void updateTurnState(void)
{
    switch (turnState)
    {
    case STRAIGHT:

        if (yaw > TURN_ON_THRESHOLD)
        {
            // Left
            turnState = LEFT_TURN;
        }
        else if (yaw < -TURN_ON_THRESHOLD)
        {
            // Right
            turnState = RIGHT_TURN;
        }

        break;


    case LEFT_TURN:

        if (yaw < -TURN_ON_THRESHOLD)
        {
            // Right
            turnState = RIGHT_TURN;
        }
        else if (yaw < TURN_OFF_THRESHOLD)
        {
            // Straight
            turnState = STRAIGHT;
        }

        break;


    case RIGHT_TURN:

        if (yaw > TURN_ON_THRESHOLD)
        {
            // Left
            turnState = LEFT_TURN;
        }
        else if (yaw > -TURN_OFF_THRESHOLD)
        {
            // Straight
            turnState = STRAIGHT;
        }

        break;
    }
}


/* Outputs */

static void updateOutputs(void)
{
    uint8_t currentType;
    uint8_t nextType;

    uint16_t predictedTicks;
    uint16_t segmentTicks;


    /* LEDs */

    switch (turnState)
    {
    case LEFT_TURN:

        // Left

        LED_FL_ON();
        LED_RL_ON();

        LED_FR_OFF();
        LED_RR_OFF();

        break;


    case RIGHT_TURN:

        // Right

        LED_FR_ON();
        LED_RR_ON();

        LED_FL_OFF();
        LED_RL_OFF();

        break;


    case STRAIGHT:

        // Straight

        LED_FL_OFF();
        LED_RL_OFF();
        LED_FR_OFF();
        LED_RR_OFF();

        break;
    }


    /* Learning */

    if (!Track_isDone())
    {
        Motor_setSpeed(SPEED_LEARN);

        return;
    }


    /* Race */

    currentType =
        Track_currentType();

    nextType =
        Track_nextType();


    /* Straight */

    if (currentType == STRAIGHT)
    {
        predictedTicks =
            Track_predictedTicks();

        segmentTicks =
            Track_segmentTicks();


        /* Brake before turn */

        if ((nextType != STRAIGHT)
            && (predictedTicks > BRAKE_LEAD_TICKS)
            && (segmentTicks >=
                predictedTicks - BRAKE_LEAD_TICKS))
        {
            Motor_setSpeed(SPEED_RACE_TURN);
        }
        else
        {
            Motor_setSpeed(SPEED_RACE_STRAIGHT);
        }
    }


    /* Turn */

    else
    {
        /* Accelerate before exit */

        if (Track_turnProgressReached(TURN_ACCEL_PERCENT))
        {
            Motor_setSpeed(SPEED_RACE_STRAIGHT);
        }
        else
        {
            Motor_setSpeed(SPEED_RACE_TURN);
        }
    }
}


/* I2C interrupt */

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    uint8_t rx_val = 0;

    switch (__even_in_range(UCB0IV, 12))
    {
    case 0:
        break;

    case 2:
        break;

    case 4:
        break;

    case 6:
        break;

    case 8:
        break;

    case USCI_I2C_UCRXIFG:

        rx_val = UCB0RXBUF;

        if (RXByteCtr)
        {
            RX_buffer[ReceiveIndex++] = rx_val;
            RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
            UCB0CTL1 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
            UCB0IE &= ~UCRXIE;
        }

        break;

    case 12:
        break;

    default:
        break;
    }
}


/* Clock */

void initClockTo16MHz(void)
{
    UCSCTL3 |= SELREF_2;
    UCSCTL4 |= SELA_2;

    __bis_SR_register(SCG0);

    UCSCTL0 = 0x0000;
    UCSCTL1 = DCORSEL_5;
    UCSCTL2 = FLLD_0 + 487;

    __bic_SR_register(SCG0);

    __delay_cycles(500000);

    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
    }
    while (SFRIFG1 & OFIFG);
}


void Sampling_timer_init(void)
{
    TA0CCR0  = (16000000UL / 8 / SAMPLE_RATE_HZ) - 1;   // SMCLK 16 MHz / 8
    TA0CCTL0 = CCIE;
    TA0CTL   = TASSEL_2 | ID_3 | MC_1 | TACLR;
}


/* Sensor reset */

void SensorReset(void)
{
    SENSOR_POWER_OFF();

    __delay_cycles(1600000);

    SENSOR_POWER_DIRPORT |= SENSOR_POWER_PIN;
    SENSOR_POWER_ON();

    __delay_cycles(1600000);
}