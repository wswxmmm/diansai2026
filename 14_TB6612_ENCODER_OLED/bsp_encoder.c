#include "bsp_encoder.h"

static volatile int32_t g_encoderCount[ENCODER_COUNT] = {0, 0};
static int32_t g_lastCount[ENCODER_COUNT] = {0, 0};

static void update_quadrature(EncoderId id, GPIO_Regs *port, uint32_t a_pin, uint32_t b_pin)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(port, a_pin | b_pin);

    if ((status & a_pin) == a_pin) {
        if ((DL_GPIO_readPins(port, b_pin) & b_pin) != 0U) {
            g_encoderCount[id]++;
        } else {
            g_encoderCount[id]--;
        }
        DL_GPIO_clearInterruptStatus(port, a_pin);
    }

    if ((status & b_pin) == b_pin) {
        if ((DL_GPIO_readPins(port, a_pin) & a_pin) != 0U) {
            g_encoderCount[id]--;
        } else {
            g_encoderCount[id]++;
        }
        DL_GPIO_clearInterruptStatus(port, b_pin);
    }
}

void Encoder_Init(void)
{
    Encoder_Reset();
    NVIC_ClearPendingIRQ(ENCODER_L_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_L_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODER_R_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_R_INT_IRQN);
}

void Encoder_Reset(void)
{
    __disable_irq();
    g_encoderCount[ENCODER_LEFT] = 0;
    g_encoderCount[ENCODER_RIGHT] = 0;
    g_lastCount[ENCODER_LEFT] = 0;
    g_lastCount[ENCODER_RIGHT] = 0;
    __enable_irq();
}

int32_t Encoder_GetCount(EncoderId id)
{
    int32_t count;

    __disable_irq();
    count = g_encoderCount[id];
    __enable_irq();

    return count;
}

WheelSpeed Encoder_UpdateSpeed(EncoderId id)
{
    WheelSpeed speed;
    int32_t count = Encoder_GetCount(id);
    int32_t delta = count - g_lastCount[id];

    g_lastCount[id] = count;

    speed.count = count;
    speed.delta = delta;
    speed.pps = (delta * 1000) / (int32_t) ENCODER_SAMPLE_MS;
    speed.rps_milli = (speed.pps * SPEED_DECIMAL_SCALE) /
        (int32_t) ENCODER_COUNTS_PER_REV;

    return speed;
}

void GROUP1_IRQHandler(void)
{
    update_quadrature(ENCODER_LEFT,
        ENCODER_L_PORT, ENCODER_L_LA_PIN, ENCODER_L_LB_PIN);
    update_quadrature(ENCODER_RIGHT,
        ENCODER_R_PORT, ENCODER_R_RA_PIN, ENCODER_R_RB_PIN);
}