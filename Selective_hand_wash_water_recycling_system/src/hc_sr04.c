/*
 * hc_sr04.c
 *
 * HC-SR04 标准四针液位驱动（RA4M2_MINI）
 *   TRIG = P402 / U1-2，3.3V 输出（外部 3.3→5V 电平转换）
 *   ECHO = P409 / U2-3，IRQ6，5V 容忍（建议串 1k）
 *
 * 不使用 GPT：Echo 上升/下降沿 IRQ，DWT CYCCNT 测脉宽。
 */

#include "hc_sr04.h"

#include <stddef.h>
#include "hal_data.h"
#include "r_ioport.h"

#define HC_TRIG_PIN          BSP_IO_PORT_04_PIN_02   /* P402 */
#define HC_ECHO_PIN          BSP_IO_PORT_04_PIN_09   /* P409 */
#define HC_IRQ_CHANNEL       6U
#define HC_VECTOR_INDEX      9U
#define HC_EVENT_IRQ6        0x007U                  /* ELC_EVENT_ICU_IRQ6 */

#define HC_TRIG_PERIOD_MS    100U
#define HC_ECHO_TIMEOUT_MS   30U
#define HC_MIN_PULSE_US      100U
#define HC_MAX_PULSE_US      25000U

static bool     g_hc_ready;
static uint32_t g_cycles_per_us;
static uint32_t g_last_trig_ms;
static bool     g_waiting;
static volatile bool     g_have_rise;
static volatile bool     g_pulse_ready;
static volatile uint32_t g_rise_cycles;
static volatile uint32_t g_pulse_cycles;
static volatile uint32_t g_trig_count;
static volatile uint32_t g_edge_count;
static volatile uint32_t g_rise_count;
static volatile uint32_t g_fall_count;
static volatile uint32_t g_timeout_count;
static volatile uint32_t g_last_pulse_us;

static uint16_t g_hist_mm[3];
static uint8_t  g_hist_count;
static uint8_t  g_hist_pos;
static uint16_t g_mm;
static bool     g_valid;
static uint32_t g_last_valid_ms;

static void hc_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * g_cycles_per_us;
    uint32_t guard = 0U;

    while (((uint32_t)(DWT->CYCCNT - start) < ticks) && (guard < 1000000U))
    {
        guard++;
    }
}

static void hc_add_sample(uint16_t mm, uint32_t now_ms)
{
    g_hist_mm[g_hist_pos] = mm;
    g_hist_pos = (uint8_t)((g_hist_pos + 1U) % 3U);
    if (g_hist_count < 3U)
    {
        g_hist_count++;
    }

    if (g_hist_count >= 3U)
    {
        uint16_t a = g_hist_mm[0];
        uint16_t b = g_hist_mm[1];
        uint16_t c = g_hist_mm[2];
        uint16_t t;

        if (a > b)
        {
            t = a; a = b; b = t;
        }
        if (b > c)
        {
            t = b; b = c; c = t;
        }
        if (a > b)
        {
            t = a; a = b; b = t;
        }
        g_mm = b;
    }
    else
    {
        g_mm = mm;
    }

    g_valid = true;
    g_last_valid_ms = now_ms;
}

bool hc_sr04_init(void)
{
    fsp_err_t err;

    g_cycles_per_us = SystemCoreClock / 1000000U;
    if (0U == g_cycles_per_us)
    {
        g_cycles_per_us = 180U;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    err = R_IOPORT_PinCfg(&g_ioport_ctrl, HC_TRIG_PIN,
                          (uint32_t)IOPORT_CFG_PORT_DIRECTION_OUTPUT);
    if (FSP_SUCCESS != err)
    {
        return false;
    }
    (void)R_IOPORT_PinWrite(&g_ioport_ctrl, HC_TRIG_PIN, BSP_IO_LEVEL_LOW);

    err = R_IOPORT_PinCfg(&g_ioport_ctrl, HC_ECHO_PIN,
                          (uint32_t)IOPORT_CFG_PORT_DIRECTION_INPUT |
                          (uint32_t)IOPORT_CFG_IRQ_ENABLE);
    if (FSP_SUCCESS != err)
    {
        return false;
    }

    /* IRQ6: both edges, no digital filter (Echo is clean 40 kHz burst envelope). */
    R_ICU->IRQCR[HC_IRQ_CHANNEL] = 0x02U;
    R_ICU->IELSR[HC_VECTOR_INDEX] = HC_EVENT_IRQ6;

    NVIC_ClearPendingIRQ((IRQn_Type)HC_VECTOR_INDEX);
    NVIC_SetPriority((IRQn_Type)HC_VECTOR_INDEX, 6U);
    NVIC_EnableIRQ((IRQn_Type)HC_VECTOR_INDEX);

    g_hc_ready = true;
    g_waiting = false;
    g_last_trig_ms = 0U;
    g_valid = false;
    g_hist_count = 0U;
    g_hist_pos = 0U;
    g_trig_count = 0U;
    g_edge_count = 0U;
    g_rise_count = 0U;
    g_fall_count = 0U;
    g_timeout_count = 0U;
    g_last_pulse_us = 0U;
    return true;
}

void hc_sr04_task(uint32_t now_ms)
{
    if (!g_hc_ready)
    {
        return;
    }

    if (g_waiting)
    {
        if (g_pulse_ready)
        {
            uint32_t cycles = g_pulse_cycles;
            uint32_t us = cycles / g_cycles_per_us;

            g_pulse_ready = false;
            g_waiting = false;
            g_last_pulse_us = us;

            if ((us >= HC_MIN_PULSE_US) && (us <= HC_MAX_PULSE_US))
            {
                /* 20 °C 声速 343 m/s；后续可用 DS18B20 做温度修正。 */
                uint32_t mm = (us * 343U) / 2000U;

                if ((mm >= 20U) && (mm <= 2000U))
                {
                    hc_add_sample((uint16_t)mm, now_ms);
                }
            }
        }
        else if ((uint32_t)(now_ms - g_last_trig_ms) >= HC_ECHO_TIMEOUT_MS)
        {
            g_waiting = false;
            g_timeout_count++;
        }
    }

    if ((!g_waiting) && ((uint32_t)(now_ms - g_last_trig_ms) >= HC_TRIG_PERIOD_MS))
    {
        g_pulse_ready = false;
        g_have_rise = false;

        (void)R_IOPORT_PinWrite(&g_ioport_ctrl, HC_TRIG_PIN, BSP_IO_LEVEL_HIGH);
        hc_delay_us(12U);
        (void)R_IOPORT_PinWrite(&g_ioport_ctrl, HC_TRIG_PIN, BSP_IO_LEVEL_LOW);

        g_last_trig_ms = now_ms;
        g_waiting = true;
        g_trig_count++;
    }

    if (g_valid && ((uint32_t)(now_ms - g_last_valid_ms) > 1000U))
    {
        g_valid = false;
    }
}

bool hc_sr04_get_mm(uint16_t *p_mm, bool *p_valid)
{
    if (NULL != p_mm)
    {
        *p_mm = g_mm;
    }
    if (NULL != p_valid)
    {
        *p_valid = g_valid;
    }
    return g_valid;
}

void hc_sr04_get_diag(uint32_t *p_trig, uint32_t *p_edges, uint32_t *p_rise,
                      uint32_t *p_fall, uint32_t *p_timeout, uint32_t *p_last_us)
{
    if (NULL != p_trig)
    {
        *p_trig = g_trig_count;
    }
    if (NULL != p_edges)
    {
        *p_edges = g_edge_count;
    }
    if (NULL != p_rise)
    {
        *p_rise = g_rise_count;
    }
    if (NULL != p_fall)
    {
        *p_fall = g_fall_count;
    }
    if (NULL != p_timeout)
    {
        *p_timeout = g_timeout_count;
    }
    if (NULL != p_last_us)
    {
        *p_last_us = g_last_pulse_us;
    }
}

void hc_sr04_echo_isr(void)
{
    uint32_t now = DWT->CYCCNT;
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;

    (void)R_IOPORT_PinRead(&g_ioport_ctrl, HC_ECHO_PIN, &level);
    g_edge_count++;

    if (BSP_IO_LEVEL_HIGH == level)
    {
        g_rise_cycles = now;
        g_have_rise = true;
        g_rise_count++;
    }
    else if (g_have_rise)
    {
        g_pulse_cycles = now - g_rise_cycles;
        g_have_rise = false;
        g_pulse_ready = true;
        g_fall_count++;
    }

    /* 必须清 IRQ 状态位，否则边沿 IRQ 会一直重入，主循环被饿死/看门狗复位。 */
    R_ICU->IELSR_b[HC_VECTOR_INDEX].IR = 0U;
}
