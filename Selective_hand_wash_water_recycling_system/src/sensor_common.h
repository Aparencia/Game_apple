/*
 * sensor_common.h —— 传感器应用公共定义与工具
 * ---------------------------------------------------------------------------
 * 本文件只放"与具体传感器无关"的公共内容：
 *   ① 统一状态码 sensor_status_t
 *   ② 1 ms 系统时基（SysTick 中断实现，全工程唯一时间源）
 *   ③ 数学/滤波小工具（中值、排序、EMA、限幅、平方根）
 *
 * 设计约束：
 *   - 不依赖 libm（sqrt 由 Newton 迭代自行实现），避免链接选项差异；
 *   - 不分配内存、不阻塞；
 *   - 时基由 SysTick 1 ms 中断累加，可安全跨 49.7 天回绕（无符号减法）。
 */
#ifndef SENSOR_COMMON_H
#define SENSOR_COMMON_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 状态码 ============================ */
typedef enum e_sensor_status
{
    SENSOR_OK          =  0,  /* 正常 */
    SENSOR_ERR_PARAM   = -1,  /* 参数非法（空指针/越界） */
    SENSOR_ERR_INIT    = -2,  /* 未初始化/初始化失败 */
    SENSOR_ERR_TIMEOUT = -3,  /* ADC 扫描或 1-Wire 超时 */
    SENSOR_ERR_RANGE   = -4,  /* 读数超出物理可信范围（贴轨/开路/饱和） */
    SENSOR_ERR_DATA    = -5   /* 数据不可信（CRC 错/全 0/全 FF） */
} sensor_status_t;

/* ============================ 系统时基 ============================ */
/* 初始化 1 ms SysTick；返回 true=成功。全工程只应调用一次。 */
bool     timebase_init(void);

/* 读取上电以来的毫秒数（中断里累加；安全回绕）。 */
uint32_t timebase_ms(void);

/*
 * 软件看门狗：在 SysTick 中断里计一个 3 s 超时，主循环必须定期 timebase_wdt_feed()；
 * 超时用 NVIC_SystemReset() 复位，P104 复位后回到排空电平。
 * 注意：这是软件看门狗，不能替代硬件 IWDT——若主循环在关中断状态下永久卡死，
 *       本机制也失效；量产应再启用硬件 IWDT。
 */
void     timebase_wdt_arm(void);
void     timebase_wdt_feed(void);

/* ============================ 数学/滤波工具 ============================ */
/* 原地排序后取中位数（会修改传入数组）。n=0 返回 0。 */
uint16_t sensor_median_u16(uint16_t *p_values, uint16_t n);

/* ADC 码 → 毫伏（四舍五入）。raw 会被限制在 [0, full_scale]。 */
uint32_t sensor_adc_to_mv(uint16_t raw, uint16_t full_scale, uint32_t vref_mv);

/* 一阶低通（EMA）：alpha=0 保持原值，alpha=1 直接跟随。alpha 自动限幅到 [0,1]。 */
float    sensor_ema(float prev, float sample, float alpha);

/* 限幅到 [min_value, max_value]（min>max 时返回 min_value）。 */
float    sensor_clampf(float value, float min_value, float max_value);

/* 绝对值（不用 fabsf，避免 math.h 依赖）。 */
float    sensor_absf(float value);

/* NaN 判定：按 IEEE-754 位型检查（不用 x != x，避免 -Wfloat-equal 告警；
 * 也不依赖 math.h 的 isnan）。外部传入的 float 都应先过这一层清洗。 */
static inline bool sensor_is_nan_f32(float value)
{
    union
    {
        float    f;
        uint32_t u;
    } bits;

    bits.f = value;

    return (bool) (((bits.u & 0x7F800000UL) == 0x7F800000UL) &&
                   ((bits.u & 0x007FFFFFUL) != 0UL));
}

/* 平方根（Newton 迭代；value<=0 返回 0）。相对误差 < 1e-5，够传感器统计用。 */
float    sensor_sqrtf(float value);

/* 状态码转静态字符串（打印日志用，绝不返回 NULL）。 */
const char *sensor_status_str(sensor_status_t status);

#ifdef __cplusplus
}
#endif
#endif /* SENSOR_COMMON_H */
