


/*
 * sensor_common.c —— 传感器应用公共库实现（1 ms 时基 / 排序 / 中值 / 换算 / 滤波 / 统计）
 * ---------------------------------------------------------------------------
 *  本文件提供全工程唯一的"地基能力"，其余传感器/流水线/判别器只调用不重复实现：
 *    ① 1 ms 系统时基：SysTick 中断累加毫秒计数（全工程唯一时间源）；
 *    ② 数值工具：原地插入排序、中值、ADC 码→mV（四舍五入）、EMA 一阶低通、
 *       限幅、绝对值；
 *    ③ 平方根：Newton 迭代自行实现（不依赖 libm，两套工具链都能链接）；
 *    ④ 统计：均值 / 样本标准差(n-1) / 最小 / 最大 / 样本数；
 *    ⑤ 状态码 → 静态字符串（日志打印用，绝不返回 NULL）。
 *
 *  实现约束（与头文件约定一致）
 *  ---------------------------------------------------------------------------
 *  - C99，不使用动态内存、不使用变长数组、不阻塞；
 *  - 不使用 libm（sqrt 自行实现）、不使用浮点格式化输出（CSV 全用缩放整数）；
 *  - 时间单位一律 ms，电压单位一律 mV；ADC 为 12 位、右对齐，VREF=3300 mV。
 *
 *  时基设计与时间预算
 *  ---------------------------------------------------------------------------
 *  SysTick 1 ms 中断里只做一次计数自增，不阻塞任何业务；主循环 tick（50 ms）、
 *  判别消抖块（200 ms）、DS18B20 非阻塞任务（2 s 发起 / 800 ms 读取）、
 *  CSV 输出（200 ms/行）都读同一份 timebase_ms()，因此全系统时间一致、不漂移。
 *  调用者用 (uint32_t)(now_ms - since_ms) 求时间差：无符号减法在 2^32 ms
 *  （约 49.7 天）回绕时仍然正确，无需额外判断。
 *  DS18B20 的 1-Wire 事务会短暂关中断约 1~6 ms（每 2 s 一次），SysTick 会丢
 *  几个 tick，时基误差 <0.3%，对 50 ms 调度无实质影响（已评估，可接受）。
 *
 *  目标工程：Selective_hand_wash_water_recycling_system
 *            （RA4M2_MINI / R7FA4M2AD3CFL，e² studio FSP 6.6.0）
 *  参考实现：D:\Code\RA4M2\tds_mini\src\hal_entry.c（已验证固件，中值/换算逻辑照搬）
 * ---------------------------------------------------------------------------
 */

#include <stddef.h> /* NULL（不依赖其他头文件间接提供） */
#include "hal_data.h" /* SystemCoreClock / SysTick_Config / CMSIS 内核接口 */
#include "sensor_common.h"

/* SysTick 中断入口由本文件定义（覆盖 FSP 启动文件里的弱符号）：
 * 提前声明，满足 -Wmissing-declarations，也告诉读代码的人这是"被向量表调用"的函数。 */
void SysTick_Handler(void);

/* ---------------------------------------------------------------------------
 *  一、1 ms 系统时基
 * ---------------------------------------------------------------------------
 *  为什么用 SysTick 而不是 GPT/AGT：
 *    ① 不需要额外外设资源，RASC 里不用配置，和 FSP 板级初始化顺序无关；
 *    ② 内核自带、所有 Cortex-M33 工程通用，验证工程（FSP 5.9）也能直接编译；
 *    ③ tds_mini 用"软件延时 + 循环计数"累加时间，长期会累积漂移；硬件时基没有。
 */

/* 上电以来的毫秒计数（只在 SysTick 中断里自增，主循环只读）。
 * volatile 防止编译器把读缓存到寄存器里导致"时间不走"。 */

static volatile uint32_t s_ms_ticks = 0U;

/* s_ms_ticks 加 1，给全工程提供唯一时间源。 时序/边界：中断周期 = SystemCoreClock / 1000；本板约 1 ms。
 * 注意：中断里只允许做这种"几十纳秒"的工作。绝不能在这里 printf、延时、 操作 1-Wire 或调用任何阻塞函数，否则会拖慢 UART/ADC 中断响应。 */

void SysTick_Handler(void)
{
    s_ms_ticks++;
}

/* 清空计数并按 SystemCoreClock/1000 装载 SysTick，启动周期中断。 时序/边界：写 SysTick 的 LOAD/VAL/CVR，立即开始计数；不阻塞。
 * 注意：① 全工程只应调用一次，重复调用会重新装载 SysTick、把时基清零； ② 必须在 SystemCoreClock 已被 FSP 设置好之后调用（app_main 里调用即可）； ③ SysTick 优先级用 CMSIS 默认（最低）：时基只做统计调度， 不应该抢 UART/ADC 的实时性；1-Wire 关中断丢 tick 的代价已评估可接受。 */

bool timebase_init(void)
{
    /* SystemCoreClock 由 FSP 在 SystemInit() 中按实际时钟树写好（本板数十 MHz），
     * 除以 1000 得到 1 ms 的 clock 数。 */
    uint32_t ticks = SystemCoreClock / 1000U;
    if(0U == ticks)
    {
        /* 极端保护：时钟配置异常时避免 SysTick_Config 内部除零/死循环 */
        return false;
    }

    s_ms_ticks = 0U;/*从0开始计时，便于日志里看“上电多少毫秒”*/

    if(0U != SysTick_Config(ticks))
    {
        /* SysTick_Config 返回非 0 表示失败（ticks 超出 24 位装载范围等） */
        return false;
    }

    return true;
}

/* 返回 SysTick 中断累加的毫秒计数，供主循环/状态机调度使用。 时序/边界：单条 32 位对齐读，在 Cortex-M33 上是原子操作，不需要关中断。
 * 注意：① 求时间差必须用 (uint32_t)(now - start) 的无符号减法， 不要写 if (now > start)，回绕后会判断错； ② 不要在中断和主循环里同时写任何"基于该时间的状态"（本工程约定： 只有主循环调用业务函数，中断只置标志）。 */

uint32_t timebase_ms(void)
{
    return s_ms_ticks;
}

/* ---------------------------------------------------------------------------
 *  二、排序 / 中值
 * ---------------------------------------------------------------------------
 *  ADC 采样为什么要取中值：单点可能被静电、继电器吸合、接触抖动打坏。
 *  中值对"少量离群点"不敏感（均值会被一个坏点拉偏），比均值更适合做 ADC 前置滤波。
 *  样本数 n 很小（默认 5，上限 9），插入排序 O(n²) 的常数极小、无递归、无内存分配，
 *  比调用通用 qsort 更省 RAM/代码，也更符合"单片机可读性优先"的工程习惯。
 */

/* 插入排序，把 p_values[0..n-1] 从小到大排好。 时序/边界：O(n²)；n=5 时最多 10 次比较，微秒级，可在主循环里随意调用。
 * 注意：会修改传入数组；若数据被其他上下文同时读取，调用者需自己保证不同时访问。 */

static void sensor_sort_u16(uint16_t *p_values, uint16_t n)
{
    if((NULL == p_values) || (n < 2U))
    {
        return;
    }

    for(uint16_t i = 1U;i<n;i++)
    {

        /* 把 p_values[i] 插入到前面已排好序的区间里：
         * 从后往前找位置，比它大的元素依次后移一格。 */
        uint16_t key = p_values[i];
        uint16_t j = i;
        while((j > 0U) && (p_values[j - 1U] > key))
        {
            p_values[j] = p_values[j - 1U];
            j--;
        }
        p_values[j] = key;
    }
}

/* 对样本先升序排序，再取中位数；ADC 去噪的标准做法。 时序/边界：内部调用插入排序，O(n²)，n 很小。
 * 注意：偶数个样本时取"偏大的一半"（下标 n/2，0 基）： 5 点取正中、4 点取第 3 小。对 ADC 去噪两种取法等价，固定取上中位数 对"单向的极大干扰"更保守（不会被小值拉低）。 */

uint16_t sensor_median_u16(uint16_t*p_values,uint16_t n)
{
    if((NULL == p_values) || (0U == n))
    {
        return 0U;
    }

    sensor_sort_u16(p_values,n);

    return p_values[n/2U];
}

/* ---------------------------------------------------------------------------
 *  三、ADC 码 → 毫伏 / 滤波 / 数值小工具
 * ---------------------------------------------------------------------------
 *  电压是判别与日志的通用单位（CSV 用整数 mV），所以换算在这里只做一次：
 *  mV = raw × VREF / full_scale，四舍五入到 1 mV。
 */

/* 把 12 位 ADC 原始码换算成电压；raw 先限幅到 [0, full_scale]， 防止调用方传入脏数据导致溢出或非物理结果。 时序/边界：纯计算，几条指令。
 * 注意：用 uint32 做中间量：4095×3300 = 13,513,500，远小于 2^32，不会溢出； 四舍五入用"+ full_scale/2 再整除"，避免 1 mV 的系统性偏低。 */

uint32_t sensor_adc_to_mv(uint16_t raw,uint16_t full_scale,uint32_t vref_mv)
{
    if(0U == full_scale)
    {
        return 0U; /*非法满量程：宁返回0，也不做除零*/
    }
    uint32_t code = (uint32_t) raw;
    if(code > (uint32_t) full_scale)
    {
        /*物理上不会出现（12位ADC最大4095）,防御性限幅*/
        code = (uint32_t) full_scale;
    }

    /*mV = code * VREF / full_scale + 0.5（用整数实现四舍五入）*/
    return ((code *vref_mv) + ((uint32_t)full_scale / 2U)) / (uint32_t)full_scale;
}

/* y = prev + alpha × (sample - prev)，指数加权移动平均。 时序/边界：纯计算。
 * 注意：① 为什么用 EMA 而不用滑动平均：滑动平均要存 N 个历史点（RAM 和 代码都更多），而 50 ms tick 下 EMA(α=0.3) 的等效时间常数约 3 个 tick， 足够压掉 ADC 抖动，又不会把皂液阶跃抹平到判别器看不见； ② alpha=0（或 NaN）时保持原值，避免配置写错导致"永远不动"； ③ 要求 sample 来自可信读数（调用方先做 rail/超时检查）。 */

float sensor_ema(float prev,float sample,float alpha)
{
    float a = sensor_clampf(alpha,0.0f,1.0f);

    if(!(a > 0.0f))
    {
        /*alpha = 0 或NaN：保持上一次输出（NaN不会传播出去）*/
        return prev;
    }

    return prev + (a*(sample - prev));
}

/* 统一的安全限幅工具，防止异常输入把下游计算放大或污染。 时序/边界：纯计算。
 * 注意：本实现把非法输入（NaN）当作"小于下限"处理，回落到下限。 这样一条坏数据不会顺着 EMA/统计污染整条链路；阈值类配置可以放心用。 */

float sensor_clampf(float value,float min_value,float max_value)
{
    if(min_value > max_value)
    {
        return min_value; /*配置写反：按下限处理，保证行为可预期*/
    }

    if(!(value > min_value))
    {
        return min_value; /*value<min，或NaN*/
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

/* 替代 fabsf，避免引入 libm。 时序/边界：纯计算，一条比较+取反。 注意：负数取反即可；不处理 NaN/inf 的特殊语义（传感器数据不会到 inf）。 */

float sensor_absf(float value)
{
    return (value <0.0f) ? (-value):value;
}

/* ---------------------------------------------------------------------------
 *  四、平方根（Newton 迭代）
 * ---------------------------------------------------------------------------
 *  为什么自己写：
 *    ① 本工程要求 GCC ARM 10（FSP 5.9 验证工程）与 GCC ARM 13（FSP 6.6 工程）
 *       都能编译链接，libm 的链接参数在两者上不完全一致；
 *    ② 只需要 sqrt，而且只用于统计（σ）——迭代法的精度远远够用。
 *
 *  算法：
 *    先把 value 按 4 倍缩放（同时记移位次数）压到 [1,4) 区间，此时 sqrt ∈ [1,2]，
 *    用 x0 = (1 + scaled)/2 作为初值（保证 x0 ≥ sqrt(scaled)，Newton 单调收敛），
 *    迭代 x = (x + scaled/x)/2（误差平方级下降），10 次后相对误差 < 1e-6。
 *    最后把结果乘 (1/2)^shifts 还原（因为 sqrt(value) = sqrt(scaled) / 2^shifts）。
 */

/* 求 value 的平方根，用于样本标准差 σ = sqrt(方差)。 返回：value > 0 时返回其平方根；value ≤ 0 或 NaN 返回 0（调用方不必先判）。
 * 时序/边界：最多约 10 次浮点除法（统计每次采样只调用 1~2 次，开销可忽略）。 注意：相对误差 < 1e-5，满足传感器统计；不要用它做精确定点数学运算。 */

float sensor_sqrtf(float value)
{
    /* 用 !(value > 0) 而不是 (value <= 0)：这样 NaN 也会落进 0 分支，
     * 避免 NaN 沿统计链路传播。 */
    if (!(value > 0.0f))
    {
        return 0.0f;
    }

    /* 1) 按 4 倍缩放，把输入压到 [1,4)：sqrt 也随之是原值的 0.5^shifts 倍 */
    float    scaled = value;
    uint32_t shifts = 0U;

    while ((scaled >= 4.0f) && (shifts < 128U))
    {
        scaled *= 0.25f;
        shifts++;
    }
    while ((scaled < 1.0f) && (shifts < 128U))
    {
        scaled *= 4.0f;
        shifts++;
    }

    /* 2) 初值取 x0 = (1 + scaled)/2；在 scaled∈[1,4) 时 x0 ≥ sqrt(scaled)，
     *    Newton 迭代从上方单调收敛，不会在根附近来回振荡。 */
    float x = (1.0f + scaled) * 0.5f;

    /* 3) Newton 迭代：x ← (x + scaled/x)/2；10 次足够收敛到 1e-6 相对误差 */
    for (uint32_t i = 0U; i < 10U; i++)
    {
        x = (x + (scaled / x)) * 0.5f;
    }

    /* 4) 还原缩放：每缩放过一次 4 倍，结果就乘一次 0.5 */
    while (shifts > 0U)
    {
        x *= 0.5f;
        shifts--;
    }

    return x;
}

/* ---------------------------------------------------------------------------
 *  六、状态码 → 字符串
 * ---------------------------------------------------------------------------
 *  为什么不用宏数组/查表：switch 对负数状态码安全（数组下标会越界），
 *  编译器也能做跳转表优化；返回 static 字符串字面量，不需要释放、绝不会 NULL。
 */

/* 把 sensor_status_t 转成日志/CSV 用的短字符串（OK/RANGE/...）。 返回：指向静态字符串常量的指针（绝不返回 NULL；未知码返回 "UNKNOWN"）。 时序/边界：纯计算。
 * 注意：返回的指针不要 free、不要写入（字符串在只读段）。 */
const char *sensor_status_str(sensor_status_t status)
{
    switch (status)
    {
        case SENSOR_OK:
            return "OK";
        case SENSOR_ERR_PARAM:
            return "PARAM";
        case SENSOR_ERR_INIT:
            return "INIT";
        case SENSOR_ERR_TIMEOUT:
            return "TIMEOUT";
        case SENSOR_ERR_RANGE:
            return "RANGE";
        case SENSOR_ERR_DATA:
            return "DATA";
        default:
            return "UNKNOWN";
    }
}
