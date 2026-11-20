


/*
 * sensor_ds18b20.c —— DS18B20 单总线水温传感器驱动实现（句柄式 + 非阻塞任务）
 * ---------------------------------------------------------------------------
 *  本文件解决什么问题
 *  ---------------------------------------------------------------------------
 *  TDS 温度补偿必须用"水侧水温"：片上 TSN 测的是芯片温度，与水温可差 10~15 ℃，
 *  按 2 %/℃ 折算会引入 20~30% 温补误差（硬件方案 §13.1 结论：TSN 不能替代）。
 *  DS18B20 用一根线给出 0.0625 ℃ 分辨率的数字温度，但 1-Wire 是微秒级时序、
 *  且"看起来合法其实是假值"的坑很多。本驱动负责：
 *    ① 用 PFS 寄存器直接位操作实现 1-Wire（复位/写/读，微秒级）；
 *    ② Dallas/Maxim CRC8 校验 + 全 0/全 FF 判无效 + 范围检查（三层有效性）；
 *    ③ 非阻塞状态机：2 s 发起转换、800 ms 后读数，不占用 50 ms 主循环；
 *    ④ 器件不可用时回退"手输温度 / 25.0 ℃ 兜底"，并标出来源（CSV 的 src）。
 *  句柄式设计：状态放在 app_main 分配的 ds18b20_t 里，驱动不分配内存。
 *
 *  接线（断电接线，接好再上电）
 *  ---------------------------------------------------------------------------
 *  DS18B20 VDD -> 3.3 V（若模块座已供电，只接 DQ 即可）
 *  DS18B20 GND -> GND
 *  DS18B20 DQ  -> P500（U1-17），DQ 与 3.3 V 之间必须接 4.7 kΩ 上拉电阻
 *  注意：① TDS 模块不要 5 V 供电又插 DS18B20（DQ 会被上拉到 5 V，超 MCU 耐压）；
 *        ② 板子内部上拉只有几十 kΩ，替代不了外部 4.7 kΩ（长线上升沿太慢，
 *           读位会错）；③ 探头金属壳浸入水中 20~30 mm，紧邻 TDS 探头同一水体，
 *           但别贴着 TDS 电极；④ 别带电热插拔，接线一律断电做。
 *
 *  1-Wire 时序表（DS18B20 数据手册；本驱动的实现值）
 *  ---------------------------------------------------------------------------
 *   复位/存在检测：拉低 480 µs → 释放 → 70 µs 后采样（0=器件应答）
 *                  → 再等 420 µs 补满整个复位时隙（合计 ≈970 µs）
 *   写 1        ：拉低 3 µs → 释放 → 等 60 µs（时隙总长 ≥60 µs）
 *   写 0        ：拉低 60 µs → 释放 → 等 5 µs
 *   读位        ：拉低 3 µs → 释放 → 8 µs 后采样 → 再等 52 µs（时隙 63 µs）
 *   命令：Skip ROM 0xCC（总线上只有一个器件）→ Convert T 0x44
 *         或 Read Scratchpad 0xBE（读 9 字节暂存器）
 *   暂存器布局：[0]=温度 LSB、[1]=温度 MSB、[2]=TH、[3]=TL、[4]=配置、
 *              [5..7]=保留、[8]=CRC8
 *   温度：raw = (int16_t)(MSB<<8 | LSB)，t = raw / 16（℃，负温为补码）
 *   CRC：Dallas/Maxim 标准多项式 x^8 + x^5 + x^4 + 1；LSB 优先实现的异或值 0x8C。
 *
 *  为什么用 PFS 寄存器直接位操作
 *  ---------------------------------------------------------------------------
 *  1-Wire 的"写 1"只有 3 µs 低电平，调 R_IOPORT_PinWrite 的驱动函数（含参数
 *  校验、PFS 查表）可能就超过 3 µs 了。tds_mini 已验证：直接读写
 *  R_PFS->PORT[n].PIN[m].PmnPFS_b 的 PODR/PDR/PIDR 位域，速度快且稳定。
 *  引脚从 bsp_io_port_pin_t 按 (port<<8)|pin 编码推导：
 *      例如 BSP_IO_PORT_05_PIN_00 = 0x0500 → PORT[5].PIN[0]
 *
 *  事务关中断（重要）
 *  ---------------------------------------------------------------------------
 *  1-Wire 靠"微秒级低电平宽度"表达数据，任何中断（UART/ADC/SysTick）插进来
 *  都会把脉冲拉长到解析失败。因此每次 1-Wire 事务都用
 *      R_BSP_PinAccessEnable() + __disable_irq() ... __enable_irq() + PinAccessDisable()
 *  包起来（PFS 写保护也必须先 PinAccessEnable）。
 *  单次事务约 1~6 ms（读 9 字节暂存器约 5 ms），每 2 s 才发生一次，
 *  SysTick 会丢几个 tick（<0.3% 时基误差），这是设计内允许的代价。
 *  恢复中断时只在原来"开中断"的情况下才 __enable_irq()（支持嵌套调用）。
 *
 *  三层有效性检查（实验 2 真实踩坑，缺一不可）
 *  ---------------------------------------------------------------------------
 *  ① CRC8：前 8 字节算出的 CRC 必须等于第 9 字节；
 *  ② 全 0 / 全 FF 判无效：DQ 被拉死（接错孔/短路/器件坏）时暂存器可能读到
 *     全 0x00，而"8 个 0 字节的 CRC 恰好也是 0x00"——CRC 校验会通过，
 *     算出 0.00 ℃，于是温补系数 1+0.02×(0−25)=0.5，把 2979 mV 放大成
 *     5958 mV、参考 ppm 顶到 2000，整段数据报废（实验 2 实际发生）。
 *     全 FF 同理（总线一直高、器件没应答）。必须显式判 sp_or/sp_and。
 *  ③ 范围检查：只接受 -20~80 ℃；85.0 ℃ 是上电默认值/转换未完成标志，必须挡掉。
 *  上电自检不仅要"复位有应答"，还要真正读回一个有效温度才算可用
 *  （上面已说明：应答为真时暂存器仍可能是全 0）。
 *  连续 3 次读数无效 → 自动停用，回退 25.0 ℃/手输值；修好线后串口发 C 重新检测。
 *
 *  非阻塞设计（时间预算，对齐项目设想 6.2.1）
 *  ---------------------------------------------------------------------------
 *  ds18b20_task() 由主循环每 50 ms 调一次：
 *    空闲 → 距上次发起 ≥2000 ms 就发 Convert T（只发命令，不等待）；
 *    等待 → 距发起 ≥800 ms（12 位转换最长 750 ms + 余量）就回来读暂存器；
 *    失败 → 计数/回退/重试全部是非阻塞的。
 *  唯一的阻塞调用是上电 ds18b20_boot_test()（约 800 ms，只执行一次）。
 *
 *  参数（默认值来源 + 怎么标定）
 *  ---------------------------------------------------------------------------
 *  - request_period_ms = 2000：水温惯性大，2 s 一次足够（CSV 200 ms 一行会复用）；
 *  - conv_wait_ms = 800：12 位分辨率最长 750 ms + 50 ms 余量；
 *  - redetect_period_ms = 5000：未发现器件时的重试周期（防热插拔没插紧）；
 *  - fail_limit = 3：连续 3 次无效才停用，避免一次偶发错误丢掉温度；
 *  - default_temp_c = 25.0：无传感器且未手输时的兜底水温，CSV 来源标 FIX。
 *
 *  注意事项
 *  ---------------------------------------------------------------------------
 *  1) 设计约束：不使用动态内存、不使用浮点格式化输出、不依赖 libm；
 *     驱动层不打印（状态变化由 app 打印），不控阀。
 *  2) 短时读失败保留最近一次有效值；连续失败才彻底停用并切兜底。
 *  3) 手输温度（串口 T24.5）会清停用标志并触发一次重检，来源标 MAN。
 *  4) 传感器停用后不自动重试（避免假 0.00 ℃ 反复污染数据），命令 C 恢复。
 * ---------------------------------------------------------------------------
 */

#include <stddef.h>      /* NULL */
#include "sensor_ds18b20.h"   /* 已包含 hal_data.h 与 sensor_common.h */

/* ---------------------------- 协议常量 ---------------------------- */
#define DS_CMD_SKIP_ROM          0xCCU   /* 跳过 ROM 匹配：总线上只有一个器件 */
#define DS_CMD_CONVERT_T         0x44U   /* 启动一次温度转换 */
#define DS_CMD_READ_SCRATCHPAD   0xBEU   /* 读 9 字节暂存器 */
#define DS_SCRATCHPAD_LEN        9U      /* 暂存器 9 字节（含 CRC） */
#define DS_CRC8_POLY_REFLECTED   0x8CU   /* Dallas CRC8 的 LSB-first 异或值 */
#define DS_TEMP_LSB_PER_C        16.0f   /* 温度原始值 1 ℃ = 16 LSB（12 位分辨率） */

/* PFS 位操作宏：bsp_io_port_pin_t = (port << 8) | pin。
 * R_PFS->PORT[0..14].PIN[0..15].PmnPFS_b 各字段含义（数据手册）：
 *   PODR(bit0) 输出数据、PIDR(bit1) 输入数据、PDR(bit2) 方向(1=输出)、
 *   PCR(bit4) 上拉使能、PMR(bit16) 外设模式、ASEL(bit15) 模拟输入。
 * 用法：DS18B20_PFS(pin).PODR = 0U; 等。 */
#define DS18B20_PFS(pin)                                                        \
    ((R_PFS)->PORT[(((uint32_t) (pin)) >> 8U) & 0xFFU]                          \
         .PIN[((uint32_t) (pin)) & 0xFFU]                                       \
         .PmnPFS_b)

/* ---------------------------- 底层：延时 ---------------------------- */
/* 调用 FSP 的软件延时（基于循环计数，不依赖中断/定时器）。 时序/边界：阻塞，但总时长微秒级；在关中断状态下也能正常工作（不依赖 SysTick）。
 * 注意：延时精度受 CPU 主频与编译器优化影响，DS18B20 的时序窗口留了余量 （例如写 1 只要求 1~15 µs，本驱动取 3 µs），不要把它用于严格计时。 */
static void ds_delay_us(uint32_t us)
{
    R_BSP_SoftwareDelay(us, BSP_DELAY_UNITS_MICROSECONDS);
}

/* 毫秒级忙等延时（上电自检等待转换用） 时序/边界：阻塞；只在上电 ds18b20_boot_test() 里使用一次，主循环里不走这里。 注意：这是软件延时，期间中断照常响应（没有关中断）。 */
static void ds_delay_ms(uint32_t ms)
{
    R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}

/* ---------------------------- 底层：引脚合法性 ---------------------------- */
/* R_PFS 映射 PORT[0..14] × PIN[0..15]；越界会写到别的外设寄存器， 造成难以排查的硬件异常，所以任何按 pin 推导 PFS 前都先过这一关。
 * 参数：pin —— bsp_io_port_pin_t 编码值。 返回：true=合法；false=越界（调用方直接按"器件不可用"处理）。 时序/边界：纯比较。
 * 注意：默认引脚 P500 = 0x0500 一定合法；换脚时优先用 BSP_IO_PORT_xx_PIN_yy 宏。 */
static bool ds_pin_is_valid(bsp_io_port_pin_t pin)
{
    uint32_t port = ((uint32_t) pin >> 8U) & 0xFFU;
    uint32_t pin_no = (uint32_t) pin & 0xFFU;

    return (port <= 14U) && (pin_no <= 15U);
}

/* ---------------------------- 底层：关中断 + PFS 写保护 ---------------------------- */
/* 打开 PFS 写保护 + 关中断，整段事务期间保持（保证时序不被中断拉长）。 时序/边界：立即生效；关中断时长 ≈ 单次事务 1~6 ms。
 * 注意：① 必须在任何 PFS 直接写之前调用 R_BSP_PinAccessEnable()， 否则写 PFS 无效（RA 的 PFS 受保护）； ② 与 ds_critical_end() 必须成对使用；中间不要调用可能阻塞很久的函数。 */
static void ds_critical_begin(uint32_t *p_primask)
{
    R_BSP_PinAccessEnable();
    *p_primask = __get_PRIMASK();
    __disable_irq();
}

/* 按进入前的 PRIMASK 恢复中断状态，并关闭 PFS 写保护。 时序/边界：立即生效。
 * 注意：只在 primask==0 时才 __enable_irq()，这样即使调用者自己在关中断上下文 里调用，也不会被提前打开中断（支持嵌套）。 */
static void ds_critical_end(uint32_t primask)
{
    if (0U == primask)
    {
        __enable_irq();
    }
    R_BSP_PinAccessDisable();
}

/* ---------------------------- 底层：总线电平操作 ---------------------------- */
/* 把数据线变成"输出 + 输出 0"，用于复位脉冲/写 0/发起读时隙。 参数：pin —— 数据引脚。 时序/边界：寄存器写，纳秒~几十纳秒级。
 * 注意：先写 PODR=0 再切 PDR=1：如果顺序反过来，切输出的瞬间可能输出 锁存里的旧值（高电平），给器件一个错误的上升沿——tds_mini 实测踩过。 */
static void ds_line_low(bsp_io_port_pin_t pin)
{
    DS18B20_PFS(pin).PODR = 0U;   /* 先把输出锁存写 0 */
    DS18B20_PFS(pin).PDR  = 1U;   /* 再切输出：此时线上是稳定的低电平 */
}

/* 1-Wire 的"释放总线"：切回输入后由 4.7 kΩ 上拉把线拉高，器件才能应答。 参数：pin —— 数据引脚。 时序/边界：寄存器写。
 * 注意：释放后线不是"立刻高"，而是被上拉 RC 慢慢拉高——所以每次释放后 都要等规定的采样时间（70 µs / 8 µs 等），不能马上读。 */
static void ds_line_release(bsp_io_port_pin_t pin)
{
    DS18B20_PFS(pin).PDR = 0U;    /* 0=输入（高阻），由上拉电阻拉高 */
}

/* 读 PFS 的 PIDR（端口输入数据位），判断线上是高还是低。 参数：pin —— 数据引脚。 时序/边界：寄存器读。 注意：读之前必须已按协议留足采样延时（复位后 70 µs、读时隙 8 µs）。 */
static bool ds_line_read(bsp_io_port_pin_t pin)
{
    return (1U == (uint32_t) DS18B20_PFS(pin).PIDR);
}

/* ---------------------------- 底层：1-Wire 基本时序 ---------------------------- */
/* 主机拉低 480 µs 后释放，70 µs 时采样：器件存在会把总线拉低。 参数：pin —— 数据引脚。 时序/边界：阻塞约 970 µs（480 + 70 + 420）。
 * 注意：必须等满整个复位时隙（再补 420 µs）再发命令，否则器件状态机没准备好。 要在关中断临界区内调用（由外层 ping/request/read 负责）。 */
static bool ds_reset(bsp_io_port_pin_t pin)
{
    bool presence;

    ds_line_low(pin);
    ds_delay_us(480U);        /* 复位脉冲宽度：数据手册要求 ≥480 µs */
    ds_line_release(pin);
    ds_delay_us(70U);         /* 存在脉冲窗口 15~60 µs；70 µs 采样避开边沿 */
    presence = !ds_line_read(pin);   /* 低电平 = 器件应答 */
    ds_delay_us(420U);        /* 补满复位时隙，再发 ROM/功能命令 */

    return presence;
}

/* 按 DS18B20 时序写 0 或 1（低电平宽度区分）。 参数：pin —— 数据引脚；one —— true 写 1，false 写 0。
 * 时序/边界：写 1 约 63 µs（3 低 + 60 高）；写 0 约 65 µs（60 低 + 5 高）。
 * 注意：写 1 的低电平必须 ≥1 µs（本驱动 3 µs）且 <15 µs；写 0 必须 ≥60 µs。 两次写时隙之间靠"释放后的等待"自然隔开，不要连续拉低。 */
static void ds_write_bit(bsp_io_port_pin_t pin, bool one)
{
    if (one)
    {
        ds_line_low(pin);
        ds_delay_us(3U);       /* 短低脉冲表示 1 */
        ds_line_release(pin);
        ds_delay_us(60U);      /* 时隙总长 ≥60 µs，给器件采样窗口 */
    }
    else
    {
        ds_line_low(pin);
        ds_delay_us(60U);      /* 长低脉冲表示 0 */
        ds_line_release(pin);
        ds_delay_us(5U);       /* 恢复时间，避免下一时隙起点太早 */
    }
}

/* 按 1-Wire 规范从最低位开始逐位写出。 参数：pin —— 数据引脚；value —— 待写字节。 时序/边界：8 个位时隙 ≈ 0.5 ms。
 * 注意：DS18B20 是 LSB first；写 ROM/功能命令都走这里。 */
static void ds_write_byte(bsp_io_port_pin_t pin, uint8_t value)
{
    for (uint32_t i = 0U; i < 8U; i++)
    {
        ds_write_bit(pin, (value & 1U) != 0U);
        value >>= 1U;
    }
}

/* 主机发起读时隙，8 µs 后采样器件输出的位。 参数：pin —— 数据引脚。 时序/边界：拉低 3 µs → 释放 → 8 µs 采样 → 再等 52 µs（时隙 63 µs）。
 * 注意：① 读时隙必须由主机发起（拉低），否则器件不会输出； ② 8 µs 采样点要落在器件有效窗口内（器件在拉低后 15 µs 内给出数据， 但规格要求主机尽早采样，8 µs 是 tds_mini 实测稳定值）。 */
static bool ds_read_bit(bsp_io_port_pin_t pin)
{
    bool bit;

    ds_line_low(pin);
    ds_delay_us(3U);          /* 发起读时隙：拉低 ≥1 µs */
    ds_line_release(pin);
    ds_delay_us(8U);          /* 等器件把数据放到线上，再采样 */
    bit = ds_line_read(pin);
    ds_delay_us(52U);         /* 补满 60 µs 时隙（3+8+52=63 µs） */

    return bit;
}

/* 逐位读出并拼成字节，用于读暂存器 9 字节。 参数：pin —— 数据引脚。 时序/边界：8 个位时隙 ≈ 0.5 ms。 注意：低字节先出（LSB first），与写入规则一致。 */
static uint8_t ds_read_byte(bsp_io_port_pin_t pin)
{
    uint8_t value = 0U;

    for (uint32_t i = 0U; i < 8U; i++)
    {
        value >>= 1U;
        if (ds_read_bit(pin))
        {
            value |= 0x80U;
        }
    }

    return value;
}

/* ---------------------------- 底层：CRC8 ---------------------------- */
/* 对暂存器前 8 字节算 CRC，与第 9 字节比较，判断 1-Wire 传输是否出错。 时序/边界：len×8 次移位异或，微秒级。
 * 注意：多项式 x^8 + x^5 + x^4 + 1 = 0x31（MSB-first）；本实现是 LSB-first， 所以用反射值 0x8C。tds_mini 已验证该实现与器件一致。 */
static uint8_t ds_crc8(const uint8_t *p_data, uint8_t len)
{
    uint8_t crc = 0U;

    while (len > 0U)
    {
        uint8_t in_byte = *p_data++;
        len--;

        for (uint32_t i = 0U; i < 8U; i++)
        {
            uint8_t mix = (uint8_t) ((crc ^ in_byte) & 0x01U);
            crc >>= 1U;
            if (0U != mix)
            {
                crc ^= DS_CRC8_POLY_REFLECTED;
            }
            in_byte >>= 1U;
        }
    }

    return crc;
}

/* ---------------------------- 公共底层接口 ---------------------------- */
/* 判断总线上是否有 DS18B20（掉线/热插拔后重新发现器件用）。 参数：pin —— 数据引脚。 时序/边界：阻塞约 1 ms（复位时隙）。
 * 注意：本函数会自己进临界区（关中断 + PFS 写保护），调用者不要再嵌套关中断。 仅"有应答"不代表读数有效（见 read_temperature 的三层检查）。 */
static bool ds18b20_ping(bsp_io_port_pin_t pin)
{
    uint32_t primask;
    bool     present;

    if (!ds_pin_is_valid(pin))
    {
        return false;
    }

    ds_critical_begin(&primask);
    present = ds_reset(pin);
    ds_critical_end(primask);

    return present;
}

/* 复位 → Skip ROM → Convert T，让器件开始一次 12 位温度转换。 参数：pin —— 数据引脚。 时序/边界：阻塞约 1.5 ms（复位 + 2 字节写）。
 * 注意：① 命令发出后要等 ≥750 ms 再读（12 位转换时间），本驱动由 非阻塞状态机等 800 ms，不在本函数里等； ② 转换期间总线可以空闲（器件自己测），主机可去干别的。 */
static bool ds18b20_request_convert(bsp_io_port_pin_t pin)
{
    uint32_t primask;
    bool     ok;

    if (!ds_pin_is_valid(pin))
    {
        return false;
    }

    ds_critical_begin(&primask);

    ok = ds_reset(pin);
    if (ok)
    {
        ds_write_byte(pin, DS_CMD_SKIP_ROM);   /* 总线上只有一个器件 */
        ds_write_byte(pin, DS_CMD_CONVERT_T);  /* 启动温度转换 */
    }

    ds_critical_end(primask);

    return ok;
}

/* 复位 → Skip ROM → Read Scratchpad → 读 9 字节 → 校验 → 换算成 ℃。 参数：pin        —— 数据引脚； p_temp_c   —— 输出温度（℃，成功时才写）。
 * 返回：true=读到有效温度；false=无器件/CRC 错/全 0/全 FF/超范围/85 ℃。 时序/边界：阻塞约 5~6 ms（复位 + 2 字节写 + 9 字节读），全程关中断。
 * 注意：① 三层检查缺一不可，原因见文件头"实验 2 真实踩坑"； ② 失败时不修改 *p_temp_c，调用方据此保留上一次有效值； ③ 85.0 ℃ 是上电默认值/转换未完成标志，本工程水温不可能出现。 */
static bool ds18b20_read_temperature(bsp_io_port_pin_t pin, float *p_temp_c)
{
    uint8_t  sp[DS_SCRATCHPAD_LEN];
    uint32_t primask;
    bool     ok = false;

    if ((NULL == p_temp_c) || (!ds_pin_is_valid(pin)))
    {
        return false;
    }

    ds_critical_begin(&primask);

    if (ds_reset(pin))
    {
        ds_write_byte(pin, DS_CMD_SKIP_ROM);
        ds_write_byte(pin, DS_CMD_READ_SCRATCHPAD);

        for (uint8_t i = 0U; i < DS_SCRATCHPAD_LEN; i++)
        {
            sp[i] = ds_read_byte(pin);
        }

        /* ---- 检查 ②：全 0 / 全 FF 判无效（先做，最便宜的检查）----
         * DQ 被拉死时 9 字节可能全是 0x00，且 8 个 0 的 CRC 也正好是 0，
         * CRC 会"通过"——只靠 CRC 挡不住这种假值。 */
        uint8_t sp_or  = 0U;
        uint8_t sp_and = 0xFFU;
        for (uint8_t i = 0U; i < (DS_SCRATCHPAD_LEN - 1U); i++)
        {
            sp_or  |= sp[i];
            sp_and &= sp[i];
        }

        /* ---- 检查 ①：CRC8（保护真实数据不被传输误码带偏）---- */
        if ((0U != sp_or) && (0xFFU != sp_and) &&
            (ds_crc8(sp, (uint8_t) (DS_SCRATCHPAD_LEN - 1U)) == sp[8]))
        {
            /* 温度原始值：高字节在前，负温度为补码 */
            int16_t raw = (int16_t) (((uint16_t) sp[1] << 8U) | (uint16_t) sp[0]);
            float   t   = ((float) raw) / DS_TEMP_LSB_PER_C;

            /* ---- 检查 ③：范围 + 85 ℃ 上电默认值 ----
             * -20~80 ℃ 是"洗手水场景"的物理可信区间；85 ℃ 落在区间外，
             * 这里再显式判一次（防止将来有人放宽区间把假值放进来）。 */
            if ((t > DS18B20_TEMP_MIN_C) && (t < DS18B20_TEMP_MAX_C) &&
                (sensor_absf(t - DS18B20_POWER_ON_TEMP_C) > 0.01f))
            {
                *p_temp_c = t;
                ok = true;
            }
        }
    }

    ds_critical_end(primask);

    return ok;
}

/* ---------------------------- 内部：兜底温度策略 ---------------------------- */
/* 按"手输值优先，否则默认 25.0 ℃"设置 temp_c / src。 参数：p_dev —— 句柄（非 NULL）。 时序/边界：纯赋值。
 * 注意：绝不沿用"上一次实测值"作为兜底：传感器已经不可信了，继续用旧读数 会让温补看起来还在工作（隐蔽失效），不如明确标 FIX/MAN 让上层知道 "温补精度下降"。sensor_temp_c 保留最近一次有效值，仅供诊断查看。 */
static void ds18b20_apply_fallback(ds18b20_t *p_dev)
{
    if (p_dev->manual_valid)
    {
        p_dev->temp_c = p_dev->manual_temp_c;
        p_dev->src    = DS18B20_SRC_MANUAL;
    }
    else
    {
        p_dev->temp_c = p_dev->cfg.default_temp_c;
        p_dev->src    = DS18B20_SRC_FIX;
    }
}

/* ---------------------------- 公共接口：初始化 / 自检 / 任务 ---------------------------- */
/* 保存配置（带合法性检查）、清零运行状态、把数据脚配成"输入 + 上拉"。
 * 参数：p_dev —— 句柄（app_main 分配，必须非 NULL 才生效）； p_cfg —— 配置；NULL 表示全部用默认值（P500、2 s/800 ms/5 s/3 次/25 ℃）。
 * 时序/边界：几条寄存器写（R_IOPORT_PinCfg），不阻塞、不访问总线。
 * 注意：① 每个配置项都有范围检查，非法值静默回落默认值（防止上层写错参数 导致任务空转或周期为 0）； ② 初始化不检测器件（检测在 boot_test / task 里做），避免上电被打断； ③ 引脚内部上拉只是"备用"，外部 4.7 kΩ 必须焊/接好。 */
void ds18b20_init(ds18b20_t *p_dev, const ds18b20_cfg_t *p_cfg)
{
    if (NULL == p_dev)
    {
        return;
    }

    /* ---- 1) 先填默认值（来源：头文件宏，注释见文件头参数表）---- */
    p_dev->cfg.pin                = DS18B20_PIN_DEFAULT;
    p_dev->cfg.request_period_ms  = DS18B20_REQUEST_PERIOD_MS;
    p_dev->cfg.conv_wait_ms       = DS18B20_CONV_WAIT_MS;
    p_dev->cfg.redetect_period_ms = DS18B20_REDETECT_PERIOD_MS;
    p_dev->cfg.fail_limit         = (uint8_t) DS18B20_FAIL_LIMIT;
    p_dev->cfg.default_temp_c     = DS18B20_DEFAULT_TEMP_C;

    /* ---- 2) 用调用者的合法值覆盖 ---- */
    if (NULL != p_cfg)
    {
        if (ds_pin_is_valid(p_cfg->pin))
        {
            p_dev->cfg.pin = p_cfg->pin;
        }
        if (p_cfg->request_period_ms >= 100U)   /* 周期太短会挤占主循环 */
        {
            p_dev->cfg.request_period_ms = p_cfg->request_period_ms;
        }
        if (p_cfg->conv_wait_ms >= 750U)        /* 12 位转换最少 750 ms */
        {
            p_dev->cfg.conv_wait_ms = p_cfg->conv_wait_ms;
        }
        if (p_cfg->redetect_period_ms >= 100U)
        {
            p_dev->cfg.redetect_period_ms = p_cfg->redetect_period_ms;
        }
        if (p_cfg->fail_limit >= 1U)
        {
            p_dev->cfg.fail_limit = p_cfg->fail_limit;
        }
        if ((p_cfg->default_temp_c >= DS18B20_TEMP_MIN_C) &&
            (p_cfg->default_temp_c <= DS18B20_TEMP_MAX_C))
        {
            p_dev->cfg.default_temp_c = p_cfg->default_temp_c;
        }
    }

    /* ---- 3) 运行状态清零 ---- */
    p_dev->present         = false;
    p_dev->disabled        = false;
    p_dev->conv_pending    = false;
    p_dev->manual_valid    = false;
    p_dev->manual_temp_c   = p_dev->cfg.default_temp_c;
    p_dev->sensor_temp_c   = p_dev->cfg.default_temp_c;
    p_dev->temp_c          = p_dev->cfg.default_temp_c;
    p_dev->src             = DS18B20_SRC_FIX;
    p_dev->conv_start_ms   = 0U;
    p_dev->last_detect_ms  = 0U;
    p_dev->fail_streak     = 0U;
    p_dev->error_count     = 0U;

    /* ---- 4) 数据脚：输入 + 上拉（1-Wire 空闲态）----
     * R_IOPORT_PinCfg 会处理 PFS 写保护；此处不需要手动 PinAccessEnable。
     * 注意：必须配置成 GPIO 输入（PMR=0），否则 PFS 写 PODR/PDR 不生效。 */
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, p_dev->cfg.pin,
                           (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT |
                           (uint32_t) IOPORT_CFG_PULLUP_ENABLE);
}

/* 检测器件 → 发起一次转换 → 等待 → 试读一个有效温度； 只有"有应答 + 读到有效温度"才算可用，否则自动进入 disabled 状态。
 * 时序/边界：ping 约 1 ms；若无器件立即返回（不白等 800 ms）；若有器件则转换等待 conv_wait_ms（默认 800 ms），总阻塞 ≈800 ms。
 * 注意：① 为什么必须"试读一次"而不只看复位应答：DQ 被拉低时复位应答可能为真， 但暂存器全 0（实验 2 实际踩过）——必须读到有效温度才算可用； ② 失败后 disabled=true：主循环不会反复重试假 0.00 ℃，等命令 C/手输 T； ③ 本函数是唯一阻塞的驱动函数，只在上电时调用一次，不影响 50 ms 主循环。 */
bool ds18b20_boot_test(ds18b20_t *p_dev)
{
    uint32_t now_ms;
    float    t;

    if (NULL == p_dev)
    {
        return false;
    }

    now_ms = timebase_ms();

    /* 复位状态：上电自检要从"干净"状态开始（可重复调用，例如 app 重新初始化） */
    p_dev->present      = false;
    p_dev->conv_pending = false;
    p_dev->fail_streak  = 0U;

    /* ---- 1) 先看有没有器件；没有就不白等 800 ms ---- */
    if (!ds18b20_ping(p_dev->cfg.pin))
    {
        p_dev->disabled       = true;
        p_dev->last_detect_ms = now_ms;
        ds18b20_apply_fallback(p_dev);
        return false;
    }

    /* ---- 2) 有应答后再试读一次有效温度（挡住"应答真、数据假"）---- */
    if (ds18b20_request_convert(p_dev->cfg.pin))
    {
        ds_delay_ms(p_dev->cfg.conv_wait_ms);   /* 12 位转换最长 750 ms，默认等 800 ms */

        if (ds18b20_read_temperature(p_dev->cfg.pin, &t))
        {
            p_dev->present       = true;
            p_dev->disabled      = false;
            p_dev->sensor_temp_c = t;
            p_dev->temp_c        = t;
            p_dev->src           = DS18B20_SRC_SENSOR;
            p_dev->conv_start_ms = timebase_ms();   /* 下一次转换从自检结束时刻起算 */
            p_dev->last_detect_ms = now_ms;
            return true;
        }
    }

    /* ---- 3) 有应答但读不到有效温度：停用并回退 ---- */
    p_dev->present        = false;
    p_dev->disabled       = true;
    p_dev->last_detect_ms = now_ms;
    ds18b20_apply_fallback(p_dev);

    return false;
}

/* 每 50 ms 由主循环调用一次，推进"发起转换 → 等待 → 读数 → 失败计数/ 回退/停用/重试发现"的完整状态机；命令 C 通过 force_recheck 强制重检。
 * 时序/边界：不阻塞（最长的 1-Wire 事务约 6 ms，属于可接受的短临界区）。
 * 注意：① 状态优先级：force_recheck > 停用/不在线 > 正常转换流程； ② 停用后不自动重试（防假温度污染数据）；但"从未检测到"的情况下 每 redetect_period_ms 重试一次（可能是没插紧/热插拔）； ③ 短时读失败保留最近有效值（src 仍为 DS），连续 fail_limit 次才停用； ④ now_ms 必须来自 timebase_ms()，且与调用周期一一对应（本工程 50 ms）。 */
void ds18b20_task(ds18b20_t *p_dev, uint32_t now_ms, bool force_recheck)
{
    if (NULL == p_dev)
    {
        return;
    }

    /* ---- 0) 命令 C：无条件重新检测（清停用/失败计数）---- */
    if (force_recheck)
    {
        p_dev->disabled       = false;
        p_dev->fail_streak    = 0U;
        p_dev->conv_pending   = false;
        p_dev->last_detect_ms = now_ms;
        p_dev->present        = ds18b20_ping(p_dev->cfg.pin);

        if (p_dev->present)
        {
            /* 刚重新发现：把发起时刻往回拨一个周期，下一 tick 立刻开始转换 */
            p_dev->conv_start_ms = now_ms - p_dev->cfg.request_period_ms;
        }
    }

    /* ---- 1) 器件不在线 / 已停用：用兜底温度，处理周期性重试 ---- */
    if ((!p_dev->present) || p_dev->disabled)
    {
        ds18b20_apply_fallback(p_dev);

        /* 自动停用后不再自动 ping（避免假 0.00 ℃ 反复污染数据）；
         * 只有"从未检测到"（disabled=false, present=false）才周期性重试。 */
        if ((!p_dev->disabled) &&
            ((now_ms - p_dev->last_detect_ms) >= p_dev->cfg.redetect_period_ms))
        {
            p_dev->last_detect_ms = now_ms;

            if (ds18b20_ping(p_dev->cfg.pin))
            {
                p_dev->present       = true;
                p_dev->conv_pending  = false;
                p_dev->fail_streak   = 0U;
                p_dev->conv_start_ms = now_ms - p_dev->cfg.request_period_ms;
            }
        }

        return;
    }

    /* ---- 2) 空闲：到周期就发起一次转换（只发命令，不等待）---- */
    if (!p_dev->conv_pending)
    {
        if ((now_ms - p_dev->conv_start_ms) >= p_dev->cfg.request_period_ms)
        {
            if (ds18b20_request_convert(p_dev->cfg.pin))
            {
                p_dev->conv_pending  = true;
                p_dev->conv_start_ms = now_ms;
            }
            else
            {
                /* 命令阶段就丢了应答：按掉线处理，回退兜底，5 s 后重试发现 */
                p_dev->present        = false;
                p_dev->last_detect_ms = now_ms;
                ds18b20_apply_fallback(p_dev);
            }
        }

        return;
    }

    /* ---- 3) 转换等待中：到时读数，做三层有效性检查，处理失败计数 ---- */
    if ((now_ms - p_dev->conv_start_ms) >= p_dev->cfg.conv_wait_ms)
    {
        float t;

        if (ds18b20_read_temperature(p_dev->cfg.pin, &t))
        {
            p_dev->sensor_temp_c = t;
            p_dev->temp_c        = t;
            p_dev->src           = DS18B20_SRC_SENSOR;
            p_dev->fail_streak   = 0U;
            p_dev->present       = true;
        }
        else
        {
            p_dev->error_count++;
            p_dev->fail_streak++;

            if (p_dev->fail_streak >= p_dev->cfg.fail_limit)
            {
                /* 连续多次无效：停用，回退兜底，等命令 C/手输 T 恢复。
                 * fail_streak 清零，避免恢复后又立刻触发停用判定。 */
                p_dev->fail_streak = 0U;
                p_dev->present     = false;
                p_dev->disabled    = true;
                ds18b20_apply_fallback(p_dev);
            }
        }

        p_dev->conv_pending = false;   /* 无论成功失败，本次转换周期结束 */
    }
}

/* 保存手输温度并立即生效（src=MAN）；同时清除停用标志、触发重检。 参数：p_dev  —— 句柄； temp_c —— 手输温度（℃，NaN 会回落默认值，超范围会限幅）。
 * 时序/边界：纯赋值 + timebase 读一次。
 * 注意：① 手输值只在 DS18B20 不可用时才会被采用；传感器恢复后会自动切回 DS； ② 用户输入 T 往往是因为"刚修好线/刚插好"，所以顺手清 disabled 并把 last_detect_ms 往回拨一个重试周期，下一 tick 立刻 ping 一次； ③ 输入范围在驱动里限幅到 [-20,80]，app 层再按协议限 0~60 ℃。 */
void ds18b20_set_manual_temp(ds18b20_t *p_dev, float temp_c)
{
    float t;

    if (NULL == p_dev)
    {
        return;
    }

    t = temp_c;
    if (sensor_is_nan_f32(t))         /* NaN：输入解析异常时不要污染温度链路 */
    {
        t = p_dev->cfg.default_temp_c;
    }
    t = sensor_clampf(t, DS18B20_TEMP_MIN_C, DS18B20_TEMP_MAX_C);

    p_dev->manual_temp_c = t;
    p_dev->manual_valid  = true;
    p_dev->temp_c        = t;
    p_dev->src           = DS18B20_SRC_MANUAL;

    /* 清停用标志，让下一轮 task 重新尝试检测（"等待下一轮检测"） */
    p_dev->disabled       = false;
    p_dev->fail_streak    = 0U;
    p_dev->conv_pending   = false;
    p_dev->last_detect_ms = timebase_ms() - p_dev->cfg.redetect_period_ms;
}

/* ---------------------------- 公共接口：只读 getter ---------------------------- */
/* 读取当前采用的温度 参数：p_dev —— 句柄（可为 NULL）。 返回：当前采用温度 ℃（实测/手输/兜底）；p_dev 为 NULL 时返回默认 25.0 ℃。 时序/边界：纯读。
 * 注意：调用者不需要关心来源；需要区分来源时用 ds18b20_get_src()。 */
float ds18b20_get_temp_c(const ds18b20_t *p_dev)
{
    if (NULL == p_dev)
    {
        return DS18B20_DEFAULT_TEMP_C;
    }

    return p_dev->temp_c;
}

/* 读取当前温度来源 参数：p_dev —— 句柄（可为 NULL）。 返回：DS18B20_SRC_FIX / MANUAL / SENSOR；NULL 时 FIX。 时序/边界：纯读。
 * 注意：CSV 的 src 列（DS/MAN/FIX）与温补精度说明都依赖这个值。 */
ds18b20_src_t ds18b20_get_src(const ds18b20_t *p_dev)
{
    if (NULL == p_dev)
    {
        return DS18B20_SRC_FIX;
    }

    return p_dev->src;
}

/* 判断"最近一次成功读数是否真来自 DS18B20"（供 app 决定是否提示温补降级）。 参数：p_dev —— 句柄（可为 NULL）。 时序/边界：纯读。
 * 注意：刚重新检测到器件但还没读到有效温度时返回 false（src 仍是兜底来源）。 */
bool ds18b20_sensor_ok(const ds18b20_t *p_dev)
{
    if (NULL == p_dev)
    {
        return false;
    }

    return (p_dev->present) &&
           (!p_dev->disabled) &&
           (DS18B20_SRC_SENSOR == p_dev->src);
}

/* CSV/日志打印用（DS / MAN / FIX）。 返回：静态字符串常量（绝不返回 NULL；未知值返回 "FIX"）。 时序/边界：纯计算（switch）。 注意：返回的指针不要 free、不要写入。
 */
const char *ds18b20_src_str(ds18b20_src_t src)
{
    switch (src)
    {
        case DS18B20_SRC_SENSOR:
            return "DS";
        case DS18B20_SRC_MANUAL:
            return "MAN";
        case DS18B20_SRC_FIX:
            return "FIX";
        default:
            return "FIX";
    }
}
