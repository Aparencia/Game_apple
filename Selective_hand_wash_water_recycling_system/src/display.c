/*
 * display.c
 *
 *  Created on: 2026年10月30日
 *      Author: Administrator
 */

/*
 * display.c —— 判别/执行结果 → 板载 0.96" OLED 三页状态页（128x64，I2C0）
 * ---------------------------------------------------------------------------
 * v1.3.0（2026-11-13）升级：从"单页、只有清水/污水两个大字"扩展为三页现场指示：
 *
 *   P1 水质页（原 v1.1 布局，右上角加页码 1/3）
 *     大字：清水 / 污水（反显）/ 待机 / 冲洗 / 故障；
 *     状态串："CLEAR  RECOVER" 等；数值串："dEC+12mV g1.008 N+5"。
 *
 *   P2 执行器页（v1.3.0 新增：大字显示阀的实际状态，满足"状态只有清水/污水不够"）
 *     大字：回收（反显）/ 排空 / 手动（反显）/ 故障 / 待机；
 *     第二行："RECOVER  RLY:ON" / "DRAIN    RLY:OFF"（继电器实际状态）；
 *     第三行："AUTO CNT:12 ON:5.2s"（模式/吸合次数/本次通电时长）。
 *
 *   P3 详情页（v1.3.0 新增：8 行 6x8 小字）
 *     事件状态/时长、STATE/LEVEL、VALVE/RLY、MODE/CNT、dEC/g、dNTU/水温、
 *     EC0/σ、d1/d2 阈值（每行最多 21 字符）。
 *
 * 切页方式：板载 K2 短按循环 P1→P2→P3→P1（display_page_next 由 app_main 的按键
 * 任务调用）；长按 K2 仍是重采基线；串口 M 仍是打 MARK。默认停在 P1，不自动轮播
 * （演示时由人控制节奏；屏只是现场指示器，不是安全路径）。
 *
 * 刷新模型（与 v1.1 相同 + 分页）：
 *   - 页面内容变化立即重画对应区域；数值（P1 页头/数值行、P3 数值行）最快 200 ms；
 *   - 1 s 整屏心跳（掉帧/残影自恢复，也是"屏还活着"的证据）；
 *   - 只刷脏页（oled_flush 内部实现），一页约 3 ms@400 kHz，整屏约 25 ms；
 *   - 切页后下一 tick 整屏重画一次。
 *
 * 显示不是安全路径：任何 I2C 失败只返回 false，不打印、不死机、不 assert；
 * 连续 3 帧失败后 display_task 返回 false，由 app_main 打印一次 WARN 后继续用串口。
 * 不动态内存、不浮点 printf：所有数值先手工换算成整数再格式化。
 *
 * 刷新数据来源：
 *   pipeline_out_t：EC0/σ/水温/ΔEC/g/ΔNTU%/note；
 *   decision_out_t：state/level/action/event_active/d1/d2；
 *   valve_out_t   ：position/relay_on/switch_count/on_time_ms/request_reason/pin_fault。
 */

#include "display.h"
#include "app_main.h"   /* app_autocal_text：开机自检阶段显示（v2.1） */

#include <stddef.h>
#include <stdio.h>    /* snprintf：只用整数格式（禁浮点 printf，见文件头） */
#include <string.h>

#include "oled.h"

/* ============================ 常量 ============================ */

/* 页面布局（全部 8 像素对齐；改这里必须同步 16 文档的页面图） */
#define DISP_HEAD_X       0U
#define DISP_HEAD_Y       0U
#define DISP_BIG_X0       32U    /* 第一个大字左上角 x（两个 32x32 共 64 宽，居中） */
#define DISP_BIG_X1       64U    /* 第二个大字左上角 x */
#define DISP_BIG_Y        8U
#define DISP_LINE2_X      0U
#define DISP_LINE2_Y      40U
#define DISP_LINE3_X      0U
#define DISP_LINE3_Y      56U

/* 大字带宽高：x=0..127、y=8..39（含左右留白），重画时整带清黑，
 * 这样从开机页/其它页切过来时不会残留旧字。 */
#define DISP_BIG_BAND_H   32U

/* 页码画在大字带右上角（x=98 起，3 个 6x8 字符 = 18 px，不与 x=96 结束的大字重叠）。 */
#define DISP_PAGENO_X     98U
#define DISP_PAGENO_Y     8U

/* 文本缓冲：按最长内容留 1 字节余量（页头 19、状态串 15、数值串 20、详情行 21）。 */
#define DISP_R0_BUF       24U
#define DISP_R1_BUF       20U
#define DISP_R2_BUF       24U
#define DISP_ROW_BUF      24U
#define DISP_INFO_ROWS    8U

/* 内容类型：大字页（P1/P2）或纯文字页（P3） */
#define DISP_KIND_BIG     0U
#define DISP_KIND_TEXT    1U

/* 连续失败帧数达到该值 → display_task 返回 false（app 只报一次 WARN）。 */
#define DISPLAY_FAIL_LIMIT  3U

/* ============================ 内部类型 ============================ */

/* 一页的"目标内容"：先算出来，再与上一屏比较，只重画变化的区域。 */
typedef struct st_disp_page
{
    uint8_t page;                     /* 页号 1..3（日志/页码用） */
    uint8_t kind;                     /* DISP_KIND_BIG / DISP_KIND_TEXT */
    uint8_t big0;                     /* 左大字：oled_hz32_id_t 的数值（kind=BIG） */
    uint8_t big1;                     /* 右大字 */
    bool    inverse;                  /* true=反显（白底黑字） */
    char    r0[DISP_R0_BUF];          /* 6x8 页头（BIG 页） */
    char    r1[DISP_R1_BUF];          /* 8x16 第二行（BIG 页） */
    char    r2[DISP_R2_BUF];          /* 6x8 第三行（BIG 页） */
    char    rows[DISP_INFO_ROWS][DISP_ROW_BUF];  /* 8 行 6x8（TEXT 页） */
} disp_page_t;

/* ============================ 内部状态 ============================ */

/* 上一页实际画出的内容（用于差异刷新）。 */
static char    g_shown_r0[DISP_R0_BUF];
static char    g_shown_r1[DISP_R1_BUF];
static char    g_shown_r2[DISP_R2_BUF];
static char    g_shown_rows[DISP_INFO_ROWS][DISP_ROW_BUF];
static uint8_t g_shown_big0;
static uint8_t g_shown_big1;
static bool    g_shown_inverse;
static uint8_t g_shown_page;         /* 0=还没画过状态页 */
static uint8_t g_shown_kind;
static bool    g_shown_once;         /* 是否已经画过状态页（false=开机页还在屏上） */

/* 当前页（0..2）与强制整屏重画标志（切页/K2 触发）。 */
static uint8_t g_page;
static bool    g_force_full;

/* 时序/节流 */
static bool     g_first_seen;        /* 是否已记录第一次调用的 now_ms */
static uint32_t g_first_ms;          /* 第一次调用时刻（开机页保持计时基准） */
static uint32_t g_last_num_ms;       /* 上次刷新数值行的时刻 */
static uint32_t g_last_full_ms;      /* 上次整屏刷新的时刻（心跳） */
static uint8_t  g_fail_count;        /* 连续刷新失败帧数 */

/* 事件计时（P3 显示"EV:ON x.xs"用；由 event_active 上升沿开始计时） */
static bool     g_event_active;
static bool     g_event_prev_active;
static uint32_t g_event_start_ms;

/* ============================ 小工具 ============================ */

/*
 * 函数名：disp_round_i
 * 作用：浮点四舍五入到 int32（对称舍入，负值也按绝对值进舍）。
 * 参数：v —— 待舍入值（本工程量级远小于 int32 范围，不做溢出保护）。
 * 返回：舍入后的整数。
 * 时序/边界：纯计算，无副作用。
 * 注意：不用 lroundf/roundf（会拉进 libm，本工程禁 libm）。
 */
static int32_t disp_round_i(float v)
{
    if (v >= 0.0f)
    {
        return (int32_t)(v + 0.5f);
    }
    return (int32_t)(v - 0.5f);
}

/*
 * 函数名：disp_copy
 * 作用：安全字符串拷贝（带长度上限，保证 '\0' 结尾）。
 * 参数：p_dst/cap —— 目标与容量；p_src —— 源（NULL 视为空串）。
 * 返回：无。
 * 时序/边界：O(n)；截断而不是溢出。
 * 注意：本工程字符串都是静态字面量或固定缓冲，不会出现超长动态串。
 */
static void disp_copy(char *p_dst, uint32_t cap, const char *p_src)
{
    uint32_t i = 0U;

    if ((NULL == p_dst) || (0U == cap))
    {
        return;
    }

    while ((NULL != p_src) && ('\0' != *p_src) && ((i + 1U) < cap))
    {
        p_dst[i] = *p_src;
        i++;
        p_src++;
    }
    p_dst[i] = '\0';
}

/*
 * 函数名：disp_fmt_temp
 * 作用：把水温格式化成一位小数（如 "25.3"、"-3.0"），不带单位。
 * 参数：p_buf/cap —— 输出缓冲；temp_c —— 水温（℃）。
 * 返回：无。
 * 时序/边界：缓冲 ≥ 7 字节（"-99.9" + '\0'）。
 * 注意：负温度需要保留符号；本工程水温正常在 5~40 ℃，但 DS18B20 故障/冬天也不能
 *       让符号位丢失（否则 -0.5 ℃ 会显示成 0.5 ℃，温补方向就反了）。
 */
static void disp_fmt_temp(char *p_buf, uint32_t cap, float temp_c)
{
    int32_t  t10;
    uint32_t a;
    uint32_t whole;
    uint32_t i = 0U;

    if ((NULL == p_buf) || (0U == cap))
    {
        return;
    }

    t10 = disp_round_i(temp_c * 10.0f);
    if (t10 > 9999)
    {
        t10 = 9999;       /* 显示上限 999.9 ℃（实际水温不可能到，纯防格式化溢出） */
    }
    if (t10 < -9999)
    {
        t10 = -9999;
    }
    a     = (uint32_t)((t10 < 0) ? (-t10) : t10);
    whole = a / 10U;      /* 整数部分（0~999） */
    if (cap < 5U)
    {
        p_buf[0] = '\0';
        return;
    }

    /* 手工拼 "[-]<whole>.<一位小数>"：不用 snprintf，避免 -Wformat-truncation */
    if (t10 < 0)
    {
        p_buf[i] = '-';
        i++;
    }
    if (whole >= 100U)
    {
        p_buf[i] = (char)('0' + (whole / 100U));
        i++;
        whole %= 100U;
        p_buf[i] = (char)('0' + (whole / 10U));
        i++;
        p_buf[i] = (char)('0' + (whole % 10U));
        i++;
    }
    else if (whole >= 10U)
    {
        p_buf[i] = (char)('0' + (whole / 10U));
        i++;
        p_buf[i] = (char)('0' + (whole % 10U));
        i++;
    }
    else
    {
        p_buf[i] = (char)('0' + whole);
        i++;
    }
    p_buf[i] = '.';
    i++;
    p_buf[i] = (char)('0' + (a % 10U));
    i++;
    p_buf[i] = '\0';
}

/*
 * 函数名：disp_fmt_signed_dec
 * 作用：把"×10 缩放"的浮点格式化成带符号一位小数（如 "+5.0"、"-0.5"、"0.0"）。
 * 参数：p_buf/cap —— 输出；value —— 原值（内部×10 取整）。
 * 返回：无。
 * 时序/边界：缓冲 ≥ 8 字节；值域受传感器量程限制，不会溢出。
 * 注意：仅用于显示 dNTU%/变化率这类小量；正值带 '+'，零显示 "0.0"。
 */
static void disp_fmt_signed_dec(char *p_buf, uint32_t cap, float value)
{
    int32_t  x10;
    char     sign;
    uint32_t a;
    uint32_t whole;
    uint32_t frac;

    if ((NULL == p_buf) || (cap < 8U))
    {
        return;
    }

    x10 = disp_round_i(value * 10.0f);
    if (x10 > 99999)
    {
        x10 = 99999;
    }
    if (x10 < -99999)
    {
        x10 = -99999;
    }
    if (x10 < 0)
    {
        sign = '-';
        a    = (uint32_t)(-x10);
    }
    else if (x10 > 0)
    {
        sign = '+';
        a    = (uint32_t)x10;
    }
    else
    {
        sign = ' ';
        a    = 0U;
    }
    whole = a / 10U;
    frac  = a % 10U;
    (void)snprintf(p_buf, (size_t)cap, "%c%lu.%lu", sign,
                   (unsigned long)whole, (unsigned long)frac);
}

/*
 * 函数名：disp_fmt_header
 * 作用：格式化 P1 页头 "EC0=<4位mV> s<σ.一位> T<温度.一位>"；无基线时用 ---- / --- 占位。
 * 参数：p_buf/cap —— 输出；p_out —— 流水线输出。
 * 返回：无。
 * 时序/边界：缓冲 ≥ 22 字节。
 * 注意：EC0 用整数 mV（四舍五入），与 CSV 的 ec0 字段同源；σ 只显示一位小数，
 *       够现场判断"噪声大不大"，精确值看串口 CSV。
 */
static void disp_fmt_header(char *p_buf, uint32_t cap, const pipeline_out_t *p_out)
{
    char    tbuf[8];
    int32_t sig10;

    if ((NULL == p_buf) || (0U == cap) || (NULL == p_out))
    {
        return;
    }

    disp_fmt_temp(tbuf, (uint32_t)sizeof(tbuf), p_out->water_temp_c);

    if (p_out->baseline_valid)
    {
        sig10 = disp_round_i(p_out->ec_sigma_mv * 10.0f);
        if (sig10 < 0)
        {
            sig10 = 0;
        }
        if (sig10 > 999)
        {
            sig10 = 999;   /* 显示上限 99.9 mV，够用；真这么大说明探头/流动池有问题 */
        }
        (void)snprintf(p_buf, (size_t)cap, "EC0=%4u s%u.%u T%s",
                       (unsigned)disp_round_i(p_out->ec0_mv25),
                       (unsigned)((uint32_t)sig10 / 10U),
                       (unsigned)((uint32_t)sig10 % 10U),
                       tbuf);
    }
    else
    {
        (void)snprintf(p_buf, (size_t)cap, "EC0=---- s--- T%s", tbuf);
    }
}

/*
 * 函数名：disp_fmt_numbers
 * 作用：格式化 P1 数值串 "dEC%+dmV g%d.%03d N%+d"（例："dEC+12mV g1.008 N+5"）。
 * 参数：p_buf/cap —— 输出；p_out —— 流水线输出。
 * 返回：无。
 * 时序/边界：缓冲 ≥ 22 字节；N 限幅到 ±99（6x8 一行最多 21 字符）。
 * 注意：g 用"千分位整数"打印（g_milli=1008 → "g1.008"），避免浮点 printf；
 *       ΔEC 用四舍五入整数 mV（与 08 文档记录表的量级一致）。
 */
static void disp_fmt_numbers(char *p_buf, uint32_t cap, const pipeline_out_t *p_out)
{
    int32_t d_ec;
    int32_t g_milli;
    int32_t n;

    if ((NULL == p_buf) || (0U == cap) || (NULL == p_out))
    {
        return;
    }

    d_ec    = disp_round_i(p_out->d_ec_mv);
    g_milli = disp_round_i(p_out->g_ec * 1000.0f);
    n       = disp_round_i(p_out->d_ntu_pct);

    /* 限幅保证整行 ≤21 字符（6x8）：
     * "dEC%+dmV g%d.%03d N%+d" 最坏 "dEC+999mV g9.999 N+99" = 21 字符。 */
    if (d_ec > 999)
    {
        d_ec = 999;
    }
    if (d_ec < -999)
    {
        d_ec = -999;
    }
    if (g_milli < 0)
    {
        g_milli = 0;           /* g 理论上 >0；异常值兜底，防止出现 "g-1.000" */
    }
    if (g_milli > 9999)
    {
        g_milli = 9999;        /* g 显示上限 9.999（防止一行超长） */
    }
    if (n > 99)
    {
        n = 99;
    }
    if (n < -99)
    {
        n = -99;
    }

    (void)snprintf(p_buf, (size_t)cap, "dEC%+dmV g%d.%03d N%+d",
                   (int)d_ec, (int)(g_milli / 1000), (int)(g_milli % 1000), (int)n);
}

/*
 * 函数名：disp_fmt_note
 * 作用：把流水线 note（ADC_TIMEOUT / TDS_RAIL / ...）拼成 "note=xxx" 供故障页显示。
 * 参数：p_buf/cap —— 输出；p_note —— note 字符串（可能为 NULL）。
 * 返回：无。
 * 时序/边界：手动逐字符拼，不用 "%.13s"（避免 -Wformat-truncation 告警）。
 * 注意：note 已由 pipeline 限长 28 字节，这里再按输出缓冲截断。
 */
static void disp_fmt_note(char *p_buf, uint32_t cap, const char *p_note)
{
    static const char prefix[] = "note=";
    uint32_t i = 0U;

    if ((NULL == p_buf) || (0U == cap))
    {
        return;
    }

    while ((prefix[i] != '\0') && ((i + 1U) < cap))
    {
        p_buf[i] = prefix[i];
        i++;
    }
    while ((NULL != p_note) && ('\0' != *p_note) && ((i + 1U) < cap))
    {
        p_buf[i] = *p_note;
        i++;
        p_note++;
    }
    p_buf[i] = '\0';
}

/* ============================ 判断/分页小工具 ============================ */

/*
 * 函数名：disp_is_fault
 * 作用：判断本 tick 是否处于"传感器故障"（与 decision.c 的安全侧判据完全一致）。
 * 参数：p_out —— 流水线输出。
 * 返回：true=故障（ADC 超时 / 任一路状态非 OK）。
 * 时序/边界：纯函数。
 * 注意：故障优先于一切显示——即使 state 是 CLEAR 也必须显示"故障"。
 */
static bool disp_is_fault(const pipeline_out_t *p_out)
{
    return p_out->adc_timeout
        || (SENSOR_OK != p_out->tds_status)
        || (SENSOR_OK != p_out->ntu_status);
}

/*
 * 函数名：disp_regime_tag
 * 作用：工况 → 3 字母短码（OLED 行宽有限，P3 用）。
 * 参数：regime —— 工况。
 * 返回：静态字符串 "FAULT"→"FLT" / "DRY" / "SETTLE"→"SET" / "MEASURE"→"MEA" / "LOCKOUT"→"LCK"。
 * 时序/边界：纯函数。
 */
static const char *disp_regime_tag(regime_t regime)
{
    switch (regime)
    {
        case REGIME_FAULT:
            return "FLT";

        case REGIME_DRY:
            return "DRY";

        case REGIME_SETTLE:
            return "SET";

        case REGIME_MEASURE:
            return "MEA";

        case REGIME_LOCKOUT:
        default:
            return "LCK";
    }
}

/* 状态 → 首字母大写、其余小写（OLED 用；L0/L1/L2 等代码不在此函数）。 */
static const char *disp_state_title(decision_state_t state)
{
    switch (state)
    {
        case DEC_STATE_CLEAR:
            return "Clear";

        case DEC_STATE_FLUSH:
            return "Flush";

        case DEC_STATE_UNCERTAIN:
            return "Uncertain";

        case DEC_STATE_SOAP:
            return "Soap";

        case DEC_STATE_WAIT:
        default:
            return "Wait";
    }
}

/* 阀位置 → 首字母大写（OLED 用）。 */
static const char *disp_pos_title(valve_pos_t pos)
{
    return (VALVE_POS_RECOVER == pos) ? "Recover" : "Drain";
}

/* 工况原因 → 首字母大写（OLED 用；避免 "L1 NO_BASE" 全大写）。 */
static const char *disp_gate_reason_title(const char *p_reason)
{
    if (NULL == p_reason)
    {
        return "No base";
    }
    if (0 == strcmp(p_reason, "NO_BASE"))
    {
        return "No base";
    }
    if (0 == strcmp(p_reason, "NO_CAL"))
    {
        return "No cal";
    }
    if (0 == strcmp(p_reason, "SENSOR"))
    {
        return "Sensor";
    }
    if (0 == strcmp(p_reason, "DRY"))
    {
        return "Dry";
    }
    if (0 == strcmp(p_reason, "SETTLE"))
    {
        return "Settle";
    }
    if (0 == strcmp(p_reason, "LOCKOUT"))
    {
        return "Lockout";
    }
    if (0 == strcmp(p_reason, "MEASURE"))
    {
        return "Measure";
    }
    return p_reason;
}

/*
 * 函数名：disp_valve_manual
 * 作用：判断当前阀是否处于"手动覆盖"（V1/V0）状态。
 * 参数：p_valve —— 执行器输出（可为 NULL）。
 * 返回：true=手动；false=自动/无数据/初始化。
 * 时序/边界：纯函数（一次 strcmp）。
 * 注意：request_reason 是静态串（"AUTO"/"MANUAL_RECOVER"/"MANUAL_DRAIN"/"INIT"）；
 *       "INIT" 按自动处理（第一 tick 之前还没请求过）。
 */
static bool disp_valve_manual(const valve_out_t *p_valve)
{
    if ((NULL == p_valve) || (NULL == p_valve->request_reason))
    {
        return false;
    }
    if ((0 == strcmp(p_valve->request_reason, "AUTO")) ||
        (0 == strcmp(p_valve->request_reason, "INIT")))
    {
        return false;
    }
    return true;
}

/*
 * 函数名：disp_manual_tag
 * 作用：手动模式短标签："V1"（强制回收）/ "V0"（强制排空）。
 * 参数：p_valve —— 执行器输出。
 * 返回：静态字符串。
 * 时序/边界：纯函数。
 * 注意：只用于显示；方向以 position/relay 为准。
 */
static const char *disp_manual_tag(const valve_out_t *p_valve)
{
    if ((NULL != p_valve) && (NULL != p_valve->request_reason) &&
        (0 == strcmp(p_valve->request_reason, "MANUAL_RECOVER")))
    {
        return "V1";
    }
    return "V0";
}

/* ============================ 页面内容：P1 水质页 ============================ */

/*
 * 函数名：disp_build_water_page
 * 作用：构建 P1 水质页（沿用 v1.1 的映射与文案，仅多了页码）。
 * 参数：p_out/p_dec —— 输入；p_page —— 输出。
 * 返回：无。
 * 时序/边界：纯计算；每 50 ms tick 调用一次，开销可忽略。
 * 注意：优先级 = 故障 > 无基线 > 判别状态。
 */
static void disp_build_water_page(const pipeline_out_t *p_out, const decision_out_t *p_dec,
                                  disp_page_t *p_page)
{
    p_page->page = 1U;
    p_page->kind = DISP_KIND_BIG;
    disp_fmt_header(p_page->r0, (uint32_t)sizeof(p_page->r0), p_out);

    /* ① 故障：优先于一切（与 decision.c 的 fault 判据一致） */
    if (disp_is_fault(p_out))
    {
        p_page->big0    = (uint8_t)OLED_HZ32_GU;      /* 故 */
        p_page->big1    = (uint8_t)OLED_HZ32_ZHANG;   /* 障 */
        p_page->inverse = false;
        disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "L1 Sensor fault");
        disp_fmt_note(p_page->r2, (uint32_t)sizeof(p_page->r2), p_out->note);
        return;
    }

    /* ② 开机自检（v2.1）：优先于"无基线故障"显示，让现场知道装置在自己标定 */
    {
        char ac[24];

        app_autocal_text(ac, (uint32_t)sizeof(ac));
        if ('\0' != ac[0])
        {
            char hint[24];
            bool calib = (0 == strncmp(ac, "Cal ", 4U))
                      || (0 == strncmp(ac, "Auto calib", 10U));

            /* 湿态采基线时不要显示"待机"：大字改为"校准"，第二行带 n/40 进度 */
            p_page->big0 = (uint8_t)(calib ? OLED_HZ32_JIAO : OLED_HZ32_DAI);
            p_page->big1 = (uint8_t)(calib ? OLED_HZ32_ZHUN : OLED_HZ32_JI);
            disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), ac);
            app_autocal_hint(hint, (uint32_t)sizeof(hint));
            disp_copy(p_page->r2, (uint32_t)sizeof(p_page->r2), hint);
            return;
        }
    }

    /* ③ 工况门控（v2）：非测量工况先按 regime 显示。
     *    旧页面把"没有基线"一律画成故障 + "send B"，空腔/换水时现场会误以为装置坏了；
     *    现在 DRY/SETTLE/LOCKOUT 都画"待机"，只有真的 FAULT（无基线且无法判断/无干态指纹）才画故障。 */
    if (REGIME_DRY == p_dec->regime)
    {
        p_page->big0 = (uint8_t)OLED_HZ32_DAI;    /* 待 */
        p_page->big1 = (uint8_t)OLED_HZ32_JI;     /* 机 */
        disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Dry    No water");
        disp_fmt_numbers(p_page->r2, (uint32_t)sizeof(p_page->r2), p_out);
        return;
    }

    if (REGIME_SETTLE == p_dec->regime)
    {
        p_page->big0 = (uint8_t)OLED_HZ32_DAI;
        p_page->big1 = (uint8_t)OLED_HZ32_JI;
        disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Settle wait 4s");
        disp_fmt_numbers(p_page->r2, (uint32_t)sizeof(p_page->r2), p_out);
        return;
    }

    if (REGIME_LOCKOUT == p_dec->regime)
    {
        p_page->big0 = (uint8_t)OLED_HZ32_DAI;
        p_page->big1 = (uint8_t)OLED_HZ32_JI;
        disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Lockout  check");
        disp_fmt_numbers(p_page->r2, (uint32_t)sizeof(p_page->r2), p_out);
        return;
    }

    if (REGIME_FAULT == p_dec->regime)
    {
        uint16_t n = p_out->baseline_n;

        if ((n > 0U) && (n < DISPLAY_BASELINE_TOTAL))
        {
            /* W 采集进度（旧固件是上电自动采集；v2 只在命令 W 时出现） */
            p_page->big0 = (uint8_t)OLED_HZ32_DAI;    /* 待 */
            p_page->big1 = (uint8_t)OLED_HZ32_JI;     /* 机 */
            (void)snprintf(p_page->r1, (size_t)sizeof(p_page->r1),
                           "Baseline %2u/%u", (unsigned)n,
                           (unsigned)DISPLAY_BASELINE_TOTAL);
            disp_copy(p_page->r2, (uint32_t)sizeof(p_page->r2), "Probe in water");
        }
        else
        {
            p_page->big0 = (uint8_t)OLED_HZ32_GU;
            p_page->big1 = (uint8_t)OLED_HZ32_ZHANG;
            (void)snprintf(p_page->r1, (size_t)sizeof(p_page->r1), "L1 %s",
                           disp_gate_reason_title((NULL != p_dec->gate_reason)
                                                  ? p_dec->gate_reason : "NO_BASE"));
            disp_fmt_note(p_page->r2, (uint32_t)sizeof(p_page->r2), p_out->note);
        }
        return;
    }

    /* ③ MEASURE：按判别状态映射大字/状态串，第三行给数值 */
    disp_fmt_numbers(p_page->r2, (uint32_t)sizeof(p_page->r2), p_out);

    switch (p_dec->state)
    {
        case DEC_STATE_CLEAR:
            p_page->big0 = (uint8_t)OLED_HZ32_QING;   /* 清 */
            p_page->big1 = (uint8_t)OLED_HZ32_SHUI;   /* 水 */
            disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Clear  Recover");
            break;

        case DEC_STATE_UNCERTAIN:
            p_page->big0    = (uint8_t)OLED_HZ32_WU;  /* 污 */
            p_page->big1    = (uint8_t)OLED_HZ32_SHUI;
            p_page->inverse = true;
            disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "L1 Uncertain");
            break;

        case DEC_STATE_SOAP:
            p_page->big0    = (uint8_t)OLED_HZ32_WU;
            p_page->big1    = (uint8_t)OLED_HZ32_SHUI;
            p_page->inverse = true;
            disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "L2 Soap  Drain");
            break;

        case DEC_STATE_FLUSH:
            p_page->big0 = (uint8_t)OLED_HZ32_CHONG;  /* 冲 */
            p_page->big1 = (uint8_t)OLED_HZ32_XI;     /* 洗 */
            disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Flush  Drain");
            break;

        case DEC_STATE_WAIT:
        default:
            p_page->big0 = (uint8_t)OLED_HZ32_DAI;
            p_page->big1 = (uint8_t)OLED_HZ32_JI;
            disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Wait   EV:Off");
            break;
    }
}

/* ============================ 页面内容：P2 执行器页 ============================ */

/*
 * 函数名：disp_build_exec_page
 * 作用：构建 P2 执行器页：大字显示阀的实际状态（回收/排空/手动/故障/待机），
 *       第二行继电器实际状态，第三行模式/动作次数/本次通电时长。
 * 参数：p_out/p_dec/p_valve —— 输入（p_valve 可为 NULL）；p_page —— 输出。
 * 返回：无。
 * 时序/边界：纯计算；p_valve 为 NULL 时显示 "NO VALVE"（旧固件/异常兼容）。
 * 注意：大字反映"实际执行结果"（position），不是判别结论；二者在手动/故障时可能不同。
 */
static void disp_build_exec_page(const pipeline_out_t *p_out, const decision_out_t *p_dec,
                                 const valve_out_t *p_valve, disp_page_t *p_page)
{
    bool     manual;
    bool     recover;
    char     temp[24];

    p_page->page = 2U;
    p_page->kind = DISP_KIND_BIG;
    /* P2 以执行器为主；v2 起额外用 regime 区分"没水"和"没标定"，避免空腔误报故障 */

    if (NULL == p_valve)
    {
        disp_copy(p_page->r0, (uint32_t)sizeof(p_page->r0), "P2/3 Exec no valve");
        p_page->big0 = (uint8_t)OLED_HZ32_GU;
        p_page->big1 = (uint8_t)OLED_HZ32_ZHANG;
        disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Valve no data");
        disp_copy(p_page->r2, (uint32_t)sizeof(p_page->r2), "Check p_valve arg");
        return;
    }

    manual = disp_valve_manual(p_valve);

    /* 页头：页号 + 执行器 + 模式 */
    if (p_valve->pin_fault)
    {
        disp_copy(p_page->r0, (uint32_t)sizeof(p_page->r0), "P2/3 Exec  Mode:Fault");
    }
    else if (manual)
    {
        disp_copy(p_page->r0, (uint32_t)sizeof(p_page->r0), "P2/3 Exec Mode:Manual");
    }
    else
    {
        disp_copy(p_page->r0, (uint32_t)sizeof(p_page->r0), "P2/3 Exec  Mode:Auto");
    }

    /* ① 引脚故障（写/回读失败）：安全侧仍是排空，但必须让人看见 */
    if (p_valve->pin_fault)
    {
        p_page->big0 = (uint8_t)OLED_HZ32_GU;
        p_page->big1 = (uint8_t)OLED_HZ32_ZHANG;
        disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Pin fault");
        (void)snprintf(p_page->r2, (size_t)sizeof(p_page->r2),
                       "Check P104 count=%lu", (unsigned long)p_valve->fault_count);
        return;
    }

    /* ② 传感器故障（判别层安全侧：排空） */
    if (disp_is_fault(p_out))
    {
        p_page->big0 = (uint8_t)OLED_HZ32_GU;
        p_page->big1 = (uint8_t)OLED_HZ32_ZHANG;
        disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Sensor fault");
        disp_fmt_note(p_page->r2, (uint32_t)sizeof(p_page->r2), p_out->note);
        return;
    }

    /* ③ 非测量工况：没水就画"待机"，只有真的无法判断才画"无基线故障" */
    if (REGIME_DRY == p_dec->regime)
    {
        p_page->big0 = (uint8_t)OLED_HZ32_DAI;
        p_page->big1 = (uint8_t)OLED_HZ32_JI;
        disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Dry  Drain");
        disp_copy(p_page->r2, (uint32_t)sizeof(p_page->r2), "No water");
        return;
    }

    if (!p_out->baseline_valid)
    {
        uint16_t n = p_out->baseline_n;

        if ((n > 0U) && (n < DISPLAY_BASELINE_TOTAL))
        {
            p_page->big0 = (uint8_t)OLED_HZ32_DAI;
            p_page->big1 = (uint8_t)OLED_HZ32_JI;
            disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "Baselining");
            disp_copy(p_page->r2, (uint32_t)sizeof(p_page->r2), "Probe in water");
        }
        else
        {
            p_page->big0 = (uint8_t)OLED_HZ32_GU;
            p_page->big1 = (uint8_t)OLED_HZ32_ZHANG;
            disp_copy(p_page->r1, (uint32_t)sizeof(p_page->r1), "No baseline");
            disp_copy(p_page->r2, (uint32_t)sizeof(p_page->r2), "Send W (D first)");
        }
        return;
    }

    /* ④ 正常：大字 = 实际执行状态 */
    recover = (VALVE_POS_RECOVER == p_valve->position);

    if (manual)
    {
        p_page->big0    = (uint8_t)OLED_HZ32_HAND;   /* 手 */
        p_page->big1    = (uint8_t)OLED_HZ32_DONG;   /* 动 */
        p_page->inverse = true;
    }
    else if (recover)
    {
        p_page->big0    = (uint8_t)OLED_HZ32_HUI;    /* 回 */
        p_page->big1    = (uint8_t)OLED_HZ32_SHOU;   /* 收 */
        p_page->inverse = true;                      /* 反显抓眼球；排空用正显 */
    }
    else
    {
        p_page->big0 = (uint8_t)OLED_HZ32_PAI;       /* 排 */
        p_page->big1 = (uint8_t)OLED_HZ32_KONG;      /* 空 */
    }

    /* 第二行（8x16，16 字符）：阀位置 + 继电器实际状态 */
    (void)snprintf(p_page->r1, (size_t)sizeof(p_page->r1), "%s%sRly:%s",
                   recover ? "Recover" : "Drain",
                   recover ? "  " : "    ",
                   p_valve->relay_on ? "On" : "Off");

    /* 第三行（6x8，21 字符）：模式短标签 + 吸合次数 + 本次通电时长
     * （整秒显示；count/秒数都限幅到 4/3 位，保证最坏情况下不超 21 字符） */
    {
        uint32_t cnt = p_valve->switch_count;
        uint32_t sec = p_valve->on_time_ms / 1000U;

        if (cnt > 9999U)
        {
            cnt = 9999U;
        }
        if (sec > 999U)
        {
            sec = 999U;
        }
        disp_copy(temp, (uint32_t)sizeof(temp), manual ? disp_manual_tag(p_valve) : "Auto");
        (void)snprintf(p_page->r2, (size_t)sizeof(p_page->r2), "%s Cnt:%lu On:%lus",
                       temp, (unsigned long)cnt, (unsigned long)sec);
    }
}

/* ============================ 页面内容：P3 详情页 ============================ */

/*
 * 函数名：disp_build_info_page
 * 作用：构建 P3 详情页（8 行 6x8）：事件/状态/阀/模式/特征/基线/阈值。
 * 参数：now_ms、p_out、p_dec、p_valve —— 输入；p_page —— 输出。
 * 返回：无。
 * 时序/边界：纯计算；事件已持续时间来自 event_active 上升沿（见 display_task）。
 * 注意：事件超时/无事件时显示 EV:OFF；故障/无基线时最后一行改提示行。
 */
static void disp_build_info_page(uint32_t now_ms, const pipeline_out_t *p_out,
                                 const decision_out_t *p_dec, const valve_out_t *p_valve,
                                 disp_page_t *p_page)
{
    char  tbuf[8];
    char  nbuf[8];
    int32_t ec0;
    int32_t sig10;
    int32_t d_ec;
    int32_t g_milli;
    int32_t ntu_x10;
    int32_t d1_ec;
    int32_t d2_ec;
    int32_t d1_ntu;
    int32_t d2_ntu;

    p_page->page = 3U;
    p_page->kind = DISP_KIND_TEXT;

    (void)memset(p_page->rows, 0, sizeof(p_page->rows));

    /* 行 0：页号 + 工况（v2）+ 手动事件标记（事件只用于日志分段，不控制阀） */
    if (g_event_active)
    {
        uint32_t ev_s = (uint32_t)(now_ms - g_event_start_ms) / 1000U;

        if (ev_s > 99U)
        {
            ev_s = 99U;   /* 行宽限制；事件只是日志标记，不需要更长 */
        }
        (void)snprintf(p_page->rows[0], DISP_ROW_BUF, "P3/3 Reg:%s EV:%lus",
                       disp_regime_tag(p_dec->regime), (unsigned long)ev_s);
    }
    else
    {
        (void)snprintf(p_page->rows[0], DISP_ROW_BUF, "P3/3 Reg:%s EV:Off",
                       disp_regime_tag(p_dec->regime));
    }

    /* 行 1：判别状态/等级 */
    (void)snprintf(p_page->rows[1], DISP_ROW_BUF, "State:%s Lv:%s",
                   disp_state_title(p_dec->state), decision_level_str(p_dec->level));

    /* 行 2：阀实际位置 + 继电器实际状态 */
    if (NULL != p_valve)
    {
        (void)snprintf(p_page->rows[2], DISP_ROW_BUF, "Valve:%s Rly:%s",
                       disp_pos_title(p_valve->position),
                       p_valve->relay_on ? "On" : "Off");
    }
    else
    {
        disp_copy(p_page->rows[2], DISP_ROW_BUF, "Valve:no data");
    }

    /* 行 3：模式 + 吸合次数 */
    if (NULL != p_valve)
    {
        uint32_t cnt = p_valve->switch_count;

        if (cnt > 9999U)
        {
            cnt = 9999U;
        }
        (void)snprintf(p_page->rows[3], DISP_ROW_BUF, "Mode:%s Cnt:%lu",
                       disp_valve_manual(p_valve) ? disp_manual_tag(p_valve) : "Auto",
                       (unsigned long)cnt);
    }
    else
    {
        disp_copy(p_page->rows[3], DISP_ROW_BUF, "Mode:no data");
    }

    /* 行 4：dEC + g */
    d_ec    = disp_round_i(p_out->d_ec_mv);
    g_milli = disp_round_i(p_out->g_ec * 1000.0f);
    if (g_milli < 0)
    {
        g_milli = 0;
    }
    (void)snprintf(p_page->rows[4], DISP_ROW_BUF, "dEC:%+dmV g:%ld.%03ld",
                   (int)d_ec, (long)(g_milli / 1000), (long)(g_milli % 1000));

    /* 行 5：dNTU%（带符号一位小数）+ 水温 */
    ntu_x10 = disp_round_i(p_out->d_ntu_pct * 10.0f);
    if (ntu_x10 > 999)
    {
        ntu_x10 = 999;
    }
    if (ntu_x10 < -999)
    {
        ntu_x10 = -999;
    }
    disp_fmt_signed_dec(nbuf, (uint32_t)sizeof(nbuf), (float)ntu_x10 / 10.0f);
    disp_fmt_temp(tbuf, (uint32_t)sizeof(tbuf), p_out->water_temp_c);
    (void)snprintf(p_page->rows[5], DISP_ROW_BUF, "dNTU:%s%% T:%sC", nbuf, tbuf);

    /* 行 6：基线 EC0 + 噪声 σ（无基线时占位） */
    if (p_out->baseline_valid)
    {
        ec0  = disp_round_i(p_out->ec0_mv25);
        sig10 = disp_round_i(p_out->ec_sigma_mv * 10.0f);
        if (sig10 < 0)
        {
            sig10 = 0;
        }
        (void)snprintf(p_page->rows[6], DISP_ROW_BUF, "EC0:%ldmV s:%ld.%ld",
                       (long)ec0, (long)(sig10 / 10), (long)(sig10 % 10));
    }
    else
    {
        disp_copy(p_page->rows[6], DISP_ROW_BUF, "EC0:----mV s:---");
    }

    /* 行 7：默认显示当前生效的 d1/d2 阈值；故障/无基线时改为提示行 */
    if (disp_is_fault(p_out))
    {
        disp_fmt_note(p_page->rows[7], DISP_ROW_BUF, p_out->note);
    }
    else if (!p_out->baseline_valid)
    {
        disp_copy(p_page->rows[7], DISP_ROW_BUF, "Send D then W");
    }
    else
    {
        d1_ec  = disp_round_i(p_dec->d1_ec_mv);
        d2_ec  = disp_round_i(p_dec->d2_ec_mv);
        d1_ntu = disp_round_i(p_dec->d1_ntu_pct);
        d2_ntu = disp_round_i(p_dec->d2_ntu_pct);
        /* 限幅：最坏 "d1:999/99% d2:999/99%" = 21 字符（6x8 一行上限） */
        if (d1_ec < 0) { d1_ec = 0; }
        if (d1_ec > 999) { d1_ec = 999; }
        if (d2_ec < 0) { d2_ec = 0; }
        if (d2_ec > 999) { d2_ec = 999; }
        if (d1_ntu < 0) { d1_ntu = 0; }
        if (d1_ntu > 99) { d1_ntu = 99; }
        if (d2_ntu < 0) { d2_ntu = 0; }
        if (d2_ntu > 99) { d2_ntu = 99; }
        (void)snprintf(p_page->rows[7], DISP_ROW_BUF, "d1:%ld/%ld%% d2:%ld/%ld%%",
                       (long)d1_ec, (long)d1_ntu, (long)d2_ec, (long)d2_ntu);
    }
}

/*
 * 函数名：disp_build_page
 * 作用：按当前页号构建目标内容（不画屏）。
 * 参数：now_ms/p_out/p_dec/p_valve —— 输入；p_page —— 输出。
 * 返回：无。
 * 时序/边界：纯计算；每 50 ms tick 调用一次。
 * 注意：g_page 是内部页索引 0..2，对应 P1/P2/P3。
 */
static void disp_build_page(uint32_t now_ms, const pipeline_out_t *p_out,
                            const decision_out_t *p_dec, const valve_out_t *p_valve,
                            disp_page_t *p_page)
{
    (void)memset(p_page, 0, sizeof(*p_page));

    switch (g_page)
    {
        case DISPLAY_PAGE_EXEC:
            disp_build_exec_page(p_out, p_dec, p_valve, p_page);
            break;

        case DISPLAY_PAGE_INFO:
            disp_build_info_page(now_ms, p_out, p_dec, p_valve, p_page);
            break;

        case DISPLAY_PAGE_WATER:
        default:
            disp_build_water_page(p_out, p_dec, p_page);
            break;
    }
}

/* ============================ 区域重画 ============================ */

/*
 * 函数名：disp_draw_page_no
 * 作用：在大字带右上角画 "n/3" 页码（6x8，3 字符，x=98 起，不压大字）。
 * 参数：page —— 页号 1..3。
 * 返回：无。
 * 时序/边界：只改写显存；调用前大字带已清黑。
 * 注意：页码只是"当前第几页"的提示，不是安全信息。
 */
static void disp_draw_page_no(uint8_t page)
{
    char buf[8];

    (void)snprintf(buf, sizeof(buf), "%u/%u", (unsigned)page, (unsigned)DISPLAY_PAGE_COUNT);
    oled_text_6x8(DISP_PAGENO_X, DISP_PAGENO_Y, buf);
}

/*
 * 函数名：disp_redraw_header
 * 作用：重画页头（先清该 8 行带，再写 6x8 文本，避免旧串尾部残留）。
 * 参数：p_page —— 目标内容。
 * 返回：无。
 * 时序/边界：O(128*8) 显存写，无 I2C。
 */
static void disp_redraw_header(const disp_page_t *p_page)
{
    oled_fill_rect(DISP_HEAD_X, DISP_HEAD_Y, OLED_WIDTH, 8U, false);
    oled_text_6x8(DISP_HEAD_X, DISP_HEAD_Y, p_page->r0);
}

/*
 * 函数名：disp_redraw_big
 * 作用：重画大字带（y=8..39 整带清黑，再画两个 32x32 汉字 + 右上角页码）。
 * 参数：p_page —— 目标内容。
 * 返回：无。
 * 时序/边界：清带 128*32 像素 + 两个汉字各 128 字节，无 I2C。
 * 注意：为什么要整带清：切页/开机页残留都在这 32 行里；不清会留半个旧字。
 */
static void disp_redraw_big(const disp_page_t *p_page)
{
    oled_fill_rect(0U, DISP_BIG_Y, OLED_WIDTH, DISP_BIG_BAND_H, false);
    oled_hz32(DISP_BIG_X0, DISP_BIG_Y, (oled_hz32_id_t)p_page->big0, p_page->inverse);
    oled_hz32(DISP_BIG_X1, DISP_BIG_Y, (oled_hz32_id_t)p_page->big1, p_page->inverse);
    disp_draw_page_no(p_page->page);
}

/*
 * 函数名：disp_redraw_line2
 * 作用：重画第二行（8x16；先清 16 行带，再写文本）。
 * 参数：p_page —— 目标内容。
 * 返回：无。
 * 时序/边界：O(128*16) 显存写，无 I2C。
 * 注意：8x16 文本函数不负责清行尾，必须先 fill_rect。
 */
static void disp_redraw_line2(const disp_page_t *p_page)
{
    oled_fill_rect(DISP_LINE2_X, DISP_LINE2_Y, OLED_WIDTH, 16U, false);
    oled_text_8x16(DISP_LINE2_X, DISP_LINE2_Y, p_page->r1);
}

/*
 * 函数名：disp_redraw_line3
 * 作用：重画第三行（6x8；先清 8 行带，再写文本）。
 * 参数：p_page —— 目标内容。
 * 返回：无。
 * 时序/边界：O(128*8) 显存写，无 I2C。
 */
static void disp_redraw_line3(const disp_page_t *p_page)
{
    oled_fill_rect(DISP_LINE3_X, DISP_LINE3_Y, OLED_WIDTH, 8U, false);
    oled_text_6x8(DISP_LINE3_X, DISP_LINE3_Y, p_page->r2);
}

/*
 * 函数名：disp_redraw_row
 * 作用：重画详情页的一行（6x8；先清该 8 行带，再写文本）。
 * 参数：row —— 行号 0..7；p_text —— 文本。
 * 返回：无。
 * 时序/边界：O(128*8) 显存写，无 I2C。
 */
static void disp_redraw_row(uint8_t row, const char *p_text)
{
    oled_fill_rect(0U, (uint8_t)(row * 8U), OLED_WIDTH, 8U, false);
    oled_text_6x8(0U, (uint8_t)(row * 8U), p_text);
}

/* ============================ 对外接口 ============================ */

/*
 * 函数名：display_init
 * 作用：初始化 OLED 并画开机页（证明 I2C/驱动/字库整条链路可用）。
 * 参数：无。
 * 返回：true=可用；false=不可用（上层只打印一次 WARN）。
 * 时序/边界：约 150 ms（oled_init 上电延时 100 ms + 初始化命令 + 清屏 + 开机页）。
 * 注意：开机页会在屏上保持约 DISPLAY_BOOT_HOLD_MS（800 ms），由 display_task 负责
 *       计时切换，这样上电自检时人能看清 "OLED OK"。
 */
bool display_init(void)
{
    g_shown_once   = false;
    g_first_seen   = false;
    g_first_ms     = 0U;
    g_last_num_ms  = 0U;
    g_last_full_ms = 0U;
    g_fail_count   = 0U;
    g_page         = DISPLAY_PAGE_WATER;   /* 默认 P1 水质页 */
    g_force_full   = true;
    g_shown_page   = 0U;
    g_shown_kind   = DISP_KIND_BIG;
    g_event_active = false;
    g_event_prev_active = false;
    g_event_start_ms = 0U;
    (void)memset(g_shown_r0, 0, sizeof(g_shown_r0));
    (void)memset(g_shown_r1, 0, sizeof(g_shown_r1));
    (void)memset(g_shown_r2, 0, sizeof(g_shown_r2));
    (void)memset(g_shown_rows, 0, sizeof(g_shown_rows));
    g_shown_big0    = 0U;
    g_shown_big1    = 0U;
    g_shown_inverse = false;

    if (!oled_init())
    {
        return false;
    }

    /* 开机页：7 行固定文案，全部 8 像素对齐（字库/文本函数只支持页对齐） */
    oled_clear();                                            /* 全屏黑 + 8 页全脏 */
    oled_text_6x8(0U, 0U,  "RA4M2 Water Judge");
    oled_text_8x16(0U, 8U, "OLED Ok");
    oled_text_6x8(0U, 24U, "SSD1306 128x64");
    oled_text_6x8(0U, 32U, "I2C0 0x3C 400kHz");
    oled_text_6x8(0U, 40U, "SENSOR-NODE v1.3");
    oled_text_6x8(0U, 48U, "P1/P2/P3 K2=Next");
    oled_text_6x8(0U, 56U, "M=Mark (UART)");

    if (!oled_flush())
    {
        /* 初始化已成功但首帧失败：按"不可用"返回，让 app 打印 WARN（显示不是安全路径） */
        return false;
    }

    return true;
}

/*
 * 函数名：display_page_next
 * 作用：K2 短按调用：循环切到下一页（P1→P2→P3→P1），下一次 display_task 整屏重画。
 * 参数：无。
 * 返回：切换后的页号 1..3；屏幕不可用时返回 0（不改变内部页号）。
 * 时序/边界：O(1)，无 I2C。
 * 注意：显示不是安全路径；切页不影响判别/阀。
 */
uint8_t display_page_next(void)
{
    if (!oled_ready())
    {
        return 0U;
    }
    g_page = (uint8_t)((g_page + 1U) % DISPLAY_PAGE_COUNT);
    g_force_full = true;
    return (uint8_t)(g_page + 1U);
}

/*
 * 函数名：display_page_get
 * 作用：读取当前页号（1..3）。
 * 参数：无。
 * 返回：当前页号。
 * 时序/边界：O(1)。
 */
uint8_t display_page_get(void)
{
    return (uint8_t)(g_page + 1U);
}

/*
 * 函数名：display_task
 * 作用：把当前数据画到当前页（差异刷新 + 200 ms 数值节流 + 1 s 心跳 + 切页整屏重画）。
 * 参数：now_ms —— app 时基；p_out/p_dec —— 流水线与判别输出；p_valve —— 执行器输出（可 NULL）。
 * 返回：true=正常（含本 tick 无需刷新）；false=连续 3 帧失败/屏已丢失。
 * 时序/边界：每 50 ms 调一次；最坏约 25 ms（心跳整屏）；单次失败不阻塞主循环。
 * 注意：本函数不打印——WARN 由 app_main 在第一次收到 false 时打印，避免刷屏。
 */
bool display_task(uint32_t now_ms, const pipeline_out_t *p_out, const decision_out_t *p_dec,
                  const valve_out_t *p_valve)
{
    disp_page_t page;
    bool        need_full;
    bool        need_num;
    bool        big_changed;
    uint8_t     row;

    /* 屏幕不可用/已丢失：直接返回 false，让 app 的一次性 WARN 生效；不占主循环时间 */
    if (!oled_ready())
    {
        return false;
    }

    if ((NULL == p_out) || (NULL == p_dec))
    {
        return true;   /* 防御：上层还没准备好数据，本 tick 不画 */
    }

    /* 开机页保持：第一次调用记录基准时刻，之后 800 ms 内不覆盖开机页 */
    if (!g_first_seen)
    {
        g_first_seen = true;
        g_first_ms   = now_ms;
    }
    if ((now_ms - g_first_ms) < DISPLAY_BOOT_HOLD_MS)
    {
        return true;
    }

    /* 事件计时：event_active 上升沿记录起点（P3 显示已持续时间） */
    g_event_prev_active = g_event_active;
    g_event_active      = p_dec->event_active;
    if (g_event_active && (!g_event_prev_active))
    {
        g_event_start_ms = now_ms;
    }

    /* ① 算目标内容，并与上一屏比较，决定重画哪些区域 */
    disp_build_page(now_ms, p_out, p_dec, p_valve, &page);

    need_full = (!g_shown_once) || g_force_full || (page.page != g_shown_page)
             || (page.kind != g_shown_kind)
             || ((now_ms - g_last_full_ms) >= DISPLAY_HEARTBEAT_MS);
    need_num  = ((now_ms - g_last_num_ms) >= DISPLAY_NUM_PERIOD_MS);

    if (need_full)
    {
        /* 心跳/首帧/切页：整屏重画（顺带清掉旧页残留；也是"屏还活着"的自恢复手段） */
        oled_clear();
        if (DISP_KIND_BIG == page.kind)
        {
            disp_redraw_header(&page);
            disp_redraw_big(&page);
            disp_redraw_line2(&page);
            disp_redraw_line3(&page);
        }
        else
        {
            for (row = 0U; row < DISP_INFO_ROWS; row++)
            {
                disp_redraw_row(row, page.rows[row]);
            }
        }
        g_last_full_ms = now_ms;
        g_last_num_ms  = now_ms;
        g_force_full   = false;

        /* 整屏已画：缓存全部同步 */
        disp_copy(g_shown_r0, (uint32_t)sizeof(g_shown_r0), page.r0);
        disp_copy(g_shown_r1, (uint32_t)sizeof(g_shown_r1), page.r1);
        disp_copy(g_shown_r2, (uint32_t)sizeof(g_shown_r2), page.r2);
        for (row = 0U; row < DISP_INFO_ROWS; row++)
        {
            disp_copy(g_shown_rows[row], DISP_ROW_BUF, page.rows[row]);
        }
        g_shown_big0    = page.big0;
        g_shown_big1    = page.big1;
        g_shown_inverse = page.inverse;
        g_shown_page    = page.page;
        g_shown_kind    = page.kind;
    }
    else if (DISP_KIND_BIG == page.kind)
    {
        /* 大字/状态串变化：现场最关心的信息，立即重画（不等 200 ms 节流） */
        big_changed = (page.big0 != g_shown_big0) || (page.big1 != g_shown_big1)
                   || (page.inverse != g_shown_inverse);
        if (big_changed)
        {
            disp_redraw_big(&page);
        }
        if (0 != strcmp(page.r1, g_shown_r1))
        {
            disp_redraw_line2(&page);
            disp_copy(g_shown_r1, (uint32_t)sizeof(g_shown_r1), page.r1);
        }
        /* 页头（EC0/σ/温度）与第三行（dEC/g/N）是数值行：最快 200 ms 刷一次，
         * 与文档口径一致（v1.3.0 审查 P1-1：此前少了 need_num 门控，每 tick 都刷）。 */
        if (need_num)
        {
            if (0 != strcmp(page.r0, g_shown_r0))
            {
                disp_redraw_header(&page);
                disp_copy(g_shown_r0, (uint32_t)sizeof(g_shown_r0), page.r0);
            }
            if (0 != strcmp(page.r2, g_shown_r2))
            {
                disp_redraw_line3(&page);
                disp_copy(g_shown_r2, (uint32_t)sizeof(g_shown_r2), page.r2);
            }
            g_last_num_ms = now_ms;
        }
        if (big_changed)
        {
            g_shown_big0    = page.big0;
            g_shown_big1    = page.big1;
            g_shown_inverse = page.inverse;
        }
    }
    else
    {
        /* 详情页：前 3 行（事件/状态/阀）立即刷新；数值行 4~7 节流到 200 ms */
        for (row = 0U; row < 3U; row++)
        {
            if (0 != strcmp(page.rows[row], g_shown_rows[row]))
            {
                disp_redraw_row(row, page.rows[row]);
                disp_copy(g_shown_rows[row], DISP_ROW_BUF, page.rows[row]);
            }
        }
        if (need_num)
        {
            for (row = 3U; row < DISP_INFO_ROWS; row++)
            {
                if (0 != strcmp(page.rows[row], g_shown_rows[row]))
                {
                    disp_redraw_row(row, page.rows[row]);
                    disp_copy(g_shown_rows[row], DISP_ROW_BUF, page.rows[row]);
                }
            }
            g_last_num_ms = now_ms;
        }
    }

    /* ② 只刷脏页（oled_flush 内部实现）；连续失败 3 帧才报给上层 */
    if (!oled_flush())
    {
        g_fail_count++;
        if (g_fail_count >= DISPLAY_FAIL_LIMIT)
        {
            return false;
        }
        return true;   /* 前两帧失败先不打扰上层：脏页位保留，下一帧自动重试 */
    }

    g_fail_count = 0U;
    return true;
}

/*
 * 函数名：display_ok
 * 作用：查询显示是否可用。
 * 参数：无。
 * 返回：true=oled 已初始化且未判丢失。
 * 时序/边界：O(1)。
 * 注意：与 oled_ready() 同义，保留此接口是为了让 app_main 不直接依赖 oled.h。
 */
bool display_ok(void)
{
    return oled_ready();
}
