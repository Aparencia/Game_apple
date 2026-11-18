/*
 * validity.c
 *
 *  Created on: 2026年11月16日
 *      Author: Administrator
 */

/*
 * validity.c —— 工况有效性层实现（位图 + regime 状态机）
 * ---------------------------------------------------------------------------
 * 本文件是纯逻辑：不碰 ADC / 1-Wire / UART / 阀，不打印、不 delay、不动态分配。
 * 输入只有 pipeline_out_t（本 tick 的特征）+ now_ms（时基），输出全部落在 gate_t 里，
 * 因此可以脱离硬件用"手工填 pipeline_out_t"的方式验证工况迁移。
 *
 * 判据来源（Docs/软件层优化方案_v2_20261116.md）：
 *   §4 validity 位图；§5 regime 迁移表；§6.1 浸没；§6.2 稳定；§10 时序参数。
 *
 * 与文档的一处刻意差异（务必保留原因）：
 *   文档 §7.1 把 !BLANKING 写进回收许可谓词；本实现把 BLANKING 的作用改为
 *   "冻结判别"（不更新候选等级、不跑速率抢先），已生效的许可保持不变。
 *   原因：换向本身会触发屏蔽，若屏蔽又撤销许可，就形成
 *         回收 → 换向 → 屏蔽 → 排空 → 换向 → 屏蔽 … 的正反馈。
 *   屏蔽窗内 level 不变，因此 PERMIT 也不变，自激被切断（见 validity.h 文件头）。
 *
 * 安全侧：
 *   任何"不知道"的情况（无传感器、无基线、无干态指纹、工况不明）都落到
 *   REGIME_FAULT；REGIME_FAULT / DRY / SETTLE / LOCKOUT 一律禁止 decision 输出 L0。
 * ---------------------------------------------------------------------------
 * 参考：方案 v2 §4~§6、§10、§13（P0-1/P0-2）。
 */

#include "validity.h"

#include <stddef.h>
#include <string.h>

/* ---------- 静态原因字符串 ---------- */
static const char *const g_gate_reason_sensor   = "SENSOR";    /* 传感器故障 */
static const char *const g_gate_reason_no_cal   = "NO_CAL";    /* 基线与干态指纹都没有 */
static const char *const g_gate_reason_no_base  = "NO_BASE";   /* 缺水中基线 */
static const char *const g_gate_reason_dry      = "DRY";       /* 未浸没 */
static const char *const g_gate_reason_settle   = "SETTLE";    /* 稳定期 */
static const char *const g_gate_reason_measure  = "MEASURE";   /* 可判别 */
static const char *const g_gate_reason_lockout  = "LOCKOUT";   /* 故障恢复观察期 */

/* ---------- 默认配置 ---------- */
/* 数值全部来自方案 v2 §12 参数总表；现场按标定结果覆盖。 */
static const gate_cfg_t g_gate_default_cfg =
{
    .wet_ratio          = 0.30f,    /* g = EC/EC0 ≥ 0.30 才算"淹到电极" */
    .ec_wet_delta_mv    = 80.0f,    /* EC 至少比干态高 80 mV 才算有水（v2.3） */
    .ec_dry_ratio       = 3.0f,     /* 已弃用：仅保留字段，不再参与判据 */
    .ec_dry_max_mv      = 200.0f,   /* 干态指纹上限（本次实测 58 mV） */
    .ec0_min_mv         = 50.0f,    /* 基线下限：低于它=模块没供电/探头悬空 */
    .ec0_max_mv         = 1600.0f,  /* 基线上限：开路伪值约 1650 mV 与脏水本底都要排除 */
    .ec0_default_min_mv = 150.0f,   /* 默认干态（未实测）时，水中基线必须 ≥150 mV */
    .ao_water_delta_mv  = 80.0f,    /* 水 AO 至少比空腔高 80 mV（本装置水 AO 高于空气） */
    .ntu_ao_min_mv      = 20U,      /* 浊度 AO 低于它=未供电/断线 */
    .rate_stable_mv_s   = 80.0f,    /* 稳定带：EC 变化率（v2.4.1：肥皂约 50、换向约 38，30 太紧会把肥皂踢回 SETTLE） */
    .rate_stable_pct_s  = 20.0f,    /* 稳定带：浊度相对变化率（肥皂约 16%/s） */
    .settle_ms          = 4000U,    /* 入水稳定期（实测 3.8 s 才进 ±1 %） */
    .settle_min_ms      = 1500U,    /* v2.4：最早 1.5 s 进 MEASURE */
    .settle_stable_ms   = 500U,     /* 连续稳定 0.5 s 即可提前进入 */
    .settle_timeout_ms  = 6000U,    /* STABLE 一直不满足时的兜底（避免永远停在待机） */
    .blank_ms           = 400U,     /* 换向屏蔽窗 */
    .wet_debounce_ms    = 500U,     /* DRY→SETTLE 去抖 */
    .dry_debounce_ms    = 300U,     /* MEASURE→DRY 去抖（安全侧要快） */
    .unstable_ms        = 1000U,    /* MEASURE→SETTLE 去抖 */
    .fault_recover_ms   = 1000U,    /* 故障消失后进入观察期 */
    .lockout_ms         = 3000U     /* 观察期长度 */
};

/* ---------- 文件内工具 ---------- */
/*
 * 函数名：gate_clampf
 * 作用：把配置浮点值限幅；非法值（0/负/NaN）回落到默认值。
 */
static float gate_clampf(float value, float lo, float hi, float fallback)
{
    if (sensor_is_nan_f32(value))
    {
        return fallback;
    }
    if (value < lo)
    {
        return (value <= 0.0f) ? fallback : lo;
    }
    if (value > hi)
    {
        return hi;
    }
    return value;
}

/*
 * 函数名：gate_clampu32
 * 作用：把配置无符号值限幅；0 回落到默认值（0 在本模块没有"关闭"语义，
 *       屏蔽/去抖置 0 会让状态机失去消抖能力，属于危险配置）。
 */
static uint32_t gate_clampu32(uint32_t value, uint32_t lo, uint32_t hi, uint32_t fallback)
{
    if (0U == value)
    {
        return fallback;
    }
    if (value < lo)
    {
        return lo;
    }
    if (value > hi)
    {
        return hi;
    }
    return value;
}

/*
 * 函数名：gate_enter
 * 作用：迁移到新工况；只在工况真的变化时刷新时间戳与待定计时。
 * 参数：p_gate —— 句柄；regime —— 目标工况；now_ms —— 当前时刻。
 */
static void gate_enter(gate_t *p_gate, regime_t regime, uint32_t now_ms)
{
    if (p_gate->regime != regime)
    {
        p_gate->regime             = regime;
        p_gate->regime_since_ms    = now_ms;
        p_gate->wet_since_ms       = now_ms;
        p_gate->dry_since_ms       = now_ms;
        p_gate->unstable_since_ms  = now_ms;
        p_gate->fault_since_ms     = now_ms;
        p_gate->settle_stable_since_ms = now_ms;
    }
}

/*
 * 函数名：gate_elapsed
 * 作用：无符号差值计时（跨 49.7 天回绕安全）。
 */
static bool gate_elapsed(uint32_t now_ms, uint32_t since_ms, uint32_t span_ms)
{
    return ((uint32_t)(now_ms - since_ms) >= span_ms);
}

/*
 * 函数名：gate_bits_render
 * 作用：把位图逐位渲染成 "SNS|BAS|IMM|STA|BLK|FLW" 的短串。
 * 参数：bits —— 位图；p_buf —— 目标缓冲；cap —— 容量（字节）。
 * 返回：无（容量不足时按位截断，始终 '\0' 结尾）。
 */
static void gate_bits_render(uint16_t bits, char *p_buf, uint32_t cap)
{
    static const struct
    {
        uint16_t    bit;
        const char *name;
    } g_names[] =
    {
        { (uint16_t)VAL_SENSOR_OK,   "SNS" },
        { (uint16_t)VAL_BASELINE_OK, "BAS" },
        { (uint16_t)VAL_IMMERSED,    "IMM" },
        { (uint16_t)VAL_STABLE,      "STA" },
        { (uint16_t)VAL_BLANKING,    "BLK" },
        { (uint16_t)VAL_FLOW,        "FLW" }
    };
    uint32_t i;
    uint32_t len = 0U;
    uint32_t n;

    if ((NULL == p_buf) || (0U == cap))
    {
        return;
    }
    p_buf[0] = '\0';

    for (i = 0U; i < (uint32_t)(sizeof(g_names) / sizeof(g_names[0])); i++)
    {
        if (0U == (bits & g_names[i].bit))
        {
            continue;
        }
        n = (uint32_t)strlen(g_names[i].name);
        if ((len + n + 2U) >= cap)
        {
            break;
        }
        if (0U != len)
        {
            p_buf[len] = '|';
            len++;
        }
        (void)memcpy(&p_buf[len], g_names[i].name, n);
        len += n;
        p_buf[len] = '\0';
    }
}

/*
 * 函数名：gate_update_reason
 * 作用：按"工况 + 故障来源"给出静态原因串。
 * 注意：原因只用于日志/OLED，不参与控制判断。
 */
static const char *gate_update_reason(const gate_t *p_gate)
{
    switch (p_gate->regime)
    {
        case REGIME_FAULT:
            if (!p_gate->sensor_ok)
            {
                return g_gate_reason_sensor;
            }
            if ((!p_gate->baseline_ok) && (!p_gate->dry_valid))
            {
                return g_gate_reason_no_cal;   /* 既没水中基线也没干态指纹：没法判 */
            }
            return g_gate_reason_no_base;

        case REGIME_DRY:
            return g_gate_reason_dry;

        case REGIME_SETTLE:
            return g_gate_reason_settle;

        case REGIME_MEASURE:
            return g_gate_reason_measure;

        case REGIME_LOCKOUT:
        default:
            return g_gate_reason_lockout;
    }
}

/* ---------- 对外接口 ---------- */

/*
 * 函数名：gate_init
 * 作用：保存/限幅配置，清零运行状态，工况置 FAULT（安全侧），干态指纹清空。
 * 参数：p_gate —— 句柄（可为 NULL，NULL 直接返回）；p_cfg —— 配置（NULL 用默认）。
 * 返回：无。
 */
void gate_init(gate_t *p_gate, const gate_cfg_t *p_cfg)
{
    gate_cfg_t *p_c;

    if (NULL == p_gate)
    {
        return;
    }

    p_gate->cfg = (NULL != p_cfg) ? *p_cfg : g_gate_default_cfg;
    p_c = &p_gate->cfg;

    /* 逐项限幅：0/负/NaN 一律落回默认，保证状态机不会"永远触发/永远不触发" */
    p_c->wet_ratio         = gate_clampf(p_c->wet_ratio, 0.02f, 0.95f,
                                         g_gate_default_cfg.wet_ratio);
    p_c->ec_wet_delta_mv   = gate_clampf(p_c->ec_wet_delta_mv, 10.0f, 500.0f,
                                         g_gate_default_cfg.ec_wet_delta_mv);
    p_c->ec_dry_ratio      = gate_clampf(p_c->ec_dry_ratio, 1.2f, 20.0f,
                                         g_gate_default_cfg.ec_dry_ratio);
    p_c->ec_dry_max_mv     = gate_clampf(p_c->ec_dry_max_mv, 10.0f, 2000.0f,
                                         g_gate_default_cfg.ec_dry_max_mv);
    p_c->ec0_min_mv        = gate_clampf(p_c->ec0_min_mv, 5.0f, 1000.0f,
                                         g_gate_default_cfg.ec0_min_mv);
    p_c->ec0_max_mv        = gate_clampf(p_c->ec0_max_mv, 100.0f, 3300.0f,
                                         g_gate_default_cfg.ec0_max_mv);
    if (p_c->ec0_max_mv <= p_c->ec0_min_mv)
    {
        p_c->ec0_max_mv = p_c->ec0_min_mv * 4.0f;
    }
    p_c->ec0_default_min_mv = gate_clampf(p_c->ec0_default_min_mv, 50.0f, 1000.0f,
                                          g_gate_default_cfg.ec0_default_min_mv);
    p_c->ao_water_delta_mv  = gate_clampf(p_c->ao_water_delta_mv, -500.0f, 500.0f,
                                          g_gate_default_cfg.ao_water_delta_mv);
    if (0U == p_c->ntu_ao_min_mv)
    {
        p_c->ntu_ao_min_mv = g_gate_default_cfg.ntu_ao_min_mv;
    }

    p_c->rate_stable_mv_s  = gate_clampf(p_c->rate_stable_mv_s, 1.0f, 10000.0f,
                                         g_gate_default_cfg.rate_stable_mv_s);
    p_c->rate_stable_pct_s = gate_clampf(p_c->rate_stable_pct_s, 0.5f, 1000.0f,
                                         g_gate_default_cfg.rate_stable_pct_s);

    p_c->settle_ms         = gate_clampu32(p_c->settle_ms, 200U, 120000U,
                                           g_gate_default_cfg.settle_ms);
    p_c->settle_min_ms     = gate_clampu32(p_c->settle_min_ms, 200U, 120000U,
                                           g_gate_default_cfg.settle_min_ms);
    p_c->settle_stable_ms  = gate_clampu32(p_c->settle_stable_ms, 0U, 10000U,
                                           g_gate_default_cfg.settle_stable_ms);
    if (p_c->settle_min_ms > p_c->settle_ms)
    {
        p_c->settle_min_ms = p_c->settle_ms;
    }
    p_c->settle_timeout_ms = gate_clampu32(p_c->settle_timeout_ms, 2000U, 120000U,
                                           g_gate_default_cfg.settle_timeout_ms);
    if (p_c->settle_timeout_ms <= p_c->settle_ms)
    {
        p_c->settle_timeout_ms = p_c->settle_ms + 2000U;
    }
    p_c->blank_ms          = gate_clampu32(p_c->blank_ms, 0U, 10000U,
                                           g_gate_default_cfg.blank_ms);
    p_c->wet_debounce_ms   = gate_clampu32(p_c->wet_debounce_ms, 0U, 10000U,
                                           g_gate_default_cfg.wet_debounce_ms);
    p_c->dry_debounce_ms   = gate_clampu32(p_c->dry_debounce_ms, 0U, 10000U,
                                           g_gate_default_cfg.dry_debounce_ms);
    p_c->unstable_ms       = gate_clampu32(p_c->unstable_ms, 100U, 60000U,
                                           g_gate_default_cfg.unstable_ms);
    p_c->fault_recover_ms  = gate_clampu32(p_c->fault_recover_ms, 0U, 60000U,
                                           g_gate_default_cfg.fault_recover_ms);
    p_c->lockout_ms        = gate_clampu32(p_c->lockout_ms, 0U, 60000U,
                                           g_gate_default_cfg.lockout_ms);

    /* 运行时状态清零；工况先按 FAULT（安全侧），首次 update 允许直接落 DRY */
    p_gate->dry_valid        = false;
    p_gate->dry_source       = DRY_SOURCE_NONE;
    p_gate->ec_dry_mv        = 0.0f;
    p_gate->ao_dry_mv        = 0.0f;
    p_gate->regime            = REGIME_FAULT;
    p_gate->regime_since_ms   = 0U;
    p_gate->wet_since_ms      = 0U;
    p_gate->dry_since_ms      = 0U;
    p_gate->unstable_since_ms = 0U;
    p_gate->fault_since_ms    = 0U;
    p_gate->settle_stable_since_ms = 0U;
    p_gate->first_update      = true;
    p_gate->switch_seen      = false;
    p_gate->last_switch_ms   = 0U;
    p_gate->sensor_ok        = false;
    p_gate->baseline_ok      = false;
    p_gate->immersed         = false;
    p_gate->stable           = false;
    p_gate->blanking         = false;
    p_gate->bits             = 0U;
    p_gate->reason           = g_gate_reason_no_cal;
}

/*
 * 函数名：gate_store_dry
 * 作用：校验并写入干态指纹（内部共用；来源由调用方指定）。
 * 注意：EC_DRY 上限由 cfg.ec_dry_max_mv 控制（默认 200 mV）：超过它说明采集时
 *       探头多半还泡在水里，此时接受会把"湿态"当成"干态"，浸没判据就永远成立。
 */
static bool gate_store_dry(gate_t *p_gate, float ec_dry_mv, float ao_dry_mv, dry_source_t source)
{
    if (NULL == p_gate)
    {
        return false;
    }
    if (sensor_is_nan_f32(ec_dry_mv) || sensor_is_nan_f32(ao_dry_mv))
    {
        return false;
    }
    if ((ec_dry_mv < 1.0f) || (ec_dry_mv > p_gate->cfg.ec_dry_max_mv))
    {
        return false;
    }
    if ((ao_dry_mv < 0.0f) || (ao_dry_mv > 5000.0f))
    {
        return false;
    }

    p_gate->ec_dry_mv  = ec_dry_mv;
    p_gate->ao_dry_mv  = ao_dry_mv;
    p_gate->dry_valid  = true;
    p_gate->dry_source = source;
    return true;
}

/*
 * 函数名：gate_set_dry_fingerprint
 * 作用：记录实测干态指纹（命令 D 或开机自检采集完成时调用），来源标记 MEASURED。
 * 参数：p_gate —— 句柄；ec_dry_mv —— 空腔 EC 均值；ao_dry_mv —— 空腔 AO 均值。
 * 返回：true=接受；false=不可信（越界/NaN），原指纹保持不变。
 */
bool gate_set_dry_fingerprint(gate_t *p_gate, float ec_dry_mv, float ao_dry_mv)
{
    return gate_store_dry(p_gate, ec_dry_mv, ao_dry_mv, DRY_SOURCE_MEASURED);
}

bool gate_set_dry_default(gate_t *p_gate, float ec_dry_mv, float ao_dry_mv)
{
    return gate_store_dry(p_gate, ec_dry_mv, ao_dry_mv, DRY_SOURCE_DEFAULT);
}

dry_source_t gate_dry_source(const gate_t *p_gate)
{
    if (NULL == p_gate)
    {
        return DRY_SOURCE_NONE;
    }
    return p_gate->dry_source;
}

/* 读取干态指纹：未标定时返回 false（输出参数不动）。 */
bool gate_get_dry_fingerprint(const gate_t *p_gate, float *p_ec_dry_mv, float *p_ao_dry_mv)
{
    if ((NULL == p_gate) || (!p_gate->dry_valid))
    {
        return false;
    }
    if (NULL != p_ec_dry_mv)
    {
        *p_ec_dry_mv = p_gate->ec_dry_mv;
    }
    if (NULL != p_ao_dry_mv)
    {
        *p_ao_dry_mv = p_gate->ao_dry_mv;
    }
    return true;
}

/* 清除干态指纹（例如换探头/重新标定前）。 */
void gate_clear_dry_fingerprint(gate_t *p_gate)
{
    if (NULL == p_gate)
    {
        return;
    }
    p_gate->dry_valid = false;
    p_gate->dry_source = DRY_SOURCE_NONE;
    p_gate->ec_dry_mv = 0.0f;
    p_gate->ao_dry_mv = 0.0f;
}

/*
 * 函数名：gate_note_switch
 * 作用：记录一次物理换向，作为屏蔽窗起点。
 * 参数：p_gate —— 句柄；now_ms —— 换向时刻。
 */
void gate_note_switch(gate_t *p_gate, uint32_t now_ms)
{
    if (NULL == p_gate)
    {
        return;
    }
    p_gate->switch_seen    = true;
    p_gate->last_switch_ms = now_ms;
}

/*
 * 函数名：gate_update
 * 作用：每 tick 计算 validity 位图与 regime。
 * 参数：p_gate —— 句柄；p_out —— 流水线输出；now_ms —— 当前时刻。
 * 返回：无。
 * 时序/边界：pipeline_sample 之后、decision_update 之前调用；纯计算（<10 µs）。
 * 注意：本函数不修改 pipeline/阀，只读 p_out；任何 NULL 输入都保持上一次状态不变。
 */
void gate_update(gate_t *p_gate, const pipeline_out_t *p_out, uint32_t now_ms)
{
    bool sensor_ok;
    bool baseline_ok;
    bool immersed;
    bool stable;
    bool stable_for_state;
    bool blanking;
    bool fault;
    bool this_tick_fault;
    float dry_thr_mv;
    float ec_now;
    float ntu_pct_rate = 0.0f;

    if ((NULL == p_gate) || (NULL == p_out))
    {
        return;
    }

    /* ---------- ① 四条基础判据（位图输入） ---------- */
    /*
     * 传感器"可用"必须是 SENSOR_OK。ERR_RANGE 表示贴轨/开路（TDS 高轨、浊度低轨/高轨），
     * 早期版本把 ERR_RANGE 当正常，只额外拦了 TDS 贴轨，结果浊度高轨（5V 串入/模块异常，
     * AO≈4.9V）仍被判"传感器正常"，工况停在 MEASURE —— 这是不能接受的。
     * 驱动契约里 ERR_RANGE 从不表示"数据可用"，所以这里直接从严。
     */
    sensor_ok = (!p_out->adc_timeout)
             && (SENSOR_OK == p_out->tds_status)
             && (SENSOR_OK == p_out->ntu_status)
             && (!p_out->tds_rail)
             && (p_out->ntu_ao_mv >= p_gate->cfg.ntu_ao_min_mv);

    /*
     * 基线有效三条件：已标定 + 量程合理 + 明显高于干态。
     * 第三条要求 dry_valid：没有干态指纹的水中基线无法证明"是在水里采的"，
     * 若放行，先 W 后 D（或 W 时电极露空）会把空气 58 mV 当 EC0，g≡1 → 又回收空气。
     * app 层已在 W 前强制 D，这里再守一道（纵深防御）。
     */
    baseline_ok = p_out->baseline_valid
               && p_gate->dry_valid
               && (p_out->ec0_mv25 >= p_gate->cfg.ec0_min_mv)
               && (p_out->ec0_mv25 <= p_gate->cfg.ec0_max_mv)
               && (p_out->ec0_mv25 >= (p_gate->ec_dry_mv + p_gate->cfg.ec_wet_delta_mv));

    /*
     * 干态指纹若是"默认值"（没实测），水中基线还要过一道更高的绝对门限：
     * 默认 EC_DRY=60 时 3× 只有 180 mV，而"电极刚被水淋湿、腔里其实没水"很容易读到
     * 200~500 mV —— 那样的基线一旦生效，空腔 g≡1 → 又回到回收空气。
     * 400 mV 以上的读数才认为"确实有水体、不是一层水膜"。
     */
    if ((baseline_ok) && (DRY_SOURCE_MEASURED != p_gate->dry_source))
    {
        baseline_ok = (p_out->ec0_mv25 >= p_gate->cfg.ec0_default_min_mv);
    }

    ec_now = p_out->tds_mv25;
    dry_thr_mv = p_gate->ec_dry_mv + p_gate->cfg.ec_wet_delta_mv;

    if ((p_gate->dry_valid) && (baseline_ok))
    {
        /* 差值判据：比干态高 80 mV 以上才算有水；再用 g≥WET_RATIO 防"基线被写坏" */
        immersed = (ec_now >= dry_thr_mv) && (p_out->g_ec >= p_gate->cfg.wet_ratio);
    }
    else if (p_gate->dry_valid)
    {
        /* 没有水中基线时，只能用绝对差值判"有水/没水"（此时也不可能进 MEASURE） */
        immersed = (ec_now >= dry_thr_mv);
    }
    else
    {
        /* 连干态指纹都没有：不知道"有水"是什么量级，保守判未浸没（配合 baseline_ok=false → FAULT） */
        immersed = false;
    }

    if (p_out->ntu0_ao_mv > 0U)
    {
        ntu_pct_rate = (sensor_absf(p_out->ntu_rate_mv_s) * 100.0f) / (float)p_out->ntu0_ao_mv;
    }
    stable = (sensor_absf(p_out->ec_rate_mv_s) <= p_gate->cfg.rate_stable_mv_s)
          && (ntu_pct_rate <= p_gate->cfg.rate_stable_pct_s);

    blanking = p_gate->switch_seen
            && ((uint32_t)(now_ms - p_gate->last_switch_ms) < p_gate->cfg.blank_ms);

    /*
     * 换向瞬态（实测 ±35~38 mV/s）本身就会超过 rate_stable 的 30 mV/s。
     * 屏蔽窗本来就是为了"在这段时间里先别信判据"，所以窗内稳定判据也一并按稳定处理，
     * 否则一次换向就可能 MEASURE→SETTLE（再等 4 s）→ 形成 6 s 的回收/排空极限环。
     */
    stable_for_state = stable || blanking;

    /* ---------- ② 故障判定（立即生效，不等去抖） ---------- */
    /*
     * 关键取舍：没有水中基线、但已经用干态指纹确认"没水"时，工况是 DRY 而不是 FAULT。
     * 理由：DRY 是已知且安全的状态（阀恒排空），而 FAULT 会让现场误以为装置坏了；
     *      方案 §13 P0-1 的验收也要求"空腔上电 → DRY，阀不吸合"。
     */
    this_tick_fault = (!sensor_ok)
                   || ((!baseline_ok) && (!(p_gate->dry_valid && (!immersed))));

    fault = this_tick_fault;

    /* ---------- ③ regime 迁移（方案 §5） ---------- */
    if (p_gate->first_update)
    {
        /* 首次更新：所有去抖计时从这里起算，避免 since=0 时"一上电就满足去抖" */
        p_gate->wet_since_ms      = now_ms;
        p_gate->dry_since_ms      = now_ms;
        p_gate->unstable_since_ms = now_ms;
        p_gate->fault_since_ms    = now_ms;
        p_gate->first_update      = false;
        gate_enter(p_gate, fault ? REGIME_FAULT : REGIME_DRY, now_ms);
    }
    else
    {
        switch (p_gate->regime)
        {
            case REGIME_FAULT:
                if (fault)
                {
                    p_gate->fault_since_ms = now_ms;   /* 故障仍在：恢复计时重新开始 */
                }
                else if (gate_elapsed(now_ms, p_gate->fault_since_ms,
                                      p_gate->cfg.fault_recover_ms))
                {
                    gate_enter(p_gate, REGIME_LOCKOUT, now_ms);
                }
                else
                {
                    /* 等待故障条件稳定消失 */
                }
                break;

            case REGIME_LOCKOUT:
                if (fault)
                {
                    gate_enter(p_gate, REGIME_FAULT, now_ms);
                }
                else if (gate_elapsed(now_ms, p_gate->regime_since_ms, p_gate->cfg.lockout_ms))
                {
                    gate_enter(p_gate, REGIME_DRY, now_ms);
                }
                else
                {
                    /* 观察期内保持排空 */
                }
                break;

            case REGIME_DRY:
                if (fault)
                {
                    gate_enter(p_gate, REGIME_FAULT, now_ms);
                }
                else if (immersed)
                {
                    if (gate_elapsed(now_ms, p_gate->wet_since_ms,
                                     p_gate->cfg.wet_debounce_ms))
                    {
                        gate_enter(p_gate, REGIME_SETTLE, now_ms);
                    }
                }
                else
                {
                    p_gate->wet_since_ms = now_ms;   /* 没浸没：浸没计时清零 */
                }
                break;

            case REGIME_SETTLE:
                if (fault)
                {
                    gate_enter(p_gate, REGIME_FAULT, now_ms);
                }
                else if (!immersed)
                {
                    if (gate_elapsed(now_ms, p_gate->dry_since_ms,
                                     p_gate->cfg.dry_debounce_ms))
                    {
                        gate_enter(p_gate, REGIME_DRY, now_ms);
                    }
                }
                else
                {
                    bool settle_ready = false;

                    p_gate->dry_since_ms = now_ms;   /* 浸没保持：失浸没计时清零 */

                    if (stable)
                    {
                        /*
                         * v2.4 自适应提前：现场进水数据里，EC/浊度在进水后约 1 s 就稳定
                         * （rate 从 +450 降到 <30），但旧逻辑固定等 4 s，表现为"水已淹到
                         * 电极却一直待机"。现在只要连续稳定 settle_stable_ms、不早于
                         * settle_min_ms，**且 dEC ≤ 0（水不比基线脏）**，就提前进 MEASURE；
                         * dEC>0（可能正在变脏）仍等 4 s/6 s，避免慢速污染被提前放行。
                         */
                        if (gate_elapsed(now_ms, p_gate->settle_stable_since_ms,
                                         p_gate->cfg.settle_stable_ms)
                            && gate_elapsed(now_ms, p_gate->regime_since_ms,
                                            p_gate->cfg.settle_min_ms)
                            && (p_out->d_ec_mv <= 0.0f))
                        {
                            settle_ready = true;
                        }
                    }
                    else
                    {
                        p_gate->settle_stable_since_ms = now_ms;
                    }

                    if ((!settle_ready)
                        && ((gate_elapsed(now_ms, p_gate->regime_since_ms,
                                          p_gate->cfg.settle_ms)
                             && stable)
                            || gate_elapsed(now_ms, p_gate->regime_since_ms,
                                            p_gate->cfg.settle_timeout_ms)))
                    {
                        settle_ready = true;
                    }

                    if (settle_ready)
                    {
                        gate_enter(p_gate, REGIME_MEASURE, now_ms);
                    }
                }
                break;

            case REGIME_MEASURE:
            default:
                if (fault)
                {
                    gate_enter(p_gate, REGIME_FAULT, now_ms);
                }
                else if (!immersed)
                {
                    p_gate->unstable_since_ms = now_ms;   /* 失浸没期间不累积"不稳定"计时 */

                    if (gate_elapsed(now_ms, p_gate->dry_since_ms,
                                     p_gate->cfg.dry_debounce_ms))
                    {
                        gate_enter(p_gate, REGIME_DRY, now_ms);
                    }
                }
                else
                {
                    p_gate->dry_since_ms = now_ms;   /* 浸没保持：失浸没计时清零 */

                    if (!stable_for_state)
                    {
                        if (gate_elapsed(now_ms, p_gate->unstable_since_ms,
                                         p_gate->cfg.unstable_ms))
                        {
                            gate_enter(p_gate, REGIME_SETTLE, now_ms);
                        }
                    }
                    else
                    {
                        p_gate->unstable_since_ms = now_ms;   /* 稳定：不稳定计时清零 */
                    }
                }
                break;
        }
    }

    /* ---------- ④ 位图与对外字段 ---------- */
    p_gate->sensor_ok   = sensor_ok;
    p_gate->baseline_ok = baseline_ok;
    p_gate->immersed    = immersed;
    p_gate->stable      = stable;
    p_gate->blanking    = blanking;

    p_gate->bits = 0U;
    if (sensor_ok)
    {
        p_gate->bits |= (uint16_t)VAL_SENSOR_OK;
    }
    if (baseline_ok)
    {
        p_gate->bits |= (uint16_t)VAL_BASELINE_OK;
    }
    if (immersed)
    {
        p_gate->bits |= (uint16_t)VAL_IMMERSED;
    }
    if (stable)
    {
        p_gate->bits |= (uint16_t)VAL_STABLE;
    }
    if (blanking)
    {
        p_gate->bits |= (uint16_t)VAL_BLANKING;
    }
    /* 本机没有流量计/液位开关：按方案 §9"没有就置 1"处理（不阻塞回收） */
    p_gate->bits |= (uint16_t)VAL_FLOW;

    p_gate->reason = gate_update_reason(p_gate);
}

/* 工况 → 静态字符串。 */
const char *gate_regime_str(regime_t regime)
{
    switch (regime)
    {
        case REGIME_FAULT:
            return "FAULT";

        case REGIME_DRY:
            return "DRY";

        case REGIME_SETTLE:
            return "SETTLE";

        case REGIME_MEASURE:
            return "MEASURE";

        case REGIME_LOCKOUT:
        default:
            return "LOCKOUT";
    }
}

/* 最近一次 gate_update 的原因串（永不返回 NULL）。 */
const char *gate_reason_str(const gate_t *p_gate)
{
    if ((NULL == p_gate) || (NULL == p_gate->reason))
    {
        return g_gate_reason_no_cal;
    }
    return p_gate->reason;
}

/* 位图 → "SNS|BAS|IMM|STA|BLK|FLW"。 */
void gate_bits_str(const gate_t *p_gate, char *p_buf, uint32_t cap)
{
    if (NULL == p_gate)
    {
        if ((NULL != p_buf) && (cap > 0U))
        {
            p_buf[0] = '\0';
        }
        return;
    }
    gate_bits_render(p_gate->bits, p_buf, cap);
}
