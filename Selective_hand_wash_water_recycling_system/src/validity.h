/*
 * validity.h
 *
 *  Created on: 2026年11月16日
 *      Author: Administrator
 */

/*
 * validity.h —— 工况有效性层（L0 有效性位图 + L1 工况机 regime）
 * ---------------------------------------------------------------------------
 * 定位：夹在"采集流水线（sensor_pipeline）"与"水质判别（decision）"之间。
 *       它只回答两个问题：
 *         ① 现在这批数据"能不能判"（validity 位图）；
 *         ② 装置现在处在哪种工况（regime：FAULT / DRY / SETTLE / MEASURE / LOCKOUT）。
 *       它**不回答"水脏不脏"**——那是 decision 的事。
 *
 * 为什么需要这一层（方案 v2 §2/§3）：
 *   现场日志里最危险的不是阈值不准，而是"判据在错误时机生效"：
 *     - 腔内无液体（露空 43.7 mV）比清水还"干净" → 旧固件判 L0 回收空气；
 *     - 水位低于电极（有水但没淹到）读数与"空腔"同一区间 → 旧固件照样回收；
 *     - 探头开路（约 1650 mV 伪值）被判成"脏"而不是"故障"；
 *     - 换向瞬间的电磁/水力扰动被当成"皂液阶跃"。
 *   这些都不是阈值问题，而是"没有工况概念"。本层把"能不能判"从"判什么"里拆出来，
 *   下游 decision 只有在 REGIME_MEASURE 里才允许使用 ΔEC / ΔNTU。
 *
 * 分层约定（单向依赖，下游不许反推上游）：
 *   sensor_pipeline → validity(gate) → decision → app(阀/LED/日志)
 *
 * 位图语义（VAL_*）：
 *   VAL_SENSOR_OK    两路 ADC 无超时、无贴轨、驱动状态可接受；
 *   VAL_BASELINE_OK  基线已建立且落在物理量程内（水中 EC0 ≥ 3×EC_DRY）；
 *   VAL_IMMERSED     电极被水淹到（见 gate_update 里的浸没判据）；
 *   VAL_STABLE       特征量在稳定带内（变化率不超限）；
 *   VAL_BLANKING     换向后的屏蔽窗内（此时判据冻结，但已生效的许可不撤销）；
 *   VAL_FLOW         流量/液位信号（本机没有流量计，恒置 1，见方案 §9）。
 *
 * 重要工程取舍（与方案 v2 §7.1 的差异，必须保留原因说明）：
 *   方案原文把 `!BLANKING` 写进回收许可谓词。直接照做会形成正反馈：
 *     回收 → 阀换向 → 进入 400 ms 屏蔽 → 许可撤销 → 排空 → 再换向 → 再屏蔽……
 *   所以本实现把 BLANKING 的作用定为"**冻结判别**"：屏蔽窗内不更新候选等级、
 *   不跑变化率抢先，但当前已采纳的等级（以及由它导出的许可）保持不变。
 *   这样既切断了换向自激，又不会因为屏蔽而撤销刚做出的决定。
 *
 * 安全侧总原则：
 *   拿不准 → FAULT / DRY / SETTLE → 全部 DRAIN，且禁止输出 L0。
 *   只有"传感器正常 + 基线有效 + 已浸没 + 已稳定 + REGIME_MEASURE"才可能回收。
 */
#ifndef VALIDITY_H
#define VALIDITY_H

#include <stdint.h>
#include <stdbool.h>

#include "sensor_common.h"
#include "sensor_pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------- L0 有效性位图 ---------------------------- */
typedef enum e_validity_bit
{
    VAL_SENSOR_OK   = (1u << 0),   /* 两路 ADC 无超时、无贴轨、状态可接受 */
    VAL_BASELINE_OK = (1u << 1),   /* 基线已标定、量程合理、未被判漂移 */
    VAL_IMMERSED    = (1u << 2),   /* 电极被水淹到（不是"腔里有水"） */
    VAL_STABLE      = (1u << 3),   /* 特征量在稳定带内 */
    VAL_BLANKING    = (1u << 4),   /* 换向后 400 ms 屏蔽窗内 */
    VAL_FLOW        = (1u << 5)    /* 流量/液位（本机无传感器，恒置 1） */
} validity_bit_t;

/* ---------------------------- L1 工况 ---------------------------- */
typedef enum e_regime
{
    REGIME_FAULT = 0,   /* 传感器故障 / 基线无效 / 无法判断工况 → DRAIN + 报故障 */
    REGIME_DRY,         /* 确认未浸没（无液体，或水位低于电极）→ DRAIN */
    REGIME_SETTLE,      /* 刚浸没，等响应稳定（默认 4 s）→ DRAIN */
    REGIME_MEASURE,     /* 浸没 + 稳定 + 基线有效 → 允许判水质 */
    REGIME_LOCKOUT      /* 故障恢复后的观察期（默认 3 s）→ DRAIN */
} regime_t;

/* ---------------------------- 干态指纹来源 ---------------------------- */
/*
 * 为什么需要"来源"：开机自检在用户还没做 D 时也要能区分"空腔/清水"，
 * 所以先给一组保守的默认干态值（DEFAULT）；用户做 D 后升级为 MEASURED。
 * 默认值只影响"是否浸没"的绝对门限，不影响任何"回收"判据（没有水中基线照样 FAULT）。
 */
typedef enum e_dry_source
{
    DRY_SOURCE_NONE = 0,   /* 没有干态指纹 */
    DRY_SOURCE_DEFAULT,    /* 开机自检用的默认值（未实测） */
    DRY_SOURCE_MEASURED    /* 命令 D / 自检采到的实测值 */
} dry_source_t;

/* ---------------------------- 配置 ---------------------------- */
/*
 * 默认值全部在 validity.c 的 g_gate_default_cfg 里给出；
 * app_main 只需覆盖现场要调的那几项（0 值/非法值会被 gate_init 限幅回默认）。
 */
typedef struct st_gate_cfg
{
    /* 浸没判据（方案 §6.1）：g = EC/EC0 ≥ wet_ratio，且 EC ≥ ec_dry_ratio×EC_DRY */
    float    wet_ratio;             /* 默认 0.30：低于它认为是"没淹到电极" */
    float    ec_dry_ratio;          /* 默认 3.0：EC 至少是干态的 3 倍才算"有水" */
    float    ec_dry_max_mv;         /* 默认 200：干态指纹超过它就不可信（防止拿湿态当干态） */
    /* 基线量程校验：EC0 必须落在 [ec0_min, ec0_max]，且 ≥ dry_ratio×EC_DRY */
    float    ec0_min_mv;            /* 默认 50：低于它视为模块未供电/探头悬空 */
    float    ec0_max_mv;            /* 默认 1600：开路伪值约 1650 与脏水本底都要挡住 */
    float    ec0_default_min_mv;    /* 默认 400：干态指纹是 DEFAULT 时，水中基线必须 ≥ 它 */
    /* 浊度故障判据：AO 小于它视为未供电/断线（NTU_RAIL） */
    uint32_t ntu_ao_min_mv;         /* 默认 20 */
    /* 稳定判据（方案 §6.2） */
    float    rate_stable_mv_s;      /* 默认 30 mV/s（约干态噪声 3σ） */
    float    rate_stable_pct_s;     /* 默认 5 %/s（浊度相对变化率） */
    /* 时序（方案 §5 / §10） */
    uint32_t settle_ms;             /* 默认 4000：入水后稳定期 */
    uint32_t blank_ms;              /* 默认 400：换向屏蔽窗 */
    uint32_t wet_debounce_ms;       /* 默认 500：DRY→SETTLE 需连续浸没 */
    uint32_t dry_debounce_ms;       /* 默认 300：MEASURE→DRY 需连续失浸没（安全侧要快） */
    uint32_t unstable_ms;           /* 默认 1000：MEASURE→SETTLE 需连续不稳定 */
    uint32_t fault_recover_ms;      /* 默认 1000：故障条件消失后进入观察期 */
    uint32_t lockout_ms;            /* 默认 3000：观察期长度 */
} gate_cfg_t;

/* ---------------------------- 句柄 ---------------------------- */
typedef struct st_gate
{
    gate_cfg_t cfg;

    /* 干态指纹（命令 D 现场标定；没有它就无法区分"空腔"与"清水"） */
    bool       dry_valid;
    dry_source_t dry_source;        /* 干态指纹来源：NONE / DEFAULT / MEASURED */
    float      ec_dry_mv;           /* 空腔时 TDS 的 mV25 均值（本次实测 58） */
    float      ao_dry_mv;           /* 空腔时浊度 AO 均值（本机约 2337） */

    /* 工况 */
    regime_t   regime;
    uint32_t   regime_since_ms;     /* 进入当前工况的时刻 */
    /*
     * 去抖计时（每个条件独立计时，不能共用一个 pending）。
     * 早期版本共用 pending_since_ms，结果"不稳定计时"跑满后，紧接着一个失浸没采样
     * 会立刻满足 300 ms 去抖（计时被别的条件提前消耗）——现场表现就是单点抖动直接落 DRY。
     */
    uint32_t   wet_since_ms;        /* 连续浸没起始（DRY→SETTLE） */
    uint32_t   dry_since_ms;        /* 连续失浸没起始（MEASURE/SETTLE→DRY） */
    uint32_t   unstable_since_ms;   /* 连续不稳定起始（MEASURE→SETTLE） */
    uint32_t   fault_since_ms;      /* 连续"故障条件消失"起始（FAULT→LOCKOUT） */
    bool       first_update;        /* 上电第一次更新：允许直接落到 DRY */

    /* 换向屏蔽 */
    bool       switch_seen;         /* 是否发生过物理换向 */
    uint32_t   last_switch_ms;      /* 最近一次物理换向时刻 */

    /* 本 tick 的判据结果 */
    bool       sensor_ok;
    bool       baseline_ok;
    bool       immersed;
    bool       stable;
    bool       blanking;
    uint16_t   bits;                /* validity_bit_t 组合 */
    const char *reason;             /* 静态原因串（日志用，不参与控制） */
} gate_t;

/*
 * 函数名：gate_init
 * 作用：保存配置（限幅 + 补默认）、清零运行时状态，工况置 FAULT（安全侧）。
 * 参数：p_gate —— 句柄（非 NULL）；p_cfg —— 配置（可为 NULL，NULL 时全用默认）。
 * 返回：无。
 * 时序/边界：上电调用一次；可重复调用（会重置工况与干态指纹之外的运行状态）。
 * 注意：干态指纹属于"标定数据"，gate_init 会清掉它——掉电保存属于 P2，当前需重新标定。
 */
void gate_init(gate_t *p_gate, const gate_cfg_t *p_cfg);

/*
 * 函数名：gate_set_dry_fingerprint
 * 作用：记录干态指纹（命令 D 采到的空腔均值），它是"浸没判据"的参考零点。
 * 参数：p_gate —— 句柄；ec_dry_mv —— 空腔 TDS mV25 均值；ao_dry_mv —— 空腔浊度 AO 均值。
 * 返回：true=接受；false=参数不可信（越界/NaN，保持原指纹不变）。
 * 时序/边界：只在命令 D 采集完成时调用；运行期只读。
 * 注意：EC_DRY 必须明显小于水中值（默认 ≤200 mV），否则说明采样时探头还在水里。
 */
bool gate_set_dry_fingerprint(gate_t *p_gate, float ec_dry_mv, float ao_dry_mv);

/*
 * 函数名：gate_set_dry_default
 * 作用：设置"默认干态指纹"（开机自检用，未经实测），来源标记为 DRY_SOURCE_DEFAULT。
 * 参数：p_gate —— 句柄；ec_dry_mv —— 默认空腔 EC（默认 60 mV）；ao_dry_mv —— 默认空腔 AO。
 * 返回：true=接受；false=参数不可信（保持原指纹不变）。
 * 时序/边界：上电 app_main 初始化后调用一次；随后命令 D / 自检实测会覆盖为 MEASURED。
 * 注意：默认值只用于"是否浸没"的绝对门限；没有水中基线时工况仍是 FAULT，不会回收。
 */
bool gate_set_dry_default(gate_t *p_gate, float ec_dry_mv, float ao_dry_mv);

/* 干态指纹来源（NONE / DEFAULT / MEASURED）。 */
dry_source_t gate_dry_source(const gate_t *p_gate);

/* 读取干态指纹（p_ec_dry_mv / p_ao_dry_mv 可为 NULL）。返回 false=尚未标定。 */
bool gate_get_dry_fingerprint(const gate_t *p_gate, float *p_ec_dry_mv, float *p_ao_dry_mv);

/* 清除干态指纹（标定作废/现场换探头后使用）。 */
void gate_clear_dry_fingerprint(gate_t *p_gate);

/*
 * 函数名：gate_note_switch
 * 作用：通知"刚刚发生了一次物理换向"，作为 400 ms 屏蔽窗的起点。
 * 参数：p_gate —— 句柄；now_ms —— 换向时刻。
 * 返回：无。
 * 时序/边界：由 app 在检测到阀位置变化后调用（可在 decision_update 之后同一 tick）。
 * 注意：屏蔽窗只冻结判别，不撤销已生效的回收许可（原因见文件头）。
 */
void gate_note_switch(gate_t *p_gate, uint32_t now_ms);

/*
 * 函数名：gate_update
 * 作用：每 50 ms tick 计算一次 validity 位图与 regime，驱动工况迁移。
 * 参数：p_gate —— 句柄；p_out —— 本 tick 的流水线输出（只读）；now_ms —— 当前时刻。
 * 返回：无（结果在 p_gate 内）。
 * 时序/边界：必须在 pipeline_sample 之后、decision_update 之前调用；
 *             迁移去抖全部基于 now_ms，无符号差值比较，跨 49.7 天回绕安全。
 * 注意：
 *   ① FAULT 的判定是"立即"的（传感器故障/基线失效不等去抖）；
 *   ② "无基线但已确认没水"是 DRY 而不是 FAULT——否则空腔上电会一直报故障，
 *      而现场需要一眼看出"现在只是没水"（见方案 §13 P0-1 的验收）；
 *   ③ 进入 SETTLE→MEASURE 还要求 STABLE：只等时间不看稳定性，会在气泡里误判。
 */
void gate_update(gate_t *p_gate, const pipeline_out_t *p_out, uint32_t now_ms);

/* 工况 → 静态字符串（"FAULT"/"DRY"/"SETTLE"/"MEASURE"/"LOCKOUT"）。 */
const char *gate_regime_str(regime_t regime);

/* 最近一次 gate_update 给出的静态原因串（永不返回 NULL）。 */
const char *gate_reason_str(const gate_t *p_gate);

/* 位图 → 可读字符串之辅助：在 p_buf 里拼 "SNS|BAS|IMM|STA|BLK|FLW"（容量不足自动截断）。 */
void gate_bits_str(const gate_t *p_gate, char *p_buf, uint32_t cap);

#ifdef __cplusplus
}
#endif
#endif /* VALIDITY_H */
