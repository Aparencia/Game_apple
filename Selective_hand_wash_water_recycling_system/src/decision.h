/*
 * decision.h —— 水质判别（L2 分类）+ 回收许可（L3 谓词）
 * ---------------------------------------------------------------------------
 * v2 变更（2026-11-16，依据 Docs/软件层优化方案_v2_20261116.md）：
 *   旧模型是"事件闩锁"：单点 L2 即 auto_start 开启事件，事件内才可能回收，
 *   事件只能靠 30 s 超时关闭，而关闭动作本身又会触发重开 —— 于是出现现场看到的
 *   "30 s 一周期自激"、"只在开机进待机"、以及"回收空气/回收没淹到电极的水"。
 *
 *   新模型是"工况门控 + 回收许可"：
 *     ① 工况由 validity(gate) 决定：只有 REGIME_MEASURE 才允许做水质分类；
 *     ② 回收不再是"事件状态"，而是**每个 tick 重算的谓词**：
 *            PERMIT = (level == L0)
 *        其中 L0 只可能在 REGIME_MEASURE（已要求"浸没 + 稳定"）里产生；
 *        谓词为假的那一 tick 就关阀（≤50 ms），没有"超时"这个概念；
 *     ③ 事件（串口 R/E）降级为**纯日志分段标记**，不参与控制。
 *
 * 判据（L2，只在 REGIME_MEASURE 里运行）：
 *   ① ΔEC（主判据）：ΔEC > d2 → L2；ΔEC < d1 → 可清；中间 → L1；
 *   ② ΔNTU（辅判据）：相对基线百分比，浊度突增同样判 L2；
 *   ③ 双侧门限：ΔEC < -d2（或 ΔNTU < -d2）同样拦下 → L1（电极脱水/气泡/水位下降）；
 *   ④ 变化率抢先：仅正向、≥120 mV/s、连续 2 tick、换向后 400 ms 内禁用；
 *   ⑤ 块内一致：400 ms 块内 8 个采样里 ≥6 一致才动作；
 *   ⑥ 单侧迟滞：变脏/拿不准立即生效；升级到 L0 需连续 3 个块（约 1.2 s）；
 *   ⑦ 安全侧：无基线、传感器故障、工况非 MEASURE → 一律 L1 排空。
 *
 * 状态（CSV 兼容，取值不变）：
 *   WAIT       非测量工况（DRY/SETTLE/FAULT/LOCKOUT）→ 恒排空
 *   CLEAR      L0 且许可成立 → 回收（唯一会输出 RECOV 的状态）
 *   UNCERTAIN  L1 拿不准 → 排空
 *   SOAP       L2 含皂液特征 → 排空
 *   FLUSH      保留枚举值（PC 端脚本兼容），v2 起不再输出
 */
#ifndef DECISION_H
#define DECISION_H

#include "sensor_common.h"
#include "sensor_pipeline.h"
#include "validity.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DECISION_MAX_BLOCK_SAMPLES  8U

typedef enum e_decision_state
{
    DEC_STATE_WAIT = 0,   /* 非测量工况（DRY/SETTLE/FAULT/LOCKOUT） */
    DEC_STATE_FLUSH,      /* 保留值：v2 起不再输出（原"手动首段排空"） */
    DEC_STATE_CLEAR,      /* L0 且许可成立：回收 */
    DEC_STATE_UNCERTAIN,  /* L1：拿不准，排空 */
    DEC_STATE_SOAP        /* L2：含皂液特征，排空 */
} decision_state_t;

typedef enum e_decision_level
{
    DEC_LEVEL_L0 = 0,     /* 可回收 */
    DEC_LEVEL_L1,         /* 拿不准 → 排空（安全侧） */
    DEC_LEVEL_L2          /* 应放弃 → 排空 */
} decision_level_t;

typedef enum e_decision_action
{
    DEC_ACTION_DRAIN = 0, /* 排空（阀不动作/默认位） */
    DEC_ACTION_RECOVER    /* 回收（阀动作位） */
} decision_action_t;

/* 配置：默认值在 app_main.c 里给出并附标定说明 */
typedef struct st_decision_cfg
{
    /* EC 阈值（σ 倍数 + 绝对下限 + EC0 百分比，取最大者，单位 mV）
     * v2.2：现场实测清水 ΔEC 漂移约 ±5%×EC0，而固定 8 mV 只有 2.3%×EC0（EC0=348），
     * 导致清水在 CLEAR↔UNCERT 之间 2 s 周期抖动。增加按 EC0 的百分比下限。 */
    float    ec_d1_sigma;        /* 进入"可清"的 σ 倍数，默认 4.0 */
    float    ec_d2_sigma;        /* 判定"含皂液"的 σ 倍数，默认 8.0 */
    float    ec_d1_min_mv;       /* d1 绝对下限，默认 8.0 mV */
    float    ec_d2_min_mv;       /* d2 绝对下限，默认 20.0 mV */
    float    ec_d1_pct;          /* d1 相对 EC0 的百分比下限，默认 6.0 %（348→20.9 mV） */
    float    ec_d2_pct;          /* d2 相对 EC0 的百分比下限，默认 9.0 %（348→31.3 mV） */
    float    ec_hyst_ratio;      /* EC 回差比例，默认 0.25 */
    /* 浊度阈值（相对基线百分比） */
    float    ntu_d1_pct;         /* 浊度"可清"上限，默认 8.0 % */
    float    ntu_d2_pct;         /* 浊度"明显变脏"阈值，默认 25.0 % */
    float    ntu_hyst_ratio;     /* 浊度回差比例，默认 0.25 */
    /* 变化率抢先（方案 §6.4 四约束） */
    float    ec_rate_th_mv_s;    /* ΔEC 正向变化率阈值，默认 120.0 mV/s */
    float    ec_rate_min_jump_mv;/* 触发所需最小绝对跳变，默认 5.0 mV */
    float    ntu_rate_th_pct_s;  /* 浊度变化率阈值，默认 15.0 %/s（肥皂前沿≈16） */
    uint32_t soap_latch_ms;      /* 检测到 L2 后强制 DRAIN 的锁存时间，默认 45000 ms */
    uint8_t  rate_confirm_ticks; /* 连续满足多少个 tick 才抢先，默认 2 */
    /* 块内一致（方案 §6.5） */
    uint32_t block_ms;           /* 消抖时间块长度，默认 400 ms */
    uint8_t  block_samples;      /* 时间块内采样数，默认 8（50 ms × 8） */
    uint8_t  block_agree;        /* 块内一致所需个数，默认 6 */
    uint8_t  upgrade_blocks;     /* 升级到 L0 需要的连续块数，默认 3 */
    uint8_t  downgrade_blocks;   /* L0→L1 需要的连续块数，默认 2（L0→L2 仍立即） */
    /* 工况机配置（validity.c） */
    gate_cfg_t gate;
} decision_cfg_t;

/* 判别器输出（app 打印/控阀用） */
typedef struct st_decision_out
{
    decision_state_t  state;
    decision_level_t  level;
    decision_action_t action;
    bool              event_active;   /* 事件标记（v2 起仅用于日志分段） */
    bool              changed;        /* 本次 state/level/action/permit 是否变化 */
    bool              permit;         /* 回收许可（本 tick 重算，非闩锁） */
    /* 工况与有效性（方案 §4/§5） */
    uint16_t          valid_bits;     /* validity_bit_t 组合 */
    regime_t          regime;         /* 当前工况 */
    bool              immersed;       /* 电极已浸没 */
    bool              blanking;       /* 换向屏蔽窗内（判别冻结） */
    /* 本 tick 生效的阈值（随 σ 自适应） */
    float             d1_ec_mv;
    float             d2_ec_mv;
    float             d1_ntu_pct;
    float             d2_ntu_pct;
    const char       *reason;         /* 静态原因字符串（见 decision.c） */
    const char       *gate_reason;    /* 工况原因（见 validity.c） */
} decision_out_t;

/* 判别器句柄（app_main 分配） */
typedef struct st_decision
{
    decision_cfg_t    cfg;
    decision_out_t    out;
    gate_t            gate;              /* 工况机（L0+L1） */
    uint32_t          state_since_ms;
    uint32_t          event_since_ms;
    uint32_t          block_start_ms;
    uint8_t           verdict_ring[DECISION_MAX_BLOCK_SAMPLES];
    uint8_t           verdict_pos;
    uint8_t           verdict_len;
    bool              event_active;
    uint8_t           rate_hits;         /* 变化率抢先连续计数 */
    uint8_t           upgrade_blocks;    /* 连续"可清"块计数（单侧迟滞） */
    uint8_t           downgrade_hits;    /* 连续"不确定"块计数（L0→L1 去抖） */
    bool              blank_active;      /* 上一 tick 是否在屏蔽窗内（边沿检测） */
    bool              soap_latch_active;  /* M2h：L2 后强制 DRAIN 锁存 */
    uint32_t          soap_latch_until_ms;
} decision_t;

/* 初始化（p_cfg 可为 NULL；内部限幅并初始化工况机）。 */
void decision_init(decision_t *p_dec, const decision_cfg_t *p_cfg);

/* 事件标记：串口 R（开始）/ E（结束）。v2 起只做日志分段，不改变控制。 */
void decision_start_event(decision_t *p_dec, uint32_t now_ms);
void decision_end_event(decision_t *p_dec, uint32_t now_ms);

/* 阀换向通知：由 app 在检测到物理换向时调用，作为 400 ms 屏蔽窗起点。 */
void decision_note_valve_switch(decision_t *p_dec, uint32_t now_ms);

/* 每 50 ms tick 调用一次：先跑工况机，再（仅 MEASURE 时）分类并重算许可。 */
void decision_update(decision_t *p_dec, const sensor_pipeline_t *p_pipe, uint32_t now_ms);

/* M2h：清除 SOAP 锁存（重新标定/清基线时调用）。 */
void decision_clear_soap_latch(decision_t *p_dec);

/* 读取最近一次输出（只读）。 */
const decision_out_t *decision_get_out(const decision_t *p_dec);

/* 状态/等级/动作 → 静态字符串（CSV 与日志用）。 */
const char *decision_state_str(decision_state_t state);
const char *decision_level_str(decision_level_t level);
const char *decision_action_str(decision_action_t action);

#ifdef __cplusplus
}
#endif
#endif /* DECISION_H */
