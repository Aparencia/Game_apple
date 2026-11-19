/*
 * sensor_pipeline.h
 *
 *  Created on: 2026年10月30日
 *      Author: Administrator
 */

/*
 * sensor_pipeline.h —— 采集与特征流水线（双通道 ADC + 中值 + EMA + 基线 + Δ特征）
 * ---------------------------------------------------------------------------
 * 数据流（每 50 ms 一个 tick）：
 *   ADC 扫描（AN12=TDS、AN11=浊度）→ 每通道 N 次取中值
 *   → TDS：mV/温补 mV25/参考 ppm；浊度：节点 mV/AO mV
 *   → EMA 轻度平滑 → 相对基线 ΔEC、ΔNTU（方向归一化）、变化率
 *
 * 基线（本底 EC0 / NTU0，v2 起只允许命令 W 显式采集）：
 *   - 命令 W 进入"基线采集"：连续采 baseline_n 个 tick（默认 40 = 2 s），算均值与标准差 σ；
 *   - σ 过小会被下限保护（sigma_floor_*），避免阈值退化为 0；
 *   - v2 新增量程/稳定性校验：EC0 ∈ [50 mV, ec0_max_mv]、σ/EC0 ≤ CV 上限（或 σ 很小）；
 *   - 基线无效时，判别层按"安全侧"一律排空（见 decision.h）。
 *   - ⚠ 基线必须在水样中采集且电极浸没：探头在空气里是开路伪值，采出的基线是错的；
 *     上电不再自动采集（那是现场"回收空气"的根因），必须 D→W 显式标定。
 *
 * 约定：
 *   - 所有电压单位 mV；ΔEC 用 25 ℃ 补偿后的电压（mV25）；
 *   - ΔNTU 用"方向归一化后的电压差"（>0 表示比基线更脏）与相对百分比；
 *   - 本模块不打印、不控阀，只产出 pipeline_out_t 供 decision/app 使用。
 */
#ifndef SENSOR_PIPELINE_H
#define SENSOR_PIPELINE_H

#include "hal_data.h"
#include "sensor_common.h"
#include "sensor_tds.h"
#include "sensor_ts300b.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PIPELINE_RATE_POINTS   8U      /* 变化率窗口样本数（50 ms × 8 = 400 ms） */
#define PIPELINE_MAX_MEDIAN_N  9U      /* 每通道中值采样次数的上限 */

/* ---------------------------- 配置 ---------------------------- */
typedef struct st_pipeline_cfg
{
    adc_instance_ctrl_t *p_adc_ctrl;      /* ADC 实例的控制结构（&g_adc0_ctrl） */
    adc_channel_t        tds_channel;     /* TDS 通道：ADC_CHANNEL_12（P014/AN12） */
    adc_channel_t        ntu_channel;     /* 浊度通道：ADC_CHANNEL_11（P013/AN11） */
    uint16_t             adc_median_n;    /* 每个 tick 每通道采样次数（默认 5，限幅 1~9） */
    uint32_t             adc_timeout_ms;  /* 单次扫描超时（默认 5 ms） */
    uint16_t             vref_mv;         /* 3300 */
    uint16_t             adc_full_scale;  /* 4095 */
    float                ema_alpha;       /* EMA 系数（默认 0.30，限幅 0.02~1.0） */
    float                tds_cal_k;       /* TDS 标定系数（默认 1.0） */
    float                ntu_div_r1_ohm;  /* 浊度分压上臂（默认 1000） */
    float                ntu_div_r2_ohm;  /* 浊度分压下臂（默认 2000） */
    bool                 ntu_clear_high;  /* 浊度方向：true=清水电压高 */
    uint16_t             baseline_n;      /* 基线采样点数（默认 40，即 2 s） */
    float                sigma_floor_ec_mv;   /* EC 噪声下限（默认 1.5 mV） */
    float                sigma_floor_ntu_mv;  /* 浊度噪声下限（默认 2.0 mV） */
    /*
     * 基线有效性校验（v2 新增，方案 §8.1）：
     *   - 相对稳定性：σ/EC0 ≤ baseline_cv_max_pct（默认 1.0 %）；
     *   - 绝对稳定性兜底：σ ≤ baseline_sigma_abs_max_mv（默认 3.0 mV）。
     *     纯水 EC0 可能只有几十~一百 mV，1 % 相对判据在传感器噪声下限下不可达，
     *     所以"σ 很小"也算通过（两者取或）。
     *   - EC0 量程：[PIPELINE_MIN_VALID_EC0_MV, ec0_max_mv]；上限默认 2000 mV，
     *     用来排除探头开路时的 ~1650 mV 伪值。
     */
    float                baseline_cv_max_pct;      /* 默认 1.0 % */
    float                baseline_sigma_abs_max_mv;/* 默认 3.0 mV */
    float                ec0_max_mv;               /* 默认 2000 mV */
} pipeline_cfg_t;

/* ------------------------- 一次 tick 的输出 ------------------------- */
typedef struct st_pipeline_out
{
    /* 原始/换算量 */
    uint16_t tds_raw;         /* TDS 通道中值 ADC 码 */
    uint32_t tds_mv;          /* TDS 电压 mV */
    float    tds_mv25;        /* 温度补偿后 mV25 */
    float    tds_ppm25;       /* 参考 TDS ppm（仅参考） */
    uint16_t ntu_raw;         /* 浊度通道中值 ADC 码 */
    uint32_t ntu_node_mv;     /* 分压节点电压 mV */
    uint32_t ntu_ao_mv;       /* 还原的模块 AO 电压 mV */
    bool     tds_rail;        /* TDS 贴轨 */
    bool     tds_saturated;   /* TDS 超过模块标称量程 */
    sensor_status_t tds_status;
    sensor_status_t ntu_status;
    bool     adc_timeout;     /* 本 tick ADC 超时（数据不可信） */

    /* 基线与噪声 */
    bool     baseline_valid;  /* 是否已有有效基线 */
    bool     baseline_updated;/* 本次刚完成基线采集（供 app 打印一次日志） */
    uint16_t baseline_n;      /* 已采到的基线点数（采集过程中显示进度） */
    float    ec0_mv25;        /* 本底 EC0（mV25） */
    uint32_t ntu0_ao_mv;      /* 本底浊度 AO 电压 mV */
    float    ec_sigma_mv;     /* 本底 EC 噪声 σ（mV） */
    float    ntu_sigma_mv;    /* 本底浊度噪声 σ（mV） */

    /* 相对特征 */
    float    d_ec_mv;         /* ΔEC = mv25(EMA) - EC0 */
    float    g_ec;            /* g = mv25/EC0（1.0=与本底一致） */
    float    d_ec_pct;        /* ΔEC/EC0 × 100 % */
    float    d_ntu_mv;        /* 方向归一化浊度偏离（>0=更脏），mV */
    float    d_ntu_pct;       /* d_ntu_mv/ntu0 × 100 % */
    float    ec_rate_mv_s;    /* d(ΔEC)/dt，mV/s（EMA 平滑） */
    float    ntu_rate_mv_s;   /* d(ΔNTU)/dt，mV/s（EMA 平滑） */

    /* 水温与提示 */
    float    water_temp_c;    /* 本次使用的温度 */
    bool     water_temp_from_sensor; /* false=手输/兜底（温补精度下降） */
    char     note[28];        /* OK / ADC_TIMEOUT / NO_BASE / TDS_RAIL / TDS_SAT / NTU_RAIL */
} pipeline_out_t;

/* ------------------------- 三基线槽（B3 方案） ------------------------- */
#define PIPELINE_BASELINE_SLOTS 3U
typedef enum e_pipeline_baseline_slot
{
    PIPELINE_SLOT_DRAIN = 0,     /* 阀 DRAIN、静态液位 */
    PIPELINE_SLOT_RECOVER = 1,   /* 阀 RECOVER、静态液位 */
    PIPELINE_SLOT_FULL = 2,      /* 进水流动、腔体满 */
    PIPELINE_SLOT_COUNT = 3
} pipeline_baseline_slot_t;

typedef struct st_pipeline_baseline_slot_data
{
    bool     valid;
    float    ec0_mv25;
    uint32_t ntu0_ao_mv;
    float    ec_sigma_mv;
    float    ntu_sigma_mv;
} pipeline_baseline_slot_data_t;

/* ------------------------- 流水线句柄 ------------------------- */
typedef struct st_sensor_pipeline
{
    pipeline_cfg_t cfg;
    pipeline_out_t out;
    tds_cfg_t      tds_cfg_storage;
    ts300b_cfg_t   ntu_cfg_storage;
    bool           initialized;
    float          water_temp_c;
    bool           water_temp_from_sensor;

    /* B3：按阀位/流动状态保存三条基线，active_slot 指向当前生效的一条 */
    pipeline_baseline_slot_data_t slots[PIPELINE_BASELINE_SLOTS];
    uint8_t        active_slot;

    /* 基线采集 */
    bool           baseline_capturing;
    uint16_t       baseline_count;
    float          base_ec_sum;
    float          base_ec_sumsq;
    float          base_ntu_sum;
    float          base_ntu_sumsq;
    const char    *baseline_reject;   /* 非 NULL=上次基线被作废/校验失败的原因（静态串） */

    /* EMA */
    float          ec_ema;
    float          ntu_ema;

    /* 变化率窗口（环形） */
    float          rate_d[PIPELINE_RATE_POINTS];
    uint32_t       rate_t[PIPELINE_RATE_POINTS];
    uint8_t        rate_pos;
    uint8_t        rate_len;
    float          rate_ec_ema;
    float          rate_ntu_ema;
} sensor_pipeline_t;

/* 初始化（p_cfg 必须非 NULL；内部会拷贝配置并对非法值做限幅）。 */
void pipeline_init(sensor_pipeline_t *p_pipe, const pipeline_cfg_t *p_cfg);

/* 由 ADC 回调调用（app_main.c 的 adc_callback 里）。仅置位内部完成标志。 */
void pipeline_on_adc_complete(void);

/* 开始一次基线采集（上电自动/命令 B）。now_ms 用于日志时间戳。 */
void pipeline_baseline_begin(sensor_pipeline_t *p_pipe, uint32_t now_ms);

/* 是否正在采集基线。 */
bool pipeline_baseline_capturing(const sensor_pipeline_t *p_pipe);

/*
 * 函数名：pipeline_invalidate_baseline
 * 作用：作废当前基线（保留 ec0 数值供诊断，但 baseline_valid=false、特征清零）。
 * 参数：p_pipe —— 流水线句柄；p_note —— 作废原因（静态串，如 "BASE_LOW" / "CLEARED"）。
 * 返回：无。
 * 时序/边界：命令 Z（清基线）、W 采集后"EC0 < 3×干态"等场景调用；下一 tick 起
 *             decision 的 REGIME 会离开 MEASURE（安全侧）。
 * 注意：与 pipeline_baseline_begin 的区别——后者会重新开始采集，本函数只作废。
 */
void pipeline_invalidate_baseline(sensor_pipeline_t *p_pipe, const char *p_note);

/*
 * 双基线槽（B 方案）：
 *   - store：把当前 out 中有效的基线存入 slot；
 *   - select：把 slot 设为 active，并重置 EMA/速率，下一 tick 用新基线算 Δ；
 *   - set_default：跳过标定时写入默认基线；
 *   - clear_slots：清空三条（Z / 重新标定）。
 */
bool pipeline_baseline_store_slot(sensor_pipeline_t *p_pipe, uint8_t slot);
bool pipeline_baseline_select_slot(sensor_pipeline_t *p_pipe, uint8_t slot);
bool pipeline_baseline_slot_valid(const sensor_pipeline_t *p_pipe, uint8_t slot);
bool pipeline_baseline_set_default(sensor_pipeline_t *p_pipe, uint8_t slot,
                                   float ec0_mv25, uint32_t ntu0_ao_mv);
void pipeline_baseline_clear_slots(sensor_pipeline_t *p_pipe);

/* 设置当前水温（app 每次 tick 从 ds18b20 取；from_sensor=false 表示手输/兜底）。 */
void pipeline_set_water_temp(sensor_pipeline_t *p_pipe, float temp_c, bool from_sensor);

/* 执行一个 tick 的采样+换算+滤波+特征计算。返回 false=ADC 超时（out 中标记）。 */
bool pipeline_sample(sensor_pipeline_t *p_pipe, uint32_t now_ms);

/* 读取最近一次输出（只读指针）。 */
const pipeline_out_t *pipeline_get_out(const sensor_pipeline_t *p_pipe);

#ifdef __cplusplus
}
#endif
#endif /* SENSOR_PIPELINE_H */

