/*
 * sensor_tds.h
 *
 *  Created on: 2026年10月30日
 *      Author: Administrator
 */

/*
 * sensor_tds.h —— 模拟 TDS（电导率）传感器：ADC 码 → 电压 → 温度补偿 → 参考 TDS
 * ---------------------------------------------------------------------------
 * 模块电气事实（实验 2 实测 + 厂商资料，见 docs_sensor/00 与硬件方案 §4.1）：
 *   - 供电 3.3 V；模拟输出标称 0~2.3 V（实验 2 实测最高 2356 mV）；
 *   - 正相：离子浓度↑ → 输出电压↑（只要相对变化，不做方向猜测）；
 *   - 输出直连 P014/AN12（0~2.3 V < 3.0 V，直连安全）；
 *   - 探头在空气里是"开路伪值"（约 1.6 V），不是 0 ppm —— 不能拿空气当基线。
 *
 * 温度补偿（实验 2 报告 2.3 节，电解质电导率约 +2%/℃）：
 *   V25 = V / (1 + 0.02 * (T - 25))
 *
 * 参考换算（DFRobot 经验多项式，仅作过程参考，无标准液时禁止当绝对值/达标依据）：
 *   ppm = (133.42*V^3 - 255.86*V^2 + 857.39*V) * 0.5 * K
 */
#ifndef SENSOR_TDS_H
#define SENSOR_TDS_H

#include "sensor_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TDS_VREF_MV_DEFAULT   3300U     /* ADC 参考电压（板载 3.3 V） */
#define TDS_ADC_FULL_SCALE    4095U     /* 12 位 ADC 满量程 */
#define TDS_RAIL_ADC          4000U     /* ≥4000（≈3.22 V）：几乎必然接错/过压 */
#define TDS_SAT_MV            2400U     /* >2400 mV：超过模块标称 2.3 V，可能饱和 */
#define TDS_ALPHA_PER_C       0.02f     /* 电导率温度系数 ≈ 2 %/℃ */
#define TDS_TEMP_REF_C        25.0f     /* 温度补偿参考温度 */
#define TDS_TEMP_MIN_C        (-5.0f)   /* 温补时温度限幅下限（防异常值） */
#define TDS_TEMP_MAX_C        60.0f     /* 温补时温度限幅上限 */
#define TDS_COEFF_MIN         0.20f     /* 温补系数的下限（防除零/放大失控） */
#define TDS_PPM_LIMIT         2000.0f   /* 参考 ppm 限幅上限（仅防打印异常） */

/* 配置（本工程单实例；tds_init 时拷贝进模块内部 static） */
typedef struct st_tds_cfg
{
    float cal_k;    /* 标定系数：默认 1.0；有 1413 µS/cm 标准液后按实测修正 */
} tds_cfg_t;

/* 一次转换结果 */
typedef struct st_tds_reading
{
    uint16_t raw_adc;    /* 中值后的 ADC 码 */
    uint32_t mv;         /* 实测电压 mV */
    float    mv25;       /* 温度补偿到 25 ℃ 的电压 mV */
    float    ppm25;      /* 参考 TDS ppm（仅参考，不写进结论） */
    bool     rail;       /* ADC 贴轨：疑接错线/过压 */
    bool     saturated;  /* 超过模块标称量程上限，响应可能被压缩 */
    sensor_status_t status;
} tds_reading_t;

/* 初始化（p_cfg 为 NULL 时用 cal_k=1.0）。可重复调用。 */
void tds_init(const tds_cfg_t *p_cfg);

/* 核心转换：ADC 码 + 水温 → 结果。p_out 必须非 NULL。
 * 返回 SENSOR_OK / SENSOR_ERR_PARAM / SENSOR_ERR_RANGE（rail 时）。 */
sensor_status_t tds_convert(uint16_t raw_adc, float water_temp_c, tds_reading_t *p_out);

/* 只做 25 ℃ 电压 → 参考 ppm（供标定/复算用）。 */
float tds_ppm_from_mv25(float mv25);

#ifdef __cplusplus
}
#endif
#endif /* SENSOR_TDS_H */

