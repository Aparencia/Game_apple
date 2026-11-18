/*
 * sensor_ts300b.h
 *
 *  Created on: 2026年10月30日
 *      Author: Administrator
 */

/*
 * sensor_ts300b.h —— TS-300B 模拟浊度模块（与 DFRobot SEN0189 同类）
 * ---------------------------------------------------------------------------
 * 模块电气事实（硬件方案 §4.3）：
 *   - 5 V 供电（本项目从 VIN ≈4.5~4.7 V 取电，与板同开同关）；
 *   - 模拟输出 0~4.5 V，**必须经 1kΩ 串 + 2kΩ 对地分压（≤3.0 V）**再进 P013/AN11；
 *   - 分压后源阻抗 1k∥2k ≈ 667 Ω ≤ 1 kΩ（ADC 源阻抗要求）；
 *   - 无 NTU 标定：本工程只输出"相对浊度变化"，禁止写 NTU / 禁止对标国标限值；
 *   - ⚠ 输出方向以实测为准：常见批次是"清水电压高、越浑越低"（clear_high=true），
 *     也有反相批次（clear_high=false）。做一次清水/浑水对照即可确定。
 *
 * 本驱动只负责：ADC 码 → 分压节点 mV → 还原模块 AO 电压 mV → 合理性检查。
 * "方向归一化 / 相对基线偏离" 由 sensor_pipeline 统一计算（见 sensor_pipeline.h）。
 */
#ifndef SENSOR_TS300B_H
#define SENSOR_TS300B_H

#include "sensor_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TS300B_VREF_MV_DEFAULT  3300U   /* ADC 参考电压 */
#define TS300B_ADC_FULL_SCALE   4095U   /* 12 位 ADC 满量程 */
#define TS300B_RAIL_ADC         4040U   /* ≥4040（节点≈3.26 V → AO≈4.89 V）：分压/接线异常 */
#define TS300B_LOW_ADC          3U      /* ≤3（节点≈0 V → AO≈0 V）：断线/模块未供电嫌疑 */

/* 配置（单实例；ts300b_init 时拷贝进模块内部 static） */
typedef struct st_ts300b_cfg
{
    float div_r1_ohm;   /* 串联电阻 R1（默认 1000 Ω） */
    float div_r2_ohm;   /* 对地电阻 R2（默认 2000 Ω） */
    bool  clear_high;   /* true=清水时 AO 电压高（默认）；反相批次改 false */
} ts300b_cfg_t;

/* 一次转换结果 */
typedef struct st_ts300b_reading
{
    uint16_t raw_adc;     /* 中值后的 ADC 码 */
    uint32_t node_mv;     /* 分压节点电压 mV（真正进 ADC 的电压） */
    uint32_t ao_mv;       /* 还原到模块 AO 引脚的电压 mV = node_mv*(R1+R2)/R2 */
    sensor_status_t status;
} ts300b_reading_t;

/* 初始化（p_cfg 为 NULL 时用默认值：1k/2k、clear_high=true）。 */
void ts300b_init(const ts300b_cfg_t *p_cfg);

/* 核心转换：ADC 码 → 结果。p_out 必须非 NULL。
 * 返回 SENSOR_OK / SENSOR_ERR_PARAM / SENSOR_ERR_RANGE（贴轨/疑似断线）。 */
sensor_status_t ts300b_convert(uint16_t raw_adc, ts300b_reading_t *p_out);

#ifdef __cplusplus
}
#endif
#endif /* SENSOR_TS300B_H */

