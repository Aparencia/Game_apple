/*
 * sensor_ds18b20.h
 *
 *  Created on: 2026年10月30日
 *      Author: Administrator
 */

/*
 * sensor_ds18b20.h —— DS18B20 单总线水温传感器（P500 / U1-17，4.7k 上拉到 3.3 V）
 * ---------------------------------------------------------------------------
 * 用途：TDS 温度补偿必须用"水侧水温"。片上 TSN 测的是芯片温度，不能替代
 *       （硬件方案 §4.2/§13.1：温差 10~15 ℃ 会引入 20~30% 温补误差）。
 *
 * 实现要点（来自已验证固件 tds_mini v1.2，实测可用）：
 *   - 用 PFS 寄存器直接位操作模拟 1-Wire（微秒级切换方向比 HAL 快）；
 *   - 时序按 DS18B20 数据手册：复位 480µs / 写 0 60µs / 写 1 3µs / 读 3µs+采样；
 *   - 事务期间短暂关中断（约 1~6 ms），1-Wire 时序才稳；
 *   - Dallas/Maxim CRC8 校验 + "暂存器全 0/全 FF 判无效"（DQ 被拉死时 CRC 可能
 *     恰为 0，会假装读到 0.00 ℃ —— 实验 2 真实踩坑，必须挡掉）；
 *   - 温度范围 -20~80 ℃；85.0 ℃（上电默认/转换未完成）判无效；
 *   - 连续 3 次无效自动停用，回退 25.0 ℃/手输值；命令 C 可重新检测。
 *
 * 非阻塞设计：主循环每 2 s 发起一次转换，约 800 ms 后回来读数，不阻塞采样。
 */
#ifndef SENSOR_DS18B20_H
#define SENSOR_DS18B20_H

#include "hal_data.h"
#include "sensor_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DS18B20_PIN_DEFAULT          BSP_IO_PORT_05_PIN_00  /* P500 / U1-17 */
#define DS18B20_REQUEST_PERIOD_MS    2000U   /* 发起温度转换的周期 */
#define DS18B20_CONV_WAIT_MS         800U    /* 12 位转换最长 750 ms + 余量 */
#define DS18B20_REDETECT_PERIOD_MS   5000U   /* 未发现器件时的重试周期 */
#define DS18B20_FAIL_LIMIT           3U      /* 连续无效次数达到即自动停用 */
#define DS18B20_DEFAULT_TEMP_C       25.0f   /* 兜底水温（无传感器且未手输） */
#define DS18B20_TEMP_MIN_C           (-20.0f)
#define DS18B20_TEMP_MAX_C           80.0f
#define DS18B20_POWER_ON_TEMP_C      85.0f   /* 上电默认值/未完成标志 */

typedef enum e_ds18b20_src
{
    DS18B20_SRC_FIX = 0,   /* 默认 25.0 ℃ */
    DS18B20_SRC_MANUAL,    /* 串口 T 命令手输 */
    DS18B20_SRC_SENSOR     /* DS18B20 实测 */
} ds18b20_src_t;

/* 配置（ds18b20_init 时拷贝进句柄） */
typedef struct st_ds18b20_cfg
{
    bsp_io_port_pin_t pin;                 /* 数据脚（默认 P500） */
    uint32_t request_period_ms;            /* 发起转换周期 */
    uint32_t conv_wait_ms;                 /* 转换等待时间 */
    uint32_t redetect_period_ms;           /* 重新检测周期 */
    uint8_t  fail_limit;                   /* 连续无效自动停用阈值 */
    float    default_temp_c;               /* 兜底温度 */
} ds18b20_cfg_t;

/* 运行状态（由 app_main 分配；第三方只读 temp_c/src/present/disabled） */
typedef struct st_ds18b20
{
    ds18b20_cfg_t cfg;
    bool          present;         /* 当前检测到器件且读数有效 */
    bool          disabled;        /* 连续失败后自动停用，等待 force_recheck */
    bool          conv_pending;    /* 正在等转换完成 */
    bool          manual_valid;    /* 是否设置了手输温度 */
    float         manual_temp_c;   /* 手输温度 */
    float         sensor_temp_c;   /* 最近一次有效实测温度 */
    float         temp_c;          /* 当前采用温度（实测/手输/兜底） */
    ds18b20_src_t src;             /* 温度来源 */
    uint32_t      conv_start_ms;   /* 本次转换发起时刻 */
    uint32_t      last_detect_ms;  /* 上次检测时刻 */
    uint8_t       fail_streak;     /* 连续无效次数 */
    uint32_t      error_count;     /* 累计无效次数（统计用） */
} ds18b20_t;

/* 初始化：保存配置、把引脚配为"输入 + 上拉"。不阻塞。 */
void ds18b20_init(ds18b20_t *p_dev, const ds18b20_cfg_t *p_cfg);

/* 上电自检（阻塞 ≈800 ms）：检测 + 试读一次有效温度。
 * 返回 true=器件可用；false=不可用（自动进入 disabled 状态）。 */
bool ds18b20_boot_test(ds18b20_t *p_dev);

/* 周期任务：主循环每 50 ms 调用一次。负责"发起转换/到时读数/重试/停用"状态机。
 * force_recheck=true 时无条件重新检测一次（命令 C 用）。 */
void ds18b20_task(ds18b20_t *p_dev, uint32_t now_ms, bool force_recheck);

/* 设置手输温度（串口 T 命令）；同时清除 disabled，等待下一轮检测。 */
void ds18b20_set_manual_temp(ds18b20_t *p_dev, float temp_c);

/* 当前采用温度 / 来源 / 是否可用（供 app 打印）。 */
float         ds18b20_get_temp_c(const ds18b20_t *p_dev);
ds18b20_src_t ds18b20_get_src(const ds18b20_t *p_dev);
bool          ds18b20_sensor_ok(const ds18b20_t *p_dev);
const char   *ds18b20_src_str(ds18b20_src_t src);

/* 底层接口（自检/排障/单元测试用；一般不用直接调）。 */
bool ds18b20_ping(bsp_io_port_pin_t pin);
bool ds18b20_request_convert(bsp_io_port_pin_t pin);
bool ds18b20_read_temperature(bsp_io_port_pin_t pin, float *p_temp_c);

#ifdef __cplusplus
}
#endif
#endif /* SENSOR_DS18B20_H */

