/*
 * actuator_valve.h
 *
 *  Created on: 2026年11月13日
 *      Author: Administrator
 */

/*
 * actuator_valve.h —— 二位三通电磁阀执行器（P104 → SRD-05VDC-SL-C 单路继电器模块 → 12V 阀）
 * ---------------------------------------------------------------------------
 * 本模块把"逻辑位置"翻译成"继电器吸合/释放"，只做三件事：
 *   ① fail-safe 初始化：在上电最早时刻把 P104 写成"继电器释放"电平（= 排空位）；
 *   ② 带驻留/超时保护的换向：最小换向间隔 min_dwell_ms 防继电器抖动；单次连续
 *      吸合不超过 max_on_ms（阀线圈连续工作制限制），超时强制释放并冷却；
 *   ③ 引脚写/回读自检 + 动作计数，供上层打印 "# VALVE" 事件行与 P 状态。
 *
 * 逻辑位置（与 decision_action_t 一一对应）：
 *   VALVE_POS_DRAIN   = 排空位：继电器释放，阀线圈断电（弹簧复位到排空口，fail-safe）；
 *   VALVE_POS_RECOVER = 回收位：继电器吸合，12V 加到阀线圈（换向到回收口）。
 *
 * 有效电平（关键，必须与实物模块一致）：
 *   高电平触发模块：relay_on_level = BSP_IO_LEVEL_HIGH（回收=HIGH，默认）；
 *   低电平触发模块：relay_on_level = BSP_IO_LEVEL_LOW （回收=LOW）。
 *   硬件配套：高触发 → P104 与 GND 之间 10 kΩ 下拉；低触发 → P104 与 3.3 V 之间 10 kΩ 上拉。
 *   极性判错时上电瞬间继电器会误吸合，绝对不允许接 12V 阀电源后再试（见 15 文档 §4）。
 *
 * 安全约定：
 *   - 初始化失败/引脚写失败 → 不假装成功：initialized=false 或 pin_fault=true，逻辑位置保持排空；
 *   - 本模块不 printf、不 delay、不用动态内存，可在 50 ms tick 里调用；
 *   - 真正的最后一道安全是 12V 进线上的船型开关（现场可断电），软件不能替代。
 */
#ifndef ACTUATOR_VALVE_H
#define ACTUATOR_VALVE_H

#include <stdbool.h>
#include <stdint.h>

#include "hal_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 逻辑位置：DRAIN=排空（继电器释放）/ RECOVER=回收（继电器吸合） */
typedef enum e_valve_pos
{
    VALVE_POS_DRAIN = 0,
    VALVE_POS_RECOVER = 1
} valve_pos_t;

/* 配置（app_main.c 里集中给出；字段含义见文件头） */
typedef struct st_valve_cfg
{
    bsp_io_port_pin_t pin;             /* 阀控制脚：P104 = BSP_IO_PORT_01_PIN_04 */
    bool              enabled;         /* false=假执行器：配置好 fail-safe 电平但不换向 */
    bsp_io_level_t    relay_on_level;  /* 回收位（继电器吸合）时的 P104 电平 */
    uint32_t          min_dwell_ms;    /* 两次物理换向的最小间隔（默认 500 ms） */
    uint32_t          max_on_ms;       /* 单次连续吸合上限（默认 120000 ms；0=不限制） */
    uint32_t          cooldown_ms;     /* 超时强制释放后的冷却时间（默认 10000 ms） */
} valve_cfg_t;

/* 运行输出（只读；valve_get_out 返回其指针） */
typedef struct st_valve_out
{
    valve_pos_t  requested;        /* 上层本 tick 请求的逻辑位置 */
    valve_pos_t  position;         /* 实际已执行的逻辑位置（GPIO 已按此驱动） */
    const char  *request_reason;   /* 最近一次请求来源（静态串：AUTO / MANUAL / SAFETY） */
    const char  *switch_reason;    /* 最近一次实际换向原因（静态串） */
    bool         relay_on;         /* 继电器线圈是否吸合 */
    bool         initialized;      /* 引脚已配置成输出且回读正确 */
    bool         pin_fault;        /* 引脚写/回读失败（锁存，需复位；安全侧仍排空） */
    bool         max_on_blocked;   /* 连续吸合超时触发强制排空，冷却中 */
    uint32_t     switch_count;     /* 累计吸合次数（继电器动作计数，报告用） */
    uint32_t     on_time_ms;       /* 本次连续吸合已持续时长（释放时清 0） */
    uint32_t     total_on_ms;      /* 累计吸合时长 */
    uint32_t     fault_count;      /* 引脚写/回读失败累计次数 */
} valve_out_t;

/* 句柄（app_main 静态分配） */
typedef struct st_valve
{
    valve_cfg_t cfg;
    valve_out_t out;
    uint32_t    last_switch_ms;    /* 上次物理换向时刻 */
    uint32_t    on_since_ms;       /* 本次吸合开始时刻 */
    uint32_t    blocked_until_ms;  /* 冷却结束时刻（max_on_blocked 时有效） */
} valve_t;

/*
 * 函数名：valve_init
 * 作用：保存配置并做 fail-safe 初始化：把 P104 配成输出、初值="继电器释放"电平；
 *       随后回读校验。enabled=false 时同样配置该电平（保持"上电即排空"的硬件约定），
 *       但后续 valve_task 不再换向。
 * 参数：p_valve —— 句柄（非 NULL）；p_cfg —— 配置（非 NULL）。
 * 返回：无（结果看 valve_get_out()->initialized / pin_fault）。
 * 时序/边界：必须是 app_main 的第一条外设操作（早于 LED/UART/ADC），约几十 µs。
 * 注意：配置写是"PODR 与方向同一寄存器写"，不会先出高再拉低，上电无毛刺。
 */
void valve_init(valve_t *p_valve, const valve_cfg_t *p_cfg);

/*
 * 函数名：valve_request
 * 作用：上层声明本 tick 想要的逻辑位置（判别结果或手动覆盖）。
 * 参数：p_valve —— 句柄；pos —— 目标位置（越界按 DRAIN 处理）；
 *       now_ms —— 当前时刻（本函数不用，保留给未来扩展）；p_reason —— 来源静态串（可为 NULL）。
 * 返回：无。
 * 时序/边界：只写 requested，不碰 GPIO；真正换向由 valve_task 在满足驻留条件后执行。
 * 注意：安全侧调用方应在拿不到判别结果时请求 DRAIN。
 */
void valve_request(valve_t *p_valve, valve_pos_t pos, uint32_t now_ms, const char *p_reason);

/*
 * 函数名：valve_task
 * 作用：执行器状态机（每 50 ms tick 调一次）：驻留判断 → 写 P104 → 回读校验 →
 *       吸合计数/通电计时 → 连续吸合超时强制释放 → 冷却解除。
 * 参数：p_valve —— 句柄；now_ms —— 当前时刻（1 ms 时基，回绕安全）。
 * 返回：无。
 * 时序/边界：单次调用只做几个寄存器读写，微秒级；绝不 printf/delay。
 * 注意：写失败时 position 不更新、下个 tick 重试，pin_fault 锁存；初始化失败时直接返回，
 *       逻辑位置保持排空（硬件下拉/上拉保证继电器释放）。
 */
void valve_task(valve_t *p_valve, uint32_t now_ms);

/*
 * 函数名：valve_get_out
 * 作用：读取执行器输出（只读指针，生命周期与句柄相同）。
 * 参数：p_valve —— 句柄（可为 NULL）。
 * 返回：输出指针；p_valve 为 NULL 时返回 NULL。
 * 时序/边界：O(1)；可供 app 打印事件行/状态页。
 * 注意：不要在中断里长时间持有该指针做复杂计算（本工程只在主循环用）。
 */
const valve_out_t *valve_get_out(const valve_t *p_valve);

/*
 * 函数名：valve_pos_str
 * 作用：逻辑位置 → 静态字符串（"DRAIN"/"RECOVER"），供日志/状态行。
 * 参数：pos —— 位置。
 * 返回：静态字符串（未知按 "DRAIN"）。
 * 时序/边界：纯函数。
 * 注意：只用于显示；控制逻辑一律用枚举比较。
 */
const char *valve_pos_str(valve_pos_t pos);

#ifdef __cplusplus
}
#endif
#endif /* ACTUATOR_VALVE_H */
