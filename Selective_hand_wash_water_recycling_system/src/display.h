/*
 * display.h
 *
 *  Created on: 2026年10月30日
 *      Author: Administrator
 */

/*
 * display.h —— 板载 0.96" OLED 状态页（v1.3.0：三页，K2 短按切页）
 * ---------------------------------------------------------------------------
 * 定位（与 oled.h 一致）：
 *   - 这是"现场指示器"，不是安全执行路径：屏幕坏了/不刷新，判别、串口 CSV、
 *     LED/阀都不受影响；app_main 只在连续失败时打印一次 WARN；
 *   - 只画"本工程定义的量"：不画 NTU 读数、不画绝对 ppm、不说"识别出污染物"；
 *   - 不用动态内存、不用变长数组、不用浮点 printf；最多 1 KB 静态显存。
 *
 * v1.3.0 三页（默认停在 P1；K2 短按循环切换 P1→P2→P3→P1，长按仍重采基线）：
 *
 *   P1 水质页（沿用 v1.1 布局 + 右上角页码 1/3）
 *     y=0   (6x8) : "EC0=1490 s1.5 T25.3"
 *     y=8   (32x32): 大字两个：清水 / 污水 / 待机 / 冲洗 / 故障（反显=污水）
 *     y=40  (8x16): "CLEAR  RECOVER" 等状态串
 *     y=56  (6x8) : "dEC+12mV g1.008 N+5" 数值串
 *
 *   P2 执行器页（v1.3.0 新增，大字显示阀的实际状态）
 *     y=0   (6x8) : "P2/3 EXEC  MODE:AUTO/MANUAL/FAULT"
 *     y=8   (32x32): 大字两个：回收（反显）/ 排空 / 手动（反显）/ 故障 / 待机
 *     y=40  (8x16): "RECOVER  RLY:ON" / "DRAIN    RLY:OFF"（继电器实际状态）
 *     y=56  (6x8) : "AUTO CNT:12 ON:5.2s"（模式/吸合次数/本次通电时长）
 *                   手动时 "V1 CNT:12 ON:5.2s" / "V0 ..."
 *
 *   P3 详情页（v1.3.0 新增，8 行 6x8，每行最多 21 字符）
 *     "P3/3 INFO EV:ON 4.2s"       事件状态 + 已持续时间
 *     "STATE:CLEAR LEVEL:L0"       判别状态/等级
 *     "VALVE:RECOVER RLY:ON"       阀逻辑位置/继电器实际状态
 *     "MODE:AUTO CNT:12"           自动/手动 + 吸合次数
 *     "dEC:+12mV g:1.008"          ΔEC / g
 *     "dNTU:+5.0% T:25.3C"         ΔNTU% / 水温
 *     "EC0:1490mV s:1.5"           基线 / 噪声 σ
 *     "d1:8/8% d2:20/25%"          当前生效的 d1/d2（EC mV / 浊度 %）
 *
 * 刷新模型（实现见 display.c）：
 *   - 页面内容变化立即重画对应区域；数值（P1 页头/P1 数值行/P3 数值行）最快 200 ms；
 *   - 1 s 心跳整屏重画（掉帧/残影自恢复，也是"屏还活着"的证据）；
 *   - 只刷脏页（oled_flush 内部实现），一页约 3 ms@400 kHz；
 *   - 切页（display_page_next）后下一 tick 整屏重画一次。
 */
#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include "sensor_pipeline.h"
#include "decision.h"
#include "actuator_valve.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 三页轮换（K2 短按切换；默认 P1） */
#define DISPLAY_PAGE_COUNT       3U
#define DISPLAY_PAGE_WATER       0U   /* P1 水质页 */
#define DISPLAY_PAGE_EXEC        1U   /* P2 执行器页 */
#define DISPLAY_PAGE_INFO        2U   /* P3 详情页 */

/* 基线采集进度分母（与 app_main.c 的 pipeline_cfg_t.baseline_n 默认值 40 一致）。
 * 如果以后改基线点数，这个常量要同步改；否则进度页会显示 "BASELINE n/40" 不匹配。 */
#define DISPLAY_BASELINE_TOTAL   40U

/* 数值行刷新周期（ms）：状态变化立即刷新，数值最多 5 Hz。 */
#define DISPLAY_NUM_PERIOD_MS    200U

/* 整屏心跳周期（ms）：即使内容没变也重画一次（屏掉帧/残影自恢复）。 */
#define DISPLAY_HEARTBEAT_MS     1000U

/* 开机页保持时间（ms）：上电先让用户看到 "OLED OK"，再切到状态页。 */
#define DISPLAY_BOOT_HOLD_MS     800U

/*
 * 函数名：display_init
 * 作用：初始化 OLED（I2C0 + SSD1306）并画一屏开机页，证明整条显示链路可用。
 * 参数：无。
 * 返回：true=屏幕可用（后续 display_task 可调用）；false=不可用（app 只打印一次 WARN）。
 * 时序/边界：约 130~150 ms（oled_init 内部含上电延时与清屏）；必须在 UART 初始化之后调用。
 * 注意：返回 false 不是致命错误——本工程的安全路径不依赖显示。
 */
bool display_init(void);

/*
 * 函数名：display_task
 * 作用：把当前"流水线输出 + 判别输出 + 执行器输出"翻译成当前页并刷新
 *       （内容有变化/到节流周期才刷）。
 * 参数：now_ms —— app 的 1 ms 时基（用于节流与心跳；无符号回绕安全）；
 *       p_out  —— 流水线输出（EC0、σ、温度、ΔEC、g、ΔNTU%、note 等）；
 *       p_dec  —— 判别器输出（state/level/action，决定大字与状态串）；
 *       p_valve—— 执行器输出（阀逻辑位置/继电器实际状态/计数/通电时长；可为 NULL）。
 * 返回：true=正常（含"本 tick 无需刷新"）；false=连续 3 帧 I2C 刷新失败或屏幕已判丢失
 *       （app 只打印一次 WARN，之后继续用串口，不影响安全侧）。
 * 时序/边界：在 app_tick() 里每 50 ms 调一次；单次最坏约 25 ms（整屏心跳）。
 * 注意：p_out / p_dec 为 NULL 时直接返回 true（防御，不画屏也不报错）；
 *       p_valve 为 NULL 时 P2/P3 显示 "NO VALVE"（旧固件兼容）。
 */
bool display_task(uint32_t now_ms, const pipeline_out_t *p_out, const decision_out_t *p_dec,
                  const valve_out_t *p_valve);

/*
 * 函数名：display_page_next
 * 作用：K2 短按调用：切到下一页（P1→P2→P3→P1），下一 tick 整屏重画。
 * 参数：无。
 * 返回：切换后的页号（1~3，供串口日志）；屏幕不可用时返回 0（不改变内部页号）。
 * 时序/边界：O(1)，可在主循环上下文调用；不产生 I2C 传输。
 * 注意：显示不是安全路径，切页失败/屏坏不影响判别与阀。
 */
uint8_t display_page_next(void);

/*
 * 函数名：display_page_get
 * 作用：读取当前页号（1~3；屏幕不可用时仍返回 1~3，仅用于日志/调试）。
 * 参数：无。
 * 返回：当前页号 1..3。
 * 时序/边界：O(1)。
 * 注意：仅用于日志/调试。
 */
uint8_t display_page_get(void);

/*
 * 函数名：display_ok
 * 作用：查询显示是否可用（oled_init 成功且未连续失败到"丢失"阈值）。
 * 参数：无。
 * 返回：true=可用；false=初始化失败或已判丢失。
 * 时序/边界：O(1)，可在 app 里用于一次性的 WARN 判断。
 * 注意：显示不可用不代表系统故障，只是"少了一路现场指示"。
 */
bool display_ok(void);

#ifdef __cplusplus
}
#endif
#endif /* DISPLAY_H */
