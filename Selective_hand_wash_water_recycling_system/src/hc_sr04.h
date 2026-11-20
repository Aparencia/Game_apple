/*
 * hc_sr04.h
 *
 * HC-SR04 标准四针，液位（空气路径）驱动。
 *   TRIG = P402 (U1-2)  3.3V 输出，外部电平转换到 5V
 *   ECHO = P409 (U2-3)  5V 容忍，IRQ6，建议串 1k
 *
 * 说明：
 *   - 不使用 GPT；Echo 双边沿 IRQ + DWT CYCCNT 测脉宽（约 5.5 ns 分辨率）。
 *   - 非阻塞：hc_sr04_task() 每 50 ms 调一次，内部 100 ms 触发一轮。
 */
#ifndef HC_SR04_H
#define HC_SR04_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 GPIO/IRQ6/DWT；返回 false 表示引脚或中断配置失败。 */
bool hc_sr04_init(void);

/* 主循环调用，非阻塞；now_ms 用 timebase_ms()。 */
void hc_sr04_task(uint32_t now_ms);

/* 读取最近一次中值；valid=false 表示无回波/超时/尚未采样。 */
bool hc_sr04_get_mm(uint16_t *p_mm, bool *p_valid);

/* 诊断计数：触发次数/边沿/上升/下降/超时/最近脉宽。 */
void hc_sr04_get_diag(uint32_t *p_trig, uint32_t *p_edges, uint32_t *p_rise,
                      uint32_t *p_fall, uint32_t *p_timeout, uint32_t *p_last_us);

/* 当前用于换算的 CPU cycles/us（用于日志核对）。 */
uint32_t hc_sr04_get_cycles_per_us(void);

/* Echo 双边沿中断处理（vector_data.c 使用）。 */
void hc_sr04_echo_isr(void);

#ifdef __cplusplus
}
#endif
#endif /* HC_SR04_H */
