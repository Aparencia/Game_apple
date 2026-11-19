/*
 * app_main.h
 *
 *  Created on: 2026年10月30日
 *      Author: Administrator
 */

/*
 * app_main.h —— 应用入口（由 hal_entry 调用，不返回）
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 传感器应用主入口：初始化外设/传感器/判别器，进入 50 ms 主循环。 */
void app_main(void);

/*
 * 开机自检状态文本（给 OLED / 状态命令用）：
 *   把当前自检阶段写进 p_buf（如 "SELFCHK DRY 37/200" / "SELFCHK WAIT WATER"）；
 *   不在自检时写入空串。p_buf 容量建议 ≥24 B。
 */
void app_autocal_text(char *p_buf, uint32_t buf_size);

/* 是否正在开机自检（true=正在自动干态/等水/自动湿态）。 */
bool app_autocal_active(void);

#ifdef __cplusplus
}
#endif
#endif /* APP_MAIN_H */
