/*
 * oled.h —— 板载 0.96" OLED（SSD1306 兼容，128x64，I2C0）驱动接口
 * ---------------------------------------------------------------------------
 * 硬件事实（板卡原理图 V1.2.1 / 硬件方案 §1.2）：
 *   - 板载 0.96" OLED 直接连在 I2C0：P407 = IIC0_SDA、P408 = IIC0_SCL；
 *     模块自带 4.7kΩ 上拉，地址 0x3C（7-bit），无需任何外部接线；
 *   - 3.3V 供电，与 MCU 同开同关；禁止拆焊（开发板赛后收回）。
 *
 * 本驱动的定位：
 *   - 只做"现场指示器"：把判别结果画成大字（清水/污水/待机/冲洗/故障）+ 两行数值；
 *   - 不是安全执行路径：I2C 失败、屏幕丢失都不能影响判别/串口 CSV/LED/阀；
 *   - 不打印、不依赖浮点 printf、不用动态内存；显存 1 KB 静态数组，按页刷屏。
 *
 * 刷新模型（与 oled.c 实现一致）：
 *   - 1 KB 显存按 SSD1306 页式组织：gram[page][x]，bit0 = 页内最上一行；
 *   - 所有绘图函数只改显存并置脏页位；oled_flush() 只把脏页写到屏上；
 *   - 文本/汉字必须 8 像素对齐（y % 8 == 0），否则字符会跨页错位；
 *   - 一页 = 2 次 I2C 传输（3 条地址命令 + 1 控制字节 + 128 字节数据），
 *     400 kHz 下约 3 ms；全屏 8 页约 25 ms，所以上层必须节流（见 display.c）。
 *
 * 回调约定（RASC 里配错名字会链接失败，见 00 文档 §4.3）：
 *   - I2C Master 的 Callback 必须填 i2c_master_callback；
 *   - 该函数由本文件提供：只记录 TX_COMPLETE/ABORTED 事件，不打印、不做业务。
 */
#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_WIDTH          128U   /* 像素宽 */
#define OLED_HEIGHT         64U    /* 像素高（8 页） */
#define OLED_PAGE_COUNT     8U     /* 64 / 8 */
#define OLED_I2C_ADDR       0x3CU  /* SSD1306 7-bit 地址 */
#define OLED_MAX_FAIL       3U     /* 连续失败帧数达到后判"屏幕丢失" */

/*
 * 面板安装方向（SSD1306 硬件级重映射，不改显存坐标/字库/布局）：
 *   0 = 正常        列 0xA0 + 行 0xC8（本板原方向）
 *   1 = 左右镜像    列 0xA1 + 行 0xC8
 *   2 = 上下翻转    列 0xA0 + 行 0xC0
 *   3 = 180° 反转   列 0xA1 + 行 0xC0（面板倒装时用）
 *
 * 说明：本板 2026-11-01 实测 0xA1 单独使用会左右镜像，因此正常方向取 0xA0；
 * 若整块屏倒装（现场“反转”），必须同时翻列和翻行，即 0xA1 + 0xC0。
 * 只改这一个宏即可，oled.c/display.c 不用动。
 */
#define OLED_ORIENTATION     3

#if   (0 == OLED_ORIENTATION)
#define OLED_SEG_REMAP_CMD   0xA0U
#define OLED_COM_SCAN_CMD    0xC8U
#elif (1 == OLED_ORIENTATION)
#define OLED_SEG_REMAP_CMD   0xA1U
#define OLED_COM_SCAN_CMD    0xC8U
#elif (2 == OLED_ORIENTATION)
#define OLED_SEG_REMAP_CMD   0xA0U
#define OLED_COM_SCAN_CMD    0xC0U
#else
#define OLED_SEG_REMAP_CMD   0xA1U
#define OLED_COM_SCAN_CMD    0xC0U
#endif

/* 32x32 汉字索引：与 oledfont.h 的 oled_hz32_bitmaps[] 顺序一一对应（改顺序必须两边一起改）。
 * 字库共 17 个字：清水污待机冲洗故障 + v1.3.0 追加 回收排空手动 + v2.4.2 追加 校准；
 * 不要指望显示任意中文——字库只有这 17 个字形（新增字必须改 oledfont.h 并同步本枚举）。 */
typedef enum e_oled_hz32_id
{
    OLED_HZ32_QING = 0,   /* 清 */
    OLED_HZ32_SHUI,       /* 水 */
    OLED_HZ32_WU,         /* 污 */
    OLED_HZ32_DAI,        /* 待 */
    OLED_HZ32_JI,         /* 机 */
    OLED_HZ32_CHONG,      /* 冲 */
    OLED_HZ32_XI,         /* 洗 */
    OLED_HZ32_GU,         /* 故 */
    OLED_HZ32_ZHANG,      /* 障 */
    OLED_HZ32_HUI,        /* 回（v1.3.0 执行器页） */
    OLED_HZ32_SHOU,       /* 收 */
    OLED_HZ32_PAI,        /* 排 */
    OLED_HZ32_KONG,       /* 空 */
    OLED_HZ32_HAND,       /* 手 */
    OLED_HZ32_DONG,       /* 动 */
    OLED_HZ32_JIAO,       /* 校（v2.4.2 湿态基线页） */
    OLED_HZ32_ZHUN,       /* 准（v2.4.2 湿态基线页） */
    OLED_HZ32_COUNT
} oled_hz32_id_t;

/* 打开 I2C0、初始化 SSD1306、清屏（全黑）、点亮显示。
 * 时序/边界：上电调用一次，内部含 100 ms 上电延时 + 约 25 ms 清屏，总耗时约 130 ms； 必须在 UART 初始化之后调用（先保证串口能报告失败）。
 * 注意：失败不是致命错误：上层打印 WARN 后继续用串口，判别/阀不受影响。 */
bool oled_init(void);

/* 当前驱动是否可用（初始化成功且未连续失败到"丢失"阈值）。 */
bool oled_ready(void);

/* 清空显存（全黑）并把 8 页全部标脏（下次 oled_flush 全屏刷新）。 */
void oled_clear(void);

/* 显存填充矩形（不立即写屏）。 时序/边界：O(页数*宽) 次字节操作，越界部分被裁掉；用于清空文本区域，避免旧内容残留。 注意：只改显存 + 脏页位，不产生 I2C 传输。 */
void oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on);

/* 在显存写 6x8 ASCII 字符串（小号字，21 字符/行）。 参数：x/y 起点（y 必须 8 对齐）；p_text 以 '\0' 结尾（NULL 视为空串）。
 * 时序/边界：每字符 6x6 像素、字宽 6；x 越界自动停止；非 ASCII(<32 或 >126) 按空格画。
 * 注意：直接覆写该页对应 6 列的背景，不会清除行尾更右侧的旧内容——需要清整行时 上层先 oled_fill_rect。 */
void oled_text_6x8(uint8_t x, uint8_t y, const char *p_text);

/* 在显存写 8x16 ASCII 字符串（大号字，16 字符/行）。 参数：x/y 起点（y 必须 8 对齐）；p_text 以 '\0' 结尾（NULL 视为空串）。
 * 时序/边界：每字符占 8x16（两页），x 越界自动停止。 注意：字模存储为"上 8 行 + 下 8 行"（见 oledfont.h），本函数按页拆分写入。 */
void oled_text_8x16(uint8_t x, uint8_t y, const char *p_text);

/* 在显存写一个 32x32 汉字（x 必须 8 对齐、y 必须 8 对齐；占 4 页）。 时序/边界：占 32 列 x 4 页；整块覆写（含背景），所以切换正/反显不需要先清屏。
 * 注意：反显时整块背景涂白，适合"污水"这类需要抓眼球的提示。 */
void oled_hz32(uint8_t x, uint8_t y, oled_hz32_id_t id, bool inverse);

/* 把脏页写到屏幕（每页 2 次 I2C 传输：地址命令 + 数据块）。 时序/边界：单页约 3 ms（400 kHz），全屏约 25 ms；在 50 ms 主循环里调用必须节流。
 * 注意：连续 OLED_MAX_FAIL 帧失败 → 判屏幕丢失，之后所有刷新直接返回 false； 此时上层只需打印一次 WARN，继续用串口（显示不是安全路径）。 */
bool oled_flush(void);

#ifdef __cplusplus
}
#endif
#endif /* OLED_H */
