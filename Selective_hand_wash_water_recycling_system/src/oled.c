


/*
 * oled.c —— 板载 0.96" OLED（SSD1306，128x64，I2C0）驱动实现
 * ---------------------------------------------------------------------------
 * 本文件只做三件事：
 *   1) I2C0 + SSD1306 初始化（Open → 上电延时 → 初始化命令序列 → 清屏）；
 *   2) 1 KB 页式显存 + 画点/矩形/6x8 ASCII/8x16 ASCII/32x32 汉字（全部只改显存）；
 *   3) 脏页刷新：只把改过的页写屏，一页 = 地址命令 + 128 字节数据两次传输。
 *
 * 为什么按页刷而不是每次整屏刷：
 *   - 50 ms 主循环里，全屏 8 页约 25 ms 的 I2C 阻塞会明显挤占采样与判别；
 *   - 现场最常见的更新是第三行数值（ΔEC/g/ΔNTU），只脏第 7 页，约 3 ms；
 *     状态/大字变化时才刷第 0~6 页，且由 display.c 节流（状态变化立即、数值 200 ms）。
 *
 * 为什么用 FSP 的 IIC Master（而不是 GPIO 模拟 I2C）：
 *   - 板载 OLED 本来就接在 I2C0（P407/P408），RASC 里加一个 IIC Master 栈即可；
 *   - FSP 驱动处理起始/停止/ACK/超时，代码量小；轮询等待只等"本帧完成"事件，
 *     有硬上限（每页 30 ms），不会死等——屏幕坏线也只损失这一帧显示。
 *
 * 错误处理原则（显示不是安全路径）：
 *   - 任何一次 I2C 失败只返回 false，不打印、不死机、不 assert、不 __BKPT；
 *   - 连续 OLED_MAX_FAIL 帧失败 → 判"屏幕丢失"（oled_lost()=true），以后刷新直接返回；
 *   - 上层（display.c/app_main.c）负责打印一次 "# WARN: OLED lost ..."。
 *
 * 依赖：RASC 配置 I2C Master（r_iic_master / IIC0）：
 *   Name=g_i2c_master0、Channel=0、Fast-mode 400 kHz、Slave=0x3C、7-bit、
 *   Timeout=short、Callback=i2c_master_callback（本文件提供该回调）。
 */
#include "oled.h"

#include <stddef.h>
#include <string.h>

#include "hal_data.h"

/* 点阵字库（6x8 / 8x16 ASCII + 9 个 32x32 汉字），只允许本文件 include。
 * 字库用 static 数组放在头文件里：只有本编译单元生成一份，不占 Flash 之外的空间。 */
#include "oledfont.h"

/* ============================ 内部常量 ============================ */

/* SSD1306 128x64 初始化命令序列（每条单独一次 2 字节传输：控制字 0x00 + 命令）。
 * 各命令含义：
 *   0xAE 关显示；0xD5/0x80 时钟分频；0xA8/0x3F 复用率 1/64；0xD3/0x00 显示偏移 0；
 *   0x40 起始行 0；0x8D/0x14 开电荷泵；0x20/0x00 页寻址模式；
 *   列/行重映射由 oled.h 的 OLED_SEG_REMAP_CMD / OLED_COM_SCAN_CMD 决定：
 *        正常=0xA0+0xC8；180° 反转=0xA1+0xC0（面板倒装时用）；
 *   0xDA/0x12 COM 引脚配置；0x81/0xCF 对比度；
 *   0xD9/0xF1 预充电周期；0xDB/0x40 VCOMH；0xA4 显存内容显示；0xA6 正显；0xAF 开显示。
 * 说明：顺序来自 SSD1306 数据手册的典型初始化流程；除列重映射按实测面板取 0xA0 外，
 *       其余参数与常见 0.96" 模块一致。换屏后若左右反了，只改 OLED_SEG_REMAP_CMD。 */
static const uint8_t g_oled_init_cmds[] =
{
    0xAEU,
    0xD5U, 0x80U,
    0xA8U, 0x3FU,
    0xD3U, 0x00U,
    0x40U,
    0x8DU, 0x14U,
    0x20U, 0x00U,
    OLED_SEG_REMAP_CMD,
    OLED_COM_SCAN_CMD,
    0xDAU, 0x12U,
    0x81U, 0xCFU,
    0xD9U, 0xF1U,
    0xDBU, 0x40U,
    0xA4U,
    0xA6U,
    0xAFU
};

#define OLED_CTRL_CMD       0x00U   /* 控制字：后续字节是命令 */
#define OLED_CTRL_DATA      0x40U   /* 控制字：后续字节是显存数据 */
#define OLED_CMD_BUF_SIZE   4U      /* 一页的地址命令：0x00 + B0/p + 00 + 10 */
#define OLED_DATA_BUF_SIZE  (1U + OLED_WIDTH)  /* 0x40 + 128 字节数据 */
#define OLED_WAIT_LOOPS     300U    /* 单次传输等待上限：300 * 100 µs = 30 ms */
#define OLED_WAIT_STEP_US   100U    /* 等待步长（µs） */

/* ============================ 内部状态 ============================ */

/* 1 KB 显存：gram[page][x]，bit0 = 该页最上一行（SSD1306 页式格式）。 */
static uint8_t g_oled_gram[OLED_PAGE_COUNT][OLED_WIDTH];

/* 脏页位图：bit p = 第 p 页需要刷新。绘图函数只置位，oled_flush 成功后清零。 */
static uint8_t g_oled_dirty;

/* I2C 传输完成标志：由 i2c_master_callback（中断上下文）写，主循环读。
 * 注意 volatile：主循环必须每次从内存重新读取，不能缓存在寄存器里。 */
static volatile bool              g_oled_tx_done;
static volatile i2c_master_event_t g_oled_tx_event;

/* 驱动状态 */
static bool    g_oled_ready;        /* 初始化成功且未判丢失 */
static bool    g_oled_lost;         /* 连续失败达到阈值，判屏幕丢失 */
static uint8_t g_oled_fail_count;   /* 连续失败帧数 */

/* 传输缓冲：静态分配（不用动态内存；也避免每帧把 133 字节压到主循环栈上）。 */
static uint8_t g_oled_cmd_buf[OLED_CMD_BUF_SIZE];
static uint8_t g_oled_data_buf[OLED_DATA_BUF_SIZE];

/* ============================ I2C 底层 ============================ */

/* FSP IIC Master 完成/异常回调（RASC 里 Callback 必须配成这个名字）。 时序/边界：中断上下文调用；只记录事件并置完成标志，绝不在中断里打印/延时。
 * 注意：本回调是"整帧"级别的；oled_flush 用 g_oled_tx_done 轮询等待。 */
void i2c_master_callback(i2c_master_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        g_oled_tx_event = p_args->event;
    }
    else
    {
        g_oled_tx_event = I2C_MASTER_EVENT_ABORTED;
    }
    g_oled_tx_done = true;
}

/* 发起一次 I2C 写并等待整帧完成（有硬超时，不死等）。 时序/边界：单帧 133 字节 @400 kHz 约 3 ms；等待上限 30 ms。
 * 注意：失败后不清 g_oled_ready（上层按连续失败帧数判丢失），也不重试—— 重试会拉长主循环阻塞时间，交给下一帧自然重试（脏页位保留）。 */
static bool oled_i2c_write(const uint8_t *p_buf, uint32_t len)
{
    fsp_err_t err;
    uint32_t  i;

    if ((NULL == p_buf) || (0U == len))
    {
        return false;
    }

    /* 清完成标志必须在发起传输之前：否则可能把上一帧的事件当成本帧的。 */
    g_oled_tx_done  = false;
    g_oled_tx_event = I2C_MASTER_EVENT_ABORTED;

    /* FSP 6.6 的 R_IIC_MASTER_Write 形参是 uint8_t*（非 const），这里强转；
     * 驱动只读缓冲区，调用期间 p_buf 有效（静态缓冲或调用方栈上数组）。 */
    err = R_IIC_MASTER_Write(&g_i2c_master0_ctrl, (uint8_t *)p_buf, len, false);
    if (FSP_SUCCESS != err)
    {
        return false;
    }

    for (i = 0U; i < OLED_WAIT_LOOPS; i++)
    {
        if (g_oled_tx_done)
        {
            break;
        }
        R_BSP_SoftwareDelay(OLED_WAIT_STEP_US, BSP_DELAY_UNITS_MICROSECONDS);
    }

    if (!g_oled_tx_done)
    {
        /* 超时：主动 Abort，避免下一帧发不出去（Abort 失败也不管，下一帧会自然暴露）。 */
        (void)R_IIC_MASTER_Abort(&g_i2c_master0_ctrl);
        return false;
    }

    return (I2C_MASTER_EVENT_TX_COMPLETE == g_oled_tx_event);
}

/* 发一条 SSD1306 命令（控制字 0x00 + 1 字节命令）。 时序/边界：约 0.1 ms/条；初始化序列共 25 条约 3 ms。
 * 注意：只用于初始化与页地址设置；显存数据用 oled_write_page。 */
static bool oled_write_cmd(uint8_t cmd)
{
    uint8_t buf[2];

    buf[0] = OLED_CTRL_CMD;
    buf[1] = cmd;
    return oled_i2c_write(buf, 2U);
}

/* 把显存第 page 页写到屏幕。 时序/边界：两次传输（4 字节命令 + 129 字节数据），约 3 ms。 注意：页地址命令必须连续设置 B0/p、低列 0x00、高列 0x10，否则列指针会错位。 */
static bool oled_flush_page(uint8_t page)
{
    if (page >= OLED_PAGE_COUNT)
    {
        return false;
    }

    /* 1) 地址命令：页地址 + 列地址 0（低 4 位/高 4 位）。 */
    g_oled_cmd_buf[0] = OLED_CTRL_CMD;
    g_oled_cmd_buf[1] = (uint8_t)(0xB0U | page);   /* 页地址 */
    g_oled_cmd_buf[2] = 0x00U;                     /* 低列地址 0 */
    g_oled_cmd_buf[3] = 0x10U;                     /* 高列地址 0 */
    if (!oled_i2c_write(g_oled_cmd_buf, OLED_CMD_BUF_SIZE))
    {
        return false;
    }

    /* 2) 数据块：0x40 + 128 字节该页显存。 */
    g_oled_data_buf[0] = OLED_CTRL_DATA;
    (void)memcpy(&g_oled_data_buf[1], g_oled_gram[page], OLED_WIDTH);
    return oled_i2c_write(g_oled_data_buf, OLED_DATA_BUF_SIZE);
}

/* ============================ 对外初始化 ============================ */

/* 打开 I2C0、上电延时、初始化 SSD1306、清屏并点亮。 时序/边界：约 130 ms（100 ms 上电延时 + 初始化命令 + 清屏 8 页）； 必须在 UART 初始化之后调用，保证失败信息打得出。
 * 注意：上电延时 100 ms 是必要的——OLED 的电荷泵/复位需要稳定时间， 太早发初始化命令会出现"花屏/半亮"的经典现象。 */
bool oled_init(void)
{
    fsp_err_t err;
    uint32_t  i;

    g_oled_ready      = false;
    g_oled_lost       = false;
    g_oled_fail_count = 0U;
    g_oled_dirty      = 0U;
    (void)memset(g_oled_gram, 0, sizeof(g_oled_gram));

    /* 1) 打开 IIC0。回调/中断优先级来自 RASC 生成的 g_i2c_master0_cfg。 */
    err = R_IIC_MASTER_Open(&g_i2c_master0_ctrl, &g_i2c_master0_cfg);
    if (FSP_SUCCESS != err)
    {
        return false;
    }

    /* 2) 显式设一次从机地址（7-bit 0x3C）。FSP 的 Open 会按 cfg 设置，
     *    这里再设一次是防御性动作：以后若在 RASC 里改了 cfg.slave，也不会静默用错地址。 */
    err = R_IIC_MASTER_SlaveAddressSet(&g_i2c_master0_ctrl, OLED_I2C_ADDR,
                                       I2C_MASTER_ADDR_MODE_7BIT);
    if (FSP_SUCCESS != err)
    {
        return false;
    }

    /* 3) 上电稳定延时（见"注意"）。 */
    R_BSP_SoftwareDelay(100U, BSP_DELAY_UNITS_MILLISECONDS);

    /* 4) 初始化命令序列（任何一条失败即判初始化失败）。 */
    for (i = 0U; i < (uint32_t)(sizeof(g_oled_init_cmds) / sizeof(g_oled_init_cmds[0])); i++)
    {
        if (!oled_write_cmd(g_oled_init_cmds[i]))
        {
            return false;
        }
    }

    /* 5) 清屏：显存清零 + 8 页全标脏 → 全屏刷黑，避免上电显存随机内容。 */
    g_oled_ready = true;
    oled_clear();
    if (!oled_flush())
    {
        g_oled_ready = false;
        return false;
    }

    return true;
}

bool oled_ready(void)
{
    return g_oled_ready;
}

/* ============================ 显存绘图 ============================ */

/* 把 (x, y) 所在页标脏。 */
static void oled_mark_page(uint8_t y)
{
    g_oled_dirty |= (uint8_t)(1U << (y >> 3));
}

void oled_clear(void)
{
    (void)memset(g_oled_gram, 0, sizeof(g_oled_gram));
    g_oled_dirty = (uint8_t)((1U << OLED_PAGE_COUNT) - 1U);   /* 8 页全脏 */
}

/* 显存填充矩形：逐页把落在本页的行折成位掩码，再按字节整段置位/清零（无逐点调用）。
 * 参数：x/y/w/h —— 逻辑坐标与尺寸（越界部分裁掉）；on —— true 点亮 / false 熄灭。
 * 时序/边界：O(页数 * 宽) 次字节操作，无 I2C。 注意：只改显存 + 脏页位；裁剪后为空的矩形不置脏页位。 */
void oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on)
{
    uint8_t first_page;
    uint8_t last_page;
    uint8_t page;
    uint8_t mask;
    uint8_t xx;

    if ((0U == w) || (0U == h) || (x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }
    if (((uint16_t)x + (uint16_t)w) > (uint16_t)OLED_WIDTH)
    {
        w = (uint8_t)(OLED_WIDTH - x);
    }
    if (((uint16_t)y + (uint16_t)h) > (uint16_t)OLED_HEIGHT)
    {
        h = (uint8_t)(OLED_HEIGHT - y);
    }

    first_page = (uint8_t)(y >> 3);
    last_page  = (uint8_t)((y + h - 1U) >> 3);
    for (page = first_page; page <= last_page; page++)
    {
        /* 本页覆盖的行位图：首页削掉上方行，末页削掉下方行，中间页整页点亮。 */
        mask = 0xFFU;
        if (page == first_page)
        {
            mask = (uint8_t)(0xFFU << (y & 7U));
        }
        if (page == last_page)
        {
            mask = (uint8_t)(mask & (uint8_t)(0xFFU >> (7U - ((y + h - 1U) & 7U))));
        }
        for (xx = x; xx < (uint8_t)(x + w); xx++)
        {
            if (on)
            {
                g_oled_gram[page][xx] |= mask;
            }
            else
            {
                g_oled_gram[page][xx] &= (uint8_t)(~mask);
            }
        }
        oled_mark_page((uint8_t)(page * 8U));
    }
}

/* 文本渲染公共实现：w = 字宽（6 或 8），pages = 占用页数（1 或 2）。
 * 参数：x/y —— 起点（y 必须 8 像素对齐）；p_text —— '\0' 结尾（NULL 视为空串）。
 * 时序/边界：页越界直接返回；x 放不下就截断（不折行）；非可打印 ASCII 按空格画。
 * 注意：6x8 与 8x16 只差字宽/页数/字模表，故共用一条路径；每字符只置涉及的脏页位。 */
static void oled_text_run(uint8_t x, uint8_t y, uint8_t w, uint8_t pages, const char *p_text)
{
    uint8_t page;
    uint8_t cx;
    uint8_t code;
    uint8_t p;
    uint8_t col;
    const uint8_t *p_table;

    if (NULL == p_text)
    {
        return;
    }

    /* 只支持 8 像素对齐：不做跨页移位，保证实现简单、时序可控（文档已约定）。 */
    page = (uint8_t)(y >> 3);
    if (((uint16_t)page + (uint16_t)pages) > (uint16_t)OLED_PAGE_COUNT)
    {
        return;
    }

    cx = x;
    while ('\0' != *p_text)
    {
        code = (uint8_t)*p_text;
        p_text++;

        if ((code < OLED_FONT_FIRST_ASCII) || (code >= (OLED_FONT_FIRST_ASCII + OLED_FONT_ASCII_COUNT)))
        {
            code = (uint8_t)' ';   /* 非可打印 ASCII 按空格处理，避免读越界 */
        }

        if (((uint16_t)cx + (uint16_t)w) > (uint16_t)OLED_WIDTH)
        {
            break;                 /* 行尾放不下：截断，不折行 */
        }

        /* 字模布局：每页 w 字节（8x16 前 8 字节 = 上半，后 8 字节 = 下半，见 oledfont.h）。 */
        p_table = (1 == (int)pages) ? &oled_asc6x8[code - OLED_FONT_FIRST_ASCII][0]
                                    : &oled_asc8x16[code - OLED_FONT_FIRST_ASCII][0];
        for (p = 0U; p < pages; p++)
        {
            const uint8_t *p_glyph = p_table + ((uint16_t)p * (uint16_t)w);

            for (col = 0U; col < w; col++)
            {
                g_oled_gram[page + p][cx + col] = p_glyph[col];
            }
            oled_mark_page((uint8_t)(y + (uint8_t)(p * 8U)));
        }
        cx = (uint8_t)(cx + w);
    }
}

void oled_text_6x8(uint8_t x, uint8_t y, const char *p_text)
{
    oled_text_run(x, y, 6U, 1U, p_text);
}

void oled_text_8x16(uint8_t x, uint8_t y, const char *p_text)
{
    oled_text_run(x, y, 8U, 2U, p_text);
}

void oled_hz32(uint8_t x, uint8_t y, oled_hz32_id_t id, bool inverse)
{
    uint8_t page;
    uint8_t p;
    uint8_t c;
    uint8_t data;
    const uint8_t *p_glyph;

    if ((id >= OLED_HZ32_COUNT) || (x > (OLED_WIDTH - 32U)))
    {
        return;
    }

    page = (uint8_t)(y >> 3);
    if ((page + 3U) >= OLED_PAGE_COUNT)
    {
        return;
    }

    /* 32x32 字模：page p (0..3) 的第 c 列 = 第 p*8+0..7 行的 8 个像素（bit0=上）。
     * 反显 = 按位取反：整块 32x32 的背景也一起变白（"白底黑字"标签效果）。 */
    p_glyph = oled_hz32_bitmaps[id];
    for (p = 0U; p < 4U; p++)
    {
        for (c = 0U; c < 32U; c++)
        {
            data = p_glyph[(uint16_t)p * 32U + c];
            if (inverse)
            {
                data = (uint8_t)(~data);
            }
            g_oled_gram[page + p][x + c] = data;
        }
        oled_mark_page((uint8_t)(y + p * 8U));
    }
}

/* ============================ 刷屏 ============================ */

bool oled_flush(void)
{
    uint8_t page;

    if (!g_oled_ready || g_oled_lost)
    {
        return false;
    }

    for (page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        if (0U == (g_oled_dirty & (uint8_t)(1U << page)))
        {
            continue;
        }

        if (!oled_flush_page(page))
        {
            /* 先停：I2C 异常时连续重试只会拖长主循环阻塞；脏页位保留，下一帧自然重试。 */
            g_oled_fail_count++;
            if (g_oled_fail_count >= OLED_MAX_FAIL)
            {
                /* 连续失败达到阈值：判"屏幕丢失"，以后刷新直接返回 false，不再占用主循环。 */
                g_oled_lost  = true;
                g_oled_ready = false;
            }
            return false;
        }
        g_oled_dirty &= (uint8_t)(~(uint8_t)(1U << page));
    }

    g_oled_fail_count = 0U;
    return true;
}
