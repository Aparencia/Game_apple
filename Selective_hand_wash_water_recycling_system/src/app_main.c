/*
 * app_main.c
 *
 *  Created on: 2026年10月30日
 *      Author: Administrator
 */




/*
 * app_main.c —— 传感器节点应用层（初始化 / 50 ms 主循环 / 串口协议 / CSV / LED / 阀 /
 *               板载 OLED 状态页）
 * ---------------------------------------------------------------------------
 * 本文件是"唯一做外设初始化 + 唯一打印"的文件：
 *   - 上电按固定顺序初始化：P104 先置排空（fail-safe）→ LED → UART → 板载 OLED（步骤 2b）→
 *     ADC → DS18B20 自检 → pipeline → decision（含 validity 工况机）→ 启动开机自检；
 *   - 主循环由 timebase_ms() 调度成 50 ms tick（落后不补做，避免时间堆积）；
 *   - 串口是数据留痕通道：115200 8N1，命令 H/M/D/W/G/Z/T/C/P/V1/V0/VA，每 200 ms
 *     输出一行 CSV，事件行以 "# " 开头；板载 OLED 只做现场指示（不参与判别/安全侧）。
 *
 * v2.1.0（2026-11-16，开机自检）：
 *   现场反馈"每次上电都要串口敲 D/W"不现实，于是加入**开机自检**：
 *     ① 上电先落一组默认干态指纹（60 mV / 2337 mV），空腔立即显示 DRY，不再一直 NO_CAL；
 *     ② 空腔 → 自动采干态 10 s（比手动 D 的 60 s 短，只用于浸没判据）；
 *     ③ 等水：检测到电极浸没（EC ≥ 3×EC_DRY）并稳定 800 ms 后，自动采水中基线 2 s；
 *     ④ 校验通过 → 正常判别；失败最多重试 3 次，仍失败则保持 FAULT/排空（绝不带病回收）。
 *   安全边界：自检只自动采"干态"（空气参考，安全）与"确认浸没后的湿态"；
 *             没有基线时 decision 恒为 FAULT/DRY → 阀排空，不会重演旧固件"回收空气"。
 *             手动 D/W/G/Z 仍可随时接管。
 *
 * v2.1.1（2026-11-16，串口提示）：
 *   现场看 CSV 只有 17 列数字，不知道"现在什么工况、下一步做什么"。新增每 2 s 一行 `# TIP`：
 *     # TIP t=… | regime=MEASURE state=CLEAR level=L0 permit=1 |
 *       EC=1492mV g=1.001 dEC=+2mV dNTU=+2.1% T=29.1C | autocal=DONE | clean water: RECOVER
 *   它和 CSV 交替输出，以 '#' 开头（脚本可直接忽略），不改变 CSV 列数/顺序；
 *   同时在 BASELINE/DRY 完成行追加 "next: ..." 提示，并在启动时打印一次输出格式图例。
 *
 * v2.2.0（2026-11-16，清水抖动修订，依据三份现场日志）：
 *   现场三份日志（空腔/清水/肥皂）显示：清水 EC0≈348 mV，阀换向引起的水力漂移让
 *   ΔEC 达 ±16 mV（≈5%×EC0），而固定 d1=8 mV（≈2.3%）落在漂移带内，清水在
 *   CLEAR↔UNCERT 之间约 2.2 s 周期抖动（25 s 内 19 次换向，计数 36→45）。
 *   修订：① d1/d2 增加 EC0 百分比下限（6%/9%，348→20.9/31.3 mV）；
 *         ② L0→L1 需连续 2 块（0.8 s），L0→L2 仍立即；
 *         ③ NTU 阈值下调为 d1=4%、d2=8%（清水 -1.6%、肥皂 +10.9%）。
 *   用同一批日志回放：清水 ΔEC≤16<d1、ΔNTU≤-0.3<4 → 全程 L0；肥皂 ΔEC≥36.9>d2、
 *   ΔNTU≥8.6>8 → L2。阈值仍全部可调，换水样后应按新数据重标。
 *
 * v2.3.0（2026-11-17，自检识别修复）：
 *   现场反馈"自检完成后清水仍显示待机"。原因是自检/基线里两个固定倍数假设不成立：
 *   ① 清水 EC0≈348 mV，3×EC_DRY 在干态被残水抬到 120 mV 时会让清水（351）判"没水"；
 *   ② 本装置水 AO≈2639 mV **高于**空气 AO≈2241 mV，而 v2.1 自检却要求 AO 低于干态；
 *   ③ 默认干态时水中基线要求 ≥400 mV，348 的清水直接被拒。
 *   修订：浸没/基线改为"EC ≥ EC_DRY + 80 mV"；默认干态基线下限 150 mV；
 *         自动采湿态要求"EC 差值 + AO 高于空腔（默认 +80 mV）"双证据；
 *         SETTLE 增加 6 s 兜底（水一直流动、STABLE 不成立时也能进 MEASURE）。
 *   另：OLED 自检状态行改为 ≤16 字符短句（8x16 字体每行最多 16 字符）。
 *
 * v2.4.0（2026-11-17，进水识别加速 + OLED 大小写）：
 *   现场进水数据：水在 4.4 s 到达、4.8 s EC 已到 335、5.6 s 后 |rate|<30；
 *   旧逻辑固定等 4 s（8.6 s 才进 MEASURE），表现为"水已淹没电极却仍待机"。
 *   修订：SETTLE 自适应提前——连续稳定 500 ms、且不早于 1.5 s、且 dEC≤0（水不比基线脏）
 *         即可进 MEASURE；dEC>0 仍等 4 s/6 s，避免慢速污染被提前放行。
 *   OLED 英文改为"首字母大写、其余小写"（代码/单位如 L1/EC0/mV 保持大写），
 *   并把状态串控制在 8x16 字体的 16 字符以内。
 *
 * v2.4.1（2026-11-17，洗手情景仿真修正）：
 *   仿真真实洗手过程发现：肥皂水 EC 上升约 50 mV/s、浊度约 16%/s，而 v2.4 的
 *   "稳定"阈值是 30 mV/s / 5%/s，肥皂一进来工况就被判"不稳定"→ 退回 SETTLE，
 *   OLED 显示"待机"约 4 s（虽然阀已排空，但状态不是 SOAP）。
 *   修订：稳定带抬到 80 mV/s / 20%/s（仍高于换向水力漂移约 38 mV/s），
 *   肥皂期间保持 MEASURE，直接由 dEC/dNTU 判 L2/SOAP；进水/失水仍会被判不稳定。
 *
 * v2.0.0（2026-11-16，现场诊断后重做控制模型）：
 *   旧模型"事件闩锁 + 30 s 超时"被整体替换为"工况门控 + 每 tick 重算的回收许可"：
 *     validity(gate) 先判"能不能判"（FAULT/DRY/SETTLE/MEASURE/LOCKOUT），
 *     decision 只在 MEASURE 里分类，PERMIT = (level==L0)，谓词一变阀立刻跟随。
 *   直接效果：空腔/没淹到电极不再回收；换向扰动不再买到 30 s 许可；清水能被回收。
 *   同时取消上电自动采基线（那是"把空气当清水本底"的根因），改为显式标定：
 *     D（空腔 60 s 干态指纹）→ W（水中 2 s 湿态基线，带量程/CV/比值校验）。
 *   依据：Docs/软件层优化方案_v2_20261116.md、Docs/现场诊断报告_无水日志_20261116.md。
 *   兼容性：CSV 仍 17 列；state 取值仍 WAIT/CLEAR/UNCERT/SOAP（FLUSH 保留枚举不再输出）；
 *           note 列改为工况码（OK/DRY/SET/BLK/LOCK/NOBASE/SENS），新增 # GATE 事件行。
 *
 * v1.2.0（2026-11-13）起执行器升级为真实阀：
 *   P104 → SRD-05VDC-SL-C 单路继电器模块 IN；继电器常开触点驱动 12V 二位三通电磁阀。
 *   阀的逻辑位置由 actuator_valve 模块管理（驻留防抖 500 ms / 连续吸合上限 120 s /
 *   冷却 10 s / 引脚回读自检 / 动作计数）；APP_USE_VALVE=1 为默认，0=假执行器兼容模式。
 *   接线、极性判定与验收见 docs_sensor\15_执行器_二位三通电磁阀_SRD-05VDC-SL-C继电器模块.txt。
 *
 * v1.3.0（2026-11-13）起 OLED 升级为三页现场指示（K2 短按切页，长按仍重采基线）：
 *   P1 水质页（清水/污水/待机/冲洗/故障 + dEC/g/N）；
 *   P2 执行器页（大字"回收/排空/手动"+ 继电器实际状态/吸合次数/通电时长）；
 *   P3 详情页（事件状态与时长/STATE/LEVEL/阀位置/ΔEC/g/ΔNTU/水温/EC0/σ/d1/d2）；
 *   切页只影响显示，不改变判别/阀/CSV；串口 M 仍是打 MARK；详见 16 文档。
 *
 * 为什么上电第一件事是把 P104 置"排空"：
 *   复位/配置完成前引脚是高阻，若继电器已接线且 12V 已上电，阀可能被误吸合。
 *   因此固件在 app_main 的第一条外设操作里就把 P104 配成输出并锁在"继电器释放"电平
 *   （默认高触发模块 = LOW）；硬件上 P104 还要配 10 kΩ 下拉（低触发模块改上拉），
 *   覆盖复位高阻期——软件不能替代这个电阻。
 *
 * printf 重定向（复用 tds_mini v1.2 已验证实现）：
 *   - __io_putchar() 逐字节调用 R_SCI_UART_Write()，等 TX_COMPLETE 回调置位后才返回；
 *   - _write() 把 newlib 的输出拆成逐字节写；
 *   - 顺序必须是：R_SCI_UART_Open() 成功 → setvbuf(stdout, NULL, _IONBF, 0) →
 *     立刻打印 "# [BOOT] SENSOR-NODE v1.1 build ..."（随后一行
 *     "# board: RA4M2_MINI (R7FA4M2AD3CFL) | tick base: SysTick 1 ms"）。
 *     否则前 1 KB 会憋在 C 库缓冲里，现象就是"上电好几秒串口一个字都没有"。
 *   - 不允许在中断里调用 printf：__io_putchar 会忙等 TX 完成，且不可重入。
 *
 * 为什么不用浮点 printf：
 *   工程约定不使用浮点 printf（避免把 newlib 浮点格式化/软浮点支持拖进来，
 *   也避免不同链接选项下输出不稳定）；CSV 全部用"缩放整数"打印，
 *   例如 t_Cx100 = (int)(temp_c * 100 + 0.5)。浮点只用于内部计算。
 *
 * 接线（详细见 00 文档、12 文档与 15 文档）：
 *   TDS AOUT → P014/AN12；TS-300B AO → 1k+2k 分压 → P013/AN11；
 *   DS18B20 DQ → P500（4.7k 上拉到 3.3V）；UART9 → P109/P110（CH340E，115200 8N1）；
 *   LED1 → P111（高电平点亮）；K2 → P000（板载上拉，按下为低；短按切 OLED 页 / 长按重采基线）；
 *   继电器 IN → P104（U2-8）；GND → U2-18（必须共地）。
 *   继电器供电按模块类型二选一（15 文档 §3 有判定流程）：
 *     A 型（带 JD-VCC 跳线的光耦模块，推荐）：拆掉 JD-VCC 跳线，VCC → 3.3V（U1-12），
 *       JD-VCC → U2-19/20（VIN≈4.5~4.7 V）；IN 只看到 3.3V 域，高/低触发都可安全使用；
 *     B 型（三针 VCC/GND/IN）：VCC → U2-19/20（VIN）；只能用光耦输入型，且优先高触发；
 *       无光耦的 PNP 低电平触发款禁止直连 P104（IN 悬空会被抬到 ~4.3V，P104 不耐 5V）。
 *
 * 执行器安全（接真阀前必须逐条确认，详见 15 文档 §4/§7）：
 *   ① P104 与 GND 之间加 10 kΩ 下拉（高触发模块，默认）；若实测是低电平触发模块，
 *      改为 10 kΩ 上拉到 3.3V，并把 APP_VALVE_RELAY_ON_LEVEL 改成 BSP_IO_LEVEL_LOW，
 *      重新编译后再接 12V——极性接错时上电瞬间继电器会误吸合；
 *   ② 继电器线圈功耗 0.36 W（5V/69.4Ω≈72 mA），A 型拆 JD-VCC 后由 VIN 供线圈
 *      （must-operate 3.75 V，VIN 扣三极管压降后约 4.2~4.4 V 余量偏紧）；模块 VCC 就近
 *      并 100 µF + 0.1 µF，吸合瞬间线圈端应 ≥4.2 V，否则换独立 5V/≥300 mA 并共地；
 *   ③ 阀线圈并联 1N4007 续流二极管（阴极朝 +12V），12V 独立供电 + 2 A 保险丝；
 *   ④ 12V 船型开关装在继电器触点上游，现场可断电覆盖触点粘连；
 *   ⑤ 单次连续通电不超过 120 s（阀的连续工作制限制），固件按 120 s 强制释放 + 10 s 冷却；
 *      阀发烫立即断电并检查是否买到 ED 100% 的型号；
 *   ⑥ 不允许把阀串在主排水路上（红线）：本工程阀只做"回收/排空"换向，
 *      回收支路断电即关闭（fail-safe），主排水路永远通。
 *   ⑦ 继电器模块"光耦输入"在模块 VCC/GND 与板共地后不提供电气隔离，只作电平驱动；
 *      阀的反峰靠 1N4007 与触点吸收，12V 与 5V 两路电源只在端子排单点共地。
 *
 * 时间预算（项目设想 6.2.1，v2 参数）：
 *   tick 50 ms / 块 400 ms（8 采样 6 一致）/ 升级 L0 需 3 块 / CSV 200 ms /
 *   入水稳定期 4000 ms / 换向屏蔽 400 ms / 变化率抢先 120 mV/s（连续 2 tick）/
 *   DS18B20 每 2 s 发起、800 ms 后读数（非阻塞）；
 *   OLED 状态页：状态变化立即刷、数值最快 200 ms、心跳 1 s，只刷脏页；
 *   单次刷新 I2C 阻塞约 3 ms/页（400 kHz，整屏 ≈25 ms），失败不影响该 tick 的安全工作。
 *
 * 已知限制（报告口径，禁止写成能力）：
 *   - 无流量计：用"浸没 + 稳定"识别工况；R/E 只是日志分段标记，不再控制阀；
 *   - 干态指纹/水中基线都只存在 RAM（掉电丢失，P2 计划用数据 Flash 保存）；
 *   - 浊度只输出"相对基线变化 %"，不是 NTU；TDS 参考 ppm 只作过程量，不作结论；
 *   - OLED 失效（连续 3 帧刷新失败）不影响判别/CSV/阀，安全侧不依赖显示；
 *   - 本文件不识别污染物种类、不做达标判定（红线）。
 *   - 浊度只输出"相对基线变化 %"，不是 NTU；TDS 参考 ppm 只作过程量，不作结论；
 *   - 本文件不识别污染物种类、不做达标判定（红线）。
 * ---------------------------------------------------------------------------
 * 参考：spec_design.md §5/§6/§7；spec_headers.md §7/§9；硬件方案 §5.2、§6.4、§7.5。
 */

#include "app_main.h"

#include "hal_data.h"
#include "sensor_common.h"
#include "sensor_pipeline.h"
#include "sensor_ds18b20.h"
#include "decision.h"
#include "display.h"                 /* 板载 OLED 状态页（display_init/display_task） */
#include "actuator_valve.h"          /* 阀/继电器执行器（P104 → SRD-05VDC-SL-C → 12V 阀） */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/types.h>               /* off_t：newlib 系统调用原型 */
#include <sys/stat.h>                /* struct stat / S_IFCHR：printf 重定向需要的空桩 */

/* ---------- 应用参数（全部集中在这里，便于标定） ---------- */
#define APP_FW_VERSION          "SENSOR-NODE v2.4.1-FAST"  /* v2.4.1=稳定带 80/20，避免肥皂把工况踢回 SETTLE */
#define APP_TICK_MS             50U                  /* 主循环 tick（项目设想 6.2.1） */
#define APP_CSV_PERIOD_MS       200U                 /* CSV 输出周期：5 行/秒 */
#define APP_TIP_PERIOD_MS       2000U                /* 人类可读摘要（# TIP）周期：2 s */
#define APP_KEY_LONG_MS         1500U                /* K2 长按判据：≥1.5 s 重采基线 */
#define APP_RX_BUF_SIZE         64U                  /* 串口命令行缓冲（含 '\0'） */
#define APP_CSV_LINE_MAX        256U                 /* CSV 行缓冲（17 列整数+逗号足够） */
#define APP_BASELINE_N          40U                  /* 基线点数（40 × 50 ms = 2 s，方案 §10） */
#define APP_ADC_MEDIAN_N        5U                   /* 每通道每 tick 中值采样次数 */
#define APP_ADC_TIMEOUT_MS      5U                   /* 单次 ADC 扫描超时 */
#define APP_DRY_CAPTURE_MS      60000U               /* 手动 D 的采集时长（方案 §8.1：60 s） */
#define APP_DRY_CAPTURE_N       (APP_DRY_CAPTURE_MS / APP_TICK_MS)  /* 1200 点 */
#define APP_DRY_CV_MAX_PCT      5.0f                 /* 干态指纹稳定性上限（方案 §8.1） */

/* ---------- 开机自检（v2.1：不要求用户输入 D/W） ---------- */
#define APP_AUTOCAL_ENABLE       1                   /* 1=上电自动干态→等水→自动湿态；0=只手动 */
#define APP_AUTOCAL_DRY_MS       10000U              /* 自检干态采集 10 s（200 点） */
#define APP_AUTOCAL_DRY_N        (APP_AUTOCAL_DRY_MS / APP_TICK_MS)
#define APP_AUTOCAL_WATER_MS     800U                /* 检测到浸没后，稳定多久再采湿态 */
#define APP_AUTOCAL_RETRY_MS     8000U               /* 湿态失败后的重试间隔 */
#define APP_AUTOCAL_WET_TRIES    3U                  /* 湿态最多尝试次数 */
#define APP_AUTOCAL_WET_TIMEOUT  8000U               /* 单次湿态采集的兜底超时（2 s 标称，留 4× 余量） */
#define APP_AUTOCAL_FAIL_RETRY_MS 300000U            /* 自检失败后每 5 min 低频重试一次 */
/* 默认干态指纹：来自 2026-11-16 现场实测（空腔 TDS 43.7~58 mV、浊度 AO≈2337 mV）。
 * 只用于"是否浸没"的绝对门限；没有水中基线时工况仍是 FAULT，不会回收。 */
#define APP_EC_DRY_DEFAULT_MV    60.0f
#define APP_AO_DRY_DEFAULT_MV    2337.0f

/* ---------- 串口发送自检/兜底 ---------- */
/* 背景：printf 走 _write → __io_putchar，每字节用忙等等 TX_COMPLETE 回调置位。
 * 若 TXI/TEI 中断没有进来（配置错/中断被关/向量错），这里会永久卡死在上电第一行，
 * 表现就是"灯不闪、屏不亮、串口只有一两个字节"。
 * 因此加自旋上限：超时就把这一字节丢掉继续跑，连续失败达到上限后彻底放弃阻塞式打印。 */
#define APP_UART_TX_WAIT_LOOPS  300000U              /* 单字节自旋上限（约 1~7 ms @180MHz，远大于 87 µs） */
#define APP_UART_TX_FAIL_LIMIT  8U                   /* 连续超时达到该数 → 后续 printf 直接丢字符 */

/* ---------- 执行器：二位三通电磁阀 + SRD-05VDC-SL-C 单路继电器模块 ---------- */
/* APP_USE_VALVE = 1（v1.2.0 起默认）：P104 真正驱动继电器 IN；
 * APP_USE_VALVE = 0：假执行器兼容模式（P104 仍锁排空电平，但不换向，只用 LED1 演示，
 *                    实验 3 的做法；上台架前不需要改这个宏）。
 * 接线前先按 15 文档 §3/§4 判定模块类型与触发极性：
 *   A 型（带 JD-VCC 跳线，推荐）：拆跳线，VCC→3.3V、JD-VCC→VIN，IN 只看到 3.3V 域；
 *   B 型（三针）：只能用光耦输入型（优先高触发）；无光耦 PNP 低触发款禁止直连 P104。
 *   默认 HIGH = 高电平触发模块（回收=HIGH 继电器吸合、排空=LOW 释放）；
 *   若模块实测为低电平触发，改成 BSP_IO_LEVEL_LOW，并把 P104 的 10 kΩ 下拉改成
 *   10 kΩ 上拉到 3.3V，再重新编译。极性判错 + 已接 12V 时，上电瞬间阀会误吸合。 */
#define APP_USE_VALVE                1
/* ⚠ RA 引脚命名 P104 = port 1 + pin 4（不是 port 10）；对应 FSP 宏 BSP_IO_PORT_01_PIN_04。
 *   本板没有 port 10：若误写成 10 号端口的宏，代码仍能编译，但
 *   R_IOPORT_PinCfg/PinWrite 运行时会失败 → 执行器模块回读自检会报 INIT_FAIL。 */
#define APP_VALVE_PIN                BSP_IO_PORT_01_PIN_04   /* P104 / U2-8 */
#define APP_VALVE_RELAY_ON_LEVEL     BSP_IO_LEVEL_HIGH       /* 回收位（继电器吸合）电平：默认高触发模块 */
#define APP_VALVE_MIN_DWELL_MS       500U                    /* 最小换向间隔：防继电器抖动 */
#define APP_VALVE_MAX_ON_MS          120000U                 /* 单次连续吸合上限（阀连续工作制） */
#define APP_VALVE_COOLDOWN_MS        10000U                  /* 超时强制释放后的冷却时间 */

/* ---------- 板载引脚（与已验证固件 tds_mini 一致） ---------- */
#define APP_LED_PIN             BSP_IO_PORT_01_PIN_11   /* P111 / U2-2，高电平点亮 */
#define APP_KEY_PIN             BSP_IO_PORT_00_PIN_00   /* P000 / U1-14，K2，按下为低 */

/* ---------- 执行器模式（V1/V0/VA 命令） ---------- */
typedef enum e_app_valve_mode
{
    APP_VALVE_AUTO = 0,        /* 跟随判别结果（默认） */
    APP_VALVE_FORCE_RECOVER,   /* V1：强制回收（调试/演示） */
    APP_VALVE_FORCE_DRAIN      /* V0：强制排空（调试/演示） */
} app_valve_mode_t;

/* ---------- 流水线配置（默认值来源：spec §6，全部"待标定"） ---------- */
static const pipeline_cfg_t g_app_pipeline_cfg =
{
    .p_adc_ctrl         = &g_adc0_ctrl,
    .tds_channel        = ADC_CHANNEL_12,    /* P014/AN12：TDS AOUT 直连 */
    .ntu_channel        = ADC_CHANNEL_11,    /* P013/AN11：TS-300B 经 1k+2k 分压 */
    .adc_median_n       = APP_ADC_MEDIAN_N,
    .adc_timeout_ms     = APP_ADC_TIMEOUT_MS,
    .vref_mv            = 3300U,
    .adc_full_scale     = 4095U,
    .ema_alpha          = 0.15f,            /* v2：0.30→0.15，配合 4 s 稳定期 */
    .tds_cal_k          = 1.0f,             /* 无标准液时保持 1.0 */
    .ntu_div_r1_ohm     = 1000.0f,          /* 分压上臂（串） */
    .ntu_div_r2_ohm     = 2000.0f,          /* 分压下臂（对地） */
    .ntu_clear_high     = true,             /* 清水电压高；反相批次改 false（先做清水/浑水对照） */
    .baseline_n         = APP_BASELINE_N,
    .sigma_floor_ec_mv  = 1.5f,             /* σ 下限：防基线"过干净"阈值退化为 0 */
    .sigma_floor_ntu_mv = 2.0f,
    .baseline_cv_max_pct        = 1.0f,     /* 基线稳定性：σ/EC0 ≤ 1 %（方案 §8.1） */
    .baseline_sigma_abs_max_mv  = 3.0f,     /* 绝对兜底：σ ≤ 3 mV 也算稳定 */
    .ec0_max_mv                 = 1600.0f   /* v2.1：1600 以上判"开路伪值/脏水本底"，不采基线 */
};

/* ---------- 判别器配置（默认值来源：方案 v2 §12 参数总表，全部"待标定"） ---------- */
static const decision_cfg_t g_app_decision_cfg =
{
    .ec_d1_sigma         = 4.0f,      /* d1 = max(4σ, 8 mV) */
    .ec_d2_sigma         = 8.0f,      /* d2 = max(8σ, 20 mV) */
    .ec_d1_min_mv        = 8.0f,
    .ec_d2_min_mv        = 20.0f,
    .ec_d1_pct           = 6.0f,      /* v2.2：清水漂移 ±5%×EC0，d1 取 6% */
    .ec_d2_pct           = 9.0f,      /* 肥皂现场 +11.5%×EC0，d2 取 9% */
    .ec_hyst_ratio       = 0.25f,     /* 25% 迟滞，防阈值附近抖 */
    .ntu_d1_pct          = 4.0f,      /* v2.2：清水 dNTU -2.6..-0.3，肥皂 +8.6..+13.1 */
    .ntu_d2_pct          = 8.0f,      /* 肥皂最小值 8.6 > 8 → L2；清水不会触发 */
    .ntu_hyst_ratio      = 0.25f,
    .ec_rate_th_mv_s     = 120.0f,    /* v2：40→120（≈3× 实测换向瞬态 38 mV/s） */
    .ec_rate_min_jump_mv = 5.0f,
    .ntu_rate_th_pct_s   = 150.0f,    /* v2：50→150（原阈值等于永不触发） */
    .rate_confirm_ticks  = 2U,        /* 连续 2 tick 才认（排单点毛刺） */
    .block_ms            = 400U,      /* v2：200→400，与 5 Hz CSV / 4 s 稳定期匹配 */
    .block_samples       = 8U,        /* v2：4→8（400 ms / 50 ms） */
    .block_agree         = 6U,        /* v2：3→6（8 中 6，抗气泡） */
    .upgrade_blocks      = 3U,        /* 单侧迟滞：升级到 L0 需连续 3 块（1.2 s） */
    .downgrade_blocks    = 2U,        /* v2.2：L0→L1 需连续 2 块（0.8 s）；L0→L2 立即 */
    /* 工况机（方案 §12）：只写现场要调的项，其余 0 值由 gate_init 补默认 */
    .gate =
    {
        .wet_ratio          = 0.30f,      /* g = EC/EC0 ≥ 0.30 才算淹到电极 */
        .ec_wet_delta_mv    = 80.0f,      /* v2.3：EC ≥ EC_DRY + 80 mV 才算有水 */
        .ec_dry_ratio       = 3.0f,       /* 已弃用（保留字段，不参与判据） */
        .ec_dry_max_mv      = 200.0f,     /* 干态指纹上限 */
        .ec0_min_mv         = 50.0f,
        .ec0_max_mv         = 1600.0f,    /* 开路伪值 ~1650 与脏水本底都要挡住 */
        .ec0_default_min_mv = 150.0f,     /* v2.3：默认干态时基线只需 ≥150 mV（现场水 348） */
        .ao_water_delta_mv  = 80.0f,      /* v2.3：本装置水 AO 比空气高约 400 mV */
        .ntu_ao_min_mv      = 20U,
        .rate_stable_mv_s   = 80.0f,      /* v2.4.1：肥皂 ~50、换向 ~38，30 太紧会误回 SETTLE */
        .rate_stable_pct_s  = 20.0f,      /* 肥皂 ~16%/s，5 太紧 */
        .settle_ms          = 4000U,      /* 实测 3.8 s 才进 ±1 % */
        .settle_min_ms      = 1500U,      /* v2.4：最早 1.5 s 进 MEASURE */
        .settle_stable_ms   = 500U,       /* 连续稳定 0.5 s 可提前进入 */
        .settle_timeout_ms  = 6000U,      /* 水在流动/气泡多时的兜底，避免永远待机 */
        .blank_ms           = 400U,       /* 换向屏蔽窗（S6） */
        .wet_debounce_ms    = 500U,
        .dry_debounce_ms    = 300U,       /* 安全侧要快 */
        .unstable_ms        = 1000U,
        .fault_recover_ms   = 1000U,
        .lockout_ms         = 3000U
    }
};

/* ---------- 全局对象（单实例；静态分配，无动态内存） ---------- */
static sensor_pipeline_t g_pipe;   /* 采集/特征流水线（句柄由 app_main 分配） */
static decision_t        g_dec;    /* 三级判别状态机 */
static ds18b20_t         g_ds;     /* DS18B20 水温传感器 */

/* ---------- UART 发送 / 接收 ---------- */
static volatile bool    g_uart_tx_done = true;   /* TX_COMPLETE 回调置 true */
static volatile uint32_t g_uart_tx_timeouts = 0U; /* TX_COMPLETE 连续超时计数（≥上限后 printf 不再阻塞） */
static char             g_rx_buf[APP_RX_BUF_SIZE];
static volatile uint8_t g_rx_len       = 0U;
static volatile bool    g_rx_ready     = false;  /* 收满一整行（\r 或 \n） */
static volatile bool    g_rx_overflow  = false;  /* 本行超长，丢弃 */

/* ---------- 命令触发的动作标志（主循环里消费，不在 ISR 里执行） ---------- */
static bool g_force_temp_recheck = false;   /* 命令 C：下一 tick 强制重检 DS18B20 */

/* ---------- 执行器/日志状态 ---------- */
static app_valve_mode_t g_valve_mode = APP_VALVE_AUTO;

/* 阀/继电器执行器配置（时间常量见文件头"执行器"区块；应用参数集中在此） */
static const valve_cfg_t g_app_valve_cfg =
{
    .pin            = APP_VALVE_PIN,
    .enabled        = (APP_USE_VALVE != 0),
    .relay_on_level = APP_VALVE_RELAY_ON_LEVEL,
    .min_dwell_ms   = APP_VALVE_MIN_DWELL_MS,
    .max_on_ms      = APP_VALVE_MAX_ON_MS,
    .cooldown_ms    = APP_VALVE_COOLDOWN_MS
};
static valve_t    g_valve;                 /* 执行器句柄（静态分配） */
static valve_pos_t g_prev_valve_pos = VALVE_POS_DRAIN;  /* 日志边沿检测 */
static bool        g_prev_valve_fault = false;          /* 引脚故障只报一次 */
static bool        g_prev_valve_blocked = false;        /* 连续吸合超时只报一次 */

static bool     g_key_prev_down  = false;   /* 按键边沿检测（上一步是否按下） */
static bool     g_key_long_fired = false;   /* 本次按下是否已触发长按 */
static uint32_t g_key_press_ms   = 0U;      /* 本次按下时刻（长按计时） */

static bool g_prev_event_active = false;    /* 上一步事件状态（检测 EVENT 边界） */
static decision_state_t  g_prev_state  = DEC_STATE_WAIT;   /* 上一步状态（检测 STATE 边界） */
static decision_action_t g_prev_action = DEC_ACTION_DRAIN; /* 上一步动作（检测 ACTION 边界） */
static bool g_manual_event_started = false; /* 本次事件是否由手动 R 开始（日志来源标注） */
static bool g_manual_event_ended   = false; /* 本次事件是否由手动 E 结束 */
static uint32_t g_last_csv_ms = 0U;         /* 上次 CSV 输出时刻 */
static uint32_t g_last_tip_ms = 0U;         /* 上次 # TIP 摘要输出时刻 */

/* 工况日志边沿检测（方案 §11：只在变化时打 # GATE） */
static uint16_t  g_prev_valid_bits = 0U;               /* 上一步 validity 位图 */
static regime_t  g_prev_regime     = REGIME_FAULT;     /* 上一步工况 */

/* ---------- 干态指纹采集（命令 D，方案 §8.1） ---------- */
/*
 * 为什么干态指纹必须单独采：空气里 TDS 是 58 mV 左右，比清水（数百~1500 mV）低得多，
 * 而浊度在空气里又落在"最干净"的一端 —— 没有干态参考，就无法区分"空腔"与"清水"。
 * 采集期间不做任何判断（本来就是空腔，判什么都是错的），完成后写进 gate。
 */
typedef struct st_app_dry_capture
{
    bool     active;        /* 正在采集 */
    bool     auto_cap;      /* true=开机自检自动采集（可被 W 中止）；false=手动 D */
    uint16_t n;             /* 已收样本数 */
    uint16_t target_n;      /* 目标样本数（手动 1200 / 自检 200） */
    float    ec_sum;        /* Σ EC(mV25) */
    float    ec_sumsq;      /* Σ EC² */
    float    ao_sum;        /* Σ 浊度 AO(mV) */
    float    ao_sumsq;      /* Σ AO² */
    uint32_t start_ms;      /* 开始时刻（进度打印） */
} app_dry_capture_t;

static app_dry_capture_t g_dry_capture = { false, false, 0U, 0U, 0.0f, 0.0f, 0.0f, 0.0f, 0U };

/* ---------- 开机自检状态机（v2.1） ---------- */
/*
 * 目标：上电后不需要串口 D/W，装置自己完成"干态 → 等水 → 水中基线 → 正常判别"。
 * 安全前提（与旧固件"上电在空气里采基线"的根本区别）：
 *   ① 自动采的干态只用于"是否浸没"，绝不能当水中基线；
 *   ② 水中基线只在"确认浸没（EC≥3×EC_DRY）"之后才采；
 *   ③ 自检全程没有基线 → decision 仍是 FAULT/DRY → 阀恒排空，不可能回收空气；
 *   ④ 手动 D/W/G/Z 随时可覆盖自检。
 * 已知限制：没有 Flash 持久化时，每次上电都要重来；若上电时腔里已有脏水，
 *           自动采到的"水中基线"就是脏水的 —— 后果是之后更干净的水会被判"太干净"
 *           而排空（欠回收），不会误回收；需要干净水时手动 W 重标。
 */
typedef enum e_app_autocal_state
{
    APP_AC_OFF = 0,   /* 关闭/手动接管 */
    APP_AC_DRY,       /* 自动采干态 */
    APP_AC_WAIT,      /* 等水（电极浸没） */
    APP_AC_SETTLE,    /* 检测到水，稳定 800 ms */
    APP_AC_WET,       /* 自动采水中基线 */
    APP_AC_RETRY,     /* 湿态失败，等重试 */
    APP_AC_DONE       /* 完成（成功或放弃） */
} app_autocal_state_t;

typedef enum e_app_base_result
{
    APP_BASE_NONE = 0,   /* 本次基线采集还没有结论 */
    APP_BASE_OK,         /* 采集完成且校验通过 */
    APP_BASE_REJECT      /* 采集完成但校验失败 */
} app_base_result_t;

static app_autocal_state_t g_ac_state = APP_AC_OFF;
static uint32_t            g_ac_since_ms = 0U;
static uint8_t             g_ac_dry_tries = 0U;
static uint8_t             g_ac_wet_tries = 0U;
static bool                g_ac_start_pending = true;
static bool                g_ac_dry_blocked = false;   /* 自动干态因"腔里有水/电极湿"被拒 */
static float               g_dry_last_mean_mv = 0.0f;  /* 最近一次干态采集的 EC 均值（诊断用） */
static bool                g_ac_fail_retry = false;    /* 自检 base_fail 后是否允许低频重试 */
static bool                g_ac_need_manual = false;   /* 自检失败，需要人工 W（OLED 提示） */
static uint32_t            g_ac_done_since_ms = 0U;    /* 进入 DONE 的时刻 */
static app_base_result_t   g_base_result = APP_BASE_NONE;

static bool g_oled_warned = false;           /* OLED 掉线只报一次（显示不是安全路径） */

/* ---------- 文件内函数声明 ---------- */
static char        app_to_upper(char c);
static int32_t     app_scale_int(float value, float scale);
static void        app_fmt_scaled(char *p_buf, size_t buf_size, float value, int32_t scale, int decimals);
static void        app_fmt_x10(char *p_buf, size_t buf_size, float value);
static void        app_fmt_x1000(char *p_buf, size_t buf_size, float value);
static const char *app_temp_src_str(ds18b20_src_t src);
static const char *app_valve_mode_str(app_valve_mode_t mode);
static void        app_led_write(bool on);
static void        app_led_init(void);
static void        app_led_boot_blink(void);
static void        app_fatal_led_forever(void);
static void        app_valve_init(void);
static void        app_valve_apply(bool recover, uint32_t now_ms);
static void        app_valve_log(uint32_t now_ms);
static bool        app_valve_recover_requested(const decision_out_t *p_dec);
static void        app_led_task(const decision_out_t *p_dec, uint32_t now_ms);
static void        app_baseline_begin(uint32_t now_ms, const char *p_src);
static void        app_baseline_validate(uint32_t now_ms, const pipeline_out_t *p_out);
static void        app_dry_task(uint32_t now_ms, const pipeline_out_t *p_out);
static void        app_dry_begin(uint32_t now_ms, uint16_t target_n, bool auto_cap);
static void        app_autocal_start(uint32_t now_ms);
static void        app_autocal_cancel(const char *p_why);
static void        app_autocal_task(uint32_t now_ms, const pipeline_out_t *p_out);
static const char *app_autocal_state_str(app_autocal_state_t state);
static const char *app_dry_source_str(dry_source_t source);
static void        app_print_gate(uint32_t now_ms);
static const char *app_note_code(const pipeline_out_t *p_out, const decision_out_t *p_dec);
static void        app_print_mark(const char *p_src, uint32_t now_ms);
static void        app_key_task(uint32_t now_ms);
static void        app_print_help(void);
static void        app_print_baseline_ok(const pipeline_out_t *p_out);
static void        app_print_status(uint32_t now_ms);
static void        app_handle_valve_cmd(const char *p_text);
static bool        app_parse_temperature(const char *p_text, float *p_temp_c);
static void        app_handle_command(char *p_cmd, uint32_t now_ms);
static void        app_poll_command(uint32_t now_ms);
static void        app_log_changes(uint32_t now_ms, const decision_out_t *p_dec);
static void        app_print_csv(uint32_t now_ms, const pipeline_out_t *p_out,
                                const decision_out_t *p_dec);
static void        app_print_tip(uint32_t now_ms, const pipeline_out_t *p_out,
                                 const decision_out_t *p_dec);
static const char *app_tip_hint(const decision_out_t *p_dec);
static void        app_print_boot_summary(void);
static bool        app_adc_init(void);
static bool        app_uart_init(void);
static void        app_tick(uint32_t now_ms);

/* ---------- printf 重定向到 UART9（复用 tds_mini v1.2 已验证实现） ---------- */
#ifdef __GNUC__
    #define APP_PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define APP_PUTCHAR_PROTOTYPE int fputc(int ch, FILE * f)
#endif

int __io_putchar(int ch);
int _write(int fd, char *pBuffer, int size);

/* newlib-nano 的 printf 会连带引用下面 5 个系统调用（没有文件系统也必须给空桩，
 * 否则链接期报 undefined reference；这是"只重定向 _write"最常见的坑）。 */
int   _read(int fd, void *pBuffer, int size);
int   _close(int fd);
off_t _lseek(int fd, off_t offset, int whence);
int   _fstat(int fd, struct stat *pStat);
int   _isatty(int fd);

/*
 * 函数名：__io_putchar
 * 作用：newlib 的底层单字符输出：轮询 SCI9 的 TDRE 标志后直接写 TDR。
 * 参数：ch —— 待发送字符（低 8 位有效）。
 * 返回：ch（newlib 约定）。
 * 时序/边界：115200 下 1 字节约 87 µs；只在主循环上下文调用（中断里禁止 printf）。
 * 注意：① 刻意"不走 R_SCI_UART_Write + TX_COMPLETE 回调"：
 *         中断没进来时那条路径会永久卡死在上电第一行（本次现场故障就是这样），
 *         轮询 TDRE 只依赖硬件状态，最不容易卡死；
 *       ② TDRE 一直等不到也有自旋上限：丢字节继续跑，绝不让串口拖死主循环。
 */
APP_PUTCHAR_PROTOTYPE
{
    uint32_t spin = 0U;

    /* 连续多次等不到 TDRE = 串口物理层判废：直接丢字符，别让每字节 5~8 ms 的等待拖垮启动 */
    if (g_uart_tx_timeouts >= APP_UART_TX_FAIL_LIMIT)
    {
        return ch;
    }

    while (0U == (R_SCI9->SSR & 0x80U))   /* 0x80 = SCI SSR.TDRE（发送数据寄存器空） */
    {
        spin++;
        if (spin >= APP_UART_TX_WAIT_LOOPS)
        {
            g_uart_tx_timeouts++;
            return ch;                    /* 通道异常：丢字符，返回继续跑 */
        }
    }

    R_SCI9->TDR = (uint8_t) ch;
    g_uart_tx_timeouts = 0U;
    return ch;
}

/*
 * 函数名：_write
 * 作用：newlib 的 write() 钩子：把一段缓冲区拆成逐字节 __io_putchar。
 * 参数：fd —— 文件描述符（忽略）；pBuffer —— 数据；size —— 字节数。
 * 返回：写入字节数 size。
 * 时序/边界：printf 每次格式化后调用；长度受单行格式串限制（本文件最长约 200 B）。
 * 注意：逐字节写会慢（每字节一次中断+等待），但 115200 下 200 B ≈ 17 ms，
 *       远小于 200 ms CSV 周期；如需提速应改成 DMA/一次 Write 整段，暂不做。
 */
int _write(int fd, char *pBuffer, int size)
{
    int i;

    (void)fd;
    for (i = 0; i < size; i++)
    {
        (void)__io_putchar((int)(*pBuffer));
        pBuffer++;
    }
    return size;
}

/*
 * 函数名：_close / _lseek / _read / _fstat / _isatty
 * 作用：newlib-nano 的 stdio 依赖的 5 个系统调用；本工程没有文件系统，全部空桩。
 * 参数/返回：按 newlib 约定（fd/缓冲区/偏移都被忽略）。
 * 时序/边界：只在库内部调用（printf 初始化/收尾），主循环不经过 stdio；
 *             串口命令走"中断行缓冲 + 主循环解析"，不经过 _read。
 * 注意：① 不实现就链接失败——只写 _write 是不够的；
 *       ② _fstat 必须把 st_mode 置成 S_IFCHR，printf 才会按字符设备处理（不整块缓冲）；
 *       ③ _read 返回 0、_close 返回 -1，符合"没有输入流/没有可关闭文件"的语义。
 */
int _close(int fd)
{
    (void) fd;
    return -1;
}

off_t _lseek(int fd, off_t offset, int whence)
{
    (void) fd;
    (void) offset;
    (void) whence;
    return (off_t) 0;
}

int _read(int fd, void *pBuffer, int size)
{
    (void) fd;
    (void) pBuffer;
    (void) size;
    return 0;
}

int _fstat(int fd, struct stat *pStat)
{
    (void) fd;
    pStat->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd)
{
    (void) fd;
    return 1;
}

/* ---------- FSP 回调（名字由 RASC 配置指定，不可改） ---------- */
/*
 * 函数名：adc_callback
 * 作用：ADC0 扫描完成回调，通知流水线"本 tick 的扫描结束了"。
 * 参数：p_args —— FSP 回调参数（只看 event）。
 * 返回：无。
 * 时序/边界：中断上下文；流水线在 pipeline_sample() 里等这个标志（带超时）。
 * 注意：只置位，绝不做耗时操作；pipeline_on_adc_complete 是唯一允许的调用。
 */
void adc_callback(adc_callback_args_t *p_args)
{
    if (ADC_EVENT_SCAN_COMPLETE == p_args->event)
    {
        pipeline_on_adc_complete();
    }
}

/*
 * 函数名：user_uart_callback
 * 作用：UART9 回调：TX 完成置发送标志；RX 字节拼成命令行（\r 或 \n 结束一行）。
 * 参数：p_args —— FSP 回调参数（event + data）。
 * 返回：无。
 * 时序/边界：中断上下文，每字节一次；行缓冲 64 B（含 '\0'），超长置 overflow 标志。
 * 注意：不收"半行"——只有收到行尾才置 g_rx_ready，主循环整行解析；
 *       主循环处理完之前，后续字节会被丢弃（人工输入不会同时涌入，够用）。
 */
void user_uart_callback(uart_callback_args_t *p_args)
{
    char c;

    if (UART_EVENT_TX_COMPLETE == p_args->event)
    {
        g_uart_tx_done = true;
        return;
    }

    /* 没有调用 R_SCI_UART_Read() 时，收到的每个字节都会以 RX_CHAR 事件进来 */
    if (UART_EVENT_RX_CHAR == p_args->event)
    {
        c = (char)p_args->data;

        if (('\r' == c) || ('\n' == c))
        {
            if (g_rx_len > 0U)
            {
                g_rx_ready = true;   /* 通知主循环：有一整行命令了 */
            }
        }
        else if (!g_rx_ready)
        {
            if (g_rx_len < (uint8_t)(APP_RX_BUF_SIZE - 1U))
            {
                g_rx_buf[g_rx_len] = c;
                g_rx_len++;
            }
            else
            {
                g_rx_overflow = true;
            }
        }
        else
        {
            ;   /* 主循环还没取走上一条命令：本行剩余字符直接丢弃 */
        }
    }
}

/* ---------- 小工具 ---------- */
/*
 * 函数名：app_to_upper
 * 作用：ASCII 小写转大写（命令大小写不敏感，避免 ctype.h 依赖）。
 * 参数：c —— 输入字符。
 * 返回：大写字符（非小写原样返回）。
 * 时序/边界：纯函数。
 * 注意：只处理 ASCII 字母；其他字节原样返回（命令都是 ASCII）。
 */
static char app_to_upper(char c)
{
    if ((c >= 'a') && (c <= 'z'))
    {
        return (char)((c - 'a') + 'A');
    }
    return c;
}

/*
 * 函数名：app_scale_int
 * 作用：浮点按给定比例缩放成四舍五入的整数（CSV 的"缩放整数"打印靠它）。
 * 参数：value —— 浮点值；scale —— 比例（10/100/1000）。
 * 返回：缩放并四舍五入后的 int32。
 * 时序/边界：负数按远离 0 四舍五入（-0.55 → -0.6），保证 ΔEC 负值打印正确。
 * 注意：不使用 libm（roundf）；值域受传感器量程限制，不会溢出 int32。
 */
static int32_t app_scale_int(float value, float scale)
{
    float scaled;

    /* NaN/非法比例直接归零：NaN 转 int32 是未定义行为（可能得到 INT_MIN），
     * 一旦漏进 CSV 会污染整列数据，这里做最后一道兜底。 */
    if (sensor_is_nan_f32(value) || sensor_is_nan_f32(scale) || (!(scale > 0.0f)))
    {
        return 0;
    }

    scaled = value * scale;
    if (scaled > 1000000.0f)
    {
        scaled = 1000000.0f;
    }
    if (scaled < (-1000000.0f))
    {
        scaled = (-1000000.0f);
    }

    if (scaled >= 0.0f)
    {
        return (int32_t)(scaled + 0.5f);
    }
    return (int32_t)(scaled - 0.5f);
}

/*
 * 函数名：app_fmt_scaled
 * 作用：把浮点按比例缩放后格式化成"固定小数位"的字符串（整数格式化，不碰浮点 printf）。
 * 参数：p_buf —— 输出缓冲（建议 ≥20 B）；buf_size —— 缓冲大小；
 *       value —— 浮点值；scale —— 比例（10 / 1000）；decimals —— 小数位数。
 * 返回：无（结果写入 p_buf）。
 * 时序/边界：纯函数；先四舍五入成整数再拆整数/小数部分，避免 "%.1f" 这类浮点格式化。
 * 注意：负数必须先取符号再取绝对值——否则 -0.3 会被拆成 whole=0、frac=3 而丢掉负号
 *       （P 状态里 ΔEC 常为小负数，丢负号会误导判读）；p_buf 必须非 NULL。
 */
static void app_fmt_scaled(char *p_buf, size_t buf_size, float value, int32_t scale, int decimals)
{
    int32_t scaled   = app_scale_int(value, (float)scale);
    const char *sign = "";
    int32_t whole;
    int32_t frac;

    if (scaled < 0)
    {
        sign   = "-";
        scaled = -scaled;
    }
    whole = scaled / scale;
    frac  = scaled % scale;
    (void)snprintf(p_buf, buf_size, "%s%ld.%0*ld", sign, (long)whole, decimals, (long)frac);
}

/*
 * 函数名：app_fmt_x10
 * 作用：格式化成"一位小数"的整数字符串：value×10 后取整（mV / ℃ / % 等用）。
 * 参数：p_buf —— 输出缓冲（建议 ≥20 B）；buf_size —— 缓冲大小；value —— 浮点值。
 * 返回：无（结果写入 p_buf）。
 * 时序/边界：纯函数；内部走 app_fmt_scaled，负数 -0.x 也会带负号。
 * 注意：结果用 %s 拼进日志/状态行，避免浮点 printf。
 */
static void app_fmt_x10(char *p_buf, size_t buf_size, float value)
{
    app_fmt_scaled(p_buf, buf_size, value, 10, 1);
}

/*
 * 函数名：app_fmt_x1000
 * 作用：格式化成"三位小数"的整数字符串（P 状态里的 g=mv25/EC0，1.0 显示为 1.000）。
 * 参数：p_buf —— 输出缓冲（建议 ≥20 B）；buf_size —— 缓冲大小；value —— 浮点值。
 * 返回：无（结果写入 p_buf）。
 * 时序/边界：纯函数；内部走 app_fmt_scaled（value×1000，三位小数）。
 * 注意：g 是比值，一位小数分不开 1.02 与 1.08，所以 P 状态里单独用三位小数。
 */
static void app_fmt_x1000(char *p_buf, size_t buf_size, float value)
{
    app_fmt_scaled(p_buf, buf_size, value, 1000, 3);
}

/*
 * 函数名：app_temp_src_str
 * 作用：温度来源枚举 → CSV 的 src 字段（DS / MAN / FIX）。
 * 参数：src —— ds18b20_src_t。
 * 返回：静态字符串（不返回 NULL）。
 * 时序/边界：纯函数。
 * 注意：CSV 协议（00 文档 §5.2）规定取值就是 DS/MAN/FIX 三个大写短词，
 *       所以这里自己映射，不依赖别的模块的字符串大小写。
 */
static const char *app_temp_src_str(ds18b20_src_t src)
{
    switch (src)
    {
        case DS18B20_SRC_SENSOR:
            return "DS";

        case DS18B20_SRC_MANUAL:
            return "MAN";

        case DS18B20_SRC_FIX:
        default:
            return "FIX";
    }
}

/*
 * 函数名：app_valve_mode_str
 * 作用：执行器模式 → 静态字符串（P 命令/日志用）。
 * 参数：mode —— app_valve_mode_t。
 * 返回："AUTO" / "MANUAL_RECOVER" / "MANUAL_DRAIN"（未知按 AUTO）。
 * 时序/边界：纯函数。
 * 注意：只影响显示，真正的阀电平由 app_valve_recover_requested 决定。
 */
static const char *app_valve_mode_str(app_valve_mode_t mode)
{
    switch (mode)
    {
        case APP_VALVE_FORCE_RECOVER:
            return "MANUAL_RECOVER";

        case APP_VALVE_FORCE_DRAIN:
            return "MANUAL_DRAIN";

        case APP_VALVE_AUTO:
        default:
            return "AUTO";
    }
}

/* ---------- LED（P111 假执行器） ---------- */
/*
 * 函数名：app_led_write
 * 作用：写 LED 电平（高电平点亮）。
 * 参数：on —— true 亮 / false 灭。
 * 返回：无。
 * 时序/边界：R_IOPORT_PinWrite 为寄存器写，微秒级，可在主循环里随便调用。
 * 注意：LED 是本阶段的"假执行器"：亮=回收、灭=排空。
 */
static void app_led_write(bool on)
{
    (void)R_IOPORT_PinWrite(&g_ioport_ctrl, APP_LED_PIN,
                            on ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
}

/*
 * 函数名：app_led_init
 * 作用：把 P111 配成推挽输出、初值低（灭）。
 * 参数：无。
 * 返回：无。
 * 时序/边界：上电调用一次；必须在任何 LED 写之前。
 * 注意：用运行时 R_IOPORT_PinCfg（与已验证固件一致），不依赖 RASC 的引脚配置。
 */
static void app_led_init(void)
{
    (void)R_IOPORT_PinCfg(&g_ioport_ctrl, APP_LED_PIN,
                          (uint32_t)IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                          (uint32_t)IOPORT_CFG_PORT_OUTPUT_LOW);
}

/*
 * 函数名：app_led_boot_blink
 * 作用：上电快闪 3 下，表示"固件确实在跑"（与串口能否通信无关）。
 * 参数：无。
 * 返回：无。
 * 时序/边界：阻塞约 0.72 s（3 × 240 ms），只在上电初始化阶段调用。
 * 注意：串口异常时这是最直接的自检证据；看到快闪 3 下再排串口问题。
 */
static void app_led_boot_blink(void)
{
    uint32_t i;

    for (i = 0U; i < 3U; i++)
    {
        app_led_write(true);
        R_BSP_SoftwareDelay(120U, BSP_DELAY_UNITS_MILLISECONDS);
        app_led_write(false);
        R_BSP_SoftwareDelay(120U, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

/*
 * 函数名：app_fatal_led_forever
 * 作用：致命错误（UART/时基初始化失败）时用 10 Hz 快闪报警，不返回。
 * 参数：无。
 * 返回：不返回（死循环）。
 * 时序/边界：进入后不再有串口/采样功能，只能复位。
 * 注意：这是"程序没跑起来"与"程序跑了但串口坏"的最后区分手段。
 */
static void app_fatal_led_forever(void)
{
    app_led_init();
    while (1)
    {
        app_led_write(true);
        R_BSP_SoftwareDelay(80U, BSP_DELAY_UNITS_MILLISECONDS);
        app_led_write(false);
        R_BSP_SoftwareDelay(80U, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

/* ---------- 阀/继电器（P104 → SRD-05VDC-SL-C → 12V 二位三通阀） ---------- */
/*
 * 函数名：app_valve_init
 * 作用：fail-safe 第一步——把 P104 配成输出并锁在"继电器释放（排空）"电平，
 *       由 actuator_valve 模块完成电平映射与回读自检。
 * 参数：无。
 * 返回：无（结果看 valve_get_out()->initialized / pin_fault）。
 * 时序/边界：必须是 app_main 的第一条外设操作（早于 LED/UART/ADC）。
 * 注意：APP_USE_VALVE=0（假执行器）时也配置：保证接上继电器后复位/上电瞬间也是排空；
 *       硬件上仍需 P104 的 10 kΩ 下拉（低触发模块改上拉），软件不能替代。
 */
static void app_valve_init(void)
{
    valve_init(&g_valve, &g_app_valve_cfg);
}

/*
 * 函数名：app_valve_apply
 * 作用：把本 tick 的目标位置交给执行器：手动覆盖优先，否则跟随判别动作；然后跑状态机。
 * 参数：recover —— true=回收位 / false=排空位；now_ms —— 当前时间（驻留/超时计时）。
 * 返回：无。
 * 时序/边界：每个 50 ms tick 调用一次；内部只做寄存器读写，不阻塞。
 * 注意：真正的换向由 valve_task 在满足 min_dwell_ms 后执行；APP_USE_VALVE=0 时
 *       模块只锁排空、不换向，LED1 继续演示判别链路（实验 3 兼容）。
 */
static void app_valve_apply(bool recover, uint32_t now_ms)
{
    const char *p_reason;
    const valve_out_t *p_valve;
    valve_pos_t pos_before;

    if (APP_VALVE_FORCE_RECOVER == g_valve_mode)
    {
        p_reason = "MANUAL_RECOVER";
    }
    else if (APP_VALVE_FORCE_DRAIN == g_valve_mode)
    {
        p_reason = "MANUAL_DRAIN";
    }
    else
    {
        p_reason = "AUTO";
    }

    p_valve    = valve_get_out(&g_valve);
    pos_before = (NULL != p_valve) ? p_valve->position : VALVE_POS_DRAIN;

    valve_request(&g_valve, (recover ? VALVE_POS_RECOVER : VALVE_POS_DRAIN), now_ms, p_reason);
    valve_task(&g_valve, now_ms);

    /* 物理换向 → 通知工况机开 400 ms 屏蔽窗（方案 §6.4 / S6）。
     * 注意：屏蔽只冻结判别，不撤销已生效的许可（否则会形成换向自激，见 validity.h）。 */
    p_valve = valve_get_out(&g_valve);
    if ((NULL != p_valve) && (p_valve->position != pos_before))
    {
        decision_note_valve_switch(&g_dec, now_ms);
    }
}

/*
 * 函数名：app_valve_log
 * 作用：把执行器"实际换向/故障/连续吸合超时"打成事件行（每行以 "# " 开头）。
 * 参数：now_ms —— 当前时间。
 * 返回：无。
 * 时序/边界：在 app_valve_apply 之后、每个 tick 调用一次；只在边沿打印，稳态不刷屏。
 * 注意：CSV 17 列协议保持不变（正常 AUTO 模式下 action 列就是阀位置）；手动 V1/V0
 *       与判别结果不一致时，以本事件行为准——这是数据留痕里阀动作的权威来源。
 */
static void app_valve_log(uint32_t now_ms)
{
    const valve_out_t *p_out = valve_get_out(&g_valve);

    if (NULL == p_out)
    {
        return;
    }

    if (p_out->position != g_prev_valve_pos)
    {
        printf("# VALVE t=%lu pos=%s relay=%u count=%lu reason=%s\r\n",
               (unsigned long)now_ms,
               valve_pos_str(p_out->position),
               (unsigned)(p_out->relay_on ? 1U : 0U),
               (unsigned long)p_out->switch_count,
               p_out->switch_reason);
        g_prev_valve_pos = p_out->position;
    }

    if (p_out->pin_fault && (!g_prev_valve_fault))
    {
        printf("# WARN: valve pin fault (write/readback failed %lu times) -> check P104/relay\r\n",
               (unsigned long)p_out->fault_count);
    }
    g_prev_valve_fault = p_out->pin_fault;

    if (p_out->max_on_blocked && (!g_prev_valve_blocked))
    {
        printf("# WARN: valve ON > %lums -> forced DRAIN, cooldown %lums\r\n",
               (unsigned long)APP_VALVE_MAX_ON_MS,
               (unsigned long)APP_VALVE_COOLDOWN_MS);
    }
    g_prev_valve_blocked = p_out->max_on_blocked;
}


/*
 * 函数名：app_valve_recover_requested
 * 作用：决定本 tick 的阀目标：手动覆盖优先，否则跟随判别动作。
 * 参数：p_dec —— 判别输出（可为 NULL）。
 * 返回：true=回收 / false=排空。
 * 时序/边界：纯函数；VA 回自动后立刻跟随 decision。
 * 注意：安全侧——拿不到判别输出时返回排空。
 */
static bool app_valve_recover_requested(const decision_out_t *p_dec)
{
    if (APP_VALVE_FORCE_RECOVER == g_valve_mode)
    {
        return true;
    }
    if (APP_VALVE_FORCE_DRAIN == g_valve_mode)
    {
        return false;
    }
    if (NULL == p_dec)
    {
        return false;
    }
    return (DEC_ACTION_RECOVER == p_dec->action);
}

/*
 * 函数名：app_led_task
 * 作用：按状态刷 LED 图案：WAIT 1 Hz 闪 / FLUSH 5 Hz 闪 / CLEAR 常亮 /
 *       UNCERTAIN/SOAP 灭；手动 V1/V0 时直接跟随手动命令。
 * 参数：p_dec —— 判别输出；now_ms —— 当前时间（时基 ms）。
 * 返回：无。
 * 时序/边界：每 50 ms 调用一次；闪烁用 now_ms 取模，不占定时器。
 * 注意：约定"回收=亮、排空=灭"；WAIT/FLUSH 闪是为了区分"在等"与"强制排空段"。
 */
static void app_led_task(const decision_out_t *p_dec, uint32_t now_ms)
{
    bool on;

    if (APP_VALVE_FORCE_RECOVER == g_valve_mode)
    {
        on = true;    /* 手动强制回收：常亮，便于空载看继电器 */
    }
    else if (APP_VALVE_FORCE_DRAIN == g_valve_mode)
    {
        on = false;
    }
    else if (NULL == p_dec)
    {
        on = false;   /* 安全侧：没有判别结果就当排空 */
    }
    else
    {
        switch (p_dec->state)
        {
            case DEC_STATE_WAIT:
                on = ((now_ms % 1000U) < 500U);   /* 1 Hz：亮 0.5 s / 灭 0.5 s */
                break;

            case DEC_STATE_FLUSH:
                on = ((now_ms % 200U) < 100U);    /* 5 Hz：强制排空段 */
                break;

            case DEC_STATE_CLEAR:
                on = true;                        /* 可回收：常亮 */
                break;

            case DEC_STATE_UNCERTAIN:
            case DEC_STATE_SOAP:
            default:
                on = false;                       /* 排空：灭 */
                break;
        }
    }

    app_led_write(on);
}

/* ---------- 基线 / 标记 ---------- */
/*
 * 函数名：app_baseline_begin
 * 作用：开始一次基线采集（上电自动 / 命令 B / K2 长按），并打印事件行。
 * 参数：now_ms —— 当前时间（日志时间戳）；p_src —— 触发来源字符串
 *       （"UART"/"KEY"，上电传 NULL 表示不需要额外提示行）。
 * 返回：无。
 * 时序/边界：pipeline 会连续采 baseline_n 个 tick 完成（默认 20 × 50 ms = 1 s），
 *             期间 baseline_valid=false → 判别层安全侧 L1 排空。
 * 注意：探头必须在水中！空气里 TDS 是开路伪值（约 1.6 V），采出来的基线是错的。
 */
static void app_baseline_begin(uint32_t now_ms, const char *p_src)
{
    /*
     * 安全闸门（v2）：没有干态指纹就拒绝采湿态基线。
     * 否则用户若先 W、且 W 时电极恰好露空，会把"空气 58 mV"当成 EC0 存下来，
     * 之后空腔的 g≡1、ΔEC≡0 → 又回到"回收空气"的旧故障（正是 S0 的成因）。
     * 顺序必须是 D（空腔）→ W（水中），这一点在方案 §8.1 里是硬约束。
     */
    if (!g_dec.gate.dry_valid)
    {
        printf("# ERR: dry fingerprint missing -> empty the chamber and send D first\r\n");
        return;
    }

    if (g_dry_capture.active)
    {
        /* W 与 D 不能同时进行：D 采的是"空腔"，W 要求"电极浸没"，样本会互相污染 */
        printf("# ERR: dry capture in progress (%u/%u) -> wait or send D again to restart\r\n",
               (unsigned)g_dry_capture.n, (unsigned)APP_DRY_CAPTURE_N);
        return;
    }

    pipeline_baseline_begin(&g_pipe, now_ms);
    g_base_result = APP_BASE_NONE;
    printf("# BASELINE start n=%u (keep electrode immersed & still)\r\n",
           (unsigned)g_pipe.cfg.baseline_n);
    if (NULL != p_src)
    {
        printf("# BASELINE trigger=%s\r\n", p_src);
    }
}

/*
 * 函数名：app_baseline_validate
 * 作用：W 采集完成后的业务校验（pipeline 只做量程/CV，这里补"必须明显高于干态"）。
 * 参数：now_ms —— 当前时间；p_out —— 流水线输出（baseline_updated 那一 tick）。
 * 返回：无。
 * 时序/边界：由 app_tick 在 p_out->baseline_updated 为 true 时调用一次。
 * 注意：校验不过就调用 pipeline_invalidate_baseline 真正作废（不能只在日志里说不行，
 *       否则 decision 会继续用这条坏基线判水质）。
 */
static void app_baseline_validate(uint32_t now_ms, const pipeline_out_t *p_out)
{
    char ec0[20];
    char dry[20];
    char ratio[20];

    if (NULL == p_out)
    {
        return;
    }

    (void)now_ms;

    if (!p_out->baseline_valid)
    {
        /* 量程/CV 校验已经失败（pipeline 把原因写进 note） */
        g_base_result = APP_BASE_REJECT;
        printf("# BASELINE REJECT note=%s (check electrode immersed / water still)\r\n", p_out->note);
        return;
    }

    if (g_dec.gate.dry_valid)
    {
        if (p_out->ec0_mv25 < (g_dec.gate.ec_dry_mv + g_dec.gate.cfg.ec_wet_delta_mv))
        {
            pipeline_invalidate_baseline(&g_pipe, "BASE_LOW");
            g_base_result = APP_BASE_REJECT;
            app_fmt_x10(ec0, sizeof(ec0), p_out->ec0_mv25);
            app_fmt_x10(dry, sizeof(dry), g_dec.gate.ec_dry_mv);
            app_fmt_x10(ratio, sizeof(ratio), g_dec.gate.cfg.ec_wet_delta_mv);
            printf("# BASELINE REJECT note=BASE_LOW ec0=%smV dry=%smV (need +%smV)\r\n",
                   ec0, dry, ratio);
            printf("#   hint: water did not cover the electrode, or D was taken while wet\r\n");
            return;
        }
    }

    g_base_result = APP_BASE_OK;
    app_print_baseline_ok(p_out);
}

/* ---------- 干态指纹采集（命令 D） ---------- */
/*
 * 函数名：app_dry_begin
 * 作用：开始一次干态指纹采集（腔内必须无液体），完成后写入 gate。
 * 参数：now_ms —— 当前时间。
 * 返回：无。
 * 时序/边界：非阻塞，采 APP_DRY_CAPTURE_N 个 50 ms 样本（默认 60 s）；
 *             期间工况保持原状（通常为 FAULT/DRY），阀恒排空。
 * 注意：采集中途收到 D 会重新开始；采样跳过 ADC 超时/贴轨的 tick。
 */
static void app_dry_begin(uint32_t now_ms, uint16_t target_n, bool auto_cap)
{
    const pipeline_out_t *p_now = pipeline_get_out(&g_pipe);
    float ec_now = (NULL != p_now) ? p_now->tds_mv25 : 0.0f;
    bool  chamber_wet = false;

    /* 空腔校验：D 的前提是"腔里没有液体"。
     * 已经有干态指纹时，用"已浸没"或"EC ≥ 3×EC_DRY"判断；
     * 首次标定时没有参考，只能用绝对上限（EC_DRY_MAX，默认 200 mV）。 */
    if (g_dec.gate.dry_valid)
    {
        chamber_wet = g_dec.gate.immersed
                   || (ec_now >= (g_dec.gate.ec_dry_mv
                                  + g_dec.gate.cfg.ec_wet_delta_mv));
    }
    else
    {
        chamber_wet = (ec_now > g_dec.gate.cfg.ec_dry_max_mv);
    }

    if (chamber_wet)
    {
        char s_ec[20];

        if (auto_cap)
        {
            g_ac_dry_blocked = true;   /* 自检：腔里有水/电极湿，不能采干态 */
        }
        app_fmt_x10(s_ec, sizeof(s_ec), ec_now);
        printf("# ERR: chamber is NOT empty (ec=%smV) -> drain/dry it, then send D\r\n", s_ec);
        return;
    }

    if (auto_cap)
    {
        g_ac_dry_blocked = false;
    }

    /*
     * 只中止"正在进行的 W 采集"，不主动作废已生效的水中基线：
     * 采干态期间如果腔里还有水，负偏离会把它判成 TOO_CLEAN→L1→排空（安全），
     * 没必要因为一次误按 D 就把整机打到 FAULT、逼迫现场重做 W。
     */
    if (pipeline_baseline_capturing(&g_pipe))
    {
        pipeline_invalidate_baseline(&g_pipe, "ABORTED");
        printf("# NOTE: wet baseline capture aborted by dry calibration\r\n");
    }

    g_dry_capture.active   = true;
    g_dry_capture.auto_cap = auto_cap;
    g_dry_capture.n        = 0U;
    g_dry_capture.target_n = (target_n > 0U) ? target_n : (uint16_t)APP_DRY_CAPTURE_N;
    g_dry_capture.ec_sum   = 0.0f;
    g_dry_capture.ec_sumsq = 0.0f;
    g_dry_capture.ao_sum   = 0.0f;
    g_dry_capture.ao_sumsq = 0.0f;
    g_dry_capture.start_ms = now_ms;

    printf("# DRY start%s: chamber MUST be EMPTY, capture %lums (%u samples); do not pour water\r\n",
           auto_cap ? " (auto)" : "",
           (unsigned long)((uint32_t)g_dry_capture.target_n * APP_TICK_MS),
           (unsigned)g_dry_capture.target_n);
}

/*
 * 函数名：app_dry_task
 * 作用：干态指纹采集状态机（每个 tick 调一次）。
 * 参数：now_ms —— 当前时间；p_out —— 流水线输出。
 * 返回：无。
 * 时序/边界：采满即结算；均值/σ/CV 全部用"原始 mV25 样本"统计（EMA 前）。
 * 注意：校验三件事——均值落在 [1, ec_dry_max_mv]、CV ≤ 5%；任一不过就拒绝，
 *       不写指纹（宁可继续用旧指纹/继续 FAULT，也不要把湿态当干态）。
 */
static void app_dry_task(uint32_t now_ms, const pipeline_out_t *p_out)
{
    float ec;
    float ao;

    if (!g_dry_capture.active)
    {
        return;
    }

    /* 墙钟硬超时：传感器长期故障时样本数永远凑不满，不能让 active 一直挂着。
     * 2 倍采集时长（手动 120 s / 自检 20 s）足够覆盖偶发丢帧；超时直接放弃，不写指纹。 */
    if ((uint32_t)(now_ms - g_dry_capture.start_ms)
        >= ((uint32_t)g_dry_capture.target_n * APP_TICK_MS * 2U))
    {
        printf("# DRY ABORT: timeout at %lums (%u/%u samples)\r\n",
               (unsigned long)((uint32_t)g_dry_capture.target_n * APP_TICK_MS),
               (unsigned)g_dry_capture.n, (unsigned)g_dry_capture.target_n);
        g_dry_capture.active = false;
        return;
    }

    if ((NULL == p_out) || p_out->adc_timeout || p_out->tds_rail)
    {
        return;
    }
    if ((SENSOR_OK != p_out->tds_status) || (SENSOR_OK != p_out->ntu_status))
    {
        return;
    }

    ec = p_out->tds_mv25;
    ao = (float)p_out->ntu_ao_mv;

    g_dry_capture.ec_sum   += ec;
    g_dry_capture.ec_sumsq += ec * ec;
    g_dry_capture.ao_sum   += ao;
    g_dry_capture.ao_sumsq += ao * ao;
    g_dry_capture.n++;

    if (g_dry_capture.n < g_dry_capture.target_n)
    {
        if ((0U == (g_dry_capture.n % 200U)) && (g_dry_capture.n > 0U))
        {
            printf("# DRY progress n=%u/%u t=%lums\r\n",
                   (unsigned)g_dry_capture.n, (unsigned)g_dry_capture.target_n,
                   (unsigned long)(now_ms - g_dry_capture.start_ms));
        }
        return;
    }

    /* 结算 */
    {
        float n_f    = (float)g_dry_capture.n;
        float mean_e = g_dry_capture.ec_sum / n_f;
        float mean_a = g_dry_capture.ao_sum / n_f;
        float var_e  = 0.0f;
        float sigma_e;
        float cv_pct;
        bool  ok;

        g_dry_last_mean_mv = mean_e;

        if (g_dry_capture.n >= 2U)
        {
            var_e = (g_dry_capture.ec_sumsq - (n_f * mean_e * mean_e)) / (n_f - 1.0f);
        }
        if (var_e < 0.0f)
        {
            var_e = 0.0f;
        }
        sigma_e = sensor_sqrtf(var_e);
        cv_pct  = (mean_e > 0.5f) ? ((sigma_e * 100.0f) / mean_e) : 100.0f;

        ok = (mean_e >= 1.0f)
          && (mean_e <= g_dec.gate.cfg.ec_dry_max_mv)
          && (cv_pct <= APP_DRY_CV_MAX_PCT)
          && gate_set_dry_fingerprint(&g_dec.gate, mean_e, mean_a);

        if (ok)
        {
            char s_ec[20];
            char s_ao[20];
            char s_cv[20];

            app_fmt_x10(s_ec, sizeof(s_ec), mean_e);
            app_fmt_x10(s_ao, sizeof(s_ao), mean_a);
            app_fmt_x10(s_cv, sizeof(s_cv), cv_pct);
            printf("# DRY OK ec_dry=%smV ao_dry=%smV cv=%s%% n=%u -> pour water over electrode (autocal) or send W\r\n",
                   s_ec, s_ao, s_cv, (unsigned)g_dry_capture.n);

            /*
             * 新干态指纹会改变"浸没门限 = 3×EC_DRY"。若现场还存着一条旧的水中基线，
             * 而它已经低于新门限（例如残水时按了 D，把 EC_DRY 抬高了），
             * 继续留着会让 gate 直接判 FAULT 却不给原因 —— 这里就地作废并提示重做 W。
             */
            {
                const pipeline_out_t *p_now = pipeline_get_out(&g_pipe);

                if ((NULL != p_now) && p_now->baseline_valid
                    && (p_now->ec0_mv25 < (g_dec.gate.ec_dry_mv
                                           + g_dec.gate.cfg.ec_wet_delta_mv)))
                {
                    pipeline_invalidate_baseline(&g_pipe, "BASE_LOW");
                    printf("# WARN: existing wet baseline < 3x new dry fp -> invalidated, send W\r\n");
                }
            }
        }
        else
        {
            char s_ec[20];
            char s_cv[20];

            app_fmt_x10(s_ec, sizeof(s_ec), mean_e);
            app_fmt_x10(s_cv, sizeof(s_cv), cv_pct);
            printf("# DRY REJECT ec=%smV cv=%s%% (need ec<=%smV, cv<=%s%%, stable)\r\n",
                   s_ec, s_cv, "200", "5.0");
            printf("#   hint: empty the chamber and wait for drips/bubbles to settle\r\n");
        }

        g_dry_capture.active = false;
    }
}

/* ---------- 开机自检状态机（v2.1） ---------- */

/* 状态 → 静态字符串（串口日志/状态命令用）。 */
static const char *app_autocal_state_str(app_autocal_state_t state)
{
    switch (state)
    {
        case APP_AC_DRY:
            return "DRY";

        case APP_AC_WAIT:
            return "WAIT_WATER";

        case APP_AC_SETTLE:
            return "SETTLE";

        case APP_AC_WET:
            return "WET";

        case APP_AC_RETRY:
            return "RETRY";

        case APP_AC_DONE:
            return "DONE";

        case APP_AC_OFF:
        default:
            return "OFF";
    }
}

/* 切换自检阶段并打一行日志（只在状态变化时调用）。 */
static void app_autocal_enter(app_autocal_state_t state, uint32_t now_ms, const char *p_why)
{
    g_ac_state    = state;
    g_ac_since_ms = now_ms;
    if (APP_AC_DONE == state)
    {
        g_ac_done_since_ms = now_ms;
    }

    printf("# AUTOCAL state=%s%s%s\r\n",
           app_autocal_state_str(state),
           (NULL != p_why) ? " reason=" : "",
           (NULL != p_why) ? p_why : "");
}

/* 给 OLED/状态命令用的自检文本（不在自检时为空串）。 */
void app_autocal_text(char *p_buf, uint32_t buf_size)
{
    if ((NULL == p_buf) || (0U == buf_size))
    {
        return;
    }
    p_buf[0] = '\0';

    if (APP_AC_DONE == g_ac_state)
    {
        if (g_ac_need_manual)
        {
            /* 8x16 字体每行最多 16 个字符（128/8），状态串必须短 */
            (void)snprintf(p_buf, (size_t)buf_size, "Auto fail");
        }
        return;
    }

    switch (g_ac_state)
    {
        case APP_AC_DRY:
            (void)snprintf(p_buf, (size_t)buf_size, "Auto dry %u/%u",
                           (unsigned)g_dry_capture.n, (unsigned)g_dry_capture.target_n);
            break;

        case APP_AC_WAIT:
            (void)snprintf(p_buf, (size_t)buf_size, "Auto wait water");
            break;

        case APP_AC_SETTLE:
            (void)snprintf(p_buf, (size_t)buf_size, "Auto water ok");
            break;

        case APP_AC_WET:
            (void)snprintf(p_buf, (size_t)buf_size, "Auto calib");
            break;

        case APP_AC_RETRY:
            (void)snprintf(p_buf, (size_t)buf_size, "Auto retry %u",
                           (unsigned)g_ac_wet_tries);
            break;

        case APP_AC_OFF:
        case APP_AC_DONE:
        default:
            break;
    }
}

/*
 * 自检的第二行提示（6x8 字体，每行最多 21 个字符）。
 * 采用"首字母大写、其余小写"的写法（现场要求），代码/单位仍保持大写。
 */
void app_autocal_hint(char *p_buf, uint32_t buf_size)
{
    const char *p_hint = "";

    if ((NULL == p_buf) || (0U == buf_size))
    {
        return;
    }

    if ((APP_AC_DONE == g_ac_state) && (g_ac_need_manual))
    {
        p_hint = "Send W in clean water";
    }
    else
    {
        switch (g_ac_state)
        {
            case APP_AC_DRY:
                p_hint = "Keep chamber empty";
                break;

            case APP_AC_WAIT:
                p_hint = "Pour water over probe";
                break;

            case APP_AC_SETTLE:
                p_hint = "Water detected";
                break;

            case APP_AC_WET:
                p_hint = "Keep still";
                break;

            case APP_AC_RETRY:
                p_hint = "Retry soon";
                break;

            case APP_AC_OFF:
            case APP_AC_DONE:
            default:
                break;
        }
    }

    (void)snprintf(p_buf, (size_t)buf_size, "%s", p_hint);
}

/* 是否正在自检（DONE/OFF 都算结束）。 */
bool app_autocal_active(void)
{
    return (APP_AC_OFF != g_ac_state) && (APP_AC_DONE != g_ac_state);
}

/* 手动 D/W 接管：中止自检，并取消自检启动的干态采集（不动作业中基线）。 */
static void app_autocal_cancel(const char *p_why)
{
    if ((APP_AC_OFF == g_ac_state) || (APP_AC_DONE == g_ac_state))
    {
        return;
    }

    if (g_dry_capture.active && g_dry_capture.auto_cap)
    {
        g_dry_capture.active = false;
    }

    printf("# AUTOCAL cancelled (%s)\r\n", (NULL != p_why) ? p_why : "manual");
    g_ac_state = APP_AC_OFF;
}

/*
 * 启动自检：先落一组默认干态指纹，再按"当前是否有水"决定从干态还是湿态开始。
 * 调用时机：上电第一个 tick 的 decision_update 之后（此时 gate.immersed 才是最新的）。
 */
static void app_autocal_start(uint32_t now_ms)
{
#if (1 == APP_AUTOCAL_ENABLE)
    g_ac_dry_tries     = 0U;
    g_ac_wet_tries     = 0U;
    g_ac_dry_blocked   = false;
    g_ac_fail_retry    = false;
    g_ac_need_manual   = false;
    g_dry_last_mean_mv = 0.0f;
    g_base_result      = APP_BASE_NONE;

    /*
     * 只有"本次上电已经实测过干态"才允许因为"已浸没"直接去采湿态。
     * 若干态还是 DEFAULT（60 mV），上电时 EC≥180 可能只是电极被淋湿/腔里有残水，
     * 直接采湿态会把一层水膜当成水中基线 → 空腔 g≡1 → 回收空气。
     * 所以 DEFAULT 必须先走干态采集；腔里真有水时干态采集会被拒绝，
     * 随后进入 SETTLE 用"EC≥400 + 浊度 AO 下降"确认水体，证据不足则 DONE(water_unconfirmed)，
     * 由人发 W（或等腔体干燥后自动低频重试）。
     */
    if ((g_dec.gate.immersed) && (DRY_SOURCE_MEASURED == gate_dry_source(&g_dec.gate)))
    {
        app_autocal_enter(APP_AC_SETTLE, now_ms, "water_present");
    }
    else
    {
        app_autocal_enter(APP_AC_DRY, now_ms, "empty");
        app_dry_begin(now_ms, (uint16_t)APP_AUTOCAL_DRY_N, true);
    }
#else
    (void)now_ms;
#endif
}

/*
 * 自检主状态机（每 tick 一次，必须在 decision_update 之后调用）。
 * 全程没有合法基线时 decision 仍是 FAULT/DRY → 阀恒排空，不存在"自动回收空气"。
 */
static void app_autocal_task(uint32_t now_ms, const pipeline_out_t *p_out)
{
    if (APP_AC_OFF == g_ac_state)
    {
        return;
    }

    if (APP_AC_DONE == g_ac_state)
    {
        /* 手动 W 成功后要清掉"需要人工"提示，否则 OLED 会一直显示 AUTO FAIL */
        if (g_dec.gate.baseline_ok)
        {
            g_ac_need_manual = false;
            g_ac_fail_retry  = false;
            return;
        }

        /* base_fail / need_manual 之后每 5 min 低频重试：现场不接串口也有恢复路径 */
        if ((g_ac_fail_retry) && (NULL != p_out) && (!p_out->adc_timeout)
            && ((uint32_t)(now_ms - g_ac_done_since_ms) >= APP_AUTOCAL_FAIL_RETRY_MS))
        {
            g_ac_fail_retry = false;
            g_ac_wet_tries  = 0U;
            g_ac_dry_tries  = 0U;
            g_base_result   = APP_BASE_NONE;
            g_ac_need_manual = false;

            if ((g_dec.gate.immersed) && (DRY_SOURCE_MEASURED == gate_dry_source(&g_dec.gate)))
            {
                app_autocal_enter(APP_AC_SETTLE, now_ms, "periodic_retry");
            }
            else if (!g_dec.gate.immersed)
            {
                g_ac_dry_blocked = false;
                app_autocal_enter(APP_AC_DRY, now_ms, "periodic_dry");
                app_dry_begin(now_ms, (uint16_t)APP_AUTOCAL_DRY_N, true);
            }
            else
            {
                /* 有水但干态没实测：不能采湿态，等腔体状态变化后再低频重试 */
                g_ac_fail_retry    = true;
                g_ac_done_since_ms = now_ms;
            }
        }
        return;
    }

    if (NULL == p_out)
    {
        return;
    }

    /* 墙钟超时先于"数据是否有效"处理：ADC 永久故障时不能卡在 WET/RETRY 且没有日志 */
    if ((APP_AC_WET == g_ac_state)
        && ((uint32_t)(now_ms - g_ac_since_ms) >= APP_AUTOCAL_WET_TIMEOUT))
    {
        pipeline_invalidate_baseline(&g_pipe, "AUTO_ABORT");
        g_ac_wet_tries++;
        if (g_ac_wet_tries >= APP_AUTOCAL_WET_TRIES)
        {
            g_ac_fail_retry  = true;
            g_ac_need_manual = true;
            app_autocal_enter(APP_AC_DONE, now_ms, "wet_timeout");
        }
        else
        {
            app_autocal_enter(APP_AC_RETRY, now_ms, "wet_timeout");
        }
        return;
    }

    if ((APP_AC_RETRY == g_ac_state)
        && ((uint32_t)(now_ms - g_ac_since_ms) >= APP_AUTOCAL_RETRY_MS))
    {
        if (!g_dec.gate.immersed)
        {
            app_autocal_enter(APP_AC_WAIT, now_ms, "retry_no_water");
        }
        else
        {
            g_base_result = APP_BASE_NONE;
            app_baseline_begin(now_ms, "AUTO");
            if (pipeline_baseline_capturing(&g_pipe))
            {
                app_autocal_enter(APP_AC_WET, now_ms, "retry");
            }
            else
            {
                g_ac_fail_retry  = true;
                g_ac_need_manual = true;
                app_autocal_enter(APP_AC_DONE, now_ms, "retry_refused");
            }
        }
        return;
    }

    if (p_out->adc_timeout)
    {
        return;
    }

    /* 任何途径建立了"校验通过 + 工况机认可"的基线（手动/本次自检）→ 自检结束 */
    if ((APP_BASE_OK == g_base_result) && (g_dec.gate.baseline_ok))
    {
        g_ac_need_manual = false;
        g_ac_fail_retry  = false;
        app_autocal_enter(APP_AC_DONE, now_ms, "baseline_ok");
        return;
    }

    switch (g_ac_state)
    {
        case APP_AC_DRY:
            if (g_dry_capture.active)
            {
                return;   /* 还在采 */
            }

            if (DRY_SOURCE_MEASURED == gate_dry_source(&g_dec.gate))
            {
                app_autocal_enter(APP_AC_WAIT, now_ms, "dry_ok");
            }
            else if (g_ac_dry_tries < 1U)
            {
                g_ac_dry_tries++;
                app_autocal_enter(APP_AC_DRY, now_ms, "dry_retry");
                app_dry_begin(now_ms, (uint16_t)APP_AUTOCAL_DRY_N, true);
            }
            else if ((!g_ac_dry_blocked)
                     && (g_dry_last_mean_mv < g_dec.gate.cfg.ec_dry_max_mv))
            {
                /* 干态采集失败但读数一直是"干"的（只是 CV 抖）：用默认值继续等水 */
                app_autocal_enter(APP_AC_WAIT, now_ms, "dry_default");
            }
            else
            {
                /* 腔里有水/电极湿：不能自己定干态。
                 * 但仍给一次"有水"的确认机会：SETTLE 里会用 EC 差值 + 浊度 AO 两道证据
                 * 判断是不是真水体；过不了就落到 DONE(need_manual)。 */
                app_autocal_enter(APP_AC_SETTLE, now_ms, "chamber_wet_confirm");
            }
            break;

        case APP_AC_WAIT:
            if (g_dec.gate.immersed)
            {
                app_autocal_enter(APP_AC_SETTLE, now_ms, "water");
            }
            break;

        case APP_AC_SETTLE:
        {
            bool allow_wet = true;

            if (!g_dec.gate.immersed)
            {
                app_autocal_enter(APP_AC_WAIT, now_ms, "water_lost");
            }
            else if ((uint32_t)(now_ms - g_ac_since_ms) >= APP_AUTOCAL_WATER_MS)
            {
                /* 两道证据（无论干态是否实测都要过）：
                 *   ① EC 比干态高 ec_wet_delta（默认 80 mV）；
                 *   ② 浊度 AO 比空腔高 ao_water_delta（本装置水 AO 高于空气约 400 mV，
                 *      与"空气满量程"的常见假设相反，故用正差值）。
                 * 只有一层水膜的电极通常过不了第 ② 条，避免把水膜当成水体。 */
                allow_wet = (p_out->tds_mv25 >= (g_dec.gate.ec_dry_mv
                                                 + g_dec.gate.cfg.ec_wet_delta_mv))
                         && ((float)p_out->ntu_ao_mv >= (g_dec.gate.ao_dry_mv
                                                         + g_dec.gate.cfg.ao_water_delta_mv));

                if (!allow_wet)
                {
                    /* 没有足够证据证明"腔里是水"：不允许自动采湿态（否则可能采到水膜） */
                    g_ac_fail_retry  = true;
                    g_ac_need_manual = true;
                    app_autocal_enter(APP_AC_DONE, now_ms, "water_unconfirmed");
                }
                else
                {
                    app_baseline_begin(now_ms, "AUTO");
                    if (pipeline_baseline_capturing(&g_pipe))
                    {
                        app_autocal_enter(APP_AC_WET, now_ms, "calib");
                    }
                    else
                    {
                        app_autocal_enter(APP_AC_RETRY, now_ms, "start_refused");
                    }
                }
            }
            break;
        }

        case APP_AC_WET:
            if (APP_BASE_OK == g_base_result)
            {
                if (g_dec.gate.baseline_ok)
                {
                    g_ac_need_manual = false;
                    g_ac_fail_retry  = false;
                    app_autocal_enter(APP_AC_DONE, now_ms, "base_ok");
                }
                else
                {
                    /* pipeline 认为基线有效，但工况机仍拒（量程/干态差值不满足）：
                     * 不能报"自检成功"，否则会停在待机却不给原因。 */
                    g_base_result = APP_BASE_NONE;
                    app_autocal_enter(APP_AC_RETRY, now_ms, "gate_reject");
                }
            }
            else if (APP_BASE_REJECT == g_base_result)
            {
                g_ac_wet_tries++;
                if (g_ac_wet_tries >= APP_AUTOCAL_WET_TRIES)
                {
                    g_ac_fail_retry  = true;
                    g_ac_need_manual = true;
                    app_autocal_enter(APP_AC_DONE, now_ms, "base_fail");
                }
                else
                {
                    app_autocal_enter(APP_AC_RETRY, now_ms, "base_retry");
                }
            }
            else
            {
                /* 正在采（墙钟超时已在函数开头处理） */
            }
            break;

        case APP_AC_RETRY:
            /* 到时判定已在函数开头处理 */
            break;

        default:
            break;
    }
}

/* 干态指纹来源 → 短字符串（P/G 打印用）。 */
static const char *app_dry_source_str(dry_source_t source)
{
    switch (source)
    {
        case DRY_SOURCE_DEFAULT:
            return "DEFAULT";

        case DRY_SOURCE_MEASURED:
            return "MEASURED";

        case DRY_SOURCE_NONE:
        default:
            return "NONE";
    }
}

/*
 * 函数名：app_print_gate
 * 作用：命令 G：打印工况机全貌（位图、regime、浸没/稳定/屏蔽、干态指纹）。
 * 参数：now_ms —— 当前时间。
 * 返回：无。
 * 时序/边界：同步打印约 4 行；不影响 tick。
 */
static void app_print_gate(uint32_t now_ms)
{
    const gate_t *p_gate = &g_dec.gate;
    char bits[48];
    char ec_dry[20];
    char ao_dry[20];
    char s_settle[20];
    char s_blank[20];
    char s_wet[20];
    char s_ecdelta[20];
    char s_aodelta[20];
    char s_timeout[20];

    gate_bits_str(p_gate, bits, (uint32_t)sizeof(bits));
    app_fmt_x10(ec_dry, sizeof(ec_dry), p_gate->ec_dry_mv);
    app_fmt_x10(ao_dry, sizeof(ao_dry), p_gate->ao_dry_mv);
    app_fmt_x10(s_settle, sizeof(s_settle), (float)p_gate->cfg.settle_ms / 1000.0f);
    app_fmt_x10(s_blank, sizeof(s_blank), (float)p_gate->cfg.blank_ms / 1000.0f);
    app_fmt_x10(s_wet, sizeof(s_wet), p_gate->cfg.wet_ratio);
    app_fmt_x10(s_ecdelta, sizeof(s_ecdelta), p_gate->cfg.ec_wet_delta_mv);
    app_fmt_x10(s_aodelta, sizeof(s_aodelta), p_gate->cfg.ao_water_delta_mv);
    app_fmt_x10(s_timeout, sizeof(s_timeout), (float)p_gate->cfg.settle_timeout_ms / 1000.0f);

    printf("# ---------- GATE t=%lums ----------\r\n", (unsigned long)now_ms);
    printf("# gate: valid=0x%02X(%s) regime=%s reason=%s\r\n",
           (unsigned)p_gate->bits, bits, gate_regime_str(p_gate->regime),
           gate_reason_str(p_gate));
    printf("# gate: wet=%u stable=%u blanking=%u | dry_valid=%u src=%s ec_dry=%smV ao_dry=%smV\r\n",
           (unsigned)(p_gate->immersed ? 1U : 0U),
           (unsigned)(p_gate->stable ? 1U : 0U),
           (unsigned)(p_gate->blanking ? 1U : 0U),
           (unsigned)(p_gate->dry_valid ? 1U : 0U),
           app_dry_source_str(p_gate->dry_source),
           ec_dry, ao_dry);
    printf("# gate: wet_ratio=%s ec_delta>=%smV ao_delta>=%smV settle=%s/%ss blank=%ss\r\n",
           s_wet, s_ecdelta, s_aodelta, s_settle, s_timeout, s_blank);

    {
        char ac[24];

        app_autocal_text(ac, (uint32_t)sizeof(ac));
        if ('\0' != ac[0])
        {
            printf("# autocal: %s\r\n", ac);
        }
    }
}

/*
 * 函数名：app_note_code
 * 作用：把工况翻译成 CSV 的 note 列（方案 §11 的工况码），尽量保持 17 列不变。
 * 参数：p_out —— 流水线输出；p_dec —— 判别输出。
 * 返回：静态字符串：OK/DRY/SET/BLK/LOCK/NOBASE/SENS（MEASURE 且正常时沿用 pipeline note）。
 * 时序/边界：纯函数，每 200 ms 调一次。
 * 注意：工况码优先于传感器码，因为 CSV 的 note 列现在回答"为什么没回收"；
 *       传感器细节（TDS_RAIL 等）仍可在 # GATE 行与 P 命令里看到。
 */
static const char *app_note_code(const pipeline_out_t *p_out, const decision_out_t *p_dec)
{
    if (NULL == p_dec)
    {
        return "NO_DEC";
    }

    switch (p_dec->regime)
    {
        case REGIME_FAULT:
            return (0U != (p_dec->valid_bits & (uint16_t)VAL_SENSOR_OK)) ? "NOBASE" : "SENS";

        case REGIME_DRY:
            return "DRY";

        case REGIME_SETTLE:
            return "SET";

        case REGIME_LOCKOUT:
            return "LOCK";

        case REGIME_MEASURE:
        default:
            break;
    }

    if (p_dec->blanking)
    {
        return "BLK";
    }
    return (NULL != p_out) ? p_out->note : "OK";
}

/*
 * 函数名：app_print_mark
 * 作用：打印一条标记行（串口 M），供 PC 端把连续数据分段。
 * 参数：p_src —— 来源（"UART"/"KEY"）；now_ms —— 当前时间。
 * 返回：无。
 * 时序/边界：任意时刻可调用（主循环上下文）。
 * 注意：格式 "# MARK t=... src=..." 与 00 文档事件行约定一致。
 */
static void app_print_mark(const char *p_src, uint32_t now_ms)
{
    printf("# MARK t=%lu src=%s\r\n", (unsigned long)now_ms, p_src);
}

/* ---------- 按键 K2 ---------- */
/*
 * 函数名：app_key_task
 * 作用：扫描板载 K2：短按（释放时）切换 OLED 页（P1→P2→P3→P1）；
 *       按住 ≥1.5 s 触发基线校准（只触发一次，长按优先）。
 * 参数：now_ms —— 当前时间（时基 ms）。
 * 返回：无。
 * 时序/边界：每 50 ms 调一次；软件消抖靠"边沿 + 长按计时"，板上已有 4.7 kΩ 上拉。
 * 注意：长按在"按住期间"就触发（不必等松开），松开时不再切页；
 *       短按在松手时判定，避免长按/短按互相干扰；MARK 只由串口 M 发送。
 */
static void app_key_task(uint32_t now_ms)
{
    bsp_io_level_t level = BSP_IO_LEVEL_HIGH;
    bool down;

    (void)R_IOPORT_PinRead(&g_ioport_ctrl, APP_KEY_PIN, &level);
    down = (BSP_IO_LEVEL_LOW == level);   /* 按下为低 */

    if (down && (!g_key_prev_down))
    {
        g_key_press_ms   = now_ms;
        g_key_long_fired = false;
    }

    if (down && (!g_key_long_fired) &&
        ((uint32_t)(now_ms - g_key_press_ms) >= APP_KEY_LONG_MS))
    {
        g_key_long_fired = true;
        printf("# KEY: long press -> baseline recalibration\r\n");
        app_autocal_cancel("manual KEY");   /* 与 W 命令一致：手动接管，先停自检 */
        app_baseline_begin(now_ms, "KEY");
    }

    if ((!down) && g_key_prev_down)
    {
        if (!g_key_long_fired)
        {
            /* 短按：循环切 OLED 页（P1 水质 / P2 执行器 / P3 详情）；MARK 改由串口 M 发送 */
            uint8_t page = display_page_next();

            if (page > 0U)
            {
                printf("# DISPLAY page=%u/%u (K2)\r\n", (unsigned)page,
                       (unsigned)DISPLAY_PAGE_COUNT);
            }
            else
            {
                printf("# DISPLAY not usable (OLED lost)\r\n");
            }
        }
        g_key_long_fired = false;
    }

    g_key_prev_down = down;
}

/* ---------- 打印：帮助 / 基线完成 / 状态 ---------- */
/*
 * 函数名：app_print_help
 * 作用：打印全部串口命令的简要帮助（纯 ASCII，串口助手不乱码）。
 * 参数：无。
 * 返回：无。
 * 时序/边界：上电打印一次，命令 H/? 再打印；任何命令都必须在这里有说明。
 * 注意：命令大小写不敏感；发送新行（\r 或 \n）结束一行。
 */
static void app_print_help(void)
{
    printf("# ---------- commands (115200 8N1, newline, case-insensitive) ----------\r\n");
    printf("#  H  or  ?      print this help\r\n");
    printf("#  M             mark now (# MARK t=... src=UART)\r\n");
    printf("#  auto           # TIP every 2s: regime/state/permit + EC/g/dEC/T + next action\r\n");
    printf("#  self-check    automatic at boot: dry -> wait water -> wet baseline (no input)\r\n");
    printf("#  D             manual: capture DRY fingerprint (chamber EMPTY, 60 s)\r\n");
    printf("#  W             manual: capture WET baseline (electrode IMMERSED, 2 s; D first)\r\n");
    printf("#  G             print gate: valid bits / regime / dry+wet fingerprints\r\n");
    printf("#  Z             clear the wet baseline (regime -> fault/dry, drain)\r\n");
    printf("#  R / E         event markers for logs only (v2: no control effect)\r\n");
    printf("#  B             alias of W (legacy baseline command)\r\n");
    printf("#  T24.5         manual water temp 0..60 C (used when DS18B20 is absent)\r\n");
    printf("#  C             re-detect DS18B20\r\n");
    printf("#  P             print status / gate / thresholds / parameters\r\n");
    printf("#  V1 / V0 / VA  manual valve: RECOVER (relay ON) / DRAIN (relay OFF) / auto\r\n");
    printf("#  K2 short press: next OLED page (P1 water / P2 exec / P3 info)\r\n");
    printf("#  K2 long press >=1.5s: re-capture wet baseline (MARK is serial 'M' only)\r\n");
    printf("# ---------------------------------------------------------------------\r\n");
}

/*
 * 函数名：app_print_baseline_ok
 * 作用：基线采集完成时打印一次 "# BASELINE OK ..."（本底、σ），供 PC 复核。
 * 参数：p_out —— 流水线输出（含 ec0_mv25/ntu0_ao_mv/σ）。
 * 返回：无。
 * 时序/边界：由 pipeline_out_t.baseline_updated 触发，每个基线周期只打印一次。
 * 注意：示例数值只是格式示意；本底数值以现场实测为准（不能拿文档数字当结论）。
 */
static void app_print_baseline_ok(const pipeline_out_t *p_out)
{
    char ec0[20];
    char sig_ec[20];
    char sig_ntu[20];
    char dry[20];
    char ratio[20];

    app_fmt_x10(ec0, sizeof(ec0), p_out->ec0_mv25);
    app_fmt_x10(sig_ec, sizeof(sig_ec), p_out->ec_sigma_mv);
    app_fmt_x10(sig_ntu, sizeof(sig_ntu), p_out->ntu_sigma_mv);

    if (g_dec.gate.dry_valid)
    {
        app_fmt_x10(dry, sizeof(dry), g_dec.gate.ec_dry_mv);
        app_fmt_x1000(ratio, sizeof(ratio), p_out->ec0_mv25 / g_dec.gate.ec_dry_mv);
    }
    else
    {
        dry[0] = '-';
        dry[1] = '\0';
        ratio[0] = '-';
        ratio[1] = '\0';
    }

    printf("# BASELINE OK ec0=%smV ntu0=%lumV sigma_ec=%s sigma_ntu=%s dry=%s ratio=%s | next: wait ~8s for MEASURE\r\n",
           ec0, (unsigned long)p_out->ntu0_ao_mv, sig_ec, sig_ntu, dry, ratio);
}

/*
 * 函数名：app_print_status
 * 作用：命令 P：打印当前状态/基线/自适应阈值/特征/水温/执行器模式。
 * 参数：now_ms —— 当前时间。
 * 返回：无。
 * 时序/边界：同步打印约 8 行（约 200 B ≈ 17 ms），不影响 50 ms tick。
 * 注意：全部整数/字符串打印，无浮点 printf。
 */
static void app_print_status(uint32_t now_ms)
{
    const pipeline_out_t *p_out = pipeline_get_out(&g_pipe);
    const decision_out_t *p_dec = decision_get_out(&g_dec);
    char s1[20];
    char s2[20];
    char s3[20];
    char s4[20];
    char bits[48];
    char ac[24];

    printf("# ---------- STATUS t=%lums ----------\r\n", (unsigned long)now_ms);
    printf("# fw=%s tick=%ums\r\n", APP_FW_VERSION, (unsigned)APP_TICK_MS);

    app_autocal_text(ac, (uint32_t)sizeof(ac));
    if ('\0' != ac[0])
    {
        printf("# autocal: %s\r\n", ac);
    }

    if (NULL != p_dec)
    {
        printf("# dec: state=%s level=%s action=%s event=%u changed=%u reason=%s\r\n",
               decision_state_str(p_dec->state),
               decision_level_str(p_dec->level),
               decision_action_str(p_dec->action),
               (unsigned)p_dec->event_active,
               (unsigned)p_dec->changed,
               p_dec->reason);

        gate_bits_str(&g_dec.gate, bits, (uint32_t)sizeof(bits));
        printf("# gate: valid=0x%02X(%s) regime=%s wet=%u stable=%u blank=%u permit=%u reason=%s\r\n",
               (unsigned)p_dec->valid_bits, bits,
               gate_regime_str(p_dec->regime),
               (unsigned)(p_dec->immersed ? 1U : 0U),
               (unsigned)((p_dec->valid_bits & (uint16_t)VAL_STABLE) ? 1U : 0U),
               (unsigned)(p_dec->blanking ? 1U : 0U),
               (unsigned)(p_dec->permit ? 1U : 0U),
               p_dec->gate_reason);

        app_fmt_x10(s1, sizeof(s1), p_dec->d1_ec_mv);
        app_fmt_x10(s2, sizeof(s2), p_dec->d2_ec_mv);
        app_fmt_x10(s3, sizeof(s3), p_dec->d1_ntu_pct);
        app_fmt_x10(s4, sizeof(s4), p_dec->d2_ntu_pct);
        printf("# th: ec d1=%s d2=%s mV | ntu d1=%s d2=%s %%\r\n", s1, s2, s3, s4);
    }

    if (NULL != p_out)
    {
        app_fmt_x10(s1, sizeof(s1), p_out->ec0_mv25);
        app_fmt_x10(s2, sizeof(s2), p_out->ec_sigma_mv);
        app_fmt_x10(s3, sizeof(s3), p_out->ntu_sigma_mv);
        printf("# base: valid=%u capturing=%u n=%u ec0=%smV sigma_ec=%s ntu0=%lumV sigma_ntu=%s\r\n",
               (unsigned)p_out->baseline_valid,
               (unsigned)pipeline_baseline_capturing(&g_pipe),
               (unsigned)p_out->baseline_n,
               s1,
               s2,
               (unsigned long)p_out->ntu0_ao_mv,
               s3);

        printf("# out: adc_timeout=%u tds_status=%s ntu_status=%s note=%s\r\n",
               (unsigned)p_out->adc_timeout,
               sensor_status_str(p_out->tds_status),
               sensor_status_str(p_out->ntu_status),
               p_out->note);

        app_fmt_x10(s1, sizeof(s1), p_out->d_ec_mv);
        app_fmt_x1000(s2, sizeof(s2), p_out->g_ec);
        app_fmt_x10(s3, sizeof(s3), p_out->d_ntu_pct);
        app_fmt_x10(s4, sizeof(s4), p_out->ec_rate_mv_s);
        printf("# feat: d_ec=%smV g=%s d_ntu_pct=%s %% ec_rate=%smV/s\r\n", s1, s2, s3, s4);
    }

    app_fmt_x10(s1, sizeof(s1), ds18b20_get_temp_c(&g_ds));
    printf("# temp: %sC src=%s dv_ok=%u recheck_pending=%u\r\n",
           s1,
           app_temp_src_str(ds18b20_get_src(&g_ds)),
           (unsigned)ds18b20_sensor_ok(&g_ds),
           (unsigned)g_force_temp_recheck);

    {
        const valve_out_t *p_valve = valve_get_out(&g_valve);

        if (NULL != p_valve)
        {
            printf("# valve: mode=%s enabled=%u on_level=%s pos=%s relay=%u req=%s(%s) count=%lu\r\n",
                   app_valve_mode_str(g_valve_mode),
                   (unsigned)APP_USE_VALVE,
                   (BSP_IO_LEVEL_HIGH == g_app_valve_cfg.relay_on_level) ? "HIGH" : "LOW",
                   valve_pos_str(p_valve->position),
                   (unsigned)(p_valve->relay_on ? 1U : 0U),
                   valve_pos_str(p_valve->requested),
                   p_valve->request_reason,
                   (unsigned long)p_valve->switch_count);
            printf("# valve: on=%lums total_on=%lums max_on=%lums blocked=%u fault=%u/%lu reason=%s\r\n",
                   (unsigned long)p_valve->on_time_ms,
                   (unsigned long)p_valve->total_on_ms,
                   (unsigned long)g_app_valve_cfg.max_on_ms,
                   (unsigned)(p_valve->max_on_blocked ? 1U : 0U),
                   (unsigned)(p_valve->pin_fault ? 1U : 0U),
                   (unsigned long)p_valve->fault_count,
                   p_valve->switch_reason);
        }
    }
}

/* ---------- 串口命令处理 ---------- */
/*
 * 函数名：app_parse_temperature
 * 作用：解析 T 命令后面的水温（0~60 ℃，允许空格/等号前缀）。
 * 参数：p_text —— 命令首字符之后的部分（如 "24.5"）；p_temp_c —— 输出温度。
 * 返回：true=解析成功且在范围内；false=格式错/越界。
 * 时序/边界：strtod 解析 double；范围严格 0.0~60.0（与 00 文档命令表一致）。
 * 注意：不使用浮点 printf；越界/空串都返回 false，由调用方打印示例。
 */
static bool app_parse_temperature(const char *p_text, float *p_temp_c)
{
    const char *p = p_text;
    char *p_end = NULL;
    double value;

    while ((' ' == *p) || ('\t' == *p) || ('=' == *p))
    {
        p++;
    }
    if ('\0' == *p)
    {
        return false;
    }

    value = strtod(p, &p_end);
    if (p_end == p)
    {
        return false;   /* 一个数字都没解析出来 */
    }
    if ((value < 0.0) || (value > 60.0))
    {
        return false;   /* 水温范围保护：-? / >60 都拒绝，防止温补系数被拉飞 */
    }

    *p_temp_c = (float)value;
    return true;
}

/*
 * 函数名：app_handle_valve_cmd
 * 作用：解析 V 命令：V1 强制回收 / V0 强制排空 / VA 回自动。
 * 参数：p_text —— 命令首字符之后的部分（如 "1"）。
 * 返回：无。
 * 时序/边界：立即生效；手动覆盖一直保持到 VA 或复位。
 * 注意：手动覆盖优先于判别结果（调试/演示用）；调试完记得发 VA 回自动。
 */
static void app_handle_valve_cmd(const char *p_text)
{
    const char *p = p_text;
    char sub;
    bool changed_cmd = true;

    while ((' ' == *p) || ('\t' == *p) || ('=' == *p))
    {
        p++;
    }
    sub = app_to_upper(*p);

    if ('1' == sub)
    {
        g_valve_mode = APP_VALVE_FORCE_RECOVER;
        printf("# valve: FORCE RECOVER (relay ON -> 12V valve recover port)\r\n");
    }
    else if ('0' == sub)
    {
        g_valve_mode = APP_VALVE_FORCE_DRAIN;
        printf("# valve: FORCE DRAIN (relay OFF -> valve de-energized, drain port)\r\n");
    }
    else if ('A' == sub)
    {
        g_valve_mode = APP_VALVE_AUTO;
        printf("# valve: AUTO (follow decision action)\r\n");
    }
    else
    {
        changed_cmd = false;
        printf("# ERR: use V1 / V0 / VA\r\n");
    }

    if (changed_cmd)
    {
        printf("# valve mode=%s enabled=%u (0=fake LED only, 1=P104 drives relay)\r\n",
               app_valve_mode_str(g_valve_mode), (unsigned)APP_USE_VALVE);
    }
}

/*
 * 函数名：app_handle_command
 * 作用：解析一行命令并执行（H/?、M、R、E、B、T、C、P、V1/V0/VA）。
 * 参数：p_cmd —— 以 '\0' 结尾的整行命令（可含前后空格）；now_ms —— 当前时间。
 * 返回：无。
 * 时序/边界：在主循环上下文执行，命令间至少间隔一个 tick；未知命令打印提示。
 * 注意：大小写不敏感；命令只改状态/打印，不在本函数里做阻塞动作。
 */
static void app_handle_command(char *p_cmd, uint32_t now_ms)
{
    char *p = p_cmd;
    char first;

    while ((' ' == *p) || ('\t' == *p))
    {
        p++;
    }
    if ('\0' == *p)
    {
        return;
    }

    first = app_to_upper(*p);

    switch (first)
    {
        case 'H':
        case '?':
            app_print_help();
            break;

        case 'M':
            app_print_mark("UART", now_ms);
            break;

        case 'R':
        {
            const decision_out_t *p_dec = decision_get_out(&g_dec);
            bool was_active = (NULL != p_dec) && p_dec->event_active;

            decision_start_event(&g_dec, now_ms);
            if (was_active)
            {
                printf("# NOTE: event marker restarted (log segmentation only, no control)\r\n");
            }
            else
            {
                g_manual_event_started = true;
                printf("# NOTE: event marker start (log segmentation only, no control)\r\n");
            }
        }
        break;

        case 'E':
        {
            const decision_out_t *p_dec = decision_get_out(&g_dec);

            if ((NULL != p_dec) && p_dec->event_active)
            {
                decision_end_event(&g_dec, now_ms);
                g_manual_event_ended = true;
                printf("# NOTE: event marker end (log segmentation only, no control)\r\n");
            }
            else
            {
                printf("# NOTE: no active event marker to end\r\n");
            }
        }
        break;

        case 'D':
            app_autocal_cancel("manual D");
            app_dry_begin(now_ms, (uint16_t)APP_DRY_CAPTURE_N, false);
            break;

        case 'W':
            app_autocal_cancel("manual W");
            app_baseline_begin(now_ms, "UART");
            break;

        case 'G':
            app_print_gate(now_ms);
            break;

        case 'Z':
            app_autocal_cancel("manual Z");
            pipeline_invalidate_baseline(&g_pipe, "CLEARED");
            printf("# NOTE: wet baseline cleared -> regime leaves MEASURE (drain)\r\n");
            printf("#   autocal stopped; send W (or power-cycle) to calibrate again\r\n");
            break;

        case 'B':
            app_autocal_cancel("manual B");
            app_baseline_begin(now_ms, "UART");
            break;

        case 'C':
            g_force_temp_recheck = true;
            printf("# DS18B20: re-detect requested\r\n");
            break;

        case 'T':
        {
            char temp_text[20];
            float temp_c = 0.0f;

            if (app_parse_temperature(p + 1, &temp_c))
            {
                ds18b20_set_manual_temp(&g_ds, temp_c);
                app_fmt_x10(temp_text, sizeof(temp_text), temp_c);
                printf("# manual water temp = %s C (used when DS18B20 is not available)\r\n",
                       temp_text);
            }
            else
            {
                printf("# ERR: bad temperature, example T24.5 (range 0..60 C)\r\n");
            }
        }
        break;

        case 'P':
            app_print_status(now_ms);
            break;

        case 'V':
            app_handle_valve_cmd(p + 1);
            break;

        default:
            printf("# ERR: unknown command '%c' (send H for help)\r\n", first);
            break;
    }
}

/*
 * 函数名：app_poll_command
 * 作用：主循环里检查是否收到一整行命令；有则"短暂关中断"复制出来再解析。
 * 参数：now_ms —— 当前时间（传给命令处理）。
 * 返回：无。
 * 时序/边界：复制关键区 <100 µs（115200 下 1 字节 87 µs），不会丢字节。
 * 注意：关中断只为消除"主循环读到一半、回调又追加"的竞态；解析仍在开中断下做。
 */
static void app_poll_command(uint32_t now_ms)
{
    char local[APP_RX_BUF_SIZE];
    uint8_t len;
    uint8_t i;
    bool overflow;
    uint32_t primask;

    if (!g_rx_ready)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    len           = g_rx_len;
    overflow      = g_rx_overflow;
    for (i = 0U; i < len; i++)
    {
        local[i] = g_rx_buf[i];
    }
    local[len]    = '\0';
    g_rx_len      = 0U;
    g_rx_ready    = false;
    g_rx_overflow = false;

    if (0U == primask)
    {
        __enable_irq();
    }

    if (0U == len)
    {
        return;   /* 防御：只有行尾没有内容 */
    }

    if (overflow)
    {
        printf("# ERR: command line too long (max %u), ignored\r\n",
               (unsigned)(APP_RX_BUF_SIZE - 1U));
        return;
    }

    app_handle_command(local, now_ms);
}

/* ---------- 事件/状态/工况日志 ---------- */
/*
 * 函数名：app_log_changes
 * 作用：检测事件/状态/动作/工况边界，打印 "# EVENT / # STATE / # ACTION / # GATE"。
 * 参数：now_ms —— 当前时间；p_dec —— 判别输出。
 * 返回：无。
 * 时序/边界：每 tick 调用一次；只在"边界变化"时打印，稳态不刷屏。
 * 注意：v2 起事件只是日志分段（不再控制阀），EVENT 行只反映手动 R/E；
 *       工况变化单独打 "# GATE"，PC 端按行解析（新增行可忽略，CSV 17 列不变）。
 */
static void app_log_changes(uint32_t now_ms, const decision_out_t *p_dec)
{
    uint16_t gate_mask;

    if (NULL == p_dec)
    {
        return;
    }

    /* 只在"会不会影响许可"的位上打日志：STABLE 会随气泡抖动、IMMERSED 会在液面
     * 贴着电极时逐 tick 翻转，放进变化检测会刷屏并挤占 50 ms tick。
     * 工况变化本身已经覆盖了"浸没/失浸没"的语义（DRY/MEASURE 迁移）。 */
    gate_mask = (uint16_t)((uint16_t)VAL_SENSOR_OK | (uint16_t)VAL_BASELINE_OK
                         | (uint16_t)VAL_BLANKING);

    if ((p_dec->regime != g_prev_regime)
        || (0U != ((p_dec->valid_bits ^ g_prev_valid_bits) & gate_mask)))
    {
        char bits[48];

        gate_bits_str(&g_dec.gate, bits, (uint32_t)sizeof(bits));
        printf("# GATE t=%lu valid=0x%02X(%s) regime=%s wet=%u blank=%u reason=%s\r\n",
               (unsigned long)now_ms,
               (unsigned)p_dec->valid_bits,
               bits,
               gate_regime_str(p_dec->regime),
               (unsigned)(p_dec->immersed ? 1U : 0U),
               (unsigned)(p_dec->blanking ? 1U : 0U),
               p_dec->gate_reason);
        g_prev_valid_bits = p_dec->valid_bits;
        g_prev_regime     = p_dec->regime;
    }

    if (p_dec->event_active != g_prev_event_active)
    {
        if (p_dec->event_active)
        {
            printf("# EVENT t=%lu start=%s\r\n",
                   (unsigned long)now_ms,
                   g_manual_event_started ? "MANUAL" : "AUTO");
        }
        else
        {
            printf("# EVENT t=%lu end=%s\r\n",
                   (unsigned long)now_ms,
                   g_manual_event_ended ? "MANUAL" : "TIMEOUT");
        }
        g_prev_event_active   = p_dec->event_active;
        g_manual_event_started = false;
        g_manual_event_ended   = false;
    }

    if (p_dec->state != g_prev_state)
    {
        printf("# STATE t=%lu from=%s to=%s reason=%s\r\n",
               (unsigned long)now_ms,
               decision_state_str(g_prev_state),
               decision_state_str(p_dec->state),
               p_dec->reason);
        g_prev_state = p_dec->state;
    }

    if (p_dec->action != g_prev_action)
    {
        /* 同时打印"判别动作"与"阀实际位置"：min_dwell / max_on 冷却期间两者可能相反，
         * CSV 的 action 列取阀位置，这里把差异显式写出来，避免同一秒日志自相矛盾。
         * 字段追加在行尾，旧脚本按第 4 个字段取 RECOV/DRAIN 仍然可用。 */
        const valve_out_t *p_valve = valve_get_out(&g_valve);
        const char *valve_str = "?";

        if (NULL != p_valve)
        {
            valve_str = (VALVE_POS_RECOVER == p_valve->position) ? "RECOV" : "DRAIN";
        }

        printf("# ACTION t=%lu %s valve=%s\r\n",
               (unsigned long)now_ms,
               decision_action_str(p_dec->action),
               valve_str);
        g_prev_action = p_dec->action;
    }
}

/* ---------- CSV ---------- */
/*
 * 函数名：app_print_csv
 * 作用：拼一行 17 列 CSV 并打印（纯 ASCII、逗号分隔、整数缩放）。
 * 参数：now_ms —— 当前时间（t_ms 列）；p_out —— 流水线输出；p_dec —— 判别输出。
 * 返回：无。
 * 时序/边界：每 200 ms 调用一次；用 snprintf 拼进 256 B 静态栈缓冲，再 printf("%s\r\n")。
 * 注意：字段顺序/缩放严格按 00 文档 §5.2（17 列不变）：
 *       t_ms,t_Cx100,src,tds_adc,tds_mv,tds_mv25x10,tds_ppm_x10,tds_dmvx10,
 *       tds_gx1000,ntu_adc,ntu_node_mv,ntu_dpctx10,ec_ratex10,state,level,action,note
 *       note 列 v2 改造成"工况码"（OK/DRY/SET/BLK/LOCK/NOBASE/SENS，见 app_note_code）；
 *       全部整数或有界短字符串；不出现浮点 printf。
 */
static void app_print_csv(uint32_t now_ms, const pipeline_out_t *p_out,
                          const decision_out_t *p_dec)
{
    char line[APP_CSV_LINE_MAX];
    const char *action_str;
    int32_t t_c_x100;
    int32_t mv25_x10;
    int32_t ppm_x10;
    int32_t dmv_x10;
    int32_t g_x1000;
    int32_t ntu_pct_x10;
    int32_t ec_rate_x10;

    if ((NULL == p_out) || (NULL == p_dec))
    {
        return;
    }

    /*
     * action 列取"阀的实际位置"（AUTO 模式下这是最诚实的记录）：
     * decision 输出 RECOV 后，阀还要过 min_dwell（最长 500 ms）才真正换向；
     * max_on 超时/冷却期间阀会被强制排空，而 decision 仍可能请求回收。
     * 若直接打印 decision->action，CSV 会与 # VALVE 行的实际位置相反。
     * 假执行器模式（APP_USE_VALVE=0）不换向，仍打印 decision 动作以便实验 3 演示。
     */
    action_str = decision_action_str(p_dec->action);
    if (0 != APP_USE_VALVE)
    {
        const valve_out_t *p_valve = valve_get_out(&g_valve);

        if (NULL != p_valve)
        {
            action_str = (VALVE_POS_RECOVER == p_valve->position) ? "RECOV" : "DRAIN";
        }
    }

    t_c_x100     = app_scale_int(ds18b20_get_temp_c(&g_ds), 100.0f);
    mv25_x10     = app_scale_int(p_out->tds_mv25, 10.0f);
    ppm_x10      = app_scale_int(p_out->tds_ppm25, 10.0f);
    dmv_x10      = app_scale_int(p_out->d_ec_mv, 10.0f);
    g_x1000      = app_scale_int(p_out->g_ec, 1000.0f);
    ntu_pct_x10  = app_scale_int(p_out->d_ntu_pct, 10.0f);
    ec_rate_x10  = app_scale_int(p_out->ec_rate_mv_s, 10.0f);

    (void)snprintf(line, sizeof(line),
                   "%lu,%ld,%s,%u,%lu,%ld,%ld,%ld,%ld,%u,%lu,%ld,%ld,%s,%s,%s,%s",
                   (unsigned long)now_ms,
                   (long)t_c_x100,
                   app_temp_src_str(ds18b20_get_src(&g_ds)),
                   (unsigned)p_out->tds_raw,
                   (unsigned long)p_out->tds_mv,
                   (long)mv25_x10,
                   (long)ppm_x10,
                   (long)dmv_x10,
                   (long)g_x1000,
                   (unsigned)p_out->ntu_raw,
                   (unsigned long)p_out->ntu_node_mv,
                   (long)ntu_pct_x10,
                   (long)ec_rate_x10,
                   decision_state_str(p_dec->state),
                   decision_level_str(p_dec->level),
                   action_str,
                   app_note_code(p_out, p_dec));

    printf("%s\r\n", line);
}

/* ---------- 人类可读摘要（# TIP，2 s 一次） ---------- */
/*
 * 为什么需要它：CSV 是 17 列纯数字，现场（尤其不接 PC 脚本时）很难一眼看出
 * "现在是什么工况、该做什么"。TIP 行用 ASCII 打成一句话，和 CSV 交替输出，
 * 不改变 CSV 的列数/顺序；以 '#' 开头，PC 端脚本可以直接忽略。
 */
static const char *app_tip_hint(const decision_out_t *p_dec)
{
    if (NULL == p_dec)
    {
        return "no decision";
    }

    switch (p_dec->regime)
    {
        case REGIME_FAULT:
            if (0U == (p_dec->valid_bits & (uint16_t)VAL_SENSOR_OK))
            {
                return "sensor fault: check TDS/NTU probe and wiring";
            }
            return "not calibrated: wait autocal, or send D then W";

        case REGIME_DRY:
            return "no water at electrode: pour water over the probe";

        case REGIME_SETTLE:
            return "water detected: settling, keep the chamber still";

        case REGIME_LOCKOUT:
            return "sensor recovering: drain until lockout ends";

        case REGIME_MEASURE:
        default:
            break;
    }

    if (p_dec->blanking)
    {
        return "valve switched: decision frozen 400 ms";
    }

    switch (p_dec->state)
    {
        case DEC_STATE_CLEAR:
            return "clean water: RECOVER";

        case DEC_STATE_UNCERTAIN:
            return "uncertain: DRAIN";

        case DEC_STATE_SOAP:
            return "dirty/soap: DRAIN";

        case DEC_STATE_WAIT:
        default:
            return "waiting";
    }
}

static void app_print_tip(uint32_t now_ms, const pipeline_out_t *p_out,
                          const decision_out_t *p_dec)
{
    char s_ec[20];
    char s_g[20];
    char s_dec[20];
    char s_ntu[20];
    char s_temp[20];
    char s_ac[24];
    const char *p_ac = "-";

    if ((NULL == p_out) || (NULL == p_dec))
    {
        return;
    }

    app_fmt_x10(s_ec, sizeof(s_ec), p_out->tds_mv25);
    app_fmt_x1000(s_g, sizeof(s_g), p_out->g_ec);
    app_fmt_x10(s_dec, sizeof(s_dec), p_out->d_ec_mv);
    app_fmt_x10(s_ntu, sizeof(s_ntu), p_out->d_ntu_pct);
    app_fmt_x10(s_temp, sizeof(s_temp), ds18b20_get_temp_c(&g_ds));

    app_autocal_text(s_ac, (uint32_t)sizeof(s_ac));
    if ('\0' != s_ac[0])
    {
        p_ac = s_ac;
    }

    printf("# TIP t=%lums | regime=%s state=%s level=%s permit=%u | "
           "EC=%smV g=%s dEC=%smV dNTU=%s%% T=%sC | autocal=%s | %s\r\n",
           (unsigned long)now_ms,
           gate_regime_str(p_dec->regime),
           decision_state_str(p_dec->state),
           decision_level_str(p_dec->level),
           (unsigned)(p_dec->permit ? 1U : 0U),
           s_ec, s_g, s_dec, s_ntu, s_temp, p_ac,
           app_tip_hint(p_dec));
}

/* ---------- 启动摘要 ---------- */
/*
 * 函数名：app_print_boot_summary
 * 作用：上电打印参数摘要（tick/CSV/阈值/执行器模式/OLED 显示/操作提示），便于现场核对配置。
 * 参数：无。
 * 返回：无。
 * 时序/边界：初始化全部完成后调用一次（此时 g_pipe/g_dec 配置已生效）。
 * 注意：打印的阈值/参数默认值都是"待标定"，不能当实测结论。
 */
static void app_print_boot_summary(void)
{
    char s1[20];
    char s2[20];
    char s3[20];
    char s4[20];
    char s5[20];
    char s6[20];
    char s7[20];

    printf("# timing: tick=%ums csv=%ums block=%lums samples=%u agree=%u upgrade_blocks=%u downgrade_blocks=%u\r\n",
           (unsigned)APP_TICK_MS,
           (unsigned)APP_CSV_PERIOD_MS,
           (unsigned long)g_dec.cfg.block_ms,
           (unsigned)g_dec.cfg.block_samples,
           (unsigned)g_dec.cfg.block_agree,
           (unsigned)g_dec.cfg.upgrade_blocks,
           (unsigned)g_dec.cfg.downgrade_blocks);

    app_fmt_x10(s1, sizeof(s1), g_dec.cfg.ec_d1_sigma);
    app_fmt_x10(s2, sizeof(s2), g_dec.cfg.ec_d1_min_mv);
    app_fmt_x10(s3, sizeof(s3), g_dec.cfg.ec_d1_pct);
    app_fmt_x10(s4, sizeof(s4), g_dec.cfg.ec_d2_sigma);
    app_fmt_x10(s5, sizeof(s5), g_dec.cfg.ec_d2_min_mv);
    app_fmt_x10(s6, sizeof(s6), g_dec.cfg.ec_d2_pct);
    app_fmt_x10(s7, sizeof(s7), g_dec.cfg.ec_hyst_ratio * 100.0f);
    printf("# thresholds: ec d1=max(%s*sigma,%smV,%s%%EC0) d2=max(%s*sigma,%smV,%s%%EC0) hyst=%s%%\r\n",
           s1, s2, s3, s4, s5, s6, s7);

    app_fmt_x10(s1, sizeof(s1), g_dec.cfg.ntu_d1_pct);
    app_fmt_x10(s2, sizeof(s2), g_dec.cfg.ntu_d2_pct);
    app_fmt_x10(s3, sizeof(s3), g_dec.cfg.ntu_hyst_ratio * 100.0f);
    printf("# ntu d1=%s%% d2=%s%% (relative only, NOT NTU) hyst=%s%%\r\n",
           s1, s2, s3);

    app_fmt_x10(s1, sizeof(s1), g_dec.cfg.ec_rate_th_mv_s);
    app_fmt_x10(s2, sizeof(s2), g_dec.cfg.ntu_rate_th_pct_s);
    printf("# rate preempt: ec>%smV/s ntu>%s%%/s confirm=%u tick (blank 400ms after switch)\r\n",
           s1, s2, (unsigned)g_dec.cfg.rate_confirm_ticks);

    app_fmt_x10(s1, sizeof(s1), g_dec.gate.cfg.wet_ratio);
    app_fmt_x10(s2, sizeof(s2), g_dec.gate.cfg.ec_wet_delta_mv);
    app_fmt_x10(s3, sizeof(s3), g_dec.gate.cfg.ao_water_delta_mv);
    app_fmt_x10(s4, sizeof(s4), (float)g_dec.gate.cfg.settle_ms / 1000.0f);
    app_fmt_x10(s5, sizeof(s5), (float)g_dec.gate.cfg.settle_timeout_ms / 1000.0f);
    app_fmt_x10(s6, sizeof(s6), (float)g_dec.gate.cfg.blank_ms / 1000.0f);
    printf("# gate: wet_ratio=%s ec_delta>=%smV ao_delta>=%smV settle=%s/%ss blank=%ss\r\n",
           s1, s2, s3, s4, s5, s6);

    printf("# actuator: P104 -> 1ch relay module (SRD-05VDC-SL-C) -> 12V 2/3-way valve\r\n");
    printf("# actuator: enabled=%u on_level=%s (RECOVER=relay ON) min_dwell=%lums max_on=%lums cooldown=%lums\r\n",
           (unsigned)APP_USE_VALVE,
           (BSP_IO_LEVEL_HIGH == g_app_valve_cfg.relay_on_level) ? "HIGH" : "LOW",
           (unsigned long)g_app_valve_cfg.min_dwell_ms,
           (unsigned long)g_app_valve_cfg.max_on_ms,
           (unsigned long)g_app_valve_cfg.cooldown_ms);
    printf("# actuator: safe position=DRAIN (relay OFF), P104 initialized before UART (fail-safe)\r\n");
    printf("# uart: 115200 8N1 polled-TX | tx_poll_fail=%lu (0=正常, >0=发送异常)\r\n",
           (unsigned long)g_uart_tx_timeouts);
    printf("# display: SSD1306 128x64 I2C0 addr=0x3C | 3 pages: P1 water / P2 exec / P3 info\r\n");
    printf("# display: K2 short = next page; K2 long >=1.5s = baseline; M (UART) = mark\r\n");
    printf("# display: site indicator only; CSV remains the record (display is NOT safety path)\r\n");
    printf("# IMPORTANT: baseline must be captured in water; TDS probe in air is a fake ~1.6V value\r\n");
    printf("# operation: autocal at boot (dry -> wait water -> wet). manual D/W/G/Z still available.\r\n");
    printf("# operation: G (gate) / Z (clear base) / R,E (log markers) / P (status) / H (help)\r\n");
    printf("# serial legend: '#' lines are annotations/events; other lines are 17-column CSV data\r\n");
    printf("# TIP: human-readable summary every %lus (regime/state/permit/EC/g/dEC/T + next action)\r\n",
           (unsigned long)(APP_TIP_PERIOD_MS / 1000U));
    printf("# CSV header:\r\n");
    printf("# t_ms,t_Cx100,src,tds_adc,tds_mv,tds_mv25x10,tds_ppm_x10,tds_dmvx10,tds_gx1000,ntu_adc,ntu_node_mv,ntu_dpctx10,ec_ratex10,state,level,action,note\r\n");
    printf("# CSV units: t=ms | T=C*100 | mv25/ppm/dEC/rate=x10 | g=x1000 | dNTU=%%*10 | src=DS/MAN/FIX\r\n");
}

/* ---------- 外设初始化 ---------- */
/*
 * 函数名：app_uart_init
 * 作用：打开 UART9（CH340E，115200 8N1），关闭 stdout 缓冲。
 * 参数：无。
 * 返回：true=成功；false=失败（调用方用 LED 快闪报警）。
 * 时序/边界：必须在任何 printf 之前；R_SCI_UART_Open 由 RASC 生成的 g_uart9_cfg 配置。
 * 注意：setvbuf 必须紧跟 Open——否则 boot 行会憋在 C 库缓冲里（tds_mini 踩过的坑）。
 */
static bool app_uart_init(void)
{
    fsp_err_t err;
    static baud_setting_t          s_baud;         /* 运行时按真实 PCLKB 复算的波特率参数 */
    static sci_uart_extended_cfg_t s_ext;
    static uart_cfg_t              s_cfg;
    uart_cfg_t const *             p_cfg = &g_uart9_cfg;

    /* 用 FSP 自带算法按"当前真实 PCLKB"复算 115200：
     * ra_gen 里的 g_uart9_baud_setting 是 RASC 在生成那一刻按当时时钟算出的；若之后改过
     * 时钟却没重新 Generate Project Content，它就是旧的（本工程实测偏差 +1.7%）。
     * 复算失败时退回原配置，不影响启动。 */
    if (FSP_SUCCESS == R_SCI_UART_BaudCalculate(115200U, true, 15000U, &s_baud))
    {
        s_ext                = g_uart9_cfg_extend;
        s_ext.p_baud_setting = &s_baud;
        s_cfg                = g_uart9_cfg;
        s_cfg.p_extend       = &s_ext;
        p_cfg                = &s_cfg;
    }

    err = R_SCI_UART_Open(&g_uart9_ctrl, p_cfg);
    if (FSP_SUCCESS != err)
    {
        return false;
    }

    (void)setvbuf(stdout, NULL, _IONBF, 0U);
    return true;
}

/*
 * 函数名：app_adc_init
 * 作用：打开 ADC0 并配置扫描组（AN11 浊度 + AN12 TDS，单次扫描）。
 * 参数：无。
 * 返回：true=Open/ScanCfg 都成功；false=任一失败（失败也继续跑，安全侧 L1）。
 * 时序/边界：上电调用一次；pipeline_sample() 每 tick 触发扫描并读结果。
 * 注意：ADC 失败不能死机：采样会超时 → 判别层 SENSOR_FAULT → L1 → 排空（安全侧）。
 */
static bool app_adc_init(void)
{
    fsp_err_t err;
    bool ok = true;

    err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("# ERR: R_ADC_Open = %d\r\n", (int)err);
        ok = false;
    }

    err = R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc0_channel_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("# ERR: R_ADC_ScanCfg = %d\r\n", (int)err);
        ok = false;
    }

    if (!ok)
    {
        printf("# WARN: ADC unusable -> samples time out -> decision falls back to L1 (drain)\r\n");
    }
    return ok;
}

/* ---------- 一个 50 ms tick 的全部工作 ---------- */
/*
 * 函数名：app_tick
 * 作用：主循环的一个 tick：温度任务 → 温补设置 → 采样 → 判别 → 日志 → 执行器
 *       → 板载 OLED 状态页 → 按键 → 串口命令 → 200 ms CSV。
 * 参数：now_ms —— 本 tick 的时间戳（timebase_ms）。
 * 返回：无。
 * 时序/边界：每 50 ms 一次；顺序不能调换（温度先更新，判别才会用对温补值）。
 * 注意：本函数必须"非阻塞"——所有底层调用（DS18B20/ADC 采样）都由驱动做成
 *       短事务/带超时，绝不在这里 delay 等待（唯一例外是 printf 的 TX 忙等，µs 级）。
 */
static void app_tick(uint32_t now_ms)
{
    const pipeline_out_t *p_out;
    const decision_out_t *p_dec;
    bool recover;

    /* 1) 温度非阻塞状态机：每 2 s 发起转换、800 ms 后读数；命令 C 强制重检 */
    ds18b20_task(&g_ds, now_ms, g_force_temp_recheck);
    g_force_temp_recheck = false;

    /* 2) 把当前采用水温交给流水线（TDS 温补用；from_sensor=false 表示手输/兜底） */
    pipeline_set_water_temp(&g_pipe, ds18b20_get_temp_c(&g_ds), ds18b20_sensor_ok(&g_ds));

    /* 3) 采样 + 换算 + 滤波 + 基线/特征（双通道中值、EMA、ΔEC/ΔNTU、变化率） */
    (void)pipeline_sample(&g_pipe, now_ms);
    p_out = pipeline_get_out(&g_pipe);
    if (NULL == p_out)
    {
        return;   /* 防御：正常不会发生（pipeline 句柄是静态分配的） */
    }

    /* 4) 干态指纹采集（命令 D，非阻塞；完成后写入 gate 并影响本 tick 的工况） */
    app_dry_task(now_ms, p_out);

    /* 5) 判别（内部先跑工况机，只在 REGIME_MEASURE 里分类；许可每 tick 重算） */
    decision_update(&g_dec, &g_pipe, now_ms);
    p_dec = decision_get_out(&g_dec);
    if (NULL == p_dec)
    {
        return;
    }

    /* 5b) 开机自检（v2.1）：必须在 decision_update 之后（gate.immersed 才是最新的）。
     *     上电自动干态 → 等水 → 自动湿态；手动 D/W 会接管并中止自检。 */
    if (g_ac_start_pending)
    {
        g_ac_start_pending = false;
        app_autocal_start(now_ms);
    }
    app_autocal_task(now_ms, p_out);

    /* 6) 事件/状态/动作/工况变化 → "# EVENT / # STATE / # ACTION / # GATE" 事件行 */
    app_log_changes(now_ms, p_dec);

    /* 7) 基线采集完成 → 业务校验（必须明显高于干态），通过才打印 "# BASELINE OK" */
    if (p_out->baseline_updated)
    {
        app_baseline_validate(now_ms, p_out);
    }

    /* 8) 执行器：LED 假执行器（P111）+ 真阀（P104 → 继电器 → 12V 二位三通阀） */
    recover = app_valve_recover_requested(p_dec);
    app_valve_apply(recover, now_ms);
    app_valve_log(now_ms);
    app_led_task(p_dec, now_ms);

    /* 9) 板载 OLED 三页状态页（P1 水质/P2 执行器/P3 详情；只做现场指示，失败不影响安全侧） */
    if (!display_task(now_ms, p_out, p_dec, valve_get_out(&g_valve)) && (!g_oled_warned))
    {
        g_oled_warned = true;
        printf("# WARN: OLED lost at t=%lu ms -> UART only\r\n", (unsigned long)now_ms);
    }

    /* 10) 按键 K2：短按切 OLED 页 / 长按 ≥1.5 s 重新采基线 */
    app_key_task(now_ms);

    /* 11) 串口命令：整行解析，命令只改状态，耗时 <1 ms */
    app_poll_command(now_ms);

    /* 12) CSV：每 200 ms 一行（5 行/秒）；用 now_ms 直接对齐，落后不补打多行 */
    if ((uint32_t)(now_ms - g_last_csv_ms) >= APP_CSV_PERIOD_MS)
    {
        g_last_csv_ms = now_ms;
        app_print_csv(now_ms, p_out, p_dec);
    }

    /* 13) 人类可读摘要：每 2 s 一行 # TIP（不改变 CSV 列，脚本可忽略） */
    if ((uint32_t)(now_ms - g_last_tip_ms) >= APP_TIP_PERIOD_MS)
    {
        g_last_tip_ms = now_ms;
        app_print_tip(now_ms, p_out, p_dec);
    }
}

/* ---------- 应用入口 ---------- */
/*
 * 函数名：app_main
 * 作用：传感器节点入口（由 hal_entry.cpp 调用，正常情况下永不返回）：
 *       按安全顺序初始化外设/传感器/流水线/判别器，然后进入 50 ms 主循环。
 * 参数：无。
 * 返回：无（死循环）。
 * 时序/边界：初始化总耗时约 1.5~2 s（DS18B20 boot_test ≈800 ms + LED 快闪 ≈0.72 s
 *             + OLED 上电页 I2C 写 ≈几十 ms + ADC/UART 打开 ≈ms 级），之后主循环
 *             严格 50 ms tick。
 * 注意：初始化顺序是安全设计的一部分（P104 排空 → LED → UART → OLED → ADC →
 *       DS18B20 → pipeline → decision → 基线），不要随意调换。
 */
void app_main(void)
{
    char s1[20];
    uint32_t now_ms;
    uint32_t next_tick_ms;

    /* ----- 步骤 0：fail-safe 第一件事：P104=排空 ----- */
    app_valve_init();

    /* ----- 步骤 1：LED（串口坏了也能看灯判断程序是否在跑） ----- */
    app_led_init();

    /* ★ 先快闪 3 下再碰串口：这是"CPU 是否在跑我们的代码"的第一证据。
     *   若灯闪了但后面没串口/没屏，问题就锁定在 UART 这一段，而不是启动/时钟。 */
    app_led_boot_blink();

    /* ----- 步骤 2：UART（唯一数据通道）+ 立即吐 boot 行 ----- */
    if (!app_uart_init())
    {
        app_fatal_led_forever();   /* 串口都打不开：LED 10 Hz 报警，不静默死等 */
    }
    printf("\r\n\r\n# [BOOT] %s build " __DATE__ " " __TIME__ "\r\n", APP_FW_VERSION);
    printf("# board: RA4M2_MINI (R7FA4M2AD3CFL) | tick base: SysTick 1 ms\r\n");

    /* ----- 步骤 0 的回读：确认 P104 已锁在"继电器释放（排空）"电平 ----- */
    {
        const valve_out_t *p_valve = valve_get_out(&g_valve);

        if ((NULL == p_valve) || (!p_valve->initialized))
        {
            printf("# WARN: valve GPIO init FAILED -> relay not driven, system stays DRAIN\r\n");
        }
        else if (0 == APP_USE_VALVE)
        {
            printf("# actuator: fake mode (APP_USE_VALVE=0), P104 locked at relay-OFF level\r\n");
        }
        else
        {
            printf("# actuator: valve ready (P104 locked at relay-OFF/drain, on_level=%s)\r\n",
                   (BSP_IO_LEVEL_HIGH == g_app_valve_cfg.relay_on_level) ? "HIGH" : "LOW");
        }
    }

    /* ----- 步骤 2b：板载 OLED（I2C0 @0x3C 写开机页；失败只 WARN，不算致命） ----- */
    if (display_init())
    {
        printf("# display: OLED OK (SSD1306 128x64, I2C0 @0x3C)\r\n");
    }
    else
    {
        printf("# WARN: OLED not usable -> UART only (display is NOT safety path)\r\n");
    }

    /* ----- 步骤 3：1 ms 时基（必须在任何 timebase_ms 调用之前） ----- */
    if (!timebase_init())
    {
        printf("# ERR: timebase_init failed (SysTick), halting\r\n");
        app_fatal_led_forever();
    }

    /* ----- 步骤 4：ADC0 Open + ScanCfg（失败打印 ERR，但继续跑） ----- */
    (void)app_adc_init();

    /* ----- 步骤 5：DS18B20 初始化 + 上电自检（约 800 ms，仅上电一次） ----- */
    ds18b20_init(&g_ds, NULL);
    if (ds18b20_boot_test(&g_ds))
    {
        app_fmt_x10(s1, sizeof(s1), ds18b20_get_temp_c(&g_ds));
        printf("# DS18B20 boot: OK temp=%s C src=%s\r\n",
               s1, app_temp_src_str(ds18b20_get_src(&g_ds)));
    }
    else
    {
        printf("# DS18B20 boot: NOT USABLE -> fallback 25.0 C\r\n");
        printf("#   no DS18B20 is fine: send T<value> (e.g. T24.5) to set water temp manually\r\n");
        printf("#   if you did wire it: check DQ (P500/U1-17), VDD=3.3V, GND; then send C\r\n");
    }

    /* ----- 步骤 6：K2（P000）输入 + 上拉（板上已有 4.7 kΩ，内部上拉作冗余） ----- */
    (void)R_IOPORT_PinCfg(&g_ioport_ctrl, APP_KEY_PIN,
                          (uint32_t)IOPORT_CFG_PORT_DIRECTION_INPUT |
                          (uint32_t)IOPORT_CFG_PULLUP_ENABLE);

    /* ----- 步骤 7：采集/特征流水线 ----- */
    pipeline_init(&g_pipe, &g_app_pipeline_cfg);

    /* ----- 步骤 8：三级判别器 ----- */
    decision_init(&g_dec, &g_app_decision_cfg);

    /* ----- 步骤 8b：开机自检的默认干态指纹（v2.1） -----
     * 先落一组保守默认值，保证上电第一秒就能区分"空腔"与"有水"（而不是一直 NO_CAL）。
     * 它只用于浸没判据；没有水中基线时仍是 FAULT/DRY → 阀恒排空。
     * 命令 D 或自检实测会把它升级为 MEASURED。 */
    (void)gate_set_dry_default(&g_dec.gate, APP_EC_DRY_DEFAULT_MV, APP_AO_DRY_DEFAULT_MV);
    g_ac_start_pending = (0 != APP_AUTOCAL_ENABLE);

    /* ----- 步骤 9：不再上电自动采"水中基线"（v2 P0-1）-----
     * 旧代码在电极露空的空气里自动采基线，等于把"空气"当成"清水本底"，
     * 这是现场"回收空气 / 有水但没淹到电极也回收"的直接成因（方案 §8.2 硬约束 1）。
     * v2.1 的开机自检只做安全的部分：自动干态（空气参考，安全）→ 等水 → 确认浸没后
     * 才自动采水中基线；全程无基线时阀恒排空。
     */
#if (1 == APP_AUTOCAL_ENABLE)
    printf("# AUTOCAL enabled: dry(%lus) -> wait water -> wet baseline (no input needed)\r\n",
           (unsigned long)(APP_AUTOCAL_DRY_MS / 1000U));
#else
    printf("# AUTOCAL disabled: send D (empty) then W (water) manually\r\n");
#endif
    printf("#   manual override: D=60s dry, W=2s wet, G=gate, Z=clear baseline\r\n");
    printf("#   until baseline: regime=FAULT/DRY -> valve stays DRAIN (safe)\r\n");

    /* ----- 步骤 10：打印参数摘要/帮助/CSV 表头 ----- */
    app_print_boot_summary();
    app_print_help();

    g_last_csv_ms          = timebase_ms();
    /* TIP 与 CSV 错开 ~50 ms，避免每 2 s 两行同时打印把 50 ms tick 拖长 */
    g_last_tip_ms          = timebase_ms() - (APP_TIP_PERIOD_MS - 50U);
    g_prev_event_active    = false;
    g_prev_state           = DEC_STATE_WAIT;
    g_prev_action          = DEC_ACTION_DRAIN;
    g_manual_event_started = false;
    g_manual_event_ended   = false;
    g_prev_valid_bits      = 0U;
    g_prev_regime          = REGIME_FAULT;

    /* ----- 主循环：timebase_ms 调度 50 ms tick；落后不补做（时间不堆积） ----- */
    now_ms       = timebase_ms();
    next_tick_ms = now_ms + APP_TICK_MS;

    while (1)
    {
        uint32_t now = timebase_ms();

        /* 无符号差值比较：< 0x80000000 表示"已到或已过 deadline"，跨 49.7 天回绕安全 */
        if ((uint32_t)(now - next_tick_ms) < 0x80000000U)
        {
            next_tick_ms += APP_TICK_MS;

            if ((uint32_t)(now - next_tick_ms) < 0x80000000U)
            {
                /* 落后超过一个 tick（例如串口大量打印/2 s 温度事务）：直接对齐当前时间，
                 * 不补打"欠下"的 tick——补做只会让积压越来越多。 */
                next_tick_ms = now + APP_TICK_MS;
            }

            app_tick(now);
        }
    }
}

