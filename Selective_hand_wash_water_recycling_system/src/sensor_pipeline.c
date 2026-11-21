


/*
 * sensor_pipeline.c —— 采集与特征流水线实现（双通道 ADC + 中值 + EMA + 基线 + Δ特征）
 * ---------------------------------------------------------------------------
 * 作用
 *   本文件实现 sensor_pipeline.h 的全部接口：每 50 ms 由 app_main 调用一次
 *   pipeline_sample()，完成一次"ADC 采样 → 驱动换算 → EMA 滤波 → 基线累计/结算 →
 *   相对特征 → 变化率 → note 状态"的完整流程；结果全部写进 pipeline_out_t。
 *   本模块不打印、不控阀、不做判别（判别在 decision.c），也不碰除 ADC 以外的硬件。
 *
 * 为什么"中值 + EMA"两段滤波（评审重点）
 *   ① 中值（默认 5 次/tick）：ADC 偶发单点野值（气泡、电源毛刺、调度抖动）只污染
 *      一个样本，中值直接把它剔除，均值则会被一个离群点拖偏。12-bit 单次转换是
 *      微秒量级，5 次的双通道扫描相对 50 ms tick 可忽略。
 *   ② EMA（alpha=0.15）：一级低通，时间常数约 -50 ms / ln(1-0.15) ≈ 307 ms
 *      （约 6 个 tick），把中值后剩下的白噪声继续压低（方差约降到 α/(2-α) 倍），
 *      又不像长窗口平均那样把阶跃边沿拉平——后面的 400 ms 块内一致消抖还需要
 *      看到边沿。两段串联 = "先剔野值、再压噪声"，都不需要额外缓冲。
 *   ③ 基线 σ 用"中值后、EMA 前"的原始样本统计，得到的是偏保守的噪声估计：
 *      EMA 平滑后的实际噪声更小，按 4σ/8σ 定的阈值不会偏小（安全侧）。
 *
 * 为什么变化率窗口取 8 点（400 ms）
 *   直接对相邻 tick 求差分会把噪声放大 sqrt(2)/Δt（Δt=50 ms），完全不可用；
 *   对 8 点做最小二乘直线拟合，等效于对斜率又做了一次 8 点平均，噪声显著下降；
 *   400 ms 又能保证"皂液到来"这种阶跃在 0.4 s 内被变化率抢先判据看到（判别总
 *   预算 ≤1.5 s）。窗口更长（如 1 s）延迟大，更短（如 3 点）噪声大，8 点 × 50 ms
 *   是折中；斜率再经 EMA(0.35) 平滑，压掉残余抖动。
 *
 * 时间预算（对齐《项目设想》6.2.1 与 00 文档）
 *   - 每 tick：adc_median_n 次双通道扫描（默认 5）。单次 12-bit 双通道扫描在微秒
 *     量级，5 次 + 换算 + 拟合的总开销远小于 1 ms，只占 50 ms tick 的很小部分；
 *   - 判别层消抖 400 ms（8 采 6 票）+ 3 块升级，变化率抢先不等待；端到端判定目标 ≤1.5 s（报告只写实测值）。
 *
 * 关键约定 / 注意事项
 *   - 所有电压单位 mV；ΔEC 用 25 ℃ 补偿后的 mv25；ΔNTU 用方向归一化后的电压差
 *     （>0 恒表示"比基线更脏"），输出字段不叫 NTU、不代表浊度绝对值；
 *   - 基线必须在水样（本底自来水）里采：探头在空气里是开路伪值（TDS 约 1.6 V），
 *     采出来的 EC0 没有物理意义；EC0 < 50 mV（模块未供电/探头悬空/输入接地）时
 *     判基线无效，判别层按安全侧排空；
 *   - 不使用 libm（sqrt 用 sensor_sqrtf）、不使用浮点格式化输出、不动态分配、
 *     不使用变长数组；
 *   - ADC 完成标志由 app_main 的 adc_callback 调用 pipeline_on_adc_complete() 置位，
 *     本文件用文件内 volatile 标志等待；主循环侧的固定顺序是
 *     "清标志 → R_ADC_ScanStart → 等标志 → R_ADC_Read 两通道"；
 *   - 必须在 ADC 打开（R_ADC_Open + R_ADC_ScanCfg）之后再调用 pipeline_init；
 *   - 本模块只依赖 sensor_common / sensor_tds / sensor_ts300b（通过 sensor_pipeline.h）。
 *
 * 实现备注（接口契约约束）
 *   sensor_pipeline_t 里只预留了一个 Δ 历史数组 rate_d[PIPELINE_RATE_POINTS]
 *   （接口契约 spec_headers.md 固定，不得改声明），但本模块要同时输出 ΔEC 与 ΔNTU
 *   两条变化率。本工程保证流水线单实例（app_main 只分配一个 sensor_pipeline_t，
 *   因为硬件只有一组探头），所以浊度 Δ 历史用文件内 static 数组 g_ntu_rate_d[]
 *   保存，与结构体里的 rate_t / rate_pos / rate_len 共享同一个时间环（两条曲线在
 *   同一 tick 同步入环，时间坐标天然一致）。若以后要支持多实例，必须给结构体增加
 *   ntu 历史数组并同步修改接口契约。
 */
#include "sensor_pipeline.h"

#include <string.h>   /* memset：清零句柄/静态缓冲；不做任何动态分配 */

/* ---------- 文件内常量与默认值 ---------- */
#define PIPELINE_RATE_EMA_ALPHA          0.35f   /* 变化率 EMA 系数：窗口已平滑，这里只压残余抖动 */
#define PIPELINE_RATE_MIN_SPAN_MS        150U    /* 变化率窗口最小时间跨度；不足时保持上次值 */
#define PIPELINE_MIN_VALID_EC0_MV        50.0f   /* EC0 有效下限（mV25）：低于此值判基线无效 */
#define PIPELINE_MAX_BASELINE_N          600U    /* 基线点数上限（30 s），防误配与浮点累加误差 */
#define PIPELINE_MIN_EMA_ALPHA           0.02f   /* EMA 系数下限：再小就几乎不跟随、阶跃看不到 */
#define PIPELINE_MIN_TIMEOUT_MS          1U      /* ADC 超时下限：0 会变成"立即超时" */
#define PIPELINE_ADC_SPIN_LIMIT          200000U /* ADC 等待自旋上限：时基未运行时的兜底 */
#define PIPELINE_DEFAULT_VREF_MV         3300U   /* 板载 3.3 V 参考 */
#define PIPELINE_DEFAULT_ADC_FULL_SCALE  4095U   /* 12 位 ADC 满量程 */
#define PIPELINE_DEFAULT_EMA_ALPHA       0.15f   /* 默认 EMA 系数（v2：0.30→0.15，配合 4 s 稳定期） */
#define PIPELINE_DEFAULT_TDS_CAL_K       1.0f    /* 无标准液时的标定系数 */
#define PIPELINE_DEFAULT_NTU_R1_OHM      1000.0f /* TS-300B 分压上臂默认 1 kΩ */
#define PIPELINE_DEFAULT_NTU_R2_OHM      2000.0f /* TS-300B 分压下臂默认 2 kΩ */
#define PIPELINE_DEFAULT_WATER_TEMP_C    25.0f   /* DS18B20 不可用且未手输时的兜底水温 */
#define PIPELINE_TEMP_MIN_C              (-20.0f) /* 与 DS18B20 量程一致，防异常值污染温补 */
#define PIPELINE_TEMP_MAX_C              80.0f
#define PIPELINE_DEFAULT_CV_MAX_PCT      1.0f    /* 基线稳定性：σ/EC0 ≤ 1 %（方案 §8.1） */
#define PIPELINE_DEFAULT_SIGMA_ABS_MAX_MV 3.0f   /* 基线稳定性兜底：σ ≤ 3 mV 也算稳 */
#define PIPELINE_DEFAULT_EC0_MAX_MV      2000.0f /* 基线量程上限：排除开路 ~1650 mV 伪值 */

/* ---------- 文件内静态状态 ---------- */
/* ADC 扫描完成标志：由 ADC 中断回调（adc_callback → pipeline_on_adc_complete）置位，
 * 由 pipeline_adc_read_pair 在启动扫描前清零。volatile 保证编译器不会把它优化进
 * 寄存器缓存（中断与主循环共享）。单核 Cortex-M33，不需要额外的内存屏障。 */
static volatile bool g_adc_scan_done = false;

/* 浊度 Δ 历史（见文件头"实现备注"：接口契约未给第二个历史数组，工程单实例）。
 * 与 p_pipe->rate_d / rate_t 同步入环，索引含义完全相同。 */
static float g_ntu_rate_d[PIPELINE_RATE_POINTS];

/* ---------- 内部工具函数 ---------- */

/* 把静态字符串安全拷进 out.note[]（定长 28 字节），并保证以 '\0' 结尾。 参数：p_dst  目标缓冲（out.note）；cap 缓冲容量（字节）；p_text 源字符串。
 * 返回：无。 边界  ：p_dst 或 p_text 为 NULL、cap=0 时直接返回（防御性检查）；超长截断。
 * 注意：note 由本模块独占写，上层只读。这里不用 strncpy，避免"源恰好填满时 不终止"的隐患，也不依赖字符串库的具体实现。 */
static void pipeline_note_set(char *p_dst, uint32_t cap, const char *p_text)
{
    uint32_t i = 0U;

    if ((NULL == p_dst) || (NULL == p_text) || (0U == cap))
    {
        return;
    }

    while (((i + 1U) < cap) && ('\0' != p_text[i]))
    {
        p_dst[i] = p_text[i];
        i++;
    }
    p_dst[i] = '\0';
}

/* 按固定优先级刷新 out.note： ADC_TIMEOUT > TDS_RAIL > TDS_SAT > NTU_RAIL > NO_BASE > OK。
 * 注意：判别层可以只看 note + adc_timeout + baseline_valid，不必重复拼优先级； 这里把"传感器故障"放在"无基线"之前，是因为故障时基线也不可信，先报故障 更利于排障。 */
static void pipeline_note_update(sensor_pipeline_t *p_pipe)
{
    const char *p_note;

    if (p_pipe->out.adc_timeout)
    {
        /* 最高优先级：本 tick 根本没有可信数据，其它标志都没有意义 */
        p_note = "ADC_TIMEOUT";
    }
    else if (p_pipe->out.tds_rail)
    {
        /* TDS 贴轨：几乎必然是接错线/过压/模块输出异常，先报 TDS_RAIL 最利于排障 */
        p_note = "TDS_RAIL";
    }
    else if (p_pipe->out.tds_saturated)
    {
        /* 未贴轨但超过模块标称量程：读数可能被压缩，标 TDS_SAT 提醒"相对量也可能失真" */
        p_note = "TDS_SAT";
    }
    else if (SENSOR_OK != p_pipe->out.tds_status)
    {
        /* 走到这里说明 TDS 状态异常但未置 rail 标志（异常路径兜底），归入 TDS_RAIL 族；
         * 放在 NTU 之前是为了保持"先主判据通道、后辅判据通道"的排障顺序 */
        p_note = "TDS_RAIL";
    }
    else if (SENSOR_OK != p_pipe->out.ntu_status)
    {
        /* 浊度驱动只有 OK / ERR_RANGE（贴轨或疑似断线）两种实用返回，
         * 非 OK 统一归入 NTU_RAIL 族提示（具体电压在 ntu_node_mv 里可见） */
        p_note = "NTU_RAIL";
    }
    else if (!p_pipe->out.baseline_valid)
    {
        /* 传感器都正常，但没有可用本底：判别层必须按安全侧排空（fail-safe）。
         * v2：区分"从未标定（NO_BASE）"与"标定被拒/被作废（BASE_BAD / CLEARED）"，
         * 后者把具体原因直接写进 note，现场一眼能看出是"采集条件不对"而不是"忘了标定"。 */
        p_note = (NULL != p_pipe->baseline_reject) ? p_pipe->baseline_reject : "NO_BASE";
    }
    else
    {
        p_note = "OK";
    }

    pipeline_note_set(p_pipe->out.note, (uint32_t) sizeof(p_pipe->out.note), p_note);
}

/* 对 app 传入的流水线配置做工程限幅，杜绝 0/负数/NaN 造成除零、死等、 长时间采不出基线等运行期问题。 返回：无。 边界  ：指针由调用者保证非 NULL；只改本模块自己持有的副本，不回写 app 的配置。
 * 注意：限幅规则：adc_median_n 1~9、adc_timeout_ms ≥1、ema_alpha 0.02~1、 baseline_n 1~600、σ 下限 ≥0；vref/full_scale/分压电阻/cal_k 为 0 或 非法时回退默认值（这些值直接参与除法，必须防 0）。 */
static void pipeline_clamp_cfg(pipeline_cfg_t *p_cfg)
{
    /* 中值次数：超过 9 会超出 PIPELINE_MAX_MEDIAN_N 的采样数组，必须限住；
     * 0 次则没有样本可取，取 1（退化为单点）。
     * uint16 的取值全程能被 float 精确表示，这里借用 sensor_clampf 的上下限语义，
     * 与原来手写的"两行 if"逐值等价（该处没有 NaN 兜底、也没有 0 回落）。 */
    p_cfg->adc_median_n = (uint16_t) sensor_clampf((float) p_cfg->adc_median_n, 1.0f,
                                                   (float) PIPELINE_MAX_MEDIAN_N);

    /* 超时 0 ms 会立刻判超时（每次采样都失败），按 1 ms 处理 */
    if (p_cfg->adc_timeout_ms < PIPELINE_MIN_TIMEOUT_MS)
    {
        p_cfg->adc_timeout_ms = PIPELINE_MIN_TIMEOUT_MS;
    }

    /* vref / full_scale 参与 ADC→mV 的除法（在 sensor_tds/ts300b 内部），不能为 0 */
    /* vref / full_scale 是接口契约里的配置项：本模块把 ADC 码→mV 的换算交给
     * sensor_tds / sensor_ts300b 完成（两个驱动内部用同值的 3300 mV / 4095 默认宏），
     * 这里只做"非 0 限幅"，保证 app 误传 0 时不会在别处埋下除以 0 的隐患。 */
    if (0U == p_cfg->vref_mv)
    {
        p_cfg->vref_mv = PIPELINE_DEFAULT_VREF_MV;
    }
    if (0U == p_cfg->adc_full_scale)
    {
        p_cfg->adc_full_scale = PIPELINE_DEFAULT_ADC_FULL_SCALE;
    }

    /* EMA 系数：NaN 会污染整条滤波链（用 sensor_is_nan_f32 按位挡住 NaN），
     * 下限 0.02 防止"几乎不跟随"导致阶跃永远看不到，上限 1 = 不平滑（允许，调试用）。 */
    {
        float alpha = p_cfg->ema_alpha;

        if (sensor_is_nan_f32(alpha))
        {
            alpha = PIPELINE_DEFAULT_EMA_ALPHA;
        }
        if (alpha < PIPELINE_MIN_EMA_ALPHA)
        {
            alpha = PIPELINE_MIN_EMA_ALPHA;
        }
        if (alpha > 1.0f)
        {
            alpha = 1.0f;
        }
        p_cfg->ema_alpha = alpha;
    }

    /* 标定系数：<=0 或 NaN 会让参考 ppm 变成负值/NaN，回退 1.0 */
    if (!(p_cfg->tds_cal_k > 0.0f))
    {
        p_cfg->tds_cal_k = PIPELINE_DEFAULT_TDS_CAL_K;
    }

    /* 浊度分压：R2（下臂）必须 > 0（做除法），R1（上臂）可为 0（不分压）；
     * NaN 用 "!(x >= 0)" / "!(x > 0)" 的判断一并挡掉 */
    if (!(p_cfg->ntu_div_r1_ohm >= 0.0f))
    {
        p_cfg->ntu_div_r1_ohm = PIPELINE_DEFAULT_NTU_R1_OHM;
    }
    if (!(p_cfg->ntu_div_r2_ohm > 0.0f))
    {
        p_cfg->ntu_div_r2_ohm = PIPELINE_DEFAULT_NTU_R2_OHM;
    }

    /* 基线点数：至少 1 点（否则永远采不满）；上限 600（30 s）防误配把系统卡在
     * "一直采基线"的状态，也限制 float 平方和累加的误差积累。
     * 同 adc_median_n：uint16 全程精确，sensor_clampf 与手写上下限逐值等价。 */
    p_cfg->baseline_n = (uint16_t) sensor_clampf((float) p_cfg->baseline_n, 1.0f,
                                                 (float) PIPELINE_MAX_BASELINE_N);

    /* σ 下限：负值无意义（会让自适应阈值比噪声还小），NaN 也回退 0 */
    if (!(p_cfg->sigma_floor_ec_mv >= 0.0f))
    {
        p_cfg->sigma_floor_ec_mv = 0.0f;
    }
    if (!(p_cfg->sigma_floor_ntu_mv >= 0.0f))
    {
        p_cfg->sigma_floor_ntu_mv = 0.0f;
    }

    /* 基线有效性校验参数（v2 新增）：0/负/NaN 一律回落默认 */
    if (!(p_cfg->baseline_cv_max_pct > 0.0f) || (p_cfg->baseline_cv_max_pct > 100.0f))
    {
        p_cfg->baseline_cv_max_pct = PIPELINE_DEFAULT_CV_MAX_PCT;
    }
    if (!(p_cfg->baseline_sigma_abs_max_mv >= 0.0f))
    {
        p_cfg->baseline_sigma_abs_max_mv = PIPELINE_DEFAULT_SIGMA_ABS_MAX_MV;
    }
    if (!(p_cfg->ec0_max_mv > PIPELINE_MIN_VALID_EC0_MV))
    {
        p_cfg->ec0_max_mv = PIPELINE_DEFAULT_EC0_MAX_MV;
    }
}

/* 把"本 tick 没有可信数据"统一记到 out 上：adc_timeout、两路状态、原始量清零、 note 置 ADC_TIMEOUT；滤波值/基线量保持上一次的有效值不动。
 * 注意：这里把"原始量清零"而不是保留旧值，是为了让 CSV 一眼看出这一行无效； 特征/滤波量保留旧值并被 adc_timeout 标记为不可信，判别层优先判故障， 不会误用。 */
static void pipeline_mark_data_invalid(sensor_pipeline_t *p_pipe)
{
    pipeline_out_t *p_out = &p_pipe->out;

    p_out->adc_timeout = true;
    p_out->tds_raw = 0U;
    p_out->tds_mv = 0U;
    p_out->tds_mv25 = 0.0f;
    p_out->tds_ppm25 = 0.0f;
    p_out->ntu_raw = 0U;
    p_out->ntu_node_mv = 0U;
    p_out->ntu_ao_mv = 0U;
    p_out->tds_rail = false;
    p_out->tds_saturated = false;
    p_out->tds_status = SENSOR_ERR_TIMEOUT;
    p_out->ntu_status = SENSOR_ERR_TIMEOUT;

    /* 状态量变了，note 立即按优先级刷新（此时一定是 ADC_TIMEOUT） */
    pipeline_note_update(p_pipe);
}

/* 执行 adc_median_n 轮"清标志 → 启动扫描 → 等完成 → 读两通道"，然后对两个 通道各取中值。
 * 参数：p_pipe      流水线句柄（用 cfg.p_adc_ctrl / 通道号 / 超时）； p_tds_raw   TDS 通道中值输出（非 NULL）； p_ntu_raw   浊度通道中值输出（非 NULL）。
 * 时序/边界：ADC0 配置为"单次扫描组"（AN11+AN12）：一次 R_ADC_ScanStart 会把组内两个 通道都转换一遍，所以循环 N 次即可同时得到两个通道各 N 个独立样本； 每个样本都在"等回调完成"之后才 R_ADC_Read，读到的一定是本次转换结果。 边界  ：单轮等待上限 adc_timeout_ms（默认 5 ms）。另外设置自旋次数上限兜底： 若 SysTick 未启动/时基异常，timebase_ms() 恒为 0，仅靠毫秒判断会死等。
 * 注意：清标志必须放在 R_ADC_ScanStart 之前，否则可能把"上一轮扫描的完成标志" 当成本轮完成（第一次进入时也会残留初值）；启动失败也按失败上报， 由上层按 ADC 故障处理（安全侧）。 */
static bool pipeline_adc_read_pair(sensor_pipeline_t *p_pipe, uint16_t *p_tds_raw, uint16_t *p_ntu_raw)
{
    uint16_t tds_samples[PIPELINE_MAX_MEDIAN_N];   /* 固定大小、编译期常量，非变长数组 */
    uint16_t ntu_samples[PIPELINE_MAX_MEDIAN_N];
    adc_instance_ctrl_t *p_ctrl = p_pipe->cfg.p_adc_ctrl;
    uint16_t n = p_pipe->cfg.adc_median_n;
    uint16_t i;

    *p_tds_raw = 0U;
    *p_ntu_raw = 0U;

    /* 配置里已限幅，但这里再兜一次：防止未来有人绕过 pipeline_init 直接改 cfg */
    if (NULL == p_ctrl)
    {
        return false;
    }
    if (n < 1)
    {
        n = 1U;
    }
    if (n > (uint16_t) PIPELINE_MAX_MEDIAN_N)
    {
        n = PIPELINE_MAX_MEDIAN_N;
    }

    for (i = 0U; i < n; i++)
    {
        uint32_t t_start;
        uint32_t spin = 0U;

        g_adc_scan_done = false;   /* 见"注意"：必须先清后启动 */

        if (FSP_SUCCESS != R_ADC_ScanStart(p_ctrl))
        {
            return false;
        }

        t_start = timebase_ms();
        while (!g_adc_scan_done)
        {
            /* 主判断：毫秒级超时（默认 5 ms，远大于一次双通道转换时间） */
            if ((timebase_ms() - t_start) >= p_pipe->cfg.adc_timeout_ms)
            {
                break;
            }

            /* 兜底判断：时基没跑起来时上面永远不会成立，用自旋上限避免死循环。
             * 200000 次空循环在 48 MHz 下是毫秒量级，正常情况下根本到不了。 */
            spin++;
            if (spin >= PIPELINE_ADC_SPIN_LIMIT)
            {
                break;
            }
        }

        if (!g_adc_scan_done)
        {
            return false;
        }

        /* 单次扫描组已完成，两个通道的结果都在寄存器里；Clear-after-read 模式
         * 保证每读一次清一次，不会把上一次结果读两遍。任一读取失败都按本次失败处理。 */
        if (FSP_SUCCESS != R_ADC_Read(p_ctrl, p_pipe->cfg.tds_channel, &tds_samples[i]))
        {
            return false;
        }
        if (FSP_SUCCESS != R_ADC_Read(p_ctrl, p_pipe->cfg.ntu_channel, &ntu_samples[i]))
        {
            return false;
        }
    }

    /* 中值 = 排序后取中间（n 为偶数时取中间偏后，和 sensor_common 的约定一致）；
     * 比均值抗野值，单点毛刺不会传到后面的 EMA/判别。 */
    *p_tds_raw = sensor_median_u16(tds_samples, n);
    *p_ntu_raw = sensor_median_u16(ntu_samples, n);
    return true;
}

/* 清空变化率环形窗口与两条速率的 EMA（初始化/重新采基线时调用）。 返回：无。 边界  ：同时清结构体里的 EC 历史与文件内静态的浊度历史；只在"基线换了"或 "初始化"时调用，不影响正常 tick。
 * 注意：重新采基线后旧的 Δ 历史没有意义（旧基线与新基线的 Δ 基准不同）， 必须作废，否则会拿旧基线的斜率去触发新基线的变化率抢先。 */
static void pipeline_rate_reset(sensor_pipeline_t *p_pipe)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t) PIPELINE_RATE_POINTS; i++)
    {
        p_pipe->rate_d[i] = 0.0f;
        p_pipe->rate_t[i] = 0U;
        g_ntu_rate_d[i] = 0.0f;
    }
    p_pipe->rate_pos = 0U;
    p_pipe->rate_len = 0U;
    p_pipe->rate_ec_ema = 0.0f;
    p_pipe->rate_ntu_ema = 0.0f;
}

/* 对环形窗口内的 (t, Δ) 点做最小二乘直线拟合，求斜率（mV/s）。
 * 参数：p_values Δ 历史数组；p_times 时间历史数组；len 有效点数； oldest 最旧点索引；now_ms 当前时刻（最新点的时间）； p_slope 斜率输出（非 NULL）。
 * 注意：用最小二乘而不是首尾两点差分：8 个点都参与平均，斜率噪声明显更小； 两遍 O(8) 循环，不需要额外数组或动态内存。 */
static bool pipeline_rate_slope(const float *p_values, const uint32_t *p_times,
                                uint8_t len, uint8_t oldest, uint32_t now_ms, float *p_slope)
{
    float   sum_t = 0.0f;
    float   sum_d = 0.0f;
    float   mean_t;
    float   mean_d;
    float   num = 0.0f;
    float   den = 0.0f;
    uint8_t i;

    if ((NULL == p_values) || (NULL == p_times) || (NULL == p_slope))
    {
        return false;
    }
    if (len < 2)
    {
        return false;
    }
    if ((now_ms - p_times[oldest]) < PIPELINE_RATE_MIN_SPAN_MS)
    {
        /* 窗口还没攒够 150 ms（例如刚上电/刚换基线）时，回归斜率不可信，
         * 不给新值，让调用者保持上一次输出（首次为 0），避免变化率抢先误触发。 */
        return false;
    }

    /* 第一遍：均值（t 以最新时刻为 0，向前取负） */
    for (i = 0U; i < len; i++)
    {
        uint8_t idx = (uint8_t) (((uint16_t) oldest + i) % PIPELINE_RATE_POINTS);
        float   t = -((float) (now_ms - p_times[idx])) * 0.001f;

        sum_t += t;
        sum_d += p_values[idx];
    }
    mean_t = sum_t / (float) len;
    mean_d = sum_d / (float) len;

    /* 第二遍：Σ(t-t̄)(d-d̄) 与 Σ(t-t̄)²，斜率 = 前者/后者 */
    for (i = 0U; i < len; i++)
    {
        uint8_t idx = (uint8_t) (((uint16_t) oldest + i) % PIPELINE_RATE_POINTS);
        float   t = -((float) (now_ms - p_times[idx])) * 0.001f;
        float   dt = t - mean_t;

        num += dt * (p_values[idx] - mean_d);
        den += dt * dt;
    }

    if (den <= 1.0e-9f)
    {
        /* 所有点时间相同（调度异常/同一 now_ms 重复调用）时无法求斜率 */
        return false;
    }

    *p_slope = num / den;
    return true;
}

/* 把本 tick 的 ΔEC / ΔNTU 压入 8 点环形窗口，分别算两条变化率并做 EMA 平滑， 结果写入 out.ec_rate_mv_s / out.ntu_rate_mv_s。
 * 注意：EC 历史存结构体 rate_d（接口契约给的唯一 Δ 数组），浊度历史存文件内 静态数组（见文件头"实现备注"），两者与 rate_t 共用同一环形索引； rate_pos 指向"下一个写入位"，最旧点由 (pos - len) 取模得到。 */
static void pipeline_rate_update(sensor_pipeline_t *p_pipe, uint32_t now_ms)
{
    uint8_t pos;
    uint8_t len;
    uint8_t oldest;
    float   slope = 0.0f;

    pos = p_pipe->rate_pos;
    p_pipe->rate_t[pos] = now_ms;
    p_pipe->rate_d[pos] = p_pipe->out.d_ec_mv;
    g_ntu_rate_d[pos] = p_pipe->out.d_ntu_mv;

    p_pipe->rate_pos = (uint8_t) ((pos + 1U) % PIPELINE_RATE_POINTS);
    if (p_pipe->rate_len < (uint8_t) PIPELINE_RATE_POINTS)
    {
        p_pipe->rate_len++;
    }
    len = p_pipe->rate_len;
    oldest = (uint8_t) (((uint16_t) p_pipe->rate_pos + PIPELINE_RATE_POINTS - len) % PIPELINE_RATE_POINTS);

    /* 两条通道独立求斜率；哪条暂时不可信就保持哪条的上次值 */
    if (pipeline_rate_slope(p_pipe->rate_d, p_pipe->rate_t, len, oldest, now_ms, &slope))
    {
        p_pipe->rate_ec_ema = sensor_ema(p_pipe->rate_ec_ema, slope, PIPELINE_RATE_EMA_ALPHA);
    }
    if (pipeline_rate_slope(g_ntu_rate_d, p_pipe->rate_t, len, oldest, now_ms, &slope))
    {
        p_pipe->rate_ntu_ema = sensor_ema(p_pipe->rate_ntu_ema, slope, PIPELINE_RATE_EMA_ALPHA);
    }

    /* 无论本 tick 是否算出了新斜率，都输出当前 EMA 值（即"保持上次值"的语义） */
    p_pipe->out.ec_rate_mv_s = p_pipe->rate_ec_ema;
    p_pipe->out.ntu_rate_mv_s = p_pipe->rate_ntu_ema;
}

/* 基线采满 baseline_n 点后结算：均值 EC0 / NTU0、n-1 样本标准差 σ（带下限）、 有效性判定、EMA 重置，并置 baseline_updated（只在这一 tick 为 true）。
 * 注意：σ 用"中值后、EMA 前"的每 tick 样本统计——比 EMA 后的噪声略大，判据偏保守； 基线有效条件只看 EC0（主判据）：EC0 < 50 mV 视为模块未供电/探头悬空/接地， 此时整个基线与特征都不可用。有效基线才重置 EMA，无效时保持当前滤波状态， 避免把悬空伪值当成滤波起点。 */
static void pipeline_baseline_finish(sensor_pipeline_t *p_pipe)
{
    pipeline_out_t *p_out = &p_pipe->out;
    uint16_t n = p_pipe->baseline_count;
    float    n_f = (float) n;
    float    mean_ec = p_pipe->base_ec_sum / n_f;
    float    mean_ntu = p_pipe->base_ntu_sum / n_f;
    float    var_ec = 0.0f;
    float    var_ntu = 0.0f;
    float    sigma_ec;
    float    sigma_ntu;
    bool     valid;

    if (n >= 2)
    {
        /* σ² = (Σx² - n·x̄²)/(n-1)：n=20、mV 量级下 float 累加的误差远小于 σ 下限，
         * 且 σ 只用于阈值自适应并还有绝对下限兜底，精度足够。 */
        var_ec = (p_pipe->base_ec_sumsq - (n_f * mean_ec * mean_ec)) / ((float) (n - 1U));
        var_ntu = (p_pipe->base_ntu_sumsq - (n_f * mean_ntu * mean_ntu)) / ((float) (n - 1U));
    }

    /* 浮点误差可能让方差算成很小的负数，钳到 0（sqrt(负)=NaN 会污染阈值） */
    if (var_ec < 0.0f)
    {
        var_ec = 0.0f;
    }
    if (var_ntu < 0.0f)
    {
        var_ntu = 0.0f;
    }

    sigma_ec = sensor_sqrtf(var_ec);
    sigma_ntu = sensor_sqrtf(var_ntu);

    /* σ 下限：防止基线"过干净"时 σ≈0 让 d1/d2 退化为 0，阈值附近疯狂抖动 */
    if (sigma_ec < p_pipe->cfg.sigma_floor_ec_mv)
    {
        sigma_ec = p_pipe->cfg.sigma_floor_ec_mv;
    }
    if (sigma_ntu < p_pipe->cfg.sigma_floor_ntu_mv)
    {
        sigma_ntu = p_pipe->cfg.sigma_floor_ntu_mv;
    }

    p_out->ec0_mv25 = mean_ec;
    /* AO 电压用整数 mV 输出（与 out.ntu0_ao_mv 的类型一致，四舍五入） */
    p_out->ntu0_ao_mv = (uint32_t) (mean_ntu + 0.5f);
    p_out->ec_sigma_mv = sigma_ec;
    p_out->ntu_sigma_mv = sigma_ntu;
    p_out->baseline_n = n;

    /* EC0 过小 = 模块没供电 / 探头悬空 / 输入端接地；过大 = 开路伪值（~1650 mV）：
     * 这样的"本底"都没有物理意义（方案 §8.2 硬约束 2 的量化版）。 */
    valid = (mean_ec >= PIPELINE_MIN_VALID_EC0_MV) && (mean_ec <= p_pipe->cfg.ec0_max_mv);
    p_pipe->baseline_reject = NULL;

    if ((valid) && (n >= 2U))
    {
        /* 稳定性校验（方案 §8.1）：σ/EC0 ≤ CV 上限，或 σ 本身已经很小（绝对值兜底）。
         * 纯水 EC0 只有几十 mV 时，1 % 相对判据低于传感器噪声下限，必须允许"σ 很小"通过。 */
        float cv_pct = (sigma_ec * 100.0f) / mean_ec;

        if ((cv_pct > p_pipe->cfg.baseline_cv_max_pct) &&
            (sigma_ec > p_pipe->cfg.baseline_sigma_abs_max_mv))
        {
            valid = false;
            p_pipe->baseline_reject = "BASE_NOISY";
        }
    }

    if ((!valid) && (NULL == p_pipe->baseline_reject))
    {
        p_pipe->baseline_reject = (mean_ec < PIPELINE_MIN_VALID_EC0_MV) ? "BASE_LOW" : "BASE_HIGH";
    }

    p_out->baseline_valid = valid;
    p_out->baseline_updated = true;   /* 只在本 tick 为 true，app 据此打印一次日志 */
    p_pipe->baseline_capturing = false;

    if (valid)
    {
        /* 新基线生效：把 EMA 重置到本底值，下一 tick 的 Δ 从 0 附近开始，
         * 避免旧滤波状态与新基线之间出现无意义的阶跃。 */
        p_pipe->ec_ema = mean_ec;
        p_pipe->ntu_ema = mean_ntu;
    }
}

/* 由 EMA 后的 mv25 / AO 与本底 EC0 / NTU0 计算全部相对特征与变化率占位。
 * 注意：ΔEC 用 mv25(EMA)-EC0；g 用同一个 EMA 值（与 d_ec 自洽：g-1 ≈ d_ec_pct/100）； 方向归一化：默认 ntu_clear_high=true（清水 AO 电压高、越脏越低）时 ΔNTU = NTU0-AO，反相批次保持"清水电压低"则 ΔNTU = AO-NTU0， 这样 >0 恒表示"比基线更脏"，判别层不用关心模块批次方向。 */
static void pipeline_features_update(sensor_pipeline_t *p_pipe)
{
    pipeline_out_t *p_out = &p_pipe->out;
    float ec0 = p_out->ec0_mv25;
    float ntu0 = (float) p_out->ntu0_ao_mv;

    if (!p_out->baseline_valid)
    {
        /* 无基线：所有"相对本底"的量都没有意义，统一清 0（配合 note=NO_BASE） */
        p_out->d_ec_mv = 0.0f;
        p_out->g_ec = 0.0f;
        p_out->d_ec_pct = 0.0f;
        p_out->d_ntu_mv = 0.0f;
        p_out->d_ntu_pct = 0.0f;
        p_out->ec_rate_mv_s = 0.0f;
        p_out->ntu_rate_mv_s = 0.0f;
        return;
    }

    p_out->d_ec_mv = p_pipe->ec_ema - ec0;
    if (ec0 > 0.0f)
    {
        p_out->g_ec = p_pipe->ec_ema / ec0;
        p_out->d_ec_pct = (p_out->d_ec_mv / ec0) * 100.0f;
    }
    else
    {
        /* 基线有效时 ec0 ≥ 50 mV，这里只是数学上的防御，避免任何情况下除零 */
        p_out->g_ec = 0.0f;
        p_out->d_ec_pct = 0.0f;
    }

    if (p_pipe->cfg.ntu_clear_high)
    {
        p_out->d_ntu_mv = ntu0 - p_pipe->ntu_ema;   /* 清水电压高：变脏 → 电压降低 → Δ>0 */
    }
    else
    {
        p_out->d_ntu_mv = p_pipe->ntu_ema - ntu0;   /* 反相批次：变脏 → 电压升高 → Δ>0 */
    }

    if (ntu0 > 0.0f)
    {
        p_out->d_ntu_pct = (p_out->d_ntu_mv / ntu0) * 100.0f;
    }
    else
    {
        /* 浊度 AO=0（未供电/断线）：百分比没有意义，置 0；note 会报 NTU_RAIL */
        p_out->d_ntu_pct = 0.0f;
    }
}

/* ---------- 公共接口 ---------- */

/* 初始化流水线句柄：清零状态、拷贝并限幅配置、初始化下游 TDS / TS-300B 驱动、 建立"无基线"的初始输出（此时 note=NO_BASE，判别层按安全侧处理）。
 * 参数：p_pipe 由 app_main 分配的流水线句柄（非 NULL）； p_cfg  配置（非 NULL；内部拷贝一份，之后 app 再改自己的副本不影响本模块）。
 * 返回：无（void）。p_pipe / p_cfg 为 NULL 时直接返回。
 * 时序/边界：必须在 R_ADC_Open + R_ADC_ScanCfg 之后、第一次 pipeline_sample 之前调用； DS18B20 温度可以稍后再接上（水温默认 25 ℃，由 pipeline_set_water_temp 更新）。
 * 注意：句柄整体清零是为了杜绝"结构体里的脏值被当成早期输出"；随后显式给 状态/水温/note 赋初值。ADC 回调里必须调用 pipeline_on_adc_complete()， 否则首次 pipeline_sample 会等超时。 */
void pipeline_init(sensor_pipeline_t *p_pipe, const pipeline_cfg_t *p_cfg)
{
    uint8_t i;

    if ((NULL == p_pipe) || (NULL == p_cfg))
    {
        return;
    }

    /* 先把整个句柄清零：所有输出字段从 0/false 开始，避免脏值 */
    (void) memset(p_pipe, 0, sizeof(*p_pipe));
    (void) memset(g_ntu_rate_d, 0, sizeof(g_ntu_rate_d));

    /* 拷贝配置后立即做工程限幅（只改本模块的副本） */
    p_pipe->cfg = *p_cfg;
    pipeline_clamp_cfg(&p_pipe->cfg);

    /* 把驱动配置存进句柄里的 storage 并初始化驱动（驱动内部会再拷一份 static 配置） */
    p_pipe->tds_cfg_storage.cal_k = p_pipe->cfg.tds_cal_k;
    tds_init(&p_pipe->tds_cfg_storage);

    p_pipe->ntu_cfg_storage.div_r1_ohm = p_pipe->cfg.ntu_div_r1_ohm;
    p_pipe->ntu_cfg_storage.div_r2_ohm = p_pipe->cfg.ntu_div_r2_ohm;
    p_pipe->ntu_cfg_storage.clear_high = p_pipe->cfg.ntu_clear_high;
    ts300b_init(&p_pipe->ntu_cfg_storage);

    /* 水温默认 25 ℃、来源=兜底（app 每 tick 会用实际 DS18B20/手输值覆盖） */
    p_pipe->water_temp_c = PIPELINE_DEFAULT_WATER_TEMP_C;
    p_pipe->water_temp_from_sensor = false;

    /* EMA 初值 0：约定"0 表示尚未初始化"，首个有效样本直接采纳（见 pipeline_sample） */
    p_pipe->ec_ema = 0.0f;
    p_pipe->ntu_ema = 0.0f;

    /* 变化率环形窗口清零（memset 已清，这里再按契约显式初始化一次，语义更清楚） */
    for (i = 0U; i < (uint8_t) PIPELINE_RATE_POINTS; i++)
    {
        p_pipe->rate_d[i] = 0.0f;
        p_pipe->rate_t[i] = 0U;
    }
    p_pipe->rate_pos = 0U;
    p_pipe->rate_len = 0U;
    p_pipe->rate_ec_ema = 0.0f;
    p_pipe->rate_ntu_ema = 0.0f;

    /* 输出初值：还没采过样，状态按"未初始化"上报，提示 waiting；note 报无基线 */
    p_pipe->out.tds_status = SENSOR_ERR_INIT;
    p_pipe->out.ntu_status = SENSOR_ERR_INIT;
    p_pipe->out.adc_timeout = false;
    p_pipe->out.baseline_valid = false;
    p_pipe->out.baseline_updated = false;
    p_pipe->out.baseline_n = 0U;
    p_pipe->out.ec0_mv25 = 0.0f;
    p_pipe->out.ntu0_ao_mv = 0U;
    p_pipe->out.ec_sigma_mv = 0.0f;
    p_pipe->out.ntu_sigma_mv = 0.0f;
    p_pipe->out.water_temp_c = p_pipe->water_temp_c;
    p_pipe->out.water_temp_from_sensor = false;
    pipeline_note_set(p_pipe->out.note, (uint32_t) sizeof(p_pipe->out.note), "NO_BASE");

    p_pipe->initialized = true;
}

/* ADC 扫描完成通知：只置位文件内的完成标志。
 * 时序/边界：由 app_main 的 adc_callback（ADC_EVENT_SCAN_COMPLETE）在中断上下文调用； 主循环 pipeline_adc_read_pair 在启动扫描前清标志、随后轮询。
 * 注意：ISR 里只做一次 volatile 赋值：不调用 FSP 读函数、不打印、不加锁， 保证中断尽快返回；真正的 R_ADC_Read 由主循环在标志置位后完成。 */
void pipeline_on_adc_complete(void)
{
    g_adc_scan_done = true;
}

/* 开始（或重新开始）一次基线采集：清计数与累加和、立即作废旧基线、清变化率历史。
 * 时序/边界：上电 pipeline_init 之后、以及串口命令 B（探头须浸在本底水中）时调用； 之后每个 tick 的 pipeline_sample 累计 1 个样本，采满 baseline_n 点自动结算。
 * 注意：now_ms 当前不参与计算（结构体里没有存"采集开始时刻"的字段），保留该参数 是为了与接口契约一致、并为将来打印"# BASELINE start"日志留位置；日志时间戳 由 app_main 用自己的 now_ms 打印。 旧基线在重新采集期间立即作废（baseline_valid=false）：宁可 1 s 排空，也不用 "半新半旧"的基线做判别；采满后新基线一次性启用。 */
void pipeline_baseline_begin(sensor_pipeline_t *p_pipe, uint32_t now_ms)
{
    (void) now_ms;

    if ((NULL == p_pipe) || (!p_pipe->initialized))
    {
        return;
    }

    p_pipe->baseline_capturing = true;
    p_pipe->baseline_count = 0U;
    p_pipe->baseline_reject = NULL;   /* 重新采集：清掉上一次的作废原因 */
    p_pipe->base_ec_sum = 0.0f;
    p_pipe->base_ec_sumsq = 0.0f;
    p_pipe->base_ntu_sum = 0.0f;
    p_pipe->base_ntu_sumsq = 0.0f;

    p_pipe->out.baseline_valid = false;   /* 见"注意"：采集期间按无基线走安全侧 */
    p_pipe->out.baseline_updated = false;
    p_pipe->out.baseline_n = 0U;
    p_pipe->out.ec0_mv25 = 0.0f;
    p_pipe->out.ntu0_ao_mv = 0U;
    p_pipe->out.ec_sigma_mv = 0.0f;
    p_pipe->out.ntu_sigma_mv = 0.0f;
    p_pipe->out.d_ec_mv = 0.0f;
    p_pipe->out.g_ec = 0.0f;
    p_pipe->out.d_ec_pct = 0.0f;
    p_pipe->out.d_ntu_mv = 0.0f;
    p_pipe->out.d_ntu_pct = 0.0f;
    p_pipe->out.ec_rate_mv_s = 0.0f;
    p_pipe->out.ntu_rate_mv_s = 0.0f;

    /* 旧 Δ 的变化率历史属于旧基线，立即作废（否则会用旧基线的斜率误触发抢先判据） */
    pipeline_rate_reset(p_pipe);

    pipeline_note_set(p_pipe->out.note, (uint32_t) sizeof(p_pipe->out.note), "NO_BASE");
}

/* 查询是否正在采集基线（app 用来打印进度/避免重复触发）。 返回：true=正在采集；false=未采集或句柄为 NULL。 边界  ：只读，不修改状态；可在任意上下文调用。
 * 注意：采集完成后本函数返回 false，结果看 out.baseline_updated / baseline_valid。 */
bool pipeline_baseline_capturing(const sensor_pipeline_t *p_pipe)
{
    if (NULL == p_pipe)
    {
        return false;
    }
    return p_pipe->baseline_capturing;
}

/* 作废当前基线（保留 ec0 供诊断，但 baseline_valid=false、相对特征清零）。 参数：p_pipe —— 流水线句柄；p_note —— 作废原因静态串（NULL 时用 "CLEARED"）。
 * 时序/边界：命令 Z、以及 W 采集完成后"EC0 < 3×干态"等校验失败时调用； 下一 tick 起 decision 的工况会离开 MEASURE（安全侧）。
 * 注意：与 pipeline_baseline_begin 的区别——后者重新开始采集，本函数只作废； 作废原因写进 note（BASE_LOW / BASE_HIGH / BASE_NOISY / CLEARED）， 现场一眼能区分"没标定"与"标定了但条件不对"。 */
void pipeline_invalidate_baseline(sensor_pipeline_t *p_pipe, const char *p_note)
{
    pipeline_out_t *p_out;

    if ((NULL == p_pipe) || (!p_pipe->initialized))
    {
        return;
    }
    p_out = &p_pipe->out;

    p_pipe->baseline_capturing = false;
    p_pipe->baseline_count     = 0U;
    p_pipe->baseline_reject    = (NULL != p_note) ? p_note : "CLEARED";

    p_out->baseline_valid   = false;
    p_out->baseline_updated = false;
    p_out->d_ec_mv          = 0.0f;
    p_out->g_ec             = 0.0f;
    p_out->d_ec_pct         = 0.0f;
    p_out->d_ntu_mv         = 0.0f;
    p_out->d_ntu_pct        = 0.0f;
    p_out->ec_rate_mv_s     = 0.0f;
    p_out->ntu_rate_mv_s    = 0.0f;

    pipeline_rate_reset(p_pipe);
    pipeline_note_set(p_out->note, (uint32_t) sizeof(p_out->note), p_pipe->baseline_reject);
}

/* ---------- B 方案：双基线槽 ---------- */
bool pipeline_baseline_store_slot(sensor_pipeline_t *p_pipe, uint8_t slot)
{
    pipeline_out_t *p_out;

    if ((NULL == p_pipe) || (!p_pipe->initialized)
        || (slot >= (uint8_t) PIPELINE_BASELINE_SLOTS))
    {
        return false;
    }
    p_out = &p_pipe->out;
    if (!p_out->baseline_valid)
    {
        return false;
    }

    p_pipe->slots[slot].valid        = true;
    p_pipe->slots[slot].ec0_mv25     = p_out->ec0_mv25;
    p_pipe->slots[slot].ntu0_ao_mv   = p_out->ntu0_ao_mv;
    p_pipe->slots[slot].ec_sigma_mv  = p_out->ec_sigma_mv;
    p_pipe->slots[slot].ntu_sigma_mv = p_out->ntu_sigma_mv;
    return true;
}

bool pipeline_baseline_select_slot(sensor_pipeline_t *p_pipe, uint8_t slot)
{
    pipeline_out_t *p_out;

    if ((NULL == p_pipe) || (!p_pipe->initialized)
        || (slot >= (uint8_t) PIPELINE_BASELINE_SLOTS)
        || (!p_pipe->slots[slot].valid))
    {
        return false;
    }
    p_out = &p_pipe->out;

    p_pipe->active_slot      = slot;
    p_out->ec0_mv25          = p_pipe->slots[slot].ec0_mv25;
    p_out->ntu0_ao_mv        = p_pipe->slots[slot].ntu0_ao_mv;
    p_out->ec_sigma_mv       = p_pipe->slots[slot].ec_sigma_mv;
    p_out->ntu_sigma_mv      = p_pipe->slots[slot].ntu_sigma_mv;
    p_out->baseline_valid    = true;
    p_out->baseline_updated  = false;

    /* 切基线后旧 EMA/速率没有意义：用当前采样值重新播种，并清空速率环。 */
    p_pipe->ec_ema  = (p_out->tds_mv25 > 1.0f) ? p_out->tds_mv25 : p_pipe->slots[slot].ec0_mv25;
    p_pipe->ntu_ema = (p_out->ntu_ao_mv > 0U)
                      ? (float) p_out->ntu_ao_mv
                      : (float) p_pipe->slots[slot].ntu0_ao_mv;
    pipeline_rate_reset(p_pipe);
    /*
     * M2j：切槽当 tick 立即重算 dEC/g/dNTU，避免 decision_update 在同一 tick
     * 用“旧基线的 dEC + 新基线的 EC0”混合判断（现场日志 07:40:08 的 5.9→18.9
     * 跳变就是这个时序造成的）。速率已由 rate_reset 清零。
     */
    pipeline_features_update(p_pipe);
    p_out->ec_rate_mv_s  = 0.0f;
    p_out->ntu_rate_mv_s = 0.0f;
    pipeline_note_set(p_out->note, (uint32_t) sizeof(p_out->note), "OK");
    return true;
}

bool pipeline_baseline_slot_valid(const sensor_pipeline_t *p_pipe, uint8_t slot)
{
    if ((NULL == p_pipe) || (slot >= (uint8_t) PIPELINE_BASELINE_SLOTS))
    {
        return false;
    }
    return p_pipe->slots[slot].valid;
}

bool pipeline_baseline_set_default(sensor_pipeline_t *p_pipe, uint8_t slot,
                                   float ec0_mv25, uint32_t ntu0_ao_mv)
{
    if ((NULL == p_pipe) || (!p_pipe->initialized)
        || (slot >= (uint8_t) PIPELINE_BASELINE_SLOTS)
        || (ec0_mv25 <= 1.0f) || (ntu0_ao_mv == 0U))
    {
        return false;
    }

    p_pipe->slots[slot].valid        = true;
    p_pipe->slots[slot].ec0_mv25     = ec0_mv25;
    p_pipe->slots[slot].ntu0_ao_mv   = ntu0_ao_mv;
    p_pipe->slots[slot].ec_sigma_mv  = p_pipe->cfg.sigma_floor_ec_mv;
    p_pipe->slots[slot].ntu_sigma_mv = p_pipe->cfg.sigma_floor_ntu_mv;
    return true;
}

void pipeline_baseline_trim_active(sensor_pipeline_t *p_pipe, float target_ec_mv,
                                   float max_step_mv)
{
    float delta;

    if ((NULL == p_pipe) || (!p_pipe->initialized) || (!p_pipe->out.baseline_valid)
        || (target_ec_mv <= 1.0f) || (max_step_mv <= 0.0f))
    {
        return;
    }
    if (p_pipe->active_slot >= (uint8_t)PIPELINE_BASELINE_SLOTS)
    {
        return;
    }

    /* 单 tick 限幅到 ±max_step：两参都是有限值（上面已挡掉 NaN/非正），
     * 与原来手写的 ">max / <-max" 两行 if 逐值等价。 */
    delta = sensor_clampf(target_ec_mv - p_pipe->out.ec0_mv25, -max_step_mv, max_step_mv);

    p_pipe->out.ec0_mv25 += delta;
    p_pipe->slots[p_pipe->active_slot].ec0_mv25 = p_pipe->out.ec0_mv25;
}

void pipeline_baseline_clear_slots(sensor_pipeline_t *p_pipe)
{
    if ((NULL == p_pipe) || (!p_pipe->initialized))
    {
        return;
    }
    (void) memset(p_pipe->slots, 0, sizeof(p_pipe->slots));
    p_pipe->active_slot = 0U;
}

/* 保存本 tick 使用的水温与来源，供下一次 pipeline_sample 做 TDS 温补。 注意：本函数只"保存"，不立刻重算特征；下一个 tick 生效（tick 50 ms，延迟可忽略）。 */
void pipeline_set_water_temp(sensor_pipeline_t *p_pipe, float temp_c, bool from_sensor)
{
    float t = temp_c;

    if (NULL == p_pipe)
    {
        return;
    }

    if (sensor_is_nan_f32(t))
    {
        /* NaN 防护：NaN 会传染整条温补-特征链（比较/运算全变 NaN），回退兜底温度 */
        t = PIPELINE_DEFAULT_WATER_TEMP_C;
    }
    t = sensor_clampf(t, PIPELINE_TEMP_MIN_C, PIPELINE_TEMP_MAX_C);

    p_pipe->water_temp_c = t;
    p_pipe->water_temp_from_sensor = from_sensor;

    /* 立即同步到输出，保证 app 在两次 tick 之间打印也拿到最新水温 */
    p_pipe->out.water_temp_c = t;
    p_pipe->out.water_temp_from_sensor = from_sensor;
}

/* 执行一个 tick 的完整流水线（采样 → 换算 → EMA → 基线累计/结算 → 相对特征 → 变化率 → note），结果写入句柄内的 out。
 * 时序/边界：由 app_main 主循环每 50 ms 调用一次；内部会阻塞等待 ADC 完成（毫秒级上限）， 不能从中断调用。 边界  ：p_pipe 为 NULL 或未 pipeline_init 时返回 false 且不写 out（调用前必须初始化）； 基线采集期间每个样本只收"两路都 SENSOR_OK"的 tick，故障样本不推进进度。
 * 注意：超时 tick 不更新 EMA/基线/变化率，只清原始量并打故障标记；判别层看到 adc_timeout 就应走安全侧（L1 排空），不得使用保留的旧特征做判断。 */
bool pipeline_sample(sensor_pipeline_t *p_pipe, uint32_t now_ms)
{
    pipeline_out_t   *p_out;
    tds_reading_t     tds;
    ts300b_reading_t  ntu;
    uint16_t          raw_tds = 0U;
    uint16_t          raw_ntu = 0U;
    bool              ec_ok;
    bool              ntu_ok;

    if ((NULL == p_pipe) || (!p_pipe->initialized))
    {
        return false;
    }
    p_out = &p_pipe->out;

    /* 0) 每 tick 的"一次性"字段先复位：baseline_updated 只允许在结算那一 tick 为 true */
    p_out->adc_timeout = false;
    p_out->baseline_updated = false;
    p_out->water_temp_c = p_pipe->water_temp_c;
    p_out->water_temp_from_sensor = p_pipe->water_temp_from_sensor;

    /* 1) 双通道 ADC 采样：adc_median_n 轮扫描，两通道各取中值 */
    if (!pipeline_adc_read_pair(p_pipe, &raw_tds, &raw_ntu))
    {
        /* ADC 启动失败/等待超时/读取失败：本 tick 没有可信样本，清原始量并打故障标记 */
        pipeline_mark_data_invalid(p_pipe);
        return false;
    }
    p_out->tds_raw = raw_tds;
    p_out->ntu_raw = raw_ntu;

    /* 2) 驱动换算：TDS → mV/mV25/参考 ppm；浊度 → 分压节点 mV/AO mV。
     *    局部结构体先清零，防止驱动在异常路径不写字段时把栈上的随机值当结果。 */
    (void) memset(&tds, 0, sizeof(tds));
    (void) memset(&ntu, 0, sizeof(ntu));
    p_out->tds_status = tds_convert(raw_tds, p_pipe->water_temp_c, &tds);
    p_out->ntu_status = ts300b_convert(raw_ntu, &ntu);

    /* 驱动约定：SENSOR_OK=正常；SENSOR_ERR_RANGE=贴轨/越界（字段仍然有效，靠 rail/sat/
     * NTU_RAIL 标志提示）。其它返回（参数/未初始化/数据错）属于异常路径：对上层来说
     * 与"本 tick 数据不可信"等价，按 ADC 故障同一口径处理（安全侧）。 */
    ec_ok = (SENSOR_OK == p_out->tds_status) || (SENSOR_ERR_RANGE == p_out->tds_status);
    ntu_ok = (SENSOR_OK == p_out->ntu_status) || (SENSOR_ERR_RANGE == p_out->ntu_status);
    if ((!ec_ok) || (!ntu_ok))
    {
        pipeline_mark_data_invalid(p_pipe);
        return false;
    }

    p_out->tds_mv = tds.mv;
    p_out->tds_mv25 = tds.mv25;
    p_out->tds_ppm25 = tds.ppm25;
    p_out->tds_rail = tds.rail;
    p_out->tds_saturated = tds.saturated;
    p_out->ntu_node_mv = ntu.node_mv;
    p_out->ntu_ao_mv = ntu.ao_mv;

    /* 3) EMA 平滑（mv25 与 AO 各一路）。
     *    初值处理：句柄清零后 ec_ema/ntu_ema 为 0，视作"未初始化"，首个有效样本直接
     *    采纳；否则会从 0 慢慢收敛，上电前 ~10 个 tick 的 Δ 会被系统性拉偏。 */
    if (p_pipe->ec_ema <= 0.0f)
    {
        p_pipe->ec_ema = p_out->tds_mv25;
    }
    else
    {
        p_pipe->ec_ema = sensor_ema(p_pipe->ec_ema, p_out->tds_mv25, p_pipe->cfg.ema_alpha);
    }

    if (p_pipe->ntu_ema <= 0.0f)
    {
        p_pipe->ntu_ema = (float) p_out->ntu_ao_mv;
    }
    else
    {
        p_pipe->ntu_ema = sensor_ema(p_pipe->ntu_ema, (float) p_out->ntu_ao_mv, p_pipe->cfg.ema_alpha);
    }

    /* 4) 基线采集：一个 tick 累计一个样本；只收两路都完全正常的样本，
     *    避免把贴轨/故障读数写进本底。若传感器持续故障，采集会一直不完成，
     *    此时 baseline_valid=false，判别层按安全侧排空（fail-safe），这是设计内行为。 */
    if (p_pipe->baseline_capturing)
    {
        if ((SENSOR_OK == p_out->tds_status) && (SENSOR_OK == p_out->ntu_status))
        {
            float ec_sample = p_out->tds_mv25;
            float ntu_sample = (float) p_out->ntu_ao_mv;

            p_pipe->base_ec_sum += ec_sample;
            p_pipe->base_ec_sumsq += ec_sample * ec_sample;
            p_pipe->base_ntu_sum += ntu_sample;
            p_pipe->base_ntu_sumsq += ntu_sample * ntu_sample;
            p_pipe->baseline_count++;
            p_out->baseline_n = p_pipe->baseline_count;

            if (p_pipe->baseline_count >= p_pipe->cfg.baseline_n)
            {
                pipeline_baseline_finish(p_pipe);
            }
        }
    }

    /* 5) 相对特征（用本 tick 的 EMA 与已生效的基线）；有基线时才推进变化率窗口。
     *    顺序不能反：pipeline_rate_update 使用本 tick 刚算出的 d_ec_mv / d_ntu_mv。 */
    pipeline_features_update(p_pipe);
    if (p_out->baseline_valid)
    {
        pipeline_rate_update(p_pipe, now_ms);
    }

    /* 6) note 按固定优先级刷新（见 pipeline_note_update 的说明） */
    pipeline_note_update(p_pipe);

    return true;
}

/* 返回最近一次输出的只读指针（app 打印 CSV / decision 读特征都走这里）。
 * 返回：&p_pipe->out；p_pipe 为 NULL 时返回 NULL。 边界  ：只读指针，调用者不得修改；指针在句柄生命周期内有效（内部不动态分配）。
 * 注意：必须在 pipeline_init 之后调用（初始化前 out 内容未定义）； 数据是"最近一个 tick"的快照，跨 tick 使用请自行拷贝需要保留的字段。 */
const pipeline_out_t *pipeline_get_out(const sensor_pipeline_t *p_pipe)
{
    if (NULL == p_pipe)
    {
        return NULL;
    }
    return &p_pipe->out;
}
