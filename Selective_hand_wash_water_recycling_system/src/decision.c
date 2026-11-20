/*
 * decision.c —— 水质判别 + 回收许可（v2：事件闩锁 → 工况门控 + 每 tick 重算的谓词）
 * ---------------------------------------------------------------------------
 * 数据来源：只读 sensor_pipeline 产生的 pipeline_out_t（每个 50 ms tick 一份），
 *           并通过 gate_t 获取"现在能不能判"（validity 层，见 validity.c）。
 *           本文件不碰 ADC / 1-Wire / UART / 阀，也不打印——纯逻辑，便于脱离硬件测试。
 *
 * 对外输出：decision_out_t（state / level / action / permit / regime / valid_bits /
 *           自适应阈值 / reason），app_main 用它控阀、打 LED、写日志。
 *
 * v2 的主线（为什么删掉"事件"）：
 *   旧代码里"事件"是一个有记忆的状态：单点 L2 即可 auto_start，只有 30 s 超时能关，
 *   而超时释放的动作本身又会重新触发 —— 现场看到的"30 s 一周期自激"就是它。
 *   只要"开启"和"关闭"不是同一个谓词的两面，就一定会出现"一次扰动买到 30 秒许可"。
 *   新代码没有"开启"这个动作：每 50 ms 重算
 *       PERMIT = (level == L0) && IMMERSED
 *   谓词为假的那一 tick 就关阀；"超时"这个问题在谓词模型里不存在。
 *
 * 判别链（只在 REGIME_MEASURE 里执行）：
 *   ① 双侧门限分类（正向=脏；负向=可疑；两路都在下界内=可清）；
 *   ② 变化率抢先（只认正向、≥120 mV/s、连续 2 tick、换向后 400 ms 内禁用）；
 *   ③ 块内一致（400 ms / 8 点 / 6 票）；
 *   ④ 单侧迟滞（变脏/拿不准立即生效；升级到 L0 需连续 3 个块）。
 *
 * 安全侧原则（本文件最重要的设计约定）：
 *   任何"不知道"的情况一律 L1 → 排空：
 *     ADC 超时 / 传感器故障 / 基线无效 / 工况非 MEASURE / 块内投票不通过 → 全部 L1。
 *   唯一会让阀门回收的是：工况 MEASURE、块内 ≥6 票 L0、且许可谓词成立。
 * ---------------------------------------------------------------------------
 * 参考：Docs/软件层优化方案_v2_20261116.md §6/§7；Docs/现场诊断报告_无水日志_20261116.md。
 */

#include "decision.h"

#include <stddef.h>   /* NULL */

/* ---------- 静态原因字符串 ---------- */
/*
 * out.reason 只保存指针，调用方（app_main）会把它打印到串口；
 * 用字符串字面量/静态常量 → 生命周期=整个程序，绝不返回栈缓冲或动态内存。
 */
static const char *const g_reason_idle         = "IDLE";
static const char *const g_reason_clean        = "CLEAN";
static const char *const g_reason_uncertain    = "UNCERTAIN";
static const char *const g_reason_soap_high    = "SOAP_HIGH";
static const char *const g_reason_rate_jump    = "RATE_JUMP";
static const char *const g_reason_rate_pending = "RATE_PEND";
static const char *const g_reason_too_clean    = "TOO_CLEAN";
static const char *const g_reason_block_split  = "BLOCK_SPLIT";
static const char *const g_reason_blank        = "BLANK";
static const char *const g_reason_soap_latch   = "SOAP_LATCH";

/* ---------- 枚举 → 字符串/状态 静态表 ---------- */
/*
 * 下标就是枚举值（decision.h 里三个枚举都从 0 起连续），越界一律回退到旧 switch 的
 * default 取值，逐字不变：
 *   等级→状态 UNCERTAIN、等级→原因 UNCERTAIN、状态名 "UNCERT"、等级名 "L1"、动作名 "DRAIN"。
 * 表集中放在文件顶部：改字符串只改一处，不必在多个 switch 分支里找。
 */
#define DEC_TABLE_LEN(a)  (sizeof(a) / sizeof((a)[0]))

static const decision_state_t g_state_by_level[] =
{
    DEC_STATE_CLEAR, DEC_STATE_UNCERTAIN, DEC_STATE_SOAP    /* L0 / L1 / L2 */
};
static const char *const g_reason_by_level[] =
{
    "CLEAN", "UNCERTAIN", "SOAP_HIGH"                        /* L0 / L1 / L2 */
};
static const char *const g_state_name[] =
{
    "WAIT", "FLUSH", "CLEAR", "UNCERT", "SOAP"
};
static const char *const g_level_name[] =
{
    "L0", "L1", "L2"
};
static const char *const g_action_name[] =
{
    "DRAIN", "RECOV"
};

/* ---------- 默认配置 ---------- */
/*
 * p_cfg == NULL 时使用；数值与 app_main.c 的默认配置一致。
 * gate 子结构留空：gate_init 会把 0 值补成 validity.c 里的默认值。
 */
static const decision_cfg_t g_default_cfg =
{
    .ec_d1_sigma         = 4.0f,     /* 进入可清的 σ 倍数（默认 4σ） */
    .ec_d2_sigma         = 8.0f,     /* 判定含皂液的 σ 倍数（默认 8σ） */
    .ec_d1_min_mv        = 8.0f,     /* d1 绝对下限 mV，防 σ 过小阈值退化 */
    .ec_d2_min_mv        = 20.0f,    /* d2 绝对下限 mV */
    .ec_d1_pct           = 6.0f,     /* d1 相对 EC0 的百分比下限（现场清水漂移约 5%） */
    .ec_d2_pct           = 9.0f,     /* d2 相对 EC0 的百分比下限（肥皂现场约 +11.5%） */
    .ec_hyst_ratio       = 0.25f,    /* EC 迟滞 25% */
    .ntu_d1_pct          = 8.0f,     /* 浊度"可清"上限（相对基线 %） */
    .ntu_d2_pct          = 25.0f,    /* 浊度"明显变脏"阈值（相对基线 %） */
    .ntu_hyst_ratio      = 0.25f,    /* 浊度迟滞 25% */
    .ec_rate_th_mv_s     = 120.0f,   /* 正向变化率阈值（≥3× 实测瞬态 38 mV/s） */
    .ec_rate_min_jump_mv = 5.0f,     /* 触发抢先所需最小绝对跳变 mV */
    .ntu_rate_th_pct_s   = 15.0f,    /* M2h：肥皂浊度前沿≈16 %/s，150 永不触发 */
    .soap_latch_ms       = 45000U,   /* L2 后强制 DRAIN 45 s，覆盖稀释尾水 */
    .rate_confirm_ticks  = 2U,       /* 连续 2 tick 才认 */
    .block_ms            = 400U,     /* 消抖块 400 ms */
    .block_samples       = 8U,       /* 400 ms / 50 ms */
    .block_agree         = 6U,       /* 8 中 6 */
    .upgrade_blocks      = 3U,       /* 升级到 L0 需连续 3 块（1.2 s） */
    .downgrade_blocks    = 2U        /* L0→L1 需连续 2 块（0.8 s）；L0→L2 仍立即 */
};

/* ---------- 文件内函数声明 ---------- */
static decision_state_t  decision_state_from_level(decision_level_t level);
static const char       *decision_reason_from_level(decision_level_t level);
static void              decision_reset_block(decision_t *p_dec, uint32_t now_ms);
static void              decision_clear_counters(decision_t *p_dec, uint32_t now_ms);
static float             decision_thr(float sigma_mv, float min_mv, float pct, float ec0);
static void              decision_apply_output(decision_t *p_dec, decision_state_t state,
                                               decision_level_t level, decision_action_t action,
                                               bool permit, const char *reason, uint32_t now_ms);
static void              decision_update_thresholds(decision_t *p_dec, const pipeline_out_t *p_out);
static decision_level_t  decision_classify(const pipeline_out_t *p_out,
                                           float e1_ec, float e2_ec,
                                           float e1_ntu, float e2_ntu,
                                           const char **p_reason);
static decision_level_t  decision_block_update(decision_t *p_dec, decision_level_t cand,
                                               uint32_t now_ms, bool *p_evaluated, bool *p_split);

/* ---------- 小工具 ---------- */

/* 等级 → 状态（L0→CLEAR、L2→SOAP、L1/越界→UNCERTAIN）。 */
static decision_state_t decision_state_from_level(decision_level_t level)
{
    return ((unsigned)level < DEC_TABLE_LEN(g_state_by_level))
           ? g_state_by_level[level] : DEC_STATE_UNCERTAIN;
}

/* 等级 → 原因串（块投票通过时用，避免拿单点样本的原因误导日志）。 */
static const char *decision_reason_from_level(decision_level_t level)
{
    return ((unsigned)level < DEC_TABLE_LEN(g_reason_by_level))
           ? g_reason_by_level[level] : g_reason_uncertain;
}

/* 清空消抖块（投票环形缓冲 + 长度 + 块起点）。 */
static void decision_reset_block(decision_t *p_dec, uint32_t now_ms)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)DECISION_MAX_BLOCK_SAMPLES; i++)
    {
        p_dec->verdict_ring[i] = 0U;
    }
    p_dec->verdict_pos    = 0U;
    p_dec->verdict_len    = 0U;
    p_dec->block_start_ms = now_ms;
}

/*
 * 清三个独立计数器 + 消抖块。
 * 三个计数器各自对应一条去抖/迟滞路径（变化率抢先、升级到 L0、L0→L1），
 * 任何"情境变了"的时刻都必须一起清，抽成一处避免漏清某一项。
 */
static void decision_clear_counters(decision_t *p_dec, uint32_t now_ms)
{
    p_dec->rate_hits      = 0U;
    p_dec->upgrade_blocks = 0U;
    p_dec->downgrade_hits = 0U;
    decision_reset_block(p_dec, now_ms);
}

/*
 * 把新结果写入 out，并维护 changed / state_since_ms。
 * changed 只在 state / level / action / permit 任一变化时置 true。
 * state_since_ms 只在"状态"变化时刷新（日志与显示用）。
 */
static void decision_apply_output(decision_t *p_dec, decision_state_t state, decision_level_t level,
                                  decision_action_t action, bool permit, const char *reason,
                                  uint32_t now_ms)
{
    bool changed;

    changed = (p_dec->out.state != state) || (p_dec->out.level != level)
           || (p_dec->out.action != action) || (p_dec->out.permit != permit);

    if (p_dec->out.state != state)
    {
        p_dec->state_since_ms = now_ms;
    }

    p_dec->out.state        = state;
    p_dec->out.level        = level;
    p_dec->out.action       = action;
    p_dec->out.permit       = permit;
    p_dec->out.event_active = p_dec->event_active;
    p_dec->out.changed      = changed;
    p_dec->out.reason       = (NULL != reason) ? reason : g_reason_idle;

    /* M2h：第一次采纳 L2 时启动锁存；锁存期间强制 SOAP/DRAIN。 */
    if ((DEC_STATE_SOAP == state) && (!p_dec->soap_latch_active))
    {
        p_dec->soap_latch_active   = true;
        p_dec->soap_latch_until_ms = now_ms + p_dec->cfg.soap_latch_ms;
    }
}

/* 单侧 EC 阈值：max(σ 倍数项, 绝对下限, EC0 百分比下限)。
 * sigma_mv 是"倍数×σ"已乘好的一项；ec0 传 0 表示本 tick 不做百分比项（基线无效 /
 * EC0 过小），此时百分比项为 0、不改变 max 结果 —— 与旧的两段 if 逐项等价。 */
static float decision_thr(float sigma_mv, float min_mv, float pct, float ec0)
{
    float th     = sigma_mv;
    float pct_mv = ec0 * pct / 100.0f;

    if (th < min_mv)
    {
        th = min_mv;
    }
    if (pct_mv > th)
    {
        th = pct_mv;
    }
    return th;
}

/*
 * 按本底噪声 σ_ec 计算本 tick 生效的 d1/d2（EC 自适应阈值）。
 * d1 = max(ec_d1_sigma*σ, ec_d1_min_mv, ec_d1_pct%×EC0)，d2 同理且强制 d2 > d1。
 * 浊度阈值只用固定百分比（无 NTU 标定，红线：不写 NTU）。
 */
static void decision_update_thresholds(decision_t *p_dec, const pipeline_out_t *p_out)
{
    float sigma = p_out->ec_sigma_mv;
    float ec0   = 0.0f;
    float d1_ntu;
    float d2_ntu;

    if (sigma < 0.0f)
    {
        sigma = 0.0f;   /* 防御：σ 为负只可能是上游异常，按 0 处理 */
    }

    /*
     * EC0 百分比下限（v2.2，现场 2026-11-16 数据）：
     *   清水 EC0≈348 mV，阀切换引起的水力漂移 ΔEC 达 ±16 mV（≈5%×EC0），
     *   而固定下限 8 mV 只有 2.3%×EC0 —— 阈值落在漂移带内，清水就会 2 s 周期抖。
     *   改为 d1=max(4σ, 8mV, 6%×EC0)、d2=max(8σ, 20mV, 9%×EC0)：
     *   348 mV → d1≈20.9、d2≈31.3；清水峰值 16 不进 L1，肥皂 +40 仍 >d2。
     * 基线无效时不做百分比（ec0 保持 0），保持原来的绝对/σ 阈值。
     */
    if ((p_out->baseline_valid) && (p_out->ec0_mv25 > 1.0f))
    {
        ec0 = p_out->ec0_mv25;
    }

    p_dec->out.d1_ec_mv = decision_thr(p_dec->cfg.ec_d1_sigma * sigma, p_dec->cfg.ec_d1_min_mv,
                                       p_dec->cfg.ec_d1_pct, ec0);
    p_dec->out.d2_ec_mv = decision_thr(p_dec->cfg.ec_d2_sigma * sigma, p_dec->cfg.ec_d2_min_mv,
                                       p_dec->cfg.ec_d2_pct, ec0);

    if (p_dec->out.d2_ec_mv <= p_dec->out.d1_ec_mv)
    {
        float gap = p_dec->out.d1_ec_mv * p_dec->cfg.ec_hyst_ratio;

        if (gap < 1.0f)
        {
            gap = 1.0f;
        }
        p_dec->out.d2_ec_mv = p_dec->out.d1_ec_mv + gap;
    }

    d1_ntu = p_dec->cfg.ntu_d1_pct;
    d2_ntu = p_dec->cfg.ntu_d2_pct;
    if (d2_ntu <= d1_ntu)
    {
        d2_ntu = d1_ntu + 1.0f;
    }
    p_dec->out.d1_ntu_pct = d1_ntu;
    p_dec->out.d2_ntu_pct = d2_ntu;
}

/*
 * 三级分类（双侧门限）：
 *   ① 浊度单独 > 上边界，或 EC 高且浊度接近上边界 → L2（明显变脏）
 *   ② 任一路 < -上边界           → L1（"突然变太干净"同样可疑：电极脱水/气泡/水位下降）
 *   ③ 两路都 < 下边界            → L0
 *   ④ 其余                       → L1（拿不准=排空）
 * 说明：负偏离必须拦。旧代码只拦"变脏"，于是"探头插着但没水"（EC 掉到 43~58 mV）
 *       成了最安全的样子 → 直接 L0 回收。这正是现场 S1/S5a 的漏洞。
 */
static decision_level_t decision_classify(const pipeline_out_t *p_out,
                                          float e1_ec, float e2_ec,
                                          float e1_ntu, float e2_ntu,
                                          const char **p_reason)
{
    /*
     * B3 双通道：浊度单独脏 → L2；EC 单独高 → 只算 L1（液位/流动，不是肥皂）。
     * EC 要参与 L2，必须同时有接近上阈的浊度（≥0.9×e2_ntu）；
     * 现场清水满水 EC+52mV 但 dNTU 仅 +1~+7%，不能判 SOAP。
     */
    if ((p_out->d_ntu_pct > e2_ntu)
        || ((p_out->d_ec_mv > e2_ec)
            && (p_out->d_ntu_pct > (0.9f * e2_ntu))))
    {
        *p_reason = g_reason_soap_high;
        return DEC_LEVEL_L2;
    }

    if ((p_out->d_ec_mv < (-e2_ec)) || (p_out->d_ntu_pct < (-e2_ntu)))
    {
        *p_reason = g_reason_too_clean;
        return DEC_LEVEL_L1;
    }

    if ((p_out->d_ec_mv < e1_ec) && (p_out->d_ntu_pct < e1_ntu))
    {
        *p_reason = g_reason_clean;
        return DEC_LEVEL_L0;
    }

    *p_reason = g_reason_uncertain;
    return DEC_LEVEL_L1;
}

/*
 * 块内一致消抖 + 单侧迟滞。
 *   - 块满（block_samples）或到时（block_ms）后投票，某等级 ≥ block_agree 才采纳；
 *   - 票数不够（例如 4:4）→ L1（安全侧），并置 p_split 供上层标注原因；
 *   - 单侧迟滞：任何"非 L0"结果立即采纳（安全侧要快）；
 *              升级到 L0 必须连续 upgrade_blocks 个块都投出 L0（默认 3 块 ≈ 1.2 s）。
 * 返回：本 tick 采纳的等级（块未结束时返回上一次采纳值，保持输出稳定）。
 */
static decision_level_t decision_block_update(decision_t *p_dec, decision_level_t cand,
                                              uint32_t now_ms, bool *p_evaluated, bool *p_split)
{
    uint8_t i;
    uint8_t count_l0;
    uint8_t count_l1;
    uint8_t count_l2;
    uint8_t best_count;
    uint8_t n_votes;
    uint8_t need;
    decision_level_t best_level;
    decision_level_t result;

    *p_evaluated = false;
    *p_split     = false;

    if (0U == p_dec->verdict_len)
    {
        p_dec->block_start_ms = now_ms;   /* 新块：从第一个样本开始计时 */
    }

    if (p_dec->verdict_pos >= (uint8_t)DECISION_MAX_BLOCK_SAMPLES)
    {
        p_dec->verdict_pos = 0U;   /* 防御：正常流程到不了（上限 8 且满即投票） */
    }

    p_dec->verdict_ring[p_dec->verdict_pos] = (uint8_t)cand;
    p_dec->verdict_pos++;
    p_dec->verdict_len++;

    if ((p_dec->verdict_len < p_dec->cfg.block_samples) &&
        ((uint32_t)(now_ms - p_dec->block_start_ms) < p_dec->cfg.block_ms))
    {
        return p_dec->out.level;   /* 块未结束：保持上次采纳等级 */
    }

    /* 投票 */
    count_l0 = 0U;
    count_l1 = 0U;
    count_l2 = 0U;
    for (i = 0U; i < p_dec->verdict_len; i++)
    {
        switch (p_dec->verdict_ring[i])
        {
            case DEC_LEVEL_L0:
                count_l0++;
                break;

            case DEC_LEVEL_L2:
                count_l2++;
                break;

            case DEC_LEVEL_L1:
            default:
                count_l1++;   /* 非法值也按"拿不准"计票（安全侧） */
                break;
        }
    }

    best_level = DEC_LEVEL_L1;
    best_count = count_l1;
    if (count_l0 > best_count)
    {
        best_level = DEC_LEVEL_L0;
        best_count = count_l0;
    }
    if (count_l2 > best_count)
    {
        best_level = DEC_LEVEL_L2;
        best_count = count_l2;
    }

    /* 块结束：无论通过与否都要清块，下一块重新投票 */
    n_votes               = p_dec->verdict_len;
    p_dec->verdict_len    = 0U;
    p_dec->verdict_pos    = 0U;
    p_dec->block_start_ms = now_ms;
    *p_evaluated          = true;

    /*
     * 票数门槛：满块按配置的 6/8；短块（主循环被 printf 拖到 100 ms 时，400 ms 内只有
     * 4~5 个采样）按"实际样本数的 75 % 向上取整"折算。
     * 否则短块永远达不到 6 票 → 每块都 BLOCK_SPLIT → upgrade_blocks 永远到不了 3 →
     * 静默地永不回收（现场表现为"水明明是干净的，就是不回收"，且日志只有 UNCERTAIN）。
     */
    need = p_dec->cfg.block_agree;
    if (n_votes < p_dec->cfg.block_samples)
    {
        need = (uint8_t)((((uint16_t)n_votes * 3U) + 3U) / 4U);
        if (need < 1U)
        {
            need = 1U;
        }
        if (need > p_dec->cfg.block_agree)
        {
            need = p_dec->cfg.block_agree;
        }
    }

    /*
     * 证据量下限：短块折算到 n=1 时门槛会退化成"1 票决定整块"，
     * 3 个这样的块（可能只来自 3 个采样）就能把等级升到 L0 —— 这不是消抖，是抽签。
     * 少于 4 个采样、或票数不够，都作废本块（候选降为 L1），但**仍要走进下面的
     * L0→L1 去抖**：否则一次噪声块就能把 CLEAR 打掉。
     */
    if ((n_votes < 4U) || (best_count < need))
    {
        *p_split  = true;
        best_level = DEC_LEVEL_L1;
    }

    result = best_level;

    /* L2（明显变脏）永远立即生效，不受去抖影响 */
    if (DEC_LEVEL_L2 == result)
    {
        p_dec->upgrade_blocks = 0U;
        p_dec->downgrade_hits = 0U;
        return DEC_LEVEL_L2;
    }

    /*
     * L1（拿不准）：
     *   - 从 L0 掉下来需要连续 downgrade_blocks 个块（默认 2×400 ms）；
     *     现场清水的水力漂移只持续约 0.4~0.6 s，一块就过去，2 块足以挡住；
     *   - 从 L1/L2 到 L1、或本来就是 L1，立即返回。
     * L2 不走这里（上面已立即返回），所以肥皂/脏水不会被拖延。
     */
    if (DEC_LEVEL_L1 == result)
    {
        if (DEC_LEVEL_L0 == p_dec->out.level)
        {
            if (p_dec->downgrade_hits < 0xFFU)
            {
                p_dec->downgrade_hits++;
            }
            if (p_dec->downgrade_hits < p_dec->cfg.downgrade_blocks)
            {
                p_dec->upgrade_blocks = 0U;
                return DEC_LEVEL_L0;   /* 保持回收，等下一块再确认 */
            }
        }
        p_dec->downgrade_hits = 0U;
        p_dec->upgrade_blocks = 0U;
        return DEC_LEVEL_L1;
    }

    /* result == L0：任何"可清"块都会清掉 L1 去抖计数 */
    p_dec->downgrade_hits = 0U;

    if (DEC_LEVEL_L0 == p_dec->out.level)
    {
        p_dec->upgrade_blocks = 0U;
        return DEC_LEVEL_L0;
    }

    if (p_dec->upgrade_blocks < 0xFFU)
    {
        p_dec->upgrade_blocks++;
    }
    if (p_dec->upgrade_blocks >= p_dec->cfg.upgrade_blocks)
    {
        p_dec->upgrade_blocks = 0U;
        return DEC_LEVEL_L0;
    }

    return p_dec->out.level;   /* 还没攒够块数：维持原等级（排空侧） */
}

/* ---------- 对外接口 ---------- */

/*
 * 初始化：保存/限幅配置、初始化工况机、清零运行时状态。
 * 输出初值：WAIT / L1 / DRAIN / permit=false（安全侧）。
 */
void decision_init(decision_t *p_dec, const decision_cfg_t *p_cfg)
{
    decision_cfg_t *p_c;

    if (NULL == p_dec)
    {
        return;
    }

    p_dec->cfg = (NULL != p_cfg) ? *p_cfg : g_default_cfg;
    p_c = &p_dec->cfg;

    /* --- EC 阈值：限幅到物理上说得通的区间 --- */
    p_c->ec_d1_sigma   = sensor_clampf(p_c->ec_d1_sigma, 0.5f, 20.0f);
    p_c->ec_d2_sigma   = sensor_clampf(p_c->ec_d2_sigma, 1.0f, 40.0f);
    if (p_c->ec_d2_sigma <= p_c->ec_d1_sigma)
    {
        p_c->ec_d2_sigma = p_c->ec_d1_sigma * 2.0f;
    }

    p_c->ec_d1_min_mv  = sensor_clampf(p_c->ec_d1_min_mv, 0.5f, 1000.0f);
    p_c->ec_d2_min_mv  = sensor_clampf(p_c->ec_d2_min_mv, 1.0f, 2000.0f);
    if (p_c->ec_d2_min_mv <= p_c->ec_d1_min_mv)
    {
        p_c->ec_d2_min_mv = p_c->ec_d1_min_mv * 2.0f;
    }
    p_c->ec_hyst_ratio = sensor_clampf(p_c->ec_hyst_ratio, 0.0f, 0.8f);

    /* --- EC0 百分比下限：0/0 表示关闭该机制；否则保证 d2_pct > d1_pct --- */
    p_c->ec_d1_pct = sensor_clampf(p_c->ec_d1_pct, 0.0f, 30.0f);
    p_c->ec_d2_pct = sensor_clampf(p_c->ec_d2_pct, 0.0f, 50.0f);
    if ((p_c->ec_d1_pct <= 0.0f) && (p_c->ec_d2_pct <= 0.0f))
    {
        p_c->ec_d1_pct = 0.0f;
        p_c->ec_d2_pct = 0.0f;
    }
    else if (p_c->ec_d2_pct <= p_c->ec_d1_pct)
    {
        p_c->ec_d2_pct = p_c->ec_d1_pct + 1.0f;
    }

    /* --- 浊度阈值（相对基线的百分比）：同样保证 d2 > d1 --- */
    p_c->ntu_d1_pct    = sensor_clampf(p_c->ntu_d1_pct, 0.5f, 200.0f);
    p_c->ntu_d2_pct    = sensor_clampf(p_c->ntu_d2_pct, 1.0f, 500.0f);
    if (p_c->ntu_d2_pct <= p_c->ntu_d1_pct)
    {
        p_c->ntu_d2_pct = p_c->ntu_d1_pct * 2.0f;
    }
    p_c->ntu_hyst_ratio = sensor_clampf(p_c->ntu_hyst_ratio, 0.0f, 0.8f);

    /* --- 变化率抢先：阈值必须为正，否则等于每个 tick 都触发 --- */
    p_c->ec_rate_th_mv_s     = sensor_clampf(p_c->ec_rate_th_mv_s, 1.0f, 100000.0f);
    p_c->ec_rate_min_jump_mv = sensor_clampf(p_c->ec_rate_min_jump_mv, 0.5f, 1000.0f);
    p_c->ntu_rate_th_pct_s   = sensor_clampf(p_c->ntu_rate_th_pct_s, 1.0f, 100000.0f);
    if ((p_c->soap_latch_ms < 5000U) || (p_c->soap_latch_ms > 300000U))
    {
        p_c->soap_latch_ms = 45000U;
    }
    if (p_c->rate_confirm_ticks < 1U)
    {
        p_c->rate_confirm_ticks = 1U;
    }
    if (p_c->rate_confirm_ticks > 10U)
    {
        p_c->rate_confirm_ticks = 10U;
    }

    /* --- 块参数：块长至少覆盖 block_samples 个 50 ms tick --- */
    if (p_c->block_ms < 50U)
    {
        p_c->block_ms = 50U;
    }
    if (p_c->block_ms > 5000U)
    {
        p_c->block_ms = 5000U;
    }
    if (p_c->block_samples < 1U)
    {
        p_c->block_samples = 1U;
    }
    if (p_c->block_samples > (uint8_t)DECISION_MAX_BLOCK_SAMPLES)
    {
        p_c->block_samples = (uint8_t)DECISION_MAX_BLOCK_SAMPLES;
    }
    if (p_c->block_agree < 1U)
    {
        p_c->block_agree = 1U;
    }
    if (p_c->block_agree > p_c->block_samples)
    {
        p_c->block_agree = p_c->block_samples;
    }
    if ((uint32_t)p_c->block_samples * 50U > p_c->block_ms)
    {
        p_c->block_ms = (uint32_t)p_c->block_samples * 50U;
    }

    if (p_c->upgrade_blocks < 1U)
    {
        p_c->upgrade_blocks = 1U;
    }
    if (p_c->upgrade_blocks > 10U)
    {
        p_c->upgrade_blocks = 10U;
    }
    if (p_c->downgrade_blocks < 1U)
    {
        p_c->downgrade_blocks = 1U;
    }
    if (p_c->downgrade_blocks > 5U)
    {
        p_c->downgrade_blocks = 5U;
    }

    /* --- 工况机（L0 位图 + L1 regime） --- */
    gate_init(&p_dec->gate, &p_c->gate);

    /* --- 运行时状态清零（三个计数器 + 消抖块由同一处清，起点=0 ms） --- */
    p_dec->state_since_ms = 0U;
    p_dec->event_since_ms = 0U;
    p_dec->event_active   = false;
    p_dec->blank_active   = false;
    p_dec->soap_latch_active   = false;
    p_dec->soap_latch_until_ms = 0U;
    decision_clear_counters(p_dec, 0U);

    /* --- 输出初值：安全侧 WAIT/L1/DRAIN；changed=true 让上电第一帧日志可打印 --- */
    p_dec->out.state        = DEC_STATE_WAIT;
    p_dec->out.level        = DEC_LEVEL_L1;
    p_dec->out.action       = DEC_ACTION_DRAIN;
    p_dec->out.event_active = false;
    p_dec->out.changed      = true;
    p_dec->out.permit       = false;
    p_dec->out.valid_bits   = 0U;
    p_dec->out.regime       = REGIME_FAULT;
    p_dec->out.immersed     = false;
    p_dec->out.blanking     = false;
    p_dec->out.d1_ec_mv     = p_c->ec_d1_min_mv;
    p_dec->out.d2_ec_mv     = p_c->ec_d2_min_mv;
    p_dec->out.d1_ntu_pct   = p_c->ntu_d1_pct;
    p_dec->out.d2_ntu_pct   = p_c->ntu_d2_pct;
    p_dec->out.reason       = g_reason_idle;
    p_dec->out.gate_reason  = gate_reason_str(&p_dec->gate);
}

/*
 * 手动事件开始（串口 R）。
 * v2 起"事件"只是日志分段标记：不改变工况、不改变阀、不产生任何许可。
 * 保留该接口是为了让现场仍在用的 R/E 操作有明确语义（否则串口脚本会报未知命令）。
 */
void decision_start_event(decision_t *p_dec, uint32_t now_ms)
{
    if (NULL == p_dec)
    {
        return;
    }

    p_dec->event_active   = true;
    p_dec->event_since_ms = now_ms;
    p_dec->out.event_active = true;
}

/* 手动事件结束（串口 E）：只清标记。 */
void decision_end_event(decision_t *p_dec, uint32_t now_ms)
{
    if (NULL == p_dec)
    {
        return;
    }

    p_dec->event_active   = false;
    p_dec->event_since_ms = now_ms;
    p_dec->out.event_active = false;
}

/*
 * 阀换向通知：屏蔽窗起点。app 在阀真正换向后调用。
 * 若在 decision_update 之后调用，则屏蔽从下一个 tick 开始计（50 ms 滞后 < 400 ms 窗宽，可接受）。
 */
void decision_note_valve_switch(decision_t *p_dec, uint32_t now_ms)
{
    if (NULL == p_dec)
    {
        return;
    }
    gate_note_switch(&p_dec->gate, now_ms);
}

/* M2：液位变化中强制 WAIT/DRAIN（由 app 层的 level_hold 直接走 app 层门控，
 *     decision 侧不再保留独立入口）。 */

/*
 * 判别主函数（每 50 ms 一次）。
 * 流程：
 *   ① 跑工况机 → 位图 + regime；
 *   ② 非 MEASURE：无条件 WAIT/L1/DRAIN（并清块），直接返回；
 *   ③ 屏蔽窗内：冻结判别（不分类、不投票），保持已采纳等级与许可；
 *   ④ 变化率抢先（连续 N tick 确认）；
 *   ⑤ 双侧门限分类 + 块投票 + 单侧迟滞；
 *   ⑥ PERMIT = (L0 && IMMERSED) → CLEAR/RECOV，否则按等级 DRAIN。
 */
void decision_update(decision_t *p_dec, const sensor_pipeline_t *p_pipe, uint32_t now_ms)
{
    const pipeline_out_t *p_out;
    decision_level_t cand;
    decision_level_t adopted;
    decision_state_t new_state;
    decision_action_t new_action;
    const char *reason;
    bool permit;
    bool evaluated = false;
    bool split     = false;
    bool rate_event = false;
    float d1_ec;
    float d2_ec;
    float d1_ntu;
    float d2_ntu;
    float e1_ec;
    float e2_ec;
    float e1_ntu;
    float e2_ntu;
    float ntu0_mv;
    float ntu_rate_pct_s;
    bool hold_clean;
    bool hold_soap;

    if ((NULL == p_dec) || (NULL == p_pipe))
    {
        return;
    }

    p_out = pipeline_get_out(p_pipe);
    if (NULL == p_out)
    {
        return;
    }

    /* ---------- ① 工况机（L0 位图 + L1 regime） ---------- */
    gate_update(&p_dec->gate, p_out, now_ms);

    p_dec->out.valid_bits  = p_dec->gate.bits;
    p_dec->out.regime      = p_dec->gate.regime;
    p_dec->out.immersed    = p_dec->gate.immersed;
    p_dec->out.blanking    = p_dec->gate.blanking;
    p_dec->out.gate_reason = gate_reason_str(&p_dec->gate);

    /* ---------- ② 非测量工况：禁止分类，强制排空 ---------- */
    /*
     * REGIME_DRY 与 REGIME_SETTLE 下 action 恒为 DRAIN、level 恒为 L1（未知）。
     * "没浸没"不再等价于"清水"——这是 S0/S1/S2 方向相反的根因修复。
     */
    if (REGIME_MEASURE != p_dec->gate.regime)
    {
        p_dec->blank_active = false;
        decision_clear_counters(p_dec, now_ms);

        decision_apply_output(p_dec, DEC_STATE_WAIT, DEC_LEVEL_L1, DEC_ACTION_DRAIN,
                              false, gate_reason_str(&p_dec->gate), now_ms);
        return;
    }

    /* ---------- M2h：肥皂锁存（覆盖稀释尾水） ---------- */
    if (p_dec->soap_latch_active)
    {
        if ((int32_t)(p_dec->soap_latch_until_ms - now_ms) > 0)
        {
            p_dec->rate_hits      = 0U;
            p_dec->upgrade_blocks = 0U;
            p_dec->downgrade_hits = 0U;
            p_dec->blank_active   = false;
            decision_reset_block(p_dec, now_ms);
            decision_apply_output(p_dec, DEC_STATE_SOAP, DEC_LEVEL_L2, DEC_ACTION_DRAIN,
                                  false, g_reason_soap_latch, now_ms);
            return;
        }
        p_dec->soap_latch_active = false;
    }

    /* ---------- ③ 自适应阈值（每 tick 更新：σ 变了阈值跟着变） ---------- */
    decision_update_thresholds(p_dec, p_out);
    d1_ec  = p_dec->out.d1_ec_mv;
    d2_ec  = p_dec->out.d2_ec_mv;
    d1_ntu = p_dec->out.d1_ntu_pct;
    d2_ntu = p_dec->out.d2_ntu_pct;

    /* ---------- ④ 换向屏蔽：冻结判别，不撤销许可 ---------- */
    if (p_dec->gate.blanking)
    {
        if (!p_dec->blank_active)
        {
            /* 屏蔽刚开始：清掉换向前攒的票与速率计数，避免"旧情境的票"决定新动作 */
            decision_clear_counters(p_dec, now_ms);
            p_dec->blank_active = true;
        }

        adopted = p_dec->out.level;
        /* 屏蔽窗内等级被冻结，许可也随之冻结：不重新看 gate.immersed，
         * 否则换向瞬态里一个越界采样就会把许可撤掉，形成"回收→换向→屏蔽→撤销"自激。 */
        permit  = (DEC_LEVEL_L0 == adopted);
        new_state  = permit ? DEC_STATE_CLEAR : decision_state_from_level(adopted);
        new_action = permit ? DEC_ACTION_RECOVER : DEC_ACTION_DRAIN;

        decision_apply_output(p_dec, new_state, adopted, new_action, permit,
                              g_reason_blank, now_ms);
        return;
    }
    p_dec->blank_active = false;

    /* ---------- ⑤ 变化率抢先（四约束） ---------- */
    /*
     * 只认正向（变干净不是"变脏"）；阈值 ≥120 mV/s（≈3× 实测换向瞬态 38 mV/s）；
     * 连续 rate_confirm_ticks 个 tick 才认；屏蔽窗内不参与（上面已返回）。
     */
    ntu0_mv        = (float)p_out->ntu0_ao_mv;
    ntu_rate_pct_s = 0.0f;
    if (ntu0_mv > 1.0f)
    {
        ntu_rate_pct_s = (p_out->ntu_rate_mv_s * 100.0f) / ntu0_mv;
    }

    rate_event = ((p_out->ec_rate_mv_s > p_dec->cfg.ec_rate_th_mv_s) &&
                  (p_out->d_ec_mv > p_dec->cfg.ec_rate_min_jump_mv))
              || ((ntu_rate_pct_s > p_dec->cfg.ntu_rate_th_pct_s) &&
                  (p_out->d_ntu_pct > 2.0f));

    if (rate_event)
    {
        if (p_dec->rate_hits < 0xFFU)
        {
            p_dec->rate_hits++;
        }

        if (p_dec->rate_hits >= p_dec->cfg.rate_confirm_ticks)
        {
            decision_clear_counters(p_dec, now_ms);

            decision_apply_output(p_dec, DEC_STATE_SOAP, DEC_LEVEL_L2, DEC_ACTION_DRAIN,
                                  false, g_reason_rate_jump, now_ms);
            return;
        }

        /* 第一次命中：等下一个 tick 确认，本轮不改变等级（单点毛刺不改动作） */
        adopted    = p_dec->out.level;
        permit     = (DEC_LEVEL_L0 == adopted);
        new_state  = permit ? DEC_STATE_CLEAR : decision_state_from_level(adopted);
        new_action = permit ? DEC_ACTION_RECOVER : DEC_ACTION_DRAIN;
        decision_apply_output(p_dec, new_state, adopted, new_action, permit,
                              g_reason_rate_pending, now_ms);
        return;
    }
    p_dec->rate_hits = 0U;

    /* ---------- ⑥ 双阈值迟滞分类 ---------- */
    /* 保持当前等级时放宽"离开边界"：已在 L0 则 d1 放大，已在 L2 则 d2 收缩 */
    hold_clean = (DEC_LEVEL_L0 == p_dec->out.level);
    hold_soap  = (DEC_LEVEL_L2 == p_dec->out.level);

    e1_ec  = hold_clean ? (d1_ec * (1.0f + p_dec->cfg.ec_hyst_ratio)) : d1_ec;
    e2_ec  = hold_soap  ? (d2_ec * (1.0f - p_dec->cfg.ec_hyst_ratio)) : d2_ec;
    e1_ntu = hold_clean ? (d1_ntu * (1.0f + p_dec->cfg.ntu_hyst_ratio)) : d1_ntu;
    e2_ntu = hold_soap  ? (d2_ntu * (1.0f - p_dec->cfg.ntu_hyst_ratio)) : d2_ntu;

    /* 交叉保护：放宽后的下边界不能越过上边界 */
    if (e1_ec > e2_ec)
    {
        e1_ec = e2_ec;
    }
    if (e1_ntu > e2_ntu)
    {
        e1_ntu = e2_ntu;
    }

    reason = g_reason_uncertain;
    cand   = decision_classify(p_out, e1_ec, e2_ec, e1_ntu, e2_ntu, &reason);

    /* ---------- ⑦ 块内一致 + 单侧迟滞 ---------- */
    adopted = decision_block_update(p_dec, cand, now_ms, &evaluated, &split);
    if (evaluated)
    {
        if (split)
        {
            reason = g_reason_block_split;
        }
        else
        {
            reason = decision_reason_from_level(adopted);
        }
    }
    else
    {
        /* 块未结束：等级保持，原因必须跟着"已采纳等级"走，不能用本 tick 候选的原因 */
        reason = decision_reason_from_level(adopted);
    }

    /*
     * ---------- ⑧ 回收许可（每 tick 重算，不是闩锁） ----------
     * PERMIT = (已采纳等级 == L0)。这里**不再**乘瞬时的 gate.immersed：
     *   - MEASURE 工况本身已经要求"浸没 + 稳定"，失浸没要连续 300 ms 才离开工况；
     *   - 若许可直接看瞬时位，单点 ADC 抖动就会让阀在 50 ms 内来回请求，
     *     min_dwell 只会把它拖成 ~1 Hz 往返 —— 正是文件头声称要消除的那种自激。
     * 换句话说：工况里的 IMMERSED 是"去抖后的浸没"，许可用它，不用裸采样。
     */
    permit = (DEC_LEVEL_L0 == adopted);

    new_state  = permit ? DEC_STATE_CLEAR : decision_state_from_level(adopted);
    new_action = permit ? DEC_ACTION_RECOVER : DEC_ACTION_DRAIN;

    decision_apply_output(p_dec, new_state, adopted, new_action, permit, reason, now_ms);
}

/* 读取最近一次输出（只读指针）。 */
/* M2h：清除 SOAP 锁存（重新标定/清基线时调用）。 */
void decision_clear_soap_latch(decision_t *p_dec)
{
    if (NULL == p_dec)
    {
        return;
    }
    p_dec->soap_latch_active   = false;
    p_dec->soap_latch_until_ms = 0U;
}

const decision_out_t *decision_get_out(const decision_t *p_dec)
{
    if (NULL == p_dec)
    {
        return NULL;
    }
    return &p_dec->out;
}

/* 状态 → 静态字符串（CSV 取值：WAIT/FLUSH/CLEAR/UNCERT/SOAP；越界→"UNCERT"）。 */
const char *decision_state_str(decision_state_t state)
{
    return ((unsigned)state < DEC_TABLE_LEN(g_state_name)) ? g_state_name[state] : "UNCERT";
}

/* 等级 → "L0"/"L1"/"L2"（越界→"L1"，与旧 switch 的 default 一致）。 */
const char *decision_level_str(decision_level_t level)
{
    return ((unsigned)level < DEC_TABLE_LEN(g_level_name)) ? g_level_name[level] : "L1";
}

/* 动作 → "RECOV"/"DRAIN"（越界→"DRAIN"，与旧 switch 的 default 一致）。 */
const char *decision_action_str(decision_action_t action)
{
    return ((unsigned)action < DEC_TABLE_LEN(g_action_name)) ? g_action_name[action] : "DRAIN";
}
