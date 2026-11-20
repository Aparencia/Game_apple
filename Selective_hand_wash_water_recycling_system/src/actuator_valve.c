/*
 * actuator_valve.c —— 二位三通电磁阀 / 继电器执行器实现。
 *
 * 设计要点（位置映射 RECOVER=吸合 / DRAIN=释放=fail-safe、有效电平可配、min_dwell 防抖、
 * max_on 连续通电保护、写后回读自检、吸合计数）见 actuator_valve.h 文件头。
 *
 * 与 app_main 的分工：app_main 决定"该回收还是排空"，每个 50 ms tick 调
 * valve_request + valve_task；本模块不读传感器、不打印、不 delay。
 */

#include "actuator_valve.h"

/* ---------- 内部工具 ---------- */

/* 由"吸合电平"反推"释放电平"（fail-safe 电平 = 排空位电平）。 时序/边界：纯函数。 注意：高触发模块 → 释放=LOW；低触发模块 → 释放=HIGH。 */
static bsp_io_level_t valve_off_level(const valve_t *p_valve)
{
    if (BSP_IO_LEVEL_HIGH == p_valve->cfg.relay_on_level)
    {
        return BSP_IO_LEVEL_LOW;
    }
    return BSP_IO_LEVEL_HIGH;
}

/* 逻辑位置 → P104 输出电平。 时序/边界：纯函数。 注意：逻辑位置与电平的映射只允许在这里出现，避免别处再算反一次。 */
static bsp_io_level_t valve_level_for_pos(const valve_t *p_valve, valve_pos_t pos)
{
    if (VALVE_POS_RECOVER == pos)
    {
        return p_valve->cfg.relay_on_level;
    }
    return valve_off_level(p_valve);
}

/* 写 P104 并立即回读校验（防止端口宏写错/引脚被别的外设占用时的"静默不动"）。 时序/边界：两个寄存器操作，微秒级；只在 valve_init/valve_task 调用。
 * 注意：回读的是引脚实际电平（RA4M2 的 PORT 读取），继电器模块输入电流只有几 mA， 不会把推挽输出拉反，因此回读是可靠的配置自检手段。 */
static bool valve_pin_write(valve_t *p_valve, bsp_io_level_t level)
{
    fsp_err_t      err;
    bsp_io_level_t readback = BSP_IO_LEVEL_LOW;

    err = R_IOPORT_PinWrite(&g_ioport_ctrl, p_valve->cfg.pin, level);
    if (FSP_SUCCESS != err)
    {
        return false;
    }

    err = R_IOPORT_PinRead(&g_ioport_ctrl, p_valve->cfg.pin, &readback);
    if (FSP_SUCCESS != err)
    {
        return false;
    }

    return (readback == level);
}

/* ---------- 公开接口 ---------- */

/* fail-safe 初始化：保存配置，把 P104 配成输出并锁在"释放"电平，再回读校验。 参数：p_valve —— 句柄（非 NULL）；p_cfg —— 配置（非 NULL）。
 * 时序/边界：app_main 的第一条外设操作；约几十 µs；不打印。
 * 注意：FSP 的 PinCfg 把"输出电平(PODR)"和"方向(PDR)"写进同一个 PmnPFS 寄存器， 不会出现"先输出高再拉低"的毛刺；即便如此，硬件 10 kΩ 下拉/上拉仍然必须焊。 */
void valve_init(valve_t *p_valve, const valve_cfg_t *p_cfg)
{
    valve_out_t    init_out = { 0 };
    bsp_io_level_t off_level;
    uint32_t       pin_cfg;
    fsp_err_t      err;

    if ((NULL == p_valve) || (NULL == p_cfg))
    {
        return;
    }

    /* 1) 保存并限幅配置：非法有效电平按高触发处理（与工程默认一致） */
    p_valve->cfg = *p_cfg;
    if ((BSP_IO_LEVEL_HIGH != p_valve->cfg.relay_on_level) &&
        (BSP_IO_LEVEL_LOW != p_valve->cfg.relay_on_level))
    {
        p_valve->cfg.relay_on_level = BSP_IO_LEVEL_HIGH;
    }
    p_valve->last_switch_ms   = 0U;
    p_valve->on_since_ms      = 0U;
    p_valve->blocked_until_ms = 0U;

    /* 2) 输出清零（C99 结构体赋值，不用 memset，避免多一个头文件依赖） */
    p_valve->out            = init_out;
    p_valve->out.requested  = VALVE_POS_DRAIN;
    p_valve->out.position   = VALVE_POS_DRAIN;
    p_valve->out.request_reason = "INIT";
    p_valve->out.switch_reason  = "INIT";

    /* 3) fail-safe 第一步：把 P104 配成输出，初值=继电器释放（排空）电平 */
    off_level = valve_off_level(p_valve);
    pin_cfg   = (uint32_t)IOPORT_CFG_PORT_DIRECTION_OUTPUT;
    if (BSP_IO_LEVEL_HIGH == off_level)
    {
        pin_cfg |= (uint32_t)IOPORT_CFG_PORT_OUTPUT_HIGH;
    }
    else
    {
        pin_cfg |= (uint32_t)IOPORT_CFG_PORT_OUTPUT_LOW;
    }

    err = R_IOPORT_PinCfg(&g_ioport_ctrl, p_valve->cfg.pin, pin_cfg);
    if (FSP_SUCCESS != err)
    {
        p_valve->out.pin_fault     = true;
        p_valve->out.fault_count   = 1U;
        p_valve->out.switch_reason = "INIT_FAIL";
        return;
    }

    /* 4) 回读校验（同时确认 P104 没有被别的功能占用/写错端口宏） */
    if (!valve_pin_write(p_valve, off_level))
    {
        p_valve->out.pin_fault     = true;
        p_valve->out.fault_count   = 1U;
        p_valve->out.switch_reason = "INIT_FAIL";
        return;
    }

    p_valve->out.initialized   = true;
    p_valve->out.switch_reason = "INIT_DRAIN";
}

/* 记录上层想要的位置（判别/手动/安全侧），并不立即碰 GPIO。
 * 参数：p_valve —— 句柄（NULL 直接返回）；pos —— 目标位置（非法值按 DRAIN）； now_ms —— 当前时刻（本版本不用，保留接口以便以后做"最短排空时间"）； p_reason —— 请求来源静态串（NULL 时保留上一次来源）。
 * 时序/边界：O(1)，可在每个 tick 调用。 注意：来源字符串必须是静态常量（如 "AUTO"/"MANUAL_RECOVER"），模块只存指针不复制。 */
void valve_request(valve_t *p_valve, valve_pos_t pos, uint32_t now_ms, const char *p_reason)
{
    (void)now_ms;

    if (NULL == p_valve)
    {
        return;
    }

    if ((VALVE_POS_RECOVER != pos) && (VALVE_POS_DRAIN != pos))
    {
        pos = VALVE_POS_DRAIN;   /* 安全侧：不认识的位置一律按排空处理 */
    }

    p_valve->out.requested = pos;
    if (NULL != p_reason)
    {
        p_valve->out.request_reason = p_reason;
    }
}

/* 执行器状态机：冷却解除 → 判定目标 → 驻留判断 → 写引脚+回读 → 计数/计时 → 连续吸合超时强制释放。 时序/边界：每 50 ms 调用一次；只做寄存器读写，微秒级，不阻塞。
 * 注意：初始化失败/写失败时逻辑位置保持"排空"，但物理上继电器由硬件下拉/上拉与 12V 船型开关兜底；软件不能替代那两道硬件安全措施。 */
void valve_task(valve_t *p_valve, uint32_t now_ms)
{
    valve_pos_t    desired;
    valve_pos_t    old_pos;
    bsp_io_level_t target_level;
    uint32_t       dwell_elapsed;

    if (NULL == p_valve)
    {
        return;
    }

    /* 1) 初始化失败：不写引脚，逻辑位置恒为排空（硬件侧保证继电器释放） */
    if (!p_valve->out.initialized)
    {
        p_valve->out.position  = VALVE_POS_DRAIN;
        p_valve->out.relay_on  = false;
        p_valve->out.on_time_ms = 0U;
        return;
    }

    /* 2) 冷却结束解除超时锁存（无符号差值 < 0x80000000 表示 now >= blocked_until） */
    if (p_valve->out.max_on_blocked)
    {
        if ((uint32_t)(now_ms - p_valve->blocked_until_ms) < 0x80000000U)
        {
            p_valve->out.max_on_blocked = false;
        }
    }

    /* 3) 目标位置：上层请求；冷却期内禁止吸合（强制排空） */
    desired = p_valve->out.requested;
    if ((VALVE_POS_RECOVER == desired) && p_valve->out.max_on_blocked)
    {
        desired = VALVE_POS_DRAIN;
    }

    /* 4) 假执行器模式：保持排空电平，只记录请求（实验 3 的 LED 演示兼容） */
    if (!p_valve->cfg.enabled)
    {
        p_valve->out.position   = VALVE_POS_DRAIN;
        p_valve->out.relay_on   = false;
        p_valve->out.on_time_ms = 0U;
        return;
    }

    /* 5) 需要换向时先过"最小驻留"（防继电器抖动/阀频繁换向） */
    dwell_elapsed = (uint32_t)(now_ms - p_valve->last_switch_ms);
    if ((desired != p_valve->out.position) && (dwell_elapsed >= p_valve->cfg.min_dwell_ms))
    {
        target_level = valve_level_for_pos(p_valve, desired);
        old_pos      = p_valve->out.position;

        if (!valve_pin_write(p_valve, target_level))
        {
            /* 写/回读失败：不更新逻辑位置，下个 tick 重试；故障锁存供上层报警 */
            p_valve->out.fault_count++;
            p_valve->out.pin_fault     = true;
            p_valve->out.switch_reason = "PIN_FAULT";
            return;
        }

        p_valve->out.position   = desired;
        p_valve->out.relay_on   = (VALVE_POS_RECOVER == desired);
        p_valve->last_switch_ms = now_ms;

        if (VALVE_POS_RECOVER == desired)
        {
            p_valve->out.switch_count++;
            p_valve->out.switch_reason = p_valve->out.request_reason;
            p_valve->on_since_ms       = now_ms;
            p_valve->out.on_time_ms    = 0U;
        }
        else
        {
            if (VALVE_POS_RECOVER == old_pos)
            {
                p_valve->out.total_on_ms += (uint32_t)(now_ms - p_valve->on_since_ms);
            }
            p_valve->out.switch_reason = "DRAIN";
            p_valve->out.on_time_ms    = 0U;
        }
    }

    /* 6) 连续吸合计时 + 超时强制释放（阀线圈连续工作制保护） */
    if (VALVE_POS_RECOVER == p_valve->out.position)
    {
        p_valve->out.on_time_ms = (uint32_t)(now_ms - p_valve->on_since_ms);

        if ((p_valve->cfg.max_on_ms > 0U) && (p_valve->out.on_time_ms >= p_valve->cfg.max_on_ms))
        {
            if (!valve_pin_write(p_valve, valve_off_level(p_valve)))
            {
                p_valve->out.fault_count++;
                p_valve->out.pin_fault     = true;
                p_valve->out.switch_reason = "PIN_FAULT";
                return;
            }

            p_valve->out.total_on_ms += p_valve->out.on_time_ms;
            p_valve->out.position   = VALVE_POS_DRAIN;
            p_valve->out.relay_on   = false;
            p_valve->out.on_time_ms = 0U;
            p_valve->out.max_on_blocked = true;
            p_valve->last_switch_ms     = now_ms;
            p_valve->out.switch_reason  = "MAX_ON";
            p_valve->blocked_until_ms   = now_ms + p_valve->cfg.cooldown_ms;
        }
    }
}

/* 返回执行器输出只读指针。 参数：p_valve —— 句柄（可为 NULL）。 返回：输出指针；p_valve 为 NULL 时返回 NULL。 时序/边界：O(1)。
 * 注意：指针生命周期与句柄相同（app_main 静态变量），不要在栈上复制后再长期持有。 */
const valve_out_t *valve_get_out(const valve_t *p_valve)
{
    if (NULL == p_valve)
    {
        return NULL;
    }
    return &p_valve->out;
}

/* 逻辑位置 → 静态字符串。 时序/边界：纯函数。 注意：只用于日志/状态行，不参与控制判断。 */
const char *valve_pos_str(valve_pos_t pos)
{
    if (VALVE_POS_RECOVER == pos)
    {
        return "RECOVER";
    }
    return "DRAIN";
}
