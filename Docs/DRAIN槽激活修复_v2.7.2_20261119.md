# DRAIN 槽不激活修复（v2.7.2-B3）

- 日期：2026-11-19
- 固件：`SENSOR-NODE v2.7.2-B3`
- 构建：`text 84692 / data 480 / bss 11440`，0 error / 0 warning
- 基于：`v2.7.1-B3`

---

## 1. 现场日志

v2.7.1 上电：

```
# BASELINE SLOT DRAIN   ec0=348.6mV
# BASELINE SLOT RECOVER ec0=417.7mV
# SLOTS DRAIN=348.6 RECOVER=417.7 FULL=INVALID
# AUTOCAL state=DONE reason=full_timeout
```

FULL 段超时是预期的：FULL 要求 ≥ max(348.6,417.7)+10 = 427.7 mV，
而当时的“满水”只有 ~350 mV，所以保留前两槽并结束。

但之后 156 s 全部是：

```
WAIT / NOBASE / dEC=0 / g=0
```

没有任何 `# BASE slot=`，说明槽虽然有效，却从未被选为 active。

---

## 2. 根因

`app_baseline_task()` 旧逻辑：

```c
if (best != g_pipe.active_slot) {
    pipeline_baseline_select_slot(...);
}
```

- `active_slot` 初值是 **0（DRAIN）**；
- 标定结束后当前 EC≈350，最近槽也是 **DRAIN=0**；
- 于是 `best == active_slot`，条件为假；
- 但此时 `p_out->baseline_valid` 是 **false**（标定期间被 invalidate），
  并没有真正执行 `pipeline_baseline_select_slot()`。

结果：槽有效、active 无效、又永远不 select → 一直 `NOBASE`。

---

## 3. 修复

选择条件改为：

```c
if ((!p_out->baseline_valid)
    || (best != g_pipe.active_slot)
    || (p_out->ec0_mv25 与 slot.ec0 相差 > 0.5 mV)) {
    pipeline_baseline_select_slot(best);
}
```

- active 无效 → 强制 select；
- active 槽号相同但 ec0 不一致（手工 W / trim 后）→ 强制 select；
- 正常同槽同值 → 不重复切，不产生多余屏蔽窗。

---

## 4. 预期

烧录 v2.7.2 后，部分标定结束应看到：

```
# BASE slot=DRAIN ec0=348.6mV ec=350.0mV
```

随后：

```
CLEAR / L0 / DRAIN / OK
```

阀切 RECOVER：

```
# BASE slot=RECOVER ec0=417.7mV ec=418.0mV
CLEAR / L0 / RECOV / OK
```

不再出现整段 `NOBASE`，也不再用单基线 346 横跳。

---

## 5. Git

| ref | 内容 |
|---|---|
| `scheme-b3` | v2.7.2-B3 |
| `v2.7.2-B3` | 源码 + hex |
| `v2.7.1-B3` | 上一版 |
| `v2.7.0-B3` | 三槽首版 |

产物：`Docs/firmware/v2.7.2-B3.hex`
