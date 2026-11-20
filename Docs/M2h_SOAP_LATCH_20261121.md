# M2h：SOAP_LATCH + NTU 速率 + 液位相关稳定带（2026-11-21）

- 固件：`SENSOR-NODE v2.8.0-LEVEL-M2h`
- 构建：`text 85672 / data 480 / bss 11536`，0 error / 0 warning
- 基于：`v2.8.0-LEVEL-M2g`

---

## 1. SOAP_LATCH（防稀释尾水误回收）

### 触发

任一条件产生 L2/SOAP：

- dNTU 高；
- EC d2 + NTU；
- EC/NTU 速率前沿。

### 动作

```
L2 第一次被采纳
→ soap_latch_active = true
→ 45 s 内强制 SOAP/L2/DRAIN
→ decision.reason = SOAP_LATCH
→ 不判 CLEAR、不回收
```

参数：

```c
soap_latch_ms = 45000U
```

日志：

```
state=SOAP level=L2 action=DRAIN reason=SOAP_LATCH
```

说明：

- 锁存只由第一次 L2 启动；
- 锁存期间不会每 tick 续期；
- 45 s 后恢复正常分类；若仍然 L2，会重新锁存；
- 重新标定（K2 长按 / Z）会清除锁存。

---

## 2. NTU 速率判据

原来：

```
ntu_rate_th_pct_s = 150 %/s   // 实际永不触发
```

M2h：

```
ntu_rate_th_pct_s = 15 %/s
且 dNTU > 2%
```

原因：

- 肥皂浊度前沿约 16 %/s；
- 加 2% 绝对跳变，避免单点噪声。

---

## 3. 稳定带改为液位相关

| 条件 | EC 稳定带 | NTU 稳定带 |
|---|---|---|
| 液位稳定 + 水质安静（|dEC|<3mV、|dNTU|<3%） | **30 mV/s** | **5 %/s** |
| 液位变化 / 水质有事件 / 液位无效 | **80 mV/s** | **20 %/s** |

目的：

- 清水稳定时用紧带，减少“假稳定”；
- 肥皂/液位变化时用宽带，避免把正常肥皂前沿踢回 SETTLE。

---

## 4. SETTLE 用液位驱动

| 条件 | settle_ms | min | stable | timeout |
|---|---|---|---|---|
| 液位稳定 + 安静 | **1000** | **500** | **300** | **3000** |
| 其他 | 4000 | 1500 | 500 | 6000 |

不再固定等 4 s；液位稳定后 0.5~1 s 即可进入 MEASURE。

---

## 5. 空腔自检 2 s

```c
APP_AUTOCAL_DRY_MS = 2000U   // 40 点
```

原 3 s → 2 s，减少开机等待。

---

## 6. 湿态采基线

保持：

```
稳定 300 ms 后采 2 s
最多等 3 s
5 mV 漂移作废
```

AO 不再作为硬门槛，只告警。

---

## 7. 基线槽

保持 M2f：

```
阀 DRAIN   → DRAIN 槽
阀 RECOVER → RECOVER 槽
FULL 有效且 EC 到 FULL 平台 → FULL 槽
阀位槽无效 → 最近有效槽兜底
```

液位变化只冻结 trim，不冻结槽切换。

---

## 8. AO 改为 advisory

- 水体确认：以 **EC ≥ EC_DRY + 80mV** 为主；
- AO 低只打印：

```
CAL WARN: AO low ... treated as advisory
```

- baseline validate 也不再因 AO 低拒绝；
- 目的：避免塑料片/气泡导致 AO 偏低 → Auto fail。

风险：

- 如果水确实脏/肥皂，AO 不再阻止标定；
- 必须配合人工确认“这是清水”。

---

## 9. trim 限制

```c
1.0 mV ≤ |dEC| ≤ 4.0 mV
且 level_hold=false
```

超过 4 mV 不再 trim，避免把弱肥皂/液位漂移学成“干净”。

---

## 10. 已知限制

- 这份日志里超声全程 `valid=0, edge=0`，液位没有参与；
  需先修 Echo/塑料片/安装。
- 最后一段肥皂水 dEC≈+1.1mV、dNTU≈+0.3%，低于传感器检测限；
  M2h 也不会把它单独判成 SOAP；只能靠 SOAP_LATCH 覆盖同一次洗手尾水。
- 锁存 45 s 是时间策略，不是浓度测量；不同洗手时长需现场调整。

---

## 11. Git

| ref | 内容 |
|---|---|
| `sensors-level-flow` | 当前 M2h |
| `v2.8.0-LEVEL-M2h` | 源码 + hex |
| `v2.8.0-LEVEL-M2g` | AO 降级标定 |
| `v2.8.0-LEVEL-M2f` | 阀位优先选槽 |
