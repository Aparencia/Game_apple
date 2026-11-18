# 进水识别加速 + OLED 大小写（v2.4.0）

- 日期：2026-11-17
- 固件：`SENSOR-NODE v2.4.1-FAST`
- 数据：`SaveWindows2026_11_17_14-41-50.TXT`（空腔→进水→满水→液面下降→空腔）
- 构建：`text 77620 / data 480 / bss 11352`，0 error / 0 warning（`_tmp/build_v24_final.log`）

> **v2.4.1 后续**：真实洗手仿真发现肥皂水会让 EC/浊度变化率超过 30/5，工况被踢回 SETTLE，
> 因此把稳定带抬到 **80 mV/s / 20%/s**；肥皂期间保持 MEASURE、直接判 `SOAP/L2`。
> 见 `Docs/洗手情景仿真_v2.4.1_20261117.md`。

---

## 1. 问题 1：水已淹没电极，系统仍显示“待机”

### 1.1 现场数据（同一份日志）

| 相对时间 | EC(mV) | ΔEC(mV) | ΔNTU(%) | rate(mV/s) | 工况 |
|---|---|---|---|---|---|
| 4.4 s | 328.0 | −97.9 | +13.1 | +454 | DRY（去抖中） |
| 4.6 s | 336.0 | −52.4 | +4.1 | +364 | 进入 SETTLE |
| 4.8 s | 334.9 | −27.1 | −0.6 | +219 | SETTLE |
| 5.4 s | 334.2 | −4.0 | −5.2 | +34 | SETTLE |
| 5.6 s | 334.2 | −2.4 | −5.5 | +17.9 | SETTLE（已稳定） |
| 6.4 s | 337.6 | −0.6 | −5.2 | +3.6 | SETTLE |
| **8.6 s** | 333.5 | −1.3 | −6.0 | −0.7 | **才进 MEASURE** |

结论：**水在 4.4 s 就到位，5.6 s 已经稳定**；旧逻辑固定等 `settle_ms=4000`，
所以从 4.6 s 一直“待机”到 8.6 s，现场感觉“明明淹到了还不认”。

### 1.2 v2.4 改法：自适应稳定期

`SETTLE → MEASURE` 现在有三条路径（任一满足）：

| 条件 | 说明 |
|---|---|
| **连续稳定 500 ms + 不早于 1.5 s + `ΔEC ≤ 0`** | 进水稳定且“不比基线脏”时提前进入；本数据约 **6.1~6.3 s** 即可进入，比原来早约 2.3 s |
| 满 4 s 且稳定 | 原有路径，dEC>0 或稳定较晚时使用 |
| 满 6 s 兜底 | 水一直在流、`stable` 始终不成立时也不永远待机 |

为什么加 `ΔEC ≤ 0`：进水过程中如果 EC 还在往上走（dEC>0，可能正变脏），
提前进入会让慢速污染在 dEC 尚未超过 d1 时被误判为 L0。限制为“水不比基线脏”
才走快速路径；dEC>0 仍等满 4 s，保持原来的安全边界。

> 预期效果：本数据中 `SETTLE 4.6 s → MEASURE ≈6.2 s → CLEAR ≈7.4 s`，
> 而旧版是 `MEASURE 8.6 s → CLEAR 9.75 s`。

### 1.3 新增参数

| 参数 | 默认 | 位置 |
|---|---|---|
| `settle_min_ms` | 1500 ms | `gate_cfg_t` |
| `settle_stable_ms` | 500 ms | `gate_cfg_t` |
| `settle_ms` | 4000 ms | 保持 |
| `settle_timeout_ms` | 6000 ms | 保持 |

---

## 2. 问题 2：OLED 英文全大写

### 2.1 改法

OLED 上可读的英文全部改为 **首字母大写、其余小写**；以下技术代码/单位仍保持大写：

- 等级：`L0/L1/L2`
- 通道：`EC/EC0/NTU/TDS/mV`
- 页面/工况短码：`P1/P2/P3`、`REG`、`EV`、`FLT/DRY/SET/MEA/LCK`
- 故障码：`NO_BASE/NO_CAL/SENSOR/ADC_TIMEOUT` 等（`note=` 行）
- 手动标签：`V1/V0`

### 2.2 主要变化对照

| 页面 | 旧 | v2.4 |
|---|---|---|
| P1 状态 | `CLEAR  RECOVER` | `Clear  Recover` |
| P1 状态 | `DRY    NO WATER` | `Dry    No water` |
| P1 状态 | `SETTLE WAIT 4s` | `Settle wait 4s` |
| P1 状态 | `LOCKOUT  CHECK` | `Lockout  check` |
| P1 状态 | `L2 SOAP  DRAIN` | `L2 Soap  Drain` |
| P1 状态 | `WAIT   EV:OFF` | `Wait   EV:Off` |
| P1 故障 | `L1 SENSOR_FAULT` | `L1 Sensor fault` |
| P1 故障 | `L1 NO_BASE` | `L1 No base` |
| P1 基线 | `BASELINE 37/40` | `Baseline 37/40` |
| 自检 | `AUTO WAIT WATER` | `Auto wait water` |
| 自检 | `AUTO FAIL` | `Auto fail` |
| 自检提示 | `SEND W IN CLEAN WATER` | `Send W in clean water` |
| P2 | `P2/3 EXEC MODE:AUTO` | `P2/3 Exec Mode:Auto` |
| P2 | `PIN_FAULT` / `SENSOR_FAULT` | `Pin fault` / `Sensor fault` |
| P3 | `STATE:… LEVEL:…` | `State:… Level:…` |
| P3 | `VALVE:… RLY:…` | `Valve:… Rly:…` |
| P3 | `MODE:… CNT:…` | `Mode:… Cnt:…` |
| 启动页 | `RA4M2 WATER JUDGE` / `OLED OK` | `RA4M2 Water Judge` / `OLED Ok` |

> 注意：`note=OK`、`L1`、`EC0=…` 这类诊断代码仍是大写，这是刻意的（便于和 CSV 对齐）；
> 如果希望连这些也改成大小写混排，需要再单独定规则。

---

## 3. 现场验证

1. 启动行：`SENSOR-NODE v2.4.0-FAST`；
2. 启动摘要出现 `# gate: ... settle=4.0/6.0s ...`（新参数在 `P`/`G` 里也能看到）；
3. **进水测试**：水淹到电极后，应在 **1.5~2.5 s** 内离开“待机”进入 `MEASURE`，
   而不是旧版的固定 4 s；
4. 记录 `# GATE regime=MEASURE` 的时刻与 `# BASELINE OK` 的时刻，差值应明显缩短；
5. OLED 检查：`Clear Recover`、`Dry No water`、`Auto wait water` 等应为首字母大写。

### 如果提前进入后判成 UNCERT/L1

- 说明水在 1.5 s 时尚未完全稳定（dEC 还 >d1 或 dNTU 高），这是安全侧排空；
  等稳定后会自动回 CLEAR；
- 若希望更保守，把 `settle_min_ms` 调回 2500~3000；若希望更快，可降到 1000
  （不建议低于 1000，避免进水瞬态被当成水质）。

---

## 4. 已知限制

1. 快速路径只在 `ΔEC ≤ 0`（水不比基线脏）时启用；dEC>0 仍等 4 s，避免慢速污染误判。
2. 进水过程中 dNTU 会出现 +30~+45 的界面尖峰（水/空气光学差异），
   但此时工况仍为 DRY/SETTLE，不参与 L2 判定；**液面下降**时仍可能出现约 1 s 的
   伪 SOAP（见 `Docs/清水抖动分析与阈值修订` 的 P6），属另一个待修项。
3. 没有液位/流量传感器，软件只能靠 EC/NTU 推断“水是否到位”；结构/传感器仍是根治。
