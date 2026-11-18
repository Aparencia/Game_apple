# 选择性洗手水回收系统（RA4M2）

基于 Renesas RA4M2（R7FA4M2AD3CFL）的自适应洗手水判别与回收控制固件。
通过 TDS/EC + 浊度（NTU）+ 水温，判断腔体内是清水、肥皂水还是无水和故障，
用三通阀把清水回收到回收桶、脏水排空。

- 当前固件：`SENSOR-NODE v2.4.1-FAST`
- 构建：`text 77620 / data 480 / bss 11352`，0 error / 0 warning
- 目标板：RA4M2，e2studio / FSP
- 版本管理：Git（本仓库从 v2.4.1 建立基线）

---

## 1. 目录结构

```text
.
├─ Docs/                                  设计、实施、现场分析与验收文档
├─ Selective_hand_wash_water_recycling_system/
│  ├─ src/                                应用源码（本项目主要修改区）
│  ├─ ra/                                 Renesas FSP / CMSIS（厂商代码）
│  ├─ ra_gen/                             FSP 生成的配置代码
│  ├─ ra_cfg/                             FSP 配置头文件
│  ├─ script/                             链接脚本
│  ├─ docs_sensor/                        传感器/接线/屏显说明
│  ├─ configuration.xml                   FSP 配置
│  ├─ .cproject / .project                e2studio 工程配置
│  └─ Debug/                              构建输出（Git 忽略）
├─ _tmp/                                  临时日志/构建记录（Git 忽略）
├─ .metadata/                             e2studio 工作区元数据（Git 忽略）
├─ .gitignore
├─ .gitattributes
└─ README.md
```

### 核心源码

| 文件 | 作用 |
|---|---|
| `src/app_main.c` | 主循环、自检、命令、CSV/TIP 输出、参数集中配置 |
| `src/sensor_pipeline.c` | ADC 采样、标定、EMA、基线、变化率 |
| `src/validity.c/.h` | 工况门控（FAULT/DRY/SETTLE/MEASURE/LOCKOUT）、浸没/稳定判据 |
| `src/decision.c/.h` | L0/L1/L2 分类、块投票、去抖、回收许可 |
| `src/display.c` | OLED 三页显示（P1 水质、P2 执行器、P3 信息） |
| `src/actuator_valve.c` | 三通阀/继电器、最小驻留、超时保护 |
| `src/sensor_*.c` | TDS、TS-300B 浊度、DS18B20 温度驱动 |

---

## 2. 构建

### 方式 A：e2studio（推荐）

1. 用 e2studio 导入 `Selective_hand_wash_water_recycling_system`；
2. 选择 `Debug` 配置，Build；
3. 输出：
   - `Debug/Selective_hand_wash_water_recycling_system.elf`
   - `Debug/Selective_hand_wash_water_recycling_system.hex`

> `Debug/` 不纳入 Git。e2studio 会按 `.cproject` 的 `sourceEntries`（`ra`、`ra_gen`、`src`）
> 重新生成 `Debug/src/subdir.mk`，因此新增的 `validity.c` 等文件能被自动扫描到。

### 方式 B：命令行（需已有 Debug 构建目录）

```powershell
$make = "D:\Ranesas\eclipse\plugins\com.renesas.ide.exttools.gnumake.win32.x86_64_4.3.1.v20240909-0854\mk\make.exe"
$tc   = "D:\Ranesas\toolchains\gcc_arm\arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi\bin"

& $make "PATH=$tc;C:\Windows\system32;C:\Windows" `
  -C "D:\Code\Game_apple\Selective_hand_wash_water_recycling_system\Debug" `
  -j8 -B all
```

> 路径按本机安装调整；`PATH` 必须在 make 命令行里覆盖，否则找不到 `arm-none-eabi-gcc`。

---

## 3. 快速验收

1. 烧录后串口启动行应为 `SENSOR-NODE v2.4.1-FAST`；
2. 空腔上电：`Auto dry n/200` → `Auto wait water`，阀不吸合；
3. 注清水淹过电极：约 1.5~2 s 进 `MEASURE`，随后 `Clear / Recover`；
4. 肥皂水：应稳定 `Soap / L2 / Drain`，不得回收；
5. 液面降到电极以下：≤300 ms 转 `DRY`，阀排空；
6. 拔探头/断线：`FAULT / SENSOR`，阀排空。

详细验收见 `Docs/实施说明_v2_20261116.md`、`Docs/开机自检_v2.1_20261116.md`。

---

## 4. 主要文档

| 文档 | 内容 |
|---|---|
| `Docs/软件层优化方案_v2_20261116.md` | 三层控制/有效性/决策的总体方案 |
| `Docs/实施说明_v2_20261116.md` | 代码落点、参数、验收、已知限制 |
| `Docs/开机自检_v2.1_20261116.md` | 自动干态/湿态、失败重试、OLED 状态 |
| `Docs/清水抖动分析与阈值修订_20261116.md` | 现场三份日志与 v2.2 阈值依据 |
| `Docs/自检识别修复_v2.3_20261117.md` | 清水仍待机的根因与差值判据 |
| `Docs/进水识别加速与OLED大小写_v2.4_20261117.md` | 自适应 SETTLE、OLED 首字母大写 |
| `Docs/洗手情景仿真_v2.4.1_20261117.md` | 真实洗手全过程仿真与指标 |
| `Docs/气泡问题分析与处理_20261117.md` | 上方进水气泡的机械/软件处理建议 |

---

## 5. 版本管理约定

- 主分支：`main`；
- 提交信息建议：`feat:` / `fix:` / `docs:` / `build:` / `test:`；
- **不要提交**：`Debug/`、`.metadata/`、`_tmp/`、`*.log`、`*.launch`；
- 发布固件如需入库，放在 `Firmware/<version>/`（当前尚未建立），不要从 `Debug/` 直接提交；
- 修改阈值/判据时，同步更新 `Docs/` 下对应说明和验收记录。

### 基线

- `v2.4.1-baseline`：首次纳入 Git 的完整源码基线（含 `validity.c`、v2.4.1 参数与文档）。
