# orangepi-master 代码审查报告

| 项 | 内容 |
|---|---|
| 审查对象 | `C:\Users\zhai\Desktop\orangepi-master\orangepi-master`（下文路径均相对该仓库根） |
| 项目性质 | 2026 全国大学生智能汽车竞赛 · 室外 5G 远程驾驶无人车赛 · 车端工程 |
| 规模 | 218 个文件；`dev/` 运行时 Python 约 2700 行（含脚本与单测共 6439 行）；`doc/` 文档约 5700 行 |
| 审查日期 | 2026-09-23 |
| 审查范围 | 全部 Python 源码、shell 脚本、systemd 单元、网络配置、单测，以及 `doc/` 全部中文文档 |
| 问题统计 | **A 类致命 7 项 · B 类严重 12 项 · C 类文档错误 13 组 · D 类遗留 7 项** |

---

## 目录

- [一、结论速览](#一结论速览)
- [二、项目概览](#二项目概览)
- [三、错误清单](#三错误清单)
  - [A 类：致命（程序跑不起来 / 数据丢失）](#a-类致命程序跑不起来--数据丢失)
  - [B 类：严重（功能错误 / 契约破缺 / 安全相关）](#b-类严重功能错误--契约破缺--安全相关)
  - [C 类：文档与事实不符](#c-类文档与事实不符)
  - [D 类：一致性 / 遗留](#d-类一致性--遗留)
- [四、值得肯定的部分](#四值得肯定的部分)
- [五、修复优先级与建议](#五修复优先级与建议)
- [附录 A：核查方法](#附录-a核查方法)
- [附录 B：核查边界](#附录-b核查边界)

---

## 一、结论速览

### 1.1 一句话结论

**该仓库当前的自动驾驶主链路是跑不起来的**——视觉进程 `vision_main.py` 在模块顶层 import 了两个已被归档删除的模块（`vision/lane_scan.py`、`vision/lane_hough.py`），一启动即 `ModuleNotFoundError`，导致 `car-mode.sh auto` 必然失败。此外还有一处同步脚本缺陷会**不可逆地删除车端模型与现场标定值**。

### 1.2 问题分布

| 级别 | 数量 | 含义 |
|---|---|---|
| **A 类 致命** | 7 | 程序无法启动、数据丢失、或「降速」变「停车」级别的功能性破坏 |
| **B 类 严重** | 12 | 功能未接通、契约破缺、死参数、安全规范未落实 |
| **C 类 文档错误** | 13 组 | 文档描述与代码/实测/规则原文矛盾，会误导现场操作 |
| **D 类 遗留** | 7 | 死代码、接口不同构、文档自述不一致 |

### 1.3 错误速查表

| 编号 | 级别 | 位置 | 问题摘要 |
|---|---|---|---|
| A1 | 🔴 致命 | `dev/vision/vision_main.py:28-29` | 顶层 import 已归档模块 → 视觉进程启动即崩，自主链路零可用 |
| A2 | 🔴 致命 | `scripts/ssh_sync.py:185` | `--delete` 会删除车端 `*.rknn` 与 `config/site.yaml`（有意排除项被判为 stale） |
| A3 | 🔴 致命 | `dev/scripts/diag/lane_probe.py:225-231` | `_scan()` 自我递归；`_annotate` 无条件访问 `obs.center_x` → 抓帧即崩 |
| A4 | 🔴 致命 | `dev/scripts/car-mode.sh:259` | 使用未定义变量 `$DEV_MAIN`/`$DEV_SUB` + `set -u` → `status` 中途退出 |
| A5 | 🔴 致命 | `dev/scripts/capture_dataset.py:26` | 缺 `sys.path` 引导且包装脚本不设 `PYTHONPATH` → 采集流程（P0-1 硬阻塞）跑不起来 |
| A6 | 🔴 致命 | `dev/scripts/bench_test.py:251-255` | 扫线守卫挡住 `--no-motor`/`--steer-test`/`--calibrate` 三条文档命令 |
| A7 | 🔴 致命 | `dev/scripts/bench_test.py:111` | 降级降速算出 1537.5us < 死区 1545us → 「降速维持」实为停车；单测已固化该值 |
| B1 | 🟠 严重 | `dev/common/protocol.py:108-125` | `to_dict()` 漏掉 `elements` → 元素列表永远不过 UDP |
| B2 | 🟠 严重 | `dev/control/fsm.py:128` | `enter_traffic_light()` 无调用点 + 视觉端恒发 `None` → 红绿灯状态不可达 |
| B3 | 🟠 严重 | `dev/control/fsm.py:117-123` | `AVOID_CONE`/`PARKING` 是空壳（不绕行、不入库） |
| B4 | 🟠 严重 | `dev/scripts/net/car-net.sh:66` | 无参函数里用 `$1` + `set -u` → 网卡改名成功后脚本自杀 |
| B5 | 🟠 严重 | `dev/main.py:55-56` | 只捕捉 SIGINT/SIGTERM，缺 SIGABRT/SIGQUIT（违反 `AGENTS.md` §1.1.4） |
| B6 | 🟠 严重 | `dev/vision/camera_guard.py:93` | docstring 声称等待设备释放，实际未调用 `wait_device_free()` |
| B7 | 🟠 严重 | `dev/control/lane_arbiter.py:49` | `hold_s` 是死参数，`ARBITER_HOLD_S` 调了没用 |
| B8 | 🟠 严重 | `dev/scripts/net/stream-watch.sh:36-39` | 函数内用 `exit 0` 而非 `return` → sub 流整轮不被检查 |
| B9 | 🟠 严重 | `dev/scripts/car-test/test_all.sh:54` | `i2cdetect \| grep -q '40'` 恒真 → PCA9685 检测永远 PASS |
| B10 | 🟠 严重 | `dev/scripts/car-dial.service:7` | ExecStart 路径与部署路径不一致；脚本写死错误 APN `cmnet` |
| B11 | 🟠 严重 | `dev/config/settings.py` | 约 30 个 `LANE_*` 配置项零引用（唯一消费者已归档） |
| B12 | 🟠 严重 | `dev/scripts/lane_ref_test.py:252` | 转向符号与主程序相反，且不读 `STEER_SIGN` |
| C1 | 🟡 文档 | `doc/比赛规则/规则概要.md:56` | 红绿灯「抢行」罚时写成 50s（原文 20s） |
| C2 | 🟡 文档 | `doc/图传与模式切换方案.md:27` | 链路图把「副摄 video2」标成「云台」（应为下摄） |
| C3 | 🟡 文档 | `doc/小车部件测试手册.md:49,58` | 仍用已停用的官方服务器与旧流名 `cam_car0003` |
| C4 | 🟡 文档 | 三处 | `car-dial` 存废说法互相打架 |
| C5 | 🟡 文档 | `AGENTS.md:156,168` | 测试清单含不存在的 `test_lane_scan`，且「8 个文件」与实际 13 个不符 |
| C6 | 🟡 文档 | `AGENTS.md:271-272` | 观测量契约与代码不符（`LaneObservation` 不存在、`blue_ratio` 字段不存在） |
| C7 | 🟡 文档 | `doc/具体实施方案.md:48` | 仍写 `vision_main.py --debug`，该参数不存在 |
| C8 | 🟡 文档 | `doc/具体实施方案.md:690` 等 | 「安全组是唯一卡点」已过期 |
| C9 | 🟡 文档 | 多处 | 帧率/实例数/metric/扫线代码存废/frpc 状态/白线饱和度等 6 组跨文档矛盾 |
| C10 | 🟡 文档 | 多处 | 仓库根路径、`train/split_dataset.py`、`code/README.md` 三份文档、`trace_boundary()`、`frames/*.png` 等过时引用 |
| C11 | 🟡 文档 | `doc/具体实施方案.md:194-195` | 选型自相矛盾：E18-D80NK 标称 3~80cm < 自己要求的 1m |
| C12 | 🟡 文档 | 多处 | 型号/GPU 串台、X11 说法冲突、`config/README.md` 内容损坏、`hardware.yaml` 不存在等 |
| C13 | 🟡 文档 | `dev/scripts/lane_ref_test.py:51-52` | 自身 docstring 的默认值与代码不符 |
| D1–D7 | ⚪ 遗留 | 多处 | 行数自述不一致、Mock 接口不同构、无退出条件的采集循环、`cam-identify` 异常路径、`talk_player_loop.sh` 写死服务器、`wifi_active()` 死代码、`is_native` 措辞 |

---

## 二、项目概览

### 2.1 背景与硬件

2026 全国大学生智能汽车竞赛「室外 5G 远程驾驶无人车赛」的参赛工程。

| 项 | 内容 |
|---|---|
| 主控 | Orange Pi 5（RK3588S），带 NPU |
| 底盘 | XT-NetRC 阿克曼底盘 |
| 执行机构 | PCA9685（I2C5 @0x40）驱动舵机 / 电调 / 云台 |
| 摄像头 | 两路 **USB**：`/dev/video0`(index 0) = icspring = **主摄 = 云台摄**；`/dev/video2`(index 2) = Global Shutter = **副摄 = 下摄（巡线用）** |
| 通信 | SIM8262E-M2 5G 模组 |
| 目标 | **完赛，并尽量快**；因规则罚时重（停车超 20s 不动即失败、停错车位 +100s、撞锥 +20s、压线 +10s），文档明确「可靠性优先于速度」 |

### 2.2 比赛任务链

```
遥控阶段(5G)：起点 ──遥控──▶ 切换区（停入、启动自动驾驶）
自主阶段：挡板移除（蓝板 = 发车信号）
  → TRACKING      沿跑道线巡线
  → ZEBRA_STOP    斑马线停 10s + 播报队名
  → TRAFFIC_LIGHT 红绿灯（光电触发 3~10s 随机倒计时，红灯停、绿灯行）
  → AVOID_CONE    两处锥桶（蓝/红锥随机位置）
  → PARKING       上下两个停车区，裁判随机放蓝板遮挡一个 → 停未遮挡区（四轮入区）
  → 30s 内远程扫码付 0.01 元
```

### 2.3 仓库结构

| 路径 | 内容 |
|---|---|
| `AGENTS.md` | 362 行的「AI 作业手册」，等价于团队开发规范：安全红线、当前优先级、代码约定、已知陷阱表（约 25 条）、文档地图 |
| `dev/` | **比赛代码**，本地 ⇄ 车上 `/root/dev`（`main.py` / `config/` / `common/` / `vision/` / `control/` / `hardware/` / `scripts/` / `img/` / `tests/` / `tools/`） |
| `doc/` | 全部中文文档，入口是 `doc/README.md`（文档地图） |
| `code/` | PC 侧训练环境（`environment.yml` / `requirements.txt` / `setup_yolo_env.ps1` / `verify_yolo.py`） |
| `scripts/` | PC 侧运维助手：SSH 隧道、增量同步、串口救援、数据集切分 |

### 2.4 车端架构：双进程 UDP JSON

```
vision 进程：摄像头 → 扫线(每帧) + 元素检测(降频) → UDP JSON ──┐
                                                              ▼
control 进程：UDP → 仲裁层(LaneArbiter) → FSM → Planner(PID) → Driver(PCA9685)
```

**`dev/common/protocol.py`** 是两进程的观测量契约。`PerceptionMessage` 定义了：

- `center_x / left_x / right_x`——像素，基于 640 宽画面，y 向下
- `lane_confidence`——本帧扫线置信度 0~1，供仲裁层决定降级
- `board_blocked` / `start_released`——发车挡板检测（边沿触发用）
- 锥桶计数、`has_zebra_crossing`、`has_sign_a/b`、`traffic_light_state`
- `elements`——统一元素列表（模型可插拔）

并做了旧字段兼容：`is_barrier` ≡ `board_blocked`、`is_rxd_flag` ≡ `has_zebra_crossing`；`from_dict()` 容忍字段缺失与 `null`。

### 2.5 模块详解

#### `dev/vision/`（7 个模块）

| 文件 | 职责与要点 |
|---|---|
| `vision_main.py` | 视觉主进程。含 `CameraSwitcher` 做**双摄分时复用**：发车阶段用 `video0` 云台摄看蓝板，放行后切 `video2` 下摄巡线。扫线每帧跑、元素检测按 `--detect-every` 降频（默认 3 帧一次），每 0.5s 写 `/tmp/smartcar_status.json` |
| `start_gate.py` | **全项目最扎实的模块**。发车遮挡检测，三路判据：① 最大蓝色连通域面积占比 ② 蓝色主导度 `mean(clip(B-max(R,G),0,255))`（比 HSV 硬阈值抗曝光漂移）③ 可选拉普拉斯方差骤降。核心不变量：**没见过板，绝不放行**——严格边沿触发（先见板 N 帧 → 再连续 M 帧无板才放行），从设计上杜绝「上电即冲」 |
| `rknn_detector.py` | RKNN 推理封装。注释记录了三条实测踩坑：必须 **NHWC**（喂 NCHW 会输出完全错位）、必须 **RGB**（喂 BGR 置信度掉到 0.01 量级）、必须 **uint8 0~255**（归一化在图内完成）。尺寸相符时零缩放，否则 letterbox 并用 `restore()` 还原坐标 |
| `elements.py` | 模型输出 → **统一元素名**的解耦层（`zebra`/`cone`/`parking_sign`/`parking_area`/`blue_board`/`light`）。profile 写在 `config/model_profile.yaml`：`legacy`=上届 9 类分割、`smartcar2026`=自训 8 类检测、`car4cls`=车上现有 4 类试验模型。换模型只改配置 |
| `postprocess.py` | 解码 + 逐类 NMS + 语义映射，含 `_find_detection_tensor()` 在 RKNN 多路输出里找检测头 |
| `camera_guard.py` | 摄像头独占守卫：进出自动停/恢复 ffmpeg 推流服务，纯 `/proc` 扫描设备占用者，停完 `reset-failed` 防看门狗误重启 |
| `debug_view.py` | 带 X11 的可视化调试（车上无 X 服务器，属备用手段） |

#### `dev/control/`（8 个模块）

| 文件 | 职责与要点 |
|---|---|
| `controller.py` | 主循环：UDP → 仲裁 → FSM → Planner → Driver。含 failsafe 断连保护（超时 `safe_stop()`）、转向 slew rate 限速（`STEERING_SLEW_DEG_PER_S = 120`）、非阻塞语音播报、状态切换时清 PID/滤波状态 |
| `lane_arbiter.py` | 置信度仲裁层，三档降级：正常用本帧 → 连续 N 帧低置信度则降油门 + 用最近有效值维持 → 超时 `should_stop` |
| `fsm.py` | 任务状态机 `WAIT_START → TRACKING → ZEBRA_STOP → TRAFFIC_LIGHT → AVOID_CONE → PARKING → DONE`，含边沿复核与逐状态防抖计数 |
| `planner.py` | 观测量 → 控制量。误差限幅 → `ErrorFilter` → PID → 叠加中位。含 `_adaptive_throttle_scale()`（误差小提速、误差大减速），且**专门规避过电调死区陷阱**（注释：「再乘 0.85 → 1542us 掉进死区，车直接停死」） |
| `pid.py` | 位置式 PID，相对上届 C++ 实现的四点改进：**带 dt**、**积分限幅**、**微分低通**、**`reset()` 清状态**（官方漏了这条，代价是切状态时「踢腿」） |
| `filters.py` | 中位数离群剔除 + 越新权重越大的加权平均（移植自上届 `ErrorFilter.h`） |
| `udp_server.py` | 非阻塞收包（0.5s 超时），兼容 JSON 解析失败 |
| `driver.py` | `ControlTarget` → 硬件。未 `arm()` 时只允许安全停止 |

#### `dev/hardware/`

`pca9685.py`（脉宽↔占空比、角度↔脉宽映射、`center_all()`）、`mock.py`、`audio.py`（用 `subprocess.Popen` 非阻塞播放，避免阻塞控制环导致 failsafe 失效）、`imu.py`（**空壳**，`read_yaw()` 恒返回 0）。

#### `dev/scripts/`（运维重头）

- `car-mode.sh`——模式切换总入口（`status`/`auto`/`manual`/`estop`/`stream`/`logs`），带启动前体检、GO 确认、**任何退出路径都恢复手动模式**
- `bench_test.py`（570 行）——台架测试，安全层共用 `bench_common.py`（电调归零 / 舵机回中 / 退出逐项验证恢复远程控制阶段）
- `net/car-net.sh`——蜂窝优先（APN 轮询 → 厂商兜底）、WiFi 账本兜底、只占一条上行、关机优雅断开 CID
- `diag/lane_probe.py`、`diag/model_probe.py`——扫线诊断、模型自检与类别 id 反推
- 一整套 systemd 单元：frp 两端、mediamtx、`stream-watch` 看门狗、car-net 系列

#### `dev/tests/`（13 个文件）

纯函数 + `__main__` 风格单测，不依赖硬件、不引入 pytest。

### 2.6 文档体系

文档整理规范（`doc/README.md` 有职责划分表与维护约定：**一篇文档只有一个职责**）。

| 文档 | 职责 |
|---|---|
| `具体实施方案.md`（734 行） | 设计与决策的唯一来源。D1~D8 决策、8 类检测 + 3 类分割的**类别表契约**（§3.1.3）、发车遮挡检测（§3.1.7）、停车区几何先验法（§3.1.9，因裁判放法不可预测而从 v2.2 重写） |
| `执行路线图.md` | 计划/进度/风险/待确认的唯一来源，P0/P1/P2 批次 + 8 周排期 |
| `数据与模型方案.md` | 采集→标注→切分→训练→ONNX→RKNN→上板全链路 |
| `专项方案/` | 红绿灯识别与通过、停车识别与入库 |
| `车端网络方案.md` / `图传与模式切换方案.md` | 网络与图传运维，含大量实测复盘 |
| `比赛规则/` | 规则初稿 PDF + 文本提取 + 4 张关键插图 + 规则概要 |
| `实地调试清单.md` / `小车部件测试手册.md` | 现场执行清单与硬件自检 |

### 2.7 当前项目状态（据文档自述）

- **唯一硬阻塞**：训练数据 **0 标注**（2858 张原始图片已入库，但采集矩阵缺口大）
- **2026-09-19 重要路线调整**：实验室打印跑道是亮面覆膜 + 直射顶灯，白线与覆膜反光**分不开**；云台 9 档俯仰扫描最好只有 conf 0.24（可用线 0.4）。⇒ 现有两路摄像头**都拿不到俯视车道视角**，本阶段行驶策略改为**不依赖白线**：直线用固定舵角 + 中位微调，变道/绕锥/入库开环触发，停车与斑马线靠元素检测

---

## 三、错误清单

### A 类：致命（程序跑不起来 / 数据丢失）

#### A1. 视觉进程根本无法启动 —— 顶层 import 已归档模块

**位置**：`dev/vision/vision_main.py:28-29`、`dev/vision/debug_view.py:24-25`

```python
from vision.lane_hough import HoughLaneScanner
from vision.lane_scan import LaneScanner
```

**现象**：`dev/vision/lane_scan.py` 与 `dev/vision/lane_hough.py` **不在仓库里**，只存在于 `dev/attic/lane-old-20260919.tar.gz`（`tar -tzf` 可见 `vision/lane_scan.py`、`vision/lane_hough.py`、`tests/test_lane_scan.py`、`tests/test_lane_hough.py`）。而这两行是**模块级无条件 import**，两个文件后面还真的在用（`vision_main.py:115-116`、`debug_view.py:95`）。

**后果链条**：`car-mode.sh auto` 起视觉进程 → `ModuleNotFoundError` 立即退出 → 15 秒内等不到 `/tmp/smartcar_status.json` → `car-mode.sh` 判定「视觉进程启动失败」并 `die` 退回手动模式。**整个自主跑车链路（含元素检测、UDP 上报、发车检测）当前零可用**。

**对照**：`bench_test.py:71-76` 和 `lane_probe.py:227-230` 都做了 try/except 兜底，只有真正跑车用的这两个文件没有。

**关键澄清**：新的循迹实现是自包含的 `lane_ref_test.py`（432 行，参考实现版），文档也在推荐它（`实地调试清单.md` §2.6）。但它只是个**独立测试脚本，从未接入双进程链路**——这才是真正的缺口。

**修复方向**：把 `vision_main.py:28-29` 改为可选导入（照 `bench_test.py` 的写法），或把 `lane_ref_test.py` 的扫线实现提取为 `vision/lane_*.py` 并在此接线。

---

#### A2. `ssh_sync.py --delete` 会删除车端的模型和现场标定值

**位置**：`scripts/ssh_sync.py:185`（配合 `:34`、`:36`、`:39` 的排除规则）

```python
stale = [rel for rel in sorted(remote) if rel not in local]
```

**问题**：`local` 是**经过排除规则过滤后**的列表，而 `remote` 是远端**全量**列表。

| 排除规则 | 被排除的内容 |
|---|---|
| `MODEL_SUFFIXES = (".rknn", ".pt", ".onnx", ".onnx.rknn")` | 车端唯一的模型 |
| `LOCAL_ONLY_FILES = {"wifi-ledger.conf", "site.yaml"}` | **现场标定值**（`servo_center_angle`、`target_x`，由 `bench_test.py --calibrate` 写入） |

这些文件**有意不入同步**，于是必然出现在 `remote` 却不在 `local` → 被判为 `stale` → 命中 `:212-214` 的 `sftp.remove()`。`--delete` 删除的恰恰是「被有意排除同步的资源」。

**后果**：`/root/dev/models/*.rknn` 与 `/root/dev/config/site.yaml` 被不可逆删除。与 `AGENTS.md` §1.2.4「车上不要执行格式化、`rm -rf`、批量清理等不可逆操作」直接冲突。文件头把它描述为「删除远端多余文件（谨慎！默认不删）」，容易让人放心使用。

**修复方向**：`stale` 判定同时套用 `is_excluded()`，或维护一份显式的「云端本地资源」保护名单。

---

#### A3. `lane_probe.py` 的扫线函数是自我递归，且抓帧即崩

**位置**：`dev/scripts/diag/lane_probe.py:225-231`

```python
def _scan(frame):
    """有扫线模块就用它，没有就返回 None（探针仍能给出掩膜诊断）。"""
    try:
        from vision.lane_scan import LaneScanner
    except ImportError:
        return None
    return _scan(frame)          # ← 调用的是自己，不是 LaneScanner
```

**两处问题**：

1. `lane_scan` 不存在 → 恒返回 `None`；而 `_annotate()` 在 `obs = _scan(frame)` 之后**无条件**访问 `int(obs.center_x)`（`:187`）→ `AttributeError: 'NoneType' object has no attribute 'center_x'`。
2. 即便按计划恢复 `lane_scan.py`，`return _scan(frame)` 会变成**无限递归 `RecursionError`**。

**后果**：`AGENTS.md` §4.3 推荐的诊断命令 `sudo python3 scripts/diag/lane_probe.py --camera 2 --frames 6` 第一帧就 traceback，叠加图一张都存不下来（`main()` 第 456 行无条件传 `out_dir=args.out`，所以必然走到 `_annotate`）。`实地调试清单.md:31` 与 `自动驾驶开发方案.md:149` 承诺的 ✅/❌ 结论也不可能出现——`verdict_of()` 永远走「（无扫线模块）本轮只做掩膜诊断」分支。

**附带问题**：`_print_roi_sweep()`（`:147-161`）在循环里反复调 `_scan(frame)` 却不传 `ratio`，即便模块恢复，6 行「不同 ROI 下的扫线结果」数值也会完全一样。

---

#### A4. `car-mode.sh status` 因变量名写错而中途退出

**位置**：`dev/scripts/car-mode.sh:259`（对照第 40 行）

```bash
# 第 40 行只定义了这两个：
DEV_GIMBAL="/dev/video0"; DEV_DOWN="/dev/video2"

# 第 259 行用的是另两个（从未定义）：
for dev in "$DEV_MAIN" "$DEV_SUB"; do
```

**问题**：脚本是 `set -euo pipefail`，未定义变量的展开在 bash 里是**致命错误**，`|| true` 也拦不住。

**后果**：被文档列为「看当前模式/服务/进程/温度/摄像头占用」的首选命令，在打印到「摄像头占用」时整个 shell 就退出了，后面的状态文件解析、图传可达性检查全部看不到。

---

#### A5. `capture_dataset.py` 缺 `sys.path` 引导，采集流程按文档跑必崩

**位置**：`dev/scripts/capture_dataset.py:26`

```python
from vision.camera_guard import open_camera, start_ffmpeg, stop_ffmpeg
```

**问题**：它是全仓唯一**没有** `sys.path.insert(...)` 的入口脚本。

| 脚本 | 是否有 sys.path 引导 |
|---|---|
| `dev/scripts/bench_test.py` | ✅ |
| `dev/scripts/diag/lane_probe.py` | ✅ |
| `dev/scripts/lane_ref_test.py` | ✅ |
| `dev/scripts/diag/model_probe.py` | ✅ |
| **`dev/scripts/capture_dataset.py`** | ❌ |

而包装脚本也都没设 `PYTHONPATH`：`run_capture.sh`、`dev/img/camera0.sh:98`、`dev/img/camera2.sh:95` 都是裸 `python3 "$ROOT_DIR/scripts/capture_dataset.py"`。对比 `car-mode.sh:425` 启动 vision_main 时明确设了 `PYTHONPATH="$ROOT_DIR"`。

**后果**：`python3 <脚本>` 的 `sys.path[0]` 是脚本所在目录，故 `from vision.camera_guard import ...` 抛 `ModuleNotFoundError: No module named 'vision'`。**P0-1（补采集矩阵）这条唯一的硬阻塞任务目前跑不起来**（除非手动 `export PYTHONPATH`）。

> 附注：同目录的 `record_video.py` 是自包含的（只 import 标准库），**没有**这个问题，不要一并误改。

---

#### A6. `bench_test.py` 的扫线守卫会挡掉所有不需要扫线的档位

**位置**：`dev/scripts/bench_test.py:251-255`

```python
if not _HAS_LANE_SCAN and not args.no_lane:
    print("[BENCH] 扫线模块已移除：循迹档不可用。请用 scripts/lane_ref_test.py 做循迹测试，"
          "或加 --no-lane 只测蓝板/直行。")
    return 2
```

**问题**：判断只看 `args.no_lane`，与 `--no-motor` / `--calibrate` / `--steer-test` 无关，而这几条分支都在 251 行**之后**（`--calibrate` 分支在 `:283`，`--steer-test` 分支在 `:346`）。

**受影响的三条文档命令**（均不需要扫线）：

| 命令 | 用途 | 结果 |
|---|---|---|
| `bench_test.py --no-motor` | 只看读数（不动电机） | `return 2` 拒绝运行 |
| `bench_test.py --steer-test --allow-motion` | 转向方向自检 | `return 2` 拒绝运行 |
| `bench_test.py --calibrate 30` | 静止标定车道中心 | `return 2` 拒绝运行 |

**连带问题**：加 `--no-lane` 绕过后，标定分支 `:296` 又直接 `obs = scanner.scan(frame)`，此时 `scanner` 是 `None` → `AttributeError`（主循环 `:388` 反而判了 `args.no_lane`）。

**危害**：`--steer-test` 是判定 `steer_sign` 方向的**唯一**手段，它跑不了意味着现场无法确认转向符号。

---

#### A7. 降级降速算出的脉宽落在电调死区，车会直接停死

**位置**：`dev/scripts/bench_test.py:111`（`scaled_pulse`），配合 `dev/config/settings.py:172`

```python
def scaled_pulse(out_us: float, scale: float, neutral_us: float = NEUTRAL_US) -> float:
    ...
    return float(neutral_us) + (float(out_us) - float(neutral_us)) * k
```

**计算**：默认 `--speed-us 1575`、`ARBITER_HOLD_THROTTLE_SCALE = 0.5`
→ `1500 + 75 × 0.5 = 1537.5us`

**问题**：本项目实测**死区上沿是 1545us**（`ESC_DEADBAND_US`，1500 停、1540 不转、≈1545 才起转）。1537.5us 落在死区内 —— 也就是说「丢线时降速维持、不要急停」在默认参数下**等于停车**，正是注释里说要避免的「走走停停」。

**更糟的是单测把 bug 固化了**：`dev/tests/test_bench_logic.py:175`

```python
assert scaled_pulse(1575.0, 0.5) == 1537.5
```

**对照**：`planner.py:58-69` 的 `_adaptive_throttle_scale` 在百分比域里专门规避过同一个坑（注释：「降级路径（如半油门 50% ≈ 1550us）本来就在电调死区边缘，再乘 0.85 → 42.5% ≈ 1542us 掉进死区，车直接停死」），说明团队知道这个陷阱，只是**没在恒等域同步修**。同样地，`planner.py:97-98` 的 `CONE_THROTTLE_SCALE = 0.5` 算出来是 1550us，只比死区高 5us，余量偏薄。

**修复方向**：`scaled_pulse` 增加死区下限钳制（如 `max(result, ESC_DEADBAND_US + margin)`，除非 `scale == 0`），并同步修正单测期望值。

---

### B 类：严重（功能错误 / 契约破缺 / 安全相关）

#### B1. 协议 `to_dict()` 不序列化 `elements`，元素列表永远不过网

**位置**：`dev/common/protocol.py:108-125`（`to_dict`）对照 `:104`（`from_dict`）

`to_dict()` 返回 15 个字段，**漏掉了 `elements`**；而 `from_dict()` 专门解析它（`elements=[e for e in (data.get("elements") or []) if isinstance(e, dict)]`）。

**后果**：`vision_main.py:225` 明明组装了 `elements=[e.to_dict() for e in elements]`，但 `:227` 发出去的是 `msg.to_dict()` → **控制端永远收不到元素列表**。违反 `AGENTS.md` §5.1「新增字段要同时改 `from_dict`/`to_dict`」。

**为何测不出来**：`test_protocol.py:41-51` 的 `test_roundtrip_new_fields` 只覆盖了 6 个标量字段，没覆盖 `elements`。

**修复方向**：`to_dict()` 补 `"elements": self.elements`，并在 roundtrip 测试里加 `elements` 断言。

---

#### B2. 红绿灯状态机不可达，`traffic_light_state` 永远为 `None`

**位置**：`dev/control/fsm.py:128`（`enter_traffic_light`）、`dev/vision/vision_main.py:224`

`fsm.py:112-115` 有 `TRAFFIC_LIGHT` 状态的完整逻辑（连续 5 帧绿灯才放行），但进入它只有 `enter_traffic_light()` 一条路，而**全仓库没有任何调用点**（`grep -rn enter_traffic_light dev/` 只命中定义本身）。同时视觉端 `vision_main.py:224` 硬编码：

```python
traffic_light_state=None,       # 红绿灯本阶段占位（见 doc/自动驾驶开发方案.md §8）
```

**后果**：红绿灯任务目前**既无感知也无触发**。文档（`执行路线图.md` P1-3）说这是待做项，但 `AGENTS.md:77` 把 P1-3 标成「本轮落地」，容易误读为已可用。

---

#### B3. FSM 的 `AVOID_CONE` / `PARKING` 是空壳

**位置**：`dev/control/fsm.py:117-123`

```python
elif self.state == State.AVOID_CONE:
    if self._cone_frames == 0 and now - self._state_started >= 0.5:
        self._set(State.TRACKING)
elif self.state == State.PARKING:
    if now - self._state_started >= 2.0:
        self._set(State.DONE)
```

- `AVOID_CONE`：只等锥桶消失 0.5 秒就回 TRACKING，**不绕行**
- `PARKING`：只等 2 秒就无条件进 `DONE`，**不入库、不判左右车位**

`controller.py:18` 的 `DRIVING_STATES` 也把这两个状态排除在动力输出之外（进这两个状态立即电调归零，安全侧是对的）。

**问题**：文档（`执行路线图.md` P1-3/P1-4）已承认未做，但代码里没有显式的「未实现」标记，容易让人以为能跑。

---

#### B4. `car_net_up.sh` 里 `$1` 未定义 + `set -u`，改名成功后脚本自杀

**位置**：`dev/scripts/net/car-net.sh:66`（函数 `ensure_wwan0_name()`）

```bash
log "renamed $1 -> wwan0"
```

两个调用点（`:151`、`:168`）都**不传参**。脚本是 `set -uo pipefail`，未绑定 `$1` 展开为致命错误。

**后果**：当 systemd 把网卡改名成 `wwx<MAC>`（`AGENTS.md` §6 明确记载这发生过两次，且会导致「所有写死 `wwan0` 的脚本含厂商二进制全部失效」）时，脚本会在 `ip link set ... name wwan0` **成功之后**立刻死掉，拨号流程半途中断——正是这段代码要解决的场景。

---

#### B5. 主程序只捕获 SIGINT/SIGTERM，与安全规范不符

**位置**：`dev/main.py:55-56`

```python
signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)
```

`AGENTS.md` §1.1.4 要求「统一收敛到一个 `cleanup()`，捕获 `SIGINT/SIGTERM/SIGABRT/SIGQUIT`」。

**对照**：`bench_common.install_signal_guard()` 做对了（覆盖 SIGINT/TERM/HUP/QUIT/ABRT + `atexit` 注册恢复），但**主程序没做**。

**后果**：SIGABRT/SIGQUIT（如 Python 内部致命错误、某些 kill 组合）路径不走信号处理；虽然 `main.py:62-63` 的 `finally: driver.shutdown()` 能兜住电机安全，但 `controller.shutdown()`（含 `audio.stop()`）不会被调用。

---

#### B6. `camera_guard.stop_ffmpeg` 的 docstring 与实现不符，`wait_device_free` 无人调用

**位置**：`dev/vision/camera_guard.py:90-106`

docstring 写：

> 停止推流服务，并**确认它们真的释放了设备**才返回。

实际实现是：`systemctl stop` → 轮询 `is-active` → `reset-failed` → 固定 `sleep(settle_s)`。它**没有**调用同文件 `:78` 的 `wait_device_free()`——该函数**只有单测在用**（`test_camera_guard.py:35-47`），生产代码零调用。

**连带问题**：`vision_main.py` 的 `CameraSwitcher.use()`（`:52-66`）自己写裸 `cv2.VideoCapture`，没用 `open_camera()` 的重试机制。这与 `AGENTS.md` §1.2.2「Python 里用 `dev/vision/camera_guard.py` 的上下文管理器，不要手写」不一致——而 `camera_guard.py:16-21` 的文件头注释恰恰记录了「`systemctl stop` 返回**不等于**设备已释放」这个实测坑。

---

#### B7. 仲裁层的 `hold_s` 是死参数

**位置**：`dev/control/lane_arbiter.py:49`（赋值）、`:83`（实际判定）

模块 docstring 的表格承诺「维持超过 `HOLD_S` → 标记 degraded」，`settings.ARBITER_HOLD_S = 0.8` 也标注为现场可调，但 `update()` 里实际用的是：

```python
degraded = self._low_frames >= self.low_conf_frames      # 按帧数，不是按时间
```

`hold_s` 从头到尾没被读过，`ArbiterOutput.held_s` 也无人消费。**现场按注释调 `ARBITER_HOLD_S` 不会有任何效果。**

---

#### B8. `stream-watch.sh` 在函数里用 `exit 0` 而非 `return`

**位置**：`dev/scripts/net/stream-watch.sh:36-39`

```bash
  if (( now - last < BACKOFF_S )); then
    echo "[watch] $svc 需要重启（$reason），但距上次仅 $((now-last))s，退避中"
    exit 0        # ← 应该是 return
  fi
```

`SERVICES` 顺序是 main → sub，所以 `ffmpeg-stream` 在退避窗口内时，整个脚本结束，`ffmpeg-stream-sub` 那一轮**完全不被检查**，坏掉的流可以一直死着。

---

#### B9. 硬件自检里有一个「永远通过」的 PCA9685 判定

**位置**：`dev/scripts/car-test/test_all.sh:54`

```bash
i2cdetect -y 5 2>/dev/null | grep -q '40' && ok "PCA9685 @0x40 (I2C5)" || bad "PCA9685 未探测到 (I2C5)"
```

**问题**：`i2cdetect` 输出的行首标签本身就含 `40:`，所以 `grep -q '40'` 恒为真 → **总线上没有设备也永远 PASS**。

**项目自己修过同一个坑**：`car-mode.sh:177-178` 的注释写明「旧写法 `grep '\b40\b'` 会匹配行号 '40:'，总线上没有设备也永远判在线」，并改成了 `awk '$1=="40:" && $2=="40"'`。但这份权威自检脚本（`小车部件测试手册.md` 的配套工具）没同步修。

---

#### B10. `car-dial.service` 的 ExecStart 路径与仓库部署路径不一致，且脚本写死了错误 APN

**位置**：`dev/scripts/car-dial.service:7`

```ini
ExecStart=/root/car_net_up.sh
```

**问题 1（路径）**：仓库里该脚本在 `dev/scripts/car_net_up.sh`，按 `ssh_sync.py` 的映射（`dev/` → `/root/dev/`）会落在 `/root/dev/scripts/car_net_up.sh`，与 unit 里的 `/root/car_net_up.sh` 不是同一个文件。照仓库重装/换车会 `203/EXEC`。

**问题 2（APN）**：`car_dial_once.sh` / `car_net_up.sh` 写死 `APN=cmnet` + `wwan0`，而 `dev/scripts/net/car-net.conf:16` 明确记载「**实测本车联通卡：3gnet 可通、cmnet 会 CID 超时**」，`AGENTS.md` §6 也把它列为已知陷阱。

---

#### B11. 约 30 个 `LANE_*` 配置项现在零引用

**位置**：`dev/config/settings.py:86-142`、`:179-192`

以下配置项**唯一的消费者是已归档的 `lane_scan/lane_hough`**，现在全仓零引用：

```
LANE_CANNY_LOW / LANE_CANNY_HIGH / LANE_CANNY_ADAPT / LANE_CANNY_TARGET_LO/HI
LANE_CANNY_HIGH_MAX / LANE_CANNY_HIGH_MIN / LANE_CANNY_LOW_MAX / LANE_CANNY_LOW_MIN
LANE_HOUGH_THRESH / LANE_HOUGH_MIN_LEN / LANE_HOUGH_MAX_GAP
LANE_HOUGH_ALLOW_SINGLE / LANE_HOUGH_CENTER_JUMP_PX / LANE_HOUGH_HALF_W_DEFAULT_PX
LANE_HOUGH_HOLD_FRAMES / LANE_HOUGH_SINGLE_SIDE_CONF
LANE_SLOPE_MIN / LANE_SLOPE_MAX / LANE_MIN_SEG_LEN_SUM / LANE_LOOKAHEAD_RATIO
LANE_FIT_TOL_PX / LANE_FIT_ITERS / LANE_FIT_ROW_STEP / LANE_FIT_MIN_SPAN_PX
LANE_CONVERGE_MAX_RATIO / LANE_LINE_MIN_LEN_PX* / LANE_LINE_MIN_ELONG* / LANE_LINE_MAX_THICK_PX*
LANE_TRACK_MAX_JUMP_PX / LANE_TRACK_MAX_MISS / LANE_TRACK_WIDTH_TOL
LANE_TRACK_MIN_LEN / LANE_TRACK_SEED_STEP
LANE_PAIR_W_MIN_PX / LANE_PAIR_W_MAX_PX
LANE_WEIGHT_PEAK / LANE_ROW_STEP / LANE_MIN_VALID_ROWS / LANE_MAX_LINE_W_PX / LANE_WHITE_*
```

（带 `*` 的项在 `dev/scripts/` 的独立脚本里还有引用；其余为纯死配置。）

**另外两个本来就没用的**：

- `ZEBRA_THROTTLE_SCALE = 0.6`（注释写「P1-2 接入减速曲线前先用比例」，实际零引用）
- `START_USE_EDGE_DENSITY` / `START_EDGE_DENSITY_THRESH` —— `start_gate.py:123` 的 `edge_density()` 定义了却只有单测在调

**问题**：`AGENTS.md` §5.1 要求「阈值、限幅、坐标目标全部进 `config/settings.py`，标注单位与用途，现场可改」。现在这个文件里**一大半东西改了没用**，现场按注释调参会白费时间。

---

#### B12. `lane_ref_test.py` 的转向符号与主程序相反，且不读 `STEER_SIGN`

**位置**：`dev/scripts/lane_ref_test.py:252` 对照 `dev/control/planner.py:52,90`

```python
# lane_ref_test.py
angle = 90.0 - pid

# planner.py
error_units = error_px / float(settings.LANE_ERROR_SCALE) * float(settings.STEER_SIGN)
target.steering = float(settings.SERVO_CENTER_ANGLE) + self.steering_offset(x, dt)
```

同一个正误差（`center_x > target_x`），一个让角度**变小**、一个让角度**变大**——**必有一个把车打死**（按 `bench_test.py:352` 的判据「角度大 = 右转」，`lane_ref_test.py` 的方向是反的）。

**连带问题**：`lane_ref_test.py` 完全不读 `settings.STEER_SIGN`，也没有 `--steer-sign` 参数。所以文档反复给出的补救办法「往 `config/site.yaml` 写 `steer_sign: -1`」（`bench_test.py:148/361/491`、`实地调试清单.md:90`）**对它无效**——偏偏 `bench_test.py:252` 现在把用户往它这里推，而 `--steer-test` 又因 A6 跑不了，**现场无法判定谁对**。

---

### C 类：文档与事实不符

#### C1. 规则概要写错红绿灯罚时

**位置**：`doc/比赛规则/规则概要.md:56`

```
- **红绿灯**：未在停车区停车罚20s；抢行或未停车罚50s。
```

**规则原文**（`doc/比赛规则/rules2026_text.txt:203-210`）：

> (4) 无人车在进行红绿灯停车时，若未正确停在停车范围内（车头前轮超过红绿灯、未在停车区内），**加罚20s**。
> (5) 红绿灯停车时，未按照红绿灯指示**抢灯通行，则有效时间不减去红绿灯停车时长，并加罚20s**，若未进行红绿灯停车，**加罚50s**。

**正确写法**见 `doc/专项方案/红绿灯识别与通过方案.md:19`「抢灯 +20s 且不扣等待时长，未停 +50s」。`规则概要.md` 与之冲突，把「抢行」写成 50s，会误导「宁可多等」的策略权衡。

---

#### C2. 图传文档自己的链路图把摄像头角色写反

**位置**：`doc/图传与模式切换方案.md:27` 对照同文件 `:157`

```text
主摄 video0（固定）─┐
                    ├─ ffmpeg ──RTSP──▶ mediamtx :8554  (cam_car0027 / _sub)
副摄 video2（云台）─┘
```

同文件 §6 表格（`:153-159`）写的是：

| 设备 | 角色 |
|---|---|
| `/dev/video0`（index 0） | **主摄 ＝ 云台摄像头** |
| `/dev/video2`（index 2） | **副摄 ＝ 下摄** |

`AGENTS.md:12,303`、`dev/img/README.md:9-10`、`car-mode.sh:31-33` 全部是「video0 = 云台 / video2 = 下摄」。所以 §1 链路图里的「副摄 video2（云台）」把下摄标成了云台。

---

#### C3. 部件测试手册仍在使用已停用的官方服务器和旧流名

**位置**：`doc/小车部件测试手册.md:49`、`:58`、`:67`

```
| T9 | ffmpeg 推流进程 | ≥1 个，目标含 cam_car0003 |
应见 rtsp://82.157.204.126:17005/cam_car0003（主）+ /cam_car0003_sub（副）
```

**反驳证据**：`doc/图传与模式切换方案.md:15`「官方服务器 **不再使用**（`82.157.204.126` 的推流与 whip/whep 反代都已改掉）」、`:13` 流名是 `cam_car0027`；`doc/官方目录索引.md:40`「图传/网页：car0027」。

**代码同样过时**：`dev/scripts/car-test/test_all.sh:67` 的 `pgrep -f 'cam_car0003' … || warn "推流目标缺失/非 car0003"` —— 这条自检现在**必然 WARN**。

---

#### C4. `car-dial` 的存废三处说法打架

| 出处 | 说法 |
|---|---|
| **新口径** `doc/车端网络方案.md:191,206` | 「`car-dial.service`…已 `disable`，职责被 `car-net.service` 取代」「**处置：`systemctl disable --now car-dial.service`**」 |
| `doc/远程连接与服务器手册.md:257` | 「蜂窝 wwan0 无 IP → car-dial 未起 / APN 不对：`systemctl restart car-dial`，APN=cmnet」 |
| `doc/小车部件测试手册.md:15,42,141` | 「小车有网（WiFi 热点或蜂窝 car-dial）」；T4-4 判定「car-dial（蜂窝）**active**（红旗：蜂窝是比赛刚需）」；排障表同上 |
| `doc/官方目录索引.md:24` | 仍把 `car-dial.service` 列为在用服务 |

同一份 `车端网络方案.md:55` 还记载「**实测本车联通卡：3gnet 可通、cmnet 会 CID 超时**」，而旧文档教的正是 APN=cmnet。

---

#### C5. `AGENTS.md` 的单测清单含不存在的文件，且数量说错

**位置**：`AGENTS.md:156`、`:168`

```bash
for t in test_filters test_pid test_lane_arbiter test_protocol test_start_gate test_lane_scan test_control; do
  "$PY" tests/$t.py
done
```

> 提交前**必须**跑通上面全部 8 个文件。

**事实**：

- `dev/tests/test_lane_scan.py` **不存在**（在 attic 包里）→ 按文档执行第一步就 `No such file`
- `dev/tests/` 实际有 **13** 个文件
- 清单**漏掉** 6 个已存在的：`test_bench_logic` / `test_camera_guard` / `test_elements` / `test_lane_ref` / `test_rknn_detector` / `test_site_config`
- `doc/执行路线图.md:36`、`doc/实地调试清单.md:14` 用的是正确口径「13 个文件」

---

#### C6. `AGENTS.md` §5.3 的观测量契约与代码不一致

**位置**：`AGENTS.md:271-272`

```python
LaneObservation(center_x: float, confidence: float, left_x, right_x, valid_rows, source)
StartGateState(blocked: bool, armed: bool, released: bool, blue_ratio: float, timed_out: bool)
```

| 文档写的 | 代码实际 |
|---|---|
| `LaneObservation` | 仓库现存代码**零命中**，只在归档的 `vision/lane_scan.py:22` 里有 |
| `StartGateState(..., blue_ratio: float, ...)` | `dev/vision/start_gate.py:44-53` 的字段是 `blocked / armed / released / **metrics: BlueMetrics** / frames_blocked / frames_clear / timed_out / note`；`BlueMetrics`（`:30-36`）才有 `area_ratio / pixel_ratio / dominance / detail`。**没有 `blue_ratio` 字段**（`blue_ratio()` 是同文件 `:119` 的模块级函数） |

---

#### C7. `--debug` 参数不存在，但主方案仍写它

**位置**：`doc/具体实施方案.md:48`

```
| 调试方式 | 调试窗口直接集成进 vision_main.py --debug，显示 FPS、画面、关键识别状态 |
```

`dev/vision/vision_main.py:90-111` 的 argparse **没有** `--debug`。`doc/运行调试与系统调优方案.md:39` 明确写「`vision_main.py` 本身**没有** `--debug` 参数」，`doc/README.md:90` 还记录着「本次整理已修正 `运行调试与系统调优方案.md` 的 `--debug` 错误引用」——**主方案这处漏修了**。

---

#### C8. 「安全组」这个「唯一卡点」已经过期

| 出处 | 说法 |
|---|---|
| `doc/具体实施方案.md:690` | 「⚠️ **待办（唯一卡点）**：阿里云安全组需放行 8554/tcp、8889/tcp…**未放行前图传不通**」 |
| `doc/执行路线图.md:88` | 「**唯一卡点** = 阿里云安全组放行 8554/8889/8189(TCP+UDP)/8888」 |
| **实测（更新）** `doc/图传与模式切换方案.md:222` | 「**安全组现状**：已放行 `7000` `8080` `8554` `8889` `8189(TCP/UDP)` `8888`；待补：`80`、`22`、`2222`」 |

---

#### C9. 六组跨文档矛盾

| # | 事项 | 说法 A | 说法 B（正确） |
|---|---|---|---|
| 1 | 元素检测帧率 | `具体实施方案.md:219`「10~15 FPS」 | 同文件 `:402/:410/:419`「**15~30 FPS**」 |
| 2 | 红绿灯每类目标实例数 | `具体实施方案.md:209`「每类 ≥**200** 实例」 | `专项方案/红绿灯识别与通过方案.md:122-124`「≥**400**」、`数据与模型方案.md:97`「≥400」（且专项文档 `:126` 自述「比一般类别（200）要求更高」，说明 200 是「一般类」标准，被误抄进红绿灯专项） |
| 3 | 蜂窝路由 metric | `小车部件测试手册.md:36`「WiFi 600 或蜂窝 **1000**」 | `车端网络方案.md:54`「蜂窝路由 metric **100** < WiFi 600」、`AGENTS.md:300` |
| 4 | 扫线代码存废 | `执行路线图.md:56`「白线循迹代码与单测**全部保留**，…即可启用」 | 同文件 `:72`「扫线部分**已归档删除**」（后者才是事实，见 A1） |
| 5 | 官方 frpc 状态 | `远程连接与服务器手册.md:185`「**保持原样未动**」 | `图传与模式切换方案.md:44`「官方通道：`frpc.service` 与官方推流**已停止并 disable**」 |
| 6 | 白线饱和度实测 | `AGENTS.md:308`「反光最亮像素 S≈23、白线 **S≈40**」 | `执行路线图.md:42`「白线在暖光下 **S≈69~125**（与红底 S≈87~99 重叠）」 |

---

#### C10. 过时的路径与文件引用

| 位置 | 问题 |
|---|---|
| `AGENTS.md:87`、`具体实施方案.md:490`、`数据与模型方案.md:236`、`code/README.md:33` | 仓库根写 `D:\5g\orangepi\`，与当前 checkout 路径不符 |
| `具体实施方案.md:619`、`执行路线图.md:84`、`数据与模型方案.md:124-125` | 引用 `train/split_dataset.py`，该文件不存在（实际是 `scripts/split_dataset.py`，且其 docstring 首行即「按拍摄片段整组切分**检测**数据集（产出 YOLO 的 images/labels 结构）」——`数据与模型方案.md` 对它的功能描述已过时） |
| `code/README.md:3,43-45` | 引用 `doc\2026重构设计文档.md`、`doc\资料学习导读.md`、`doc\车到前预研清单.md` —— **三份都不存在** |
| `AGENTS.md:308` | 提到 `trace_boundary()` 连续跟踪函数——**任何代码里都不存在**（连归档版也没有，归档的 `lane_scan.py` 只有 `line_like/shape_filter/white_mask/fit_line*/edge_image/hough_lane_segments/lane_side_from_segments/LaneScanner`） |
| `doc/README.md:75`、`具体实施方案.md:474` | 引用 `视频讲解与word实验文档/` 目录——仓库内不存在，对应内容在 `doc/官方实验文档/` |
| `官方实验文档/5g图传配置.md:2,139-168` | 声称配套 `frames/frame_001.png … frame_030.png` —— **30 张图全部缺失**；同文件 `:4` 编号写 `car027`（应为 `car0027`） |
| `自动驾驶开发方案.md:127,230,274` | 引用 `config/ground_map.yaml`、`config/parking_map.yaml` —— 不存在，且未标注「尚不存在」 |
| `运行调试与系统调优方案.md:115-116` | 把 `start_autonomous.sh` 描述成「关闭手动服务、启动 vision + control」，实际全文只有一行 `exec …/car-mode.sh auto "$@"`（`AGENTS.md:227` 已声明是「薄包装」） |

---

#### C11. 选型自相矛盾：E18-D80NK 达不到自己要求的 1m

**位置**：`doc/具体实施方案.md:194-195`

```
官方传感器…仅 20cm 内可触发，不满足 1m 要求，必须更换
→ 推荐 E18-D80NK 红外光电开关（3~80cm 可调）
```

E18-D80NK 标称上限 **80cm < 自己要求的 1m**。`专项方案/红绿灯识别与通过方案.md:47-50` 是同一个矛盾（「触发距离要求 **≥1m**」+「E18-D80NK：**3~80cm** 可调 ✅ 首选」）。`执行路线图.md:115,208` 也把该型号当作「≥1m」的解决方案。

---

#### C12. 其它已确认的小错

| 位置 | 问题 |
|---|---|
| `dev/config/README.md` | 内容损坏：`od -c` 显示字面写入了 `` `n`n `` 而非换行（`# 配置文件目录`n`n存放 yaml/json 参数。`），markdown 渲染成一行 |
| `dev/img/README.md:82` | 「小车本身已经有桌面/X11 环境，直接运行即可」——与 `AGENTS.md:220`「**本车没有 X 服务器**（无 Xorg/Xvfb 进程），`cv2.imshow` 会直接崩」直接冲突 |
| `具体实施方案.md:89` | 写 `SIM8202E-M2`（全仓唯一出现），实际模组是 `SIM8262E-M2`（`AGENTS.md:293`、`dev/scripts/net/car-net.sh:162`、`官方实验文档/GPS模块实验.md`）；同处的 `car0003` 也应为 `car0027`；`官方目录索引.md:16` 又写 `SIM8200` |
| `具体实施方案.md:70,643` | 训练 GPU 写 `RTX 5070`，而 `code/README.md:19`（自称「PC 训练环境唯一权威说明」）写 `RTX 4060 Laptop(8GB)`、`数据与模型方案.md:153` 写「RTX 40 系」 |
| `小车部件测试手册.md:77` | 「记录中点脉宽与限幅（写回 `config/hardware.yaml`）」——该文件不存在（舵机限幅在 `settings.py:SERVO_*_ANGLE`） |
| `小车部件测试手册.md:5,24` | 写「本地副本 `scripts/car-test/test_all.sh`」，实际是 `dev\scripts\car-test\test_all.sh`；`:126` 的 `/root/car_Obstacle_Avoid` 不在 `官方目录索引.md` 的官方目录清单里 |
| `运行调试与系统调优方案.md:52` | 「关键状态：`center` / `zebra` / `cone` / `A` / `B` / `L` / `R`」——`debug_view.py:145-159` 只画 `center/conf/L/R/rows` 与 `board/armed/released + area/dom/detail`，**无 `A`/`B` 字段** |
| `具体实施方案.md:658` | 「采用 §3.1.3 的 `blue_barrier + parking_zone` 方案后可放开为 0.5」——§3.1.3 的类别早已是 `blue_board + parking_area`，`blue_barrier/parking_zone` 是 v1.2 历史名（`:558-568` 已自标「请勿使用」） |
| `官方实验文档/自建服务器图传部署文档.md` | 全文描述的是一套**未采用**的 Nginx+RTMP 预研方案（`rtmp://121.40.149.155:1935/live/car0003`、`remote_port: 7500`），与最终实施的 mediamtx（RTSP 8554 / WHEP 8889）完全不同；却被 `doc/README.md:75` 归为「官方（小豚科技）实验指导」 |

---

#### C13. `lane_ref_test.py` 的自身 docstring 与代码不符

**位置**：`dev/scripts/lane_ref_test.py:51-52` 对照 `:79`、`:287-288`

| docstring 写 | 代码实际 |
|---|---|
| `--target-ratio` 目标点…**默认 0.706** = 参考实现的 226/320 | `:79` `TARGET_RATIO0 = 375.0 / 640.0`（= **0.586**） |
| ROI「默认 0.50~0.85」 | CLI 默认 `--roi-top 0.35` / `--roi-bottom-ratio 0.58` |

（`实地调试清单.md:116` 引用的 0.586 才是对的。）

---

### D 类：一致性 / 遗留

| 编号 | 位置 | 问题 |
|---|---|---|
| **D1** | `AGENTS.md:14` vs `执行路线图.md:87` | 行数自述不一致：前者「约 **1300** 行」，后者「约 **1900** 行」；实际 `dev/` 运行时 Python 约 **2700** 行（含 scripts/tests 共 6439 行） |
| **D2** | `dev/hardware/mock.py:7-34` | `AGENTS.md` §5.1 要求「每个硬件模块提供**同接口** mock」，但 `MockPCA9685()` 构造函数不接受 `bus_id/address/freq/esc_*`，且 `set_steering_angle()` 直接把**角度**存进 `channels` 字典（真实实现存的是 **us**），没有 `angle_to_us`/`_us_to_duty`。依赖它做干跑验证会得到与真车不同的量纲 |
| **D3** | `dev/scripts/capture_dataset.py:110` | `while True` 无退出条件（`--count` 默认 0 = 无上限），违反 `AGENTS.md` §1.4.1；而 `dev/img/README.md:25-29` 自己记录了「SSH 断开后进程被 orphan 继续写，几分钟灌了 800 张 37MB」的事故，却仍把「一直拍到 Ctrl+C」当默认 |
| **D4** | `dev/scripts/cam-identify.py:31-39` | 在 `try:` 之前就 stop 了 `opi-control.service` 与两路推流，此后的 import/硬件初始化若失败会**跳过 `finally`**，车留在「既不能遥控也不能看图传」状态（`stream-watch` 只救 `failed`，不会拉起被主动停的 `inactive`） |
| **D5** | `dev/scripts/talk_player_loop.sh:11` | 写死服务器地址与账号口令 `rtsp://car:Ct7vL92xQm4@121.40.149.155:8554/talk_car0027`，而 `car-net.sh apply_server()` 不会更新它 → 换服务器时语音下行静默留在旧地址（正是 `AGENTS.md` §6「换服务器时只改了一处」那条坑） |
| **D6** | `dev/scripts/net/car-net.sh:110` | `wifi_active()` 定义了但**从未被调用**（注释还强调它是「2026-09-20 修」的检查）——死代码 |
| **D7** | `dev/vision/rknn_detector.py:59-62` | `is_native` 属性 docstring 写「**上一帧**是否走了原图直喂」，语义应为「本帧」，措辞误导 |

---

## 四、值得肯定的部分

为保证结论平衡，以下部分是踩过坑之后总结出的**正确做法**，水准明显高于一般学生项目：

1. **发车安全设计**（`dev/vision/start_gate.py`）
   三路判据融合（连通域面积 + 蓝色主导度 + 可选细节骤降），覆盖「程序启动前板已放好」与「启动后才放板」两种时序；核心不变量「**没见过板，绝不放行**」；控制端 FSM 还**独立复核**同一逻辑（`fsm.py:63-75`），不盲信视觉端闩锁。单测覆盖「从未见板绝不发车」。

2. **真实踩坑记录**（`AGENTS.md` §6，约 25 条）
   例如「以为服务 active 就等于推流正常（ffmpeg 推 RTSP 不会自动重连，进程活着但流已死）」「用 `nmcli dev status` 的 `connected` 判断网络可用会把刚连上的 WiFi 主动掐掉（DHCP 后 NM 会停在 `ip-check` 十几秒）」「用每行独立从中心往外找白点的方法找车道线会被赛道塑料膜反光打败」。这些是真正的工程资产。

3. **退出路径安全**（`car-mode.sh`）
   启动前体检、GO 确认、「**任何退出路径（正常结束/Ctrl+C/断线）都恢复手动模式**」，不把车留在中间态。

4. **摄像头独占处理**（`dev/vision/camera_guard.py`）
   纯 `/proc` 扫描设备占用者（不依赖 `fuser`），停服务后 `reset-failed` 防看门狗误重启，并解释了「`systemctl stop` 返回不等于设备已释放」这个实测坑（虽然实现本身有 B6 的缺陷，但**问题意识是对的**）。

5. **PID 相对上届实现的四点改进**（`dev/control/pid.py`）
   带 `dt`、积分限幅、微分低通、`reset()` 清状态。文档还明确了「不要照抄官方实现的坑」（无 dt、无极分抗饱和、无看门狗、泊车用 `exit(0)` 收尾不给电调中立）。

6. **死区陷阱的识别**（`dev/control/planner.py:58-69`）
   注释完整记录了「半油门 1550us 本就在死区边缘，再乘 0.85 → 1542us 掉进死区，车直接停死」，说明团队对电调特性有清醒认识。（虽然 `bench_test.py` 的恒等域版本没同步修，见 A7。）

7. **文档体系**（`doc/README.md`）
   一文档一职责 + 文档地图 + 维护约定（「设计变了改 A、计划变了改 B，别再往一处堆」）+ 契约需同步修改的显式约定 + 变更记录。

8. **CRLF 问题的根治**（`.gitattributes`）
   `* text=auto eol=lf` 配合 `ssh_sync.py` 的 `content_equal()` 行尾无关比较，解决了「Windows 工作区 core.autocrlf 导致大小比较误判每个文件都变了，一次性全量上传覆盖掉车端更新的实现」这个真实事故。

---

## 五、修复优先级与建议

### 5.1 建议的修复顺序

| 顺序 | 编号 | 理由 | 改动量 |
|---|---|---|---|
| 1 | **A2** | **风险最高且不可逆**（会删车端模型和标定值）。在修复前，**先停用 `ssh_sync.py --delete`** | 小 |
| 2 | **A1** | 不修则自主链路零可用，其余车端问题都无从验证 | 小 |
| 3 | **A4 / A5 / A6** | 车端三个日常入口全断（状态查看、数据采集、台架测试） | 各 1~3 行 |
| 4 | **A7** | 「降速」变「停车」会直接触发「停止超 20s 判失败」 | 小（含单测期望值） |
| 5 | **A3** | 诊断工具是排查扫线问题的主要手段，且留有递归地雷 | 小 |
| 6 | **B1 / B2 / B4 / B6 / B9** | 契约破缺与安全相关，改动局部 | 小 |
| 7 | **B7 / B11 / B12** | 死参数与死配置会误导现场调参，需与 A1 的修复方向一起决定 | 中 |
| 8 | **C 类** | 建议与现场调试同步批量修正，避免现场照错文档操作 | 中 |

### 5.2 三个建议的批量修复方案

**（1）A1 与 B11/B12 是同一个根因的三处表现。**
扫线模块被归档后，运行时接线（`vision_main.py`）、配置（`settings.py` 的 `LANE_*`）、测试脚本（`lane_ref_test.py` 的转向符号）三处都没跟上。建议一并处理：

- 把 `lane_ref_test.py` 里自包含的扫线实现提取为 `dev/vision/lane_hough.py`（或新建 `lane_scan.py`）
- `vision_main.py:28-29` 改为该模块的真实导入，并统一走 `settings.STEER_SIGN`
- 清理或标注 `settings.py` 里零引用的 `LANE_*`，给仍在用的加「当前生效」标记
- 让 `lane_ref_test.py` 调用同一个实现（消除双实现符号不一致）

**（2）A2 的删除策略需要你确认。**
我倾向于：`stale` 判定同时套用 `is_excluded()`，并把 `--delete` 的预览输出分成「将删除」与「跳过（受保护）」两类，让操作者能看见「模型/标定值被保护」而不是静默跳过。另一种更保守的做法是 `--delete` 只处理「本地曾经同步过、现在删掉了」的文件（需要一份同步清单），不做全量差集。

**（3）电调死区应抽成一个共用钳制函数。**
A7（`bench_test.py` 恒等域）与 `planner.py`（百分比域）是同一个物理约束的两处实现，一处规避了、一处没有。建议在 `bench_common.py` 或 `settings.py` 提供一个统一的 `clamp_above_deadband(us)`，两处共用，并在单测里断言「任何 >0 的动力输出都必须 ≥ `ESC_DEADBAND_US`」。

### 5.3 需要人工确认的事项（本报告无法在仓库内闭环）

| # | 事项 | 说明 |
|---|---|---|
| 1 | `oldCode/src/vision/vision.cpp` 与 `control.cpp` | `实地调试清单.md:98`、`lane_ref_test.py:9,16,69,98`、`settings.py:107` 都引用它，但仓库内不存在（推测只在 A 机 DSH 本地）。B12 的转向符号需要以它为基准判定谁对 |
| 2 | `config/ground_map.yaml` / `config/parking_map.yaml` | `自动驾驶开发方案.md` 引用，文义像「待创建」，需确认 |
| 3 | `/root/car_Obstacle_Avoid` | `小车部件测试手册.md:126` 引用，不在 `官方目录索引.md` 清单里 |
| 4 | `train/` 目录与 2858 张原始图片 | `数据与模型方案.md:16` 称在 `train/project/`，但仓库根无 `train/`（`.gitignore` 也排除了数据集），需到 A 机核对 |
| 5 | 图传「保留另一路推流给裁判」 | `car-mode.sh:391` 声称保留另一路，但 `vision_main.py:166` 的 `camera_exclusive()` 默认停**两路**且贯穿整个主循环 → 自驾期间图传对裁判实际是黑的。需确认这是接受的行为还是待修的一致性问题 |

---

## 附录 A：核查方法

本报告的每条结论均通过以下方式**实际验证**，未依赖推测：

1. **文件存在性**：对每个被引用的路径执行 `ls` / `find` / `tar -tzf`（如确认 `lane_scan.py` 不在仓库、在 attic 包内）
2. **符号引用**：`grep -rn` 统计每个 `settings` 常量的引用次数（用于确认 B11 的死配置）
3. **调用链**：追踪 `enter_traffic_light()`、`wait_device_free()`、`edge_density()`、`wifi_active()` 的全部调用点（用于确认 B2/B6/B11/D6）
4. **规则原文比对**：把 `规则概要.md` 的罚时表逐条对照 `rules2026_text.txt` 的原文（用于确认 C1）
5. **字节级检查**：`od -c` 确认 `dev/config/README.md` 的损坏方式；检查全部 `.sh/.service/.timer/.conf` 均为 LF（`.gitattributes` 生效，**未发现** CRLF 问题）
6. **分支顺序分析**：`bench_test.py` 的 `return 2` 守卫与 `--no-motor`/`--calibrate`/`--steer-test` 分支的先后位置（用于确认 A6）
7. **AST 作用域扫描**：对 `dev/` 与 `scripts/` 全部 `.py` 做未定义变量扫描，**零命中** —— 即不存在第二个 `out_us` 类问题
8. **单测实跑**：`dev/tests` 13 个文件本地全部通过，仅 `test_site_config.py` 因本机缺 PyYAML 而红（`config/site.py` 会优雅降级并打印警告，属环境依赖，非逻辑 bug）
9. **已排除的疑点**：`lane_ref_test.py:421` 的 `driving`/`frame` 经控制流追踪确认不是 NameError（`seen_board` 置位时循环体必然已完整执行）；`car-net.service` 的 `Type=oneshot + Restart=on-failure` 在 systemd ≥ 244 是允许的，不算 bug

## 附录 B：核查边界

以下内容**未**纳入本次审查，或**无法**在仓库内验证：

- **未运行真车/硬件**：所有涉及 PCA9685、电调、摄像头、5G 模组的结论均来自静态代码分析，未上台架实测
- **未访问车端**：车上 `/root/dev` 的实际内容、systemd 运行状态、日志均未查看；本报告只对**仓库内**的文件负责
- **未核对规则 PDF 原文与插图**：`比赛规则/` 的 PDF 与 PNG 未做逐页人工比对，C1 用的是仓库内的文本提取件（`rules2026_text.txt`）
- **未评估的项**：`dev/tools/convert_rknn.py` 的转换参数正确性、`scripts/train.py` 与 `code/verify_yolo.py` 的训练/验证逻辑、RKNN INT8 量化精度影响 —— 这些需要真实模型与数据才能验证
- **文档中「待确认」的事项**（如 4 类模型的 id0/id2 类名、斑马线 30cm 停区在哪一侧、是否有轮速编码器）属项目自身的开放问题，不是错误

---

*报告结束。*
