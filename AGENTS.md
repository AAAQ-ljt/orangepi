# AGENTS.md — 智能车项目 AI 作业手册

> 本文件是**所有 AI 助手（ZCode / Claude / 其他 agent）在本仓库工作时的第一读物**，等价于团队开发规范。
> 人类队员也请先读这一份。原始的长版规范已合并进本文件（`doc/开发规范.md` 已删除）。
> 最后更新：2026-09-15

---

## 0. 三十秒背景

**项目**：2026 全国大学生智能汽车竞赛 · 室外 5G 远程驾驶无人车赛。
**硬件**：Orange Pi 5（RK3588S）+ XT-NetRC 阿克曼底盘 + PCA9685 + 双摄（**两路都是 USB**：`/dev/video0`=icspring **云台主摄** / `/dev/video2`=Global Shutter **下摄副摄，巡线用**）+ SIM8200 5G。
**目标**：**完赛，并尽量快**。规则里"停车超 20s 不动 = 比赛失败"，"停错车位/无轮入区 +100s"，"撞锥 +20s"，"压线 +10s/次"——所以**可靠性优先于速度**。
**当前阶段**：车端骨架已通（约 1300 行），**唯一硬阻塞是训练数据没有标注**；巡线、减速停准、停车入库、红绿灯触发都还没实现。

**先看两篇文档再动手**：
- `doc/执行路线图.md` —— 现在该做什么（P0/P1/P2 批次、基线盘点、待确认清单、风险）
- `doc/具体实施方案.md` —— 技术方案与决策（含类别表契约 §3.1.3）

---

## 1. 硬性规则（违反即视为严重错误）

### 1.1 安全（人身与硬件）

1. **任何可能驱动电机的操作，必须先确认四轮悬空或场地空旷**。真车跑车必须有人在旁、急停可达。
2. 真车运行必须显式 `--real --arm`；**默认参数永远是 dry-run**（`main.py` 不带 `--real` 不碰硬件）。
3. 调试期电调限速 `--max-us 1600`（`ESC_DEBUG_MAX_US`）；**未经验证不得改成 `ESC_MAX_US=2000`**。
   ⚠️ 电调有**死区**：1500us 停、**≈1545us 才起转**（`ESC_DEADBAND_US`，2026-09-16 架空实测）。
   任何"要让车动"的脉宽必须 > 1545（起步/循迹 **1560**；2026-09-20 用户实测 1575 太快，已统一降速）；
   命令行给了低于死区的值，台架脚本会**直接拒绝运行**——曾经因为默认 1530/1550 白跑一整次台架。
4. **所有退出路径都必须安全停车**：电调归零、舵机回中、释放 PCA9685 与摄像头、恢复 ffmpeg 推流。
   统一收敛到一个 `cleanup()`，捕获 `SIGINT/SIGTERM/SIGABRT/SIGQUIT`。**绝不允许某条退出路径只 `return` 不释放**。
5. 禁止在信号回调里做阻塞操作（sleep、网络请求）。
6. 软件里**不得写死**能让车"上电即冲出去"的逻辑。发车必须是**边沿触发**（见 `dev/vision/start_gate.py`）：
   "先确认有遮挡 → 再等遮挡消失"的跳变才算发车信号，**绝不能用"当前无遮挡"直接发车**。

### 1.2 车端环境（官方资产不可破坏）

1. **不碰官方目录与官方服务**：`/root/opi-control`、`/root/car_Keyboard_Control`、`/root/ParkingVision`、`/root/frp`、nginx、`ffmpeg-stream(.service/-sub)`、`talk-player`、`car-dial`。
   我们的代码只放 `/root/dev`（本地对应 `dev/`）。改动前**先备份**。
2. **摄像头是独占设备**：任何程序打开摄像头前**必须**先 `systemctl stop ffmpeg-stream.service ffmpeg-stream-sub.service`，退出时**必须**恢复。
   Python 里用 `dev/vision/camera_guard.py` 的上下文管理器，不要手写。
3. **systemd / 驱动级改动必须经队长确认**，不得擅自 `systemctl disable/enable` 官方服务。
4. 车上不要执行格式化、`rm -rf`、批量清理等不可逆操作；改配置前记录原值。

### 1.3 仓库与提交

1. 远端唯一：`AAAQ-ljt/orangepi`；分支 `master`。**每天收工前 push**。
2. `.gitignore` 是**白名单**：只跟踪 `code/`、`dev/`、`doc/`、`scripts/`（+ 本文件与 `.gitattributes`）。其余一律不入库。
3. **禁止提交**：模型权重（`*.pt/*.onnx/*.rknn`）、数据集、`runs/`、视频（`*.mp4/*.avi`）、日志、`*.pid`。
4. **禁止提交**任何密钥/令牌/密码到新文件（现有文档里的 frp token 属历史遗留，不要扩散）。
5. 提交信息写"做了什么、为什么"，不要 `1`、`update` 这类无意义信息。

### 1.4 AI 编码红线

1. 不写**没有退出条件**的循环；不创建无法退出的后台线程。
2. 不在主循环里做阻塞 I/O（网络请求、大文件写入、同步播放音频、每帧 `cv2.imwrite`）。
3. 不忽略异常：至少打印日志并让上层能感知。
4. 不在没有 mock / 测试的情况下把高风险运动代码直接上真车。
5. 不为了"跑通"而删掉安全判断（超时兜底、failsafe、限幅、去抖）。

---

## 2. 当前该做什么（按优先级）

> 权威版本在 `doc/执行路线图.md`；这里只放摘要，**动手前先看那篇的对应条目和出口标准**。

| 批次 | 任务 | 状态 |
|---|---|---|
| **P0-1** | **标注 + 补齐采集矩阵**（`traffic_light_off`、远距离、负样本、红锥桶、蓝板压中线）→ 切分 → `data.yaml` | ⬜ 未开始（唯一硬阻塞，越早越好） |
| **P0-2** | **修发车逻辑**：边沿触发 + 去抖 + 超时兜底（`start_gate.py` + FSM + 协议） | 🟡 本轮落地 |
| **P0-3** | 采购光电传感器（≥1m、阈值可调） | ⬜ 需人工 |
| **P0-4** | 现场几何实测（赛道宽、停车区黄线、斑马线 30cm 停区、是否有编码器、红灯长等 vs 20s 判失败） | ⬜ 需到场 |
| **P1-1** | 扫线 + 误差滤波 + PID（`lane_scan.py` / `filters.py` / `pid.py`） | 🟡 本轮落地 |
| **P1-2** | 距离估计与减速停准（框底边 → 地面坐标 → 速度曲线） | ⬜ |
| **P1-3** | FSM 主体重构（斑马线 10s、红绿灯光电、锥桶绕行、停车五阶段） | ⬜ |
| **P1-4** | 停车：蓝板接地点 + IPM 判左右 → 五阶段入库 | ⬜ |
| **P1-5** | 日志降频 + 语音播放非阻塞 | 🟡 本轮落地 |
| **P2-*** | 阈值现场标定、双摄分时复用、语义分割（条件触发）、性能优化、技术手册 | ⬜ |

---

## 3. 仓库结构

```text
D:\5g\orangepi\
├── AGENTS.md            ← 本文件（agent 作业手册）
├── dev/                 ← 比赛代码（本地 ⇄ 车上 /root/dev）
│   ├── main.py          ← 控制端入口（--real --arm 才动硬件）
│   ├── config/          ← settings.py（阈值/限幅集中在此，禁止散落硬编码）
│   ├── common/          ← protocol.py（UDP 消息契约）、通用工具
│   ├── vision/          ← 视觉进程：lane_scan / start_gate / rknn_detector / postprocess / camera_guard
│   ├── control/         ← 控制进程：fsm / planner / pid / filters / lane_arbiter / driver / udp_server
│   ├── hardware/        ← pca9685 / imu / audio / mock（mock 必须与真实实现同接口）
│   ├── scripts/         ← 车端调试/诊断/运维脚本（sh + py）
│   ├── img/             ← 摄像头拍照/录像包装脚本
│   ├── tests/           ← 本地单测（不碰硬件，用 mock）
│   └── tools/           ← 模型转换等离线工具
├── doc/                 ← 全部文档，**入口是 doc/README.md（文档地图）**
├── code/                ← PC 训练环境（README.md 是环境唯一说明；权重/数据集不入库）
├── scripts/             ← PC 侧 ssh 助手（ssh_car.py / ssh_put.py / ssh_get.py / ssh_server.py）
├── train/               ← 原始图片与切分脚本（**不入库**）
└── python/              ← 早期 PC 联调脚本（历史，一般不用改）
```

### 3.1 车端 `/root` 目录规范（保持整洁）

原则：**我们的东西只在 `/root/dev`；官方目录只读；运行期产物集中管理；不留一次性脚本。**

```text
/root/
├── <官方目录>            # car_Tracking / car_Navigation / opi-control / ParkingVision /
│                        # hardware_Test / SIM8200_for_RPI / frp … 保持原样（不改不删）
├── talk_player_loop.sh  # 官方语音下行循环（内容已切到自建 mediamtx，改动在 git 里）
├── car_net_up.sh        # 团队早期 5G 拨号脚本（已被 car-net.sh 取代，保留作历史参考）
└── dev/                 # ★ 我们的比赛代码（= 仓库 dev/，用 scripts/ssh_sync.py 同步）
    ├── main.py  vision/  control/  hardware/  common/  config/  tests/   # 运行时模块
    ├── scripts/
    │   ├── net/         # 网络子系统：car-net.sh + car-net.conf + wifi-ledger.conf
    │   │                #   + systemd 单元 + stream-watch 看门狗 + 10-wwan0.link
    │   ├── car-mode.sh  # 工作模式切换（手动/自驾/急停/图传）
    │   ├── car-test/    # 硬件自检 test_all.sh
    │   ├── car-frp/  server-frp/   # frp 两端配置留存（可复现）
    │   └── capture_dataset.py / record_video.py / cam-identify.py / safe_pwm_init.sh …
    ├── img/             # 拍照/录像脚本 + 采集数据（**数据不入 git**）
    ├── logs/            # 运行日志（不入 git；可随时清空）
    ├── models/          # 模型（不入 git）
    ├── backup-*/        # 官方配置备份（只读保留，回滚用）
    └── attic/           # 归档：淘汰的脚本/数据打成 tar.gz 放着，需要时再翻
```

规矩：

1. 新脚本只放三处：`dev/scripts/net|ops|diag`（车端运维/诊断）、`dev/vision|control|hardware`（运行时）、`dev/img`（采集）；
2. **不在 `/root` 下新建文件/目录**（一次性的诊断脚本也一样，进 `dev/scripts/` 或 `attic/`）；
3. 采集数据、日志、模型不进 git，也不散落在别处；
4. **删文件前先验证引用**：
   `grep -rl "<文件名>" /etc/systemd/system /root/dev /root/*.sh 2>/dev/null`，
   确认无引用后**先归档到 `dev/attic/`** 再删；
5. 官方目录/services（含 `rc.local` 里的 `simcom-cm`）默认不动，要动必须先备份并把改动写进文档。

---

## 4. 开发流程

### 4.1 本地（Windows，无车）

本机默认 python **没有** numpy/cv2/pytest。用带依赖的解释器：

```bash
PY="E:/venvs/smartcar-ultra/Scripts/python.exe"   # numpy 2.2 + cv2 5.0
cd dev && export PYTHONPATH=.

# 单测（纯函数 + __main__ 风格，不需要 pytest）
for t in test_filters test_pid test_lane_arbiter test_protocol test_start_gate test_lane_scan test_control; do
  "$PY" tests/$t.py
done
"$PY" tests/test_integration.py        # 端到端：起控制端 + UDP 打假数据（dry-run）

# 视觉单模块自测（不需要摄像头/模型）
"$PY" vision/vision_main.py --test-image 某张赛道图.jpg   # 打印 lane / start gate 判据
"$PY" main.py --port 5000                                 # dry-run 控制端
```

- 所有新模块**必须能在没有摄像头、没有 PCA9685、没有模型的机器上 import 并测试**（用 mock 与合成图）。
- 测试文件命名 `test_*.py`，用断言 + `if __name__ == "__main__"` 直接跑，别引入新的测试框架依赖。
- 提交前**必须**跑通上面全部 8 个文件。

### 4.2 连车（免交互）

```bash
python scripts/ssh_car.py "uname -a"                   # 走 frp 隧道（推荐）
python scripts/ssh_car.py --direct hostname -I          # 热点直连兜底
MSYS_NO_PATHCONV=1 python scripts/ssh_sync.py --dry-run  # 预览 dev/ → /root/dev 的差异（增量同步）
MSYS_NO_PATHCONV=1 python scripts/ssh_sync.py            # 真正同步（只传变化的文件）
python scripts/ssh_put.py dev/xxx.py /root/dev/xxx.py   # 单文件上传
python scripts/ssh_get.py /root/dev/log.txt .            # 下载

# 网络全断时（或本地网络封了 SSH 端口）—— 走串口，不经网络
python scripts/serial_car.py "ip -4 a show wwan0"        # 串口执行命令（COM8@1500000，自动登录）
python scripts/serial_push.py dev/scripts/net/car-net.sh /root/dev/scripts/net/car-net.sh --mode 755
```

⚠️ 传**绝对远端路径**时加 `MSYS_NO_PATHCONV=1` 最稳（Git Bash 的路径转换坑，见 §6）。
`ssh_put.py` / `ssh_get.py` / `serial_push.py` / `ssh_sync.py` 都会自动纠正被转换的路径，
所以忘加也不会真出错（纠正是共享模块 `scripts/remote_path.py`）。
详细参数、排障见 `doc/远程连接与服务器手册.md` 与 `doc/车端网络方案.md`。

### 4.3 车上运行

```bash
# ★ 模式切换统一入口（上电默认是"手动遥控"）
bash /root/dev/scripts/car-mode.sh status          # 看当前模式/服务/进程/温度/摄像头占用
bash /root/dev/scripts/car-mode.sh auto            # 进入自动驾驶【默认 DRY-RUN，电机不动】
bash /root/dev/scripts/car-mode.sh auto --real     # 真正跑车（要输入 GO 确认；务必轮子架空或场地空旷）
bash /root/dev/scripts/car-mode.sh manual          # 回到手动遥控（任何退出路径也会自动恢复）
bash /root/dev/scripts/car-mode.sh estop           # 紧急停止：立刻归零
bash /root/dev/scripts/car-mode.sh stream up       # 图传推流（自建服务器）拉起/检查
bash /root/dev/scripts/car-mode.sh logs vision     # 看视觉日志

# 其它
bash /root/dev/scripts/safe_pwm_init.sh            # 只确保 PCA9685 回到安全值
bash /root/dev/scripts/run_capture.sh --folder lane  # 拍照（自动停/恢复推流）

# 网络（蜂窝优先 / WiFi 账本兜底 / 服务器可切换）—— 详见 doc/车端网络方案.md
bash /root/dev/scripts/net/car-net.sh status        # 出口/蜂窝/WiFi/隧道/图传一眼看清
bash /root/dev/scripts/net/car-net.sh auto          # 上电策略（开机自动执行，也可手动跑）
bash /root/dev/scripts/net/car-net.sh cellular up   # 拨号（APN 轮询 → 厂商兜底）
bash /root/dev/scripts/net/car-net.sh cellular down # 优雅断开（释放 QMI 会话与 CID）
bash /root/dev/scripts/net/car-net.sh apply         # 把 car-net.conf 的服务器配置应用到系统

# 台架测试（四轮必须架空，或场地空旷能随时断电）—— 安全层共用 scripts/bench_common.py
sudo python3 /root/dev/scripts/bench_test.py --no-motor                  # 只看读数（不动电机）
sudo python3 /root/dev/scripts/bench_test.py --allow-motion --max-seconds 30   # 正式跑：板在→停/板开→低速对齐→循迹/再见板→停
sudo python3 /root/dev/scripts/bench_test.py --calibrate 30              # 只想单独标定车道中心时用（平时不用，正式跑会自动标定）
sudo python3 /root/dev/scripts/bench_test.py --steer-test --allow-motion # 转向方向自检（电调中位，只看前轮）
bash /root/dev/scripts/run_tests.sh                                      # 车上跑全部单测
```

- 需要看画面：**本车没有 X 服务器**（无 Xorg/Xvfb 进程），`cv2.imshow` 之类的窗口程序会直接崩
  （`x11.sh` 这个名字在车上并不存在，旧文档里的说法已作废）。
  看画面的正规途径有三条：
  1. **浏览器看图传**（自建 mediamtx：主摄/副摄两路，见 `doc/图传与模式切换方案.md`）——调相机角度、看车前方都用它；
  2. **`dev/scripts/diag/lane_probe.py`**：抓帧 + 掩膜/跟踪叠加图 + 终端数字结论，专治"扫线到底看到了什么"；
  3. 若你的 SSH 客户端自带 X 服务器（MobaXterm / Xming）且 `DISPLAY` 能连通，才可以用 `--show` / `debug_view.py`。
- 日志：`/root/dev/logs/{vision,control}.log`、`/tmp/smartcar_status.json`。
- `start_autonomous.sh` / `stop_autonomous.sh` 现在是 `car-mode.sh` 的薄包装，保留只为兼容旧文档。
- **绝不要**在不清楚车是否在动、是否有人在遥控的情况下跑 `auto --real`。

### 4.4 提交前自查

- [ ] 本地测试全绿
- [ ] 静态检查干净：`cd dev && "E:/venvs/smartcar-ultra/Scripts/python.exe" -m pyflakes $(git ls-files '*.py' | sed 's|^dev/||')`
      （**必须**：2026-09-19 就是因为没跑它，把一个未定义变量 `out_us` 送上台架，跑到第 5 帧才炸）
- [ ] 没有新增硬编码阈值/I​P/密码（阈值进 `config/settings.py`）
- [ ] 所有资源获取都有对应的释放路径
- [ ] 退出路径都安全停车
- [ ] 没有提交权重/数据/日志
- [ ] 改动的文档已同步（尤其类别表 = 契约）

---

## 5. 代码约定

### 5.1 分层与边界

```text
vision 进程：摄像头 → 扫线(每帧) + 元素检测(离散事件) → UDP JSON
control 进程：UDP → FSM(任务状态) → Planner(观测量→控制量) → Driver(PCA9685)
```

- **硬件操作只允许出现在 `hardware/`**，业务代码不得直接碰 GPIO/PWM/I2C。
- **观测量契约在 `common/protocol.py`**；新增字段要同时改 `from_dict`/`to_dict`，并保持对旧字段的兼容（`from_dict` 必须容忍缺失/`null`）。
- **阈值、限幅、坐标目标全部进 `config/settings.py`**，标注单位与用途，现场可改。
- 每个硬件模块提供同接口 mock（`hardware/mock.py`）。

### 5.2 坐标系与单位（容易搞错，统一在此）

| 量 | 约定 |
|---|---|
| `center_x` / `left_x` / `right_x` | **像素**，基于 640 宽画面（`IMG_W=640`），y 向下 |
| `TARGET_X` | 期望车道中心像素值（摄像头偏装，**不是 320**）；台架/主程序在起步瞬间自动标定并写 `config/site.yaml` |
| 转向 | 舵机**角度** 0~180，90 为中位；`SERVO_MIN/MAX_ANGLE` 限幅 |
| 油门 | `-100~100` 百分比；`ESC_STOP_US=1500`、**`ESC_DEADBAND_US=1545`（低于它车不动）**、`ESC_CREEP_US=1560`、`ESC_DEBUG_MAX_US=1600` |
| 误差单位（PID 输入） | `error_units = clamp(center_x - TARGET_X, ±160) / 4`（沿官方惯例，便于复用其 PID 起点参数） |
| PID 输出 | 转向**角度增量**，限幅 ±`LANE_STEER_LIMIT_DEG` |

### 5.3 视觉模块接口

```python
LaneObservation(center_x: float, confidence: float, left_x, right_x, valid_rows, source)
StartGateState(blocked: bool, armed: bool, released: bool, blue_ratio: float, timed_out: bool)
```

- 扫线**每帧都跑**（轻量，给控制环 30~50Hz 的连续性）；元素检测只出**离散事件**（15~30FPS 足够）。
- 置信度低不是"丢帧就算"，必须**连续 N 帧**低置信度才降级（防抖）。
- 竞速真正怕的不是帧率低，而是**误检**（误刹车/误停车）。**先保误检率，再提帧率。**

### 5.4 控制约定

- PID **必须带 `dt`**、输出限幅、可选积分限幅；`reset()` 要清 `integral`/`prev_error`（官方参考实现漏了这条，换来的是切状态时的"踢腿"）。
- 误差进入 PID 前先过 `ErrorFilter`（中位数离群剔除 + 越新权重越大）。
- 转向输出**必须限速**（slew rate），不允许从 0 直接跳到满舵。
- 控制环不得因为视觉丢帧而抖动：仲裁层负责"降级 → 保持 → 停车"。

---

## 6. 已知陷阱（踩过的坑，别再犯）

| 坑 | 后果 / 正确做法 |
|---|---|
| **以为安全组被"覆盖"了** | 端口从某些网络连不上，先分清是**安全组**还是**你本地网络**：换个观测点（小车本身）测同一端口。2026-09-16 实测：SG 一直正常，是校园网出口封了 22/2222 这类 SSH 端口 |
| 用团队自己写的 qmicli 脚本给 SIM8262E-M2 拨号 | 该模组时序特殊，手写 qmicli 容易 `CID allocation failed`；**先试 APN 轮询（`car-net.sh cellular up`），失败再用厂商 `simcom-cm`，最后 `cellular recover` 做 USB 硬复位** |
| **以为模组坏了/拨号脚本有问题，其实是网卡被改名** | systemd 会把 `wwan0` 改成 `wwx<MAC>`，所有写死 `wwan0` 的脚本（含厂商二进制）全部失效。已修：脚本运行时自动探测 + `/etc/systemd/network/10-wwan0.link` 固定名字。排查：`dmesg \| grep -E 'qmi\|wwan' \| tail` |
| 以为"服务 active"就等于"推流正常" | ffmpeg 推 RTSP **不会自动重连**：网络一断进程还活着但流已死。已加 `stream-watch.timer`（30s 检查到媒体服务器的连接，断了就重启服务） |
| 让看门狗"服务不在就重启" | **会被它抢走摄像头**：自动驾驶要停掉占用下摄的那路推流，看门狗若把它拉起来，视觉进程的摄像头就没了。已改为**只救 `failed`（崩溃）的服务**，`inactive`（被人主动停）一律不碰 |
| 用 `nmcli dev status` 的 `connected` 判断网络可用 | DHCP 拿到地址后 NM 会停在 **`ip-check`** 十几秒，此时网络其实已经可用；用"接口有没有 IPv4 地址"判断，否则会把刚连上的 WiFi 主动掐掉 |
| 以为手机热点在旁边就一定能连上 | 关联成功 ≠ 拿到 IP：热点 DHCP 不给租约时会报 `ip-config-unavailable`；`car-net.sh wifi up` 已重试 3 次，仍失败可在账本加静态地址 |
| 强杀/断电对待蜂窝连接 | 会留下**未释放的 CID** → 下次拨号必然失败；退出前用 `car-net.sh cellular down`（关机已自动挂上） |
| 同时让蜂窝和 WiFi 都自动连 | 两条默认路由互抢，表现为"时通时断"；用 `car-net.sh policy` 保证**只占一条上行**（蜂窝 100 < WiFi 600） |
| 换服务器时只改了一处 | 推流地址与 frp 隧道地址是两套配置，容易半切换；统一改 `car-net.conf` 后 `car-net.sh apply` |
| **Git Bash 会把 `/root/xxx` 这类参数转成 Windows 路径** | 远端收到的是 `C:/Program Files/Git/root/...`，报 `No such file`。传绝对路径时加 **`MSYS_NO_PATHCONV=1`**；`ssh_put/ssh_get/serial_push/ssh_sync` 已内置自动纠正（`scripts/remote_path.py`） |
| 摄像头 index 搞反（会把云台画面当赛道） | **实测确认（2026-09-16）**：`/dev/video0`(index 0) = icspring = **主摄＝云台摄像头**（推流 `cam_car0027`，红绿灯环节看灯）；`/dev/video2`(index 2) = Global Shutter = **副摄＝下摄**（推流 `cam_car0027_sub`，**巡线扫线用它**）。自动驾驶默认 `--camera 2`；不确定时跑 `dev/scripts/cam-identify.py` 复验 |
| 以为摄像头能跑 30fps | 实测 640×480 只有 **~15fps**（驱动谎报 30）。视觉进程 14 FPS 是摄像头限制，不是我们的代码慢 |
| **下摄"看着赛道"就以为白线在画面里** | 2026-09-19 抓图实锤：下摄俯仰角太**平**（几乎水平看出去）时，画面里只有赛道中段 + 塑料膜褶皱反光，两条白线贴在**画面左右外沿/画面外** → 扫线无论如何都锁不到线（实测：左 18 行、右 8 行碎片，conf 0）。**先看画面再调参数**：`sudo python3 scripts/diag/lane_probe.py --camera 2 --frames 6`（终端直接给结论 + 叠加图），把两条白线调进画面下半部再谈阈值 |
| 以为「云台压下去」就能拿到俯视视角（**巡线固定用下摄 video2，云台不参与巡线**） | 2026-09-19 实测**云台俯仰行程不够**：`--sweep-tilt` 扫了 40°~140° 共 9 档，最好的一档（tilt=105°）conf 也只有 0.24（可用要 ≥0.4），画面依旧是"顺着赛道看"（上方还能看到实验室）。**结论：现有两路摄像头都拿不到"俯视车道"的视角，白线循迹在本车上做不成**（代码与单测保留，等镜头能朝下/换广角再启用） |
| 把"扫线循迹"当成完赛的必要条件 | **不是**。规则只罚"压线 +10s/次"，而赛道是直道：直线用**固定舵角 + 中位微调**（`site.yaml` 里改 `servo_center_angle`）就能不压线；变道/绕锥/入库可以按官方上届的做法**开环（时间/距离触发）**，停车与斑马线用**元素检测**触发。别让一个拿不到视角的功能卡死整个进度 |
| 用"每行独立从中心往外找第一个白点"找车道线 | 打印跑道塑料膜的反光是**低饱和亮区**，颜色上与白线无法区分（实测反光最亮像素 S≈23、白线 S≈40），且正好在画面中间 → 每行都会先撞上反光。已改为 **`trace_boundary()` 连续跟踪**（多种子 + 双向行走；线：连续/平滑/宽度稳定，反光：一团一团/宽度突变），并保留形状过滤（尺度无关的细长比）与**配对宽度闸门**兜底 |
| 用"当前无挡板"直接发车 | **上电即冲**。必须边沿触发（`start_gate.py`） |
| `is_barrier` 之类字段硬编码成常量 | 会让 FSM 走进错误分支；协议字段必须来自真实感知 |
| 每帧 `print` 完整状态 | 15~25FPS 下日志本身就吃掉可观 CPU；改为每 N 帧或写状态文件 |
| 同步 `subprocess.run` 播放音频 | 阻塞整个控制循环，期间 failsafe 失效；改后台线程 |
| 用 YOLO 的"车道线检测框中心"当车道中心 | 只在上届 seg 模型下成立；本届用**扫线**为主力（`lane_scan.py`） |
| 把"左右"做进类别标签 | `fliplr` 增广会把标签翻错；左右是**空间关系**，用几何判定 |
| Roboflow 里做 train/val/test 划分 | 它随机划分会把连续帧拆到两边 → 验证集虚高；**必须按拍摄片段整组切分** |
| 用上届模型权重 | `dev/models/best11nseg.rknn` 类别与本届不符，只能验证工具链 |
| 停车判定看"黄框是否可见" | 裁判放法不可预测；用**外框 + 蓝板接地点 + IPM** 的几何先验 |
| 用 `cx < 图像宽/2` 判左右车位 | 车斜着进场时必错；要比 IPM 俯视图里 `board_x` 与外框中点 |
| 停车/斑马线"看到就断油" | 会滑过停车点（压线 +10s、未停够 10s 再 +20s）；需要**距离估计 + 减速曲线** |
| 两个模型同时无脑调 RKNN | NPU 争抢导致总吞吐下降；先"单推理线程 + 任务队列"，必要时再 core mask 隔离 |
| 主循环里 `cv::imwrite` / X11 `imshow` | 卡住控制回路 / headless 下直接崩；调试预览用开关控制 |
| 依赖 `main.py --dry-run` 参数 | 该参数不存在，dry-run 是**默认行为** |

---

## 7. 参考实现（官方上届 C++ 代码，只读参考）

`2025年比赛资料/智能车代码包+软件操作手册/` 下两套（opencv 版、yolo 版）。值得移植的：

| 移植项 | 出处 | 价值 |
|---|---|---|
| 逐行扫线 + 纵向加权误差 | `opencv版本…/code/image.cpp:1224-1331` + `image.h:40-56` | 比"检测框中心"鲁棒，天然兼容单侧线丢失 |
| 误差滤波器 | `yolo版本…/HardWare/ErrorFilter.h` | 中位数离群 + 新值加权，抗 YOLO 跳变 |
| PD 参数起点 | `yolo版本…/Controller/State/src/TrackingState.cpp:12-22` | 巡线 kp 0.2~0.25 / kd 0.5~0.8（纯 PD） |
| 状态机防抖阈值 | 同上各 State | 挡板 3 帧、斑马线 3 帧、停车牌 3 帧投票 |

**不要照抄它们的坑**：`laneChangeCount` 初值写死导致半数状态不可达、泊车用 `exit(0)` 收尾不给电调中立、PID 无 dt / 无积分抗饱和、无看门狗、变道泊车全靠固定时间开环。

---

## 8. 文档地图（详见 `doc/README.md`）

| 我要… | 看这篇 |
|---|---|
| **去操场调试** | `doc/实地调试清单.md` |
| 知道现在该干什么 | `doc/执行路线图.md` |
| 理解方案与决策 | `doc/具体实施方案.md` |
| 采数据 / 标注 / 训练 / 转 RKNN | `doc/数据与模型方案.md` |
| 连车 / frp / 服务器 | `doc/远程连接与服务器手册.md` |
| 调试入口 / 系统调优 | `doc/运行调试与系统调优方案.md` |
| 红绿灯、停车专项设计 | `doc/专项方案/` |
| 上车自检 | `doc/小车部件测试手册.md` |
| 比赛规则与已核对几何 | `doc/比赛规则/规则概要.md` |
| 车上 `/root` 官方目录 | `doc/官方目录索引.md` |

---

## 9. 与人类队员的约定

- AI 可以：写/改 `dev/`、`doc/`、`scripts/`、`code/` 内的代码与文档；跑本地测试；通过 `scripts/ssh_car.py` 做**只读**检查与上传我们自己的文件。
- AI 必须**先问人**：改 systemd / 驱动 / 官方服务；上车跑真车运动（`--arm`）；删除非本会话创建的文件；改供电相关。
- 车端问题时优先看日志：`journalctl -u <service> -n 50`、`/tmp/vision_main.log`、`/tmp/smartcar_status.json`。
