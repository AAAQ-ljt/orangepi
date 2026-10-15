# AGENTS.md — 智能车项目 AI 作业手册

> 本文件是**所有 AI 助手（ZCode / Claude / 其他 agent）在本仓库工作时的第一读物**，等价于团队开发规范。
> 人类队员也请先读这一份。原始的长版规范已合并进本文件（`doc/开发规范.md` 已删除）。
> 最后更新：2026-10-15——**现场事故复盘 + 四块修复**（详见 `doc/开发文档.md` §7 的 2026-10-15 条目）：① **闸门误发车**（下摄压低不够时"车自己车盖的上沿"落进 `START_GATE_ROI`，那圈暗蓝灰像素面积 0.06~0.14 压着 0.06 阈值抖 → 两次没放板就发车）已修 = ROI 底边 0.95→0.80 + 面积路径**也要主导度≥35** + 画面裁剪 `FRAME_CROP_LINE_Y=384`；当场 30 次武装**零假报**。② **单侧垃圾读数**加两道护栏（单侧也过跳变门限、"回中位保速"加 1.2s 时限）。③ **★ 车停不下来（事故级）**：PCA9685 一直输出最后一次脉宽，相机 UVC 卡死/进程被杀时车会带着油门前冲、Ctrl+C 无效 → 新增**独立进程看门狗** `scripts/esc_watchdog.py`（1s 不喂心跳强写 1500us）+ 读帧挪 daemon 线程（卡死不再阻塞主循环）。④ **增益实测**：亮光下 `kp 0.10 + kd 0.50 @ 1585` 最稳、**kd ≥0.65 会 D 过载**、**暗光 16.6fps 是独立退化因素**。⑤ 日志增强：`session.json`（参数+停机原因）、`lane_ab_summary.py` / `cam_cv2_fps.py` / `car_test_sweep.sh`；现场单 `dev/doc/field-lane-ab-20261015.md`。
> 2026-10-14——**循迹重写为最终脚本**：摄像头机械固定完成、实拍素材 `train/opencv-level1/video/true_track_sub_video/2.mp4`（3573 帧 @28.9fps）到手 → 旧的一整套（`lane_ref_test.py` 实跑入口 / `lane_official.py` 官方口径离线件 / `centroid_follow.py`）**已归档**（`archive/20261014-lane-scripts/` + `dev/attic/lane-old-20261014.tar.gz`）；新入口 = **`dev/scripts/lane_final_test.py`**（单文件：脊搜索检测 + PD 控制 + 记录 + 安全；离线 `--frames/--video/--calibrate`、在线 `--no-motor/--allow-motion`），标定与全片验证报告在 `train/opencv-level1/lane_test/1014_lane_final/report.md`（全片双侧 82%、剔除末段 89%、丢失 0%、中心 p90 5.8px、单帧 10ms）。⚠️ 检测链路**不再照抄官方 Hough**（官方链路在这套素材上锁的是路面刷痕：默认 ROI 双侧率仅 25%），改法/证据见报告。`vision_main` 这条 A 路径仍用 `lane_ref.py`（本轮不动）。旧记录：2026-10-12——①**图传全链复核**：车端两路推流 `ss` 实证 ESTAB→自建 `121.40.149.155`、官方 82.157.204.126 零流量；**`cam_car0027` 是流名、从未停用**（09-15 停的是官方通道；`ffmpeg-stream*` 为 enabled——"只 stop 不 disable"会随重启复活）；新网页入口 `https://car1.liyekai380.dpdns.org/`（实测后端 cam_car0027 有流）。②**双摄身份 VID:PID 更正**：`/dev/video0`=32e4:0234「Global Shutter Camera」、`/dev/video2`=2993:0858「icspring camera」（与旧记载相反，云台归属待 `cam-identify.py` 复核）。③**2026-10-13 镜头压低**：现场档改 ROI 0.15~0.75 / 前瞻带 0.17~0.26、新增贴边容差 `LANE_PAIR_EDGE_TOL_PX=40`（旧档在车道占满画面时会全盲、自检不发车）——细节见 `doc/开发文档.md` §2.2；④**巡线下摄真帧率随光照 9~30fps**（亮环境实测 29.8、暗环境 AE 长曝光掉到 9~16；"15fps"是暗环境真实值）。**循迹控制律已按实测帧率折算**（2026-10-12）：增益/窗口的登记口径 = 15fps（现场整定帧率），运行时时按实测 dt 折算成连续时间形式（`dev/control/rate.py` + `PidRef.step(e, dt)`），所以**换帧率不用重标 kp/kd**；现场 A/B 表继续按 15fps 口径写数字。2026-10-11 记录：巡线**前瞻带根治**：下摄平视角下真车道线是虚线、在 y≈195~205 就从画面左右边缘出去 → 自动前瞻带加"不贴边"余量后落到 y≈120~168，自检可发车已复验；`kd` 折算方向修正、`prime()` 防踢腿。现场命令单已删除（2026-10-15；新的循迹现场清单见 `lane_test/1014_lane_final/report.md` §6）。其余同 v11：**模型轨 = 红绿灯专属 3 类 light3cls，8 类契约与 4cls 全部废弃/退役，其余元素全 OpenCV**；RKNN 转换移交队友。**doc 已重构：开发相关内容只信 `doc/开发文档.md`，旧文档进 `doc/attic/` 只读不维护**）

---

## 0. 三十秒背景

**项目**：2026 全国大学生智能汽车竞赛 · 室外 5G 远程驾驶无人车赛。
**硬件**：Orange Pi 5（RK3588S）+ XT-NetRC 阿克曼底盘 + PCA9685 + 双摄（**两路都是 USB**；2026-10-12 VID:PID 实测：`/dev/video0`=32e4:0234「Global Shutter Camera」模组 / `/dev/video2`=2993:0858「icspring camera」模组＝**下摄副摄，巡线用**——与旧记载相反，云台归属待 `cam-identify.py` 复核）+ SIM8200 5G。
**目标**：**完赛，并尽量快**。规则里"停车超 20s 不动 = 比赛失败"，"停错车位/无轮入区 +100s"，"撞锥 +20s"，"压线 +10s/次"——所以**可靠性优先于速度**。
**当前阶段**：发车（P0-2）、巡线（P1-1，受视角限制不作为完赛路径依赖；10-11 前瞻带根治后自检可发车）、日志降频与语音非阻塞（P1-5）、锥桶 HSV 检测与 S 型绕行（P1-3 部分）已落地；**当前硬阻塞 = 红绿灯 3 类（off/red/green）标注与补拍 + 操场现场标定**（HSV 主通道 W3 即可通路，模型作为第二校验通道，见 `doc/补拍清单.md`）。**现场流程/命令**：`doc/开发文档.md` §5 + `train/opencv-level1/lane_test/1014_lane_final/report.md` §6（10-11 那两份现场单已删除，2026-10-15 文档清理）。

**先看文档再动手**：
- `doc/开发文档.md` —— **唯一开发文档**：现状 / 决策 / 元素设计 / 数据模型 / 计划进度 / 代码交接 / 执行记录全在里面（旧路线图/方案/专项/循迹交接等已归档 `doc/attic/`，**只读不维护**）
- 现场流程/命令：`doc/开发文档.md` §5 + `train/opencv-level1/lane_test/1014_lane_final/report.md` §6（10-11 的两份现场单已删除，2026-10-15 文档清理）

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
7. **★ 任何"软件停车"都必须有一条不依赖主控进程的退路**（2026-10-15 事故教训）：PCA9685 会**一直输出最后一次
   写进去的脉宽** —— 主控卡死/被杀，车就带着油门继续跑，且 Ctrl+C 只是给主循环设标志、此时无效。
   所以动电机的运行**必须**有独立看门狗在位（`scripts/esc_watchdog.py`，`lane_final_test.py --allow-motion`
   会自动拉起；**跑前确认日志里有「电调看门狗已拉起」**），并且操作员**手边随时能断电**。
8. **车不停时的处置顺序**：① **直接断电**（唯一覆盖"总线/驱动板本身坏了"的手段）；② `car-mode.sh estop`；
   ③ Ctrl+C（只管我们的脚本，脚本卡死时无效）。**别把手机/网页停在摇杆页**：我们的脚本退出后会恢复
   opi-control，那是官方手动驾驶服务，摇杆的输入会直接让车动。

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

> 权威版本在 `doc/开发文档.md` §4；这里只放摘要，**动手前先看那篇的对应条目和出口标准**。

| 批次 | 任务 | 状态 |
|---|---|---|
| **P0-1** | **红绿灯 3 类标注 + 补拍**（灯灭态优先、负样本≥500、停车位多片段、红锥 HSV 标定素材）→ 切分 → 训练（`code/train_light.sh`，1 天） | 🟡 已启动（数据盘点完，见 `doc/补拍清单.md`；其余元素全 OpenCV 不依赖标注） |
| **P0-2** | **修发车逻辑**：边沿触发 + 去抖 + 超时兜底（`start_gate.py` + FSM + 协议） | ✅ 已完成（剩现场标定 `START_BLUE_*`；现场流程见 `1014_lane_final/report.md` §6） |
| **P0-3** | 采购光电传感器（≥1m、阈值可调） | ⬜ 需人工 |
| **P0-4** | 现场几何实测（赛道宽、停车区黄线、斑马线 30cm 停区、是否有编码器、红灯长等 vs 20s 判失败） | 🟡 10-10 操场执行（含官方口径核对） |
| **P1-1** | 扫线 + 误差滤波 + PID（**入口 `scripts/lane_final_test.py`**（2026-10-14 最终版：脊搜索检测 + PD + 记录 + 安全，单文件自包含）；旧 `lane_ref_test.py` / `lane_official.py` / `centroid_follow.py` **已归档**；A 路径检测器 `lane_ref.py` 仍在用） | ✅ 代码级完成 + 1014 素材全片验证（双侧 82%/剔除末段 89%/丢失 0%/中心 p90 5.8px/单帧 10ms，报告 `lane_test/1014_lane_final/report.md`）；现场待办：静止自检标定 target_x + 实跑复验 |
| **P1-2** | 距离估计与减速停准（框底边 → 地面坐标 → 速度曲线） | ⬜ |
| **P1-3** | FSM 主体重构（斑马线 10s、红绿灯光电、锥桶绕行、停车五阶段） | 🟡 锥桶部分已落地（`cone_detect` hsv + S 型绕行）；斑马线/灯/停车待接 |
| **P1-4** | 停车：蓝板接地点 + IPM 判左右 → 五阶段入库 | ⬜ |
| **P1-5** | 日志降频 + 语音播放非阻塞 | ✅ 已完成（`PRINT_EVERY_N` + `audio.py` 后台播放） |
| **P2-*** | 阈值现场标定、双摄分时复用、性能优化、技术手册 | ⬜（语义分割轨已放弃，v11） |

---

## 3. 仓库结构

```text
D:\5g\orangepi\
├── AGENTS.md            ← 本文件（agent 作业手册）
├── dev/                 ← 比赛代码（本地 ⇄ 车上 /root/dev）
│   ├── main.py          ← 控制端入口（--real --arm 才动硬件）
│   ├── config/          ← settings.py（阈值/限幅集中在此，禁止散落硬编码）
│   ├── common/          ← protocol.py（UDP 消息契约）、通用工具
│   ├── vision/          ← 视觉进程：lane_ref / start_gate / rknn_detector / postprocess / camera_guard
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
    ├── img/             # 拍照/录像脚本（脚本入库；拍出来的图片不入库）
    ├── data/            # ★ 采集数据统一放这里（**整目录不入 git**，2026-10-15 起；
    │                    #   原 setup_eyes/ show_test/ 等已迁入：`dev/data/<用途>/`）
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

# 单测（纯函数 + __main__ 风格，不需要 pytest）—— 直接全部跑一遍
for t in tests/test_*.py; do "$PY" "$t" || echo "FAIL $t"; done
# 当前 21 个文件：filters/pid/lane_arbiter/protocol/start_gate/control/elements/site_config/
#   camera_guard/rknn_detector/bench_logic/lane_ref/lane_final/lane_final_run/color_elements/
#   cone_detect/imu/parking_logic/lane_rate/vision_replay + integration
#   （lane_official / lane_ref_run 随旧脚本归档，2026-10-14）
"$PY" tests/test_integration.py        # 端到端：起控制端 + UDP 打假数据（dry-run）
"$PY" tests/test_lane_ref_run.py       # 循迹：假摄像头跑完整状态机（自检→发车→跟线→丢线停车）

# 视觉单模块自测（不需要摄像头/模型）
"$PY" vision/vision_main.py --test-image 某张赛道图.jpg   # 打印 lane / start gate 判据
"$PY" scripts/lane_final_test.py --image 某张赛道图.jpg    # 最终循迹脚本单图读数
"$PY" scripts/lane_final_test.py --frames ../train/opencv-level1/video/frames/2 --calibrate  # 素材标定档
"$PY" main.py --port 5000                                 # dry-run 控制端
```

- 所有新模块**必须能在没有摄像头、没有 PCA9685、没有模型的机器上 import 并测试**（用 mock 与合成图）。
- 测试文件命名 `test_*.py`，用断言 + `if __name__ == "__main__"` 直接跑，别引入新的测试框架依赖。
- 提交前**必须**跑通 `tests/test_*.py` 全部文件（当前 21 个）；高风险运动代码（循迹、台架、
  退出路径）除了纯函数测试，还要有"假硬件跑完整流程"的集成测试（照 `tests/test_lane_final_run.py`）。

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
python scripts/serial_push.py xx.tar.gz /tmp/xx.tar.gz --binary --chunk 3500   # 二进制必须 --binary

⚠️ 串口三条铁律（2026-09-21 实测踩过）：
1. **绝不要发 Ctrl+C** —— Rockchip FIQ Debugger 以它为触发键，一按就掉进 `debug>`（系统还在跑）；
   切回来敲 `console`。卡住的命令行只发回车。
2. 命令要**短、单层引号、不用 `$(...)`/`\"`**：长命令会被传坏（`echo`→`ecHo`），
   出现"半截执行"（删了文件却没留归档）；archive/rm 这类破坏性动作分开跑并立刻 `ls` 验证。
3. 车有蜂窝网时优先**让车自己从 GitHub 拉仓库**（4MB/~20s），串口只补未 push 的提交。
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
| **以为"程序活着"就有刹车** | **PCA9685 保持最后一次脉宽**：主控卡死（相机 UVC 卡死让 `cap.read()` 无限阻塞）或进程被杀 → 车带着最后的油门一直跑、**Ctrl+C 无效**（信号只设标志），现场两次都要断电。修法：独立进程看门狗（`esc_watchdog.py`，1s 不喂心跳强写 1500us）+ 读帧 daemon 线程（帧龄超时即退出）；**跑前确认日志有"电调看门狗已拉起"，车不停先断电** |
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
| 摄像头 index 搞反（会把云台画面当赛道） | **2026-10-12 VID:PID 实测变更**：`/dev/video0`=32e4:0234「Global Shutter Camera」、`/dev/video2`=2993:0858「icspring camera」——**与 2026-09-16 记载相反**（模组疑似换位/枚举变动），**云台归属跑 `dev/scripts/cam-identify.py` 复核后定稿**。功能口径不变：巡线固定 `--camera 2`（video2）。⚠️ UVC 设备号不跨重插稳定 → 建议 udev 按 USB 口（7-1.3/7-1.4）固定设备名 |
| 以为摄像头能跑 30fps | **2026-10-12 实测更正**：主推流那路真 30fps；**巡线下摄在暗环境真实出帧率只有 9~18fps**（同一颗亮环境跑满 30；cv2 与裸 ffmpeg 读数一致 → 瓶颈不在代码/读取方式——**2026-10-15 更正：这句是错的**，当时 cv2 带着 `CAP_PROP_BUFFERSIZE=1`（把驱动砍半），见本表最后一行）；且 ffmpeg 推流的"30fps"里可能混**编码器复制帧**（当天实测 39%，主摄 0%）——**别拿"流到达 30 帧"当传感帧率**；控制台 15 帧=下摄真实出帧率。**控制律已按实测帧率折算**（登记口径 15fps，见 `dev/control/rate.py`）：帧率从 15 变到 30 不再需要重标增益 |
| 以为 `cam_car0027` 已停 / 官方服务器还在推流 | **2026-10-12 全链复核**：车端双路推流 `ss -tnp` 实证 ESTAB→自建 `121.40.149.155:8554`，官方 `82.157.204.126` 零连接（`frpc.service` 09-15 起 disabled）。`cam_car0027` 是**流名**（沿用官方命名风格）——09-15 换的是**服务器地址**，流名没改、一直在用；`ffmpeg-stream*` 单元 **enabled（开机自启）**，**只 stop 不 disable 的"停"会随重启复活**（2026-10-12 开机即被 systemd 自动拉起）。新网页入口 `https://car1.liyekai380.dpdns.org/`（实测其后端 `cam_car0027` 有流）——**要停流先确认新图传不依赖它**；改 systemd 需队长确认 |
| **下摄"看着赛道"就以为白线在画面里** | 2026-09-19 抓图实锤：下摄俯仰角太**平**（几乎水平看出去）时，画面里只有赛道中段 + 塑料膜褶皱反光，两条白线贴在**画面左右外沿/画面外** → 扫线无论如何都锁不到线（实测：左 18 行、右 8 行碎片，conf 0）。**先看画面再调参数**：`sudo python3 scripts/diag/lane_probe.py --camera 2 --frames 6`（终端直接给结论 + 叠加图），把两条白线调进画面下半部再谈阈值 |
| 以为「云台压下去」就能拿到俯视视角（**巡线固定用下摄 video2，云台不参与巡线**） | 2026-09-19 实测**云台俯仰行程不够**：`--sweep-tilt` 扫了 40°~140° 共 9 档，最好的一档（tilt=105°）conf 也只有 0.24（可用要 ≥0.4），画面依旧是"顺着赛道看"。**下摄视角拿不到"俯视车道"，白线循迹受视角硬约束** —— 但 09-21 Canny+Hough 重构后实车可锁线（09-23 操场可跟线，S 弯画龙已上三护栏：单侧限幅/弱对限幅/画龙检测）；10-10 现场选型定案：**④ 边缘法唯一**（②③ 质心法反光拉偏，淘汰）；**不作为完赛路径依赖**（直道开环 + 元素检测触发即可完赛） |
| **把前瞻带 / ROI 设在线"贴边出画"的那一段** | 下摄平视角下真车道线是**虚线**、在 y≈195~205 就从画面左右边缘出去（该行真值 x≈21/643）→ 贴边段拟合截距噪声最大（643 被读成 686、中点偏 ~29px）且频繁丢单侧。已根治：`rows_with_support` 加"不贴边 30px"余量让**自动前瞻带避开该段（落到 y≈120~168）**；**到场先看图验线对**（2026-10-14 起用 `lane_final_test.py --no-motor`；标定档与证据见 `train/opencv-level1/lane_test/1014_lane_final/report.md`），别把带子按回贴边区 |
| 把"扫线循迹"当成完赛的必要条件 | **不是**。规则只罚"压线 +10s/次"，而赛道是直道：直线用**固定舵角 + 中位微调**（`site.yaml` 里改 `servo_center_angle`）就能不压线；变道/绕锥/入库可以按官方上届的做法**开环（时间/距离触发）**，停车与斑马线用**元素检测**触发。别让一个拿不到视角的功能卡死整个进度 |
| 凭注释/二手结论判断"参考实现怎么做的" | **必须查原码**。2026-09-21 查 `oldCode/src/vision/vision.cpp::picture()` 才发现"参考实现单侧丢线不给中心"**是错的**——它对丢线的一侧直接代画面边界（`l=0` / `r=frame.cols`），读数照样给中心；照错结论改出来的"单侧就丢帧"让车一丢线就停（详见 `doc/开发文档.md` §2.2/§6.4） |
| 把 `settings` 里的阈值直接喂给另一个模块 | **先核单位**。`ERROR_FILTER_OUTLIER=15` 的单位是"误差单位（1 单位 = 4px）"，而循迹脚本的误差是**像素** → 直接传 15 会把任何 >15px 的正常变化当离群点丢掉、滤波后误差长期冻在旧值（正确值 60px，2026-09-21 修） |
| 用"每行独立从中心往外找第一个白点"找车道线 | 打印跑道塑料膜的反光是**低饱和亮区**，颜色上与白线无法区分（实测反光最亮像素 S≈23、白线 S≈40），且正好在画面中间 → 每行都会先撞上反光。已改为 **`trace_boundary()` 连续跟踪**（多种子 + 双向行走；线：连续/平滑/宽度稳定，反光：一团一团/宽度突变），并保留形状过滤（尺度无关的细长比）与**配对宽度闸门**兜底 |
| 用"当前无挡板"直接发车 | **上电即冲**。必须边沿触发（`start_gate.py`） |
| `is_barrier` 之类字段硬编码成常量 | 会让 FSM 走进错误分支；协议字段必须来自真实感知 |
| 每帧 `print` 完整状态 | 15~25FPS 下日志本身就吃掉可观 CPU；改为每 N 帧或写状态文件 |
| 同步 `subprocess.run` 播放音频 | 阻塞整个控制循环，期间 failsafe 失效；改后台线程 |
| 用 YOLO 的"车道线检测框中心"当车道中心 | 只在上届 seg 模型下成立；本届用**扫线**为主力（`lane_ref.py`） |
| 把"左右"做进类别标签 | `fliplr` 增广会把标签翻错；左右是**空间关系**，用几何判定 |
| Roboflow 里做 train/val/test 划分 | 它随机划分会把连续帧拆到两边 → 验证集虚高；**必须按拍摄片段整组切分** |
| 用上届模型权重 | `dev/models/best11nseg.rknn` 类别与本届不符，只能验证工具链 |
| 停车判定看"黄框是否可见" | 裁判放法不可预测；用**外框 + 蓝板接地点 + IPM** 的几何先验 |
| 用 `cx < 图像宽/2` 判左右车位 | 车斜着进场时必错；要比 IPM 俯视图里 `board_x` 与外框中点 |
| 停车/斑马线"看到就断油" | 会滑过停车点（压线 +10s、未停够 10s 再 +20s）；需要**距离估计 + 减速曲线** |
| 两个模型同时无脑调 RKNN | NPU 争抢导致总吞吐下降；先"单推理线程 + 任务队列"，必要时再 core mask 隔离 |
| 主循环里 `cv::imwrite` / X11 `imshow` | 卡住控制回路 / headless 下直接崩；调试预览用开关控制 |
| 依赖 `main.py --dry-run` 参数 | 该参数不存在，dry-run 是**默认行为** |
| **图传延迟 30s+ 且恒定不自愈**（2026-10-11 已修） | ffmpeg 输入队列 `thread_queue_size 512` 是"延迟水库"（512 帧按真实帧率可攒十几~三十秒）：5G 上行一旦低于编码码率，TCP 推流背压把它灌满，之后 WebRTC 按 1x 慢放旧帧，只有重启才能清空。已改两个 `ffmpeg-stream*` 单元：queue→16（积压上限 ~1s，卡顿时丢帧代替攒延迟）+ capped CRF（`-crf 23 -maxrate 1200k -bufsize 300k`，两路 ≤2.5Mbps）——改后源端滞后恒定 ~2s、drop=0。积压测量：`/var/log/ffmpeg-stream.log` 的 `time=` 与进程 `etimes` 相减。原件备份 `/root/dev/backup-20261011/`。另：网页只有 WebRTC、没有 HLS 代码，"延迟大=HLS 兜底"的旧说法作废 |
| **开机后 frp/SSH 全断、frpc 卡 activating**（2026-10-13 已修） | 不是 frp 的问题，是**车没网**：`car-net.service` 的 ExecStart 直接执行 `/root/dev/scripts/net/car-net.sh auto`，脚本一丢执行位 systemd 就报 `203/EXEC`、重试 10 次放弃 → 不拨号 → 无上行 → frpc 连不上服务器。**根因在仓库**：全库 `.sh` 在 git 里都是 `100644`（无执行位），凡"按仓库原样同步到车"必复现（同步工具不会替 git 补 +x）；之前能用是因为有人手动拨过号，把坑盖住了。车端已修：全部 `.sh` 补 `+x`，`systemctl reset-failed car-net.service` 后 restart 实测通过（日志"上电联网完成（蜂窝）"）。**仓库侧待办**：对 `dev/scripts` 下被 systemd 直接执行的入口脚本 `git update-index --chmod=+x` 后提交，否则下次全量同步/换车再踩。排查路径：`car-net.sh status` 看"默认出口/蜂窝"，`journalctl -b -u car-net.service` 搜 `203` |
| **cv2 里 `CAP_PROP_BUFFERSIZE=1` 会把出帧率砍半**（2026-10-15 小车实测） | 同一颗下摄、同一画面：**BUFFERSIZE=1 → 14.1fps；不设（驱动默认 4）→ 28.0fps**；ffmpeg 走同一设备 29.8fps。单缓冲时驱动必须等应用把 buffer 放回才能抓下一帧。`vision/camera_guard.open_camera()` 已改为 **不设 BUFFERSIZE + 显式 FPS=30**——之前所有 cv2 工具（循迹脚本/采集/探针）都只吃到 ~14fps，"模组硬限制 15fps"就是它；修后循迹主循环 **14.7 → 29.9fps**。**别设回 1**；要压滞后用 2~4 |
| **直接往车端推 Windows 工作区的 `.sh`（CRLF）** | 工作区字节被原样推到车上 → `set -euo pipefail` 报 `pipefail: invalid option name`、脚本当场死（2026-10-15 car-mode.sh 实例）。**推 shell/systemd/conf 前先转 LF**（`.gitattributes` 已加 `*.sh/*.service/*.timer/*.link/*.conf → eol=lf`）；`ssh_sync` 推的一批目前都是 `.py/.md`（CRLF 无害），但别推 `.sh` 除非确认过换行 |

---

## 7. 参考实现（官方代码，只读参考）

> **2026-10-08 更新**：官方资料已统一收进 `比赛资料/`（**约 18GB，全部不入库**，见 `.gitignore` 白名单）。
>
> - `比赛资料/2025年智能车代码包+软件操作手册/` —— 上届（2025）两套 C++ 代码（opencv 版、yolo 版）
>   + `newCode_详细技术文档.md`（3174 行，上届全套技术解析）
> - `比赛资料/5G远程驾驶无人车(2026)学习资料/竞赛进阶案例/14.5G车室外赛代码＋一镜到底开箱/`
>   —— **本届室外赛官方样例代码**：`demo(中文注释).cpp`（含完整赛道状态机 + 各元素 HSV 阈值）、
>   `双线循迹代码.txt`（纯循迹版）。**其寻线链路与我们 `dev/vision/lane_ref.py` 逐参数一致**
>   （Canny 60/140 + 自适应 ±2/±4、HoughLinesP rho=1/θ=0.05/thr=50/minLen=30/maxGap=5、
>   斜率 0.25~2.0 分左右、逐行中线均值、单侧代画面边界、误差 = center − 320）——
>   这证明我们的基线方向不是猜的，**ROI 是它唯一值得对照的差异**（它用画面底部 y 300~480）。
> - 同目录 `穿越火线demo/demo代码/` —— 独立小工程（`pid.c`/`pca9685.c`/`imu.c`），
>   锥桶绕行几何判据（锥度先验、接地点带宽、间隙安全）齐全
> - `无人车硬件配置/python/` —— 官方上位机 `pid_monitor.py`（UDP 8766/8767 调参）、`pc_keyboard_tcp_server.py`

`2025年智能车代码包+软件操作手册/` 下两套（opencv 版、yolo 版）。值得移植的：

| 移植项 | 出处 | 价值 |
|---|---|---|
| 逐行扫线 + 纵向加权误差 | `opencv版本…/code/image.cpp:1224-1331` + `image.h:40-56` | 比"检测框中心"鲁棒，天然兼容单侧线丢失 |
| 误差滤波器 | `yolo版本…/HardWare/ErrorFilter.h` | 中位数离群 + 新值加权，抗 YOLO 跳变 |
| PD 参数起点 | `yolo版本…/Controller/State/src/TrackingState.cpp:12-22` | 巡线 kp 0.2~0.25 / kd 0.5~0.8（纯 PD） |
| 状态机防抖阈值 | 同上各 State | 挡板 3 帧、斑马线 3 帧、停车牌 3 帧投票 |

**不要照抄它们的坑**（2026-10-08 已回原码核实）：`laneChangeCount` 初值写死导致半数状态不可达、泊车用 `exit(0)` 收尾不给电调中立、变道泊车全靠固定时间开环；
**PID 无 dt 已实锤**（`穿越火线demo/pid.c`：微分是 `Kd*(Error0-Error1)`、积分是 `ErrorInt += Error0`，都不带时间 → 有效增益随帧率漂移，所以它们干脆 `Ki≡0`，文档也写"Ki 不建议使用"）；
无 slew rate（只有 ±15 硬限幅，官方 yolo 版还有一处把舵机 clamp **注释掉**了）；
**丢线没有降级通路**——双侧全丢时中点 =(0+319)/2 恒为 159.5 → 误差固定 ≈ −22 单位 → **朝一侧打死舵**，既不回中也不停车；
⚠️ **同名宏在不同官方文件里语义相反**：`SERVO_MIN/MAX` 在 `双线循迹代码.txt` 里是 `1000(最右)/2000(最左)`，在 `穿越火线demo/demo.cpp` 里却是 `2000(左极限)/1000(右极限)` —— 读官方代码必须逐文件确认极性。

---

## 8. 文档地图（详见 `doc/README.md`）

| 我要… | 看这篇 |
|---|---|
| **去操场** | 通用流程/元素判据 `doc/开发文档.md` §2/§5；**循迹现场清单** `train/opencv-level1/lane_test/1014_lane_final/report.md` §6；10-11 两份现场单已删除（2026-10-15） |
| 知道现在该干什么 / 理解方案决策 / 代码交接 | `doc/开发文档.md`（**唯一开发文档**；旧路线图/方案/专项/循迹交接已归档 `doc/attic/`） |
| 采数据 / 标注 / 训练 / 转 RKNN | `doc/开发文档.md` §3 + `doc/补拍清单.md`（light3cls；8 类与 4cls 已废弃） |
| 连车 / frp / 服务器 | `doc/远程连接与服务器手册.md` |
| 小车联网（蜂窝/WiFi/隧道/串口救援） | `doc/车端网络方案.md` |
| 图传与模式切换（car-mode.sh） | `doc/图传与模式切换方案.md` |
| 红绿灯/锥桶/停车/循迹判据细节 | `doc/开发文档.md` §2（专项旧文在 `doc/attic/专项方案/`） |
| 上车自检 | `doc/开发文档.md` §6.3 + `bash dev/scripts/car-test/test_all.sh` |
| 比赛规则与已核对几何 | `doc/比赛规则/规则概要.md` |
| 官方实验教程（只读） | `doc/官方实验文档/`（原始资料在仓库根 `比赛资料/`） |
| 车上 `/root` 官方目录 | `doc/官方目录索引.md` |

---

## 9. 与人类队员的约定

- AI 可以：写/改 `dev/`、`doc/`、`scripts/`、`code/` 内的代码与文档；跑本地测试；通过 `scripts/ssh_car.py` 做**只读**检查与上传我们自己的文件。
- AI 必须**先问人**：改 systemd / 驱动 / 官方服务；上车跑真车运动（`--arm`）；删除非本会话创建的文件；改供电相关。
- 车端问题时优先看日志：`journalctl -u <service> -n 50`、`/tmp/vision_main.log`、`/tmp/smartcar_status.json`。
