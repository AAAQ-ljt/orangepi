# dev — 2026 智能车比赛代码

本目录对应小车上的 `/root/dev`，是比赛代码开发目录。

## 目录说明

| 目录 | 说明 |
|---|---|
| `main.py` | 启动器 / 总入口 |
| `config/` | yaml / json 参数 |
| `common/` | JSON 消息协议、数据类 |
| `vision/` | 视觉进程：摄像头采集、YOLO/RKNN 推理、后处理、UDP 发送 |
| `control/` | 控制进程：UDP 接收、FSM 状态机、舵机/电机控制 |
| `hardware/` | 底层硬件封装：PCA9685、IMU、音频等 |
| `scripts/` | 车端调试/诊断脚本 |
| `tests/` | 本地 mock 测试 |
| `tools/` | 数据采集/标注/转换（后续按需添加） |

## 架构

采用双进程 UDP JSON 架构：

```text
vision 进程: 摄像头 → YOLO/RKNN → JSON → UDP
control 进程: UDP → FSM 状态机 → PCA9685/舵机/电机
```

两个进程都运行在 OrangePi 5 上。
