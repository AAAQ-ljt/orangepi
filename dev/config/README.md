# 配置文件目录

存放 yaml/json 参数（2026-09-23 修复：原文件内容损坏，换行被写成字面量 `` `n``n ``，markdown 渲染成一行）。

## 各文件职责

| 文件 | 内容 |
|---|---|
| `settings.py` | 阈值/限幅/坐标目标的唯一权威（AGENTS.md §5.1），标单位与用途，现场可改 |
| `site.yaml` | **车端本地标定值**（`servo_center_angle` / `target_x` / `steer_sign`），由 `bench_test.py --calibrate` 或起步自动标定写入；**不入 git、不被同步覆盖**（ssh_sync 排除） |
| `model_profile.yaml` | 模型类别表（元素名 ↔ 类别 id），`vision/elements.py` 读取 |

> 注：历史文档提到的 `hardware.yaml`、`ground_map.yaml`、`parking_map.yaml` 均不存在，舵机限幅在 `settings.py` 的 `SERVO_*_ANGLE`。