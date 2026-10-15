# dev/video —— 录像目录 / 副摄录制脚本

本目录是**副摄（下摄 `/dev/video2`）录像**的专用目录，两样东西：

| 文件 | 说明 |
|---|---|
| `record.py` | 录像脚本（自动停/恢复副摄推流，视频存本目录） |
| `*.mp4` | 录出来的视频（运行产物，`.gitignore` 不入库，可随时删/下载） |

## 用法（SSH 上车跑，需 root —— 要调 systemctl）

```bash
# ① 开始录像：自动停副摄推流、等 /dev/video2 释放，然后前台录制
python3 /root/dev/video/record.py start

# ② 在同一个 SSH 窗口按 Ctrl+C 结束；或者另开一个 SSH 再跑一次结束：
python3 /root/dev/video/record.py stop

# 其它常用
python3 /root/dev/video/record.py start --duration 300     # 限时 300s，到点自动停
python3 /root/dev/video/record.py start --name lane_1015   # 存 video/lane_1015/ 子目录
python3 /root/dev/video/record.py start --detach           # 后台录（SSH 断线不停，用 stop 结束）
python3 /root/dev/video/record.py status                   # 是否在录 / 推流状态
python3 /root/dev/video/record.py restore                  # 应急：把副摄推流拉起来
```

* 视频文件名：`cam2_<日期>_<时间>.mp4`（`cam2` = `/dev/video2` 副摄），默认存本目录。
* 录制中每 30s 在终端打一次心跳（已录时长 + 文件大小）。
* **停/恢复推流走 `vision.camera_guard`**（与循迹/采集/探针同一路径）：停完等设备真释放、
  顺手 `reset-failed`，所以不会被 `stream-watch` 看门狗把推流抢回去。
* 任何退出路径都会恢复推流；另带 watchdog 子进程 + `stop/status/restore` 兜底，
  **不会把推流长期忘在停着的状态**。
* mp4 带 `+frag_keyframe`：断电/进程被硬杀，文件仍可播。
* 需要把视频拉回本地：`python scripts/ssh_get.py /root/dev/video/cam2_xxx.mp4 .`

> 主摄（云台 `/dev/video0`）不走这里；巡线视角 = 副摄，所以录数据用本脚本正好。
> 旧的 `dev/scripts/record_video.py` 仍在（`img/camera2.sh video` 会调它），但默认 device 0、
> 没有起停分离与兜底，日常录像用本目录的 `record.py`。
