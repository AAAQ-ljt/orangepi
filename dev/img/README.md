# img 拍照/录像脚本使用说明

本目录用于存放采集到的图片/视频，所有输出强制保存在本目录内。

## 摄像头角色（2026-09-16 实测确认）

| 脚本 | 设备 | 角色 |
|---|---|---|
| `camera0.sh` | `/dev/video0`（icspring） | **主摄 ＝ 云台摄像头**（红绿灯环节抬头看灯；推流 `cam_car0027`） |
| `camera2.sh` | `/dev/video2`（Global Shutter） | **副摄 ＝ 下摄**（朝向赛道地面，**巡线扫线用**；推流 `cam_car0027_sub`） |

> ⚠️ 采集训练数据时，**扫线/赛道类数据用 `camera2.sh`**（下摄视角，与部署时视觉输入一致）。

## 运行位置

- 如果在 `/root/dev` 目录：`bash img/camera0.sh ...`
- 如果已经在 `/root/dev/img` 目录：`bash camera0.sh ...`

## ⚠️ 别忘了限制张数

`photo` 模式默认**一直拍到 Ctrl+C**。如果是远程/脚本调用（SSH 断开后进程会被 orphan 继续跑），
一定要加 `--count`：

```bash
bash img/camera0.sh photo --folder lane --count 100     # 拍 100 张自动停
```

（2026-09-16 就踩过这个坑：一次远程调用没加 `--count`，SSH 断开后进程继续写，几分钟灌了 800 张 37MB。）

## 摄像头0（/dev/video0）

```bash
# 在 dev 根目录
bash img/camera0.sh photo --folder test1 --show

# 在 img 目录内
bash camera0.sh photo --folder test1 --show

# 录像
bash img/camera0.sh video --folder test1
```

## 摄像头2（/dev/video2）

```bash
# 在 dev 根目录
bash img/camera2.sh photo --folder test2 --show

# 在 img 目录内
bash camera2.sh photo --folder test2 --show

# 录像
bash img/camera2.sh video --folder test2
```

## 关于 X11 / 弹窗预览

### 情况 1：通过 SSH 远程操作，想在自己电脑上看到画面

必须使用 **SSH X11 转发**，并且是从**你自己的电脑**连接到小车：

```bash
ssh -X -p 2222 root@121.40.149.155
```

或者在 MobaXterm 里开启 X11 forwarding。

连接后直接运行：

```bash
cd /root/dev/img
bash camera0.sh photo --folder test1 --show
```

注意：

- 不要在小车上再 `ssh -X` 到小车自己，这样 X11 转发通常不会到你电脑；
- 如果小车有 HDMI 显示器，直接在车上本地终端运行，不用 SSH。

### 情况 2：小车直接接显示器/HDMI，在车上操作

小车本身已经有桌面/X11 环境，直接运行即可：

```bash
bash camera0.sh photo --folder test1 --show
```

### 情况 3：没有显示环境

如果 `DISPLAY` 未设置，脚本会提示：

```text
WARNING: DISPLAY not set, preview window may not show. Saving will continue.
```

此时不会崩溃，照片仍会正常保存。

## 说明

- 不指定 `--folder` 或 `--output` 时，默认保存到 `img/capture`；
- 目录不存在会自动创建；
- 脚本退出后会自动恢复对应推流服务；
- 只能保存到 `img` 内，否则会报错。
