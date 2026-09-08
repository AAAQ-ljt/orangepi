# img 拍照/录像脚本使用说明

本目录用于存放采集到的图片/视频，所有输出强制保存在本目录内。

## 运行位置

- 如果在 `/root/dev` 目录：`bash img/camera0.sh ...`
- 如果已经在 `/root/dev/img` 目录：`bash camera0.sh ...`

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
