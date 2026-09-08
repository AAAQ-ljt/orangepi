# img 拍照/录像脚本使用说明

本目录用于存放采集到的图片/视频，所有输出强制保存在本目录内。

## 摄像头0（/dev/video0）

```bash
# 拍照，保存到 img/test1
bash img/camera0.sh photo --folder test1

# 拍照，直接指定 img 内的路径
bash img/camera0.sh photo --output /root/dev/img/test1

# 录像
bash img/camera0.sh video --folder test1
```

## 摄像头2（/dev/video2）

```bash
# 拍照
bash img/camera2.sh photo --folder test2

# 拍照，直接指定 img 内的路径
bash img/camera2.sh photo --output /root/dev/img/test2

# 录像
bash img/camera2.sh video --folder test2
```

## 说明

- 如果不指定 `--folder` 或 `--output`，默认保存到 `img/capture`；
- 目录不存在会自动创建；
- 脚本退出后会自动恢复对应推流服务；
- 只能保存到 `img` 内，否则会报错。
