# img 拍照/录像脚本使用说明

本目录用于存放采集到的图片/视频，所有输出强制保存在本目录内。

## 运行位置

- 如果在 `/root/dev` 目录：`bash img/camera0.sh ...`
- 如果已经在 `/root/dev/img` 目录：`bash camera0.sh ...`

## 摄像头0（/dev/video0）

```bash
# 在 dev 根目录
bash img/camera0.sh photo --folder test1

# 在 img 目录内
bash camera0.sh photo --folder test1

# 录像
bash img/camera0.sh video --folder test1
```

## 摄像头2（/dev/video2）

```bash
# 在 dev 根目录
bash img/camera2.sh photo --folder test2

# 在 img 目录内
bash camera2.sh photo --folder test2

# 录像
bash img/camera2.sh video --folder test2
```

## 说明

- 不指定 `--folder` 或 `--output` 时，默认保存到 `img/capture`；
- 目录不存在会自动创建；
- 脚本退出后会自动恢复对应推流服务；
- 只能保存到 `img` 内，否则会报错。
