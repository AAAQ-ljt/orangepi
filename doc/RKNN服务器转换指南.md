# RKNN 模型转换指南（服务器版）

> 目标：在现有 frp 服务器（2 核 / 1.6G 内存 / 4G swap / Ubuntu 26.04 x86_64）上，把 YOLO 模型从 ONNX 转换成 RKNN，供 OrangePi 5 小车 NPU 推理。
> 前置条件：已经有 ONNX 模型文件。

---

## 一、服务器配置评估

实测：

```text
CPU:    2 核
内存:   1.6 GiB
Swap:   4.0 GiB
磁盘:   31G 可用
系统:   Ubuntu 26.04 LTS x86_64
Python: 3.14.4
```

结论：

- **可以使用，但比较紧张**；
- 转换 YOLO11s-seg 这类小模型，4G swap 应该能扛住；
- 如果转换过程中 OOM，可以：
  1. 增加 swap；
  2. 关闭 frp 等非必要服务；
  3. 使用更小的模型 `best11nseg.pt`。

---

## 二、需要准备的文件

```text
best11sseg.onnx
```

如果没有 ONNX，可以先在 Windows 电脑上用 ultralytics 导出：

```powershell
cd D:\5g\orangepi\dev\tools
E:\venvs\smartcar-ultra\Scripts\python.exe -c "from ultralytics import YOLO; YOLO('best11sseg.pt').export(format='onnx', imgsz=640, opset=12)"
```

然后把 ONNX 上传到服务器。

---

## 三、安装兼容 Python

服务器默认 Python 3.14 太新，RKNN-Toolkit2 通常需要 Python 3.10 / 3.11。

### 方式一：使用 Miniconda（推荐）

```bash
cd /opt
wget https://repo.anaconda.com/miniconda/Miniconda3-py311_24.1.2-0-Linux-x86_64.sh
bash Miniconda3-py311_24.1.2-0-Linux-x86_64.sh -b -p /opt/conda
export PATH=/opt/conda/bin:$PATH
conda create -n rknn python=3.11 -y
conda activate rknn
```

### 方式二：使用 apt 安装 Python 3.10

如果不想用 Miniconda，也可以使用 deadsnakes PPA：

```bash
apt update
apt install -y software-properties-common
add-apt-repository -y ppa:deadsnakes/ppa
apt install -y python3.10 python3.10-venv python3.10-dev
python3.10 -m venv /opt/rknn-venv
source /opt/rknn-venv/bin/activate
```

---

## 四、安装系统依赖

```bash
apt update
apt install -y python3-dev build-essential cmake \
    libxslt1-dev libglib2.0-0 libgl1 libgomp1
```

---

## 五、安装 RKNN-Toolkit2

在虚拟环境中执行：

```bash
pip install --upgrade pip
pip install rknn-toolkit2
```

如果 PyPI 没有或版本不匹配，可以从 Rockchip 官方 GitHub 下载 wheel：

```bash
pip install rknn_toolkit2-*.whl
```

验证：

```bash
python -c "from rknn.api import RKNN; print('rknn-toolkit2 ok')"
```

---

## 六、上传 ONNX 到服务器

在电脑上执行：

```bash
scp best11sseg.onnx root@121.40.149.155:/opt/rknn/
```

或者使用项目里的 `scripts/ssh_put.py`。

---

## 七、转换脚本

在服务器创建 `/opt/rknn/convert_rknn.py`：

```python
#!/usr/bin/env python3
from rknn.api import RKNN
import sys

INPUT_ONNX = "best11sseg.onnx"
OUTPUT_RKNN = "best11sseg.rknn"
TARGET_PLATFORM = "rk3588"

def main():
    rknn = RKNN(verbose=True)

    # 1. 配置
    if rknn.config(target_platform=TARGET_PLATFORM) != 0:
        print("config failed")
        sys.exit(1)

    # 2. 加载 ONNX
    if rknn.load_onnx(model=INPUT_ONNX) != 0:
        print("load_onnx failed")
        sys.exit(1)

    # 3. 构建模型
    # 先使用 FP16，不需要量化数据集；如果后续要 INT8，再准备 dataset.txt
    if rknn.build(do_quantization=False) != 0:
        print("build failed")
        sys.exit(1)

    # 4. 导出 RKNN
    if rknn.export_rknn(OUTPUT_RKNN) != 0:
        print("export failed")
        sys.exit(1)

    # 5. 释放
    rknn.release()
    print(f"[OK] {OUTPUT_RKNN} generated")

if __name__ == "__main__":
    main()
```

---

## 八、执行转换

```bash
cd /opt/rknn
source /opt/conda/bin/activate rknn   # 或 source /opt/rknn-venv/bin/activate
python convert_rknn.py
```

成功后生成：

```text
/opt/rknn/best11sseg.rknn
```

---

## 九、把 RKNN 传到小车

```bash
scp /opt/rknn/best11sseg.rknn root@121.40.149.155:/root/dev/models/
```

> 注意：小车是通过服务器转发的，不能直接用 `scp` 到小车？  
> 如果小车走 frp SSH，可以先传到服务器，再通过 `ssh -p 2222 root@121.40.149.155` 的隧道上传到小车：
> ```bash
> scp -P 2222 best11sseg.rknn root@121.40.149.155:/root/dev/models/
> ```

---

## 十、常见问题

### 1. 内存不足 / OOM

```text
Killed
```

处理：

```bash
# 增加 swap
fallocate -l 8G /swapfile
chmod 600 /swapfile
mkswap /swapfile
swapon /swapfile
```

或临时停止 frp：

```bash
systemctl stop frps
# 转换完成后恢复
systemctl start frps
```

### 2. Python 版本不支持

```text
ModuleNotFoundError: No module named 'rknn'
```

确认你在 Python 3.10/3.11 虚拟环境中。

### 3. 缺少 ONNX 模型

先在电脑上用 ultralytics 导出：

```powershell
E:\venvs\smartcar-ultra\Scripts\python.exe -c "from ultralytics import YOLO; YOLO('best11sseg.pt').export(format='onnx', imgsz=640, opset=12)"
```

### 4. 转换后小车加载失败

- 确认 RKNN 是在 `rk3588` 平台生成的；
- 确认小车端 `rknn-toolkit-lite2` 已安装；
- 确认小车 `/root/dev/models/` 下文件存在。

---

## 十一、推荐执行顺序

1. 电脑导出 `best11sseg.onnx`；
2. 上传到服务器；
3. 服务器安装 Python 3.11 + RKNN-Toolkit2；
4. 服务器执行 `convert_rknn.py` 生成 `.rknn`；
5. 将 `.rknn` 传到小车；
6. 小车用 `rknnlite` 加载测试；
7. 跑通后接入小车本地视觉流程。