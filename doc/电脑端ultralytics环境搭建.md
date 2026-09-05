# 电脑端 Ultralytics 环境搭建与视觉端运行指南

> 目标：在一台 Windows 电脑上安装 Python + PyTorch + Ultralytics，运行上一届 YOLO 视觉代码，并把感知结果通过 UDP 发送给小车控制端。

---

## 一、电脑要求

- Windows 10/11
- Python 3.10 或 3.11（推荐 3.11）
- 如果有 NVIDIA 显卡，建议安装 CUDA 版 PyTorch，速度更快；
- 如果没有 NVIDIA 显卡，可以用 CPU 运行，但帧率会低一些。

---

## 二、安装 Python

如果电脑还没有 Python，去官网下载：

```text
https://www.python.org/downloads/
```

安装时**务必勾选**：

```text
Add Python to PATH
```

安装完成后验证：

```powershell
python --version
```

应输出类似：

```text
Python 3.11.x
```

---

## 三、创建虚拟环境

推荐用 `venv`，不需要额外装 Anaconda。

在项目目录下创建环境：

```powershell
cd D:\5g\orangepi
python -m venv .venv-ultra
```

激活环境：

```powershell
.\.venv-ultra\Scripts\Activate.ps1
```

> 如果 PowerShell 禁止执行脚本，先运行：
> ```powershell
> Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
> ```

激活后，命令行前面会出现：

```text
(.venv-ultra) PS D:\5g\orangepi>
```

---

## 四、升级 pip

```powershell
python -m pip install --upgrade pip
```

---

## 五、安装 PyTorch

### 有 NVIDIA 显卡（推荐）

先确认显卡：

```powershell
nvidia-smi
```

有输出说明有 NVIDIA 驱动。

然后安装 CUDA 版 PyTorch：

```powershell
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

> 也可以根据你显卡驱动版本选择 `cu118`、`cu121`、`cu124` 等。

### 没有 NVIDIA 显卡（CPU 版）

```powershell
pip install torch torchvision
```

这样默认安装 CPU 版。

---

## 六、安装 Ultralytics 和其他依赖

```powershell
pip install ultralytics opencv-python numpy scikit-image
```

也可以直接使用仓库里准备好的文件：

```powershell
cd D:\5g\orangepi\dev\tools
pip install -r requirements-vision.txt
```

---

## 七、验证环境

### 1. 验证 PyTorch

```powershell
python -c "import torch; print(torch.__version__); print('cuda:', torch.cuda.is_available())"
```

有显卡时：

```text
cuda: True
```

没有显卡：

```text
cuda: False
```

CPU 也能跑，只是慢。

### 2. 验证 Ultralytics

```powershell
python -c "import ultralytics; print(ultralytics.__version__)"
```

### 3. 用上一届模型验证

假设权重在：

```text
D:\5g\orangepi\2025年比赛资料\智能车代码包+软件操作手册\yolo版本视觉+控制代码\yolo版本视觉识别\model\best11sseg.pt
```

运行验证脚本：

```powershell
cd D:\5g\orangepi\dev\tools
python verify_vision_model.py --model "D:\5g\orangepi\2025年比赛资料\...\best11sseg.pt" --source 一张测试图片.jpg
```

能看到检测/分割输出说明环境 OK。

---

## 八、运行上一届视觉代码

视觉代码位置：

```text
D:\5g\orangepi\2025年比赛资料\智能车代码包+软件操作手册\yolo版本视觉+控制代码\yolo版本视觉识别
```

### 摄像头模式

```powershell
cd D:\5g\orangepi\2025年比赛资料\智能车代码包+软件操作手册\yolo版本视觉+控制代码\yolo版本视觉识别\src

python main.py --camera 0 --model ../model/best11sseg.pt --udp-ip 10.23.159.43 --udp-port 5000
```

### 图片/视频模式

```powershell
python main.py --source 图片或视频路径 --model ../model/best11sseg.pt --udp-ip 10.23.159.43 --udp-port 5000
```

> `--udp-ip` 改为小车当前 IP。  
> 如果不知道小车 IP，可以在小车上执行：
> ```bash
> ip addr show
> ```
> 找 `wlx...` 或 `wwan0` 的 IPv4 地址。

---

## 九、常见问题

### 1. `ultralytics` 导入失败

```text
ModuleNotFoundError: No module named 'ultralytics'
```

说明没有激活虚拟环境，或者没安装成功。

```powershell
pip install ultralytics
```

### 2. 摄像头打不开

- 确认摄像头被其他软件占用；
- 试试 `--camera 1`；
- 或者在电脑上先用相机 App 测试。

### 3. 小车收不到 UDP

- 电脑和小车必须在同一网络；
- 确认小车 IP 正确；
- 确认小车控制端正在监听 5000；
- 关闭电脑防火墙或允许 Python 通过防火墙。

### 4. 模型路径错误

```text
FileNotFoundError
```

检查权重路径是否存在，建议用绝对路径。

### 5. 没有 GPU，速度慢

- 正常现象；
- 可以先用 `best11nseg.pt`（nano 模型）测试，速度更快。

---

## 十、推荐联调顺序

1. 先跑通 `verify_vision_model.py`，确认模型能加载；
2. 小车启动控制端 dry-run：
   ```bash
   cd /root/dev
   PYTHONPATH=/root/dev python3 main.py --port 5000
   ```
3. 电脑运行视觉端，观察小车控制端日志；
4. 确认 FSM 能切换后，再考虑 `--real`。

---

> 详细联调步骤见 `doc/电脑视觉端联调.md`。