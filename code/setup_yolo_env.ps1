# 一键创建 / 重建 YOLOv11 环境(conda 前缀模式)
# 环境位置:E:\yolov11\env;conda 包缓存:E:\yolov11\pkgs;pip 缓存:E:\yolov11\pip-cache
param(
    [string]$EnvDir = "E:\yolov11",
    [string]$Mirror = "https://pypi.tuna.tsinghua.edu.cn/simple"
)

$ErrorActionPreference = "Stop"
$env:CONDA_PKGS_DIRS = Join-Path $EnvDir "pkgs"
$env:PIP_CACHE_DIR = Join-Path $EnvDir "pip-cache"
New-Item -ItemType Directory -Path $EnvDir -Force | Out-Null

Write-Host "== 1/2 创建 conda 环境: $EnvDir\env (python 3.11) =="
conda create -p (Join-Path $EnvDir "env") python=3.11 pip -y

Write-Host "== 2/3 安装 CUDA 版 torch/torchvision (NVIDIA GPU;无显卡机器跳过此步,ultralytics 将使用 CPU 版) =="
# 说明:PyPI 默认分发已改为 CPU 版;download.pytorch.org 直连不稳,国内用阿里云 pytorch-wheels 镜像直链。
# 可用版本请在浏览器打开 https://mirrors.aliyun.com/pytorch-wheels/cu126/ 查看(找到 torch-*-cp311-*-win_amd64 与配套 torchvision)。
& (Join-Path $EnvDir "env\python.exe") -m pip install --force-reinstall -i $Mirror `
  "torch @ https://mirrors.aliyun.com/pytorch-wheels/cu126/torch-2.13.0+cu126-cp311-cp311-win_amd64.whl" `
  "torchvision @ https://mirrors.aliyun.com/pytorch-wheels/cu126/torchvision-0.28.0+cu126-cp311-cp311-win_amd64.whl"

Write-Host "== 3/3 安装 ultralytics + opencv-python =="
& (Join-Path $EnvDir "env\python.exe") -m pip install -i $Mirror ultralytics opencv-python

Write-Host ""
Write-Host "完成!使用方式:"
Write-Host "  激活环境 : conda activate $EnvDir\env"
Write-Host "  验证环境 : python verify_yolo.py"
Write-Host "  (在 code\ 目录下运行,模型/测试图默认指向 2025 资料)"
