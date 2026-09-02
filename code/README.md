# code — 2026 智能车项目代码目录

本目录是 2026 重构项目的代码根目录(结构规划见 `doc\2026重构设计文档.md` §5)。
仓库只跟踪 `code\` 与 `doc\`(见根目录 `.gitignore` 白名单)。

## 当前内容

| 文件 | 说明 |
|---|---|
| `setup_yolo_env.ps1` | 一键创建/重建 YOLOv11 环境(conda 前缀模式,装在 `E:\yolov11`) |
| `environment.yml` | conda 环境规格(可复现) |
| `requirements.txt` | pip 核心依赖 |
| `verify_yolo.py` | 环境+权重验证脚本(加载 2025 `best11nseg.pt` 跑一次推理) |

## YOLOv11 环境

- 环境位置:**`E:\yolov11\env`**(conda 前缀环境,python 3.11)
- conda 包缓存:`E:\yolov11\pkgs`;pip 缓存:`E:\yolov11\pip-cache`
- GPU:RTX 4060 Laptop(8GB);**CUDA 版 torch 需从 pytorch 镜像直链安装**(PyPI 默认分发已改为 CPU 版,download.pytorch.org 直连不稳):
  `pip install --force-reinstall "torch @ https://mirrors.aliyun.com/pytorch-wheels/cu126/torch-2.13.0+cu126-cp311-cp311-win_amd64.whl" "torchvision @ https://mirrors.aliyun.com/pytorch-wheels/cu126/torchvision-0.28.0+cu126-cp311-cp311-win_amd64.whl"`
- 验证 GPU:`python verify_yolo.py` 输出中 `cuda: True` 即为 CUDA 生效

```powershell
conda activate E:\yolov11\env      # 激活环境
python verify_yolo.py              # 验证环境(在 code\ 目录下运行)
python verify_yolo.py --device cpu # 无 GPU 时
```

## 快速开始(训练)

```powershell
conda activate E:\yolov11\env
cd D:\5g\orangepi\code
# 从 2025 权重做迁移微调(数据集就绪后):
# python -m ultralytics train model=<2025权重路径> data=<数据集yaml> epochs=50 imgsz=640
```

> 数据集与训练产物(`datasets\`、`runs\`)已被 git 忽略;模型权重(`*.pt/*.onnx/*.rknn`)不入 git,
> 建议用网盘/NAS 管理大文件。

## 参考文档

- `doc\资料学习导读.md` — 资料阅读顺序
- `doc\车到前预研清单.md` — PC 预研任务(A/B 节与本目录直接相关)
- `doc\2026重构设计文档.md` — 整体设计(§4.2 感知、§7 数据集策略)
