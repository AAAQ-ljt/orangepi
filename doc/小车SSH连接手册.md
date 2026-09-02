# 小车 SSH 连接手册（AI 自动连接用）

> 本文件供后续 AI（其他对话或智能体）查看后**自动连接小车**使用。
> 所有信息基于 2026 年实测验证，可放心直接操作。

---

## 1. 一句话总结

小车是 **Orange Pi 5（RK3588，Ubuntu 20.04）**，两种连接方式（**优先走服务器转发**，隧道不通再用热点直连）：

| 方式 | 命令 | 场景 |
|---|---|---|
| 🥇 服务器转发（推荐） | `ssh -p 2222 root@121.40.149.155`（密码 `orangepi`） | 任何能上网的机器，不受热点 IP 变化影响 |
| 🥈 热点局域网直连 | `ssh root@10.68.1.43`（密码 `orangepi`） | 与本机同一手机热点下，隧道故障时兜底 |

Windows 本机已装 paramiko，直接用 `scripts\ssh_car.py`（免交互，见第 5 节）。

---

## 2. 硬件与系统信息（实测）

| 项目 | 值 |
|---|---|
| 主机名 | `orangepi5` |
| 板卡 | Orange Pi 5（Rockchip RK3588，aarch64） |
| 系统 | Ubuntu 20.04.6 LTS (Focal Fossa) |
| 内核 | `5.10.160-rockchip-rk3588` |
| 架构 | aarch64 |
| SSH 服务 | 默认 openssh，端口 22，允许 root 密码登录 |

---

## 3. 网络拓扑与连接参数

### 3.1 方式一：服务器转发（主链路）

```
[小车 OrangePi5] --WiFi(手机热点)---> [frp 隧道] <---公网--- [服务器 121.40.149.155:2222] <--- [任意机器]
    sshd:22 ──frpc──▶ 121.40.149.155:7000           frps 转发 2222 → 小车 22
```

| 参数 | 值 |
|---|---|
| Host | `121.40.149.155` |
| 端口 | `2222` |
| 用户名 | `root` |
| 密码 | `orangepi` |

> 链路组成：小车 `frpc-ssh`（systemd 开机自启、断线自动重连）→ 服务器 `frps`。
> 配置细节见 `doc\frp内网穿透配置.md`。

### 3.2 方式二：热点局域网直连（备用）

```
[小车 OrangePi5] ——WiFi—— [手机热点] ——WiFi—— [本机 Windows]
   10.68.1.43                              (电脑与小车在同一热点下)
```

| 参数 | 值 |
|---|---|
| Host | `10.68.1.43` |
| 端口 | `22` |
| 用户名 | `root` |
| 密码 | `orangepi` |

**注意**：
- 热点 DHCP 分配的 IP **可能变化**。连不上时先让用户确认小车当前 IP（小车上 `hostname -I`，或热点后台看设备列表）。
- 手机热点通常**禁 ICMP**，`ping` 不通**不代表**连不上，要用 **TCP** 判断：
  ```powershell
  Test-NetConnection 10.68.1.43 -Port 22   # TcpTestSucceeded = True 即通
  Test-NetConnection 121.40.149.155 -Port 2222
  ```

---

## 4. 本机环境（Windows，已就绪）

| 项 | 情况 |
|---|---|
| Python | 3.11（`python` 可用） |
| paramiko | **已安装 5.0.0** |
| 助手脚本 | `E:\develop_software\yolov11\scripts\ssh_car.py`（执行命令）、`scripts\ssh_put.py`（传文件） |

若 paramiko 丢失/失效，重新安装：`python -m pip install paramiko`

---

## 5. 连接方法（推荐：助手脚本，免交互）

```powershell
# 默认走服务器转发(推荐)
python scripts/ssh_car.py uname -a
python scripts/ssh_car.py "uname -a; uptime"

# 隧道不通时,热点直连兜底
python scripts/ssh_car.py --direct hostname -I

# 命令超时(默认 30s)
python scripts/ssh_car.py --timeout 60 "ps aux | head -10"
```

⚠️ 命令参数以 `-` 开头时（如 `hostname -I`），请在命令前加 `--` 分隔：
```powershell
python scripts/ssh_car.py hostname -- -I
```

上传文件到小车：
```powershell
python scripts/ssh_put.py 本地文件 /root/远端路径          # 默认走隧道
python scripts/ssh_put.py --direct 本地文件 /root/...      # 热点直连
python scripts/ssh_put.py --server 本地文件 /opt/...       # 传给公网服务器
```

脚本内部逻辑：`paramiko.SSHClient` + `AutoAddPolicy`，用户名/密码硬编码在脚本顶部常量；输出 stdout、stderr、`[exit code: N]`。

---

## 6. 备选方案（paramiko 不可用时的降级路径）

1. **OpenSSH 本机客户端**（`C:\Windows\System32\OpenSSH\ssh.exe`）——**无法脚本化输密码**，只适合人工交互，不适合 AI 自动连接。
2. **plink（PuTTY）**——支持 `-pw` 传密码：
   ```powershell
   plink -ssh -pw orangepi -batch root@121.40.149.155 -P 2222 "uname -a"
   ```
   首次连接需先接受主机密钥（`echo y | plink ...`）。本机未安装，需要时从 `https://the.earth.li/~sgtatham/putty/latest/w64/plink.exe` 下载。
3. 原则上优先修好 paramiko，方案 1/2 不必真的用。

---

## 7. 注意事项（重要）

- ⚠️ **不要乱动小车上的正在运行的服务/进程**：实测负载 3~4、多个用户在线，说明有 ROS 等任务在跑，可能还有别的 AI/人在同时操作。只做只读检查，改配置前先记录原值。
- 小车 IP 可能因热点重启变化——隧道模式不受影响，这也是推荐它的原因。
- 小车是比赛/实验用车，SSH 只用于调试。
- ⚠️ **安全**：`root/orangepi` 是官方默认密码，且现已通过公网 2222 暴露——**强烈建议尽快改密码或启用密钥登录**；若改密码，需同步更新 `scripts\ssh_car.py`、`scripts\ssh_put.py` 顶部常量与本文件。

---

## 8. 快速验证清单（连接后跑什么）

```powershell
python scripts/ssh_car.py "uname -a; hostname; uptime"
# 期望：Linux orangepi5 ... aarch64；hostname=orangepi5；exit code 0
```

出现 `[exit code: 0]` 且内容合理 = 连接成功，可以开始干活。