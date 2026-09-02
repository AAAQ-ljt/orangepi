# FRP 内网穿透配置 — 小车 SSH 转发（服务器端 121.40.149.155）

> 用途：通过公网服务器 `121.40.149.155` 中转，稳定访问小车内网 SSH（手机热点局域网 IP 经常变化的问题）。
> 客户端（小车上）已配置完毕：`/root/frp/frpc-ssh.ini` + systemd 服务 `frpc-ssh`（开机自启、断线自动重连）。
> 版本：frp **0.39.0**（小车端就是这个版本，服务器端必须用同版本，协议最稳）。

---

## 0. 参数速查（两边必须一致）

| 参数 | 值 |
|---|---|
| 服务器地址 | `121.40.149.155` |
| 控制端口 bind_port | `7000` |
| SSH 转发端口 remote_port | `2222` |
| 认证 token | `375b12b2e5614357ce8225bcce93df4e` |
| 小车端用户 | `root` / `orangepi` |

连接效果：`ssh -p 2222 root@121.40.149.155`（任何能上网的机器）就能进小车。

---

## ⚠️ 安装顺序坑（实测踩过）

解压官方 release 包（`tar xzf ... -C /opt/frp`）会**覆盖同名的 `frps.ini`**（包里自带的默认配置只有 `bind_port = 7000`，没有 token）。
**正确顺序**：先解压，**再**写入/上传自定义 `frps.ini`。部署后务必检查：`grep token /opt/frp/frps.ini` 有输出才算对，否则小车端会报 `token in login doesn't match`。

## 1. 服务器安装 frps（Linux x86_64，建议 /opt）

```bash
cd /opt
wget https://github.com/fatedier/frp/releases/download/v0.39.0/frp_0.39.0_linux_amd64.tar.gz
tar xzf frp_0.39.0_linux_amd64.tar.gz
mv frp_0.39.0_linux_amd64 frp
/opt/frp/frps --version     # 应输出 0.39.0
```

> GitHub 下载慢时，可用加速前缀（如 `https://ghproxy.com/https://github.com/...`）或找镜像源，版本必须 0.39.0。

## 2. 服务端配置 `/opt/frp/frps.ini`

```ini
[common]
bind_port = 7000
token = 375b12b2e5614357ce8225bcce93df4e

# ---- 可选：Web 管理面板 ----
# dashboard_port = 7500
# dashboard_user = admin
# dashboard_pwd = 换成你自己的强密码
```

## 3. systemd 开机自启 `/etc/systemd/system/frps.service`

```ini
[Unit]
Description=FRP Server Service
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/opt/frp/frps -c /opt/frp/frps.ini
WorkingDirectory=/opt/frp
Restart=always
RestartSec=5
User=root

[Install]
WantedBy=multi-user.target
```

```bash
systemctl daemon-reload
systemctl enable --now frps
systemctl status frps --no-pager | head -10
```

## 4. 防火墙（两层都要放行！）

**① 服务器系统防火墙**（ufw / firewalld）：
```bash
ufw allow 7000/tcp && ufw allow 2222/tcp    # 或 firewall-cmd --permanent --add-port=7000/tcp ...
```

**② 阿里云安全组（网页控制台）**——★最容易漏的一步：
- 实例 → 安全组 → 入方向规则 → 添加：
  - 协议 TCP / 端口 **7000** / 授权对象 `0.0.0.0/0`（控制通道，也可限定自己 IP）
  - 协议 TCP / 端口 **2222** / 授权对象 `0.0.0.0/0`（SSH 通道，**建议先限定成自己/学校出口 IP**，用宽带不固定就先全放行、尽快改密码）

## 5. 验证（三步）

1. 服务器看监听：`ss -lntp | grep frps` → 应见 `:7000`；小车连上后多出 `:2222`
2. 小车看日志（我这边可代查）：`journalctl -u frpc-ssh -n 5` → 出现 `start proxy success` 即成功
3. 随便一台机器：`ssh -p 2222 root@121.40.149.155`，密码 `orangepi`

## 6. ⚠️ 安全提醒（转发出去就是公网暴露）

- 转发后 `root/orangepi`（官方默认密码）将暴露在公网，**尽快执行二选一**：
  - 改小车密码（车上 `passwd`），改完同步更新 `scripts/ssh_car.py` 与本项目手册；或
  - 启用 SSH 密钥登录并关闭密码登录
- 安全组 2222 端口尽量限定来源 IP；控制端口 7000 同理（只允许小车连入）。
- 若 2222 与服务器已有服务冲突，换端口即可——两边 `remote_port` / 安全组同步改。

## 7. 故障排查速查

| 现象 | 原因 |
|---|---|
| 小车日志 `login to server failed` | 服务器 frps 没起 / 7000 未放行 / token 不一致 |
| 客户端显示成功但 `ssh -p 2222` 连不上 | 2222 未在安全组放行 / 2222 被服务器占用 |
| `connect: connection refused`（服务器侧） | frps 未启动或端口没监听 |

## 8. 文件清单（本仓库留存，可复现）

- 客户端配置：`scripts/car-frp/frpc-ssh.ini`（车上 `/root/frp/frpc-ssh.ini`）
- 客户端服务：`scripts/car-frp/frpc-ssh.service`（车上 `/etc/systemd/system/frpc-ssh.service`）
- 服务端配置：`scripts/server-frp/frps.ini`（服务器 `/opt/frp/frps.ini`）、`scripts/server-frp/frps.service`（`/etc/systemd/system/frps.service`）
- 下载脚本：`scripts/server-frp/download_frps.sh`（镜像源依次尝试，最后直连 GitHub）
- 本地助手：`scripts/ssh_car.py`（执行命令，隧道/直连双模式）、`scripts/ssh_put.py`（传文件）、`scripts/ssh_server.py`（操作服务器）
- 官方 frpc（连 82.157.204.126 的网页/图传服务）:**保持原样未动**

## 9. 部署与验证记录（2026-09-02 实测）

| 项 | 结果 |
|---|---|
| 服务器 frps 0.39.0（Ubuntu 26.04 x86_64） | ✅ `/opt/frp/frps`，systemd `frps` active，监听 `*:7000` |
| 小车 frpc-ssh（frp 0.39.0 ARM64，复用官方二进制） | ✅ systemd `frpc-ssh` enabled，日志 `login to server success` + `[ssh] start proxy success` |
| 服务器本机隧道连通性（`/dev/tcp/127.0.0.1/2222`） | ✅ TUNNEL-OK（穿透到达小车 sshd） |
| 公网实测 `ssh -p 2222 root@121.40.149.155` | ✅ 登录成功，落到小车（orangepi5 / aarch64） |
| 阿里云安全组 | ✅ 已放行 7000 + 2222 入方向 |
| 踩坑记录 | 解压 release 包覆盖 frps.ini 导致 token 丢失（见顶部警告） |