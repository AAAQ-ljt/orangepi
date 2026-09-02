# 小车 SSH 连接手册（AI 自动连接用）

> 本文件供后续 AI（其他对话或智能体）查看后**自动连接小车**使用。
> 所有信息基于 2026 年实测验证，可放心直接操作。

---

## 1. 一句话总结

小车是 **Orange Pi 5（RK3588，Ubuntu 20.04）**，通过**手机热点局域网**可 SSH 登录：
`ssh root@10.68.1.43`，密码 `orangepi`（网络答对：Windows 本机已装 paramiko，直接用下面的脚本即可，无需交互输密码）。

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
- IP 是热点 DHCP 分配的，**可能变化**。连不上时先让用户确认小车当前 IP（可在小车上跑 `hostname -I`，或找热点后台看已连接设备）。
- 手机热点通常**禁 ICMP**，`ping` 不通**不代表**连不上，要用 **TCP 22 端口**判断：
  ```powershell
  Test-NetConnection 10.68.1.43 -Port 22   # TcpTestSucceeded = True 即通
  ```

---

## 4. 本机环境（Windows，已就绪）

| 项 | 情况 |
|---|---|
| Python | 3.10.10（pyenv-win 管理，命令 `python` 可用） |
| paramiko | **已安装 v5.0.0**（`python -c "import paramiko; print(paramiko.__version__)"` 验证） |
| 助手脚本 | `D:\5g\orangepi\dsh_ssh.py`（同目录还有一份备份在 `%TEMP%\dsh_ssh.py`） |

若 paramiko 丢失/失效，重新安装：
```powershell
python -m pip install paramiko
```

---

## 5. 连接方法（推荐：助手脚本）

脚本路径：**`D:\5g\orangepi\dsh_ssh.py`**（脚本内容见第 7 节，万一文件没了照着重建即可）。

基本用法：
```powershell
python D:\5g\orangepi\dsh_ssh.py "要执行的命令"
```

示例：
```powershell
# 探活 + 看系统
python D:\5g\orangepi\dsh_ssh.py "uname -a; uptime"

# 看进程
python D:\5g\orangepi\dsh_ssh.py "ps aux | head -30"
```

脚本内部逻辑（实现细节，供参考）：
- `paramiko.SSHClient` + `AutoAddPolicy`（自动接受首次主机密钥，无需交互）
- 用户名/密码硬编码在脚本顶部 `HOST/USER/PWD` 常量
- 可选第二个参数为命令超时秒数（默认 30）
- 输出：命令 stdout、stderr、`[exit code: N]`

---

## 6. 备选方案（paramiko 不可用时的降级路径）

1. **OpenSSH 本机客户端**（`C:\Windows\System32\OpenSSH\ssh.exe` 存在）——但**无法脚本化输密码**，只适合人工交互，不适合 AI 自动连接。
2. **plink（PuTTY）**——支持 `-pw` 传密码：
   ```powershell
   plink -ssh -pw orangepi -batch root@10.68.1.43 "uname -a"
   ```
   首次连接需先接受主机密钥（`echo y | plink ...`），`-batch` 模式下会自动拒绝未知密钥。本机未安装，需要时从 `https://the.earth.li/~sgtatham/putty/latest/w64/plink.exe` 下载（该源速度慢，耐心等）。
3. 原则上优先修好 paramiko（pip 装一次就行），方案 1/2 不必真的用。

---

## 7. 助手脚本全文（dsh_ssh.py）

```python
import sys
import paramiko

HOST = "10.68.1.43"
USER = "root"
PWD = "orangepi"


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "echo connected"
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(
        HOST,
        username=USER,
        password=PWD,
        timeout=15,
        banner_timeout=15,
        auth_timeout=15,
    )
    stdin, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    rc = stdout.channel.recv_exit_status()
    sys.stdout.write(out)
    if err:
        sys.stdout.write("[stderr]\n" + err)
    sys.stdout.write(f"\n[exit code: {rc}]\n")
    c.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
```

---

## 8. 注意事项（重要）

- ⚠️ **不要乱动小车上的正在运行的服务/进程**：实测负载 3~4、7 个用户在线，说明有 ROS 等任务在跑，可能还有别的 AI/人在同时操作。只做只读检查（查看、复制），改配置前先记录原值。
- 小车 IP 可能因热点重启变化，连不上先查 IP。
- 小车是比赛/实验用车（项目目录 `D:\5g\orangepi` 下有比赛资料、MaidKit 代码等），SSH 只用于调试，控制小车请走项目里现有的代码/协议。
- 密码 `orangepi` 是 Orange Pi 官方默认密码，安全性低——若担心被蹭网，建议用户之后改密码，但改完要同步更新本文件第 3/7 节。

---

## 9. 快速验证清单（连接后跑什么）

```powershell
python D:\5g\orangepi\dsh_ssh.py "uname -a; hostname; uptime"
# 期望：Linux orangepi5 ... aarch64；hostname=orangepi5；exit code 0
```

出现 `[exit code: 0]` 且内容合理 = 连接成功，可以开始干活。