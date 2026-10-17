# 驾驶模式（官方遥控网页的一个可选界面）

> 建立于 2026-10-16。给官方遥控网页（`http://121.40.149.155:8080/`）加一个**【驾驶模式】**按钮：
> 点进去把官方那堆控件换成干净的速度/手柄驾驶界面 + 参数配置区，**视频照常播、不刷新、不换地址**。
> 地址栏始终是 `http://121.40.149.155:8080/`。

---

## 0. 文件清单

| 文件 | 作用 |
|---|---|
| `rc-drive.js` | **主交付物**：单文件自包含（CSS + 面板 UI + 参数配置 + 手柄轮询 + 安全兜底），IIFE 隔离、变量/类名全带 `rcd` 前缀 |
| `deploy_rc.py` | 部署/还原：拉车端 `index.html` → 幂等插一行 → 上传（`--dry-run` / `--restore` / `--local-page`） |
| `mock_ws.py` | 本地假后端（**仅标准库**）：静态页 + 手写 WebSocket + 照抄车端语义的假状态机，不需要车和摄像头 |
| `index.html` / `opi-control-app.py` | 官方页与车端后端的**参考副本**（从车上取下来对照用；车端路径分别是 `/var/www/orangepi-rc/index.html`、`/root/opi-control/opi-control-app.py`） |
| `check_js.py` | `rc-drive.js` 静态自查：**用了但没声明的标识符** / 声明了没调用的函数 / `getElementById` 的 id 一致性（`node --check` 只查语法，抓不到「声明被注释吞掉」这类坑） |
| `../../dev/tests/test_web_deploy.py` | 补丁/还原的纯函数单测（幂等、逐字节还原、真官方页往返） |

---

## 1. 部署与还原

```bash
# 预览（会尽量拉车端文件核对；拉不到就用仓库副本预览，不会改任何东西）
python dev/web/deploy_rc.py --dry-run

# 部署：车端 mkdir -p /root/dev/backup-<日期> + cp -n 备份 → 本地插一行 → 上传 index.html + rc-drive.js
python dev/web/deploy_rc.py

# 还原：**就地剥掉补丁那两行**（拉当前页 → 删标记行 → 上传），只撤销我们加的东西
python dev/web/deploy_rc.py --restore

# 还原的另一种口径：改用 /root/dev/backup-*/index.html 里的原件
# ⚠️ 那些备份是不同日期的快照（实测 0915 那份 74875 字节、当前页 79018 字节），
#    拿旧的覆盖会把别人后来对官方页的改动一起回退掉 —— 用它会先打印差异
python dev/web/deploy_rc.py --restore --from-backup

# 完全离线：只改本地这份（本地联调用）
python dev/web/deploy_rc.py --local-page 某份/index.html
```

部署后的自检（plan 里的口径）：

```bash
curl -sI http://121.40.149.155:8080/                         # 期望 200
curl -s  http://121.40.149.155:8080/ | grep -c smartcar-rc-drive   # 期望 1
```

**只动 `/var/www/orangepi-rc/` 下的两个文件**，不碰 nginx / systemd / opi-control / frp。
⚠️ 这是往**官方资产**里插一行（AGENTS.md §1.2/§9）：执行前需队长点头，有一键还原。
⚠️ 官方若跑一次 `sync_to_car.py`，会用它的镜像覆盖掉这一行 → **重跑 `deploy_rc.py` 即可**（补丁幂等）。

插入的内容就是两行（在最后一个 `</body>` 之前）：

```html
<!-- smartcar-rc-drive: 驾驶模式（由 dev/web/deploy_rc.py 注入，勿手改） -->
<script src="/rc-drive.js"></script>
```

### 1.1 车端事实核对（2026-10-16 只读实测）

| 项 | 实测结果 |
|---|---|
| 车端页面 | `/var/www/orangepi-rc/index.html`（**CRLF**），md5 `cfcfe14f54bf382b8d2169c8b009b65f`；本仓库 `dev/web/index.html` 是它的**内容副本** —— 仓库按 `.gitattributes` 的 `* text=auto eol=lf` 把它存成 LF，所以两边 md5 不同（要逐字节比就先都 `tr -d '\r'`）。`F:/smart_car/...` 那份是官方原版镜像（`cam_car0003`、舵机符号相反），**不是**车端当前页 |
| 车端后端 | **`/root/opi-control/app.py`**（不是 `opi-control-app.py`），md5 `3e17329f070c0601a78b37dcf824306e` —— 与 `dev/web/opi-control-app.py` 内容一致（同样受 LF 归一化影响）；由 `opi-control.service` 以 `uvicorn app:app --port 9001` 启动，nginx 反代到 `:8080`（所以页面同源 `/ws` 就是它） |
| 云台反转 | 实测 `app.py:98` 是 **`tilt = 180 - tilt`**（第 97 行注释写"云台水平反转（左右反向）"，**注释与代码不符**） |
| ★ 云台 pan/tilt 交叉接 | 2026-10-17 现场实测：本车 **`set_pan_angle()` 实际驱动上下、`set_tilt_angle()` 实际驱动左右**。旁证：官方页 `sendGimbalNow()` 就是无条件 `send({pan: tilt, tilt: pan})` —— 它一直在对调。本脚本默认 `gimbal.swapAxes: true` 照抄这个对调，云台方向即与官方页一致 |
| 是否已打补丁 | 否（`grep -c smartcar-rc-drive` = 0）；车端已有 `/root/dev/backup-20260915|20261011|20261015/`，`deploy_rc.py` 会再建 `backup-<今天>` |
| 部署前置 | `opi-control.service` 当前是 **inactive**（`car-mode.sh status` 显示"未知/空闲"）——此时页面能进驾驶模式，但状态条会显示 `WS: 未连接`（**没有后端**）。要端到端验，先让车进入手动遥控模式（`bash /root/dev/scripts/car-mode.sh manual`） |


---

## 2. 本地联调（不需要车 / 摄像头）

```bash
python dev/web/mock_ws.py                      # 用仓库里的官方页副本，默认 127.0.0.1:8090
python dev/web/mock_ws.py --page F:/smart_car/newCarDOC/orangepiCode/var/www/orangepi-rc/index.html
python dev/web/mock_ws.py --verbose-drive      # 每一帧 drive 都打印
```

它把官方页在**运行时**注入那行 script（不改原文件），并提供一个假 `/ws`：状态机照抄车端
`opi-control-app.py`（arm 要求油门为 0、estop 挡解锁、drive 未解锁报错、drive 里 configure 行程、
gimbal 的 `tilt = 180 - tilt`、8s failsafe、5Hz telemetry），收到的消息都打印出来。
浏览器打开 `http://127.0.0.1:8090/` 即可点【驾驶模式】逐项验证。

没有真手柄时，可以在页面控制台里塞一个假的来验手柄链路：

```js
window.__fakeGp = {
  id: "Fake Pad", index: 0, connected: true, mapping: "standard", timestamp: Date.now(),
  axes: [0, 0, 0, 0],                                   // 左摇杆X / 左摇杆Y / 右摇杆X / 右摇杆Y
  buttons: Array.from({length: 16}, () => ({pressed:false, value:0})),
};
navigator.getGamepads = () => [window.__fakeGp];         // 然后点官方【手柄控制】按钮
window.__fakeGp.axes[0] = 0.8;                            // 左摇杆右推
window.__fakeGp.buttons[7] = {pressed:true, value:1};      // RT 全按
window.__fakeGp.buttons[6] = {pressed:true, value:0.5};    // LT 半按
window.__fakeGp.axes[2] = 0.9; window.__fakeGp.axes[3] = -0.9;  // 右摇杆 → 云台
```

---

## 3. 协议对照（前端 → 车端）

| 消息 | 字段 | 车端行为（`opi-control-app.py`） |
|---|---|---|
| `drive` | `steering`、`steer_center`、`throttle`、`throttle_min_us`、`throttle_center_us`、`throttle_max_us`、`throttle_us` | 每帧 `configure_esc_range()` + `configure_steering_center()` 重配行程；`steering` 被后端钳到 `steer_center ±30`；**未解锁/急停中直接回 error，不执行** |
| `gimbal` | `pan`、`tilt` | `pan` 原样下发；**`tilt = 180 - tilt`**（源码如此） |
| `arm` | `enabled` | 急停中拒绝；**油门不为 0 时拒绝解锁**（回 `{"type":"error","msg":"estop active"}`） |
| `estop` / `clear_estop` | — | 急停＝强制上锁＋油门归零＋方向回中 |
| `ping` | — | 回 ack，并刷新后端 8s failsafe 看门狗时间戳 |

前端**不新建 WebSocket**：沿用官方那一个（`getWsUrl()` = 同源 `/ws`），只 `addEventListener("message")`
读遥测（不覆盖官方 `onmessage`）；官方 ws 每次重连都是新对象，所以每 250ms 检查一次、需要就重新挂监听。

---

## 4. 参数配置区

| 参数 | 默认 | 范围 | 说明 |
|---|---|---|---|
| 转向满舵角度（灵敏度） | 30° | 5~30 | 摇杆推到底相对中值的偏转量。30° = 官方 `humanToServo` 的 0.3 系数，也是后端软限位上限 |
| 舵机中值 `steer_center` | 90 | 60~120 | 后端软限位 = 中值 ±30 |
| 油门最小值 | 1300us | 1200~1499 | 倒车方向的最大脉宽 |
| 油门中值 | 1500us | 1450~1550 | 停止脉宽 |
| 油门最大值 | 1600us | 1501~1600 | **硬钳 1600**（AGENTS.md §1.1.3：未经验证不得改成 2000），输入框和代码双保险 |
| 云台速度 | 2 单位/帧 | 1~6 | 右摇杆每帧的云台角度增量 |

「参数配置」里还有一组**手柄映射**（对不上就改号，改完点【保存】，不用改代码）：

| 字段 | 默认 | 说明 |
|---|---|---|
| 转向轴 | 0 | 左摇杆 X |
| 前进键 RT | 7 | 扳机值 > 0.5 算按下 |
| 倒车键 LT | 6 | 同上 |
| 云台水平轴 | 2 | 右摇杆 X |
| 云台俯仰轴 | 3 | 右摇杆 Y |

另有一组**方向反向**勾选框（现场"推的方向和动作相反"时勾，改完点【保存】）：

| 勾选框 | 什么时候勾 | 本车默认 |
|---|---|---|
| 转向反向 | 推摇杆往右、车却往左转 | 不勾 |
| 云台左右/上下对调 | 推右摇杆左右、云台却上下动 | **勾上**（本车 pan/tilt 交叉接） |
| 云台水平反向 | 推右摇杆往右、云台却往左 | 不勾 |
| 云台俯仰反向 | 推右摇杆往上、云台却往下 | 不勾 |

编号范围 0~31（浏览器 Gamepad 的口径，**轴和键是两套独立编号**）；越界会被钳到 31 并在日志里说明；
`RT` 与 `LT` 撞号、转向轴与云台轴撞号会**在日志里告警**（撞号时现场必然有一个功能不动）。
映射会一起存进 `rc-drive-config`，刷新后还在；语义相同的三个（转向轴/前进键/后退键）也会同步进官方对应输入框。

- 校验：**min < 中值 < max**，冲突时以中值为锚把两端拉回合法区间；保存时钳位并在面板日志里说明（如 `钳位：throttleMaxUs 1700→1600`）。
- 保存：写 `localStorage["rc-drive-config"]`（**与官方 `rc-ui-config` 分开**），并**尽力同步**进官方四个输入框
  （`steerCenterInput`/`throttleMinUsInput`/`throttleCenterUsInput`/`throttleMaxUsInput`）+ 官方 `CONFIG` 对象，
  避免同页两套 UI 打架。官方 `saveConfig()` 自己引用了一个不存在的 `#controlHostInput`（会抛错），所以**不调它**。
- 另有【恢复默认】。
- **实时死区提示**：按当前三个 us 参数算"油门要推多少车才起步"，随参数即时刷新。
  前进按实测的 **1545us** 起转；后退按 1500 对称假设的 **1455us**（**未实测**，README 与界面都标注了）。
  默认 1500/1600/1300 下 = 前进 >45%、后退 >23%。

---

## 5. 手柄映射

常量都在 `rc-drive.js` 顶部的 `RCD_CONFIG` 里：

```js
axes: { steer: 0, rt: 7, lt: 6, pan: 2, tilt: 3 },   // 左摇杆X / RT / LT / 右摇杆X / 右摇杆Y
deadzone: 0.12,                                       // 摇杆死区（云台用两倍死区）
gimbalRate: 2,                                        // 云台每帧角度增量
sendIntervalMs: 80,                                   // 12.5Hz 下发
```

- **模式写死为"左摇杆转向 + LT/RT 油门"**，不做模式切换：`throttle = RT.value - LT.value`，LT=倒车、RT=前进。
- 转向：`steering = steer_center - x * fullLockDeg`（与官方 `humanToServo` 同号：摇杆右推 = 角度减小 = 右）。
  左右反了就把 `steerInvert` 改 `true`。
- 云台：右摇杆 X → pan、Y → tilt（上推 = 角度增大），每帧最多动 `gimbalRate`，只在变化时下发。

### 手柄开关用官方那个按钮（唯一有技术风险的点）

官方按钮是 `onclick="toggleGamepadControl()"`，经典脚本里 `function` 声明会挂到 `window`，所以我们
**运行时替换 `window.toggleGamepadControl`**：点官方按钮启动/停止**我们的**轮询，官方 `startGamepadLoop` 永不启动
（已实测 `gamepadLoopRunning === false`）。同时：

1. 进入驾驶模式时把官方 `gamepadEnabled` **强制置真**，让官方"松手回中 / `sendDriveNow`"走它自己的
   `if (gamepadEnabled) return;` 提前返回，不会拿官方滑条的 0 值打掉我们的油门；
2. 再额外包装 `sendDriveNow` / `resetSteeringToCenter` / `resetThrottleToCenter` 做悬停保险
   （万一 `gamepadEnabled` 赋值失败，这三层也拦得住）；
3. 官方按钮文字由我们维护（官方 `updateGamepadButton()` 会按它自己的 flag 写"开"，与我们开关状态无关）；
4. 退出时**原样还原**替换掉的函数、官方 flag 与按钮文字。

`gamepadEnabled` 是 `let` 全局**词法绑定**（不是 `window` 属性），所以只能用间接 `eval` 按名字读写；
所有接管点都有 `typeof` + `try/catch` 守卫，官方页若改内部结构就退化成"自带按钮 + 打日志"，不会搞崩整页。

---

## 6. 安全边界（一条没删）

- 油门最大值**硬钳 1600us**：输入框 `max` + 保存钳位 + 每帧下发前再 `Math.min` 一次。
- `min < 中值 < max` 校验；舵机角度再按 `中值 ±30` 钳一次（与后端软限位一致）。
- 手柄死区 0.12（云台 0.24），防止摇杆松弛导致车自己走。
- **500ms ping 心跳**（驾驶模式下由本面板再补一条，官方那条也在跑）→ 不触发后端 8s failsafe。
- WS 断开：官方自带 3s 重连；若官方 `connectWS` 不可见，面板自己每 3s 补一次并打日志。
- **退出驾驶模式先强制油门归零 + 方向回中并补发一帧**，否则藏掉面板后车会保持原油门直到 8s failsafe。
- 手柄**断开**即自动关闭手柄控制并把油门归零（已实测：80% → 强制 0 帧）。
- 解锁失败时把后端那句含糊的 `estop active` 翻译成"先点【上锁】归零油门再解锁"。
- 重复的后端报错（未解锁时 12.5Hz 回 `system not armed`）只记一次，不刷屏。
- 面板自身不驱动电机：它只是把滑条/手柄变成 `drive`/`gimbal` 帧，**发车仍由官方【解锁】+ 后端 arm 门控**。

---

## 7. 与官方页的耦合点（官方改版后要复核这几处）

- `window.toggleGamepadControl` / 全局 `let gamepadEnabled` / `updateGamepadButton` / `arm`
- 全局 `ws` 实例（只加监听，不覆盖 `onmessage`）
- 选择器：`.top-right`、`.top-left`、`#throttleWidget`、`#steeringWidget`、`#gimbalWidget`、`.log-box`、
  `#configPanel`、`#configBtn`、`#gamepadBtn`、`#videoElement`
- 官方四个配置输入框 id（见 §4）

进入驾驶模式只**隐藏**（先记原 `style` 属性，退出原样还原，含设置面板开合状态）：`.top-left`、
三个 `.floating-widget`、`.log-box`、`#configPanel` + `#configBtn`。
**保留官方 `.top-right` 整行**（解锁/上锁/急停/清除急停/打开声音/摄像头切换/手柄控制/音响/按住说话），
`#videoElement` 全程可见、不重连。

面板自己的构成：顶部状态条（WS/armed/estop/throttle/steering，读官方那条 WS 的 telemetry）、油门竖直滑条
（读数含 us）、方向水平滑条（读数含舵机角度）、可折叠**参数配置区**、可折叠**手柄调试区**、**可滚动日志**。

---

## 7.4 面板摆放（挡视线就挪走 / 收起来）

- **拖动**：按住面板**标题栏**（「驾驶模式」那一行，鼠标会变抓手）拖到任意位置。位置存
  `localStorage["rc-drive-pos"]`（**单独一个键**，清参数不影响摆位），下次进驾驶模式还放那儿。
  拖动会被夹住——**至少留 60px 在屏幕里**，拖到屏幕外也不会"弄丢"；窗口缩小后会自动夹回可见范围。
- **双击标题栏** = 位置复位到左下角（并清掉存的位置），省得手动拖回角落。
- **收起**：标题栏右边的【收起 ◀】把面板整个藏起来，**只在左边缘留一条半透明细拉手**
  （竖排「驾驶模式」，默认 32% 不透明度，鼠标移上去才变亮——就是为了不挡视线又不至于找不着）。
  点它面板回到原来的位置。
- 收起**只影响界面**：手柄控制、心跳、日志记录照跑（收起来的只是面板和"手柄调试"的实时刷新）。
  想继续看日志就别收，或者收之前先扫一眼。
- **每次进驾驶模式都从"展开"开始**（位置沿用上次），避免有人收起来后忘了、以为驾驶模式坏了。
- 退出驾驶模式时面板和拉手**都**收掉（不留任何残留元素挡画面）。

---

## 7.5 手柄调试区 + 日志（现场先确认"按键匹配"，再看"我做了什么"）

### 手柄调试区

面板里「手柄调试（实时读数 / 先确认按键匹配）」展开即可，**不需要先开【手柄控制】**——现场先拨一下摇杆、
按一下扳机，确认它到底是几号轴/几号键，再决定要不要开控制：

- **映射行**：把当前配置翻成人话（`转向 = 左摇杆 axes[0] ｜ 油门 = RT buttons[7] 前进 / LT buttons[6] 倒车 …`）；
- **最后变化行**：`最后变化：buttons[7] = 1.000（RT前进）`——拨哪个就显示哪个，没映射的写"（未映射）"；
- **轴/键格子**：全部列出来，**已映射的绿底带角色名**，当前有值的蓝底，**刚动过的那个黄底**（保持 1.5s）；
- **没有手柄时它会直接告诉你为什么**（展开调试区时也会在日志里打一条）：
  - 页面没焦点 → 「先点一下页面/视频区域拿到焦点，再按手柄任意键或摇一下摇杆」。
    ⚠️ **Chromium 的规矩：只有在「页面有焦点时按过手柄键」之后，网页才拿得到手柄** ——
    手柄在 Windows 里连着、别的程序也能用，不等于网页能看到它；
  - 有焦点但 `getGamepads()` 还是空槽 → 按 Xbox 键唤醒 / 查是不是被 Steam、DS4Windows 之类独占了 / 换 USB 口。
- 检测到手柄时会打一条 `检测到手柄：<id>（axes N / buttons M）`，轴键数量对得上就该能用了。

对不上就直接改「参数配置 → 手柄映射」里的 5 个号（改完点【保存】），改完实时读数里的绿底角色名会跟着变，
不用重新部署。**先在这里确认映射，再开【手柄控制】**，省得车一上来就按错键。

### 日志（"我用手柄做了什么"）

底部日志固定高度、可滚动、保留 60 行、带【清空】。**新日志会自动跟到最新一行**（不用手动拖滚动条），
但你手动往上翻看旧日志时它不会打断你 —— 只有滚动条本来就贴着底部时才跟随，拖回底部后继续跟随。
展开「参数配置 / 手柄调试」时，面板会自动把刚展开的那块带进视野（面板自己也是个滚动容器）。

它记**动作**而不是每帧数值：

```
21:55:36 [手柄] RT 前进 buttons[7] 按下
21:55:36 [手柄] 油门 前进100% → 1600us
21:55:36 [手柄] 油门 前进50% → 1550us       ← 油门按 25% 分档，捏一半不会刷屏
21:55:37 [手柄] 油门回零 → 1500us
21:55:37 [手柄] LT 倒车 buttons[6] 按下
21:55:37 [手柄] 油门 后退60% → 1380us
21:55:38 [手柄] 转向 axes[0] 动了 (0.80)     ← 摇杆从静止到动，把轴号报一次
21:55:38 [手柄] 转向 右80 → 舵机 66°
21:55:39 [手柄] 转向 axes[0] 回中
21:55:39 [手柄] 云台开始动（右摇杆）
21:55:40 [手柄] 云台停 水平 110 / 俯仰 110   ← 云台记"开始动/停"边沿，不按帧刷
```

规则：扳机按下/松开用**施密特阈值**（按下 ≥0.5、松开 <0.35，停在 0.5 不来回刷）；油门按
0 / 1~24% / 25~49% / 50~74% / 75~99% / 100% **分档**，换档才记；转向变化 ≥3° 且间隔 ≥250ms 才记；
云台只记"开始动 / 停（带最终角度）"。滑条操作记成 `[滑条] …`、手柄记成 `[手柄] …`，一眼分清是谁动的。
模拟量的细节看实时读数，日志负责"时间线上发生过什么"。节流参数都在 `RCD_CONFIG`（`actionGateMs` /
`actionStepPct` / `actionStepDeg` / `debugRenderMs` / `debugHoldMs` / `logLines`）。

---

## 8. 已知限制

1. 官方 `humanToServo` 的 **0.3 系数是硬编码**的：满舵角度只作用于我们的面板与我们的手柄轮询；
   切回官方滑条开车时灵敏度仍是官方的 30°。
2. **云台方向（2026-10-17 现场定案）**：本车 **pan/tilt 是交叉接的** —— `set_pan_angle()` 实际驱动上下、
   `set_tilt_angle()` 实际驱动左右。官方页 `sendGimbalNow()` 无条件 `send({pan: tilt, tilt: pan})` 就是在对调，
   本脚本默认 `gimbal.swapAxes: true` 照抄它，于是：
   - 推右摇杆**往右** → 只让 `tilt` 线增大 → 后端 `180 - tilt` 后 `set_tilt_angle` 参数减小 → **物理向右** ✓
   - 推右摇杆**往上** → 只让 `pan` 线增大 → `set_pan_angle` 参数增大 → **物理向上** ✓
   （反推依据：旧版把水平意图发到 `pan` 线时，现场看到"推右=上仰、推上=往左"，正好是交叉接的表现。）
   现场若还反，面板「参数配置 → 方向反向」里三个勾选框直接翻：`云台水平反向`（左右反）、`云台俯仰反向`（上下反）、
   `云台左右/上下对调`（推左右却上下动，本车默认勾）。
   注意 `pwm.set_pan_angle/set_tilt_angle` 内部若还有一层反转，只能到车上看实际动作才能定稿。
3. 官方页若被替换，隐藏选择器/接管点可能失配——每处都有存在性判断，缺了会退化成"入口按钮固定在右上角 +
   打日志"，不会搞崩整页。
4. **手柄轮询是 `requestAnimationFrame` 驱动的**（与官方一致）：浏览器标签页切到后台/隐藏时 rAF 会暂停，
   手柄帧随之停发 → 后端 8s failsafe 把油门归零（**这是想要的失效方向**）。所以手柄开车必须让标签页保持前台。
5. 退出后官方元素上可能留一个空的 `style=""`（Chromium 对"动过 style 对象再 removeAttribute"的行为），
   等价于没有内联样式，渲染结果与退出前一致（已实测计算样式完全一致），无功能影响。
6. **本次不做**：键盘 WASD/方向键控制、视频/音响/对讲改造（保持官方原样）。

---

## 9. 本地验证结论（2026-10-16，`mock_ws.py` + 浏览器实测）

已验证：官方原控件/滑条/状态条在进入前一切正常；入口按钮文字双向切换；进入后官方 7 个元素隐藏、
`.top-right` 整行与 `#videoElement` 保留；WS 沿用官方连接（未新建）；点官方【手柄控制】→ 标签变"开"、
官方 flag 置真、官方循环未启动；手柄左摇杆 → 转向（`90 - x*满舵`）、RT → +%、LT → -%、右摇杆 → 云台
（带速度限幅、下发值带 tilt 反转补偿）；驾驶模式期间调用官方回中/`sendDriveNow` **0 帧**；
参数保存到独立 key、越界被钳（1700→1600）、死区提示随参数变、官方输入框同步；刷新后参数仍在；
滑条拖动出帧、松手回中并补发；退出→官方控件与函数原样还原（退出后 `sendDriveNow` 又能发帧）、
收到 throttle=0 强制帧；**反复进出 5 次**仍只有 1 个入口按钮/1 个面板/1 个 style 标签；
手柄断开自动断油。

实测中还修掉两个真 bug（都出在"第二次进入"这类路径上，靠反复进出才暴露）：
`storageKey` 漏写导致参数写进 `localStorage["undefined"]`（界面照样说"已保存"）；
`rcdEnter()` 没把面板 `display` 还原导致第二次进来看不到面板（只有按钮文字变）。
为此在 `RCD_CONFIG` 上加了启动自检（缺键直接报错）。

---

## 10. 改完 rc-drive.js 必跑的自查

```bash
python dev/web/check_js.py     # 语法 + 未声明标识符 + 未调用函数 + getElementById 的 id 一致性
```

为什么单靠 `node --check` 不够：2026-10-16 一次编辑把换行吃掉了，
`const RCD_SOFT_LIMIT_DEG = 30;` 被并进上一行注释里 → 它**根本没被声明**，
而 `node --check` 只看语法，照样通过。症状是「点进驾驶模式后日志一片空白、手柄一个包都发不出去」
（`rcdSetSteerFromHuman` / `rcdMapDrive` 里用到它，每帧抛 ReferenceError）。
`check_js.py` 就是补这一刀的；改完 JS 记得跑（AGENTS.md §4.4 已把它列进提交前自查）。
