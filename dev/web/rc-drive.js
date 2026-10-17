/*!
 * rc-drive.js —— 官方遥控网页（orangepi-rc/index.html）的「驾驶模式」补丁
 * ---------------------------------------------------------------------------
 * 单文件自包含：CSS + 面板 UI + 参数配置 + 手柄轮询 + 安全兜底，全部在这个 IIFE 里。
 * 由 dev/web/deploy_rc.py 在官方 index.html 的 </body> 前插一行 <script src="/rc-drive.js"> 引入；
 * 页面地址、视频（#videoElement）、WebSocket 全部沿用官方那套，本脚本不新建连接、不换地址。
 *
 * 设计要点（细节见 dev/web/README.md）：
 *   1. 入口是注入官方 .top-right 的一个按钮，文字在「驾驶模式」/「返回主页面」间切换。
 *   2. 进入后只隐藏官方控件（先记 style 原值，退出原样还原，含设置面板开合状态）。
 *   3. 手柄开关**复用官方那个按钮**（#gamepadBtn）：运行时替换 window.toggleGamepadControl，
 *      官方 startGamepadLoop 永不启动；同时把官方 gamepadEnabled 置真，让官方
 *      「松手回中 / sendDriveNow」走它自己的 `if (gamepadEnabled) return;` 提前返回，
 *      不会拿官方滑条的 0 值打掉我们的油门。
 *   4. 退出时把替换掉的函数原样还原。所有接管点都有 typeof + try/catch 守卫：
 *      官方页若改内部结构，退化成自带按钮 + 打日志，不会搞崩整页。
 *   5. 安全（AGENTS.md §1.1）：油门最大值硬钳 1600us；min<中值<max 校验；死区 0.12；
 *      500ms ping 心跳；退出驾驶模式先强制油门归零 + 方向回中并补发一帧。
 *
 * 与官方页面耦合的点（官方改版后需要复核）：
 *   · window.toggleGamepadControl / gamepadEnabled / updateGamepadButton / arm
 *   · 全局 ws 实例（只挂 addEventListener，不覆盖官方 onmessage）
 *   · 选择器：#gamepadBtn、.top-right、.top-left、#throttleWidget、#steeringWidget、
 *             #gimbalWidget、.log-box、#configPanel、#configBtn、#videoElement
 *   · 官方四个配置输入框：steerCenterInput / throttleMinUsInput /
 *             throttleCenterUsInput / throttleMaxUsInput
 *
 * ⚠️ 云台方向：车端 opi-control-app.py 的 set_gimbal() 里写的是 `tilt = 180 - tilt`
 *    （源码如此；旧说法"pan = 180 - pan"与源码不符）。本脚本用 RCD_CONFIG.gimbal 的
 *    panInvert / tiltInvert 两个常量做补偿，默认 tiltInvert=true 抵消后端那次反转。
 *    现场若发现云台左右/上下反了，翻对应常量即可（README「云台方向」一节）。
 */
(function () {
  "use strict";

  /* ======================================================================
   * 1. 默认值常量块（现场要调的东西都在这里；配置区保存的是这些键的覆盖值）
   * ==================================================================== */
  const RCD_CONFIG = {
    version: "1.0.0",
    storageKey: "rc-drive-config", // 与官方 rc-ui-config 分开存，互不覆盖
    posKey: "rc-drive-pos", // 面板拖到哪儿了（单独一个键，清配置不影响摆位）

    /* —— 转向 —— */
    fullLockDeg: 30, // 满舵角度（灵敏度）：摇杆推到底时相对中值的偏转量。5~30
    steerCenter: 90, // 舵机中值。60~120（后端软限位 = 中值 ±30）
    steerInvert: false, // 摇杆右推是否让舵机角度减小（官方 humanToServo 约定：右 = 角度减小）

    /* —— 油门（脉宽 us）—— */
    throttleMinUs: 1300, // 后退最大脉宽。1200~1499
    throttleCenterUs: 1500, // 停止中位脉宽。1450~1550
    throttleMaxUs: 1600, // 前进最大脉宽。1501~1600（硬钳）

    /* —— 电调死区（AGENTS.md §1.1.3 实测：1500 停、≈1545 才起转）—— */
    escStartUs: 1545, // 前进起转脉宽（实测）
    escStartRevUs: 1455, // 后退起转脉宽（按 1500 对称假设，**未实测**）

    /* —— 手柄 —— */
    deadzone: 0.12, // 摇杆死区（与官方 GAMEPAD_DEADZONE 同口径）
    axes: { steer: 0, rt: 7, lt: 6, pan: 2, tilt: 3 }, // 左摇杆X / RT / LT / 右摇杆X / 右摇杆Y
    gimbalRate: 2, // 云台速度：每帧角度增量。1~6
    sendIntervalMs: 80, // 下发节流：80ms ≈ 12.5Hz（与官方一致）

    /* —— 云台 —— */
    gimbal: {
      center: 90,
      min: 0,
      max: 180,
      swapAxes: true, // ★ 本车 pan/tilt 交叉接（官方页也一直对调），默认勾上；换车/换接法才需要关
      panInvert: false, // 左右反了 → 勾 true（面板「云台水平反向」）
      tiltInvert: false, // 上下反了 → 勾 true（面板「云台俯仰反向」）
    },

    /* —— 网络 / UI —— */
    pingMs: 500, // 心跳
    uiTickMs: 250, // 状态条刷新
    reconnectMs: 3000, // WS 断开重连
    logLines: 60, // 日志保留多少行（显示区固定高、可滚动，所以留够回看）
    actionGateMs: 250, // 动作日志最小间隔（同一类动作别刷屏）
    actionStepPct: 5, // 油门变化多少个百分点才记一条
    actionStepDeg: 3, // 转向变化多少度才记一条
    debugRenderMs: 100, // 手柄调试读数刷新间隔
    debugHoldMs: 1500, // "刚动过的那个"高亮保持多久
    panelId: "rcd-panel",
    marker: "smartcar-rc-drive",
  };

  /* 配置区输入范围（同时是保存时的钳位范围，硬边界不允许越过） */
  const RCD_FIELDS = [
    {
      key: "fullLockDeg",
      label: "满舵角度（灵敏度）",
      unit: "°",
      min: 5,
      max: 30,
      step: 1,
      hint: "摇杆推到底相对中值的偏转量。30° = 官方 0.3 系数，也是后端软限位上限。",
    },
    { key: "steerCenter", label: "舵机中值", unit: "°", min: 60, max: 120, step: 1, hint: "后端软限位 = 中值 ±30。" },
    { key: "throttleMinUs", label: "油门最小值", unit: "us", min: 1200, max: 1499, step: 1, hint: "倒车方向的最大脉宽。" },
    { key: "throttleCenterUs", label: "油门中值", unit: "us", min: 1450, max: 1550, step: 1, hint: "停止脉宽。" },
    {
      key: "throttleMaxUs",
      label: "油门最大值",
      unit: "us",
      min: 1501,
      max: 1600,
      step: 1,
      hint: "硬钳 1600（AGENTS.md §1.1.3：未经验证不得改成 2000）。",
    },
    { key: "gimbalRate", label: "云台速度", unit: "单位/帧", min: 1, max: 6, step: 1, hint: "右摇杆每帧的云台角度增量。" },
  ];

  /* 手柄映射：现场发现按键/轴对不上，直接在这儿改号（改完点【保存】），不用去改代码。
     0~31 是浏览器 Gamepad 的编号范围；轴和键是两套独立编号，别混。 */
  const RCD_AXIS_FIELDS = [
    { key: "steer", label: "转向轴", hint: "默认 0（左摇杆X）" },
    { key: "rt", label: "前进键 RT", hint: "默认 7" },
    { key: "lt", label: "倒车键 LT", hint: "默认 6" },
    { key: "pan", label: "云台水平轴", hint: "默认 2（右摇杆X）" },
    { key: "tilt", label: "云台俯仰轴", hint: "默认 3（右摇杆Y）" },
  ];
  const RCD_AXIS_MIN = 0;
  const RCD_AXIS_MAX = 31;

  /* 云台方向：现场"推的方向和动作相反"时改这几个，改完点【保存】。
     swapAxes 默认勾上 —— 本车 pan/tilt 交叉接（官方页 sendGimbalNow 也一直对调）。 */
  const RCD_INVERT_FIELDS = [
    { id: "rcd-cfg-gimbal-swap", label: "云台左右/上下对调（本车默认勾）", key: "swapAxes", hint: "" },
    { id: "rcd-cfg-inv-pan", label: "云台水平反向", key: "panInvert", hint: "" },
    { id: "rcd-cfg-inv-tilt", label: "云台俯仰反向", key: "tiltInvert", hint: "" },
  ];

  /* 转向反向是独立的一项（不属于云台） */
  const RCD_STEER_INVERT = { id: "rcd-cfg-inv-steer", label: "转向反向", hint: "推摇杆往右、车却往左转" };

  const RCD_HARD_MAX_US = 1600; // ★ 油门最大值硬钳，任何路径都不许超过
  const RCD_SOFT_LIMIT_DEG = 30; // 后端 set_steering 的软限位：中值 ±30

  /* 启动自检：RCD_CONFIG 少一个键，症状会非常隐蔽
     （比如 storageKey 忘了写 → setItem(undefined) → 存进 localStorage["undefined"]，
       界面照样说"已保存"）。所以这里显式点一遍。 */
  const RCD_REQUIRED_KEYS = [
    "storageKey", "posKey", "panelId", "marker", "fullLockDeg", "steerCenter", "steerInvert",
    "throttleMinUs", "throttleCenterUs", "throttleMaxUs", "deadzone", "axes",
    "gimbalRate", "sendIntervalMs", "gimbal", "pingMs", "uiTickMs", "reconnectMs", "logLines",
    "actionGateMs", "actionStepPct", "actionStepDeg", "debugRenderMs", "debugHoldMs",
  ];

  function rcdSelfCheck() {
    const missing = RCD_REQUIRED_KEYS.filter((k) => RCD_CONFIG[k] === undefined);
    if (!missing.length) return true;
    const msg = "★ 内部配置缺键（补丁写错了，功能会静默失效）: " + missing.join("、");
    if (window.console && console.error) console.error("[rcd] " + msg);
    rcdLog(msg);
    return false;
  }

  /* 官方选择器：进入驾驶模式要藏起来的东西（退出原样还原） */
  const RCD_HIDE_SELECTORS = [
    ".top-left", // 官方状态条（我们的面板自带）
    "#throttleWidget", // 官方油门滑条
    "#steeringWidget", // 官方方向滑条
    "#gimbalWidget", // 官方云台滑条
    ".log-box", // 官方日志框
    "#configPanel", // 官方设置面板
    "#configBtn", // 官方「设置」按钮
  ];

  /* ======================================================================
   * 2. 小工具
   * ==================================================================== */
  const $ = (sel, root) => (root || document).querySelector(sel);
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const isNum = (v) => typeof v === "number" && isFinite(v);

  function toInt(v, dflt) {
    const n = parseInt(v, 10);
    return Number.isFinite(n) ? n : dflt;
  }

  /* 安全地取官方全局 `let` 绑定（ws / gamepadEnabled / CONFIG / gamepadLoopRunning）。
     `typeof x` 对未声明的标识符也安全；对已声明的绑定则读到真实值。 */
  function rcdGlobal(name, dflt) {
    try {
      // eslint-disable-next-line no-eval
      const v = (0, eval)(name); // 直接 eval：拿到全局词法绑定，而不是 window 属性
      return typeof v === "undefined" ? dflt : v;
    } catch (_) {
      return dflt;
    }
  }

  /* ======================================================================
   * 3. 运行期状态
   * ==================================================================== */
  const S = {
    active: false, // 是否处于驾驶模式
    gamepadOn: false, // 我们的手柄轮询开关
    loopRunning: false,
    hookInstalled: false,
    orig: {}, // 官方原函数（退出时还原）
    hidden: [], // [{el, style}] 被隐藏的官方元素
    prevOfficialFlag: false, // 进入前官方的 gamepadEnabled
    wsRef: null, // 已挂过监听的那个 ws 实例
    tel: { armed: false, estop: false, throttle: 0, steering: 90 },
    drive: { steering: 90, throttle: 0 },
    gimbal: { pan: 90, tilt: 90 },
    logs: [],
    lastSend: 0,
    lastPing: 0,
    lastTick: 0,
    lastReconnect: 0,
    lastGimbalKey: "",
    armIntent: null, // 最近一次点「解锁/上锁」的意图，用于把后端报错翻译成人话
    lastErrKey: "",
    lastErrAt: 0,
    cfg: null, // 生效配置
    collapsed: false, // 面板是否收起成左侧细拉手
    // 手柄调试读数
    gpDebugRunning: false,
    gpLastChange: null, // {kind:"axes"|"buttons", index, value, at}
    gpPrevAxes: [],
    gpPrevButtons: [],
    gpLastRender: 0,
    // 动作日志的"上次记了什么"，用来只在变化时记一条
    action: { thrBand: undefined, steer: null, steerAt: 0, gimbalMoving: undefined, axisOn: {}, btnOn: {} },
  };

  /* ======================================================================
   * 4. 配置读写（localStorage 键与官方 rc-ui-config 分开）
   * ==================================================================== */
  function rcdClampField(key, value) {
    const spec = RCD_FIELDS.filter((f) => f.key === key)[0];
    if (!spec) return value;
    let v = toInt(value, RCD_CONFIG[key]);
    v = clamp(v, spec.min, spec.max);
    if (key === "throttleMaxUs") v = Math.min(v, RCD_HARD_MAX_US); // 双保险
    return v;
  }

  function rcdClampAxisIndex(v, dflt) {
    const n = toInt(v, dflt);
    return clamp(n, RCD_AXIS_MIN, RCD_AXIS_MAX);
  }

  function rcdLoadConfig() {
    const cfg = Object.assign({}, RCD_CONFIG);
    cfg.axes = Object.assign({}, RCD_CONFIG.axes);
    cfg.gimbal = Object.assign({}, RCD_CONFIG.gimbal);
    let saved = null;
    try {
      saved = JSON.parse(localStorage.getItem(RCD_CONFIG.storageKey) || "null");
    } catch (_) {
      saved = null;
    }
    if (saved && typeof saved === "object") {
      RCD_FIELDS.forEach((f) => {
        if (saved[f.key] !== undefined) cfg[f.key] = rcdClampField(f.key, saved[f.key]);
      });
      if (saved.axes && typeof saved.axes === "object") {
        Object.keys(cfg.axes).forEach((k) => {
          if (saved.axes[k] !== undefined) cfg.axes[k] = rcdClampAxisIndex(saved.axes[k], cfg.axes[k]);
        });
      }
      if (typeof saved.steerInvert === "boolean") cfg.steerInvert = saved.steerInvert;
      if (saved.gimbal && typeof saved.gimbal === "object") {
        ["center", "min", "max"].forEach((k) => {
          if (isNum(saved.gimbal[k])) cfg.gimbal[k] = toInt(saved.gimbal[k], cfg.gimbal[k]);
        });
        ["swapAxes", "panInvert", "tiltInvert"].forEach((k) => {
          if (typeof saved.gimbal[k] === "boolean") cfg.gimbal[k] = saved.gimbal[k];
        });
      }
    }
    rcdFixThrottleOrder(cfg);
    return cfg;
  }

  /* 保证 min < 中值 < max：冲突时以「中值」为锚，把越界的两端拉回合法区间。 */
  function rcdFixThrottleOrder(cfg) {
    cfg.throttleCenterUs = rcdClampField("throttleCenterUs", cfg.throttleCenterUs);
    cfg.throttleMinUs = rcdClampField("throttleMinUs", cfg.throttleMinUs);
    cfg.throttleMaxUs = rcdClampField("throttleMaxUs", cfg.throttleMaxUs);
    if (cfg.throttleMinUs >= cfg.throttleCenterUs) {
      cfg.throttleMinUs = cfg.throttleCenterUs - 1;
    }
    if (cfg.throttleMaxUs <= cfg.throttleCenterUs) {
      cfg.throttleMaxUs = cfg.throttleCenterUs + 1;
    }
    cfg.throttleMinUs = clamp(cfg.throttleMinUs, 1200, 1499);
    cfg.throttleMaxUs = clamp(cfg.throttleMaxUs, 1501, RCD_HARD_MAX_US);
    return cfg;
  }

  function rcdSaveConfigToStorage() {
    try {
      localStorage.setItem(
        RCD_CONFIG.storageKey,
        JSON.stringify({
          version: RCD_CONFIG.version,
          fullLockDeg: S.cfg.fullLockDeg,
          steerCenter: S.cfg.steerCenter,
          throttleMinUs: S.cfg.throttleMinUs,
          throttleCenterUs: S.cfg.throttleCenterUs,
          throttleMaxUs: S.cfg.throttleMaxUs,
          gimbalRate: S.cfg.gimbalRate,
          // 方向反向：现场"推的方向和动作相反"时翻的就是这几个，必须落盘
          steerInvert: S.cfg.steerInvert,
          gimbal: {
            center: S.cfg.gimbal.center,
            min: S.cfg.gimbal.min,
            max: S.cfg.gimbal.max,
            swapAxes: S.cfg.gimbal.swapAxes,
            panInvert: S.cfg.gimbal.panInvert,
            tiltInvert: S.cfg.gimbal.tiltInvert,
          },
          // 手柄映射也要落盘，否则现场改完号一刷新又回去了
          axes: {
            steer: S.cfg.axes.steer,
            rt: S.cfg.axes.rt,
            lt: S.cfg.axes.lt,
            pan: S.cfg.axes.pan,
            tilt: S.cfg.axes.tilt,
          },
        }),
      );
      return true;
    } catch (e) {
      rcdLog("保存失败（localStorage 不可用）: " + e.message);
      return false;
    }
  }

  /* 尽力同步进官方四个输入框 + 官方 CONFIG 对象，避免同页两套 UI 打架。
     官方 saveConfig() 自己引用了一个不存在的 #controlHostInput（会抛错），
     所以这里不调它，只做字段级同步。 */
  function rcdSyncOfficialConfig() {
    const map = {
      steerCenterInput: "steerCenter",
      throttleMinUsInput: "throttleMinUs",
      throttleCenterUsInput: "throttleCenterUs",
      throttleMaxUsInput: "throttleMaxUs",
    };
    Object.keys(map).forEach((id) => {
      const el = document.getElementById(id);
      if (el) el.value = String(S.cfg[map[id]]);
    });
    // 手柄映射里语义相同的三个（官方那两个"云台左/右按钮"是按键，我们的云台是轴，别乱同步）
    const axisMap = {
      steerAxisInput: "steer",
      throttlePosBtnInput: "rt",
      throttleNegBtnInput: "lt",
    };
    Object.keys(axisMap).forEach((id) => {
      const el = document.getElementById(id);
      if (el) el.value = String(S.cfg.axes[axisMap[id]]);
    });
    const official = rcdGlobal("CONFIG", null);
    if (official && typeof official === "object") {
      official.steerCenter = S.cfg.steerCenter;
      official.throttleMinUs = S.cfg.throttleMinUs;
      official.throttleCenterUs = S.cfg.throttleCenterUs;
      official.throttleMaxUs = S.cfg.throttleMaxUs;
      try {
        const prev = JSON.parse(localStorage.getItem("rc-ui-config") || "{}");
        localStorage.setItem("rc-ui-config", JSON.stringify(Object.assign(prev, {
          steerCenter: S.cfg.steerCenter,
          throttleMinUs: S.cfg.throttleMinUs,
          throttleCenterUs: S.cfg.throttleCenterUs,
          throttleMaxUs: S.cfg.throttleMaxUs,
        })));
      } catch (_) {}
    } else {
      rcdLog("官方 CONFIG 对象不可见，仅同步了输入框");
    }
  }

  /* ======================================================================
   * 5. 协议（与车端 opi-control-app.py 对齐）
   * ==================================================================== */
  function rcdThrottleUs(normalized) {
    const v = clamp(toInt(normalized, 0), -100, 100);
    const c = S.cfg.throttleCenterUs;
    if (v === 0) return c;
    if (v > 0) return Math.round(c + (v / 100) * (S.cfg.throttleMaxUs - c));
    return Math.round(c + (v / 100) * (c - S.cfg.throttleMinUs));
  }

  function rcdDrivePayload() {
    return {
      type: "drive",
      steering: Math.round(S.drive.steering),
      steer_center: Math.round(S.cfg.steerCenter),
      throttle: Math.round(S.drive.throttle),
      // 三个 us 参数随每帧下发，后端每次 drive 会 configure_* 重配行程
      throttle_min_us: Math.round(S.cfg.throttleMinUs),
      throttle_center_us: Math.round(S.cfg.throttleCenterUs),
      throttle_max_us: Math.min(Math.round(S.cfg.throttleMaxUs), RCD_HARD_MAX_US),
      throttle_us: rcdThrottleUs(S.drive.throttle),
    };
  }

  function rcdSend(obj) {
    const sock = rcdGlobal("ws", null);
    if (!sock || sock.readyState !== 1) return false;
    try {
      sock.send(JSON.stringify(obj));
      return true;
    } catch (e) {
      rcdLog("发送失败: " + e.message);
      return false;
    }
  }

  function rcdSendDrive(force) {
    const now = Date.now();
    if (!force && now - S.lastSend < S.cfg.sendIntervalMs) return;
    S.lastSend = now;
    rcdSend(rcdDrivePayload());
  }

  /* 逻辑值 → 线路值。
     ★ 本车 pan/tilt 是**交叉接的**：`set_pan_angle()` 实际驱动上下、`set_tilt_angle()` 实际驱动左右。
     旁证：官方页 sendGimbalNow() 就是无条件 `send({pan: tilt, tilt: pan})`——它一直在对调，
     我们照抄这个对调，云台方向就和官方页一致了（2026-10-17 现场反馈"左右/上下反了"即此）。
     内部约定：S.gimbal.pan 大 = 右，S.gimbal.tilt 大 = 上（与官方那两个滑条同号）。 */
  function rcdGimbalWire() {
    const g = S.cfg.gimbal;
    const span = g.max + g.min;
    const h = S.gimbal.pan; // 水平意图（大 = 右）
    const v = S.gimbal.tilt; // 俯仰意图（大 = 上）
    // 对调：水平意图走 tilt 线、俯仰意图走 pan 线（= 官方页的做法）
    const wirePan = g.swapAxes ? v : h;
    const wireTilt = g.swapAxes ? h : v;
    return {
      type: "gimbal",
      pan: Math.round(g.panInvert ? span - wirePan : wirePan),
      tilt: Math.round(g.tiltInvert ? span - wireTilt : wireTilt),
    };
  }

  function rcdSendGimbal(force) {
    const wire = rcdGimbalWire();
    const key = wire.pan + "/" + wire.tilt;
    if (!force && key === S.lastGimbalKey) return;
    S.lastGimbalKey = key;
    rcdSend(wire);
  }

  /* ======================================================================
   * 6. 日志（面板里 4 行）
   * ==================================================================== */
  function rcdLog(msg) {
    const line = new Date().toLocaleTimeString() + " " + msg;
    S.logs.push(line);
    while (S.logs.length > RCD_CONFIG.logLines) S.logs.shift();
    const el = document.getElementById("rcd-log");
    if (el) {
      // 自动跟到最新一行（否则新日志在下面看不见，得手动拖滚动条）；
      // 但如果你正往上翻看旧日志，就不打断你 —— 只有本来贴着底部时才跟随。
      const stick = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
      el.textContent = S.logs.join("\n");
      if (stick) el.scrollTop = el.scrollHeight;
    }
    if (window.console && console.log) console.log("[rcd] " + msg);
  }

  /* 重复的后端报错（未解锁时 12.5Hz 打回来的 "system not armed"）只记一次 */
  function rcdLogError(msg) {
    const now = Date.now();
    if (msg === S.lastErrKey && now - S.lastErrAt < 2000) return;
    S.lastErrKey = msg;
    S.lastErrAt = now;
    if (S.armIntent === true) {
      // 后端 arm 失败时不管哪种原因都回 "estop active"，所以这里按我们自己的遥测来分辨
      const hint = S.tel.estop
        ? "现在在急停里：先点【清除急停】，再点【解锁】"
        : "油门不为 0 时后端拒绝解锁：先点【上锁】把油门归零，再点【解锁】";
      rcdLog("解锁失败（后端回 " + msg + "）—— " + hint);
      S.armIntent = null;
      return;
    }
    rcdLog("后端拒绝：" + msg);
  }

  /* ======================================================================
   * 7. 面板 UI
   * ==================================================================== */
  const RCD_CSS = [
    "#rcd-panel{position:fixed;left:16px;bottom:16px;z-index:120;width:min(440px,calc(100vw - 32px));",
    "max-height:min(80vh,720px);overflow:auto;background:rgba(18,22,28,.9);color:#f2f5f8;",
    "border:1px solid rgba(255,255,255,.18);border-radius:16px;padding:12px 14px;",
    "font:13px/1.45 Arial,sans-serif;box-shadow:0 8px 28px rgba(0,0,0,.45);box-sizing:border-box}",
    "#rcd-panel *{box-sizing:border-box}",
    ".rcd-head{display:flex;align-items:baseline;gap:8px;margin-bottom:8px;cursor:grab;",
    "user-select:none;touch-action:none}", /* 标题栏 = 拖动把手 */
    ".rcd-head.rcd-dragging{cursor:grabbing}",
    ".rcd-title{font-size:15px;font-weight:700;white-space:nowrap}",
    ".rcd-tip{font-size:11px;color:#9fb0c0;flex:1}",
    ".rcd-collapse{border:0;border-radius:8px;padding:3px 8px;font-size:12px;cursor:pointer;",
    "background:rgba(255,255,255,.16);color:#e6eef5;white-space:nowrap}",
    ".rcd-collapse:hover{background:rgba(255,255,255,.3)}",
    /* 收起后只留的这个"隐蔽拉手"：贴左边缘、半透明，鼠标移上去才亮 */
    "#rcd-tab{position:fixed;left:0;z-index:120;display:none;width:22px;padding:12px 0;border:0;",
    "border-radius:0 12px 12px 0;background:rgba(18,22,28,.6);color:#cfe3f5;font-size:12px;",
    "letter-spacing:2px;writing-mode:vertical-rl;text-align:center;cursor:pointer;opacity:.32;",
    "transition:opacity .15s}",
    "#rcd-tab:hover{opacity:.95;background:rgba(21,101,192,.8)}",
    ".rcd-status{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:10px}",
    ".rcd-chip{background:rgba(255,255,255,.12);border-radius:999px;padding:3px 9px;font-size:12px;font-weight:700}",
    ".rcd-chip.ok{background:#2e7d32}.rcd-chip.bad{background:#d32f2f}.rcd-chip.warn{background:#e07b00}",
    ".rcd-sliders{display:flex;gap:16px;align-items:flex-start;margin-bottom:10px}",
    ".rcd-col{display:flex;flex-direction:column;gap:4px}",
    ".rcd-col.grow{flex:1;min-width:0}",
    ".rcd-label{font-size:12px;color:#9fb0c0}",
    ".rcd-thr-row{display:flex;align-items:center;gap:10px}",
    ".rcd-vslider{writing-mode:vertical-lr;direction:rtl;-webkit-appearance:slider-vertical;",
    "appearance:slider-vertical;width:30px;height:112px;accent-color:#2e7d32}",
    ".rcd-hslider{width:100%;accent-color:#1565c0}",
    ".rcd-readout{font-size:12px;font-weight:700;color:#ffd54f;white-space:nowrap}",
    ".rcd-hintline{font-size:11px;color:#9fb0c0}",
    ".rcd-config{margin:4px 0 8px;border-top:1px solid rgba(255,255,255,.14);padding-top:6px}",
    ".rcd-config summary{cursor:pointer;font-weight:700;font-size:13px;padding:2px 0}",
    ".rcd-row{display:grid;grid-template-columns:1fr 96px;gap:8px;align-items:center;margin:6px 0}",
    ".rcd-row label{font-size:12px}",
    ".rcd-row .unit{font-size:11px;color:#9fb0c0;margin-left:4px}",
    ".rcd-row input{width:100%;padding:4px 6px;border-radius:8px;border:1px solid rgba(255,255,255,.25);",
    "background:rgba(255,255,255,.08);color:#fff;font-size:13px}",
    ".rcd-fieldhint{font-size:11px;color:#9fb0c0;margin:-4px 0 6px}",
    ".rcd-check{display:flex;align-items:center;gap:6px;font-size:12px;margin:5px 0;cursor:pointer}",
    ".rcd-check input{width:15px;height:15px;accent-color:#1565c0;cursor:pointer}",
    ".rcd-sub-title{font-size:12px;font-weight:700;color:#cfd8e3;margin:10px 0 4px;",
    "border-top:1px solid rgba(255,255,255,.14);padding-top:8px}",
    ".rcd-actions{display:flex;gap:8px;margin-top:8px}",
    ".rcd-actions button{border:0;border-radius:10px;padding:7px 12px;font-size:13px;cursor:pointer;color:#fff;background:#1565c0}",
    ".rcd-actions button.sec{background:#5a6673}",
    ".rcd-logbar{display:flex;align-items:center;gap:8px;margin:8px 0 4px;font-size:12px;color:#9fb0c0}",
    ".rcd-logbar .rcd-fill{flex:1}",
    ".rcd-logbar button{border:0;border-radius:8px;padding:3px 10px;font-size:12px;cursor:pointer;color:#fff;background:#5a6673}",
    ".rcd-log{white-space:pre-wrap;font:11px/1.5 Consolas,monospace;color:#b9c7d4;",
    "background:rgba(0,0,0,.35);border-radius:10px;padding:6px 8px;height:104px;overflow-y:auto}",
    /* 手柄调试区 */
    ".rcd-gp-map{font-size:11px;color:#9fb0c0;margin-bottom:6px;word-break:break-all}",
    ".rcd-gp-last{font:12px/1.4 Consolas,monospace;background:rgba(21,101,192,.25);border-radius:8px;",
    "padding:5px 8px;margin-bottom:6px;color:#e3f2fd}",
    ".rcd-gp-last.hit{background:rgba(224,123,0,.35);color:#fff}",
    ".rcd-gp-grid{display:flex;flex-wrap:wrap;gap:4px;margin-bottom:6px}",
    ".rcd-gp-item{font:11px/1.3 Consolas,monospace;background:rgba(255,255,255,.08);border-radius:6px;",
    "padding:2px 6px;color:#c7d3de;border:1px solid transparent}",
    ".rcd-gp-item.role{color:#fff;background:rgba(46,125,50,.35);border-color:rgba(46,125,50,.6)}",
    ".rcd-gp-item.live{color:#fff;background:rgba(21,101,192,.5)}",
    ".rcd-gp-item.active{color:#111;background:#ffd54f;border-color:#fff}",
    ".rcd-gp-sub{font-size:11px;color:#9fb0c0;margin:2px 0 4px}",
  ].join("");

  function rcdInjectCss() {
    if (document.getElementById("rcd-style")) return;
    const style = document.createElement("style");
    style.id = "rcd-style";
    style.textContent = RCD_CSS;
    (document.head || document.documentElement).appendChild(style);
  }

  /* 展开一个折叠区时把它带进视野（面板自己就是个滚动容器，展开的内容常在折叠线以下） */
  function rcdRevealOnOpen(el) {
    if (!el || !el.open) return;
    try {
      el.scrollIntoView({ block: "nearest" });
    } catch (_) {}
  }

  function rcdBuildPanel() {
    if (document.getElementById(RCD_CONFIG.panelId)) return;
    const rows = RCD_FIELDS.map((f) => {
      const hint = f.hint ? '<div class="rcd-fieldhint">' + f.hint + "</div>" : "";
      return (
        '<div class="rcd-row"><label for="rcd-cfg-' + f.key + '">' + f.label +
        '<span class="unit">(' + f.min + "~" + f.max + " " + f.unit + ")</span></label>" +
        '<input id="rcd-cfg-' + f.key + '" type="number" min="' + f.min + '" max="' + f.max +
        '" step="' + f.step + '"></div>' + hint
      );
    }).join("");

    const axisRows = RCD_AXIS_FIELDS.map((f) => (
      '<div class="rcd-row"><label for="rcd-cfg-axes-' + f.key + '">' + f.label +
      '<span class="unit">(0~31 号)</span></label>' +
      '<input id="rcd-cfg-axes-' + f.key + '" type="number" min="0" max="31" step="1"></div>' +
      (f.hint ? '<div class="rcd-fieldhint">' + f.hint + "</div>" : "")
    )).join("");

    const invertRows =
      '<label class="rcd-check"><input type="checkbox" id="' + RCD_STEER_INVERT.id + '"> ' +
      RCD_STEER_INVERT.label + "</label>" +
      RCD_INVERT_FIELDS.map((f) => (
        '<label class="rcd-check"><input type="checkbox" id="' + f.id + '"> ' + f.label + "</label>"
      )).join("");

    const panel = document.createElement("div");
    panel.id = RCD_CONFIG.panelId;
    panel.className = "rcd-panel";
    panel.innerHTML =
      '<div class="rcd-head" id="rcd-drag" title="按住这里可以拖动面板；双击复位到左下角">' +
      '<span class="rcd-title">驾驶模式</span>' +
      '<span class="rcd-tip">左摇杆转向 · RT 前进 · LT 倒车 · 右摇杆云台（开关用官方【手柄控制】按钮）</span>' +
      '<button type="button" class="rcd-collapse" id="rcd-collapse" title="收起面板（只留左侧一条细拉手，点它再展开）">收起 ◀</button>' +
      "</div>" +
      '<div class="rcd-status">' +
      '<span class="rcd-chip" id="rcd-chip-ws">WS: —</span>' +
      '<span class="rcd-chip" id="rcd-chip-armed">armed: —</span>' +
      '<span class="rcd-chip" id="rcd-chip-estop">estop: —</span>' +
      '<span class="rcd-chip" id="rcd-chip-thr">油门: —</span>' +
      '<span class="rcd-chip" id="rcd-chip-steer">方向: —</span>' +
      "</div>" +
      '<div class="rcd-sliders">' +
      '<div class="rcd-col"><div class="rcd-label">油门</div>' +
      '<div class="rcd-thr-row">' +
      '<input id="rcd-throttle" class="rcd-vslider" type="range" min="-100" max="100" step="1" value="0">' +
      '<div class="rcd-readout" id="rcd-thr-val">停止 (1500us)</div></div></div>' +
      '<div class="rcd-col grow"><div class="rcd-label">方向</div>' +
      '<input id="rcd-steer" class="rcd-hslider" type="range" min="-100" max="100" step="1" value="0">' +
      '<div class="rcd-readout" id="rcd-steer-val">方向 (90°)</div>' +
      '<div class="rcd-hintline">滑条松手自动回中；退出驾驶模式会强制归零并补发一帧</div></div>' +
      "</div>" +
      '<details class="rcd-config" id="rcd-config"><summary>参数配置</summary>' +
      rows +
      '<div class="rcd-sub-title">手柄映射（对不上就改号，改完点保存）</div>' +
      axisRows +
      '<div class="rcd-sub-title">方向反向（推的方向和动作相反时勾上）</div>' +
      invertRows +
      '<div class="rcd-fieldhint">云台方向对不上时按症状勾：推右摇杆往右、云台却往左 → 勾「云台水平反向」；' +
      "推上、云台却往下 → 勾「云台俯仰反向」；推右摇杆左右、云台却上下动 → 勾「左右/上下对调」（本车默认就要勾）。" +
      "改完点【保存】，映射行和实时读数会立刻跟着变。</div>" +
      '<div class="rcd-fieldhint" id="rcd-deadband">—</div>' +
      '<div class="rcd-actions"><button id="rcd-save">保存</button>' +
      '<button id="rcd-default" class="sec">恢复默认</button></div></details>' +
      // 手柄调试：不依赖"手柄控制"开关，开着就能先确认按键/轴是否匹配（开着面板就会实时读）
      '<details class="rcd-config" id="rcd-debug"><summary>手柄调试（实时读数 / 先确认按键匹配）</summary>' +
      '<div class="rcd-gp-map" id="rcd-gp-map">—</div>' +
      '<div class="rcd-gp-last" id="rcd-gp-last">未检测到手柄</div>' +
      '<div class="rcd-gp-sub">轴（黄底 = 刚动过的那个）</div>' +
      '<div class="rcd-gp-grid" id="rcd-gp-axes"></div>' +
      '<div class="rcd-gp-sub">按键（黄底 = 刚按下的那个）</div>' +
      '<div class="rcd-gp-grid" id="rcd-gp-buttons"></div>' +
      '<div class="rcd-fieldhint">拨一下摇杆/按一下扳机，黄底那个就是它；和上面「映射」对不上就来改配置里的轴/键号。</div>' +
      "</details>" +
      '<div class="rcd-logbar"><span>日志（手柄/滑条做了什么）</span><span class="rcd-fill"></span>' +
      '<button id="rcd-log-clear" type="button">清空</button></div>' +
      '<div class="rcd-log" id="rcd-log"></div>';
    document.body.appendChild(panel);
    rcdBuildTab(); // 收起后用来"叫回来"的那条细拉手
    rcdMakeDraggable(panel, document.getElementById("rcd-drag"));
    document.getElementById("rcd-collapse").addEventListener("click", () => rcdSetCollapsed(true));

    RCD_FIELDS.forEach((f) => {
      const el = document.getElementById("rcd-cfg-" + f.key);
      if (el) el.addEventListener("change", rcdUpdateDeadbandHint);
    });
    const thr = document.getElementById("rcd-throttle");
    const steer = document.getElementById("rcd-steer");
    thr.addEventListener("input", () => {
      S.drive.throttle = clamp(toInt(thr.value, 0), -100, 100);
      rcdRenderDriveReadout();
      rcdSendDrive(false);
      rcdLogDriveAction("滑条");
    });
    steer.addEventListener("input", () => {
      rcdSetSteerFromHuman(clamp(toInt(steer.value, 0), -100, 100));
      rcdRenderDriveReadout();
      rcdSendDrive(false);
      rcdLogDriveAction("滑条");
    });
    // 松手回中（鼠标/触摸都要覆盖），并补发一帧
    ["change", "mouseup", "touchend", "pointerup", "touchcancel"].forEach((evt) => {
      thr.addEventListener(evt, () => rcdResetThrottle(true));
      steer.addEventListener(evt, () => rcdResetSteer(true));
    });
    document.getElementById("rcd-save").addEventListener("click", rcdApplyConfigFromInputs);
    document.getElementById("rcd-default").addEventListener("click", () => {
      S.cfg = rcdLoadConfigDefaults();
      rcdFillConfigInputs();
      rcdUpdateDeadbandHint();
      rcdSaveConfigToStorage();
      rcdSyncOfficialConfig();
      rcdRenderGpMap();
      rcdLog("已恢复默认参数并保存");
    });
    // 手柄调试区：展开就实时读（不需要先开"手柄控制"），收起就停掉轮询
    const dbg = document.getElementById("rcd-debug");
    dbg.addEventListener("toggle", () => {
      rcdRenderGpMap();
      if (dbg.open) {
        rcdLog("手柄调试已展开：拨摇杆/按扳机，黄底那个就是它");
        if (!rcdFirstGamepad()) rcdLog("现在没检测到手柄 —— " + rcdGpWhyNoPad());
        rcdRevealOnOpen(dbg);
        rcdStartGpDebug();
      } else {
        rcdStopGpDebug();
      }
    });
    const cfg = document.getElementById("rcd-config");
    if (cfg) cfg.addEventListener("toggle", () => rcdRevealOnOpen(cfg));
    document.getElementById("rcd-log-clear").addEventListener("click", () => {
      S.logs = [];
      const el = document.getElementById("rcd-log");
      if (el) el.textContent = "";
      rcdLog("日志已清空");
    });
    rcdRenderGpMap();
  }

  function rcdShowPanel() {
    const panel = document.getElementById(RCD_CONFIG.panelId);
    if (!panel) return false;
    panel.style.display = ""; // 清掉 rcdExit 设的 none（清空即回到 CSS 默认值）
    rcdSetCollapsed(false, true); // 每次进驾驶模式都从"展开"开始（位置沿用上次拖的）
    return true;
  }

  /* ======================================================================
   * 7.5 面板摆放：拖动 + 收起成左侧细拉手
   *   现场最常见的抱怨是"挡视线"——所以面板要能挪走、也要能整个收起来，
   *   收起来之后还得有个不显眼但找得到的地方把它叫回来。
   * ==================================================================== */
  function rcdPlacePanel(x, y) {
    const panel = document.getElementById(RCD_CONFIG.panelId);
    if (!panel) return;
    const w = panel.offsetWidth || 440;
    const h = panel.offsetHeight || 240;
    const keep = 60; // 至少留 60px 在屏幕里，拖到边上也不会"弄丢"
    const nx = clamp(x, keep - w, Math.max(keep - w, window.innerWidth - keep));
    const ny = clamp(y, 0, Math.max(0, window.innerHeight - 40));
    panel.style.left = Math.round(nx) + "px";
    panel.style.top = Math.round(ny) + "px";
    panel.style.right = "auto";
    panel.style.bottom = "auto";
    rcdPlaceTab();
  }

  function rcdPlaceTab() {
    const tab = document.getElementById("rcd-tab");
    const panel = document.getElementById(RCD_CONFIG.panelId);
    if (!tab || !panel) return;
    const top = parseInt(panel.style.top, 10);
    const y = clamp(isNum(top) ? top : 120, 96, Math.max(96, window.innerHeight - 140));
    tab.style.top = Math.round(y) + "px";
  }

  function rcdSavePos() {
    const panel = document.getElementById(RCD_CONFIG.panelId);
    if (!panel) return;
    try {
      const left = parseInt(panel.style.left, 10);
      const top = parseInt(panel.style.top, 10);
      if (!isNum(left) || !isNum(top)) localStorage.removeItem(RCD_CONFIG.posKey); // 复位过的就清掉
      else localStorage.setItem(RCD_CONFIG.posKey, JSON.stringify({ left: left, top: top }));
    } catch (_) {}
  }

  function rcdApplySavedPos() {
    let saved = null;
    try {
      saved = JSON.parse(localStorage.getItem(RCD_CONFIG.posKey) || "null");
    } catch (_) {
      saved = null;
    }
    if (saved && isNum(saved.left) && isNum(saved.top)) {
      rcdPlacePanel(saved.left, saved.top);
      return true;
    }
    return false;
  }

  function rcdResetPanelPos() {
    const panel = document.getElementById(RCD_CONFIG.panelId);
    if (!panel) return;
    panel.style.left = "";
    panel.style.top = "";
    panel.style.right = "";
    panel.style.bottom = ""; // 回到 CSS 默认：左下角
    try {
      localStorage.removeItem(RCD_CONFIG.posKey);
    } catch (_) {}
    rcdPlaceTab();
    rcdLog("面板位置已复位到左下角");
  }

  function rcdMakeDraggable(panel, handle) {
    let dx = 0;
    let dy = 0;
    let dragging = false;
    handle.addEventListener("pointerdown", (e) => {
      if (e.target && e.target.closest && e.target.closest("button")) return; // 收起按钮不触发拖动
      const r = panel.getBoundingClientRect();
      dx = e.clientX - r.left;
      dy = e.clientY - r.top;
      dragging = true;
      handle.classList.add("rcd-dragging");
      try {
        handle.setPointerCapture(e.pointerId);
      } catch (_) {}
      e.preventDefault();
    });
    handle.addEventListener("pointermove", (e) => {
      if (!dragging) return;
      rcdPlacePanel(e.clientX - dx, e.clientY - dy);
      e.preventDefault();
    });
    const stop = (e) => {
      if (!dragging) return;
      dragging = false;
      handle.classList.remove("rcd-dragging");
      try {
        handle.releasePointerCapture(e.pointerId);
      } catch (_) {}
      rcdSavePos();
    };
    handle.addEventListener("pointerup", stop);
    handle.addEventListener("pointercancel", stop);
    handle.addEventListener("dblclick", rcdResetPanelPos); // 双击复位，省得手动拖回角落
  }

  /* 收起 = 面板藏起来、左侧留一条细拉手；展开 = 反过来。位置一直沿用。 */
  function rcdSetCollapsed(on, silent) {
    S.collapsed = !!on;
    const panel = document.getElementById(RCD_CONFIG.panelId);
    const tab = document.getElementById("rcd-tab");
    if (panel) panel.style.display = on ? "none" : "";
    if (tab) tab.style.display = on && S.active ? "block" : "none";
    rcdPlaceTab();
    if (on) {
      rcdStopGpDebug(); // 面板都看不见了，别再按帧刷读数
    } else {
      const dbg = document.getElementById("rcd-debug");
      if (dbg && dbg.open) rcdStartGpDebug();
    }
    if (!silent) rcdLog(on ? "面板已收起（点左侧细拉手再展开）" : "面板已展开");
  }

  function rcdBuildTab() {
    if (document.getElementById("rcd-tab")) return;
    const tab = document.createElement("button");
    tab.id = "rcd-tab";
    tab.type = "button";
    tab.textContent = "驾驶模式";
    tab.title = "展开驾驶模式面板（收起时它就贴在左边缘，鼠标移上去会变亮）";
    tab.addEventListener("click", () => rcdSetCollapsed(false));
    document.body.appendChild(tab);
  }

  function rcdLoadConfigDefaults() {
    const cfg = Object.assign({}, RCD_CONFIG);
    cfg.axes = Object.assign({}, RCD_CONFIG.axes);
    cfg.gimbal = Object.assign({}, RCD_CONFIG.gimbal);
    return rcdFixThrottleOrder(cfg);
  }

  function rcdFillConfigInputs() {
    RCD_FIELDS.forEach((f) => {
      const el = document.getElementById("rcd-cfg-" + f.key);
      if (el) el.value = String(S.cfg[f.key]);
    });
    RCD_AXIS_FIELDS.forEach((f) => {
      const el = document.getElementById("rcd-cfg-axes-" + f.key);
      if (el) el.value = String(S.cfg.axes[f.key]);
    });
    rcdSetInvertInputs();
  }

  function rcdSetInvertInputs() {
    const map = {
      "rcd-cfg-inv-steer": S.cfg.steerInvert,
      "rcd-cfg-gimbal-swap": S.cfg.gimbal.swapAxes,
      "rcd-cfg-inv-pan": S.cfg.gimbal.panInvert,
      "rcd-cfg-inv-tilt": S.cfg.gimbal.tiltInvert,
    };
    Object.keys(map).forEach((id) => {
      const el = document.getElementById(id);
      if (el) el.checked = !!map[id];
    });
  }

  function rcdReadInvertInputs() {
    const get = (id, dflt) => {
      const el = document.getElementById(id);
      return el ? !!el.checked : dflt;
    };
    S.cfg.steerInvert = get("rcd-cfg-inv-steer", S.cfg.steerInvert);
    S.cfg.gimbal.swapAxes = get("rcd-cfg-gimbal-swap", S.cfg.gimbal.swapAxes);
    S.cfg.gimbal.panInvert = get("rcd-cfg-inv-pan", S.cfg.gimbal.panInvert);
    S.cfg.gimbal.tiltInvert = get("rcd-cfg-inv-tilt", S.cfg.gimbal.tiltInvert);
  }

  function rcdApplyConfigFromInputs() {
    let clamped = [];
    RCD_FIELDS.forEach((f) => {
      const el = document.getElementById("rcd-cfg-" + f.key);
      if (!el) return;
      const raw = toInt(el.value, S.cfg[f.key]);
      const v = rcdClampField(f.key, raw);
      if (v !== raw) clamped.push(f.key + " " + raw + "→" + v);
      S.cfg[f.key] = v;
    });
    RCD_AXIS_FIELDS.forEach((f) => {
      const el = document.getElementById("rcd-cfg-axes-" + f.key);
      if (!el) return;
      const raw = toInt(el.value, S.cfg.axes[f.key]);
      const v = rcdClampAxisIndex(raw, S.cfg.axes[f.key]);
      if (v !== raw) clamped.push("axes." + f.key + " " + raw + "→" + v);
      S.cfg.axes[f.key] = v;
    });
    rcdFixThrottleOrder(S.cfg);
    rcdReadInvertInputs();
    rcdFillConfigInputs();
    rcdUpdateDeadbandHint();
    rcdRenderGpMap();
    const ok = rcdSaveConfigToStorage();
    rcdSyncOfficialConfig();
    // 同一个编号被两个角色占了 → 现场必然有一个功能不动，直接点出来
    const a = S.cfg.axes;
    const dup = [];
    if (a.rt === a.lt) dup.push("RT 与 LT 同为 " + a.rt);
    if (a.steer === a.pan || a.steer === a.tilt) dup.push("转向轴与云台轴撞号");
    if (a.pan === a.tilt) dup.push("云台水平/俯仰同为 " + a.pan);
    rcdLog(
      (ok ? "参数已保存" : "参数未落盘") +
        (clamped.length ? "（钳位：" + clamped.join("、") + "）" : "") +
        (dup.length ? " ⚠️ 映射撞号：" + dup.join("；") : ""),
    );
  }

  /* 实时死区提示：按当前三个 us 参数算「油门要推多少车才起步」 */
  function rcdUpdateDeadbandHint() {
    const el = document.getElementById("rcd-deadband");
    if (!el) return;
    const readInputs = () => {
      const get = (key) => {
        const input = document.getElementById("rcd-cfg-" + key);
        return input ? rcdClampField(key, toInt(input.value, S.cfg[key])) : S.cfg[key];
      };
      const c = get("throttleCenterUs");
      const lo = Math.min(get("throttleMinUs"), c - 1);
      const hi = Math.max(get("throttleMaxUs"), c + 1);
      return { c, lo, hi };
    };
    const { c, lo, hi } = readInputs();
    const fwd = hi > c ? Math.ceil(((RCD_CONFIG.escStartUs - c) / (hi - c)) * 100) : 101;
    const rev = c > lo ? Math.ceil(((c - RCD_CONFIG.escStartRevUs) / (c - lo)) * 100) : 101;
    const fmt = (v) => (v <= 0 ? "任何幅度" : v > 100 ? "满行程也到不了" : ">" + v + "%");
    el.textContent =
      "当前行程：前进油门需 " + fmt(fwd) + "（起转脉宽 " + RCD_CONFIG.escStartUs +
      "us，实测）、后退需 " + fmt(rev) + "（按 " + RCD_CONFIG.escStartRevUs +
      "us 对称假设，未实测）车才起步。";
  }

  function rcdRenderDriveReadout() {
    const thr = document.getElementById("rcd-thr-val");
    const steer = document.getElementById("rcd-steer-val");
    if (thr) {
      const v = Math.round(S.drive.throttle);
      const us = rcdThrottleUs(v);
      thr.textContent =
        v === 0 ? "停止 (" + us + "us)" : (v > 0 ? "前进" + v + "%" : "后退" + Math.abs(v) + "%") + " (" + us + "us)";
    }
    if (steer) {
      const h = Math.round(((S.cfg.steerCenter - S.drive.steering) / Math.max(1, S.cfg.fullLockDeg)) * 100);
      const dir = h === 0 ? "方向" : h < 0 ? "左" + Math.abs(h) : "右" + h;
      steer.textContent = dir + " (" + Math.round(S.drive.steering) + "°)";
    }
  }

  function rcdSetSteerFromHuman(human) {
    const sign = S.cfg.steerInvert ? -1 : 1;
    S.drive.steering = clamp(
      Math.round(S.cfg.steerCenter - sign * (human / 100) * S.cfg.fullLockDeg),
      S.cfg.steerCenter - RCD_SOFT_LIMIT_DEG,
      S.cfg.steerCenter + RCD_SOFT_LIMIT_DEG,
    );
  }

  function rcdResetThrottle(sendNow) {
    const el = document.getElementById("rcd-throttle");
    if (el) el.value = "0";
    if (S.drive.throttle === 0 && !sendNow) return;
    S.drive.throttle = 0;
    rcdRenderDriveReadout();
    if (sendNow) rcdSendDrive(true);
  }

  function rcdResetSteer(sendNow) {
    const el = document.getElementById("rcd-steer");
    if (el) el.value = "0";
    rcdSetSteerFromHuman(0);
    rcdRenderDriveReadout();
    if (sendNow) rcdSendDrive(true);
  }

  /* 手柄轮询时把我们的滑条同步过去（纯显示） */
  function rcdSyncSliderUI() {
    const thr = document.getElementById("rcd-throttle");
    const steer = document.getElementById("rcd-steer");
    if (thr) thr.value = String(Math.round(S.drive.throttle));
    if (steer) {
      const h = Math.round(((S.cfg.steerCenter - S.drive.steering) / Math.max(1, S.cfg.fullLockDeg)) * 100);
      steer.value = String(clamp(h, -100, 100));
    }
    rcdRenderDriveReadout();
  }

  /* ======================================================================
   * 8. 状态条 / WS
   * ==================================================================== */
  function rcdChip(id, text, cls) {
    const el = document.getElementById(id);
    if (!el) return;
    el.textContent = text;
    el.className = "rcd-chip" + (cls ? " " + cls : "");
  }

  function rcdRenderStatus() {
    const sock = rcdGlobal("ws", null);
    const open = !!sock && sock.readyState === 1;
    rcdChip("rcd-chip-ws", "WS: " + (open ? "已连接" : sock ? "已断开" : "无连接"), open ? "ok" : "bad");
    rcdChip("rcd-chip-armed", "armed: " + S.tel.armed, S.tel.armed ? "ok" : "");
    rcdChip("rcd-chip-estop", "estop: " + S.tel.estop, S.tel.estop ? "bad" : "");
    rcdChip("rcd-chip-thr", "油门: " + S.tel.throttle, "");
    rcdChip("rcd-chip-steer", "方向: " + S.tel.steering, "");
  }

  /* 官方 ws 实例每次重连都会新建对象，所以每 tick 检查一次是否需要重新挂监听 */
  function rcdAttachWs() {
    const sock = rcdGlobal("ws", null);
    if (!sock || sock === S.wsRef) return;
    S.wsRef = sock;
    try {
      sock.addEventListener("message", rcdOnWsMessage);
    } catch (e) {
      rcdLog("挂 WS 监听失败: " + e.message);
    }
  }

  function rcdOnWsMessage(event) {
    let data = null;
    try {
      data = JSON.parse(event.data);
    } catch (_) {
      return;
    }
    if (!data || typeof data !== "object") return;
    if (data.type === "telemetry") {
      S.tel.armed = !!data.armed;
      S.tel.estop = !!data.estop;
      if (isNum(data.throttle)) S.tel.throttle = data.throttle;
      if (isNum(data.steering)) S.tel.steering = data.steering;
      return;
    }
    if (data.type === "error") {
      rcdLogError(String(data.msg || "unknown"));
    }
  }

  function rcdTick() {
    if (!S.active) return;
    rcdAttachWs();
    rcdRenderStatus();
    rcdSyncGamepadLabel();
    // 官方 connectWS 自带 3s 重连；万一它没跑（官方页改版），我们自己补一次
    const sock = rcdGlobal("ws", null);
    const open = !!sock && sock.readyState === 1;
    if (!open && Date.now() - S.lastReconnect > RCD_CONFIG.reconnectMs) {
      S.lastReconnect = Date.now();
      const fn = rcdGlobal("connectWS", null);
      if (typeof fn === "function") {
        try {
          fn();
          rcdLog("WS 未连接，已调用官方 connectWS()");
        } catch (e) {
          rcdLog("WS 重连失败: " + e.message);
        }
      } else {
        rcdLog("WS 未连接，且官方 connectWS 不可见");
      }
    }
  }

  function rcdPing() {
    if (!S.active) return;
    // 官方 setInterval 已经在 500ms 打 ping；这里再补一条是为了"面板自己在跑"时
    // 即便官方心跳被改动也不会掉进后端 8s failsafe（armed 静止时尤其重要）。
    rcdSend({ type: "ping" });
  }

  /* ======================================================================
   * 9. 官方控件接管（手柄按钮 / 死手开关守卫 / 解锁提示）
   * ==================================================================== */
  function rcdInstallHooks() {
    if (S.hookInstalled) return;
    S.hookInstalled = true;

    const toggle = rcdGlobal("toggleGamepadControl", null);
    if (typeof toggle === "function") {
      S.orig.toggleGamepadControl = window.toggleGamepadControl;
      window.toggleGamepadControl = function () {
        try {
          rcdToggleGamepad();
        } catch (e) {
          rcdLog("手柄开关异常: " + e.message);
        }
      };
    } else {
      rcdLog("官方 toggleGamepadControl 不可见：退化成面板内开关");
    }

    // 悬停保险：官方「松手回中」路径若因为 gamepadEnabled 赋值失败仍然生效，
    // 这三个包装会拦掉它，避免官方滑条的 0 值打掉我们的油门。
    ["sendDriveNow", "resetSteeringToCenter", "resetThrottleToCenter"].forEach((name) => {
      const fn = window[name];
      if (typeof fn !== "function") return;
      S.orig[name] = fn;
      window[name] = function () {
        if (S.active) return undefined; // 驾驶模式期间一律不理会官方滑条路径
        return S.orig[name].apply(this, arguments);
      };
    });

    // 解锁意图：把后端那句含糊的报错翻译成"先上锁归零"（后端 arm 失败也回 estop active）
    const armFn = window.arm;
    if (typeof armFn === "function") {
      S.orig.arm = armFn;
      window.arm = function (enabled) {
        S.armIntent = !!enabled;
        return S.orig.arm.apply(this, arguments);
      };
    }
  }

  function rcdRestoreHooks() {
    if (!S.hookInstalled) return;
    Object.keys(S.orig).forEach((name) => {
      try {
        window[name] = S.orig[name];
      } catch (_) {}
    });
    S.orig = {};
    S.hookInstalled = false;
  }

  /* 官方 gamepadEnabled 是 `let` 全局词法绑定（不是 window 属性），
     所以只能用 eval 按名字读写；失败就退化成"只靠包装函数兜底"。 */
  function rcdGetOfficialFlag() {
    return !!rcdGlobal("gamepadEnabled", false);
  }

  function rcdSetOfficialFlag(on) {
    try {
      // eslint-disable-next-line no-eval
      (0, eval)("gamepadEnabled = " + (on ? "true" : "false") + ";");
      return true;
    } catch (e) {
      rcdLog("官方 gamepadEnabled 赋值失败（已靠包装兜底）: " + e.message);
      return false;
    }
  }

  function rcdUpdateOfficialGamepadButton() {
    const fn = rcdGlobal("updateGamepadButton", null);
    if (typeof fn === "function") {
      try {
        fn();
      } catch (_) {}
    }
  }

  /* 驾驶模式期间官方按钮文字由我们维护（官方 updateGamepadButton 会按它自己的
     gamepadEnabled 写"开"，与我们的开关状态无关，所以每 tick 再写一次）。 */
  function rcdSyncGamepadLabel() {
    const btn = document.getElementById("gamepadBtn");
    if (!btn) return;
    const text = "手柄控制：" + (S.gamepadOn ? "开" : "关");
    if (btn.textContent !== text) btn.textContent = text;
    const cls = S.gamepadOn ? "warn" : "secondary";
    if (btn.className !== cls) btn.className = cls;
  }

  function rcdToggleGamepad() {
    S.gamepadOn = !S.gamepadOn;
    if (S.gamepadOn) {
      rcdSetOfficialFlag(true); // 死手开关守卫：官方回中路径提前返回
      const gp = rcdFirstGamepad();
      if (!gp) rcdLog("未检测到手柄：请连接手柄并按任意键唤醒");
      rcdLog("手柄控制已开启（左摇杆转向 / RT 前进 / LT 倒车 / 右摇杆云台）");
      rcdStartLoop();
    } else {
      rcdLog("手柄控制已关闭");
    }
    rcdSyncGamepadLabel();
  }

  /* ======================================================================
   * 10. 手柄读取与映射
   * ==================================================================== */
  function rcdFirstGamepad() {
    if (!navigator.getGamepads) return null;
    const gps = navigator.getGamepads();
    if (!gps) return null;
    for (let i = 0; i < gps.length; i++) {
      if (gps[i] && gps[i].connected) return gps[i];
    }
    return null;
  }

  function rcdAxis(gp, index, dflt) {
    const v = gp.axes ? gp.axes[index] : undefined;
    return isNum(v) ? v : dflt;
  }

  function rcdButton(gp, index, dflt) {
    const b = gp.buttons ? gp.buttons[index] : undefined;
    return b && isNum(b.value) ? b.value : dflt;
  }

  function rcdDeadzone(v, dz) {
    const d = isNum(dz) ? dz : S.cfg.deadzone;
    return Math.abs(v) < d ? 0 : v;
  }

  function rcdMapDrive(gp) {
    const x = rcdDeadzone(rcdAxis(gp, S.cfg.axes.steer, 0));
    const rt = rcdButton(gp, S.cfg.axes.rt, 0);
    const lt = rcdButton(gp, S.cfg.axes.lt, 0);
    const t = clamp(rcdDeadzone(rt - lt), -1, 1);
    const human = clamp(Math.round(x * 100), -100, 100);
    const sign = S.cfg.steerInvert ? -1 : 1;
    const steering = clamp(
      Math.round(S.cfg.steerCenter - sign * x * S.cfg.fullLockDeg),
      S.cfg.steerCenter - RCD_SOFT_LIMIT_DEG,
      S.cfg.steerCenter + RCD_SOFT_LIMIT_DEG,
    );
    return { steering: steering, throttle: clamp(Math.round(t * 100), -100, 100), human: human };
  }

  function rcdMapGimbal(gp) {
    const g = S.cfg.gimbal;
    // 云台用两倍死区：摇杆回中不干净时别让云台自己慢慢爬
    const x = rcdDeadzone(rcdAxis(gp, S.cfg.axes.pan, 0), S.cfg.deadzone * 2);
    const y = rcdDeadzone(rcdAxis(gp, S.cfg.axes.tilt, 0), S.cfg.deadzone * 2);
    let pan = S.gimbal.pan;
    let tilt = S.gimbal.tilt;
    if (x) pan = clamp(Math.round(pan + (x > 0 ? 1 : -1) * S.cfg.gimbalRate), g.min, g.max);
    if (y) tilt = clamp(Math.round(tilt + (y > 0 ? -1 : 1) * S.cfg.gimbalRate), g.min, g.max);
    return { pan: pan, tilt: tilt };
  }

  function rcdStartLoop() {
    if (S.loopRunning) return;
    S.loopRunning = true;
    requestAnimationFrame(rcdGamepadLoop);
  }

  function rcdGamepadLoop() {
    if (!S.active || !S.gamepadOn) {
      S.loopRunning = false;
      return;
    }
    const gp = rcdFirstGamepad();
    const now = Date.now();
    if (gp && now - S.lastSend >= S.cfg.sendIntervalMs) {
      rcdLogGpEdges(gp); // 先记边沿（按下/松开、摇杆从静止到动），再算这次要发什么
      const d = rcdMapDrive(gp);
      S.drive.steering = d.steering;
      S.drive.throttle = d.throttle;
      rcdSendDrive(true);
      rcdSyncSliderUI();
      const g = rcdMapGimbal(gp);
      const gMoved = g.pan !== S.gimbal.pan || g.tilt !== S.gimbal.tilt;
      if (gMoved) {
        S.gimbal = g;
        rcdSendGimbal(true);
      }
      rcdLogGimbalMotion(gMoved, "手柄");
      rcdLogDriveAction("手柄");
    }
    requestAnimationFrame(rcdGamepadLoop);
  }

  function rcdOnGamepadDisconnected() {
    if (!S.active) return;
    if (!rcdFirstGamepad() && S.gamepadOn) {
      S.gamepadOn = false;
      rcdSyncGamepadLabel();
      rcdLog("手柄已断开，手柄控制自动关闭");
      rcdResetThrottle(true); // 断连即刻断油
    }
  }

  /* ======================================================================
   * 10.5 手柄调试读数 + 动作日志
   *   现场最先要回答的两个问题：
   *     "我拨的这个摇杆到底是几号轴？" / "我按的这个键有没有被映射上？"
   *   以及"我刚那一脚油门发出去的是什么"。前者看实时读数，后者看日志。
   *   调试读数**不依赖**"手柄控制"开关——开着就能先确认映射，再决定要不要开控制。
   * ==================================================================== */
  function rcdRoleOfAxis(i) {
    const a = S.cfg.axes;
    if (i === a.steer) return "转向";
    if (i === a.pan) return "云台水平";
    if (i === a.tilt) return "云台俯仰";
    return "";
  }

  function rcdRoleOfButton(i) {
    const a = S.cfg.axes;
    if (i === a.rt) return "RT前进";
    if (i === a.lt) return "LT倒车";
    return "";
  }

  function rcdRenderGpMap() {
    const el = document.getElementById("rcd-gp-map");
    if (!el) return;
    const a = S.cfg.axes;
    el.textContent =
      "映射：转向 = 左摇杆 axes[" + a.steer + "] ｜ 油门 = RT buttons[" + a.rt +
      "] 前进 / LT buttons[" + a.lt + "] 倒车 ｜ 云台 = 右摇杆 axes[" + a.pan + "] 水平 / axes[" +
      a.tilt + "] 俯仰 ｜ 死区 " + S.cfg.deadzone + " ｜ 满舵 " + S.cfg.fullLockDeg + "°";
  }

  /* 记录"刚动过的是哪一个"，用来在读数里打黄底 */
  function rcdTrackGpChanges(gp, now) {
    const axes = gp.axes || [];
    for (let i = 0; i < axes.length; i++) {
      const v = Number(Number(axes[i]).toFixed(3));
      const prev = typeof S.gpPrevAxes[i] === "number" ? S.gpPrevAxes[i] : 0;
      if (Math.abs(v - prev) > 0.05 && Math.abs(v) > 0.08) {
        S.gpLastChange = { kind: "axes", index: i, value: v, at: now };
      }
      S.gpPrevAxes[i] = v;
    }
    const btns = gp.buttons || [];
    for (let i = 0; i < btns.length; i++) {
      const raw = btns[i] && typeof btns[i].value === "number" ? btns[i].value : 0;
      const v = Number(raw.toFixed(3));
      const prev = typeof S.gpPrevButtons[i] === "number" ? S.gpPrevButtons[i] : 0;
      if (Math.abs(v - prev) > 0.05 && v > 0.05) {
        S.gpLastChange = { kind: "buttons", index: i, value: v, at: now };
      }
      S.gpPrevButtons[i] = v;
    }
  }

  function rcdGpChip(kind, i, value, role) {
    const lc = S.gpLastChange;
    const active = !!lc && lc.kind === kind && lc.index === i &&
      Date.now() - lc.at < RCD_CONFIG.debugHoldMs;
    const cls = ["rcd-gp-item"];
    if (role) cls.push("role");
    if (Math.abs(value) > 0.05) cls.push("live");
    if (active) cls.push("active");
    const label = (kind === "axes" ? "a" : "b") + i;
    return '<span class="' + cls.join(" ") + '">' + label + " " + value.toFixed(3) +
      (role ? " " + role : "") + "</span>";
  }

  /* 没有手柄时把"为什么没有"说清楚——不然现场只能看到一片空白。
     Chromium 的规矩：**页面有焦点时按过手柄的键**，网页才拿得到手柄。 */
  function rcdGpWhyNoPad() {
    if (!navigator.getGamepads) return "这个浏览器不支持 Gamepad API（换 Chrome/Edge）";
    const slots = navigator.getGamepads() || [];
    const n = slots.length;
    if (!document.hasFocus()) {
      return "页面现在没有焦点（点一下页面/视频区域拿到焦点），再按手柄任意键或摇一下摇杆——" +
        "浏览器只在「页面有焦点时按过手柄键」之后才把手柄交给网页（当前 getGamepads() 返回 " + n + " 个空槽）";
    }
    return "浏览器还没认到手柄（当前 getGamepads() 返回 " + n + " 个空槽）：① 按一下手柄的 Xbox 键唤醒；" +
      "② 若还不行，看是不是被 Steam / DS4Windows / 其他软件独占了手柄；③ 换根线或换个 USB 口试试";
  }

  function rcdRenderGpDebug(gp) {
    const lastEl = document.getElementById("rcd-gp-last");
    const axesEl = document.getElementById("rcd-gp-axes");
    const btnEl = document.getElementById("rcd-gp-buttons");
    if (!lastEl || !axesEl || !btnEl) return;
    if (!gp) {
      lastEl.className = "rcd-gp-last";
      lastEl.textContent = "未检测到手柄：" + rcdGpWhyNoPad();
      axesEl.textContent = "";
      btnEl.textContent = "";
      return;
    }
    const now = Date.now();
    rcdTrackGpChanges(gp, now);
    const lc = S.gpLastChange;
    const fresh = !!lc && now - lc.at < RCD_CONFIG.debugHoldMs;
    lastEl.className = "rcd-gp-last" + (fresh ? " hit" : "");
    if (fresh) {
      const role = lc.kind === "axes" ? rcdRoleOfAxis(lc.index) : rcdRoleOfButton(lc.index);
      lastEl.textContent = "最后变化：" + lc.kind + "[" + lc.index + "] = " + lc.value.toFixed(3) +
        (role ? "（" + role + "）" : "（未映射）");
    } else {
      lastEl.textContent = "拨一下摇杆 / 按一下扳机，这里会显示它到底是几号";
    }
    axesEl.innerHTML = (gp.axes || [])
      .map((v, i) => rcdGpChip("axes", i, Number(v) || 0, rcdRoleOfAxis(i)))
      .join("");
    btnEl.innerHTML = (gp.buttons || [])
      .map((b, i) => rcdGpChip("buttons", i, (b && Number(b.value)) || 0, rcdRoleOfButton(i)))
      .join("");
  }

  function rcdStartGpDebug() {
    if (S.gpDebugRunning) return;
    S.gpDebugRunning = true;
    S.gpLastRender = 0;
    requestAnimationFrame(rcdGpDebugLoop);
  }

  function rcdGpDebugLoop() {
    if (!S.gpDebugRunning) return;
    const now = Date.now();
    if (now - S.gpLastRender >= RCD_CONFIG.debugRenderMs) {
      S.gpLastRender = now;
      rcdRenderGpDebug(rcdFirstGamepad());
    }
    requestAnimationFrame(rcdGpDebugLoop);
  }

  function rcdStopGpDebug() {
    S.gpDebugRunning = false;
  }

  /* 油门按"档"记：0 / 1~24% / 25~49% / 50~74% / 75~99% / 100%，换档才记一条，
     否则扳机捏一半能刷出几十行。模拟量的细节交给上面的实时读数看。 */
  function rcdThrBand(v) {
    if (v === 0) return "z";
    return (v > 0 ? "+" : "-") + Math.min(4, Math.floor(Math.abs(v) / 25));
  }

  function rcdLogDriveAction(source) {
    const now = Date.now();
    const A = S.action;
    const thr = Math.round(S.drive.throttle);
    const band = rcdThrBand(thr);
    if (A.thrBand === undefined) {
      A.thrBand = band; // 进模式时的初值不记
    } else if (band !== A.thrBand) {
      A.thrBand = band;
      const us = rcdThrottleUs(thr);
      rcdLog("[" + source + "] " + (thr === 0
        ? "油门回零 → " + us + "us"
        : "油门 " + (thr > 0 ? "前进" + thr : "后退" + Math.abs(thr)) + "% → " + us + "us"));
    }
    const steer = Math.round(S.drive.steering);
    if (A.steer === null || A.steer === undefined) {
      A.steer = steer;
    } else if (Math.abs(steer - A.steer) >= RCD_CONFIG.actionStepDeg &&
               now - A.steerAt >= RCD_CONFIG.actionGateMs) {
      A.steer = steer;
      A.steerAt = now;
      const human = Math.round(((S.cfg.steerCenter - steer) / Math.max(1, S.cfg.fullLockDeg)) * 100);
      const dir = human === 0 ? "回中" : human < 0 ? "左" + Math.abs(human) : "右" + human;
      rcdLog("[" + source + "] 转向 " + dir + " → 舵机 " + steer + "°");
    }
  }

  /* 云台按"边沿"记：开始动一条、停下来一条（带最终角度）。
     摇杆按住时云台是持续爬升的，按帧记会刷屏，而现场真正想知道的就是"它动没动、停在哪"。 */
  function rcdLogGimbalMotion(moving, source) {
    const A = S.action;
    if (A.gimbalMoving === undefined) A.gimbalMoving = false;
    if (moving && !A.gimbalMoving) {
      A.gimbalMoving = true;
      rcdLog("[" + source + "] 云台开始动（右摇杆）");
    } else if (!moving && A.gimbalMoving) {
      A.gimbalMoving = false;
      rcdLog("[" + source + "] 云台停 水平 " + S.gimbal.pan + " / 俯仰 " + S.gimbal.tilt);
    }
  }

  /* 按下/松开与"摇杆从静止到动了"的边沿：这一条最直接回答"键/轴匹配上没有" */
  function rcdLogGpEdges(gp) {
    const a = S.cfg.axes;
    const A = S.action;
    [["rt", "RT 前进"], ["lt", "LT 倒车"]].forEach((pair) => {
      const idx = a[pair[0]];
      const v = rcdButton(gp, idx, 0);
      const prev = A.btnOn[pair[0]];
      // 施密特触发：按下要 ≥0.5，松开要掉到 0.35 以下——否则扳机停在 0.5 会"按下/松开"来回刷
      const on = prev ? v > 0.35 : v >= 0.5;
      if (prev === undefined) {
        A.btnOn[pair[0]] = on;
        return;
      }
      if (on !== prev) {
        A.btnOn[pair[0]] = on;
        rcdLog("[手柄] " + pair[1] + " buttons[" + idx + "] " + (on ? "按下" : "松开"));
      }
    });
    [["steer", "转向", 1], ["pan", "云台水平", 2], ["tilt", "云台俯仰", 2]].forEach((pair) => {
      const idx = a[pair[0]];
      const v = rcdDeadzone(rcdAxis(gp, idx, 0), S.cfg.deadzone * pair[2]);
      const on = v !== 0;
      if (A.axisOn[pair[0]] === undefined) {
        A.axisOn[pair[0]] = on;
        return;
      }
      if (on !== A.axisOn[pair[0]]) {
        A.axisOn[pair[0]] = on;
        rcdLog("[手柄] " + pair[1] + " axes[" + idx + "] " +
          (on ? "动了 (" + v.toFixed(2) + ")" : "回中"));
      }
    });
  }

  function rcdResetActionLog() {
    S.action = { thrBand: undefined, steer: null, steerAt: 0, gimbalMoving: undefined, axisOn: {}, btnOn: {} };
    S.gpPrevAxes = [];
    S.gpPrevButtons = [];
    S.gpLastChange = null;
  }

  /* ======================================================================
   * 11. 进入 / 退出驾驶模式
   * ==================================================================== */
  function rcdHideOfficial() {
    const seen = [];
    RCD_HIDE_SELECTORS.forEach((sel) => {
      Array.prototype.slice.call(document.querySelectorAll(sel)).forEach((el) => {
        if (seen.indexOf(el) >= 0) return;
        seen.push(el);
        S.hidden.push({ el: el, style: el.getAttribute("style") });
        el.style.display = "none";
      });
    });
    if (!document.getElementById("videoElement")) {
      rcdLog("警告：未找到 #videoElement，视频可能被遮挡");
    }
  }

  function rcdRestoreOfficial() {
    S.hidden.forEach((it) => {
      try {
        // 原值可能是 null（没有 style 属性）或 ""（有但是空的）——两种都还原成"没有属性"
        if (!it.style) it.el.removeAttribute("style");
        else it.el.setAttribute("style", it.style);
      } catch (_) {}
    });
    S.hidden = [];
  }

  function rcdEnter() {
    if (S.active) return;
    S.active = true;

    // 官方手柄循环若正在跑：先借官方开关关掉它，再置真做死手守卫
    S.prevOfficialFlag = rcdGetOfficialFlag();
    if (S.prevOfficialFlag && typeof S.orig.toggleGamepadControl === "function") {
      try {
        S.orig.toggleGamepadControl(); // 关掉官方循环（loop 下一帧自行退出）
      } catch (_) {}
      setTimeout(() => rcdSetOfficialFlag(true), 120);
    } else {
      rcdSetOfficialFlag(true);
    }

    rcdHideOfficial();
    rcdBuildPanel();
    rcdShowPanel(); // ★ 退出时把面板 display 设成了 none，这里必须显式还原，否则再进来只有按钮文字变
    rcdApplySavedPos(); // 上次拖到哪儿就还放哪儿（挡视线的位置不用每次重拖）
    rcdSelfCheck();
    rcdFillConfigInputs();
    rcdUpdateDeadbandHint();
    S.drive.throttle = 0;
    S.drive.steering = S.cfg.steerCenter;
    S.gimbal = { pan: S.cfg.gimbal.center, tilt: S.cfg.gimbal.center };
    rcdResetActionLog(); // 新一次驾驶：动作日志从零开始（否则第一条会是个没意义的"回零"）
    rcdResetThrottle(false);
    rcdResetSteer(false);
    rcdRenderDriveReadout();
    rcdRenderStatus();
    rcdSyncGamepadLabel();
    rcdAttachWs();
    rcdSetEntryLabel(true);
    rcdLog("已进入驾驶模式（未解锁时后端会拒绝 drive）");
    rcdLog("点官方【手柄控制】按钮接管手柄；【急停】立即归零");
  }

  function rcdExit() {
    if (!S.active) return;
    // ★ 先强制油门归零 + 方向回中并补发一帧，否则藏掉面板后车会保持原油门到 8s failsafe
    S.drive.throttle = 0;
    S.drive.steering = S.cfg.steerCenter;
    rcdResetThrottle(false);
    rcdResetSteer(false);
    rcdSendDrive(true);
    rcdSendGimbal(true);

    S.gamepadOn = false;
    S.active = false; // 让包装函数恢复工作（在还原之前先失效）
    rcdStopGpDebug(); // 面板要藏了，调试轮询也停掉
    rcdRestoreOfficial();
    rcdRestoreHooks();
    rcdSetOfficialFlag(S.prevOfficialFlag);
    rcdUpdateOfficialGamepadButton();

    const panel = document.getElementById(RCD_CONFIG.panelId);
    if (panel) panel.style.display = "none";
    const tab = document.getElementById("rcd-tab");
    if (tab) tab.style.display = "none"; // 退出驾驶模式后连拉手也不留
    S.collapsed = false;
    rcdSetEntryLabel(false);
    S.logs = [];
    const logEl = document.getElementById("rcd-log");
    if (logEl) logEl.textContent = "";
    rcdLog("已退出驾驶模式：油门已归零、方向已回中、官方控件已还原");
  }

  function rcdToggleMode() {
    if (S.active) rcdExit();
    else rcdEnter();
  }

  /* ======================================================================
   * 12. 入口按钮
   * ==================================================================== */
  function rcdSetEntryLabel(inDrive, el) {
    const btn = el || document.getElementById("rcd-entry");
    if (!btn) return;
    btn.textContent = inDrive ? "返回主页面" : "驾驶模式";
    btn.className = inDrive ? "warn" : "secondary";
    btn.title = inDrive ? "退出驾驶模式，还原官方界面" : "进入速度/手柄驾驶界面";
  }

  function rcdInjectEntryButton() {
    if (document.getElementById("rcd-entry")) return;
    const btn = document.createElement("button");
    btn.id = "rcd-entry";
    btn.type = "button";
    // 注意：这一步必须传元素本身——此刻按钮还没进 DOM，按 id 查是查不到的
    rcdSetEntryLabel(false, btn);
    btn.addEventListener("click", rcdToggleMode);
    const row = $(".top-right");
    if (row) {
      row.insertBefore(btn, row.firstChild);
    } else {
      // 官方页改版兜底：右上角没了就自己固定一个
      btn.style.position = "fixed";
      btn.style.top = "16px";
      btn.style.right = "16px";
      btn.style.zIndex = "130";
      document.body.appendChild(btn);
      rcdLog("未找到官方 .top-right，入口按钮改为固定右上角");
    }
  }

  /* ======================================================================
   * 13. 启动
   * ==================================================================== */
  function rcdBoot() {
    if (window.__rcdDriveLoaded) {
      if (window.console && console.warn) console.warn("[rcd] 已加载过，跳过重复注入");
      return;
    }
    window.__rcdDriveLoaded = true;
    S.cfg = rcdLoadConfig();
    rcdInjectCss();
    rcdInjectEntryButton();
    rcdInstallHooks(); // 提前装好：手柄按钮在任何时候点都不该走官方逻辑

    window.addEventListener("gamepadconnected", (e) => {
      const g = e.gamepad;
      rcdLog("检测到手柄：" + (g && g.id ? g.id : "未知") + "（axes " + ((g && g.axes) || []).length +
        " / buttons " + ((g && g.buttons) || []).length + "）");
      if (S.active) rcdLog("展开【手柄调试】先确认按键；确认后点官方【手柄控制】按钮开始用手柄开车");
    });
    window.addEventListener("gamepaddisconnected", rcdOnGamepadDisconnected);
    // 窗口变小后，之前拖到边上的面板可能跑到屏幕外 —— 重新夹回可见范围
    window.addEventListener("resize", () => {
      if (!S.active) return;
      const panel = document.getElementById(RCD_CONFIG.panelId);
      const left = panel ? parseInt(panel.style.left, 10) : NaN;
      if (panel && isNum(left)) rcdPlacePanel(left, parseInt(panel.style.top, 10) || 0);
      else rcdPlaceTab();
    });

    setInterval(rcdTick, RCD_CONFIG.uiTickMs);
    setInterval(rcdPing, RCD_CONFIG.pingMs);

    if (window.console && console.log) {
      console.log("[rcd] 驾驶模式补丁已加载 v" + RCD_CONFIG.version + "（" + RCD_CONFIG.marker + "）");
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", rcdBoot);
  } else {
    rcdBoot();
  }
})();
