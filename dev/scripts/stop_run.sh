#!/usr/bin/env bash
# ============================================================================
# stop_run.sh —— 我们的「停车按钮」（2026-10-16 新增；现场专用）
#
# 用法（车上 root）：
#   bash /root/dev/scripts/stop_run.sh              # 停：SIGTERM → 等它自己安全收尾 → 核验
#   bash /root/dev/scripts/stop_run.sh --file       # ★ 停止文件通道（不依赖 PID/命令行；主循环每帧查）
#   bash /root/dev/scripts/stop_run.sh --status     # 只看谁在跑（不动任何东西）
#   bash /root/dev/scripts/stop_run.sh --kill-now   # 不等宽限（TERM 上次没反应/明显卡死时）
#   bash /root/dev/scripts/stop_run.sh --no-restore # 停止后不去恢复手动阶段（默认会恢复）
#   bash /root/dev/scripts/stop_run.sh --park       # 【未实现】结束流程：蓝板→停车位（见下）
#
# 为什么要有它（2026-10-16 现场教训）：
#   · **Ctrl+C 不一定到得了车**：现场那次跑在没有分配 PTY 的通道里，按 Ctrl+C 只掐了本地那一段，
#     车的进程根本没收到信号（7 份日志里一行「[BENCH] 收到信号」都没有）→ 只能抓车/挡天空触发护栏，
#     再关电调重启。按键不可靠，**要把"停车"变成一个能明确调用的命令**。
#   · `car-mode.sh estop/manual` 的 stop_ours() **杀不到 lane_final_test.py / bench_test.py**
#     （它们不写 /run/car-mode/*.pid，也不匹配那两条 pkill 模式）——estop 写一次 1500us，
#     下一帧（~30Hz）我们又把 1600us 写回去；而且 estop 会启 opi-control 与我们**抢写 PCA9685**。
#   · 看门狗（esc_watchdog.py）只覆盖"主控进程死/卡死"（心跳停），**不是停车按钮**。
#
# 停止顺序（任何一步都往安全侧走）：
#   ① SIGTERM → 跑动脚本自己的收尾链：driver.safe_stop()（电调 1500us、舵机回中）→ 释放 PCA9685
#      → 恢复 opi-control（这一步内部有 ~2s 等待，所以宽限要给够）；
#   ② 宽限（默认 8s）还没退 → SIGKILL → 心跳停 → **看门狗 1s 内强写 1500us**；
#   ③ 硬杀路径下再用 `car-mode.sh manual` 把手动阶段与两路推流恢复到位（可 --no-restore 跳过）；
#   ④ 核验：目标都没了 / 看门狗在不在 / opi-control 与推流状态 / 最新会话日志有没有「收尾·结束」行。
#
# ★ --park（将来）：正式比赛的"结束"不是被外力停住，而是**让车自己开进停车位**：
#     ① 下摄识别**蓝色挡板** → ② 判左右车位（IPM + 板接地点）→ ③ 五阶段入库（doc/开发文档.md 的 P1-4）。
#   那一步现在**还没实现**：--park 会**明确报错、什么都不做**（免得现场以为按了就自动停车）。
#   落地时要在这里做三件事：把"走停车流程"的请求发给跑动脚本（信号或 UDP 命令）→ 等它报"入库完成"
#   → 再回到本脚本的 ③④ 收尾核验。在那之前，本脚本只做"停"这一件事。
#
# 安全：本脚本只做"停止 + 核验"，**绝不会启动电机**；没有目标在跑时它是空操作。
# ============================================================================
set -euo pipefail

ROOT_DIR="${CAR_DEV_ROOT:-/root/dev}"
SCRIPTS="$ROOT_DIR/scripts"
LOGS="$ROOT_DIR/logs"
WAIT_S="${STOP_RUN_WAIT_S:-8}"          # TERM 之后等它自己收尾的秒数（脚本内部要 ~2s 恢复服务）
# ★ 停止文件（2026-10-17）：与 lane_final_test.py 的 --stop-file 同一路径——**文件存在 = 安全停车**。
#   它不依赖 PID/命令行特征，主循环每帧查；`--file` 模式就是 touch 它。
STOP_FILE="${STOP_RUN_FILE:-/dev/shm/lane_stop}"

# 被本脚本接管的入口（按命令行特征匹配；用绝对路径避免误伤同名文件）
# ⚠️ 匹配的是**绝对路径**——照 doc 里的调用方式（`sudo python3 /root/dev/scripts/lane_final_test.py …`）没问题；
#    若你是先 `cd` 进目录再 `python3 lane_final_test.py`，命令行里没有这个绝对路径 → 本脚本找不到它，
#    那种情况手工停：`pkill -f lane_final_test.py`。
TARGETS=(
  "$SCRIPTS/lane_final_test.py"         # 循迹（含 --cone 相位）
  "$SCRIPTS/bench_test.py"              # 台架
  "$ROOT_DIR/vision/vision_main.py"     # A 路径视觉进程
  "$ROOT_DIR/main.py"                   # A 路径控制进程
)

MODE="stop"; RESTORE=1; VIA_FILE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --status)     MODE="status"; shift ;;
    --file)       VIA_FILE=1; shift ;;
    --kill-now)   WAIT_S=0; shift ;;
    --wait)       WAIT_S="$2"; shift 2 ;;
    --no-restore) RESTORE=0; shift ;;
    --park)       MODE="park"; shift ;;
    -h|--help)    sed -n '3,20p' "$0"; exit 0 ;;
    *) echo "❌ 不认识的参数：$1（-h 看用法）"; exit 2 ;;
  esac
done

info() { printf '%s\n' "$*"; }
warn() { printf '⚠️  %s\n' "$*"; }
ok()   { printf '✅ %s\n' "$*"; }
bad()  { printf '❌ %s\n' "$*"; }

# 车上一定有（procps；car-mode.sh 也在用 pkill）——缺了就明确报错，别静默变成空操作
if [[ "$MODE" != "park" ]]; then
  command -v pgrep >/dev/null 2>&1 || {
    bad "找不到 pgrep（procps 包）→ 本脚本无法判断谁在跑，什么都没做。"
    info "   装：apt-get install -y procps（车上本来就有）"
    exit 3
  }
fi

# ---------------------------------------------------------------- 找目标
_pids_of() {                             # 参数=模式；输出 pid（每行一个），不含本脚本自己
  local pat="$1"
  # 只收**纯数字**：pgrep 输出被别的东西污染时，宁可当"没找到"，也不能拿垃圾去 kill/判定
  pgrep -f "$pat" 2>/dev/null | tr -d '\r' | grep -E '^[0-9]+$' | grep -vx "$$" || true
}

_scan_pids() {                           # 所有目标模式扫一遍（去重）——这是"还在不在"的**唯一裁判**
  local pat pid
  for pat in "${TARGETS[@]}"; do
    while read -r pid; do
      [[ -n "$pid" ]] && printf '%s\n' "$pid"
    done < <(_pids_of "$pat")
  done | sort -u
}

_scan_has() {                            # 该 pid 现在还能被扫到吗
  _scan_pids | grep -qx "$1"
}

_alive() {                               # ★ 僵尸不算"还在跑"（进程已经死了，只是父进程还没收尸）
  # 为什么不能用 kill -0 单独判：父进程若正阻塞在别处（现场就是：wrapper/sudo 在等整条流水线），
  # 退出的子进程会**变成僵尸**——`kill -0` 对僵尸照样返回成功（2026-10-16 车上自测踩到：
  # 进程其实已经安全退出，脚本却判成"没退"、白升一级 SIGKILL）。僵尸的 /proc/<pid>/cmdline 是空的，
  # pgrep -f 读不到它 ⇒ **用重扫来判活**才是对的。
  local pid="$1"
  kill -0 "$pid" 2>/dev/null || return 1
  _scan_has "$pid" || return 1
  return 0
}

_running() {                             # 输出 "pid cmdline" 行（供人看 + 供后面 kill）
  local pat
  for pat in "${TARGETS[@]}"; do
    while read -r pid; do
      [[ -n "$pid" ]] || continue
      printf '%s %s\n' "$pid" "$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-110 \
        || echo '(cmdline 读不到)')"
    done < <(_pids_of "$pat")
  done
  return 0
}

_watchdog_pid() { pgrep -f "esc_watchdog\.py" 2>/dev/null | head -1 || true; }

_svc_state() {                           # 只在有 systemctl 的机器（车上）有意义
  command -v systemctl >/dev/null 2>&1 || { echo "n/a"; return 0; }
  if systemctl is-active --quiet "$1" 2>/dev/null; then echo "active"; else echo "未启动"; fi
}

_log_tail_verdict() {                     # 最新会话日志里有没有"安全收尾"的痕迹
  local newest
  newest="$(ls -t "$LOGS"/lane_*.log 2>/dev/null | head -1 || true)"
  [[ -n "$newest" ]] || { info "   （$LOGS 里没有 lane_*.log，跳过日志核验）"; return 0; }
  info "   最新会话日志：$newest"
  if grep -q "\[BENCH\] 收尾" "$newest" 2>/dev/null; then
    ok "   日志里有「[BENCH] 收尾」→ 这趟走了完整安全收尾（电调归零/服务已恢复）"
  else
    warn "   日志里**没有**「[BENCH] 收尾」→ 进程是被硬杀的（车已由看门狗/硬停保障，"
    info "   但这条日志缺 result，复盘时按'被打断'对待）"
  fi
}

# ---------------------------------------------------------------- --status
if [[ "$MODE" == "status" ]]; then
  info "===== 谁在跑 ====="
  mapfile -t RUNNING < <(_running)
  if [[ ${#RUNNING[@]} -eq 0 ]]; then
    info "   （没有我们的跑动进程在运行）"
  else
    printf '   %s\n' "${RUNNING[@]}"
  fi
  wd="$(_watchdog_pid)"
  info "===== 相关状态 ====="
  info "   看门狗（esc_watchdog.py）：$([[ -n "$wd" ]] && echo "在跑 (pid=$wd)" || echo "不在")"
  info "   停止文件（$STOP_FILE）：$([[ -e "$STOP_FILE" ]] && echo '**存在**（主循环下一帧就会停车）' || echo '不在（正常）')"
  info "   opi-control.service：$(_svc_state opi-control.service)（active = 网页摇杆/键盘现在能让车动）"
  info "   ffmpeg-stream(.service/-sub)：$(_svc_state ffmpeg-stream.service) / $(_svc_state ffmpeg-stream-sub.service)"
  exit 0
fi

# ---------------------------------------------------------------- --park（未实现）
if [[ "$MODE" == "park" ]]; then
  bad "--park 还没实现：停车流程（蓝板识别 → IPM 判左右 → 五阶段入库，P1-4）尚未落地。"
  info "   ⇒ **本次什么都没做**（车还在按原来的方式跑）。"
  info "   要立即停车：不带参数再跑一次本脚本（或 bash $0）。"
  info "   实现时要做：给跑动脚本发「走停车流程」的请求（信号/UDP）→ 等它报「入库完成」→ 再收尾核验。"
  exit 2
fi

# ---------------------------------------------------------------- 停
info "===== stop_run：停止我们的跑动进程（ROOT_DIR=$ROOT_DIR）====="
mapfile -t RUNNING < <(_running)
if [[ ${#RUNNING[@]} -eq 0 ]]; then
  info "没有我们的跑动进程在运行 —— 空操作，不碰任何硬件。"
  info "   （顺带：opi-control.service = $(_svc_state opi-control.service)"
  info "     看门狗：$([[ -n "$(_watchdog_pid)" ]] && echo 在跑 || echo 不在)）"
  exit 0
fi

info "找到 ${#RUNNING[@]} 个目标："
printf '   %s\n' "${RUNNING[@]}"
pids=(); mapfile -t pids < <(_scan_pids)          # 去重后的 pid 列表（同一个进程被两条模式命中时只算一个）

# ★ ①a 停止文件通道（--file）：不依赖 PID/命令行特征 —— lane_final_test.py 主循环每帧查它。
#    先只发这一次、给它 WAIT_S 秒自己走完收尾；没人响应（脚本没这个开关/主循环卡死）再升级到 TERM。
if (( VIA_FILE == 1 )); then
  info "①a 停止文件：touch $STOP_FILE（lane_final_test.py 主循环每帧查 → 走正常安全收尾）"
  : > "$STOP_FILE" || warn "   写不进 $STOP_FILE（权限/不存在？）→ 直接走 TERM"
  survivors=()
  deadline=$(( $(date +%s) + WAIT_S ))
  while (( $(date +%s) < deadline )); do
    survivors=()
    for pid in "${pids[@]}"; do _alive "$pid" && survivors+=("$pid"); done
    [[ ${#survivors[@]} -eq 0 ]] && break
    sleep 0.5
  done
  if [[ ${#survivors[@]} -eq 0 ]]; then
    ok "①a 停止文件生效：目标都已安全退出（电调归零/舵机回中/服务已恢复）"
  else
    warn "①a 停止文件没人响应（${#survivors[@]} 个还在）——可能不是循迹脚本、或主循环卡住；升级到 TERM"
    VIA_FILE=0
  fi
fi

if (( VIA_FILE == 0 )); then
  info "① SIGTERM → 让它们自己安全收尾（电调 1500us、舵机回中、恢复 opi-control）"
  kill -TERM "${pids[@]}" 2>/dev/null || true
fi

survivors=()
if (( WAIT_S > 0 )); then
  info "   等最多 ${WAIT_S}s（脚本内部恢复服务要 ~2s，别提前杀）…"
  deadline=$(( $(date +%s) + WAIT_S ))
  while (( $(date +%s) < deadline )); do
    survivors=()
    for pid in "${pids[@]}"; do _alive "$pid" && survivors+=("$pid"); done
    [[ ${#survivors[@]} -eq 0 ]] && break
    sleep 0.5
  done
else
  for pid in "${pids[@]}"; do _alive "$pid" && survivors+=("$pid"); done
fi

if [[ ${#survivors[@]} -eq 0 ]]; then
  ok "② 全部自己退出了（走了完整安全收尾）"
  hard=0
else
  warn "② ${#survivors[@]} 个没退（pid: ${survivors[*]}）→ SIGKILL"
  kill -KILL "${survivors[@]}" 2>/dev/null || true
  sleep 1.5
  hard=1
  wd="$(_watchdog_pid)"
  if [[ -n "$wd" ]]; then
    ok "   看门狗在（pid=$wd）→ 心跳停，1s 内由它强写 1500us/回中（这是它该干的活）"
  else
    warn "   看门狗不在 → 没有独立兜底；若车还在动就断电"
  fi
  if (( RESTORE == 1 )) && [[ -f "$SCRIPTS/car-mode.sh" ]]; then
    info "③ 硬杀路径：用 car-mode.sh manual 恢复手动阶段与两路推流"
    bash "$SCRIPTS/car-mode.sh" manual || warn "car-mode.sh manual 返回非零（看它上面的输出）"
  elif (( RESTORE == 0 )); then
    warn "③ 跳过恢复（--no-restore）——记住：opi-control 与推流可能还是停的"
  else
    warn "③ 找不到 $SCRIPTS/car-mode.sh，跳过恢复 —— 手动跑：bash $SCRIPTS/car-mode.sh manual"
  fi
fi

# ---------------------------------------------------------------- 核验
# ★ 不要只看"我收集到的那些 pid 还在不在"（pid 可能被回收/复用、pgrep 输出可能被污染）——
#   按**同一把尺子重新扫一遍**才算数（2026-10-16 自测里踩到过"假成功"）。
info "④ 核验（重新扫一遍，不看旧 pid）："
mapfile -t rescan < <(_running)
if [[ ${#rescan[@]} -eq 0 ]]; then
  ok "   重新扫描：没有我们的跑动进程了"
else
  bad "   重新扫描仍有目标在跑："
  printf '   %s\n' "${rescan[@]}"
fi
info "   opi-control.service = $(_svc_state opi-control.service)"
info "   ffmpeg-stream(.service/-sub) = $(_svc_state ffmpeg-stream.service) / $(_svc_state ffmpeg-stream-sub.service)"
_log_tail_verdict

info "⚠️ 手动阶段一旦是 active，网页摇杆/键盘**现在就能让车动** —— 别把手机/网页停在摇杆页。"
if [[ ${#rescan[@]} -gt 0 ]]; then exit 1; fi
if (( hard == 1 )); then ok "stop_run 完成（走了硬杀路径）"; else ok "stop_run 完成（温和停）"; fi
exit 0
