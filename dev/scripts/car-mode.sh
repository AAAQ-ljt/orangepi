#!/usr/bin/env bash
# ============================================================================
# car-mode.sh —— 小车工作模式切换（手动遥控 / 自动驾驶 / 急停 / 图传）
#
# 设计原则
#   1. 上电默认是"手动遥控"（opi-control + 推流 + 网页），本脚本负责切到"自动驾驶"；
#   2. 任何退出路径（正常结束 / Ctrl+C / 断线）都会**恢复手动模式**，不会把车留在
#      "既没有遥控、也没有自动驾驶"的中间态；
#   3. 进入自动驾驶前做体检并要人确认（电机会上电，安全第一）；
#   4. 只停"必须停"的服务：占用同一个摄像头的推流、会抢 PCA9685 的 opi-control、
#      会占音频的 talk-player；**另一路推流保留**，裁判仍能看到画面（规则要求回传）。
#
# 用法
#   car-mode.sh status                     查看当前模式与关键状态（只读）
#   car-mode.sh auto [选项]                进入自动驾驶（前台运行控制进程）
#   car-mode.sh manual                     回到手动遥控
#   car-mode.sh estop                      紧急停止（立刻归零，不恢复服务）
#   car-mode.sh stream <up|down|status>    图传推流控制
#   car-mode.sh logs [vision|control]      查看日志
#
# auto 选项
#   --dry-run         控制进程不使能动力（默认不带 --real），用于验证视觉/状态机
#   --camera N        我们使用哪一路摄像头（0=云台主摄 /dev/video0，2=下摄 /dev/video2）
#                     默认 2：巡线扫线用下摄；此时保留云台主摄推流给裁判看
#   --model PATH      RKNN 模型路径（不给则只跑扫线+发车检测）
#   --max-us N        电调最大脉宽（默认 1600，调试限速；注意 1500 停、≈1545 才起转）
#   --port N          UDP 端口（默认 5000）
#   --minimal         连不需要的服务也停掉（省 CPU，但裁判看不到画面）
#   --yes             跳过确认（仍会做体检）
# ============================================================================
set -euo pipefail

ROOT_DIR="/root/dev"
RUN_DIR="/run/car-mode"
LOG_DIR="$ROOT_DIR/logs"
STATUS_FILE="/tmp/smartcar_status.json"
MEDIA_ENV="/etc/default/smartcar-media"

# 摄像头角色（2026-09-16 实测确认）：0 = icspring = 云台主摄；2 = Global Shutter = 下摄（巡线用）
DEV_GIMBAL="/dev/video0"; DEV_DOWN="/dev/video2"
SVC_MAIN="ffmpeg-stream.service"; SVC_SUB="ffmpeg-stream-sub.service"

# 手动模式需要的服务（退出自动驾驶时全部拉起）
MANUAL_SERVICES=(
  opi-control.service
  talk-player.service
  nginx.service
  frpc.service
  frpc-ssh.service
  ffmpeg-stream.service
  ffmpeg-stream-sub.service
)

# ------------------------------------------------------------------ 输出工具
if [[ -t 1 ]]; then
  C_RED=$'\e[31m'; C_GRN=$'\e[32m'; C_YEL=$'\e[33m'; C_CYN=$'\e[36m'; C_BLD=$'\e[1m'; C_RST=$'\e[0m'
else
  C_RED=""; C_GRN=""; C_YEL=""; C_CYN=""; C_BLD=""; C_RST=""
fi
info()  { printf '%s[car-mode]%s %s\n' "$C_CYN" "$C_RST" "$*"; }
ok()    { printf '%s[  ok  ]%s %s\n' "$C_GRN" "$C_RST" "$*"; }
warn()  { printf '%s[ warn ]%s %s\n' "$C_YEL" "$C_RST" "$*"; }
err()   { printf '%s[ fail ]%s %s\n' "$C_RED" "$C_RST" "$*" >&2; }
die()   { err "$*"; exit 1; }

svc_active() { systemctl is-active --quiet "$1" 2>/dev/null; }
pid_alive()  { [[ -f "$1" ]] && kill -0 "$(cat "$1")" 2>/dev/null; }
read_pid()   { [[ -f "$1" ]] && cat "$1" 2>/dev/null || true; }

# ------------------------------------------------------------------ 基础检查
require_root() { [[ ${EUID} -eq 0 ]] || die "需要 root 权限（sudo bash $0 ...）"; }

mkdir -p "$RUN_DIR" "$LOG_DIR"

media_host() {
  # 从 /etc/default/smartcar-media 里解析出媒体服务器主机名（用于连通性自检）
  [[ -f "$MEDIA_ENV" ]] || return 0
  sed -n 's/^MEDIA_RTSP=.*@\([^:/]*\).*/\1/p' "$MEDIA_ENV" | head -1
}

media_reachable() {
  # 媒体服务器 8554 是否可达（不可达时不必反复重启 ffmpeg，省得刷日志烧 CPU）
  local host; host=$(media_host)
  [[ -n "$host" ]] || return 1
  timeout 5 bash -c "cat < /dev/null > /dev/tcp/$host/8554" 2>/dev/null
}

# ------------------------------------------------------------------ 我们的进程
stop_ours() {
  local killed=0
  # 先温和后强硬，避免留下半死不活的进程占着摄像头/PCA9685
  for name in control vision; do
    local pf="$RUN_DIR/$name.pid"
    if pid_alive "$pf"; then
      local pid; pid=$(cat "$pf")
      info "停止 $name 进程 (pid=$pid)"
      kill -TERM "$pid" 2>/dev/null || true
      for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
      kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
      killed=1
    fi
    rm -f "$pf"
  done
  # 兜底：按命令行特征清理（PID 文件丢失时；用绝对路径匹配，dry-run/real 都能命中）
  pkill -f "$ROOT_DIR/vision/vision_main.py" 2>/dev/null && killed=1 || true
  pkill -f "$ROOT_DIR/main.py" 2>/dev/null && killed=1 || true
  [[ $killed -eq 1 ]] && ok "已停止自动驾驶进程" || info "没有正在运行的自动驾驶进程"
}

restore_manual() {
  info "恢复手动遥控模式"
  local streams_ok=1
  local skipped=()
  media_reachable || streams_ok=0
  for svc in "${MANUAL_SERVICES[@]}"; do
    if [[ "$svc" == ffmpeg-stream* && $streams_ok -eq 0 ]]; then
      warn "跳过 $svc（媒体服务器 $(media_host):8554 不可达，避免无谓的重启循环）"
      skipped+=("$svc"); continue
    fi
    systemctl start "$svc" 2>/dev/null || true
  done
  sleep 2

  # 逐项核对（与台架脚本同一套做法）：恢复"远程控制阶段"必须可验证，不能只说"启动了"
  echo "--- 恢复结果逐项核对 ---"
  local all_ok=1
  for svc in "${MANUAL_SERVICES[@]}"; do
    if [[ " ${skipped[*]-} " == *" $svc "* ]]; then
      printf '  ⚠️  %-24s 跳过（媒体服务器不可达）\n' "${svc%.service}"
      continue
    fi
    if svc_active "$svc"; then
      printf '  ✅ %-24s active\n' "${svc%.service}"
    else
      printf '  ❌ %-24s 未启动\n' "${svc%.service}"
      all_ok=0
    fi
  done
  if [[ $all_ok -eq 1 ]]; then
    ok "已恢复**远程控制阶段**（可正常遥控/看图传）"
  else
    err "有服务未起来！请立刻检查：systemctl status <服务名>；手动遥控可能不可用"
  fi
}

# ------------------------------------------------------------------ 体检
preflight() {
  local camera_dev="$1" dry_run="$2" model="$3" port="$4"
  local errors=0

  [[ -f "$ROOT_DIR/main.py" ]] || { err "缺少 $ROOT_DIR/main.py（先同步代码）"; errors=$((errors+1)); }
  [[ -f "$ROOT_DIR/vision/vision_main.py" ]] || { err "缺少 $ROOT_DIR/vision/vision_main.py"; errors=$((errors+1)); }

  # python 依赖
  if python3 - <<'PY' >/dev/null 2>&1
import cv2, numpy  # noqa
PY
  then ok "python3 依赖（cv2/numpy）可用"
  else err "python3 缺 cv2/numpy"; errors=$((errors+1)); fi

  [[ -e "$camera_dev" ]] && ok "摄像头 $camera_dev 存在" || { err "摄像头 $camera_dev 不存在"; errors=$((errors+1)); }

  # 摄像头是否被别的进程占用（我们自己停掉的那路不算）
  if command -v fuser >/dev/null 2>&1 && [[ -e "$camera_dev" ]]; then
    local holder
    holder=$(fuser "$camera_dev" 2>/dev/null || true)
    [[ -n "$holder" ]] && warn "摄像头仍被占用：$holder（可能停服务后未释放）" || ok "摄像头空闲"
  fi

  # PCA9685（非 dry-run 才必须）
  # ⚠️ 不能在 opi-control 运行时检查：i2c 总线的地址选择是排他的，
  # 手动遥控正驱动 PCA9685 时，i2cdetect 的 ioctl(I2C_SLAVE) 会被 -EBUSY
  # 直接打断（stderr 被 2>/dev/null 吞掉，stdout 为空）→ 误报"未检测到"。
  # 2026-09-19 两次 --real 失败都是这个原因。所以：
  #   opi-control 在跑 → 跳过（权威检查移到 cmd_auto 停服之后做）；
  #   opi-control 没跑 → 就地检查。
  # 检测判据用 awk 精确匹配 "40: 40"——旧写法 grep '\b40\b' 会匹配行号"40:"，
  # 总线上没有设备也永远判在线（体检验尸都验不出来）。
  if [[ "$dry_run" -eq 0 ]]; then
    if command -v i2cdetect >/dev/null 2>&1; then
      if svc_active opi-control.service; then
        info "opi-control 运行中，PCA9685 检查移到停服后进行"
      else
        pca_ok=0
        for _try in 1 2 3; do
          if i2cdetect -y 5 2>/dev/null | awk '$1=="40:" && $2=="40"{f=1} END{exit !f}'; then
            pca_ok=1; break
          fi
          sleep 0.5
        done
        if [[ $pca_ok -eq 1 ]]; then ok "PCA9685 @0x40 在线"
        else err "PCA9685 未检测到（i2c-5，已重试 3 次）——检查接线/供电"
             errors=$((errors+1))
        fi
      fi
    else
      warn "没有 i2cdetect，跳过 PCA9685 检查"
    fi
  fi

  # 模型文件
  if [[ -n "$model" ]]; then
    [[ -f "$model" ]] && ok "模型存在：$model" || warn "模型不存在，将退化为纯扫线模式：$model"
  else
    info "未指定模型 → 只跑扫线 + 发车检测"
  fi

  # UDP 端口占用
  if ss -lun 2>/dev/null | grep -q ":$port\b"; then
    err "UDP 端口 $port 已被占用（可能有残留进程，先 car-mode.sh manual）"; errors=$((errors+1))
  else ok "UDP 端口 $port 空闲"; fi

  # 媒体服务器连通性（图传能不能推上去）
  local host; host=$(media_host)
  if [[ -n "$host" ]]; then
    if media_reachable; then
      ok "自建媒体服务器 $host:8554 可达"
    else
      warn "自建媒体服务器 $host:8554 不可达（图传推不上去；不影响自主跑车）"
    fi
  fi

  return $((errors > 0 ? 1 : 0))
}

confirm() {
  local msg="$1"
  printf '%s%s%s\n' "$C_BLD" "$msg" "$C_RST"
  printf '确认无误请输入 %sGO%s 继续（其他任意输入取消）：' "$C_BLD" "$C_RST"
  local ans; read -r ans
  [[ "$ans" == "GO" || "$ans" == "go" ]]
}

# ------------------------------------------------------------------ 子命令
cmd_status() {
  require_root
  local mode="未知/空闲"
  if pid_alive "$RUN_DIR/control.pid"; then
    mode="${C_GRN}自动驾驶中${C_RST}"
  elif svc_active opi-control.service; then
    mode="${C_CYN}手动遥控${C_RST}"
  fi
  echo "================= 小车模式与状态 ================="
  printf '当前模式      : %s\n' "$mode"
  printf '运行时长/负载 : %s  负载 %s\n' "$(uptime -p 2>/dev/null | sed 's/up //')" "$(cut -d' ' -f1-3 /proc/loadavg)"
  printf '温度          : %s\n' "$(awk '{printf "%.1f°C", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo '未知')"
  echo "--- 服务 ---"
  for svc in "${MANUAL_SERVICES[@]}"; do
    local st; st=$(systemctl is-active "$svc" 2>/dev/null | head -1)
    printf '  %-26s %s\n' "${svc%.service}" "${st:-unknown}"
  done
  echo "--- 我们的进程 ---"
  for name in vision control; do
    local pf="$RUN_DIR/$name.pid"
    if pid_alive "$pf"; then printf '  %-8s pid=%s 运行中\n' "$name" "$(cat "$pf")"
    else printf '  %-8s 未运行\n' "$name"; fi
  done
  echo "--- 摄像头占用 ---"
  for dev in "$DEV_MAIN" "$DEV_SUB"; do
    local holder; holder=$(fuser "$dev" 2>/dev/null || true)
    printf '  %-14s %s\n' "$dev" "${holder:-空闲}"
  done
  echo "--- 视觉状态（vision 进程写出的状态文件）---"
  if [[ -f "$STATUS_FILE" ]]; then
    python3 -c "import json,sys;d=json.load(open('$STATUS_FILE'));print('  fps=%s loop=%sms center=%s conf=%s board=%s released=%s'%(d.get('fps'),d.get('loop_ms'),d.get('center_x'),d.get('lane_confidence'),d.get('board_blocked'),d.get('start_released')))" 2>/dev/null || echo "  （文件存在但解析失败）"
  else
    echo "  无（视觉进程未运行过）"
  fi
  local host; host=$(media_host)
  echo "--- 图传 ---"
  printf '  媒体服务器 : %s\n' "${host:-未配置}"
  if [[ -n "$host" ]]; then
    if timeout 5 bash -c "cat < /dev/null > /dev/tcp/$host/8554" 2>/dev/null; then
      printf '  推流端口   : %s可达%s\n' "$C_GRN" "$C_RST"
    else
      printf '  推流端口   : %s不可达（检查安全组/网络）%s\n' "$C_YEL" "$C_RST"
    fi
  fi
  echo "================================================="
}

cmd_stream() {
  require_root
  case "${1:-status}" in
    up)
      info "拉起图传推流（自建服务器）"
      media_reachable || warn "媒体服务器 $(media_host):8554 当前不可达，推流会失败并重试（检查安全组/网络）"
      systemctl reset-failed "$SVC_MAIN" "$SVC_SUB" 2>/dev/null || true
      systemctl start "$SVC_MAIN" "$SVC_SUB"
      sleep 4
      for svc in "$SVC_MAIN" "$SVC_SUB"; do
        svc_active "$svc" && ok "$svc 运行中" || err "$svc 启动失败，看日志：car-mode.sh logs stream"
      done
      ;;
    down)
      info "停止图传推流"
      systemctl stop --no-block "$SVC_MAIN" "$SVC_SUB" || true
      sleep 2
      pkill -9 -f 'ffmpeg.*rtsp' 2>/dev/null || true
      ok "已停止"
      ;;
    status)
      for svc in "$SVC_MAIN" "$SVC_SUB"; do
        printf '  %-26s %s\n' "${svc%.service}" "$(systemctl is-active "$svc" 2>/dev/null || echo unknown)"
      done
      journalctl -u "${SVC_MAIN%.service}" -n 3 --no-pager 2>/dev/null | tail -3
      ;;
    *) die "用法：car-mode.sh stream <up|down|status>" ;;
  esac
}

cmd_logs() {
  case "${1:-control}" in
    vision)  tail -n "${2:-40}" "$LOG_DIR/vision.log" 2>/dev/null || echo "无 vision 日志" ;;
    control) tail -n "${2:-40}" "$LOG_DIR/control.log" 2>/dev/null || echo "无 control 日志" ;;
    stream)  journalctl -u "${SVC_MAIN%.service}" -n "${2:-40}" --no-pager ;;
    *) die "用法：car-mode.sh logs [vision|control|stream] [行数]" ;;
  esac
}

cmd_estop() {
  require_root
  warn "紧急停止：立即切断自动驾驶输出"
  # 先杀控制进程（它会 safe_stop），再杀视觉；最后用 opi-control 把 PWM 压回安全值
  stop_ours
  systemctl start opi-control.service 2>/dev/null || true
  ok "已归零。手动模式服务已恢复（如需彻底手动请执行 car-mode.sh manual）"
}

cmd_manual() {
  require_root
  stop_ours
  restore_manual
}

cmd_auto() {
  require_root
  local dry_run=1 camera=2 model="" max_us=1600 port=5000 minimal=0 skip_confirm=0 no_lane=0

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --dry-run) dry_run=1 ;;
      --real)    dry_run=0 ;;
      --camera)  camera="$2"; shift ;;
      --model)   model="$2"; shift ;;
      --max-us)  max_us="$2"; shift ;;
      --port)    port="$2"; shift ;;
      --no-lane) no_lane=1 ;;
      --minimal) minimal=1 ;;
      --yes|-y)  skip_confirm=1 ;;
      *) die "未知选项：$1" ;;
    esac
    shift
  done

  # 默认用下摄（巡线）；--camera 0 可切云台主摄
  local camera_dev="$DEV_DOWN" camera_svc="$SVC_SUB"
  if [[ "$camera" == "0" ]]; then camera_dev="$DEV_GIMBAL"; camera_svc="$SVC_MAIN"; fi

  echo "================= 进入自动驾驶 ================="
  printf '模式        : %s\n' "$([[ $dry_run -eq 1 ]] && echo "${C_YEL}DRY-RUN（电机不使能，仅验证视觉/状态机）${C_RST}" || echo "${C_RED}REAL —— 电机将上电${C_RST}")"
  printf '摄像头      : %s (index %s)\n' "$camera_dev" "$camera"
  printf '模型        : %s\n' "${model:-无（只跑扫线+发车检测）}"
  printf '电调上限    : %sus\n' "$max_us"
  printf 'UDP 端口    : %s\n' "$port"
  [[ $dry_run -eq 1 ]] && info "要真跑车请加 --real（默认 dry-run 是安全默认值）"
  echo "------------------------------------------------"

  # 清掉可能残留的上一轮进程（否则摄像头/PCA9685 会被占住）
  stop_ours

  info "体检中…"
  preflight "$camera_dev" "$dry_run" "$model" "$port" || die "体检未通过，已放弃（车保持手动模式）"

  if [[ $skip_confirm -eq 0 ]]; then
    echo "------------------------------------------------"
    [[ $dry_run -eq 1 ]] || warn "确认：四轮悬空或场地空旷，遥控/急停通道就绪，挡板已就位"
    confirm "即将停掉手动遥控与占用 ${camera_dev} 的推流，启动自动驾驶。" || { info "已取消，车保持手动模式"; return 0; }
  fi

  # ---- 切换环境 ----
  info "停止会冲突的服务"
  systemctl stop opi-control.service 2>/dev/null || true          # 抢 PCA9685，必须停
  systemctl stop talk-player.service 2>/dev/null || true          # 占音频，避免抢播报
  systemctl stop --no-block "$camera_svc" 2>/dev/null || true     # 占我们要用的摄像头
  if [[ $minimal -eq 1 ]]; then
    systemctl stop --no-block "$SVC_MAIN" "$SVC_SUB" 2>/dev/null || true
    systemctl stop nginx.service frpc.service 2>/dev/null || true
    warn "已启用 --minimal：裁判将看不到画面"
  else
    info "保留另一路推流给裁判看画面：$([[ "$camera_svc" == "$SVC_MAIN" ]] && echo "$SVC_SUB" || echo "$SVC_MAIN")"
  fi
  sleep 2

  # ---- 停服后复核 PCA9685（真跑才需要；权威检查）----
  # 此刻 opi-control 已停、i2c 总线归我们独占，检测结果才可信（原因见 preflight 注释）。
  # 失败则恢复服务、保持手动模式并放弃——绝不允许在没确认舵机/电调驱动在线的情况下上动力。
  if [[ $dry_run -eq 0 ]] && command -v i2cdetect >/dev/null 2>&1; then
    pca_ok=0
    for _try in 1 2 3; do
      if i2cdetect -y 5 2>/dev/null | awk '$1=="40:" && $2=="40"{f=1} END{exit !f}'; then
        pca_ok=1; break
      fi
      sleep 0.5
    done
    if [[ $pca_ok -ne 1 ]]; then
      err "PCA9685 未检测到（i2c-5，停服后复核 3 次仍失败）——检查接线/供电，车保持手动模式"
      stop_ours
      restore_manual
      die "停服后复核未通过，已放弃上动力"
    fi
    ok "PCA9685 @0x40 在线（停服后复核通过）"
  fi

  # ---- 起视觉 ----
  local vision_args=(--udp-ip 127.0.0.1 --udp-port "$port" --camera "$camera" --status-file "$STATUS_FILE")
  [[ -n "$model" ]] && vision_args+=(--model "$model")
  [[ $no_lane -eq 1 ]] && vision_args+=(--no-lane)

  info "启动视觉进程（日志：$LOG_DIR/vision.log）"
  : > "$STATUS_FILE" 2>/dev/null || true
  {
    echo "===== $(date '+%F %T') vision 启动 ====="
  } >> "$LOG_DIR/vision.log"
  PYTHONPATH="$ROOT_DIR" nohup python3 -u "$ROOT_DIR/vision/vision_main.py" "${vision_args[@]}" \
    >> "$LOG_DIR/vision.log" 2>&1 &
  echo $! > "$RUN_DIR/vision.pid"

  # 等视觉进入工作状态（状态文件出现 = 已经在出帧）
  local waited=0
  while [[ $waited -lt 15 ]]; do
    pid_alive "$RUN_DIR/vision.pid" || break
    [[ -s "$STATUS_FILE" ]] && break
    sleep 0.5; waited=$((waited+1))
  done
  if pid_alive "$RUN_DIR/vision.pid" && [[ -s "$STATUS_FILE" ]]; then
    ok "视觉进程已出帧（等待 $((waited/2))s）"
  elif pid_alive "$RUN_DIR/vision.pid"; then
    warn "视觉进程在跑但还没写出状态文件，继续（可 car-mode.sh logs vision 查看）"
  else
    stop_ours; restore_manual
    die "视觉进程启动失败，已恢复手动模式。请看 $LOG_DIR/vision.log"
  fi

  # ---- 起控制（前台，随时 Ctrl+C）----
  local ctrl_args=(--port "$port" --max-us "$max_us")
  [[ $dry_run -eq 0 ]] && ctrl_args+=(--real --arm)

  # 退出时无论何种原因（正常结束/Ctrl+C/断线）都恢复手动模式，绝不把车留在中间态
  cleanup() {
    local code=$?
    trap - EXIT INT TERM HUP
    echo
    warn "退出自动驾驶"
    stop_ours
    restore_manual
    exit "$code"
  }
  trap cleanup EXIT INT TERM HUP

  echo "------------------------------------------------"
  ok "进入自动驾驶：$([[ $dry_run -eq 1 ]] && echo 'DRY-RUN' || echo 'REAL')"
  info "按 Ctrl+C 结束并恢复手动模式；另开终端可看：car-mode.sh status / logs"
  echo "------------------------------------------------"
  set +e
  # 用进程替换把输出同时送到终端和日志；$! 拿到的是 python 的 PID（这样才能被干净地停掉）
  PYTHONPATH="$ROOT_DIR" python3 -u "$ROOT_DIR/main.py" "${ctrl_args[@]}" \
    > >(tee -a "$LOG_DIR/control.log") 2>&1 &
  local ctrl_pid=$!
  echo "$ctrl_pid" > "$RUN_DIR/control.pid"
  wait "$ctrl_pid"
  set -e
}

usage() { sed -n '2,32p' "$0"; exit "${1:-0}"; }

main() {
  local cmd="${1:-status}"
  [[ $# -gt 0 ]] && shift || true
  case "$cmd" in
    status)  cmd_status "$@" ;;
    auto)    cmd_auto "$@" ;;
    manual)  cmd_manual "$@" ;;
    estop)   cmd_estop "$@" ;;
    stream)  cmd_stream "$@" ;;
    logs)    cmd_logs "$@" ;;
    help|-h|--help) usage 0 ;;
    *) err "未知命令：$cmd"; usage 1 ;;
  esac
}

main "$@"
