#!/usr/bin/env bash
# 实验室循迹测试包装（2026-10-15）——速度 1560us（今天现场实测稳住的档）+ stdout 全落盘。
#
# 用法（车上，root）：
#   bash /root/dev/scripts/lab_lane_test.sh                    # ① 只看读数（不动电机，8s）
#   bash /root/dev/scripts/lab_lane_test.sh --run              # ② 真跑（输入 GO 确认；轮子架空或场地空旷）
#   bash /root/dev/scripts/lab_lane_test.sh --run --kd 0.24    # ③ A/B：加大阻尼
#   bash /root/dev/scripts/lab_lane_test.sh --run --kp 0.10 --kd 0.25   # ④ A/B：对齐上届阻尼比（Kd/Kp=2.5）
#   bash /root/dev/scripts/lab_lane_test.sh --run --speed-us 1580       # ⑤ 复现"越快越蛇行"（对照档）
#   bash /root/dev/scripts/lab_lane_test.sh --run --cone --speed-us 1585 --cone-speed-us 1575 \
#        --save-every-s 0.2                                            # ⑥ 锥桶相位（锥桶摆车前 2~3m）
#
# 其余参数原样透传给 `scripts/lane_final_test.py`（--target-x / --single-max-s / --max-seconds …）。
# 本脚本只做三件事：打印本轮参数（含今天新上的闸门档/裁剪档）、把 stdout 另存一份、
# 跑完自动指出会话目录并 tail 出 CSV 末尾行（复盘再也不用靠回忆参数）。
#
# 安全（AGENTS.md §1.1）：只有 `--run` 才动车（= 显式 --allow-motion）；
#   必须在轮子架空、或场地空旷且有人持急停的情况下跑。
# ★ 想停（2026-10-17）：① 首选 `touch /dev/shm/lane_stop`（任何通道都能发；lane_final_test.py 主循环每帧查，
#   走与 Ctrl+C 相同的安全收尾）② 车本地 `bash /root/dev/scripts/stop_run.sh`（或 `--file`）
#   ③ 本窗口 Ctrl+C（**只在"本窗口前台手敲这个脚本"时有效**）④ 断电。
set -euo pipefail

DEV=/root/dev
LANE="$DEV/scripts/lane_final_test.py"
LOG_DIR="$DEV/logs"
RUN=0
PROBE_S=8
PASSTHRU=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run) RUN=1; shift ;;
    --probe-seconds) PROBE_S="$2"; shift 2 ;;
    *) PASSTHRU+=("$1"); shift ;;
  esac
done

if [[ $EUID -ne 0 ]]; then
  echo "❌ 要用 root 跑（sudo bash $0 ...）"
  exit 1
fi
if [[ ! -f "$LANE" ]]; then
  echo "❌ 找不到 $LANE（先同步 dev/ 到车上）"
  exit 1
fi

STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$LOG_DIR/lane_${STAMP}.log"
mkdir -p "$LOG_DIR"

confirm() {
  printf '%s\n' "$1"
  printf '确认无误请输入 GO 继续（其他任意输入取消）：'
  local ans; read -r ans
  [[ "$ans" == "GO" || "$ans" == "go" ]]
}

# 参数一览（从 settings 读今天的档，避免"以为在用新档其实没同步"）
PARAMS=$(python3 - "$DEV" <<'PY' || echo "（读 settings 失败）"
import sys
sys.path.insert(0, sys.argv[1])
from config import settings
print(f"闸门 ROI={settings.START_GATE_ROI}（底边 0.80=车盖不入闸门）")
print(f"面积判据 {settings.START_BLUE_AREA_THRESH} 需主导度≥{settings.START_BLUE_DOMINANCE_THRESH}"
      f" 或面积≥{settings.START_BLUE_AREA_THRESH_BIG}")
print(f"画面裁剪 保留 y<{settings.FRAME_CROP_LINE_Y}（车盖不入画）")
PY
)

# 有效速度：透传参数里的 --speed-us 优先（否则默认 1560）——打印必须跟实际一致，别硬编码
# （2026-10-15 现场踩到：GO 提示印"速度 1560us"，实际跑的是 1600us，容易误判）
SPEED=1560
for ((i = 0; i < ${#PASSTHRU[@]}; i++)); do
  if [[ "${PASSTHRU[$i]}" == "--speed-us" && $((i + 1)) -lt ${#PASSTHRU[@]} ]]; then
    SPEED="${PASSTHRU[$((i + 1))]}"
  fi
done

# 真跑时命令行里要带的速度（有透传就不注入默认值，避免 "1560 ... 1600" 这种叠写）
SPEED_ARGS=(--speed-us 1560)
for ((i = 0; i < ${#PASSTHRU[@]}; i++)); do
  if [[ "${PASSTHRU[$i]}" == "--speed-us" ]]; then
    SPEED_ARGS=()
  fi
done

echo "================= 实验室循迹测试 ================="
echo "$PARAMS"
echo "本轮速度  : ${SPEED}us（透传优先；默认 1560 = 白天实测稳，1585~1590 需 kp0.10+kd0.5）"
echo "本机日志  : $LOG"
echo "额外参数  : ${PASSTHRU[*]:-（无）}"
echo "================================================="

# ---------------- ① 只看读数（不动电机） ----------------
echo
echo "[1/2] 只读检查（--no-motor，${PROBE_S}s：看读数、别碰电机）"
echo "      · 车摆正在车道里；闸门会打印'等待蓝板…'的蓝色读数（面积/主导度）"
echo "      · 绿框=左线、红框=右线、黄圈=中心；err 越接近 0 越说明 target_x 对"
set +e
python3 -u "$LANE" --no-motor --max-seconds "$PROBE_S" \
  --save-dir "$LOG_DIR/lane_probe_${STAMP}" ${PASSTHRU[@]+"${PASSTHRU[@]}"} 2>&1 | tee -a "$LOG"
PROBE_RC=${PIPESTATUS[0]}
set -e
if [[ $PROBE_RC -ne 0 ]]; then
  echo "⚠️ 只读检查退出码 $PROBE_RC（看上面输出；读不到摄像头就查 car-mode.sh status）"
fi

if [[ $RUN -eq 0 ]]; then
  echo
  echo "✅ 只读检查结束（没有动电机）。确认读数没问题后，加 --run 跑真车："
  echo "    bash $0 --run ${PASSTHRU[*]:-}"
  echo "日志：$LOG"
  exit 0
fi

# ---------------- ② 真跑（显式 --allow-motion） ----------------
echo
if ! confirm "⚠️ 即将【真正动电机】：速度 ${SPEED}us、闸门在（蓝板挡一下再移开才发车）。
   请确认：轮子已架空、或场地空旷且有人持急停；手边随时能断电。
   ★ 想停（2026-10-17 起按这个顺序）：
     ① 首选·停止文件 —— 任何通道都能发（另一个会话/串口/网页终端）：
        touch /dev/shm/lane_stop
     ② 车本地会话里：bash /root/dev/scripts/stop_run.sh（= 按 PID 发 TERM）
     ③ 本窗口 Ctrl+C —— **只在'本窗口前台手敲这个脚本'时有效**（启动日志会打印自检行）；
        无 PTY / 后台 & / nohup / 在别的窗口按，都到不了车。
     挡板能停是靠丢线（会滑一段），闸门发车后即闩锁、**没有'再见板即停'**。"; then
  echo "已取消（未动车）。"
  exit 1
fi

echo "[2/2] 真跑：蓝板挡到下摄正前方 → 移开 = 发车；想停：touch /dev/shm/lane_stop（首选）"
set +e
python3 -u "$LANE" --allow-motion ${SPEED_ARGS[@]+"${SPEED_ARGS[@]}"} \
  --max-seconds "${MAX_SECONDS:-60}" \
  --save-dir "$LOG_DIR/lane_run_${STAMP}" ${PASSTHRU[@]+"${PASSTHRU[@]}"} 2>&1 | tee -a "$LOG"
RC=${PIPESTATUS[0]}
set -e

echo
echo "================= 收尾 ================="
NEWEST=$(ls -dt "$LOG_DIR"/lane_run* "$LOG_DIR"/lane_2* 2>/dev/null | head -1 || true)
if [[ -n "${NEWEST:-}" && -f "$NEWEST/run.csv" ]]; then
  echo "会话目录：$NEWEST"
  echo "--- run.csv 末尾 5 行 ---"
  tail -5 "$NEWEST/run.csv"
  echo "--- 相位分布 ---"
  python3 - "$NEWEST/run.csv" <<'PY'
import csv, sys
from collections import Counter
rows = list(csv.DictReader(open(sys.argv[1], newline="", encoding="utf-8")))
print(" ".join(f"{p}:{n}" for p, n in Counter(r["phase"] for r in rows).items()))
PY
fi
echo "stdout 日志：$LOG"
echo "退出码：$RC（0=正常结束；1=读不到摄像头/异常；看日志末几行）"
exit "$RC"
