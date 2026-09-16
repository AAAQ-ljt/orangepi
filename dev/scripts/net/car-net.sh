#!/usr/bin/env bash
# ============================================================================
# car-net.sh —— 小车网络总管（蜂窝优先 / WiFi 账本兜底 / 服务器端点可切换）
#
# 设计目标（为什么这么设计）：
#   1. **上电即用**：一根 systemd 服务在开机后执行 `auto`，先试流量卡，不行才试 WiFi；
#   2. **只占一条上行**：任一路连上就不再折腾另一路（蜂窝路由 metric 更优，
#      并把 WiFi 的 autoconnect 关掉），避免两条链路互相抢路由；
#   3. **换卡能用**：APN 按列表轮询（cmnet/3gnet/ctnet…），插别的运营商的卡也能连；
#   4. **优雅断开**：`cellular down` 会释放 QMI 客户端与 CID、清地址与路由，
#      不留"僵尸 CID"（这正是之前 car-dial 反复失败的原因）；
#   5. **可解耦**：所有可变项在 car-net.conf（端点/策略）与 wifi-ledger.conf（WiFi 账本），
#      换服务器/换热点都不需要改脚本。
#
# 用法：
#   car-net.sh status                   当前状态（蜂窝/WiFi/路由/隧道/图传）
#   car-net.sh auto                     上电策略：蜂窝优先 → WiFi 兜底（systemd 调这个）
#   car-net.sh cellular up|down|status  @ 蜂窝拨号 / 优雅断开 / 状态
#   car-net.sh wifi sync|export|list|up|down
#   car-net.sh apply                    把 car-net.conf 应用到系统（媒体地址 + frp 配置）
#   car-net.sh tunnel restart           重启隧道（frpc-ssh）
# ============================================================================
set -uo pipefail

CONF_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
CONF="$CONF_DIR/car-net.conf"
LOG_DIR="/root/dev/logs"
LOG="$LOG_DIR/car-net.log"

# 默认值（可被 car-net.conf 覆盖）
PREFER_CELLULAR=yes; WIFI_FALLBACK=yes
CELL_TIMEOUT=30; WIFI_TIMEOUT=45
WIFI_LEDGER="$CONF_DIR/wifi-ledger.conf"
APN_LIST="cmnet,3gnet,ctnet"
CELL_METRIC=100; WIFI_METRIC=600
VENDOR_DIAL_FALLBACK=yes; MODEM_HARD_RESET=yes; MODEM_USB_ID=""
MEDIA_RTSP=""; FRP_SERVER_ADDR=""; FRP_SERVER_PORT=7000; FRP_TOKEN=""
FRP_SSH_PORT=2222; FRP_WEB_PORT=8080; FRP_DOMAIN_PORT=""
WIFI_IF=""; WWAN_IF="wwan0"; QMI_DEV="/dev/cdc-wdm0"

# shellcheck disable=SC1090
[[ -f "$CONF" ]] && source "$CONF"

if [[ -t 1 ]]; then
  R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; C=$'\e[36m'; B=$'\e[1m'; N=$'\e[0m'
else
  R=""; G=""; Y=""; C=""; B=""; N=""
fi
info() { printf '%s[net]%s %s\n' "$C" "$N" "$*"; }
ok()   { printf '%s[ ok ]%s %s\n' "$G" "$N" "$*"; }
warn() { printf '%s[warn]%s %s\n' "$Y" "$N" "$*"; }
err()  { printf '%s[fail]%s %s\n' "$R" "$N" "$*" >&2; }
log()  { mkdir -p "$LOG_DIR"; printf '[%s] %s\n' "$(date '+%F %T')" "$*" >> "$LOG"; }

require_root() { [[ ${EUID} -eq 0 ]] || { err "需要 root（sudo bash $0 ...）"; exit 1; }; }

# ------------------------------------------------------------------ 小工具
iface_ip() { ip -4 -o addr show "$1" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1; }

# 出网测试：优先用 curl（走指定接口），没有 curl 就退回 /dev/tcp
net_ok() {
  local iface="${1:-}" target="${2:-223.5.5.5}"
  if command -v curl >/dev/null 2>&1; then
    if [[ -n "$iface" ]]; then
      curl -s --max-time 6 --interface "$iface" -o /dev/null "http://$target" 2>/dev/null \
        || curl -s --max-time 6 --interface "$iface" -o /dev/null "http://www.baidu.com" 2>/dev/null
    else
      curl -s --max-time 6 -o /dev/null "http://www.baidu.com" 2>/dev/null
    fi
  else
    timeout 6 bash -c "cat </dev/null >/dev/tcp/$target/80" 2>/dev/null
  fi
}

default_iface() { ip route show default 2>/dev/null | awk '{print $5}' | head -1; }

cellular_ip()  { iface_ip "$WWAN_IF"; }
wifi_active()  { nmcli -t -f DEVICE,STATE 2>/dev/null | grep -q "^${WIFI_IF}:connected"; }

# ------------------------------------------------------------------ 蜂窝
VENDOR_DIALER="/root/SIM8200_for_RPI/Goonline/simcom-cm"   # 厂商拨号器（对本模组已验证可用）
MODEM_USB_ID=""            # 例：2-1（USB 总线2端口1）；留空则自动探测（按 idVendor 1e0e:9001）

# 找到 5G 模组的 USB 设备路径（sysfs 名，如 2-1）
find_modem_usb() {
  [[ -n "$MODEM_USB_ID" ]] && { echo "$MODEM_USB_ID"; return 0; }
  local d
  for d in /sys/bus/usb/devices/*-*; do
    [[ -f "$d/idVendor" && -f "$d/idProduct" ]] || continue
    if [[ "$(cat "$d/idVendor" 2>/dev/null)" == "1e0e" ]]; then
      basename "$d"; return 0
    fi
  done
  return 1
}

# 硬复位模组：USB 解绑/绑定（等价于重新插拔），能清掉所有卡死的 QMI 状态。
# 这是"兜底也要能连上"的最后手段——只要模组硬件没坏，复位后就能重新拨号。
modem_hard_reset() {
  local usb
  usb=$(find_modem_usb) || { warn "找不到 5G 模组的 USB 设备（idVendor 1e0e），跳过硬复位"; return 1; }
  info "硬复位模组：USB $usb unbind/bind"
  log "modem hard reset ($usb)"
  # 先把可能占着 QMI 的进程清掉，否则复位后又被占用
  pkill -f "simcom-cm" 2>/dev/null || true
  pkill -f "qmicli" 2>/dev/null || true
  sleep 2
  echo "$usb" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null || { warn "unbind 失败（缺权限？）"; return 1; }
  sleep 4
  echo "$usb" > /sys/bus/usb/drivers/usb/bind 2>/dev/null || { warn "bind 失败"; return 1; }
  # 等 QMI 控制口重新出现（模组重启 + udev 建节点，通常 10~25s）
  local waited=0
  while (( waited < 45 )); do
    if [[ -e "$QMI_DEV" && -d "/sys/class/net/$WWAN_IF" ]]; then
      ok "模组已重新枚举（$QMI_DEV 就绪）"
      sleep 3
      [[ -e /sys/class/net/$WWAN_IF/qmi/raw_ip ]] && echo Y > "/sys/class/net/$WWAN_IF/qmi/raw_ip" 2>/dev/null
      ip link set "$WWAN_IF" up 2>/dev/null
      return 0
    fi
    sleep 2; waited=$((waited+2))
  done
  warn "复位后 45s 内没等到 $QMI_DEV / $WWAN_IF"
  return 1
}

# 厂商拨号器：SIM8262E-M2 的 QMI 时序比较特殊，官方这个小程序是"已验证可用"的路径。
# 它可能自带默认 APN，所以换卡时不一定成功 —— 失败了再走我们的 APN 轮询。
# 厂商拨号器（兜底路径）：SIM8262E-M2 的官方小程序，对本模组验证过可用，
# 但它可能只用固定 APN，所以放在 APN 轮询之后。
vendor_dial() {
  [[ -x "$VENDOR_DIALER" ]] || { warn "找不到厂商拨号器：$VENDOR_DIALER"; return 1; }
  if pgrep -f "simcom-cm" >/dev/null 2>&1; then
    info "厂商拨号器已在运行（可能卡住了，先重启它）"
    pkill -f "simcom-cm" 2>/dev/null || true
    sleep 3
  fi
  info "启动厂商拨号器（$VENDOR_DIALER）"
  ( cd "$(dirname "$VENDOR_DIALER")" && nohup ./simcom-cm >/var/log/simcom-cm.log 2>&1 & )
  local waited=0
  while (( waited < 30 )); do
    if [[ -n "$(cellular_ip)" ]] && net_ok "$WWAN_IF"; then
      ok "厂商拨号成功：$(cellular_ip)"
      log "vendor dial OK ip=$(cellular_ip)"
      return 0
    fi
    sleep 2; waited=$((waited+2))
  done
  warn "厂商拨号 30s 内未出网（日志 /var/log/simcom-cm.log）"
  log "vendor dial FAILED"
  return 1
}

# APN 轮询拨号：不同运营商的卡 APN 不同，逐个试，第一个出网的就算成功
apn_dial() {
  local apn ip gw dns settings out why
  IFS=',' read -r -a apns <<< "$APN_LIST"
  for apn in "${apns[@]}"; do
    apn="$(echo "$apn" | tr -d ' ')"
    [[ -z "$apn" ]] && continue
    info "尝试 APN=$apn"
    out=$(timeout 12 qmicli -d "$QMI_DEV" -p --wds-start-network="apn=$apn,ip-type=4" --client-no-release-cid 2>&1)
    if grep -qiE 'error|cannot|failed' <<< "$out" && ! grep -qi 'already' <<< "$out"; then
      why=$(tail -1 <<< "$out")
      warn "APN=$apn 启动失败：$why"
      log "apn=$apn start failed: $why"
      continue
    fi
    sleep 3
    settings=$(timeout 12 qmicli -d "$QMI_DEV" -p --wds-get-current-settings 2>&1)
    ip=$(grep -oP 'IPv4 address:\s*\K[0-9.]+' <<< "$settings" | head -1)
    gw=$(grep -oP 'IPv4 gateway address:\s*\K[0-9.]+' <<< "$settings" | head -1)
    dns=$(grep -oP 'IPv4 primary DNS:\s*\K[0-9.]+' <<< "$settings" | head -1)
    if [[ -z "$ip" || -z "$gw" ]]; then
      warn "APN=$apn 没拿到地址（IP=$ip GW=$gw）"
      log "apn=$apn no address"
      continue
    fi
    ip addr flush dev "$WWAN_IF" 2>/dev/null
    ip addr replace "$ip/32" dev "$WWAN_IF" || continue
    ip route replace default via "$gw" dev "$WWAN_IF" metric "$CELL_METRIC" onlink || true
    [[ -n "$dns" ]] && resolvectl dns "$WWAN_IF" "$dns" 2>/dev/null
    sleep 2
    if net_ok "$WWAN_IF"; then
      ok "蜂窝已连接：APN=$apn IP=$ip GW=$gw DNS=$dns"
      log "cellular OK apn=$apn ip=$ip gw=$gw"
      return 0
    fi
    warn "APN=$apn 有地址但出网失败"
    log "apn=$apn no internet"
  done
  return 1
}

cellular_up() {
  local waited=0
  log "cellular up 开始（APN 列表: $APN_LIST）"

  if [[ -n "$(cellular_ip)" ]] && net_ok "$WWAN_IF"; then
    ok "蜂窝已在线（$(cellular_ip)），无需重拨"
    log "cellular 已在线，跳过拨号"
    return 0
  fi

  if [[ ! -e "$QMI_DEV" ]]; then
    err "找不到 QMI 设备 $QMI_DEV（模组没插好/未上电？）"
    return 1
  fi
  # 模组要处于 raw_ip 模式才能用 QMI 拨号（厂商拨号器也是这么设的）
  [[ -e /sys/class/net/$WWAN_IF/qmi/raw_ip ]] && echo Y > "/sys/class/net/$WWAN_IF/qmi/raw_ip" 2>/dev/null
  ip link set "$WWAN_IF" up 2>/dev/null

  # ① 先做 APN 轮询：最快、换卡即用、不依赖厂商二进制
  if apn_dial; then return 0; fi

  # ② 实在不行再用厂商拨号器兜底（对本模组验证过，但它可能只会用某个固定 APN）
  if [[ "${VENDOR_DIAL_FALLBACK:-yes}" == "yes" ]]; then
    warn "APN 轮询失败，改用厂商拨号器兜底"
    vendor_dial && return 0
  fi

  # ③ 最后手段：硬复位模组再走一遍（等价重新插拔，清掉卡死的 QMI 状态）
  if [[ "${MODEM_HARD_RESET:-yes}" == "yes" ]]; then
    warn "仍未拨通，尝试硬复位模组……"
    if modem_hard_reset; then
      sleep 2
      apn_dial && return 0
      vendor_dial && return 0
    fi
  fi

  err "三条路径都没拨通（检查：卡是否插好/欠费/天线/模组供电；APN 可在 car-net.conf 里改）"
  log "cellular FAILED (apn+vendor+reset)"
  return 1
}

cellular_down() {
  log "cellular down（优雅断开）"
  if [[ ! -e "$QMI_DEV" ]]; then
    info "没有 QMI 设备，只清理地址与路由"
  else
    # 1) 停止网络（拿不到 CID 也不要紧，尽力而为）
    local cid
    cid=$(timeout 10 qmicli -d "$QMI_DEV" -p --wds-get-current-settings 2>&1 \
          | grep -oP 'CID:\s*\K[0-9]+' | head -1 || true)
    [[ -z "$cid" ]] && cid=$(timeout 10 qmicli -d "$QMI_DEV" -p --wds-noop 2>&1 \
          | grep -oP 'CID:\s*\K[0-9]+' | head -1 || true)
    if [[ -n "$cid" ]]; then
      timeout 15 qmicli -d "$QMI_DEV" -p --wds-stop-network="$cid" --client-no-release-cid \
        >/dev/null 2>&1 && ok "已停止数据会话（CID=$cid）" || warn "停止数据会话失败（CID=$cid），继续清理"
      timeout 10 qmicli -d "$QMI_DEV" -p --wds-release-client-id="$cid" >/dev/null 2>&1 \
        && ok "已释放 CID=$cid" || warn "释放 CID=$cid 失败（下次拨号会自动重新分配）"
    else
      warn "拿不到 CID（可能本来就没会话）"
    fi
  fi
  ip addr flush dev "$WWAN_IF" 2>/dev/null || true
  ip route del default dev "$WWAN_IF" 2>/dev/null || true
  resolvectl revert "$WWAN_IF" 2>/dev/null || true
  ok "蜂窝已断开（上电后 car-net.sh auto 会重新拨号）"
}

# ------------------------------------------------------------------ WiFi 账本
ledger_entries() {
  [[ -f "$WIFI_LEDGER" ]] || return 0
  grep -vE '^\s*(#|$)' "$WIFI_LEDGER" | while IFS='|' read -r ssid pwd prio static; do
    ssid="$(echo "${ssid:-}" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
    pwd="$(echo "${pwd:-}" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
    prio="$(echo "${prio:-50}" | tr -d ' ')"
    static="$(echo "${static:-}" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
    [[ -z "$prio" ]] && prio=50
    [[ -z "$ssid" ]] && continue
    printf '%s|%s|%s|%s\n' "$ssid" "$pwd" "$prio" "$static"
  done
}

wifi_iface() {
  if [[ -n "$WIFI_IF" ]]; then echo "$WIFI_IF"; return; fi
  nmcli -t -f DEVICE,TYPE dev status 2>/dev/null | awk -F: '$2=="wifi"{print $1; exit}'
}

wifi_sync() {
  local iface; iface=$(wifi_iface)
  [[ -z "$iface" ]] && { err "找不到 WiFi 网卡"; return 1; }
  local n=0
  while IFS='|' read -r ssid pwd prio static; do
    local exists=no
    nmcli -t -f NAME con show 2>/dev/null | grep -Fxq "$ssid" && exists=yes
    if [[ -z "$pwd" && "$exists" == "no" ]]; then
      warn "跳过 $ssid：账本里没有密码"
      continue
    fi
    if [[ "$exists" == "no" ]]; then
      nmcli con add type wifi con-name "$ssid" ifname "$iface" ssid "$ssid" \
        wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$pwd" \
        ipv4.route-metric "$WIFI_METRIC" connection.autoconnect no >/dev/null 2>&1 \
        && ok "已创建连接：$ssid（优先级 $prio）" || { warn "创建失败：$ssid"; continue; }
    fi
    # 静态地址（可选第 4 列，格式 IP/前缀,网关）——热点 DHCP 不给地址时用这个绕过去
    if [[ -n "$static" ]]; then
      local addr="${static%%,*}" gw="${static#*,}"
      if nmcli con modify "$ssid" ipv4.method manual ipv4.addresses "$addr" \
           ${gw:+ipv4.gateway "$gw"} ipv4.route-metric "$WIFI_METRIC" >/dev/null 2>&1; then
        ok "$ssid 使用静态地址 $addr${gw:+（网关 $gw）}"
      else
        warn "$ssid 静态地址写入失败：$static"
      fi
    else
      nmcli con modify "$ssid" ipv4.method auto ipv4.route-metric "$WIFI_METRIC" connection.autoconnect no >/dev/null 2>&1
    fi
    n=$((n+1))
  done < <(ledger_entries)
  info "账本同步完成，共处理 $n 条"
}

wifi_export() {
  local out=""
  while IFS= read -r name; do
    [[ -z "$name" ]] && continue
    local type pwd
    type=$(nmcli -t -g connection.type con show "$name" 2>/dev/null)
    [[ "$type" != "802-11-wireless" ]] && continue
    pwd=$(nmcli -s -t -g 802-11-wireless-security.psk con show "$name" 2>/dev/null | head -1)
    out+="${name}|${pwd}|50"$'\n'
  done < <(nmcli -t -f NAME con show 2>/dev/null)
  {
    echo "# 由 car-net.sh wifi export 于 $(date '+%F %T') 导出（可直接编辑）"
    printf '%s' "$out"
  } > "$WIFI_LEDGER"
  ok "已导出到 $WIFI_LEDGER（含 $(grep -cvE '^\s*(#|$)' "$WIFI_LEDGER") 条）"
}

wifi_list() {
  echo "--- 账本（$WIFI_LEDGER）---"
  ledger_entries | while IFS='|' read -r ssid pwd prio; do
    printf '  %-24s 优先级 %-4s 密码 %s\n' "$ssid" "$prio" "$([[ -n $pwd ]] && echo 已设置 || echo 未设置)"
  done
  echo "--- 当前可见热点 ---"
  nmcli -t -f SSID,SIGNAL,SECURITY dev wifi list 2>/dev/null | head -12 | sed 's/^/  /'
}

# 按账本优先级挑一个"当前能搜到"的热点
pick_best_ssid() {
  local visible
  visible=$(nmcli -t -f SSID dev wifi list 2>/dev/null | sort -u)
  local best="" best_prio=9999 ssid pwd prio static
  while IFS='|' read -r ssid pwd prio static; do
    grep -Fxq "$ssid" <<< "$visible" || continue
    if (( prio < best_prio )); then best="$ssid"; best_prio=$prio; fi
  done < <(ledger_entries)
  [[ -n "$best" ]] && echo "$best"
}

# 上次连接失败的原因（从 NM 日志里抓，便于判断是密码错还是 DHCP 没给地址）
wifi_fail_reason() {
  journalctl -u NetworkManager --since '2 min ago' --no-pager 2>/dev/null \
    | grep -oE "\-> failed \(reason '[a-z0-9-]+'\)" | tail -1 | sed "s/-> failed (reason //; s/)//"
}

# 判定 WiFi 是否真的可用：**拿到 IPv4 地址**才算（不能用 NM 的 connected 状态——
# DHCP 拿到地址后 NM 还会做连通性检查、停在 ip-check 十几秒，那时其实已经能用了）
wifi_up_ok() {
  local iface="$1" ip st
  ip=$(iface_ip "$iface")
  [[ -n "$ip" ]] || return 1
  st=$(nmcli -t -f DEVICE,STATE dev status 2>/dev/null | awk -F: -v d="$iface" '$1==d{print $2}')
  case "$st" in
    connected|ip-check|ip-config) return 0 ;;
    *) return 1 ;;
  esac
}

wifi_up() {
  local iface; iface=$(wifi_iface)
  [[ -z "$iface" ]] && { err "找不到 WiFi 网卡"; return 1; }

  local attempt best
  for attempt in 1 2 3; do
    best=$(pick_best_ssid)
    if [[ -z "$best" ]]; then
      warn "账本里没有当前能搜到的热点（可用 wifi list 查看）"
      return 1
    fi
    info "第 $attempt/3 次尝试连接：$best"
    timeout $((WIFI_TIMEOUT / 2 + 10)) nmcli --wait "$((WIFI_TIMEOUT / 2))" con up "$best" >/dev/null 2>&1 || true

    local waited=0
    while (( waited < WIFI_TIMEOUT )); do
      if wifi_up_ok "$iface"; then
        ok "WiFi 已连接（$(iface_ip "$iface")）"
        return 0
      fi
      sleep 2; waited=$((waited+2))
    done
    local why; why=$(wifi_fail_reason)
    warn "第 $attempt 次失败${why:+（$why）}"
    log "wifi attempt $attempt on '$best' failed ${why:+($why)}"
    # 只有在确实没拿到地址时才断开重试，避免把刚拿到地址的连接掐掉
    if ! wifi_up_ok "$iface"; then
      nmcli dev disconnect "$iface" >/dev/null 2>&1 || true
    fi
    sleep 3
  done
  err "WiFi 重试 3 次仍未连上；若是 'ip-config-unavailable'（热点没给 DHCP 地址），"
  err "可给账本该热点加第 4 列静态地址，例如：Aaaq|密码|10|10.129.7.50/24,10.129.7.169"
  return 1
}

# 策略：只占一条上行（蜂窝优先；有蜂窝就关掉 WiFi 自动连，反之亦然）
apply_policy() {
  local iface; iface=$(wifi_iface)
  local cell_ip; cell_ip=$(cellular_ip)
  if [[ -n "$cell_ip" ]] && net_ok "$WWAN_IF"; then
    ok "上行 = 蜂窝（$cell_ip）；关闭 WiFi 自动连接，避免两条链路互抢"
    while IFS= read -r name; do
      nmcli con modify "$name" connection.autoconnect no >/dev/null 2>&1
    done < <(nmcli -t -f NAME,TYPE con show 2>/dev/null | awk -F: '$2=="802-11-wireless"{print $1}')
    nmcli dev disconnect "$iface" >/dev/null 2>&1
  elif [[ -n "$(iface_ip "$iface")" ]]; then
    ok "上行 = WiFi（$(iface_ip "$iface")）"
    while IFS= read -r name; do
      nmcli con modify "$name" connection.autoconnect yes >/dev/null 2>&1
    done < <(nmcli -t -f NAME,TYPE con show 2>/dev/null | awk -F: '$2=="802-11-wireless"{print $1}')
  else
    warn "当前没有任何上行链路"
  fi
  # 路由优先级兜底（蜂窝 100 < WiFi 600，蜂窝优先）
  local gw
  gw=$(ip route show default dev "$WWAN_IF" 2>/dev/null | awk '{print $3}' | head -1)
  [[ -n "$gw" ]] && ip route replace default via "$gw" dev "$WWAN_IF" metric "$CELL_METRIC" onlink 2>/dev/null
}

# ------------------------------------------------------------------ 应用服务器配置
apply_server() {
  [[ -n "$MEDIA_RTSP" ]] || { err "car-net.conf 里 MEDIA_RTSP 为空"; return 1; }
  cat > /etc/default/smartcar-media <<EOF
# 由 car-net.sh apply 生成（$(date '+%F %T')）—— 改配置请改 $CONF
MEDIA_RTSP=$MEDIA_RTSP
EOF
  ok "已写入 /etc/default/smartcar-media"

  cat > /root/frp/frpc-ssh.ini <<EOF
# 由 car-net.sh apply 生成（$(date '+%F %T')）—— 改配置请改 $CONF
[common]
server_addr = $FRP_SERVER_ADDR
server_port = $FRP_SERVER_PORT
token = $FRP_TOKEN

[ssh]
type = tcp
local_ip = 127.0.0.1
local_port = 22
remote_port = $FRP_SSH_PORT

[web-ui]
type = tcp
local_ip = 127.0.0.1
local_port = 80
remote_port = $FRP_WEB_PORT
EOF
  if [[ -n "$FRP_DOMAIN_PORT" ]]; then
    cat >> /root/frp/frpc-ssh.ini <<EOF

[web-domain]
type = tcp
local_ip = 127.0.0.1
local_port = 80
remote_port = $FRP_DOMAIN_PORT
EOF
  fi
  ok "已写入 /root/frp/frpc-ssh.ini（服务器 $FRP_SERVER_ADDR:$FRP_SERVER_PORT）"

  systemctl restart frpc-ssh.service 2>/dev/null && ok "frpc-ssh 已重启" || warn "frpc-ssh 重启失败"
  for svc in ffmpeg-stream.service ffmpeg-stream-sub.service; do
    systemctl is-enabled --quiet "$svc" 2>/dev/null || continue
    systemctl reset-failed "$svc" 2>/dev/null || true
    systemctl restart "$svc" 2>/dev/null && ok "$svc 已重启" || warn "$svc 重启失败"
  done
  log "apply_server -> $FRP_SERVER_ADDR / $MEDIA_RTSP"
}

# ------------------------------------------------------------------ 上电策略
cmd_auto() {
  require_root
  log "=== auto 开始（prefer_cellular=$PREFER_CELLULAR）==="
  local tried_cell=0
  if [[ "$PREFER_CELLULAR" == "yes" ]]; then
    tried_cell=1
    info "① 先试流量卡（最多等 ${CELL_TIMEOUT}s）"
    local waited=0
    while (( waited < CELL_TIMEOUT )); do
      if [[ -n "$(cellular_ip)" ]] && net_ok "$WWAN_IF"; then
        ok "蜂窝已就绪（$(cellular_ip)）"
        break
      fi
      sleep 2; waited=$((waited+2))
    done
    if [[ -z "$(cellular_ip)" ]] || ! net_ok "$WWAN_IF"; then
      warn "等待超时，尝试主动拨号"
      cellular_up || true
    fi
  fi

  if [[ -n "$(cellular_ip)" ]] && net_ok "$WWAN_IF"; then
    apply_policy
    ok "上电联网完成（蜂窝）"
    log "auto 完成：蜂窝"
    return 0
  fi

  # 蜂窝不可用 → WiFi 兜底
  if [[ "$WIFI_FALLBACK" != "yes" ]]; then
    warn "蜂窝不可用且配置禁止 WiFi 兜底"
    return 1
  fi
  info "② 蜂窝不可用，改用 WiFi 账本兜底"
  wifi_sync >/dev/null 2>&1 || true
  if wifi_up; then
    apply_policy
    ok "上电联网完成（WiFi）"
    log "auto 完成：WiFi"
    return 0
  fi
  err "蜂窝与 WiFi 都没连上 —— 请看 $LOG，或插串口排查"
  log "auto 失败：无上行"
  return 1
}

# ------------------------------------------------------------------ 状态
cmd_status() {
  echo "================= 网络状态 ================="
  local cell_ip wifi_ip def
  cell_ip=$(cellular_ip); wifi_ip=$(iface_ip "$(wifi_iface)"); def=$(default_iface)
  local cell_out="" wifi_state="未连接"
  if [[ -n "$cell_ip" ]]; then net_ok "$WWAN_IF" && cell_out="(出网 OK)"; fi
  [[ -n "$wifi_ip" ]] && wifi_state="已连"
  printf '默认出口      : %s\n' "${def:-无}"
  printf '蜂窝 %-8s : %s\n' "$WWAN_IF" "${cell_ip:-无地址} $cell_out"
  printf 'WiFi %-8s : %s\n' "$(wifi_iface)" "${wifi_ip:-无地址} ($wifi_state)"
  printf '策略          : 蜂窝优先=%s WiFi兜底=%s（蜂窝路由 metric %s / WiFi %s）\n' \
    "$PREFER_CELLULAR" "$WIFI_FALLBACK" "$CELL_METRIC" "$WIFI_METRIC"
  echo "--- 隧道与服务 ---"
  for svc in frpc-ssh nginx opi-control; do
    printf '  %-12s %s\n' "$svc" "$(systemctl is-active "$svc" 2>/dev/null | head -1)"
  done
  echo "--- 媒体服务器 ---"
  local mh; mh=$(echo "$MEDIA_RTSP" | sed -n 's/.*@\([^:/]*\).*/\1/p')
  if [[ -n "$mh" ]]; then
    timeout 5 bash -c "cat </dev/null >/dev/tcp/$mh/8554" 2>/dev/null \
      && printf '  %s:8554 %s可达%s\n' "$mh" "$G" "$N" \
      || printf '  %s:8554 %s不可达%s\n' "$mh" "$Y" "$N"
  fi
  echo "============================================"
}

usage() { sed -n '2,30p' "$0"; exit "${1:-0}"; }

main() {
  local cmd="${1:-status}"; shift || true
  case "$cmd" in
    status)   cmd_status ;;
    auto)     cmd_auto ;;
    cellular) case "${1:-status}" in
                up) require_root; cellular_up ;;
                down) require_root; cellular_down ;;
                recover) require_root; modem_hard_reset && { sleep 2; apn_dial || vendor_dial; } ;;
                reset) require_root; modem_hard_reset ;;
                status) printf '蜂窝 %s: %s\n' "$WWAN_IF" "$(cellular_ip || echo 无地址)" ;;
                *) err "用法：car-net.sh cellular up|down|recover|reset|status"; exit 1 ;;
              esac ;;
    wifi)     case "${1:-list}" in
                sync)   require_root; wifi_sync ;;
                export) require_root; wifi_export ;;
                list)   wifi_list ;;
                up)     require_root; wifi_up ;;
                down)   require_root; nmcli dev disconnect "$(wifi_iface)" >/dev/null 2>&1; ok "WiFi 已断开" ;;
                *) err "用法：car-net.sh wifi sync|export|list|up|down"; exit 1 ;;
              esac ;;
    apply)    require_root; apply_server ;;
    policy)   require_root; apply_policy ;;
    tunnel)   require_root; systemctl restart frpc-ssh.service && ok "frpc-ssh 已重启" ;;
    help|-h|--help) usage 0 ;;
    *) err "未知命令：$cmd"; usage 1 ;;
  esac
}
main "$@"
