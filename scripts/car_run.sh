#!/usr/bin/env bash
#
# D-Racer CAR-side launcher. Run this ON THE CAR (topst@...), not the PC.
#
# It starts the vehicle nodes (camera + low-level control) inside a tmux
# session so that if your SSH connection drops mid-drive, the launch keeps
# running on the car and the vehicle does NOT stop. Reconnect and re-attach
# to get your screen back.
#
# The car runs:   ros2 launch bisa vehicle.launch.py   (camera_node + control_node)
# The PC runs:    ros2 launch bisa driving.launch.py route_mode:=OUT
# Both machines must share the same ROS_DOMAIN_ID and LAN.
#
# Usage (on the car):
#   bash scripts/car_run.sh            # start (idempotent) then attach
#   bash scripts/car_run.sh start      # start only, do not attach
#   bash scripts/car_run.sh attach     # attach to the running session
#   bash scripts/car_run.sh status     # is it running?
#   bash scripts/car_run.sh stop       # stop the car safely (neutral throttle)
#
# Env overrides:
#   WS=~/D-Racer-Kit   ROS_DOMAIN_ID=7   WIFI_IFACE=wlan0   bash scripts/car_run.sh
#
set -euo pipefail

SESSION="${SESSION:-dracer}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
DOMAIN="${ROS_DOMAIN_ID:-0}"

# Default workspace = the repo this script lives in (…/scripts/car_run.sh -> repo root).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$(dirname "$SCRIPT_DIR")}"

info() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m!!\033[0m %s\n' "$*"; }

require_tmux() {
  if ! command -v tmux >/dev/null 2>&1; then
    warn "tmux is not installed. Install it first:  sudo apt install -y tmux"
    exit 1
  fi
}

disable_wifi_powersave() {
  # Best-effort: WiFi power-save makes SSH drop on idle. Never fatal.
  local iface="${WIFI_IFACE:-}"
  if [ -z "$iface" ] && command -v iw >/dev/null 2>&1; then
    iface="$(iw dev 2>/dev/null | awk '/Interface/{print $2; exit}')"
  fi
  [ -z "$iface" ] && return 0
  if sudo -n iw dev "$iface" set power_save off >/dev/null 2>&1; then
    info "WiFi power_save off on $iface"
  else
    warn "Could not auto-disable WiFi power_save on $iface."
    warn "  Run once manually:  sudo iw dev $iface set power_save off"
  fi
}

start() {
  require_tmux
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    info "Session '$SESSION' already running. Attach:  tmux attach -t $SESSION"
    return 0
  fi

  if [ ! -f "$WS/install/setup.bash" ]; then
    warn "Workspace not built: $WS/install/setup.bash missing."
    warn "  Build first:  cd $WS && colcon build --symlink-install"
    exit 1
  fi

  disable_wifi_powersave

  info "Starting vehicle.launch.py in tmux session '$SESSION'"
  info "  workspace   = $WS"
  info "  ROS distro  = $ROS_DISTRO_NAME"
  info "  ROS_DOMAIN_ID = $DOMAIN"

  # Build the env inside the pane via send-keys (avoids nested-quote issues).
  tmux new-session -d -s "$SESSION" -n vehicle
  tmux send-keys -t "$SESSION" "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash" C-m
  tmux send-keys -t "$SESSION" "source '${WS}/install/setup.bash'" C-m
  tmux send-keys -t "$SESSION" "export ROS_DOMAIN_ID=${DOMAIN} ROS_LOCALHOST_ONLY=0" C-m
  tmux send-keys -t "$SESSION" "ros2 launch bisa vehicle.launch.py" C-m

  info "Started. The car keeps running even if SSH drops."
  info "  Attach (watch logs): tmux attach -t $SESSION      (detach: Ctrl+b then d)"
  info "  Stop safely:         bash scripts/car_run.sh stop"
}

attach() {
  require_tmux
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    warn "No session '$SESSION'. Start it:  bash scripts/car_run.sh start"
    exit 1
  fi
  tmux attach -t "$SESSION"
}

status() {
  require_tmux
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    info "Session '$SESSION' is RUNNING."
  else
    info "Session '$SESSION' is NOT running."
  fi
}

stop() {
  require_tmux
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    info "Session '$SESSION' is not running."
    return 0
  fi
  # Ctrl+C the launch so control_node's shutdown applies neutral throttle,
  # give it a moment, then tear down the session.
  info "Sending Ctrl+C to stop the launch (control_node applies neutral throttle)..."
  tmux send-keys -t "$SESSION" C-c
  sleep 3
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  info "Stopped session '$SESSION'."
}

case "${1:-start-and-attach}" in
  start)            start ;;
  attach)           attach ;;
  status)           status ;;
  stop)             stop ;;
  start-and-attach) start; attach ;;
  *) warn "Unknown command: $1"; echo "Usage: $0 [start|attach|status|stop]"; exit 1 ;;
esac
