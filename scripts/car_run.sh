#!/usr/bin/env bash
#
# D-Racer autonomous launcher. Run this ON THE CAR (topst@...), not the PC.
# The ROS launch runs inside a detached tmux session, so closing or losing SSH
# does not stop the vehicle.
#
# Usage (on the car):
#   bash scripts/car_run.sh start                         # onboard, OUT route
#   bash scripts/car_run.sh start route_mode:=IN          # onboard, IN route
#   bash scripts/car_run.sh attach                        # live logs (detach: Ctrl+b, d)
#   bash scripts/car_run.sh status
#   bash scripts/car_run.sh stop                          # graceful stop
#
# Optional environment overrides:
#   ROS_DOMAIN_ID=2 SESSION=dracer WS=~/D-Racer-Kit bash scripts/car_run.sh start
#   CAR_ROS_LOCALHOST_ONLY=0 bash scripts/car_run.sh start  # PC GUI/viz tuning only
#   LAUNCH_FILE=vehicle.launch.py bash scripts/car_run.sh start
#
set -euo pipefail

SESSION="${SESSION:-dracer}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
DOMAIN="${ROS_DOMAIN_ID:-2}"
LOCALHOST_ONLY="${CAR_ROS_LOCALHOST_ONLY:-1}"
LAUNCH_FILE="${LAUNCH_FILE:-onboard.launch.py}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PATH="$SCRIPT_DIR/$(basename "${BASH_SOURCE[0]}")"
WS="${WS:-$(dirname "$SCRIPT_DIR")}"

info() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m!!\033[0m %s\n' "$*"; }

require_tmux() {
  if ! command -v tmux >/dev/null 2>&1; then
    warn "tmux is not installed. Install it first: sudo apt install -y tmux"
    exit 1
  fi
}

find_unmanaged_launches() {
  pgrep -af '^/usr/bin/python3 /opt/ros/[^ ]+/bin/ros2 launch bisa (onboard|vehicle)\.launch\.py( |$)' || true
}

disable_wifi_powersave() {
  # Best-effort only: Wi-Fi power saving can make the operator connection flaky.
  local iface="${WIFI_IFACE:-}"
  if [ -z "$iface" ] && command -v iw >/dev/null 2>&1; then
    iface="$(iw dev 2>/dev/null | awk '/Interface/{print $2; exit}')"
  fi
  [ -z "$iface" ] && return 0
  if sudo -n iw dev "$iface" set power_save off >/dev/null 2>&1; then
    info "WiFi power_save off on $iface"
  else
    warn "Could not disable WiFi power_save on $iface (launch will continue)."
  fi
}

# Executed only as the command owned by tmux. `exec` makes the tmux session
# lifetime exactly match the ROS launch lifetime, so `status` cannot report a
# dead launch as running merely because an idle shell was left behind.
run_launch() {
  # ROS/ament setup scripts probe optional variables that may be unset.
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID="$DOMAIN"
  export ROS_LOCALHOST_ONLY="$LOCALHOST_ONLY"
  cd "$WS"
  exec ros2 launch bisa "$LAUNCH_FILE" "$@"
}

start() {
  require_tmux
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    info "Session '$SESSION' is already running."
    info "  Logs: tmux attach -t $SESSION"
    return 0
  fi

  local existing
  existing="$(find_unmanaged_launches)"
  if [ -n "$existing" ]; then
    warn "Another vehicle launch is already running outside session '$SESSION':"
    printf '%s\n' "$existing"
    warn "Stop that launch first; refusing to create duplicate control nodes."
    exit 1
  fi

  if [ ! -f "$WS/install/setup.bash" ]; then
    warn "Workspace is not built: $WS/install/setup.bash is missing."
    warn "Build first: cd $WS && colcon build --symlink-install"
    exit 1
  fi

  disable_wifi_powersave

  local -a command=(
    env
    "WS=$WS"
    "ROS_DISTRO=$ROS_DISTRO_NAME"
    "ROS_DOMAIN_ID=$DOMAIN"
    "ROS_LOCALHOST_ONLY=$LOCALHOST_ONLY"
    "LAUNCH_FILE=$LAUNCH_FILE"
    "$SCRIPT_PATH"
    __run
    "$@"
  )
  local command_string
  printf -v command_string '%q ' "${command[@]}"

  info "Starting $LAUNCH_FILE in detached tmux session '$SESSION'"
  info "  workspace=$WS  ROS_DOMAIN_ID=$DOMAIN  ROS_LOCALHOST_ONLY=$LOCALHOST_ONLY"
  tmux new-session -d -s "$SESSION" -n onboard "$command_string"

  # Catch immediate setup/launch failures while keeping normal startup fast.
  sleep 1
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    warn "Launch exited during startup. Check the latest log under ~/.ros/log."
    exit 1
  fi

  info "Started. It will keep running after SSH disconnects."
  info "  Logs: tmux attach -t $SESSION   (detach: Ctrl+b, then d)"
  info "  Stop: bash scripts/car_run.sh stop"
}

attach() {
  require_tmux
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    warn "No running session '$SESSION'. Start it first."
    exit 1
  fi
  tmux attach -t "$SESSION"
}

status() {
  require_tmux
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    local pane_pid pane_command
    pane_pid="$(tmux display-message -p -t "$SESSION" '#{pane_pid}')"
    pane_command="$(tmux display-message -p -t "$SESSION" '#{pane_current_command}')"
    info "RUNNING: session=$SESSION pid=$pane_pid command=$pane_command"
  else
    local existing
    existing="$(find_unmanaged_launches)"
    if [ -n "$existing" ]; then
      warn "UNMANAGED launch is running (not controlled by this script):"
      printf '%s\n' "$existing"
      return 2
    fi
    info "NOT RUNNING: session=$SESSION"
    return 1
  fi
}

stop() {
  require_tmux
  if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    info "Session '$SESSION' is not running."
    return 0
  fi

  info "Sending Ctrl+C for a graceful stop (neutral throttle)..."
  tmux send-keys -t "$SESSION" C-c

  local count
  for count in 1 2 3 4 5 6; do
    tmux has-session -t "$SESSION" 2>/dev/null || break
    sleep 1
  done
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    warn "Launch did not exit after 6 seconds; closing its tmux session."
    tmux kill-session -t "$SESSION"
  fi
  info "Stopped session '$SESSION'."
}

action="${1:-start}"
if [ "$#" -gt 0 ]; then
  shift
fi

case "$action" in
  start)  start "$@" ;;
  attach) attach ;;
  status) status ;;
  stop)   stop ;;
  __run)  run_launch "$@" ;;
  *)
    warn "Unknown command: $action"
    echo "Usage: $0 [start [launch_arg:=value ...]|attach|status|stop]"
    exit 1
    ;;
esac
