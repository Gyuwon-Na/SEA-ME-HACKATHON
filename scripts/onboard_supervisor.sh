#!/usr/bin/env bash
# Restart the all-on-vehicle ROS launch unless an operator requests stop.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$(dirname "$SCRIPT_DIR")}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
DOMAIN="${ROS_DOMAIN_ID:-42}"
ENABLE_ACTUATION="${ENABLE_ACTUATION:-true}"
ROUTE_MODE="${ROUTE_MODE:-OUT}"
LOG_DIR="${LOG_DIR:-$WS/log/competition}"
RESTART_DELAY_SEC="${RESTART_DELAY_SEC:-2}"

mkdir -p "$LOG_DIR"
exec > >(tee -a "$LOG_DIR/onboard.log") 2>&1

set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1

launch_pid=""
stopping=0

stop_launch() {
  stopping=1
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap stop_launch INT TERM EXIT

health_check() {
  local nodes ready
  nodes="$(timeout 4 ros2 node list 2>/dev/null || true)"
  grep -qx "/camera_node" <<<"$nodes" || return 1
  grep -qx "/bisa_detector_node" <<<"$nodes" || return 1
  grep -qx "/bisa_autonomous_node" <<<"$nodes" || return 1
  if [[ "$ENABLE_ACTUATION" == "true" ]]; then
    grep -qx "/control_node" <<<"$nodes" || return 1
  fi
  ready="$(timeout 4 ros2 topic echo --once /bisa/detector/ready 2>/dev/null || true)"
  grep -Eq 'data: *(true|True)' <<<"$ready" || return 1
  timeout 4 ros2 topic echo --once --qos-reliability best_effort \
    /camera/image/compressed >/dev/null 2>&1 || return 1
  timeout 4 ros2 topic echo --once --qos-reliability best_effort \
    /bisa/detections >/dev/null 2>&1 || return 1
  timeout 4 ros2 topic echo --once --qos-reliability best_effort \
    /control >/dev/null 2>&1
}

while (( ! stopping )); do
  printf '\n[%s] starting competition launch (actuation=%s route=%s domain=%s)\n' \
    "$(date -Ins)" "$ENABLE_ACTUATION" "$ROUTE_MODE" "$DOMAIN"
  ros2 launch bisa onboard.launch.py \
    route_mode:="$ROUTE_MODE" \
    enable_camera:=true \
    enable_joystick:=false \
    enable_actuation:="$ENABLE_ACTUATION" \
    publish_debug_image:=false &
  launch_pid=$!
  unhealthy_count=0
  # NCNN CPU warmup can take several seconds. Start monitoring after a grace
  # period, then restart the whole stack after three consecutive failed checks.
  sleep 12
  while kill -0 "$launch_pid" 2>/dev/null && (( ! stopping )); do
    if health_check; then
      unhealthy_count=0
    else
      unhealthy_count=$((unhealthy_count + 1))
      printf '[%s] health check failed (%d/3)\n' "$(date -Ins)" "$unhealthy_count"
      if (( unhealthy_count >= 3 )); then
        printf '[%s] critical runtime unhealthy; restarting full launch\n' "$(date -Ins)"
        kill -INT "$launch_pid" 2>/dev/null || true
        break
      fi
    fi
    sleep 2
  done
  wait "$launch_pid" 2>/dev/null
  result=$?
  launch_pid=""
  (( stopping )) && break
  printf '[%s] launch exited rc=%s; restarting in %ss\n' \
    "$(date -Ins)" "$result" "$RESTART_DELAY_SEC"
  sleep "$RESTART_DELAY_SEC"
done

trap - INT TERM EXIT
printf '[%s] competition supervisor stopped\n' "$(date -Ins)"
