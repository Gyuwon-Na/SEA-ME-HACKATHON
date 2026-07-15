#!/usr/bin/env bash
set -euo pipefail

CAR_IP="${CAR_IP:-192.168.0.113}"
CAR_USER="${CAR_USER:-topst}"
CAR_WS="${CAR_WS:-/home/topst/D-Racer-Kit}"
REMOTE="${CAR_USER}@${CAR_IP}"
SSH=(ssh -o ConnectTimeout=5)

"${SSH[@]}" "${REMOTE}" "mkdir -p '${CAR_WS}/src' '${CAR_WS}/scripts'"

rsync -av --delete -e "ssh -o ConnectTimeout=5" \
  --exclude build \
  --exclude install \
  --exclude log \
  --exclude .git \
  ./src/ "${REMOTE}:${CAR_WS}/src/"

rsync -av -e "ssh -o ConnectTimeout=5" \
  ./scripts/car_run.sh "${REMOTE}:${CAR_WS}/scripts/car_run.sh"

"${SSH[@]}" "${REMOTE}" "
  set -e
  cd '${CAR_WS}'
  source /opt/ros/humble/setup.bash
  colcon build
  source install/setup.bash
  test -f install/bisa/share/bisa/checkpoints/best_ncnn_model/model.ncnn.param
  test -f install/bisa/share/bisa/checkpoints/best_ncnn_model/model.ncnn.bin
  ros2 launch bisa onboard.launch.py --show-args >/dev/null
  echo 'vehicle sync/build/launch check complete'
"
