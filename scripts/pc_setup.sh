#!/usr/bin/env bash
#
# D-Racer PC (operator laptop/desktop) setup.
#
# Run this on your Ubuntu PC — NOT on the car. It clones/updates the repo,
# installs the Python dependencies the compute node needs, builds the workspace,
# and prints how to match the ROS domain and launch.
#
# The car runs:   ros2 launch bisa vehicle.launch.py
# The PC runs:    ros2 launch bisa driving.launch.py route_mode:=OUT
#
# Both machines must be on the same LAN and use the SAME ROS_DOMAIN_ID.
#
# Usage:
#   bash pc_setup.sh                  # clone into ~/D-Racer-Kit and set up
#   WS=~/myws bash pc_setup.sh        # clone into a custom directory
#   ROS_DOMAIN_ID=7 bash pc_setup.sh  # also write that domain id to ~/.bashrc
#
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/Gyuwon-Na/SEA-ME-HACKATHON.git}"
BRANCH="${BRANCH:-refactoring}"
WS="${WS:-$HOME/D-Racer-Kit}"
DOMAIN="${ROS_DOMAIN_ID:-0}"

echo "==> D-Racer PC setup"
echo "    repo:   $REPO_URL ($BRANCH)"
echo "    target: $WS"
echo "    domain: $DOMAIN"

# --- 0. sanity: ROS 2 Humble present ----------------------------------------
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "!! ROS 2 Humble not found at /opt/ros/humble."
  echo "   Install it first: https://docs.ros.org/en/humble/Installation.html"
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# --- 1. clone or update the repo --------------------------------------------
if [ -d "$WS/.git" ]; then
  echo "==> Updating existing repo in $WS"
  git -C "$WS" fetch origin "$BRANCH"
  git -C "$WS" checkout "$BRANCH"
  git -C "$WS" pull --ff-only origin "$BRANCH"
else
  echo "==> Cloning into $WS"
  git clone --branch "$BRANCH" "$REPO_URL" "$WS"
fi

# --- 2. system dependencies (sudo) ------------------------------------------
echo "==> Installing system packages (sudo may prompt)"
sudo apt-get update
sudo apt-get install -y \
  python3-pip python3-tk \
  ros-humble-rclpy ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-rcl-interfaces \
  python3-opencv

# --- 3. Python dependencies for detection (best.pt) -------------------------
# On an x86 PC the default torch wheel works (the +cpu special-case is only for
# the car's aarch64 A72). ultralytics pulls a compatible torch automatically.
echo "==> Installing Python detection deps (ultralytics + torch)"
python3 -m pip install --user --upgrade pip
python3 -m pip install --user ultralytics pyyaml

# ArUco lives in opencv-contrib. If 'import cv2.aruco' fails with system OpenCV,
# fall back to the contrib wheel.
if ! python3 -c "import cv2.aruco" >/dev/null 2>&1; then
  echo "==> System OpenCV lacks aruco; installing opencv-contrib-python"
  python3 -m pip install --user "opencv-contrib-python<4.10"
fi

# --- 4. build the workspace --------------------------------------------------
echo "==> Building (colcon)"
cd "$WS"
colcon build --packages-select bisa control_msgs camera control battery battery_msgs \
  || colcon build   # fall back to building everything if msg deps need it

# --- 5. domain id persistence -----------------------------------------------
if ! grep -q "ROS_DOMAIN_ID=$DOMAIN" "$HOME/.bashrc" 2>/dev/null; then
  echo "==> Writing ROS_DOMAIN_ID=$DOMAIN to ~/.bashrc"
  {
    echo ""
    echo "# D-Racer: match the car's ROS domain"
    echo "export ROS_DOMAIN_ID=$DOMAIN"
    echo "export ROS_LOCALHOST_ONLY=0"
  } >> "$HOME/.bashrc"
fi

cat <<EOF

==> Done.

NEXT STEPS
  1. The detection model ($WS/src/bisa/checkpoints/best.pt) is already in the
     repo, so it came with the clone. (Lane following + ArUco + steering
     visualization also work even without it.)

  2. Make sure the CAR uses the same domain:  export ROS_DOMAIN_ID=$DOMAIN

  3. New shell, then:
       source /opt/ros/humble/setup.bash
       source $WS/install/setup.bash

  4. On the CAR:   ros2 launch bisa vehicle.launch.py
     On THIS PC:   ros2 launch bisa driving.launch.py route_mode:=OUT

  5. Check topics on the PC:
       ros2 topic echo /detect_green
       ros2 topic echo /detect_sign      # left/right on one line
       ros2 topic echo /detect_aruco     # ids=[3] when marker 3 is seen
EOF
