#!/bin/bash
set -e

CAR_IP=192.168.0.113
CAR_USER=topst
CAR_WS=/home/topst/D-Racer-Kit

rsync -av --delete \
  --exclude build \
  --exclude install \
  --exclude log \
  --exclude .git \
  ./src/ ${CAR_USER}@${CAR_IP}:${CAR_WS}/src/

ssh ${CAR_USER}@${CAR_IP} "cd ${CAR_WS} && colcon build"
