#!/usr/bin/env python3
"""조향 중앙값(STEER_TRIM) 정렬 도구.

서보를 실시간으로 움직여 앞바퀴가 정확히 정면을 향하는 지점을 찾은 뒤,
그 값을 src/config/vehicle_config.yaml 의 STEER_TRIM 으로 저장한다.

조작:
  a / d      : 좌 / 우 로 0.02 씩 조정
  A / D      : 좌 / 우 로 0.005 씩 미세 조정 (Shift)
  0          : 서보 물리 중립(0.0)으로 이동
  s          : 현재 값을 STEER_TRIM 으로 저장
  q          : 저장 없이 종료
"""
import os
import sys
import termios
import tty

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, 'src', 'topst_utils'))

import yaml
from topst_utils.d3racer import D3Racer

CONFIG_PATH = os.path.join(REPO_ROOT, 'src', 'config', 'vehicle_config.yaml')
STEP = 0.02
FINE = 0.005


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def load_config():
    with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def save_trim(value):
    # 원본 파일의 순서/주석을 최대한 보존하기 위해 라인 기반으로 교체
    with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    replaced = False
    for i, line in enumerate(lines):
        if line.strip().startswith('STEER_TRIM:'):
            lines[i] = f'STEER_TRIM: {value:.3f}\n'
            replaced = True
            break
    if not replaced:
        lines.append(f'STEER_TRIM: {value:.3f}\n')
    with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
        f.writelines(lines)


def main():
    cfg = load_config()
    steer = float(cfg.get('STEER_TRIM', 0.0))

    racer = D3Racer(i2c_bus=3, pca9685_addr=0x40, steering_channel=0, throttle_channel=1)
    racer.set_throttle_percent(0.0)  # 안전: 스로틀 중립
    racer.set_steering_percent(steer)

    print('=== 조향 중앙값 정렬 ===')
    print('a/d: ±0.02   A/D: ±0.005(미세)   0: 물리중립   s: 저장   q: 종료')
    print(f'시작값 STEER_TRIM = {steer:.3f}\n')
    try:
        while True:
            sys.stdout.write(f'\r현재 steering = {steer:+.3f}   ')
            sys.stdout.flush()
            k = read_key()
            if k in ('q', '\x03'):
                print('\n저장하지 않고 종료합니다.')
                break
            elif k == 'a':
                steer -= STEP
            elif k == 'd':
                steer += STEP
            elif k == 'A':
                steer -= FINE
            elif k == 'D':
                steer += FINE
            elif k == '0':
                steer = 0.0
            elif k == 's':
                save_trim(steer)
                print(f'\nSTEER_TRIM = {steer:.3f} 저장 완료 -> {CONFIG_PATH}')
                break
            else:
                continue
            steer = max(-1.0, min(1.0, steer))
            racer.set_steering_percent(steer)
    finally:
        racer.set_steering_percent(steer)
        racer.pwm.close()


if __name__ == '__main__':
    main()
