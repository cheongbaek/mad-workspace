#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PyTorch 모델 정의 + 라벨/상태 정규화 — 학습 스크립트와 추론 노드가 공유하는 단일 소스.

benz.ino 플랫폼 기준:
  출력 = [조향각(-30~30 deg), 주행PWM(-255~255)]
  상태(state, 멀티모달 입력) = [현재 조향각(시리얼 피드백), 직전 명령 각도, 직전 명령 PWM]

architecture나 정규화 스케일이 학습/추론에서 어긋나면 모델이 그대로 안 먹히므로,
반드시 이 한 곳만 고쳐서 양쪽에 반영되게 한다.
"""

import torch
import torch.nn as nn

from cnn.lane_features import FEATURE_DIM

MAX_ANGLE = 30.0    # benz.ino ANG_MIN/MAX
MAX_PWM   = 255.0   # benz.ino PWM constrain

STATE_DIM = 3       # [cur_angle_n, prev_angle_n, prev_pwm_n]


# ── 정규화 (학습 타깃/상태 입력 공통) ──────────────────────────

def norm_angle(angle_deg):
    return float(angle_deg) / MAX_ANGLE

def norm_pwm(pwm):
    return float(pwm) / MAX_PWM

def make_state(cur_angle_deg, prev_angle_deg, prev_pwm):
    """state 벡터(3차원, -1~1)."""
    return [norm_angle(cur_angle_deg), norm_angle(prev_angle_deg), norm_pwm(prev_pwm)]

def encode_targets(angle_deg, pwm):
    """실제 단위 → 학습 타깃 [angle_n, pwm_n] (-1~1)."""
    return [norm_angle(angle_deg), norm_pwm(pwm)]

def decode_outputs(angle_n, pwm_n, allow_reverse=False, max_pwm=MAX_PWM):
    """모델 출력(-1~1, tanh) → (조향각 deg, 주행PWM). 안전 clamp 포함."""
    angle = float(angle_n) * MAX_ANGLE
    pwm   = float(pwm_n) * MAX_PWM
    angle = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
    lo = -max_pwm if allow_reverse else 0.0
    pwm = max(lo, min(max_pwm, pwm))
    return angle, pwm


# ── Variant A: 전체 이미지 + state → CNN ──────────────────────

class RawPilotNet(nn.Module):
    """(3,H,W) 이미지 + (STATE_DIM,) 상태 → [angle_n, pwm_n] (tanh).

    PilotNet 스타일 Conv 트렁크 뒤에 state를 concat하는 멀티모달 구조.
    """

    def __init__(self, in_h=120, in_w=160, state_dim=STATE_DIM):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, kernel_size=3, stride=1), nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1), nn.ReLU(),
            nn.Flatten(),
        )
        with torch.no_grad():
            flat_dim = self.conv(torch.zeros(1, 3, in_h, in_w)).shape[1]

        self.fc = nn.Sequential(
            nn.Linear(flat_dim + state_dim, 100), nn.ReLU(),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 2),
            nn.Tanh(),
        )

    def forward(self, image, state):
        x = self.conv(image)
        x = torch.cat([x, state], dim=1)
        return self.fc(x)


# ── Variant B: 차선 특징 + state → MLP ────────────────────────

class FeatureMLP(nn.Module):
    """차선 특징(FEATURE_DIM) + 상태(STATE_DIM) → [angle_n, pwm_n] (tanh)."""

    IN_DIM = FEATURE_DIM + STATE_DIM

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(self.IN_DIM, 64), nn.ReLU(),
            nn.Linear(64, 32), nn.ReLU(),
            nn.Linear(32, 2),
            nn.Tanh(),
        )

    def forward(self, x):
        return self.net(x)
