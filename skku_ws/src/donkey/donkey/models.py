#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PyTorch 모델 정의 — train.py(학습)와 drive.py(주행)가 공유하는 단일 소스.

두 모델 다 출력은 [angle_n, pwm_n] (tanh, -1~1). 실단위 변환은 common.denorm.

- LaneNet : 차선 특징 벡터(FEATURE_DIM=7) → MLP      (lane_XXX 데이터용)
- AllNet  : 카메라 프레임 전체(3x120x160) → CNN        (all_XXX 데이터용)
            NVIDIA PilotNet 스타일 (정리자료/학습기반주행_E2E 참고, 새로 작성)
"""

import torch
import torch.nn as nn

from donkey.lane_detect import FEATURE_DIM

# AllNet 입력 크기 — train.py와 drive.py가 여기서 같이 읽는다
IMG_W, IMG_H = 160, 120


class LaneNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(FEATURE_DIM, 64), nn.ReLU(),
            nn.Linear(64, 32), nn.ReLU(),
            nn.Linear(32, 2),
            nn.Tanh(),
        )

    def forward(self, x):
        return self.net(x)


class AllNet(nn.Module):
    def __init__(self, in_h=IMG_H, in_w=IMG_W):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, 3), nn.ReLU(),
            nn.Conv2d(64, 64, 3), nn.ReLU(),
            nn.Flatten(),
        )
        with torch.no_grad():
            flat = self.conv(torch.zeros(1, 3, in_h, in_w)).shape[1]
        self.fc = nn.Sequential(
            nn.Linear(flat, 100), nn.ReLU(),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 2),
            nn.Tanh(),
        )

    def forward(self, x):
        return self.fc(self.conv(x))


def build_model(data_type: str) -> nn.Module:
    """data_type: 'lane' 또는 'all' — meta.json의 type 값과 동일."""
    if data_type == "lane":
        return LaneNet()
    if data_type == "all":
        return AllNet()
    raise ValueError(f"알 수 없는 데이터 타입: {data_type}")
