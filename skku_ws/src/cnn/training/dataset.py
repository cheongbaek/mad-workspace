#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dataset_recorder가 만든 session_*/ 데이터를 PyTorch Dataset으로 로드.

labels.csv 스키마:
  frame_idx, t_sec, cur_angle, prev_angle, prev_pwm, angle, pwm, sonar1..6

- RawDrivingDataset     : (이미지, state) → [angle_n, pwm_n]   (Variant A)
- FeatureDrivingDataset : (허프특징+state) → [angle_n, pwm_n]  (Variant B)

정규화/특징추출은 cnn/ 패키지의 models.py, lane_features.py를 그대로 import해서
추론 노드와 완전히 동일한 계산을 쓴다(순수 스크립트 실행을 위해 sys.path로 로드).
"""

import csv
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset

# training/은 ROS 빌드 대상이 아니므로 형제 패키지 디렉터리를 직접 path에 얹는다.
# cnn.models / cnn.lane_features 라는 동일한 import 이름을 유지하기 위해 src/cnn을 추가.
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from cnn.models import encode_targets, make_state          # noqa: E402
from cnn.lane_features import extract_lane_features         # noqa: E402


def _load_rows(session_dirs):
    rows = []
    for session_dir in session_dirs:
        session_dir = Path(session_dir).expanduser()
        with open(session_dir / "labels.csv", newline="") as f:
            for row in csv.DictReader(f):
                row["_session_dir"] = session_dir
                rows.append(row)
    return rows


def _read_frame(row):
    img_path = row["_session_dir"] / "images" / f"{int(row['frame_idx']):06d}.jpg"
    frame = cv2.imread(str(img_path))
    if frame is None:
        raise FileNotFoundError(img_path)
    return frame


def _state_of(row):
    return make_state(float(row["cur_angle"]),
                      float(row["prev_angle"]),
                      float(row["prev_pwm"]))


def _target_of(row):
    return encode_targets(float(row["angle"]), float(row["pwm"]))


class RawDrivingDataset(Dataset):
    def __init__(self, session_dirs, input_w=160, input_h=120):
        self.rows = _load_rows(session_dirs)
        self.input_w = input_w
        self.input_h = input_h

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, idx):
        row = self.rows[idx]
        frame = _read_frame(row)
        frame = cv2.resize(frame, (self.input_w, self.input_h))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        image = torch.from_numpy(frame.transpose(2, 0, 1))
        state = torch.tensor(_state_of(row), dtype=torch.float32)
        target = torch.tensor(_target_of(row), dtype=torch.float32)
        return image, state, target


class FeatureDrivingDataset(Dataset):
    """허프 특징은 __getitem__에서 이미지로부터 매번 계산한다(프레임 단위 순수 함수라
    셔플 순서와 무관하게 추론 때와 동일한 값)."""

    def __init__(self, session_dirs):
        self.rows = _load_rows(session_dirs)

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, idx):
        row = self.rows[idx]
        feat = extract_lane_features(_read_frame(row))
        x = torch.tensor(list(feat) + _state_of(row), dtype=torch.float32)
        target = torch.tensor(_target_of(row), dtype=torch.float32)
        return x, target
