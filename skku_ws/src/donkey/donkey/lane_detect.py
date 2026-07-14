#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""차선 정보 판별 — 허프변환 기반 (정리자료/차선인식/hough_lane_detection.py 참고, 새로 작성).

카메라 BGR 프레임 1장 → 차선 특징 벡터(각도값·기울기 등). 딥러닝 인식 없음.

수집(collect_lane)과 주행(drive, lane 모델일 때)이 이 모듈 하나를 공유한다 —
두 쪽의 특징 계산이 다르면 학습된 모델이 실차에서 안 맞으므로 반드시 여기만 수정.

특징 7개 (모두 대략 -1~1 스케일, CSV 열 이름 = FEATURE_NAMES):
  left_found   왼쪽 차선 검출 여부 (0/1)
  left_x       기준행에서 왼쪽 차선 위치 x/화면폭 (0~1, 미검출 0)
  left_slope   왼쪽 대표직선 기울기 (clip ±3 후 /3)
  right_found  (0/1)
  right_x      (0~1, 미검출 1)
  right_slope  (-1~1)
  offset       차선 중점의 화면중심 대비 오프셋 (-1~1) — 조향의 핵심 신호
"""

import math

import cv2
import numpy as np

FEATURE_NAMES = ["left_found", "left_x", "left_slope",
                 "right_found", "right_x", "right_slope", "offset"]
FEATURE_DIM = len(FEATURE_NAMES)

# 640x480 기준 파라미터 (정리자료 hough_drive.py 값 기반).
# 카메라 장착 각도가 다르면 ROI 행 범위를 조정한다 — 수집·주행 양쪽에 자동 반영됨.
ROI_START_ROW = 300
ROI_END_ROW   = 380
BASE_ROW      = 40        # ROI 내부 기준 수평선
CANNY_TH      = (60, 75)
HOUGH_THRESHOLD  = 50
MIN_LINE_LENGTH  = 50
MAX_LINE_GAP     = 20
SLOPE_THRESH     = 0.2    # 수평에 가까운 선분은 노이즈로 버림


def _fit_line(lines):
    """선분들의 기울기 평균 + 끝점 평균으로 대표직선 y=mx+b. 없으면 (0,0)=미검출."""
    if not lines:
        return 0.0, 0.0
    x_sum = y_sum = m_sum = 0.0
    for x1, y1, x2, y2 in lines:
        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1) if x2 != x1 else 0.0
    n = len(lines)
    x_avg, y_avg = x_sum / (n * 2), y_sum / (n * 2)
    m = m_sum / n
    return m, y_avg - m * x_avg


def extract_features(frame_bgr: np.ndarray) -> np.ndarray:
    """BGR 프레임 1장 → FEATURE_DIM 차원 float32 벡터."""
    h, w = frame_bgr.shape[:2]
    sy, sx = h / 480.0, w / 640.0

    roi = frame_bgr[int(ROI_START_ROW * sy):int(ROI_END_ROW * sy), :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, *CANNY_TH)

    all_lines = cv2.HoughLinesP(
        edges, 1, math.pi / 180, HOUGH_THRESHOLD,
        minLineLength=int(MIN_LINE_LENGTH * sx), maxLineGap=int(MAX_LINE_GAP * sx))

    left, right = [], []
    if all_lines is not None:
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            slope = 1000.0 if x2 == x1 else float(y2 - y1) / float(x2 - x1)
            if abs(slope) <= SLOPE_THRESH:
                continue
            if slope < 0 and x2 < w / 2:
                left.append([x1, y1, x2, y2])
            elif slope > 0 and x1 > w / 2:
                right.append([x1, y1, x2, y2])

    m_l, b_l = _fit_line(left)
    m_r, b_r = _fit_line(right)
    base = BASE_ROW * sy

    l_found, r_found = m_l != 0.0, m_r != 0.0
    x_l = (base - b_l) / m_l if l_found else None
    x_r = (base - b_r) / m_r if r_found else None

    est_width = w * 0.6   # 한쪽만 보일 때 반대편 추정용 대략적 차선폭
    if l_found and r_found:
        mid = (x_l + x_r) / 2.0
    elif l_found:
        mid = x_l + est_width / 2.0
    elif r_found:
        mid = x_r - est_width / 2.0
    else:
        mid = w / 2.0

    def _slope_n(m):
        return float(np.clip(m, -3.0, 3.0)) / 3.0

    return np.array([
        1.0 if l_found else 0.0,
        (x_l / w) if l_found else 0.0,
        _slope_n(m_l) if l_found else 0.0,
        1.0 if r_found else 0.0,
        (x_r / w) if r_found else 1.0,
        _slope_n(m_r) if r_found else 0.0,
        float(np.clip((mid - w / 2.0) / (w / 2.0), -1.0, 1.0)),
    ], dtype=np.float32)
