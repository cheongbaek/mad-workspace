#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""차선 특징 추출 — 허프변환 기반 고전 CV. YOLO/딥러닝 인식 불사용.

정리자료/차선인식/hough_lane_detection.py(원본: my_hough 패키지)의 핵심 로직을
이식·축약한 것. "카메라 BGR 프레임 1장 → 저차원 특징 벡터"만 수행한다.

Variant B(차선 특징 학습)의 입력을 만들며, 학습(training/dataset.py)과
추론(feature_infer_node.py)이 이 모듈 하나를 공유한다 — 두 경로의 특징 계산이
조금이라도 다르면(train/inference skew) 모델이 실차에서 안 먹히므로 반드시
이 파일만 수정한다.

특징은 프레임 단위 순수 함수로 계산한다(이전 프레임 상태 없음) — 학습 시
데이터셋을 임의 순서로 섞어 읽어도 추론 때와 동일한 값이 나오게 하기 위함.
검출 실패는 found 플래그(0/1)로 모델에 그대로 알려주고 모델이 학습으로 대처한다.

FEATURE_DIM = 7:
  [0] left_found   (0/1)
  [1] x_left_n     기준행에서 왼쪽 차선 교점 x / 화면폭  (0~1, 미검출 시 0)
  [2] m_left       왼쪽 대표직선 기울기 (clip ±3 후 /3 → -1~1)
  [3] right_found  (0/1)
  [4] x_right_n    (0~1, 미검출 시 1)
  [5] m_right      (-1~1)
  [6] offset_n     차선 중점의 화면중심 대비 오프셋 (-1~1, 양쪽 다 미검출 시 0)
"""

import math

import cv2
import numpy as np

FEATURE_DIM = 7

# 기본 파라미터 — 640x480 기준 (원본 hough_drive.py 값). 트랙/카메라 각도가 다르면
# feature_infer_node의 ROS 파라미터로 조정한다(학습·추론 양쪽 동일하게!).
DEFAULT_PARAMS = {
    "roi_start_row": 300,   # ROI 상단 행 (480 높이 기준)
    "roi_end_row":   380,   # ROI 하단 행
    "base_row":      40,    # ROI 내부 기준 수평선 (교점 계산용)
    "canny_th1":     60,
    "canny_th2":     75,
    "hough_threshold": 50,
    "min_line_length": 50,
    "max_line_gap":    20,
    "slope_thresh":  0.2,   # 이 미만 기울기(수평선)는 노이즈로 버림
}


def _fit_representative_line(lines):
    """선분들의 기울기 평균 + 끝점 평균으로 대표직선 y=mx+b. 없으면 (0,0)=미검출."""
    if not lines:
        return 0.0, 0.0
    x_sum = y_sum = m_sum = 0.0
    for x1, y1, x2, y2 in lines:
        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1) if x2 != x1 else 0.0
    size = len(lines)
    x_avg, y_avg = x_sum / (size * 2), y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg
    return m, b


def extract_lane_features(frame_bgr: np.ndarray, params: dict = None) -> np.ndarray:
    """BGR 프레임 1장 → FEATURE_DIM 차원 특징 벡터(float32, 대략 -1~1 스케일)."""
    p = dict(DEFAULT_PARAMS)
    if params:
        p.update(params)

    h, w = frame_bgr.shape[:2]
    # 파라미터는 640x480 기준이므로 다른 해상도가 들어오면 비율로 환산
    sy, sx = h / 480.0, w / 640.0
    roi_start = int(p["roi_start_row"] * sy)
    roi_end   = int(p["roi_end_row"] * sy)
    base_row  = int(p["base_row"] * sy)

    roi = frame_bgr[roi_start:roi_end, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, p["canny_th1"], p["canny_th2"])

    all_lines = cv2.HoughLinesP(
        edges, 1, math.pi / 180, p["hough_threshold"],
        minLineLength=int(p["min_line_length"] * sx),
        maxLineGap=int(p["max_line_gap"] * sx))

    left_lines, right_lines = [], []
    if all_lines is not None:
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            slope = 1000.0 if x2 == x1 else float(y2 - y1) / float(x2 - x1)
            if abs(slope) <= p["slope_thresh"]:
                continue
            if slope < 0 and x2 < w / 2:
                left_lines.append([x1, y1, x2, y2])
            elif slope > 0 and x1 > w / 2:
                right_lines.append([x1, y1, x2, y2])

    m_l, b_l = _fit_representative_line(left_lines)
    m_r, b_r = _fit_representative_line(right_lines)

    left_found  = m_l != 0.0
    right_found = m_r != 0.0
    x_left  = (base_row - b_l) / m_l if left_found else None
    x_right = (base_row - b_r) / m_r if right_found else None

    # 중점 오프셋: 한쪽만 검출 시 대략적 차선폭(화면폭 0.6배)으로 반대편 추정
    est_width = w * 0.6
    if left_found and right_found:
        mid = (x_left + x_right) / 2.0
    elif left_found:
        mid = x_left + est_width / 2.0
    elif right_found:
        mid = x_right - est_width / 2.0
    else:
        mid = w / 2.0

    def _clip_slope(m):
        return float(np.clip(m, -3.0, 3.0)) / 3.0

    feat = np.array([
        1.0 if left_found else 0.0,
        (x_left / w) if left_found else 0.0,
        _clip_slope(m_l) if left_found else 0.0,
        1.0 if right_found else 0.0,
        (x_right / w) if right_found else 1.0,
        _clip_slope(m_r) if right_found else 0.0,
        float(np.clip((mid - w / 2.0) / (w / 2.0), -1.0, 1.0)),
    ], dtype=np.float32)
    return feat
