#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""수집 (차선 정보 방식) — 카메라에서 차선 특징(각도값·기울기 등)만 뽑아 기록.

이미지 파일은 저장하지 않는다. 매 프레임 허프변환으로 특징 7개를 계산해
data/lane_XXX/log.csv 에 [기본열 + 특징열]로 한 행씩 쌓는다.

실행: ros2 launch donkey collect_lane.launch.py
"""

from donkey.collect_base import CollectBase, run_collector
from donkey.lane_detect import extract_features, FEATURE_NAMES


class CollectLane(CollectBase):
    SESSION_PREFIX = "lane"

    def __init__(self):
        super().__init__("collect_lane")

    def extra_columns(self):
        return FEATURE_NAMES

    def process_frame(self, frame, idx):
        feat = extract_features(frame)
        return [f"{v:.4f}" for v in feat]


def main(args=None):
    run_collector(CollectLane)


if __name__ == "__main__":
    main()
