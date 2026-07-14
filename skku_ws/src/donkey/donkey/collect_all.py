#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""수집 (전체 카메라 방식) — 카메라 프레임 전체를 jpg로 저장하며 기록.

data/all_XXX/images/000000.jpg ... + log.csv (기본열 + image 파일명).

실행: ros2 launch donkey collect_all.launch.py
"""

import cv2

from donkey.collect_base import CollectBase, run_collector


class CollectAll(CollectBase):
    SESSION_PREFIX = "all"

    def __init__(self):
        super().__init__("collect_all")
        self.images_dir = self.session_dir / "images"
        self.images_dir.mkdir(exist_ok=True)

    def extra_columns(self):
        return ["image"]

    def process_frame(self, frame, idx):
        name = f"{idx:06d}.jpg"
        cv2.imwrite(str(self.images_dir / name), frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 95])
        return [name]


def main(args=None):
    run_collector(CollectAll)


if __name__ == "__main__":
    main()
