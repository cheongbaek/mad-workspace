#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cv_bridge 없이 sensor_msgs/Image <-> numpy(BGR) 변환.

cv_bridge는 Windows용 ROS2 Humble 배포본에 포함되어 있지 않다(빌드하려면 vision_opencv를
소스에서 직접 컴파일해야 해서 무겁고 깨지기 쉬움). 이 프로젝트가 실제로 쓰는 인코딩은
bgr8/rgb8/mono8 뿐이라, 그 범위만 직접 구현해 Windows/Ubuntu 어디서든 추가 설치 없이
동작하게 한다. (cam 패키지의 동일 파일과 내용이 같다 — 서로 다른 워크스페이스라 공유
import가 불가능해 각 패키지에 하나씩 둔다.)
"""

import numpy as np
import cv2
from sensor_msgs.msg import Image


def imgmsg_to_bgr8(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> BGR uint8 ndarray (cv_bridge.imgmsg_to_cv2(msg, 'bgr8') 대응)."""
    if msg.encoding == "mono8":
        channels = 1
    elif msg.encoding in ("bgr8", "rgb8"):
        channels = 3
    else:
        raise ValueError(f"지원하지 않는 인코딩: {msg.encoding} (bgr8/rgb8/mono8만 지원)")

    row_bytes = msg.width * channels
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    arr = arr[:, :row_bytes]  # step에 정렬 패딩이 있는 경우 잘라냄
    arr = arr.reshape(msg.height, msg.width, channels) if channels > 1 \
        else arr.reshape(msg.height, msg.width)

    if channels == 1:
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return arr.copy()  # 이미 bgr8


def bgr8_to_imgmsg(frame: np.ndarray, frame_id: str = "") -> Image:
    """BGR uint8 ndarray -> sensor_msgs/Image(encoding='bgr8') (cv_bridge.cv2_to_imgmsg 대응)."""
    msg = Image()
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
    msg.header.frame_id = frame_id
    return msg
