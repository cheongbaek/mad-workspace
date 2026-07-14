# listener_k : /out 구독 -> out.csv 저장 (1초 간격 스로틀)
#
# - /out (std_msgs/String) 메시지("<21번펄스>,<20번펄스>,<조향각>,<브레이크>")를 구독
#   (walker_k가 A보드/B보드 텔레메트리 + 마지막 브레이크 명령을 합쳐 이 형식으로 발행.
#   브레이크는 B보드에 센서 피드백이 없어 마지막 명령값이 그대로 들어옴)
# - 마지막 저장 후 1초가 지나기 전에 온 메시지는 무시
# - out.csv는 노드 시작 시 덮어쓰기(없으면 생성), 첫 행부터 바로 데이터
#   형식: 21번펄스,20번펄스,조향각,브레이크
# - 정상 동작 중에는 로그 출력 없이 조용히 저장만 함 (오류만 로그)

import csv
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CSV_FILE = 'out.csv'
ACCEPT_INTERVAL_S = 1.0   # 이 간격 안에 온 메시지는 무시


class ListenerK(Node):
    def __init__(self):
        super().__init__('listener_k')

        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.csv_path = os.path.join(base_dir, CSV_FILE)

        # 있으면 덮어쓰기, 없으면 생성 (첫 행부터 바로 데이터)
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)

        self.last_accept = 0.0   # 시작 직후 첫 메시지는 바로 수용
        self.sub = self.create_subscription(String, '/out', self.on_out, 10)

    def on_out(self, msg):
        now = time.monotonic()
        if now - self.last_accept < ACCEPT_INTERVAL_S:
            return   # 1초 간격 미달 → 무시
        self.last_accept = now

        self.writer.writerow(msg.data.split(','))
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ListenerK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
