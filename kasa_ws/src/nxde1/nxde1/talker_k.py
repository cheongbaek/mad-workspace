# talker_k : in.csv 시퀀스를 읽어 순서대로 /in 토픽 발행
#
# in.csv 양식 (첫 행은 헤더로 무시, 둘째 행부터):
#   (유지시간),(주행펄스),(조향각도),(브레이크),(모드)[,memo]
#   - 브레이크/모드 칸이 비어 있으면 0으로 처리
# 발행 메시지: "주행펄스 조향각도 브레이크 모드" (std_msgs/String, 아두이노 입력 한 줄 형식)
# 각 행을 한 번 발행한 뒤 유지시간(초) 동안 0.5초마다 동일 메시지 재발행 (안정성 강화),
# 유지시간이 지나면 다음 행으로. 마지막 행까지 끝나면 마지막 메시지를 0.5초 주기로 계속 재발행

import csv
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CSV_FILE = 'in.csv'
REPUB_PERIOD_S = 0.5   # 동일 메시지 재발행 주기


def _cell(row, i):
    """row의 i번째 칸을 안전하게 반환 (없으면 빈 문자열)"""
    return row[i].strip() if len(row) > i else ""


def _int_or_zero(s):
    """빈 칸이면 0, 아니면 int 변환"""
    return int(s) if s != "" else 0


def load_sequence(filename):
    """in.csv를 읽어 시퀀스 리스트로 반환.
       각 항목: (duration, pulse, angle, brake, mode)
       첫 행은 헤더로 무시, 빈 줄/# 주석 줄 건너뜀."""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(base_dir, filename)
    if not os.path.exists(path):
        raise FileNotFoundError(f"시퀀스 파일을 찾을 수 없습니다: {path}")

    try:
        f = open(path, 'r', encoding='utf-8-sig')
        f.read(); f.seek(0)
    except UnicodeDecodeError:
        f = open(path, 'r', encoding='cp949')

    sequence = []
    with f:
        reader = csv.reader(f)
        header_skipped = False
        for lineno, row in enumerate(reader, start=1):
            if not row:
                continue
            first = row[0].strip()
            if first == '' or first.startswith('#'):
                continue
            if not header_skipped:
                header_skipped = True   # 첫 행은 무조건 헤더로 간주하고 건너뜀
                continue
            if len(row) < 3:
                print(f"[경고] {lineno}번째 줄 값 부족(최소 유지시간,주행펄스,조향각), 건너뜀: {row}")
                continue

            try:
                duration = float(row[0])
                pulse    = int(row[1])
                angle    = int(row[2])
                brake    = _int_or_zero(_cell(row, 3))
                mode     = _int_or_zero(_cell(row, 4))
            except ValueError:
                print(f"[경고] {lineno}번째 줄 숫자 변환 실패, 건너뜀: {row}")
                continue

            sequence.append((duration, pulse, angle, brake, mode))
    return sequence


class TalkerK(Node):
    def __init__(self):
        super().__init__('talker_k')
        self.pub = self.create_publisher(String, '/in', 10)

    def publish_and_hold(self, idx, total, pulse, angle, brake, mode, msg, duration):
        """msg를 유지시간 동안 REPUB_PERIOD_S 간격으로 재발행하며 현재 행 진행상황 출력"""
        end_t = time.monotonic() + duration
        while rclpy.ok():
            remain = end_t - time.monotonic()
            if remain <= 0:
                return
            time.sleep(min(REPUB_PERIOD_S, remain))
            if time.monotonic() < end_t:
                self.pub.publish(msg)
                self.get_logger().info(
                    f"[{idx}/{total}] 진행 중: 펄스={pulse} 조향={angle} 브레이크={brake} 모드={mode} "
                    f"(남은 유지시간 {max(0.0, end_t - time.monotonic()):.1f}s)")

    def run_sequence(self, sequence):
        msg = None
        last = None
        for idx, (duration, pulse, angle, brake, mode) in enumerate(sequence, start=1):
            if not rclpy.ok():
                return
            msg = String()
            msg.data = f"{pulse} {angle} {brake} {mode}"
            self.pub.publish(msg)
            self.get_logger().info(
                f"[{idx}/{len(sequence)}] /in <- \"{msg.data}\" "
                f"(유지 {duration}s, {REPUB_PERIOD_S}s마다 재발행)")
            self.publish_and_hold(idx, len(sequence), pulse, angle, brake, mode, msg, duration)
            last = (idx, pulse, angle, brake, mode)

        if msg is None:
            return   # 빈 시퀀스: main의 spin으로 대기
        idx, pulse, angle, brake, mode = last
        self.get_logger().info(
            f"시퀀스 종료. 마지막 행을 {REPUB_PERIOD_S}s 주기로 계속 재발행합니다 (Ctrl+C로 종료).")
        while rclpy.ok():
            time.sleep(REPUB_PERIOD_S)
            self.pub.publish(msg)
            self.get_logger().info(
                f"[{idx}/{idx}] 진행 중(마지막 행 유지): 펄스={pulse} 조향={angle} 브레이크={brake} 모드={mode}")


def main(args=None):
    rclpy.init(args=args)
    node = TalkerK()
    try:
        sequence = load_sequence(CSV_FILE)
    except FileNotFoundError as e:
        node.get_logger().error(str(e))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info(f"in.csv에서 {len(sequence)}개 행 로드")
    try:
        node.run_sequence(sequence)
        rclpy.spin(node)   # 빈 시퀀스 등 재발행 루프가 없을 때 대기 유지 (Ctrl+C로 종료)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
