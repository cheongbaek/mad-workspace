#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rec.py  -  rosbag2 추출기 (분석 친화적 버전)
─────────────────────────────────────────────────────────────
가장 최근 rosbag2_* 폴더를 자동 탐색하여 다음 파일들을 생성:

  extracted_<bag_name>/
    ├─ <topic>.jsonl              : 토픽별 원본(JSON Lines)
    ├─ <topic>.csv                : 토픽별 표 형태 (pandas/엑셀 호환)
    ├─ driving_debug_named.csv    : /driving_debug 27필드를 이름 있는 열로 펼침
    ├─ merged_timeline.csv        : 핵심 신호 통합 시계열
    └─ _summary.txt               : 토픽별 통계 (메시지 수, 주기, 시간 범위)

실행:  python3 rec.py
─────────────────────────────────────────────────────────────
"""

import os
import glob
import json
import csv
from collections import OrderedDict, defaultdict

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict


# ═══════════════════════════════════════════════════════════════
#  /driving_debug 27필드 이름 매핑 (driving.py 와 일치)
# ═══════════════════════════════════════════════════════════════
DRIVING_DEBUG_FIELDS = [
    'wp_idx',          # 0
    'wp_total',        # 1
    'd2dest',          # 2
    'cte',             # 3
    'cte_raw',         # 4
    'cte_integral',    # 5
    'dynamic_lfd',     # 6
    'speed_ratio',     # 7
    'raw_steer',       # 8
    'clamped_steer',   # 9
    'steer_step_limit',# 10
    'target_spd',      # 11
    'signed_spd',      # 12
    'current_speed',   # 13
    'route_turn_deg',  # 14
    'dist_to_curve',   # 15
    'ahead_steer',     # 16
    'p_term',          # 17
    'i_term',          # 18
    'd_term',          # 19
    'd_term_lpf',      # 20
    'loop_dt',         # 21
    'is_rev',          # 22
    'hard_corner',     # 23
    'finish_progress', # 24
    'spd_reason_code', # 25
    'finish_mode_code',# 26
]

# 통합 타임라인에 포함할 토픽과 추출 필드
# (topic, [(csv_column_name, dotted_path_in_msg_dict), ...])
MERGED_TIMELINE_SPEC = [
    ('/cmd_vel_raw', [
        ('cmd_v_ms',  'linear.x'),
        ('cmd_steer', 'angular.z'),
    ]),
    ('/encoder', [
        ('enc_tick', 'data'),
    ]),
    ('/ego_state', [
        # ego_state 의 실제 필드명은 사용자 메시지에 따라 다르므로
        # 존재하지 않는 경로는 자동으로 건너뜀
        ('ego_x',    'x'),
        ('ego_y',    'y'),
        ('ego_yaw',  'yaw'),
        ('ego_v',    'v'),
    ]),
    ('/driving_debug', [
        ('cte',           'data[3]'),
        ('clamped_steer', 'data[9]'),
        ('target_spd',    'data[11]'),
        ('current_speed', 'data[13]'),
        ('p_term',        'data[17]'),
        ('d_term_lpf',    'data[20]'),
    ]),
]


# ═══════════════════════════════════════════════════════════════
#  유틸
# ═══════════════════════════════════════════════════════════════
def get_latest_bag_dir(base_path='.'):
    bag_dirs = [d for d in glob.glob(os.path.join(base_path, 'rosbag2_*'))
                if os.path.isdir(d)]
    if not bag_dirs:
        return None
    bag_dirs.sort(key=os.path.getmtime, reverse=True)
    return bag_dirs[0]


def flatten_dict(d, parent_key='', sep='.'):
    """중첩 dict를 평탄화. list는 'key[i]' 형태로 인덱스 표시."""
    items = []
    if isinstance(d, dict):
        for k, v in d.items():
            new_key = f'{parent_key}{sep}{k}' if parent_key else k
            items.extend(flatten_dict(v, new_key, sep).items())
    elif isinstance(d, (list, tuple)):
        # 너무 긴 배열(공분산 36개 등)은 'len' 만 기록
        if len(d) > 12:
            items.append((f'{parent_key}_len', len(d)))
        else:
            for i, v in enumerate(d):
                new_key = f'{parent_key}[{i}]'
                items.extend(flatten_dict(v, new_key, sep).items())
    else:
        items.append((parent_key, d))
    return dict(items)


def get_by_path(msg_dict, path):
    """
    'linear.x' 또는 'data[3]' 같은 점/대괄호 경로로 값을 추출.
    경로가 없으면 None 반환.
    """
    cur = msg_dict
    try:
        # 'data[3]' → ['data', '3']
        tokens = []
        buf = ''
        i = 0
        while i < len(path):
            c = path[i]
            if c == '.':
                if buf:
                    tokens.append(buf)
                    buf = ''
            elif c == '[':
                if buf:
                    tokens.append(buf)
                    buf = ''
                j = path.index(']', i)
                tokens.append(int(path[i+1:j]))
                i = j
            else:
                buf += c
            i += 1
        if buf:
            tokens.append(buf)

        for t in tokens:
            if isinstance(t, int):
                cur = cur[t]
            else:
                cur = cur[t]
        return cur
    except (KeyError, IndexError, TypeError):
        return None


# ═══════════════════════════════════════════════════════════════
#  메인 추출
# ═══════════════════════════════════════════════════════════════
def extract_latest_bag():
    latest_bag = get_latest_bag_dir()
    if not latest_bag:
        print("❌ 현재 폴더에 'rosbag2_'로 시작하는 기록 폴더가 없습니다.")
        return

    print(f"✅ 최근 기록 발견: {latest_bag}")

    output_dir = f"extracted_{os.path.basename(latest_bag)}"
    os.makedirs(output_dir, exist_ok=True)

    # ─── bag 열기 ───
    storage_options = StorageOptions(uri=latest_bag, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # 토픽별 자료구조
    jsonl_handles = {}          # topic → file handle
    rows_per_topic = defaultdict(list)   # topic → [flat_row_dict, ...]
    msg_count      = defaultdict(int)
    t_first        = defaultdict(lambda: None)
    t_last         = defaultdict(lambda: None)

    # 통합 타임라인용
    merged_rows = []
    merged_target_topics = {t for t, _ in MERGED_TIMELINE_SPEC}

    # /driving_debug 전용 named CSV
    driving_debug_rows = []

    bag_t0 = None
    total = 0

    print(f"📂 '{output_dir}' 폴더에 추출 중...")
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if bag_t0 is None:
            bag_t0 = timestamp

        t_sec = timestamp * 1e-9
        t_rel = (timestamp - bag_t0) * 1e-9

        # 메시지 디코딩
        try:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            msg_dict = message_to_ordereddict(msg)
        except Exception as e:
            print(f"⚠️ 디코딩 실패: {topic} ({e})")
            continue

        # ─── JSONL 저장 ───
        if topic not in jsonl_handles:
            safe_name = topic.replace('/', '_').strip('_')
            jsonl_path = os.path.join(output_dir, f'{safe_name}.jsonl')
            jsonl_handles[topic] = open(jsonl_path, 'w', encoding='utf-8')

        jsonl_handles[topic].write(json.dumps({
            'timestamp': timestamp,
            't_sec': t_sec,
            't_rel': t_rel,
            'data': msg_dict,
        }, ensure_ascii=False) + '\n')

        # ─── CSV용 평탄화 행 누적 ───
        flat = flatten_dict(msg_dict)
        # 헤더 등 제거하고 싶다면 아래 주석 해제
        # for k in list(flat.keys()):
        #     if k.startswith('header'):
        #         del flat[k]
        row = OrderedDict()
        row['t_sec'] = t_sec
        row['t_rel'] = t_rel
        row.update(flat)
        rows_per_topic[topic].append(row)

        # ─── /driving_debug 이름 매핑 CSV ───
        if topic == '/driving_debug':
            data_arr = msg_dict.get('data', [])
            named = OrderedDict()
            named['t_sec'] = t_sec
            named['t_rel'] = t_rel
            for i, name in enumerate(DRIVING_DEBUG_FIELDS):
                named[name] = data_arr[i] if i < len(data_arr) else None
            driving_debug_rows.append(named)

        # ─── 통합 타임라인 ───
        if topic in merged_target_topics:
            mrow = OrderedDict()
            mrow['t_sec'] = t_sec
            mrow['t_rel'] = t_rel
            mrow['source_topic'] = topic
            for ttopic, fields in MERGED_TIMELINE_SPEC:
                if ttopic != topic:
                    continue
                for col, path in fields:
                    mrow[col] = get_by_path(msg_dict, path)
            merged_rows.append(mrow)

        # 통계
        msg_count[topic] += 1
        if t_first[topic] is None:
            t_first[topic] = t_sec
        t_last[topic] = t_sec

        total += 1
        if total % 2000 == 0:
            print(f'... {total}개 처리 중 ...')

    # JSONL 핸들 닫기
    for f in jsonl_handles.values():
        f.close()

    # ─── 토픽별 CSV 저장 ───
    for topic, rows in rows_per_topic.items():
        if not rows:
            continue
        safe_name = topic.replace('/', '_').strip('_')
        csv_path = os.path.join(output_dir, f'{safe_name}.csv')

        # 모든 행의 키 합집합 → 열 순서 결정
        all_keys = ['t_sec', 't_rel']
        seen = set(all_keys)
        for r in rows:
            for k in r.keys():
                if k not in seen:
                    all_keys.append(k)
                    seen.add(k)

        with open(csv_path, 'w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=all_keys)
            writer.writeheader()
            for r in rows:
                writer.writerow({k: r.get(k, '') for k in all_keys})

    # ─── driving_debug_named.csv ───
    if driving_debug_rows:
        path = os.path.join(output_dir, 'driving_debug_named.csv')
        with open(path, 'w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(
                f, fieldnames=['t_sec', 't_rel'] + DRIVING_DEBUG_FIELDS)
            writer.writeheader()
            writer.writerows(driving_debug_rows)
        print(f'📊 driving_debug_named.csv ({len(driving_debug_rows)}행)')

    # ─── merged_timeline.csv ───
    if merged_rows:
        # 열 순서: t_sec, t_rel, source_topic, 그 뒤 등장 순
        all_keys = ['t_sec', 't_rel', 'source_topic']
        seen = set(all_keys)
        for r in merged_rows:
            for k in r.keys():
                if k not in seen:
                    all_keys.append(k)
                    seen.add(k)
        merged_rows.sort(key=lambda x: x['t_sec'])

        path = os.path.join(output_dir, 'merged_timeline.csv')
        with open(path, 'w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=all_keys)
            writer.writeheader()
            for r in merged_rows:
                writer.writerow({k: r.get(k, '') for k in all_keys})
        print(f'📈 merged_timeline.csv ({len(merged_rows)}행)')

    # ─── 요약 ───
    summary_path = os.path.join(output_dir, '_summary.txt')
    with open(summary_path, 'w', encoding='utf-8') as f:
        f.write(f'rosbag: {latest_bag}\n')
        f.write(f'총 메시지: {total}\n')
        if bag_t0 is not None:
            duration = (t_last[max(t_last, key=lambda k: t_last[k])]
                        - bag_t0 * 1e-9)
            f.write(f'기록 시간: {duration:.2f} s\n')
        f.write('\n토픽별 통계:\n')
        f.write(f'{"topic":40s} {"count":>8s} {"dur_s":>8s} '
                f'{"hz":>7s} {"type":s}\n')
        f.write('-' * 100 + '\n')
        for topic in sorted(msg_count.keys()):
            cnt  = msg_count[topic]
            dur  = (t_last[topic] - t_first[topic]) if cnt > 1 else 0.0
            hz   = (cnt - 1) / dur if dur > 0 else 0.0
            tp   = type_map.get(topic, '?')
            f.write(f'{topic:40s} {cnt:>8d} {dur:>8.2f} '
                    f'{hz:>7.2f} {tp:s}\n')

    print(f'📝 _summary.txt 작성 완료')
    print(f'\n🎉 완료! 총 {total}개 메시지 → {output_dir}/')


if __name__ == '__main__':
    extract_latest_bag()
