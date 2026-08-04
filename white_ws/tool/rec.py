#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rec.py  -  rosbag2 추출기 (분석 친화적 버전)
─────────────────────────────────────────────────────────────
[v1.1] 어디서 실행하든(cwd 무관) ~/white_ws/white_ws/ros2bag 를 고정
       기준으로 최신 rosbag2_* 를 찾고 extracted_* 도 그 안에 생성한다
       (map_check.py의 기본 경로와 동일 — 새 로스백이 안 보이던 문제 해결).

[v1.2] 토픽 화이트리스트(KEEP_TOPICS) + 로스백 직접 지정(--bag) 추가.
       · 이미지 토픽(/image_raw, /lane/debug 등)은 기본 스킵한다. 1920x1080
         Image 1장이 JSONL로 약 25MB라 362장이면 9GB — 저장소 계층
         (StorageFilter)에서 걸러 디코딩조차 하지 않는다.
       · 자동 선택 시 알맹이 없는 로스백(노드가 안 뜬 채 녹화돼 /rosout 만
         든 것)은 건너뛴다. 예전엔 mtime 최신이라는 이유로 그런 빈 로스백이
         뽑혀서 lane_metrics.jsonl 이 안 생겼다.

최신 rosbag2_* 폴더를 자동 탐색(또는 --bag 지정)하여 다음 파일들을 생성:

  extracted_<bag_name>/
    ├─ <topic>.jsonl              : 토픽별 원본(JSON Lines)
    ├─ <topic>.csv                : 토픽별 표 형태 (pandas/엑셀 호환)
    ├─ driving_debug_named.csv    : /driving_debug 32필드를 이름 있는 열로 펼침
    │                               (구버전 27필드 bag은 뒤 5칸이 빈칸으로 남음)
    ├─ lane_metrics_named.csv     : /lancd e_metrics 10필드
    ├─ fusion_debug_named.csv     : /fusion_debug 13필드
    ├─ merged_timeline.csv        : 핵심 신호 통합 시계열
    └─ _summary.txt               : 토픽별 통계 + 스킵된 토픽 목록

실행:
  python3 rec.py                          # 최신(알맹이 있는) 로스백 자동 추출
  python3 rec.py --bag rosbag2_2026_07_15-22_24_21   # 특정 로스백 지정
  python3 rec.py --list                   # 로스백 목록만 보기
  python3 rec.py --all-topics             # 필터 해제(이미지 포함 전부 — 매우 큼)
─────────────────────────────────────────────────────────────
"""

import os
import glob
import json
import csv
import argparse
from collections import OrderedDict, defaultdict

from rosbag2_py import (SequentialReader, StorageOptions, ConverterOptions,
                        StorageFilter)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict

# [v1.1] rec.py를 어느 디렉터리에서 실행하든(cwd 무관) 항상 이 고정 경로의
# rosbag2_*를 찾고 extracted_*도 여기에 생성한다. map_check.py의
# DEFAULT_BAG_DIR과 반드시 같은 경로로 유지할 것 — 다르면 "새 로스백/루트가
# 갱신 안 되는" 증상(추출 결과가 엉뚱한 cwd에 생기는 문제)이 재발한다.
ROS2BAG_DIR = os.path.expanduser("~/white_ws/white_ws/ros2bag")

# ═══════════════════════════════════════════════════════════════
#  [v1.2] 추출할 토픽 화이트리스트
#    여기 없는 토픽은 StorageFilter 단계에서 통째로 스킵된다.
#    ★새 분석을 추가해 토픽이 더 필요해지면 여기에 한 줄 추가할 것.
#      (예: GPS 단절 구간을 따로 보고 싶으면 '/gps_status' — 다만 단절 여부는
#       /fusion_debug 의 dr_active(6) 에도 이미 들어 있다.)
# ═══════════════════════════════════════════════════════════════
KEEP_TOPICS = {
    # ── map_check_fusion.py 가 직접 읽는 4개 ──────────────────
    '/ego_state',     # 궤적·heading·speed        (필수)
    '/fix',           # GPS 원본, (F) 패널 잔차    (필수)
    '/lane_metrics',  # 카메라 차선 → ④⑤ 오버레이 + ⑦ 융합 시뮬
    '/imu/data',      # 요각속도 적분 → ⑦ 융합 시뮬
    # ── IMU/카메라 헤딩보정 검증용 named CSV ──────────────────
    '/fusion_debug',  # gps_imu 헤딩보정 계측 (cam_head_applied 등 13필드)
    '/driving_debug', # 횡보정 섀도 계측 (cam_lat_err 27~31) + merged_timeline
    '/lane/state',
    # ── merged_timeline.csv 구성에만 쓰이는 나머지 ────────────
    '/cmd_vel_raw',
    '/encoder',
    # ── [신호등/횡단보도 게이트 검증] ─────────────────────────
    #   perception 인지(/tl/state·/stop_line_dist) → camera_judgment 판단(/judgment_state) →
    #   게이트 전/후 속도(/cmd_vel_drive vs /cmd_vel_raw) 를 시간정렬로 대조하면
    #   "빨간불 인지→정지→초록불 재출발" 이 제대로 됐는지 로그로 확인된다.
    #   driving 은 이제 /judgment_state 를 구독해 TL_STOP 중 진행/타이머/적분을 동결한다.
    '/tl/state',        # perception: 신호등 분류 (GREEN/RED/RED_FAR/UNKNOWN)
    '/stop_line_dist',  # perception: 정지선까지 거리 [m]
    '/crosswalk_dist',  # perception: 횡단보도까지 거리 [m]
    '/judgment_state',  # camera_judgment: 게이트 상태 (PASS/TL_STOP/TL_BRAKE/TL_SLOW/CW_SLOW)
    '/cmd_vel_drive',   # driving 원명령(게이트 전) — /cmd_vel_raw(게이트 후)와 대조
    # ── [속도 상한 검증] ──────────────────────────────────────
    #   아두이노 속도PID 의 실제 PWM. 포화(상한 붙음) 여부를 직접 확인해야
    #   "속도 부족이 출력한계 때문인지"가 추측이 아니라 측정이 된다.
    '/motor_pwm',
    # ── [조향 지연 실측] ──────────────────────────────────────
    #   A15 포텐셔미터 실측 조향각(deg). driving_debug 의 clamped_steer(명령)와
    #   시간정렬해 비교하면 명령→실현 지연·비율을 추정이 아니라 측정으로 구한다.
    '/steer_angle_measured',
    #   조향PID 의 실제 PWM. /steer_angle_measured 와 짝지어 보면 각도가 안 변하는
    #   구간이 기구 고착(PWM 큼)인지 전원/PID 미출력(PWM≈0)인지 가른다.
    '/steer_pwm',
    # ── [융합 설정 기록] ──────────────────────────────────────
    #   gps_imu 가 기동 시 + 파라미터 변경 시마다 발행하는 설정 문자열
    #   (mode / cam_head ON·OFF / cam_trust ...). 이게 없으면 "이 로스백이 어떤
    #   설정으로 찍혔는지"를 알 수 없어 A/B 비교 자체가 성립하지 않는다.
    '/fusion_config',
}

# 이 중 하나라도 실제 메시지가 있어야 '알맹이 있는 로스백'으로 본다.
# (노드가 안 뜬 채 녹화되면 /rosout 만 몇 개 든 로스백이 생기는데, mtime 은
#  그게 최신이라 자동선택에 걸려버린다 — 그걸 거르는 용도.)
CORE_TOPICS = {'/ego_state', '/fix'}


# ═══════════════════════════════════════════════════════════════
#  /driving_debug 32필드 이름 매핑 (driving.py _publish_debug 와 일치)
#    0~26 : 기존 경로추종 필드
#    27~31: [카메라 융합] GPS단절 횡보정 섀도 계측 (driving v6.8)
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
    # [2026-07-29 수정] 14/16/23/24/26 이름이 낡아 실제 발행값과 달랐다.
    #   driving.py _publish_debug 실제 순서 기준으로 정정(주석의 '23=near피크,26=far피크' 반영).
    #   구 이름(route_turn_deg/ahead_steer/hard_corner/finish_progress/finish_mode_code)으로
    #   분석하면 완전히 다른 물리량을 보게 된다 — 실제로 오분석이 났다.
    'near_demand_deg', # 14  근거리 요구조향[deg] (구: route_turn_deg)
    'far_dist',        # 15  다가오는 코너까지 거리[m] (99=없음)
    'far_demand_deg',  # 16  원거리 요구조향[deg] — 감속 게이팅용 (구: ahead_steer)
    'p_term',          # 17
    'i_term',          # 18
    'd_term',          # 19
    'd_term_lpf',      # 20
    'loop_dt',         # 21
    'is_rev',          # 22
    'near_peak_deg',   # 23  근거리 곡률 피크[deg] (구: hard_corner)
    'approach_active', # 24  종점 접근 감속 활성 0/1 (구: finish_progress)
    'spd_reason_code', # 25  SPD_REASON_CODE (driving.py 상단 표)
    'far_peak_deg',    # 26  원거리 곡률 피크[deg] (구: finish_mode_code)
    # ── [카메라 융합] GPS단절 횡보정 섀도 계측 ──────────────────────────────
    #   cam_lat_err = cte_now − lane_cte[wp] = '매핑 선 대비 횡오차'.
    #   ★검증: RTK 정상 구간에서 cte(3) 와 cam_lat_err(27) 를 겹쳐 볼 것.
    #     부호·크기 일치 → driving.CAM_LAT_ENABLE=True 로 켜도 된다.
    #     부호만 반대     → driving.CAM_LAT_SIGN=-1.0.
    #     무상관          → 켜지 말 것(차선 인식/캘리브 문제).
    'cam_lat_err',     # 27  매핑 선 대비 횡오차 [m] (우측+)
    'cam_lat_ok',      # 28  게이트 통과(1) / 차단(0)
    'cam_cte_now',     # 29  현재 카메라 cte_rear [m]
    'cam_cte_map',     # 30  맵에 기록된 그 지점의 cte [m]
    'cam_lat_bias',    # 31  실제 적용 중인 누적 보정량 [m] (ENABLE=False면 항상 0)
]

# ═══════════════════════════════════════════════════════════════
#  /lane_metrics 10필드 (camera_judgment 브리지 출력과 일치)
# ═══════════════════════════════════════════════════════════════
LANE_METRICS_FIELDS = [
    'cte_rear_m',      # 0
    'cte_near_m',      # 1
    'theta_lane_deg',  # 2
    'curvature_1pm',   # 3
    'conf_eff',        # 4
    'lane_width_m',    # 5
    'flags',           # 6
    'd_near_m',        # 7
    'conf_raw',        # 8
    'seq',             # 9
]

# ═══════════════════════════════════════════════════════════════
#  /fusion_debug 13필드 (gps_imu.py _publish_fusion_debug 와 일치)
# ═══════════════════════════════════════════════════════════════
FUSION_DEBUG_FIELDS = [
    'cam_ok',            # 0
    'cam_cte_rear',      # 1
    'theta_lane_deg',    # 2
    'cam_conf',          # 3
    'cam_age_s',         # 4
    'flags',             # 5
    'dr_active',         # 6
    'cam_head_applied',  # 7  이번 보정량 [deg]
    'cam_head_count',    # 8  누적 보정횟수
    'cam_head_total',    # 9  누적 보정량 [deg]
    'heading',           # 10
    'yaw_offset',        # 11
    'loop_dt',           # 12
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
        # [0713] /ego_state는 Float64MultiArray — gps_imu.py 규약(main_loop 발행순):
        #   data[0]=lat data[1]=lon data[2]=fused_x data[3]=fused_y
        #   data[4]=heading data[5]=speed data[6]=(미사용) data[7]=pitch data[8]=terrain_code
        # 예전엔 'x'/'y'/'yaw'/'v' 경로를 썼는데 이 메시지엔 그런 필드가 없어 항상 공란이었음.
        ('ego_lat',  'data[0]'),
        ('ego_lon',  'data[1]'),
        ('ego_x',    'data[2]'),
        ('ego_y',    'data[3]'),
        ('ego_yaw',  'data[4]'),
        ('ego_v',    'data[5]'),
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
def list_bag_dirs(base_path=ROS2BAG_DIR):
    """ROS2BAG_DIR 안의 rosbag2_* 폴더를 최신(mtime) 순으로 반환."""
    bag_dirs = [d for d in glob.glob(os.path.join(base_path, 'rosbag2_*'))
                if os.path.isdir(d)]
    bag_dirs.sort(key=os.path.getmtime, reverse=True)
    return bag_dirs


def bag_topic_counts(bag_dir):
    """metadata.yaml 에서 {토픽명: 메시지수} 를 읽는다. 실패하면 None.
    (bag 을 열지 않고 메타데이터만 보므로 2GB 짜리라도 즉시 끝난다.)"""
    meta = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.exists(meta):
        return None
    try:
        import yaml
        with open(meta, encoding='utf-8') as f:
            info = yaml.safe_load(f)['rosbag2_bagfile_information']
        return {t['topic_metadata']['name']: t['message_count']
                for t in info['topics_with_message_count']}
    except Exception:
        return None


def get_latest_bag_dir(base_path=ROS2BAG_DIR):
    """최신 rosbag2_* 중 CORE_TOPICS 가 실제로 든 것을 고른다.
    빈 로스백(노드 미기동 상태로 녹화된 /rosout 뿐인 것)은 건너뛰고 이유를 알린다."""
    bag_dirs = list_bag_dirs(base_path)
    if not bag_dirs:
        return None

    for d in bag_dirs:
        counts = bag_topic_counts(d)
        if counts is None:
            # 메타데이터를 못 읽으면 판단 보류 — 일단 후보로 인정한다.
            return d
        if any(counts.get(t, 0) > 0 for t in CORE_TOPICS):
            return d
        print(f"⏭  건너뜀: {os.path.basename(d)} "
              f"(메시지 {sum(counts.values())}개, {'/'.join(sorted(CORE_TOPICS))} 없음)")

    # 전부 알맹이가 없으면 그래도 최신을 돌려주되 경고한다.
    print(f"⚠️ CORE_TOPICS({'/'.join(sorted(CORE_TOPICS))})가 든 로스백이 없습니다. "
          f"최신 것으로 진행합니다.")
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
def extract_latest_bag(bag_path=None, keep_topics=KEEP_TOPICS):
    """bag_path를 지정하면 그 로스백을, 없으면 ROS2BAG_DIR의 최신 로스백을 추출.
    keep_topics=None 이면 필터 없이 전 토픽을 추출한다(이미지 포함 — 매우 큼)."""
    latest_bag = bag_path or get_latest_bag_dir()
    if not latest_bag:
        print(f"❌ {ROS2BAG_DIR} 안에 'rosbag2_'로 시작하는 기록 폴더가 없습니다.")
        return

    print(f"✅ 최근 기록 발견: {latest_bag}")

    # [v1.1] cwd 무관 — 항상 ROS2BAG_DIR 안에 생성(map_check.py가 보는 위치와 일치)
    output_dir = os.path.join(ROS2BAG_DIR, f"extracted_{os.path.basename(latest_bag)}")
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

    # ─── [v1.2] 토픽 필터 ───
    #   StorageFilter 는 db3 쿼리 단계에서 거르므로 스킵 토픽의 blob 은
    #   읽지도 디코딩하지도 않는다. 아래 루프의 in-loop 가드는 이 API 가
    #   조용히 무시될 경우를 대비한 안전망.
    skipped = sorted(set(type_map) - set(keep_topics)) if keep_topics else []
    if keep_topics:
        present = sorted(set(type_map) & set(keep_topics))
        reader.set_filter(StorageFilter(topics=present))
        missing = sorted(set(keep_topics) - set(type_map))
        print(f"🔍 추출 토픽 {len(present)}개: {', '.join(present)}")
        if missing:
            print(f"   (이 로스백에 없음: {', '.join(missing)})")
        if skipped:
            print(f"⏭  스킵 {len(skipped)}개: {', '.join(skipped)}")
    else:
        print("⚠️ --all-topics: 필터 없이 전 토픽 추출 (이미지 포함 — 수 GB 가능)")

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
    # [카메라 융합] /lane_metrics, /fusion_debug 전용 named CSV
    lane_metrics_rows = []
    fusion_debug_rows = []

    bag_t0 = None
    total = 0

    print(f"📂 '{output_dir}' 폴더에 추출 중...")
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # [v1.2] 안전망 — set_filter 가 안 먹은 경우에도 여기서 걸러진다.
        #   bag_t0 확정보다 먼저 걸러야 스킵 토픽이 t_rel 기준시각을 못 건드린다.
        if keep_topics and topic not in keep_topics:
            continue

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

        # ─── [카메라 융합] /lane_metrics 이름 매핑 CSV ───
        if topic == '/lane_metrics':
            data_arr = msg_dict.get('data', [])
            named = OrderedDict()
            named['t_sec'] = t_sec
            named['t_rel'] = t_rel
            for i, name in enumerate(LANE_METRICS_FIELDS):
                named[name] = data_arr[i] if i < len(data_arr) else None
            lane_metrics_rows.append(named)

        # ─── [카메라 융합] /fusion_debug 이름 매핑 CSV ───
        if topic == '/fusion_debug':
            data_arr = msg_dict.get('data', [])
            named = OrderedDict()
            named['t_sec'] = t_sec
            named['t_rel'] = t_rel
            for i, name in enumerate(FUSION_DEBUG_FIELDS):
                named[name] = data_arr[i] if i < len(data_arr) else None
            fusion_debug_rows.append(named)

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

    # ─── [카메라 융합] lane_metrics_named.csv ───
    if lane_metrics_rows:
        path = os.path.join(output_dir, 'lane_metrics_named.csv')
        with open(path, 'w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(
                f, fieldnames=['t_sec', 't_rel'] + LANE_METRICS_FIELDS)
            writer.writeheader()
            writer.writerows(lane_metrics_rows)
        print(f'📊 lane_metrics_named.csv ({len(lane_metrics_rows)}행)')

    # ─── [카메라 융합] fusion_debug_named.csv ───
    if fusion_debug_rows:
        path = os.path.join(output_dir, 'fusion_debug_named.csv')
        with open(path, 'w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(
                f, fieldnames=['t_sec', 't_rel'] + FUSION_DEBUG_FIELDS)
            writer.writeheader()
            writer.writerows(fusion_debug_rows)
        print(f'📊 fusion_debug_named.csv ({len(fusion_debug_rows)}행)')

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
        # t_last 가 비면 max() 가 ValueError — 필터로 남은 토픽이 0개일 때 실제로 발생한다.
        if bag_t0 is not None and t_last:
            duration = (t_last[max(t_last, key=lambda k: t_last[k])]
                        - bag_t0 * 1e-9)
            f.write(f'기록 시간: {duration:.2f} s\n')
        if skipped:
            f.write(f'\n스킵된 토픽 ({len(skipped)}개, KEEP_TOPICS 화이트리스트 밖):\n')
            for s in skipped:
                f.write(f'  - {s:38s} {type_map.get(s, "?")}\n')
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


def main():
    ap = argparse.ArgumentParser(
        description='rosbag2 추출기 — 기본은 분석에 필요한 토픽만 뽑는다.')
    ap.add_argument('--bag', default=None,
                    help='추출할 로스백. 폴더명만 줘도 되고(ROS2BAG_DIR 기준) '
                         '전체 경로도 된다. 생략 시 최신 로스백 자동 선택.')
    ap.add_argument('--list', action='store_true',
                    help='로스백 목록과 핵심 토픽 유무만 출력하고 종료')
    ap.add_argument('--all-topics', action='store_true',
                    help='KEEP_TOPICS 필터 해제 — 이미지 포함 전 토픽 추출(수 GB)')
    args = ap.parse_args()

    if args.list:
        bags = list_bag_dirs()
        if not bags:
            print(f'❌ {ROS2BAG_DIR} 에 rosbag2_* 폴더가 없습니다.')
            return
        print(f'\n{ROS2BAG_DIR} (최신순)\n')
        print(f'{"로스백":42s} {"메시지":>8s}  {"ego":>4s} {"fix":>4s} {"lane":>5s} {"imu":>4s}')
        print('-' * 78)
        for d in bags:
            c = bag_topic_counts(d) or {}
            def mark(t):
                n = c.get(t, 0)
                return str(n) if n else '·'
            print(f'{os.path.basename(d):42s} {sum(c.values()):>8d}  '
                  f'{mark("/ego_state"):>4s} {mark("/fix"):>4s} '
                  f'{mark("/lane_metrics"):>5s} {mark("/imu/data"):>4s}')
        print()
        return

    bag = args.bag
    if bag:
        # 폴더명만 준 경우 ROS2BAG_DIR 기준으로 해석
        if not os.path.isdir(bag):
            cand = os.path.join(ROS2BAG_DIR, bag)
            if not os.path.isdir(cand):
                print(f'❌ 로스백을 찾을 수 없습니다: {bag}')
                print(f'   (--list 로 목록 확인)')
                return
            bag = cand
        bag = os.path.abspath(bag)

    extract_latest_bag(bag_path=bag,
                       keep_topics=None if args.all_topics else KEEP_TOPICS)


if __name__ == '__main__':
    main()
