#!/usr/bin/env python3
"""
Trajectory Comparison & Camera-Fusion Tool
==========================================
map_check.py 를 원본으로 그대로 확장한 버전입니다.
  · 실행 방식(대화형 파일 선택)과 6-패널 분석(A~F)은 map_check.py 와 100% 동일하게 유지.
  · (A) 궤적 패널에 실제 주행 중 카메라로 인식된 차선(양쪽)을 추가로 오버레이합니다
    — 맵핑 경로 옆에 표시. 전부 실제 기록된 위치·카메라 데이터이며, 예상/재구성
    경로(시뮬레이션) 오버레이는 없습니다. [2026-07-30] IMU+카메라 단독 재구성
    (연속/구간 dropout 두 가지 모두)을 제거함 — 실측 데이터만 표시.

실행 방법 (map_check.py 와 동일):
    python3 map_check_fusion.py                         # 기본 경로 자동 탐색 후 대화형 선택
    python3 map_check_fusion.py \
        --route-dir ~/white_ws/gps_data \
        --bag-dir   ~/white_ws/ros2bag  \
        --out-dir   ~/white_ws/results  \
        --no-show                        # 창 표시 없이 이미지만 저장

의존성:
    pip install pandas matplotlib numpy scipy
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.spatial import KDTree


# ══════════════════════════════════════════════════════
#  기본 경로 (스크립트 위치 기준 자동 산출 → OS 무관)
# ══════════════════════════════════════════════════════
BASE_DIR          = Path(__file__).resolve().parent.parent
DEFAULT_ROUTE_DIR = BASE_DIR / "gps_data"
DEFAULT_BAG_DIR   = BASE_DIR / "ros2bag"

W = 60  # 터미널 박스 너비


# ══════════════════════════════════════════════════════
#  터미널 UI 헬퍼
# ══════════════════════════════════════════════════════

def box_top(title: str):
    print(f"\n╔{'═' * W}╗")
    pad = (W - len(title)) // 2
    print(f"║{' ' * pad}{title}{' ' * (W - pad - len(title))}║")
    print(f"╠{'═' * W}╣")


def box_row(text: str):
    # 한글 포함 시 글자 폭 보정 (한글 1자 = 2칸)
    display_len = sum(2 if ord(c) > 0x7F else 1 for c in text)
    pad = max(0, W - 2 - display_len)
    print(f"║  {text}{' ' * pad}║")


def box_sep():
    print(f"╠{'─' * W}╣")


def box_bot():
    print(f"╚{'═' * W}╝")


def prompt_int(prompt: str, lo: int, hi: int) -> int:
    """lo ~ hi 범위의 정수를 입력받을 때까지 반복."""
    while True:
        try:
            raw = input(f"\n  {prompt} [{lo}~{hi}]: ").strip()
            val = int(raw)
            if lo <= val <= hi:
                return val
            print(f"  ⚠  {lo}~{hi} 사이의 숫자를 입력하세요.")
        except (ValueError, EOFError):
            print("  ⚠  숫자를 입력하세요.")


# ══════════════════════════════════════════════════════
#  파일 선택 대화형 로직 (map_check.py 와 동일, lane 표시만 추가)
# ══════════════════════════════════════════════════════

def select_route(route_dir: Path) -> Path:
    """gps_data 폴더에서 route CSV 선택."""
    csvs = sorted(route_dir.glob("route_*.csv"))
    if not csvs:
        sys.exit(f"\n오류: {route_dir} 에 route_*.csv 파일이 없습니다.")

    box_top("STEP 1 / 2  —  사전 매핑 경로 파일 선택")
    for i, p in enumerate(csvs, 1):
        size_kb = p.stat().st_size // 1024
        box_row(f"[{i:2d}]  {p.name:<40}  {size_kb:>4} KB")
    box_bot()

    idx = prompt_int("번호 입력", 1, len(csvs))
    chosen = csvs[idx - 1]
    print(f"\n  ✔  선택됨: {chosen.name}")
    return chosen


def select_bag_folder(bag_dir: Path) -> Path:
    """ros2bag 폴더에서 extracted_* 폴더 선택. (ego/fix 필수, lane 은 있으면 표시)"""
    folders = sorted(
        p for p in bag_dir.iterdir()
        if p.is_dir() and p.name.startswith("extracted_")
    )
    if not folders:
        sys.exit(f"\n오류: {bag_dir} 에 extracted_* 폴더가 없습니다.")

    box_top("STEP 2 / 2  —  추출된 ROS2 bag 폴더 선택")
    box_row(f"{'':5}{'폴더명':<40}  ego fix lane")
    box_sep()
    for i, p in enumerate(folders, 1):
        ego_ok  = "✓" if (p / "ego_state.jsonl").exists()    else "✗"
        fix_ok  = "✓" if (p / "fix.jsonl").exists()          else "✗"
        lane_ok = "✓" if (p / "lane_metrics.jsonl").exists() else "·"
        mark    = "▶" if ego_ok == "✓" and fix_ok == "✓" else " "
        box_row(f"{mark}[{i:2d}]  {p.name:<40}   {ego_ok}   {fix_ok}   {lane_ok}")
    box_bot()
    box_row("lane ✓ = 카메라 차선 데이터 있음 (융합 오버레이 가능)")
    box_bot()

    idx = prompt_int("번호 입력", 1, len(folders))
    chosen = folders[idx - 1]
    print(f"\n  ✔  선택됨: {chosen.name}")
    return chosen


def resolve_paths(bag_dir: Path, route_dir: Path):
    """대화형으로 파일 경로를 확정하고 반환."""
    route_path = select_route(route_dir)
    bag_folder = select_bag_folder(bag_dir)

    ego_path   = bag_folder / "ego_state.jsonl"
    fix_path   = bag_folder / "fix.jsonl"
    lane_path  = bag_folder / "lane_metrics.jsonl"   # 없으면 융합 생략
    imu_path   = bag_folder / "imu_data.jsonl"
    state_path = bag_folder / "lane_state.jsonl"     # 원시 차선 다항식 (없으면 곡선 생략)

    missing = [str(p) for p in [ego_path, fix_path] if not p.exists()]
    if missing:
        sys.exit("\n오류: 다음 파일이 없습니다:\n  " + "\n  ".join(missing))

    print(f"\n{'─' * W}")
    print(f"  route        : {route_path.name}")
    print(f"  ego_state    : {ego_path}")
    print(f"  fix          : {fix_path}")
    print(f"  lane_metrics : {'(없음 → 융합 오버레이 생략)' if not lane_path.exists() else lane_path.name}")
    print(f"  imu_data     : {'(없음 → 순수 IMU 경로 근사)' if not imu_path.exists() else imu_path.name}")
    print(f"  lane_state   : {'(없음 → 원시 차선곡선 생략 — rec.py 재추출 필요)' if not state_path.exists() else state_path.name}")
    print(f"{'─' * W}\n")

    return ego_path, fix_path, route_path, lane_path, imu_path, state_path


# ══════════════════════════════════════════════════════
#  데이터 로드 (map_check.py 와 동일 + lane/imu 추가)
# ══════════════════════════════════════════════════════

def load_ego_state(path: Path) -> pd.DataFrame:
    """ego_state.jsonl  →  data.data = [lat, lon, x, y, heading°, speed, ...]"""
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            ts  = obj["timestamp"]
            d   = obj["data"]["data"]
            # ego_state 규약(실측 확인):
            #   d[0]=lat, d[1]=lon, d[2]=fused_x, d[3]=fused_y,
            #   d[4]=heading[deg], d[5]=speed[m/s]
            heading = d[4]
            speed   = d[5] if len(d) > 5 else 0.0
            records.append({
                "timestamp_ns": ts,
                "timestamp_s":  ts * 1e-9,
                "lat":          d[0],
                "lon":          d[1],
                "x_m":          d[2],
                "y_m":          d[3],
                "heading":      heading,
                "speed":        speed,
            })
    df = pd.DataFrame(records).sort_values("timestamp_s").reset_index(drop=True)
    df["elapsed_s"] = df["timestamp_s"] - df["timestamp_s"].iloc[0]
    return df


def load_fix(path: Path) -> pd.DataFrame:
    """fix.jsonl  →  GPS NavSatFix"""
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            ts  = obj["timestamp"]
            d   = obj["data"]
            records.append({
                "timestamp_ns": ts,
                "timestamp_s":  ts * 1e-9,
                "lat":          d["latitude"],
                "lon":          d["longitude"],
                "altitude":     d["altitude"],
                "gps_status":   d["status"]["status"],
            })
    df = pd.DataFrame(records).sort_values("timestamp_s").reset_index(drop=True)
    df["elapsed_s"] = df["timestamp_s"] - df["timestamp_s"].iloc[0]
    return df


def load_route(path: Path) -> pd.DataFrame:
    """route CSV  →  latitude, longitude, heading, speed, steer"""
    df = pd.read_csv(path)
    df = df.rename(columns={"latitude": "lat", "longitude": "lon"})
    return df.reset_index(drop=True)


def load_lane_metrics(path: Path) -> pd.DataFrame:
    """lane_metrics.jsonl → 카메라 차선 인식 지표.
    data.data = [cte_rear_m, cte_near_m, theta_lane_deg, curvature, conf_eff,
                 lane_width_m, flags, d_near_m, conf_raw, seq]
    """
    if not path.exists():
        return pd.DataFrame()
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            d   = obj["data"]["data"]
            if len(d) < 7:
                continue
            records.append({
                "timestamp_s":    obj.get("t_sec", obj["timestamp"] * 1e-9),
                "cte_rear":       d[0],
                "theta_lane_deg": d[2],
                "conf_eff":       d[4],
                "lane_width":     d[5],
                "flags":          int(d[6]),
            })
    if not records:
        return pd.DataFrame()
    return pd.DataFrame(records).sort_values("timestamp_s").reset_index(drop=True)


def load_lane_state(path: Path) -> pd.DataFrame:
    """lane_state.jsonl → perception 원시 차선 다항식 (BEV 픽셀 좌표계).
    data.data = [aL, bL, cL, aR, bR, cR, conf, half_w_px]
        x_px(y_px) = a·y² + b·y + c   (y=0 이 화면 위쪽=먼 곳, y=bev_h 가 차량 쪽)
    카메라가 '실제로 본' 차선 모양 그 자체 — lane_metrics 는 이걸 스칼라로
    압축한 것이고, 여기서는 곡선 전체를 지도좌표로 투영해 그린다.
    """
    if not path.exists():
        return pd.DataFrame()
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            d   = obj["data"]["data"]
            if len(d) < 8:
                continue
            records.append({
                "timestamp_s": obj.get("t_sec", obj["timestamp"] * 1e-9),
                "aL": d[0], "bL": d[1], "cL": d[2],
                "aR": d[3], "bR": d[4], "cR": d[5],
                "conf": d[6],
            })
    if not records:
        return pd.DataFrame()
    return pd.DataFrame(records).sort_values("timestamp_s").reset_index(drop=True)


def load_imu_yaw(path: Path) -> pd.DataFrame:
    """imu_data.jsonl → 요각속도(z, rad/s). (+z 적분이 ego heading 재현 확인됨)"""
    if not path.exists():
        return pd.DataFrame()
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            d   = obj["data"]
            wz  = d.get("angular_velocity", {}).get("z", 0.0)
            records.append({
                "timestamp_s": obj.get("t_sec", obj["timestamp"] * 1e-9),
                "yaw_rate":    wz,
            })
    if not records:
        return pd.DataFrame()
    return pd.DataFrame(records).sort_values("timestamp_s").reset_index(drop=True)


# ══════════════════════════════════════════════════════
#  좌표 변환 / 분석 (map_check.py 와 동일)
# ══════════════════════════════════════════════════════

def latlon_to_meters(lat, lon, lat0, lon0):
    R = 6_371_000.0
    x = np.radians(lon - lon0) * R * np.cos(np.radians(lat0))
    y = np.radians(lat - lat0) * R
    return x, y


def _to_meters_xy(lat, lon, lat0):
    R  = 6_371_000.0
    sx = np.radians(1) * R * np.cos(np.radians(lat0))
    sy = np.radians(1) * R
    return np.column_stack([lon * sx, lat * sy])


def compute_cross_track_error(ego_lat, ego_lon, route_lat, route_lon):
    lat0    = route_lat.mean()
    route_m = _to_meters_xy(route_lat, route_lon, lat0)
    ego_m   = _to_meters_xy(ego_lat,   ego_lon,   lat0)
    dist, _ = KDTree(route_m).query(ego_m)
    return dist


def compute_heading_error(ego_heading, route, ego_lat, ego_lon):
    lat0    = route["lat"].mean()
    route_m = _to_meters_xy(route["lat"].values, route["lon"].values, lat0)
    ego_m   = _to_meters_xy(ego_lat, ego_lon, lat0)
    _, idx  = KDTree(route_m).query(ego_m)
    diff    = ego_heading - route["heading"].values[idx]
    return (diff + 180) % 360 - 180


def report_cam_head_sign(ego: pd.DataFrame, hdg_err: np.ndarray,
                         conf_min: float = 0.5, gyro_max_dps: float = 10.0,
                         min_speed: float = 0.3, max_lag_s: float = 1.2):
    """[CAM_HEAD_SIGN 실측] gps_imu 의 카메라 헤딩보정 부호를 GPS/맵 정답지로 판정한다.

    gps_imu 는 theta_lane 을 '지도접선 − 헤딩'(CCW+)으로 선언하고
        yaw_offset += CAM_HEAD_SIGN · gain · theta_lane
    로 정렬한다. 반면 여기 hdg_err 는 '헤딩 − 접선'이라 정의상 정확히 반대다. 따라서
        theta_lane ≈ −hdg_err   →   회귀 기울기 ≈ −1  이 나와야 규약이 맞다.

      slope ≈ −1 → 규약 성립 → CAM_HEAD_SIGN = +1.0 (현행 유지)
      slope ≈ +1 → theta 부호 반대 → CAM_HEAD_SIGN = -1.0 으로 뒤집을 것

    ★[2026-07-16 확정] 이 판정으로 CAM_HEAD_SIGN=+1.0 이 실측확정됐다
      (rosbag2_2026_07_15-22_24_21: n=90 피크 lag=0.70s slope=-0.45 R²=0.68 → 규약 성립).
      이 블록은 이후 주행에서의 회귀 재검용으로 유지한다 — 카메라 재장착/캘리브 변경 시
      slope 부호가 뒤집히면 즉시 잡아내는 안전망.

    게이트는 gps_imu._apply_cam_heading_correction 과 맞춘다 — 보정이 실제로 걸리는
    표본에서만 판정해야 의미가 있다.
      · conf 하한 / 직진(자이로) : 원래부터 있던 게이트
      · min_speed : ★필수. 정지 중엔 차가 못 돌아 theta 가 상수로 남는다(실측 -12° 고정).
        이 표본을 섞으면 '변하지 않는 x' 가 회귀를 뭉개서 판정이 무조건 실패한다
        (실측: 속도게이트 없이 n=344 중 227 이 정지 → R²=0.08 로 판정불가가 나왔다).

    [preview lag] theta 는 순시 오차가 아니라 preview 다 — perception 이 lookahead_ratio
      (=0.45) 행, 즉 차량 앞 ~1.5m 지점의 차선 방향을 재기 때문에 hdg_err 보다 앞선다.
      실측 피크가 lag 0.80s 였고 1.8m/s 에서 1.5m/1.8 = 0.83s 로 기하와 일치한다.
      따라서 lag=0 고정 회귀는 상관을 과소평가한다 → 아래에서 lag 를 훑어 피크로 판정한다.
    """
    if "theta_lane_deg" not in ego.columns:
        return
    print(f"{'─' * W}")
    print("  [CAM_HEAD_SIGN 실측] theta_lane vs heading_err")

    th   = ego["theta_lane_deg"].values
    conf = ego["conf_eff"].values if "conf_eff" in ego.columns else np.zeros(len(ego))
    ok   = (conf >= conf_min) & np.isfinite(th) & np.isfinite(hdg_err)
    if "speed" in ego.columns:
        ok &= np.abs(ego["speed"].values) >= min_speed
    else:
        print("    ⚠️ speed 컬럼 없음 — 정지 표본을 못 걸렀다. 결과 신뢰도 낮음.")
    if "yaw_rate" in ego.columns:   # 직진만: 코너에선 theta 가 곡률에 오염된다
        yr = ego["yaw_rate"].values
        ok &= np.isfinite(yr) & (np.abs(np.degrees(yr)) <= gyro_max_dps)

    if ok.sum() < 30:
        print(f"    표본 부족 (n={int(ok.sum())}) — 주행+차선인식+직진 구간이 짧다. 판정 보류.")
        print(f"      → 60초 이상, 차선이 보이는 직선을 포함해 재주행할 것.")
        return
    if np.nanstd(hdg_err[ok]) < 0.5:
        print(f"    heading_err 스윙 부족 (std={np.nanstd(hdg_err[ok]):.2f}°) — 판정 보류.")
        return

    # preview lag 스캔 (ego 20Hz 가정) → |r| 최대 지점으로 판정
    best = None
    for lag in range(0, int(max_lag_s * 20) + 1):
        x = hdg_err[lag:] if lag else hdg_err
        y = th[:len(th) - lag] if lag else th
        s = ok[:len(ok) - lag] if lag else ok
        s = s & np.isfinite(x) & np.isfinite(y)
        if s.sum() < 25:
            continue
        r = np.corrcoef(x[s], y[s])[0, 1]
        if best is None or abs(r) > abs(best[1]):
            best = (lag, r, np.polyfit(x[s], y[s], 1)[0], int(s.sum()))
    if best is None:
        print("    lag 스캔 표본 부족 — 판정 보류.")
        return
    lag, r, slope, n = best
    print(f"    n={n}  피크 lag={lag*0.05:.2f}s  slope={slope:+.2f}  R²={r**2:.2f}"
          f"   (theta 는 preview 라 lag>0 이 정상)")
    if r ** 2 < 0.3:
        print(f"    ⚠️ 상관 약함(R²={r**2:.2f}) — 부호 판정 불가. 차선 인식/캘리브 확인.")
    elif slope < 0:
        print(f"    ✅ slope<0 → 규약 성립 → gps_imu.CAM_HEAD_SIGN = +1.0 (2026-07-16 확정과 일치)")
    else:
        print(f"    ❌ slope>0 → theta 부호 반대 → gps_imu.CAM_HEAD_SIGN = -1.0 으로 뒤집을 것")


def print_stats(label: str, arr: np.ndarray, unit: str = "m"):
    print(f"  {label}")
    print(f"    mean={np.mean(arr):.4f} {unit}  "
          f"std={np.std(arr):.4f}  "
          f"max={np.max(arr):.4f}  "
          f"95th%={np.percentile(arr,95):.4f} {unit}")


# ══════════════════════════════════════════════════════
#  [신규] 카메라 오버레이 계산
# ══════════════════════════════════════════════════════

def attach_camera_signals(ego: pd.DataFrame, lane: pd.DataFrame,
                          imu: pd.DataFrame) -> pd.DataFrame:
    """ego 타임라인에 lane_metrics / imu 요각속도를 시간정렬(merge_asof)로 부착."""
    m = ego.sort_values("timestamp_s").reset_index(drop=True)

    if not lane.empty:
        m = pd.merge_asof(m, lane, on="timestamp_s",
                          direction="nearest", tolerance=0.15)
    for c in ["cte_rear", "theta_lane_deg", "conf_eff", "lane_width", "flags"]:
        if c not in m.columns:
            m[c] = np.nan

    if not imu.empty:
        m = pd.merge_asof(m, imu, on="timestamp_s",
                          direction="nearest", tolerance=0.15)
    if "yaw_rate" not in m.columns:
        m["yaw_rate"] = np.nan

    return m


def compute_lane_markings(ex, ey, heading_deg, cte, width, conf, min_conf=0.5):
    """양쪽 차선이 인식된 샘플(conf≥min_conf & width>0)에서, 차량 자세를 이용해
    좌/우 차선 표식의 지도상(local meter) 좌표를 복원한다.
        차선중심 = 차량위치 + cte·(좌측법선)
        좌차선   = 중심 + (width/2)·좌측법선
        우차선   = 중심 − (width/2)·좌측법선

    [부호 수정] /lane_metrics 의 cte 는 '우측+' 다 — camera_judgment 가 발행 직전에 부호를
    뒤집어 GPS CTE 규약(driving.compute_segment_cte 의 좌측법선 투영 = 차량이 경로 우측이면
    양수)에 맞춘다. 즉 cte>0 = 차량이 차선중심보다 우측 이므로
        cte = (차량 − 중심)·우측법선  →  중심 = 차량 − cte·우측법선 = 차량 + cte·좌측법선
    기존 코드는 '중심 = 차량 − cte·좌측법선'(=cte>0 을 좌측으로 가정)이라 차선 표식을
    차량 기준 반대편에 그리고 있었다. 예: 동쪽 주행 중 cte=+0.5(우측 0.5m)면 중심은
    북쪽 0.5m 인데 남쪽 0.5m 로 찍혔다.
    """
    lx, ly, rx, ry = [], [], [], []
    for i in range(len(ex)):
        c = conf[i]
        w = width[i]
        if not (np.isfinite(c) and np.isfinite(w)) or c < min_conf or w <= 0:
            continue
        h = math.radians(heading_deg[i])
        nx, ny = -math.sin(h), math.cos(h)          # 좌측 법선 (heading +90°)
        ct = cte[i] if np.isfinite(cte[i]) else 0.0
        ccx = ex[i] + ct * nx
        ccy = ey[i] + ct * ny
        hw = w / 2.0
        lx.append(ccx + hw * nx); ly.append(ccy + hw * ny)
        rx.append(ccx - hw * nx); ry.append(ccy - hw * ny)
    return (np.array(lx), np.array(ly)), (np.array(rx), np.array(ry))


# ── BEV 기하 상수 — camera_judgment / perception 파라미터와 반드시 일치 ──
#   (one_launch.py 는 pixel_to_meter_bev=0.006 만 명시, 나머지는 노드 기본값)
BEV_PX2M       = 0.006   # [m/px]  pixel_to_meter_bev
BEV_W          = 640     # [px]
BEV_H          = 480     # [px]
BEV_LOOK_RATIO = 0.45    # lookahead_ratio → y_look = 216 (전방 ~2.1m)
BEV_D0_REAR    = 0.55    # [m] bev_bottom_ahead_rear_m (뒷차축→BEV 최하단)


def compute_camera_lane_curves(t_ego, ex, ey, heading_deg,
                               lane_state: pd.DataFrame,
                               min_conf=0.5, max_dt=0.15, n_pts=12):
    """/lane/state 원시 다항식을 지도(local meter) 좌표의 짧은 곡선들로 투영.

    카메라 프레임 하나가 본 것 = BEV 상의 좌/우 차선 곡선 x_px(y_px).
    각 프레임을 그 시각의 ego 자세에 얹어 지도에 놓는다:
        전방거리  d(y)  = BEV_D0_REAR + (BEV_H − y)·BEV_PX2M      (뒷차축 기준)
        횡오프셋 r(y)  = (x_px(y) − BEV_W/2)·BEV_PX2M            (화면 우측+)
        지도점    p     = ego + d·(cos h, sin h) + r·(sin h, −cos h)
    y 는 BEV_H(차량 바로 앞 0.55m) → y_look(전방 ~2.1m) 범위만 그린다 —
    camera_judgment 가 실제로 계측에 쓰는 구간이다.

    ⚠️ 절대 위치는 여전히 ego 자세에 의존한다(카메라엔 절대좌표가 없다).
       다만 ④⑤ 복원 점과 달리 '카메라가 본 곡선 모양·기울기'가 그대로 보이므로,
       ego 헤딩이 맞다면 이 곡선들은 주행방향과 나란한 연속 리본을 이뤄야 한다.
       리본이 지그재그로 끊기면 → 차선인식 자체가 튀는 것,
       리본은 매끈한데 통째로 기울면 → ego 헤딩(또는 카메라 요 캘리브) 오차.
    반환: (left_curves, right_curves) — 각각 [(xs, ys), ...]
    """
    left_curves, right_curves = [], []
    if lane_state.empty:
        return left_curves, right_curves

    y_look = BEV_H * BEV_LOOK_RATIO
    ys_px  = np.linspace(BEV_H, y_look, n_pts)
    d_fwd  = BEV_D0_REAR + (BEV_H - ys_px) * BEV_PX2M    # 각 행의 전방거리 [m]

    ts = lane_state["timestamp_s"].values
    # 각 lane_state 샘플에 가장 가까운 ego 샘플 매칭 (merge_asof nearest 와 동일 규약)
    idx = np.searchsorted(t_ego, ts).clip(1, len(t_ego) - 1)
    idx = np.where(np.abs(t_ego[idx] - ts) <= np.abs(t_ego[idx - 1] - ts),
                   idx, idx - 1)

    for k, row in lane_state.iterrows():
        if row["conf"] < min_conf:
            continue
        i = idx[k]
        if abs(t_ego[i] - ts[k]) > max_dt:
            continue
        h  = math.radians(heading_deg[i])
        fx, fy = math.cos(h),  math.sin(h)      # 전방 단위벡터
        rx_, ry_ = math.sin(h), -math.cos(h)    # 우측 단위벡터
        for fit, out in ((("aL", "bL", "cL"), left_curves),
                         (("aR", "bR", "cR"), right_curves)):
            a, b, c = row[fit[0]], row[fit[1]], row[fit[2]]
            if abs(a) + abs(b) + abs(c) < 1e-9:   # 미검출 차선
                continue
            x_px  = a * ys_px**2 + b * ys_px + c
            lat_r = (x_px - BEV_W * 0.5) * BEV_PX2M
            out.append((ex[i] + d_fwd * fx + lat_r * rx_,
                        ey[i] + d_fwd * fy + lat_r * ry_))
    return left_curves, right_curves


# ══════════════════════════════════════════════════════
#  시각화 (map_check.py 6-패널 + (A) 오버레이 추가)
# ══════════════════════════════════════════════════════

def plot_comparison(ego, fix, route, cte, hdg_err,
                    output_path: str, title_tag: str, show: bool,
                    overlays: dict = None):

    lat0 = route["lat"].mean()
    lon0 = route["lon"].mean()

    rx, ry = latlon_to_meters(route["lat"].values, route["lon"].values, lat0, lon0)
    ex, ey = latlon_to_meters(ego["lat"].values,   ego["lon"].values,   lat0, lon0)

    fig = plt.figure(figsize=(18, 14))
    fig.suptitle(f"Trajectory Comparison + Camera Fusion  ·  {title_tag}",
                 fontsize=13, fontweight="bold", y=0.98)
    gs = gridspec.GridSpec(3, 3, figure=fig,
                           hspace=0.44, wspace=0.38,
                           left=0.06, right=0.97,
                           top=0.93, bottom=0.06)

    # (A) 궤적 오버레이 — 선마다 확실히 구분되는 단색 팔레트
    #   (CTE 색상 정보는 (B),(E) 패널에 그대로 유지)
    ax = fig.add_subplot(gs[0:2, 0:2])
    #  ① 맵핑 경로 : 검정 파선 (기준선)
    ax.plot(rx, ry, color="#000000", ls="--", lw=2.2, alpha=0.85, zorder=3,
            label="① Mapped route (reference)")
    #  ② 실제 주행 : 진한 파랑 실선
    ax.plot(ex, ey, color="#2563eb", lw=2.6, alpha=0.95, zorder=5,
            label="② Actual drive (ego, GPS+IMU)")

    # ── [신규] 카메라 오버레이 ──────────────────────────
    if overlays:
        left  = overlays.get("lane_left")
        right = overlays.get("lane_right")
        #  ④ 인식 차선 L : 초록 / ⑤ 인식 차선 R : 주황
        if left is not None and len(left[0]):
            ax.scatter(left[0], left[1],  c="#15803d", s=18, alpha=0.9, zorder=6,
                       marker="o", edgecolors="none", label="④ Detected lane L (camera)")
        if right is not None and len(right[0]):
            ax.scatter(right[0], right[1], c="#f59e0b", s=18, alpha=0.9, zorder=6,
                       marker="o", edgecolors="none", label="⑤ Detected lane R (camera)")
        #  ⑦⑧ 원시 차선곡선(/lane/state) : 카메라가 프레임마다 실제 본 모양(전부 실측 데이터)
        for curves, color, lbl in (
                (overlays.get("cam_curves_left"),  "#0d9488", "⑦ Camera-seen lane L (/lane/state)"),
                (overlays.get("cam_curves_right"), "#9333ea", "⑧ Camera-seen lane R (/lane/state)")):
            if not curves:
                continue
            for j, (cx_, cy_) in enumerate(curves):
                ax.plot(cx_, cy_, color=color, lw=0.9, alpha=0.45, zorder=6,
                        label=lbl if j == 0 else None)

    #  시작/끝 : 검정 마커(모양으로 구분)
    ax.scatter(ex[0],  ey[0],  c="#111111", s=95, zorder=8, marker="^",
               edgecolors="white", linewidths=1.2, label="Start")
    ax.scatter(ex[-1], ey[-1], c="#111111", s=95, zorder=8, marker="s",
               edgecolors="white", linewidths=1.2, label="End")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title("(A) Trajectory Overlay  +  Camera Lanes (실측 데이터만)")
    #  범례는 그래프와 겹치지 않도록 (A) 아래 바깥으로 배치
    ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.15),
              ncol=3, framealpha=0.92)
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)

    # (B) CTE 시계열
    ax = fig.add_subplot(gs[0, 2])
    ax.plot(ego["elapsed_s"].values, cte, color="#dc2626", lw=1.2)
    ax.axhline(np.mean(cte), color="#7f1d1d", ls="--", lw=1,
               label=f"mean {np.mean(cte):.3f} m")
    ax.fill_between(ego["elapsed_s"].values, 0, cte, alpha=0.15, color="#dc2626")
    ax.set_xlabel("Elapsed time (s)"); ax.set_ylabel("CTE (m)")
    ax.set_title("(B) Cross-Track Error")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # (C) Heading error
    ax = fig.add_subplot(gs[1, 2])
    ax.plot(ego["elapsed_s"].values, hdg_err, color="#7c3aed", lw=1.2)
    ax.axhline(0, color="gray", ls="--", lw=0.8)
    ax.fill_between(ego["elapsed_s"].values, 0, hdg_err,
                    where=(hdg_err >= 0), alpha=0.15, color="#7c3aed")
    ax.fill_between(ego["elapsed_s"].values, 0, hdg_err,
                    where=(hdg_err <  0), alpha=0.15, color="#db2777")
    ax.set_xlabel("Elapsed time (s)"); ax.set_ylabel("Heading error (°)")
    ax.set_title("(C) Heading Error vs Route"); ax.grid(True, alpha=0.3)

    # (D) 속도 프로파일
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(ego["elapsed_s"].values, np.abs(ego["speed"].values),
            color="#0891b2", lw=1.2, label="ego |speed|")
    if "speed" in route.columns:
        ax.axhline(route["speed"].abs().mean(), color="#0e7490", ls="--", lw=1.2,
                   label=f"route mean {route['speed'].abs().mean():.1f} m/s")
    ax.set_xlabel("Elapsed time (s)"); ax.set_ylabel("Speed (m/s)")
    ax.set_title("(D) Speed Profile")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # (E) CTE 히스토그램
    ax = fig.add_subplot(gs[2, 1])
    ax.hist(cte, bins=40, color="#f97316", edgecolor="white",
            linewidth=0.4, alpha=0.85)
    ax.axvline(np.mean(cte),           color="#7c2d12", ls="--", lw=1.2,
               label=f"mean={np.mean(cte):.3f} m")
    ax.axvline(np.percentile(cte, 95), color="#dc2626", ls=":",  lw=1.2,
               label=f"95th%={np.percentile(cte,95):.3f} m")
    ax.set_xlabel("CTE (m)"); ax.set_ylabel("Count")
    ax.set_title("(E) CTE Distribution")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # (F) GPS fix vs ego 잔차
    ax   = fig.add_subplot(gs[2, 2])
    R    = 6_371_000.0
    midx = np.searchsorted(ego["timestamp_s"].values,
                           fix["timestamp_s"].values).clip(0, len(ego) - 1)
    dlat = (fix["lat"].values - ego["lat"].values[midx]) * np.radians(1) * R
    dlon = (fix["lon"].values - ego["lon"].values[midx]) * np.radians(1) * R \
           * np.cos(np.radians(lat0))
    rms  = np.sqrt(np.mean(dlat**2 + dlon**2))
    sc   = ax.scatter(dlon, dlat, c=fix["elapsed_s"].values,
                      cmap="viridis", s=8, alpha=0.6)
    plt.colorbar(sc, ax=ax, label="Elapsed time (s)", shrink=0.7)
    ax.axhline(0, color="gray", lw=0.6, ls="--")
    ax.axvline(0, color="gray", lw=0.6, ls="--")
    ax.set_xlabel("ΔEast (m)  [fix − ego]")
    ax.set_ylabel("ΔNorth (m)  [fix − ego]")
    ax.set_title(f"(F) GPS fix vs ego_state  RMS={rms:.4f} m")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)

    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"\n  그림 저장 → {output_path}")
    if show:
        print("  (창을 닫으면 종료됩니다)")
        plt.show()
    plt.close()


# ══════════════════════════════════════════════════════
#  CSV 저장 (map_check.py 와 동일)
# ══════════════════════════════════════════════════════

def save_stats_csv(ego, cte, hdg_err, output_path: str):
    pd.DataFrame({
        "timestamp_s":       ego["timestamp_s"].values,
        "elapsed_s":         ego["elapsed_s"].values,
        "lat":               ego["lat"].values,
        "lon":               ego["lon"].values,
        "heading_deg":       ego["heading"].values,
        "speed_mps":         ego["speed"].values,
        "cross_track_err_m": cte,
        "heading_err_deg":   hdg_err,
    }).to_csv(output_path, index=False)
    print(f"  통계 CSV 저장 → {output_path}")


# ══════════════════════════════════════════════════════
#  main
# ══════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="실제 차량 궤적 vs 사전 매핑 경로 비교 + 카메라 융합 오버레이"
    )
    parser.add_argument("--route-dir", default=str(DEFAULT_ROUTE_DIR),
                        help=f"route CSV 폴더 (default: {DEFAULT_ROUTE_DIR})")
    parser.add_argument("--bag-dir",   default=str(DEFAULT_BAG_DIR),
                        help=f"ROS2 bag 루트 폴더 (default: {DEFAULT_BAG_DIR})")
    parser.add_argument("--out-dir",   default=None,
                        help="출력 디렉터리 (default: 선택한 bag 폴더 내부)")
    parser.add_argument("--no-show",   action="store_true",
                        help="matplotlib 창 표시 없이 이미지만 저장")
    args = parser.parse_args()

    route_dir = Path(args.route_dir).expanduser()
    bag_dir   = Path(args.bag_dir).expanduser()

    print(f"\n{'═' * W}")
    print(f"  Trajectory + Camera Fusion Tool  (ROS2 Humble / Ubuntu 22.04)")
    print(f"{'═' * W}")

    ego_path, fix_path, route_path, lane_path, imu_path, state_path = resolve_paths(bag_dir, route_dir)

    out_dir = Path(args.out_dir).expanduser() if args.out_dir else ego_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    # 데이터 로드
    print("데이터 로드 중...")
    ego   = load_ego_state(ego_path)
    fix   = load_fix(fix_path)
    route = load_route(route_path)
    lane  = load_lane_metrics(lane_path)
    imu   = load_imu_yaw(imu_path)
    lane_state = load_lane_state(state_path)
    print(f"  ego_state    : {len(ego):,} rows  ({ego['elapsed_s'].iloc[-1]:.1f} s)")
    print(f"  fix          : {len(fix):,} rows")
    print(f"  route        : {len(route):,} waypoints")
    print(f"  lane_metrics : {len(lane):,} rows" + ("" if not lane.empty else "  (융합 오버레이 생략)"))
    print(f"  imu_data     : {len(imu):,} rows")
    print(f"  lane_state   : {len(lane_state):,} rows" + ("" if not lane_state.empty else "  (원시 차선곡선 생략)"))

    # ego 에 카메라/IMU 신호 부착
    ego = attach_camera_signals(ego, lane, imu)

    # 지표 계산 (map_check.py 와 동일)
    print("\n지표 계산 중...")
    cte     = compute_cross_track_error(
                  ego["lat"].values, ego["lon"].values,
                  route["lat"].values, route["lon"].values)
    hdg_err = compute_heading_error(
                  ego["heading"].values, route,
                  ego["lat"].values, ego["lon"].values)

    R      = 6_371_000.0
    lat0   = route["lat"].mean()
    lon0   = route["lon"].mean()
    midx   = np.searchsorted(ego["timestamp_s"].values,
                             fix["timestamp_s"].values).clip(0, len(ego) - 1)
    dlat_m = (fix["lat"].values - ego["lat"].values[midx]) * np.radians(1) * R
    dlon_m = (fix["lon"].values - ego["lon"].values[midx]) * np.radians(1) * R \
             * np.cos(np.radians(lat0))
    dist_m = np.sqrt(dlat_m**2 + dlon_m**2)

    # ── route-주행 불일치 가드 ──────────────────────────────
    #   평균 CTE 가 수십 m 면 궤적 문제가 아니라 '다른 날 route 를 고른 것'이다
    #   (실측: 07-15 주행에 07-14 route 를 골라 CTE 66m/heading 150° 가 나왔다).
    #   주행궤적 중심과 가장 가까운 route 후보를 알려줘 재선택을 유도한다.
    if np.mean(cte) > 10.0:
        print(f"\n  ⚠️  평균 CTE {np.mean(cte):.1f} m — route 와 주행 위치가 크게 어긋납니다.")
        print(f"      다른 날짜의 route 를 선택했을 가능성이 높습니다. 후보(주행궤적 중심과의 거리):")
        ego_lat_c, ego_lon_c = ego["lat"].mean(), ego["lon"].mean()
        cands = []
        for p in sorted(route_dir.glob("route_*.csv")):
            try:
                r_df = load_route(p)
                dx, dy = latlon_to_meters(r_df["lat"].mean(), r_df["lon"].mean(),
                                          ego_lat_c, ego_lon_c)
                cands.append((math.hypot(dx, dy), p.name))
            except Exception:
                continue
        for d, name in sorted(cands)[:3]:
            mark = "▶" if d < 10 else " "
            print(f"      {mark} {name:<45} {d:8.1f} m")

    print(f"\n{'─' * W}")
    print("  분석 결과")
    print(f"{'─' * W}")
    print_stats("Cross-Track Error  (ego vs route)",   cte,             "m")
    print_stats("Heading Error      (ego vs route)",   np.abs(hdg_err), "°")
    print_stats("GPS fix vs ego_state  position diff", dist_m,          "m")

    report_cam_head_sign(ego, hdg_err)

    # ── [신규] 카메라 오버레이 계산 ──────────────────────
    overlays = None
    if not lane.empty:
        ex, ey = latlon_to_meters(ego["lat"].values, ego["lon"].values, lat0, lon0)
        conf   = ego["conf_eff"].values
        width  = ego["lane_width"].values
        cte_c  = ego["cte_rear"].values
        n_det  = int(np.sum((conf >= 0.5) & (width > 0)))

        lane_left, lane_right = compute_lane_markings(
            ex, ey, ego["heading"].values, cte_c, width, conf, min_conf=0.5)

        overlays = {"lane_left": lane_left, "lane_right": lane_right}
        print(f"{'─' * W}")
        print(f"  카메라 양쪽차선 인식 구간: {n_det} / {len(ego)} 샘플")

    # 원시 /lane/state 곡선 투영 (lane_metrics 없이도 가능)
    if not lane_state.empty:
        ex, ey = latlon_to_meters(ego["lat"].values, ego["lon"].values, lat0, lon0)
        cl, cr = compute_camera_lane_curves(
            ego["timestamp_s"].values, ex, ey, ego["heading"].values, lane_state)
        if overlays is None:
            overlays = {}
        overlays["cam_curves_left"]  = cl
        overlays["cam_curves_right"] = cr
        print(f"  원시 차선곡선(/lane/state) 투영: L {len(cl)} / R {len(cr)} 프레임")
    print(f"{'─' * W}")

    tag         = ego_path.parent.name.replace("extracted_rosbag2_", "")
    plot_output = str(out_dir / f"fusion_check_{tag}.png")
    csv_output  = str(out_dir / f"trajectory_stats_{tag}.csv")
    title_tag   = f"{route_path.stem}  ←→  {ego_path.parent.name}"

    print("\n시각화 생성 중...")
    plot_comparison(ego, fix, route, cte, hdg_err,
                    plot_output, title_tag, show=(not args.no_show),
                    overlays=overlays)
    save_stats_csv(ego, cte, hdg_err, csv_output)
    print("\n완료!")


if __name__ == "__main__":
    main()
