#!/usr/bin/env python3
"""
카메라 ↔ GPS 조향 일치 증명 (발표용 그림 생성기)
================================================
"GPS와 카메라를 융합했을 때 두 센서가 바라보는 조향(heading)이 같다"를
정량적으로 증명하는 단일 그림을 만든다.

증명의 논리
-----------
서로 정보를 공유하지 않는 두 개의 독립 센서 체인이,
같은 물리량 —— "도로 방향 대비 차량이 얼마나 틀어져 있는가(ψ, CCW+, deg)" —— 을
각각 독립적으로 측정한다.

  · 카메라(영상 파이프라인)  : ψ_cam = theta_lane_deg           (/lane_metrics[2])
        = BEV 차선 접선방향 − 차량 heading  (CCW+)
  · GPS+맵(GNSS+IMU+사전맵) : ψ_gps = route_tangent − ego_heading
        = −heading_err  (ego heading 은 GPS 지배 융합값, route 는 사전 매핑 접선)

두 신호가 (1) 부호가 같고 (2) 기울기 ≈ +1 이고 (3) 상관이 높으면,
두 센서가 "같은 조향"을 본다는 뜻이다. 카메라 보정은 맵을 전혀 쓰지 않으므로
(gps_imu 의 theta 보정은 map-free) 이 일치는 우연이 아니라 물리적 정합이다.

게이트(gps_imu._apply_cam_heading_correction 과 동일):
  conf≥0.5, |speed|≥0.3m/s(정지 배제), |gyro|≤10°/s(코너 곡률오염 배제)
preview lag: theta 는 전방(lookahead ~1.5m)을 보는 preview 라 heading_err 보다
  앞선다 → lag 를 훑어 |r| 최대 지점에서 판정.

사용:
  python3 prove_cam_gps_steering.py \
      --bag  ros2bag/extracted_rosbag2_2026_07_16-22_02_40 \
      --route gps_data/route_20260716_215853_remodeled.csv \
      --out  results/cam_gps_steering_proof.png
기본값은 "가장 최근" bag/route 자동 선택.
"""
import argparse, json, math, sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

BASE = Path(__file__).resolve().parent.parent


# ── 로더 ────────────────────────────────────────────────
def load_ego(p):
    rec = []
    for line in open(p):
        line = line.strip()
        if not line:
            continue
        o = json.loads(line); d = o["data"]["data"]
        rec.append(dict(t=o["timestamp"] * 1e-9, lat=d[0], lon=d[1],
                        heading=d[4], speed=d[5]))
    return pd.DataFrame(rec).sort_values("t").reset_index(drop=True)


def load_lane(p):
    rec = []
    for line in open(p):
        line = line.strip()
        if not line:
            continue
        o = json.loads(line); d = o["data"]["data"]
        if len(d) < 7:
            continue
        rec.append(dict(t=o.get("t_sec", o["timestamp"] * 1e-9),
                        theta=d[2], conf=d[4]))
    return pd.DataFrame(rec).sort_values("t").reset_index(drop=True)


def load_imu(p):
    rec = []
    for line in open(p):
        line = line.strip()
        if not line:
            continue
        o = json.loads(line); d = o["data"]
        rec.append(dict(t=o.get("t_sec", o["timestamp"] * 1e-9),
                        yaw=d.get("angular_velocity", {}).get("z", 0.0)))
    return pd.DataFrame(rec).sort_values("t").reset_index(drop=True)


def to_m(lat, lon, lat0):
    R = 6_371_000.0
    return np.column_stack([lon * np.radians(1) * R * np.cos(np.radians(lat0)),
                            lat * np.radians(1) * R])


def latest(paths):
    """기록시각(폴더/파일명 끝의 YYYY_MM_DD-HH_MM_SS) 기준 최신.
    ★ mtime 으로 고르면 안 된다 — 옛 bag 을 오늘 재추출하면 mtime 이 최신이 돼
       '가장 최근 주행'이 아닌 걸 집는다(워크스페이스 유령폴더 이슈와 동일 함정)."""
    import re
    def key(p):
        m = re.search(r"(\d{4}_?\d{2}_?\d{2}[-_]\d{2}_?\d{2}_?\d{2})", p.name)
        return m.group(1) if m else ""   # 타임스탬프 없는 폴더(noise_test 등)는 최하위
    cand = [p for p in paths if key(p)]  # 타임스탬프 있는 것만 후보
    return sorted(cand, key=key)[-1] if cand else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default=None, help="extracted_* 폴더")
    ap.add_argument("--route", default=None, help="route CSV")
    ap.add_argument("--out", default=str(BASE / "results" / "cam_gps_steering_proof.png"))
    ap.add_argument("--max-lag-s", type=float, default=1.2)
    args = ap.parse_args()

    # 가장 최근 bag/route 자동 선택
    bagdir = Path(args.bag) if args.bag else latest(
        [p for p in (BASE / "ros2bag").iterdir()
         if p.is_dir() and p.name.startswith("extracted_")
         and (p / "lane_metrics.jsonl").exists()])
    if bagdir is None:
        sys.exit("bag 을 찾을 수 없습니다.")
    if not bagdir.is_absolute():
        bagdir = BASE / bagdir
    route_p = Path(args.route) if args.route else latest(
        list((BASE / "gps_data").glob("route_*_remodeled.csv")))
    if not route_p.is_absolute():
        route_p = BASE / route_p

    print(f"bag   : {bagdir.name}")
    print(f"route : {route_p.name}")

    ego = load_ego(bagdir / "ego_state.jsonl")
    lane = load_lane(bagdir / "lane_metrics.jsonl")
    imu = load_imu(bagdir / "imu_data.jsonl")
    route = pd.read_csv(route_p).rename(columns={"latitude": "lat", "longitude": "lon"})

    lat0 = route["lat"].mean()
    rm = to_m(route["lat"].values, route["lon"].values, lat0)
    em = to_m(ego["lat"].values, ego["lon"].values, lat0)
    dist, idx = KDTree(rm).query(em)
    # ψ_gps = route_tangent − heading  (= −heading_err), CCW+, [-180,180)
    hdg_err = (ego["heading"].values - route["heading"].values[idx] + 180) % 360 - 180
    ego["psi_gps"] = -hdg_err
    ego["cte"] = dist

    m = pd.merge_asof(ego.rename(columns={"t": "timestamp_s"}),
                      lane.rename(columns={"t": "timestamp_s"}),
                      on="timestamp_s", direction="nearest", tolerance=0.15)
    m = pd.merge_asof(m, imu.rename(columns={"t": "timestamp_s"}),
                      on="timestamp_s", direction="nearest", tolerance=0.15)

    t = m["timestamp_s"].values - m["timestamp_s"].values[0]
    psi_cam = m["theta"].values          # 카메라
    psi_gps = m["psi_gps"].values        # GPS+맵
    conf = m["conf"].values
    spd = m["speed"].values
    yr = m["yaw"].values

    ok = (conf >= 0.5) & np.isfinite(psi_cam) & np.isfinite(psi_gps) & (np.abs(spd) >= 0.3)
    ok &= np.isfinite(yr) & (np.abs(np.degrees(yr)) <= 10.0)

    # preview lag 스캔 (ego 20Hz) → |r| 최대
    best = None
    for lag in range(0, int(args.max_lag_s * 20) + 1):
        x = psi_gps[lag:] if lag else psi_gps           # GPS(뒤로 shift → 과거)
        y = psi_cam[:len(psi_cam) - lag] if lag else psi_cam  # cam(앞)
        s = (ok[:len(ok) - lag] if lag else ok) & np.isfinite(x) & np.isfinite(y)
        if s.sum() < 25:
            continue
        r = np.corrcoef(x[s], y[s])[0, 1]
        if best is None or abs(r) > abs(best[0]):
            best = (r, lag, s, x, y)
    r, lag, s, xa, ya = best
    xg, yc = xa[s], ya[s]
    # 카메라 샘플 시각(색상용) — lag 적용 시 cam 은 앞쪽 [:len-lag] 구간
    t_cam = (t[:len(t) - lag] if lag else t)[s]
    slope, intercept = np.polyfit(xg, yc, 1)
    n = len(xg)
    diff = yc - xg
    rms = np.sqrt(np.mean(diff ** 2))

    print(f"\n★ 결과: n={n}  lag={lag*0.05:.2f}s  slope={slope:+.3f}  "
          f"r={r:+.3f}  R²={r**2:.3f}  RMS차={rms:.2f}°  CTE평균={dist.mean():.3f}m")

    # ── 그림: 한 장, 두 선이 겹친다 ──────────────────────
    C_CAM = "#15803d"   # 카메라 초록
    C_GPS = "#2563eb"   # GPS 파랑
    bias = float(np.mean(diff))   # 카메라 마운트 요 상수 편차 (모양 비교 위해 제거)

    # 표시용: 주행중 + 차선 신뢰 구간 전체 (연속선). 상관 수치는 직진구간(위 분석) 기준.
    disp = (conf >= 0.5) & np.isfinite(psi_cam) & np.isfinite(psi_gps) & (np.abs(spd) >= 0.3)

    fig, ax = plt.subplots(figsize=(11, 5.2))
    ax.plot(t[disp], psi_gps[disp], color=C_GPS, lw=2.6, label="GPS + Map")
    ax.plot(t[disp], psi_cam[disp] - bias, color=C_CAM, lw=2.6, alpha=0.9,
            label=f"Camera  ({abs(bias):.1f}° mount bias removed)")
    ax.axhline(0, color="gray", lw=0.8, ls="--")
    ax.set_xlabel("time (s)", fontsize=12)
    ax.set_ylabel("steering  (heading offset from road, deg)", fontsize=12)
    ax.set_title("Camera and GPS see the same steering",
                 fontsize=16, fontweight="bold", pad=12)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12, loc="upper right", framealpha=0.92)
    ax.text(0.015, 0.05,
            f"correlation  r = {r:+.2f}      slope = {slope:+.2f}      n = {n}",
            transform=ax.transAxes, ha="left", va="bottom", fontsize=11.5,
            bbox=dict(boxstyle="round,pad=0.45", fc="white", ec="#111", alpha=0.92))
    fig.tight_layout()

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=170, bbox_inches="tight")
    print(f"\n그림 저장 → {args.out}")


if __name__ == "__main__":
    main()
