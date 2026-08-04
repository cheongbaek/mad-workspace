#!/usr/bin/env python3
"""
trust_sweep.py — 카메라 헤딩 트러스트 sweep (9:0.5:0.5 의 카메라 몫이 좋은 값인지 검증)

원리
====
카메라 권한(트러스트)이 실제로 의미 있는 구간은 GPS 두절(DR)이다. "주행 중 임의 지점에서
GPS 가 window_s 초 끊긴다"고 가정하고, 그 순간 실제 자세에서 출발해 IMU 자이로 적분 +
카메라 헤딩보정(theta_lane 을 0 으로 당김)만으로 경로를 재구성한다. 재구성이 실주행에서
멀어진 거리(drift)가 작을수록 좋다.

카메라 트러스트를 0 → 1.5 로 훑으며 drift 를 재면, drift 를 최소화하는 트러스트가
'좋은 값'이다.
  · trust = 0.0  → 순수 IMU(카메라 미사용) 기준선
  · trust = 0.5  → 현행 정상주행 값(CAM_HEADING_TRUST)
  · trust = 1.0  → 현행 DR 값(CAM_HEADING_TRUST_DR)  ← DR 재구성이라 이게 현행 규약
해석
  · 최소가 1.0 근처   → 현행 DR 트러스트 확정
  · 0.0(카메라 끔)이 최소와 비슷/더 작음 → 카메라가 DR 에서 도움 안 됨 → 트러스트 낮출 근거
  · 곡선이 평평       → 트러스트에 둔감(0.5~1.0 어디든 무방)

gps_imu.py 규약 그대로: gain 0.04 · clamp ±0.5°/step · 속도게이트 0.3 m/s · 카메라 마운트
바이어스(theta 평균) 제거. 데이터 로딩은 map_check_fusion.py 함수를 재사용한다.
읽기 전용 오프라인 분석 — 차량 코드/토픽 무관, 로스백만 있으면 매번 돌려 추세를 쌓으면 된다.

사용
====
    python3 trust_sweep.py                              # 대화형 bag 선택
    python3 trust_sweep.py --bag <extracted_폴더경로>   # 직접 지정
    python3 trust_sweep.py --window 5 --stride 2        # dropout 지속/시작간격[s]
    python3 trust_sweep.py --trusts 0,0.25,0.5,0.75,1.0,1.5
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

# ── map_check_fusion 재사용 (같은 폴더) ──────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent))
import map_check_fusion as mcf   # noqa: E402

# gps_imu.py 규약 상수 (map_check_fusion 과 동일 값)
CAM_HEAD_GAIN      = mcf.CAM_HEAD_GAIN        # 0.04
CAM_HEAD_CLAMP_DEG = mcf.CAM_HEAD_CLAMP_DEG   # 0.5
CAM_MIN_SPEED      = mcf.CAM_MIN_SPEED        # 0.3
W = 64


def _box(title):
    print(f"\n{'═'*W}\n  {title}\n{'═'*W}")


def compute_theta_bias(speed, conf, theta, min_speed, conf_thr):
    moving = (np.abs(speed) >= min_speed) & (conf >= conf_thr) & np.isfinite(theta)
    return float(np.nanmean(theta[moving])) if moving.any() else 0.0, int(moving.sum())


def dropout_drift(t, speed, yaw_rate, theta, conf, ex, ey, heading_deg,
                  trust, theta_bias, window_s, stride_s, conf_thr):
    """트러스트 하나에 대해 dropout window 재구성 → 각 window 최대 drift 배열."""
    n = len(t)
    clamp = math.radians(CAM_HEAD_CLAMP_DEG)
    # dropout 시작점: 움직이는 중 + 뒤로 window_s 여유 + stride 간격 솎기
    picked, last_t = [], -1e9
    for i in range(n):
        if abs(speed[i]) >= CAM_MIN_SPEED and (t[-1]-t[i]) >= window_s and (t[i]-last_t) >= stride_s:
            picked.append(i); last_t = t[i]
    drifts = []
    for i0 in picked:
        h = math.radians(heading_deg[i0]); x, y = ex[i0], ey[i0]; dmax = 0.0; i = i0
        while i + 1 < n and (t[i+1] - t[i0]) <= window_s:
            i += 1
            dt = min(max(t[i] - t[i-1], 1e-3), 0.5)
            w  = yaw_rate[i] if np.isfinite(yaw_rate[i]) else 0.0
            v  = speed[i]    if np.isfinite(speed[i])    else 0.0
            h += w * dt                                    # IMU 자이로 적분(항상)
            if conf[i] >= conf_thr and abs(v) >= CAM_MIN_SPEED and np.isfinite(theta[i]):
                corr = CAM_HEAD_GAIN * trust * math.radians(theta[i] - theta_bias)
                h += max(-clamp, min(clamp, corr))         # 카메라 헤딩보정
            x += v * dt * math.cos(h); y += v * dt * math.sin(h)
            dmax = max(dmax, math.hypot(x - ex[i], y - ey[i]))
        drifts.append(dmax)
    return np.array(drifts) if drifts else np.array([])


def main():
    ap = argparse.ArgumentParser(description="카메라 헤딩 트러스트 sweep (DR drift 최소화 값 탐색)")
    ap.add_argument("--bag", default=None, help="extracted_* 폴더 경로(생략 시 대화형 선택)")
    ap.add_argument("--bag-dir", default=str(mcf.DEFAULT_BAG_DIR))
    ap.add_argument("--window", type=float, default=5.0, help="GPS 끊김 지속[s]")
    ap.add_argument("--stride", type=float, default=2.0, help="dropout 시작 간격[s]")
    ap.add_argument("--conf-thr", type=float, default=0.5, help="양쪽차선 conf 하한")
    ap.add_argument("--trusts", default="0,0.25,0.5,0.75,1.0,1.5",
                    help="쉼표구분 트러스트 목록")
    ap.add_argument("--no-debias", action="store_true", help="마운트 바이어스 제거 끄기")
    args = ap.parse_args()

    bag = Path(args.bag).expanduser() if args.bag else mcf.select_bag_folder(Path(args.bag_dir).expanduser())
    ego_p, lane_p, imu_p = bag/"ego_state.jsonl", bag/"lane_metrics.jsonl", bag/"imu_data.jsonl"
    for p in (ego_p, lane_p, imu_p):
        if not p.exists():
            sys.exit(f"오류: {p} 없음 — ego_state/lane_metrics/imu_data 가 있는 bag 이어야 함.")

    ego  = mcf.load_ego_state(ego_p)
    lane = mcf.load_lane_metrics(lane_p)
    imu  = mcf.load_imu_yaw(imu_p)
    if lane.empty:
        sys.exit("오류: lane_metrics 비어있음 — 카메라 OFF 로 주행했을 수 있음.")
    ego = mcf.attach_camera_signals(ego, lane, imu)

    lat0, lon0 = ego["lat"].iloc[0], ego["lon"].iloc[0]
    ex, ey = mcf.latlon_to_meters(ego["lat"].values, ego["lon"].values, lat0, lon0)
    t       = ego["timestamp_s"].values
    speed   = ego["speed"].values
    yaw     = ego["yaw_rate"].values
    theta   = ego["theta_lane_deg"].values
    conf    = ego["conf_eff"].values
    heading = ego["heading"].values

    theta_bias, n_move = (0.0, 0) if args.no_debias else compute_theta_bias(
        speed, conf, theta, CAM_MIN_SPEED, args.conf_thr)

    _box(f"카메라 트러스트 sweep  ·  {bag.name}")
    dur = t[-1] - t[0]
    n_valid = int(np.sum((conf >= args.conf_thr) & np.isfinite(theta) & (np.abs(speed) >= CAM_MIN_SPEED)))
    print(f"  주행 {dur:.0f}s / ego {len(ego)}행 | 유효표본(직진·차선·이동) {n_valid}행")
    print(f"  카메라 마운트 바이어스: {theta_bias:+.2f}° (n={n_move})"
          + ("  [제거]" if not args.no_debias else "  [미제거]"))
    print(f"  window={args.window:.0f}s stride={args.stride:.0f}s  gain={CAM_HEAD_GAIN} clamp=±{CAM_HEAD_CLAMP_DEG}°/step\n")

    trusts = [float(x) for x in args.trusts.split(",")]
    rows = []
    for tr in trusts:
        d = dropout_drift(t, speed, yaw, theta, conf, ex, ey, heading,
                          tr, theta_bias, args.window, args.stride, args.conf_thr)
        if d.size == 0:
            rows.append((tr, None)); continue
        rows.append((tr, (float(np.median(d)), float(np.percentile(d, 95)),
                          float(np.max(d)), len(d))))

    if all(r[1] is None for r in rows):
        sys.exit("  ⚠️ dropout window 표본 부족 — 더 길게/빠르게 주행한 로스백 필요.")

    # 최소 median 트러스트
    valid = [(tr, m) for tr, m in rows if m is not None]
    best_tr, best = min(valid, key=lambda r: r[1][0])
    pure = dict(valid).get(0.0)

    print(f"  {'trust':>6} │ {'median':>8} {'95%tile':>8} {'worst':>8} │ n   비고")
    print(f"  {'─'*6}─┼─{'─'*8}─{'─'*8}─{'─'*8}─┼────────")
    for tr, m in rows:
        if m is None:
            print(f"  {tr:>6.2f} │ {'표본부족':>8}"); continue
        med, p95, wrst, nn = m
        tag = []
        if tr == 0.0:   tag.append("순수IMU")
        if tr == 0.5:   tag.append("정상주행")
        if tr == 1.0:   tag.append("DR현행")
        if tr == best_tr: tag.append("◀ 최소")
        print(f"  {tr:>6.2f} │ {med:>7.2f}m {p95:>7.2f}m {wrst:>7.2f}m │ {nn:<3} {' '.join(tag)}")

    _box("판정")
    print(f"  drift 최소 트러스트 = {best_tr:.2f}  (median {best[0]:.2f}m)")
    if pure is not None:
        imp = pure[0] - best[0]
        rel = 100.0 * imp / pure[0] if pure[0] > 1e-6 else 0.0
        print(f"  순수IMU(trust0) median {pure[0]:.2f}m 대비 개선 {imp:+.2f}m ({rel:+.0f}%)")
        if best_tr == 0.0 or rel < 5.0:
            print("  → 카메라 기여 미미/악화. 현행 트러스트가 과할 수 있음(신호 유효성부터 재확인).")
        elif abs(best_tr - 1.0) < 1e-6:
            print("  → DR 현행값(1.0) 이 최적. 규약 유지.")
        else:
            print(f"  → DR 트러스트를 {best_tr:.2f} 로 조정 검토(정상주행은 그 절반 비례).")
    print(f"\n  ⚠️ 표본이 적으면(n<10) 참고치. 신호 유효성은 map_check_fusion 의 [CAM_HEAD_SIGN 실측] 병행.")


if __name__ == "__main__":
    main()
