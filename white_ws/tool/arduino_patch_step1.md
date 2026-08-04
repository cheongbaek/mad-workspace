# 아두이노 1단계 패치 — d_val 절삭 수정 + PWM 실측

현재 스케치(`PS2 우선권 + 통신 워치독 페일세이프판`)에 **두 곳만** 손댑니다.
PWM 클램프(205)와 조향은 **이번엔 건드리지 않습니다** — 변수를 하나씩 확인하기 위해서입니다.

---

## 패치 1 — `d_val` 절삭 제거

### 왜

```c
d_val = (signed long)((float)d_val_raw * scale);   // C 캐스팅 = 항상 내림
```

`loop()`가 빨라 `elapsed_us`는 보통 10000~11000us → `scale ≈ 0.91~1.00`.
곱한 결과가 소수인데 **정수 캐스팅이 소수부를 버립니다**. 매 10ms 평균 0.5틱씩
계통적으로 잃고, `d_val`이 작은 저속일수록 손실 비율이 큽니다.

| 속도 | 실제 틱 | 0.5틱 손실 비율 |
|---|---|---|
| 0.5 m/s | 1.77 | **28%** |
| 1.0 m/s | 3.54 | 14% |
| 2.0 m/s | 7.07 | 7% |

로스백 3개 14개 창으로 검증했더니, 이 모델로 속도 의존성이 설명되고 남는 건
**속도와 무관한 일정 배율 1.07**(상관 r=+0.017)뿐이었습니다.

### 어디를

`Velo_PID_Control()` 안, 전역 변수 선언부에 한 줄 추가하고 두 줄을 교체합니다.

**① 전역 변수 추가** — `int PWM = 0;` 아래

```c
int           velo_val = 0;
float         Error = 0, d_Error = 0, old_Error = 0, Sum_Error = 0;
int           PWM = 0;

float         d_resid = 0.0f;      // ★ 추가: 절삭으로 버려지는 소수부를 다음 주기로 이월
```

**② `Velo_PID_Control()` 안 교체**

```c
  float scale = (float)VELO_PID_PERIOD_US / (float)elapsed_us;

  // ── 기존 (절삭) ──────────────────────────────────
  // d_val = (signed long)((float)d_val_raw * scale);

  // ── 수정 (반올림 + 잔차 이월) ────────────────────
  float d_exact = (float)d_val_raw * scale + d_resid;
  d_val   = (signed long)lroundf(d_exact);
  d_resid = d_exact - (float)d_val;      // 버린 소수를 다음 주기에 더해준다
```

**③ 정지 시 잔차 초기화** — 같은 함수의 `velo_val == 0` 분기

```c
  if (velo_val == 0) {
    PWM = 0;
    Sum_Error = 0;
    old_Error = 0;
    d_resid = 0.0f;        // ★ 추가: 정지 중 잔차가 남아 재출발 시 튀는 것 방지
  } else {
```

> `math.h`는 이미 `#include` 되어 있어 `lroundf()`를 바로 쓸 수 있습니다.
>
> **잔차 이월(`d_resid`)까지 넣는 이유**: 반올림만 하면 매 주기 오차는 ±0.5로 줄지만
> 여전히 랜덤하게 남습니다. 잔차를 다음 주기로 넘기면 **장기 적산 오차가 0으로 수렴**해
> DR(추측항법) 거리 정확도가 확실히 좋아집니다.

---

## 패치 2 — PWM 실측 (한 줄)

### 왜

지금까지 "2.2 m/s에 PWM 252"는 **한 점을 지나는 외삽**이었습니다. 물리 모델은

```
v = A·duty − B        A = k·V_bat,  B = k·I·R (부하 오프셋)
```

인데 측정점이 하나뿐이라 **A, B를 확정할 수 없습니다**. B 가정에 따라 220~252로 갈립니다.
서로 다른 속도에서 `(duty, v)` 쌍을 두 개 이상 얻으면 회귀로 확정됩니다.

### 어디를

`serial_tx_poll()` 안, 기존 `E` 프레임 출력 **다음에** 추가:

```c
  Serial.print("E,");
#if DIAG_DT
  Serial.print(d_val);
  Serial.print(",");
  Serial.println(last_dt_us);
#else
  Serial.println(d_val);
#endif

  // ★ 추가: 속도PID 의 실제 PWM 출력. E 프레임과 독립된 별도 프레임이라
  //   기존 파싱 규약(motor.py)에 영향이 없다.
  Serial.print("P,");
  Serial.println(PWM);
```

> **ROS 쪽은 이미 준비 완료** — `motor.py`가 `P,pwm`을 파싱해 `/motor_pwm`으로 발행하고,
> `rec.py` KEEP_TOPICS(16개)에 포함돼 로스백에 자동 기록됩니다. 펌웨어가 `P`를 안 보내면
> 그 분기는 실행되지 않으므로 하위호환입니다.
>
> 통신량: 100Hz × 약 8바이트 = 800 B/s 추가. 115200 baud(약 11.5 kB/s)에 여유 충분합니다.

---

## 실측 절차

### A. 패치 1 검증 — 엔코더 스케일이 속도와 무관해졌는가

평소처럼 주행하고 로스백을 받습니다. 확인 지표:

```
10초 창별 GPS/엔코더 비율이 '속도와 무관한 일정값'이 되어야 한다
```

| | 수정 전 (실측) | 수정 후 (기대) |
|---|---|---|
| 0.5 m/s | 1.43 | ~1.07 |
| 1.2 m/s | 1.23 | ~1.07 |
| 2.1 m/s | 1.10 | ~1.07 |
| **속도 상관** | **r = −0.83** | **r ≈ 0** |

일정값으로 수렴하면 그 값을 `WHEEL_CIRCUMFERENCE`에 곱해 스케일을 확정합니다
(`driving.py`, `motor.py` **양쪽** 같은 값이어야 합니다).

### B. 패치 2 — PWM–속도 관계 확정

**여러 속도에서 정상상태 데이터**가 필요합니다. 한 번의 주행 중 재빌드 없이 바꿀 수 있습니다:

```bash
ros2 param set /driving_node max_speed_ms 1.0    # 10초 이상 유지
ros2 param set /driving_node max_speed_ms 1.4
ros2 param set /driving_node max_speed_ms 1.8
ros2 param set /driving_node max_speed_ms 2.2
```

가능하면 **평지 구간에서** 각 속도를 10초 이상 유지하세요(경사가 섞이면 부하가 달라져
회귀가 흐려집니다). 로스백을 주시면 `(duty, v)` 회귀로 A, B를 뽑아
**"2.2 m/s에 실제로 PWM 얼마"**를 추정이 아닌 실측으로 확정해 드립니다.

**PWM이 205에 붙어 있는 구간**은 포화라 회귀에서 제외해야 합니다 — 그래서 낮은 속도부터
데이터를 받는 게 중요합니다.

---

## 이번에 하지 않는 것

| 항목 | 이유 |
|---|---|
| PWM 클램프 205 → 255 | 실측으로 필요 PWM을 확정한 뒤에 결정 |
| 조향 데드밴드/적분 리셋 | 한 번에 한 변수만 — 속도계 정확도부터 |
| `WHEEL_CIRCUMFERENCE` 보정 | 패치 1 후 재측정해야 정확한 값이 나옴 |

---

## 요약

1. 전역에 `float d_resid = 0.0f;` 추가
2. `d_val` 계산 2줄 → 반올림 + 잔차 이월 3줄로 교체
3. `velo_val == 0` 분기에 `d_resid = 0.0f;` 추가
4. `serial_tx_poll()`에 `Serial.print("P,"); Serial.println(PWM);` 2줄 추가
5. 플래시 → 속도를 1.0 / 1.4 / 1.8 / 2.2 로 바꿔가며 평지 주행 → 로스백
