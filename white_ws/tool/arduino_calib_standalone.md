# 아두이노 단독 캘리브레이션 (ROS 불필요)

시리얼 모니터(115200)만 열고 진행합니다. **`TICKS_PER_REV = 300` 은 이미 확인됨** →
남은 미지수는 **바퀴 둘레**와 **PWM–속도 관계** 둘뿐입니다.

---

## 측정 1 — 바퀴 둘레 (제일 중요, 5분)

### 원리

```
바퀴둘레 = 이동거리 / (엔코더카운트 / 300)
```

5m 만 밀어도 판별됩니다:

| 가정 둘레 | 5m 이동 시 카운트 |
|---|---|
| 0.8482 m (지름 27.0cm, 현재 설정) | **1768** |
| 0.9076 m (지름 28.9cm, 로스백 추정) | **1653** |

115 카운트 차이 — 엔코더 분해능으로 충분히 구분됩니다.

### 코드 추가

기존 스케치에 **아래 함수 하나**를 추가하고, `process_rx_line()` 에 명령 하나를 답니다.

```c
// ── 캘리브레이션: 엔코더 원시 카운트 출력 ──────────────────────────────
//   'Z' : 카운트 0 리셋
//   'R' : 현재 카운트 + 환산 결과 출력
long calib_zero = 0;

void calib_report() {
  long now_cnt = -1L * readEncoder(1);
  long delta   = now_cnt - calib_zero;
  float rev    = (float)delta / 300.0f;
  Serial.print(F("[CALIB] count=")); Serial.print(delta);
  Serial.print(F("  rev="));         Serial.print(rev, 3);
  Serial.print(F("  현재상수기준거리=")); Serial.print(rev * 0.8482f, 3);
  Serial.println(F(" m"));
  Serial.println(F("        → 실제이동거리 / rev = 진짜 바퀴둘레"));
}
```

`process_rx_line()` 맨 앞에 추가:

```c
void process_rx_line(const char* line) {
  // ── 캘리브레이션 명령 ──
  if (line[0] == 'Z') { calib_zero = -1L * readEncoder(1);
                        Serial.println(F("[CALIB] zero set")); return; }
  if (line[0] == 'R') { calib_report(); return; }

  // (이하 기존 코드 그대로)
  if ((line[0] == 'C' || line[0] == 'S') && line[1] == ',') {
```

### 하는 법

1. 바닥에 **5m 를 줄자로 정확히** 표시 (길수록 정확 — 10m 면 더 좋음)
2. 차를 시작선에 놓고 시리얼 모니터에 **`Z`** 입력 → `zero set`
3. **차를 손으로 밀어** 끝선까지 (모터 전원 꺼도 됨, 엔코더는 SPI 로 계속 셈)
   - ⚠️ 바퀴가 미끄러지지 않게 천천히, 일직선으로
4. **`R`** 입력 → `count`, `rev` 출력
5. 계산:

```
진짜 바퀴둘레 = 실제이동거리(5.00 m) / rev
```

**예시**: 5.00m 밀었는데 `rev=5.510` 이 나오면 → 둘레 = 5.00/5.510 = **0.9074 m**

3회 반복해서 평균 내면 확실합니다.

### 결과 반영

```c
// 아두이노에는 바퀴둘레 상수가 없다(틱 단위로만 동작) → 수정 불필요
```

ROS 쪽 **두 파일 모두** 같은 값으로:
- `driving.py` : `WHEEL_CIRCUMFERENCE = 0.9076`  (현재 `0.27 * math.pi`)
- `motor.py`   : `WHEEL_CIRC = 0.9076`           (현재 `0.8482`)

---

## 측정 2 — PWM ↔ 속도 관계 (v = A·duty − B 확정)

### 원리

지금까지 "2.2 m/s 에 PWM 252" 는 **한 점을 지나는 외삽**이었습니다.
서로 다른 PWM 에서 정상상태 속도를 재면 A, B 가 회귀로 확정됩니다.

### 코드 추가

```c
// ── PWM 스윕 테스트 ────────────────────────────────────────────────
//   'T' : 시작.  PWM 을 단계적으로 올리며 각 단계 정상상태 d_val 출력
//   'X' : 중단
//   ⚠️ 차가 실제로 달린다. 직선 30m 이상 확보하고, 사람이 옆에서 대기할 것.
//      PS2 리모컨을 건드리면 즉시 중단된다(기존 우선권 로직 그대로).
bool  sweep_on   = false;
int   sweep_idx  = 0;
unsigned long sweep_t0 = 0;
long  sweep_sum  = 0;
int   sweep_n    = 0;
const int SWEEP_PWM[] = {60, 80, 100, 120, 140, 160, 180, 205};
const int SWEEP_N = sizeof(SWEEP_PWM)/sizeof(SWEEP_PWM[0]);
#define SWEEP_SETTLE_MS 2000UL     // 정상상태 대기
#define SWEEP_SAMPLE_MS 1000UL     // 평균 구간

void sweep_stop(const __FlashStringHelper* why) {
  sweep_on = false; motor_control(0);
  Serial.print(F("[SWEEP] stop: ")); Serial.println(why);
}

void sweep_update() {
  if (!sweep_on) return;

  // 리모컨 개입 시 즉시 중단(사람 우선)
  if (controller_true || millis() < remote_override_until_ms) {
    sweep_stop(F("remote override")); return;
  }

  unsigned long el = millis() - sweep_t0;
  int pwm = SWEEP_PWM[sweep_idx];
  motor_control(pwm);

  if (el > SWEEP_SETTLE_MS) {              // 안정화 후 샘플 누적
    sweep_sum += d_val; sweep_n++;
  }
  if (el > SWEEP_SETTLE_MS + SWEEP_SAMPLE_MS) {
    float avg = sweep_n ? (float)sweep_sum/sweep_n : 0.0f;
    Serial.print(F("[SWEEP] PWM=")); Serial.print(pwm);
    Serial.print(F("  duty="));      Serial.print(pwm/255.0f, 3);
    Serial.print(F("  d_val_avg=")); Serial.println(avg, 2);
    sweep_idx++; sweep_sum = 0; sweep_n = 0; sweep_t0 = millis();
    if (sweep_idx >= SWEEP_N) { sweep_stop(F("done")); }
  }
}
```

`process_rx_line()` 에 명령 추가 (`Z`/`R` 옆에):

```c
  if (line[0] == 'T') { sweep_on = true; sweep_idx = 0; sweep_t0 = millis();
                        sweep_sum = 0; sweep_n = 0;
                        Serial.println(F("[SWEEP] start")); return; }
  if (line[0] == 'X') { sweep_stop(F("user")); return; }
```

`loop()` 안, `Velo_PID_Control();` **바로 앞**에:

```c
  // ★ 스윕 중에는 속도PID 를 건너뛰고 직접 PWM 을 준다
  if (sweep_on) {
    sweep_update();
    serial_tx_poll();
    update_status_led(false, false, true);
    return;                    // 아래 PID/제어 로직 실행 안 함
  }

  Velo_PID_Control();
```

> `d_val` 은 `Velo_PID_Control()` 이 갱신하는데 스윕 중엔 그걸 건너뜁니다.
> **`sweep_update()` 안에서 엔코더를 직접 읽도록** 하려면 아래를 `sweep_update()` 시작부에 추가:
> ```c
>   static unsigned long lp = 0;
>   if (micros() - lp >= VELO_PID_PERIOD_US) {
>     long c = -1L * readEncoder(1);
>     d_val = c - old_val; old_val = c;
>     lp += VELO_PID_PERIOD_US;
>   }
> ```

### 하는 법

1. **직선 30m 이상** 확보 (8단계 × 3초 ≈ 24초, 평균 1.5 m/s → 약 36m)
2. 시리얼 모니터에 **`T`** 입력
3. 차가 스스로 가속하며 8단계를 지납니다 — **옆에서 따라가며 대기**
4. 위험하면 **`X`** 또는 **PS2 스틱**을 건드리면 즉시 중단
5. 출력 예:

```
[SWEEP] PWM=60   duty=0.235  d_val_avg=1.85
[SWEEP] PWM=100  duty=0.392  d_val_avg=3.42
[SWEEP] PWM=140  duty=0.549  d_val_avg=5.01
[SWEEP] PWM=205  duty=0.804  d_val_avg=7.60
```

### 결과 해석

`d_val` → 속도: `v = d_val × 둘레 / (300 × 0.01)`

이 표를 주시면 **`v = A·duty − B` 회귀**로 A, B 를 확정해서
"2.2 m/s 에 필요한 PWM" 을 추정이 아닌 **실측값**으로 계산해 드립니다.

**나눠서 해도 됩니다** — 4단계씩 두 번(짧은 직선 두 번)으로도 충분합니다.
`SWEEP_PWM[]` 배열만 편집하세요.

---

## 안전 수칙

- 스윕 중에는 **속도PID 가 꺼집니다** (개루프). 명령한 PWM 이 그대로 나갑니다
- PS2 리모컨 우선권은 그대로 살아 있어 **스틱만 건드리면 즉시 정지**
- 통신 워치독(500ms)은 스윕 중 `return` 으로 건너뛰므로, **`X` 또는 리모컨으로만 정지**됩니다
  - 불안하면 `sweep_update()` 안에 `if (millis()-sweep_start > 40000UL) sweep_stop(...)` 같은
    전체 타임아웃을 추가하세요
- 첫 시도는 **PWM 60~120 만** 넣어 낮은 속도로 동작을 확인한 뒤 범위를 넓히세요

---

## 순서 정리

1. **측정 1 (바퀴 둘레)** — 위험 0, 5분. 이것만으로 스케일 7% 오차 확정
2. `d_val` 절삭 패치 적용 (`arduino_patch_step1.md` 패치 1)
3. **측정 2 (PWM 스윕)** — 직선 확보 후
4. 결과 보고 `WHEEL_CIRCUMFERENCE` 보정 + PWM 클램프 상향 결정
