// Arduino motor/steering controller [PS2 우선권 + 통신 워치독 페일세이프판]
//
// ═══════════════════════════════════════════════════════════════════════════
// ★ 2026-07-29 수정 4건 (로스백 실측 기반) — 아래 [FIX-n] 주석으로 표시
//
//  [FIX-1] d_val 절삭 → 반올림 + 잔차 이월
//     문제: d_val = (long)(d_val_raw * scale) 의 정수 캐스팅이 항상 '내림'.
//           loop() 가 빨라 scale≈0.91~1.00 이라 매 10ms 소수부를 통째로 버렸다.
//     실측: 1.2 m/s 주행 시 적산거리가 실제의 약 80% (50초에 11m 손실).
//           GPS/엔코더 비율이 저속 1.43 ~ 고속 1.10 으로 속도 의존(r=-0.83).
//           → DR(추측항법) 주행이 50초에 11.5m 이탈한 근본 원인.
//     조치: lroundf() 로 반올림하고, 버린 소수는 d_resid 에 담아 다음 주기에 더한다.
//           → 장기 적산 오차가 0 으로 수렴(시뮬 -19.8% → ±0%).
//     ※ 바퀴 둘레(0.8482m)는 정상이다. 타이어 114/30R8.3 기하둘레 0.877m 에
//       하중침하 3~5% 를 적용하면 0.833~0.851 로 현재값과 일치하고,
//       제원표 최고속도 8km/h 로도 교차검증됨(PWM205 → 1.786 vs 실측 1.79).
//
//  [FIX-2] 조향 데드밴드 1.0° → 0.4°
//     문제: ROS 조향 명령 변화가 평균 0.28° 인데 데드밴드가 1.0° 라
//           명령 변화의 97.4% 가 무시됐다. 조향이 계단식으로만 움직여
//           명령한 조향의 49% 만 실현(lag 1.05s, r=+0.646).
//     효과: 1.22m/s 에서 데드밴드발 횡오차 0.140m → 0.056m (-60%).
//     ⚠️ 정지 중 조향모터가 미세하게 떨면(치터링) 0.5~0.6 으로 올릴 것.
//
//  [FIX-3] 데드밴드 안에서 적분 리셋 제거
//     문제: 데드밴드에 들어갈 때마다 error_i_s = 0 으로 버려서, 벗어날 때마다
//           적분을 처음부터 다시 쌓느라 응답이 늦고 리밋사이클이 생겼다.
//     ※ error_i_s 는 '조향각 오차'(A15 포텐셔미터)의 적분이다. IMU 와 무관하며,
//       IMU 헤딩 적분·보정은 ROS 의 gps_imu.py 가 따로 담당한다.
//     ※ 데드밴드 안에서는 return 이 먼저라 적분 갱신 코드에 도달하지 않는다
//       → 지워도 커지지 않고 그대로 멈춰 있을 뿐이며, I_CLAMP(40) 상한도 그대로다.
//
//  [FIX-4] PWM 텔레메트리 추가 ('P,<pwm>')
//     목적: 속도PID 의 실제 PWM 출력을 로스백에 남겨 "속도 부족이 출력 포화 때문인지"
//           를 추측이 아니라 측정으로 확인한다. E 프레임과 독립이라 기존 파싱 무영향.
//     ROS 준비 완료: motor.py 가 'P,pwm' → /motor_pwm 발행, rec.py 가 로스백 기록.
//
//  ※ 속도 PWM 상한은 205 유지(VELO_PWM_MAX). 모터가 정격 24V 인데 리튬 완충 29.4V ×
//    (205/255) = 23.6V 로 정격에 딱 맞다. 255 로 올리면 정격 +22% 과전압이라 발열·
//    수명 저하 위험. 최고속도 8km/h(2.22m/s)는 제원상 한계이며 205 에서 1.79m/s 가
//    정상이다. 올리려면 배터리 종류와 모터 정격을 먼저 확인할 것.
// ═══════════════════════════════════════════════════════════════════════════
//
// ★ [2026-07-30 FIX-5] 조향각 실측 텔레메트리 추가 ('A,<angle>')
//    목적: LFD 상향 재시도(직선 조향 리밋사이클, PP 기하 91% 원인 진단) 중 ROS 쪽
//          조향 지연(τ)을 IMU 요레이트 역산으로 '추정'했는데 로스백마다 0.70~1.25s 로
//          갈려 위상여유 설계가 실측과 안 맞았다(|CTE| +82~94%로 원복). 지연을 추정이
//          아니라 A15 포텐셔미터 실측(Steer_Angle_Measure)으로 직접 재는 게 다음 순서.
//    ROS: motor.py 가 'A,angle' → /steer_angle_measured 발행, rec.py 가 로스백 기록.
//    P 프레임과 같은 방식(E 프레임과 독립된 별도 라인)이라 기존 파싱에 영향 없음.
// ═══════════════════════════════════════════════════════════════════════════
//
// 기존 유지 사항
//  ★ ROS 통신 워치독 페일세이프
//    - state=true(자율주행)인데 COMM_TIMEOUT_MS 이상 C 명령이 끊기면 구동모터 즉시 정지.
//    - state 는 유지되므로 통신 복구 시 자동 재개. 리모컨 override 중엔 개입 안 함.
// 1. PS2 read 동안 Serial RX 인터럽트만 임시 차단 (UCSR0B의 RXCIE0 토글)
// 2. PS2 폴링 주기 30ms
// 3. PS2 통신 stale 자동 감지 + 재설정
// 4. 리모컨 Override 윈도우 300ms
//
// 단위/통신 체계
// - Serial 수신: C,velocity,steer  /  S,0|1
// - velocity 단위: tick/10ms, 부호 포함
// - Serial 송신: E,d_val  (DIAG_DT=1일 때 E,d_val,dt_us)  +  P,PWM  +  A,angle  ← [FIX-4][FIX-5]

#include <MsTimer2.h>
#include <PS2X_lib.h>
#include <SPI.h>
#include <math.h>

#define DIAG_DT 0

// ── 전역 명령 변수 ─────────────────────────────────────────────────────────
int  velocity    = 0;
int  steer_angle = 0;
bool state       = false;

// ── Serial 수신 버퍼 ────────────────────────────────────────────────────────
static char    rx_line[64];
static uint8_t rx_idx = 0;

#define MAX_VEL 255

// ★ ROS 통신 워치독 ────────────────────────────────────────────────────────
//  자율주행 중(state=true) C 명령이 COMM_TIMEOUT_MS 이상 끊기면 즉시 정지.
//  정상 명령 주기(50ms)의 10배라 정상 주행에서는 절대 안 걸린다.
#define COMM_TIMEOUT_MS 500UL
unsigned long last_cmd_rx_ms = 0;

// ── 구동 모터 ───────────────────────────────────────────────────────────────
#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4
#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7

void motor_control(int pwm) {
  if (pwm > 0) {
    digitalWrite(MOTOR1_ENA, HIGH); digitalWrite(MOTOR1_ENB, LOW);  analogWrite(MOTOR1_PWM, pwm);
    digitalWrite(MOTOR2_ENA, HIGH); digitalWrite(MOTOR2_ENB, LOW);  analogWrite(MOTOR2_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(MOTOR1_ENA, LOW);  digitalWrite(MOTOR1_ENB, HIGH); analogWrite(MOTOR1_PWM, -pwm);
    digitalWrite(MOTOR2_ENA, LOW);  digitalWrite(MOTOR2_ENB, HIGH); analogWrite(MOTOR2_PWM, -pwm);
  } else {
    digitalWrite(MOTOR1_ENA, LOW);  digitalWrite(MOTOR1_ENB, LOW);  analogWrite(MOTOR1_PWM, 0);
    digitalWrite(MOTOR2_ENA, LOW);  digitalWrite(MOTOR2_ENB, LOW);  analogWrite(MOTOR2_PWM, 0);
  }
}

// ── 엔코더 (LS7366R SPI) ────────────────────────────────────────────────────
#define ENC1_ADD 22
#define ENC2_ADD 23

void initEncoders() {
  pinMode(ENC1_ADD, OUTPUT); pinMode(ENC2_ADD, OUTPUT);
  digitalWrite(ENC1_ADD, HIGH); digitalWrite(ENC2_ADD, HIGH);
  SPI.begin();
  digitalWrite(ENC1_ADD, LOW);  SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC1_ADD, HIGH);
  digitalWrite(ENC2_ADD, LOW);  SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC2_ADD, HIGH);
}

long readEncoder(int no) {
  unsigned int c1, c2, c3, c4;
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0x60);
  c1 = SPI.transfer(0); c2 = SPI.transfer(0);
  c3 = SPI.transfer(0); c4 = SPI.transfer(0);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
  return ((long)c1<<24)|((long)c2<<16)|((long)c3<<8)|(long)c4;
}

void clearEncoderCount(int no) {
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0x98); SPI.transfer(0); SPI.transfer(0); SPI.transfer(0); SPI.transfer(0);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
  delayMicroseconds(100);
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0xE0);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
}

// ── 속도 PID ────────────────────────────────────────────────────────────────
#define velo_Kp  12
#define velo_Ki  0.25
#define velo_Kd  15

// 속도 PWM 상한. 모터 정격 24V 보호용(리튬 완충 29.4V × 205/255 = 23.6V).
// 올리려면 배터리 종류·모터 정격 확인 후. 255 는 정격 +22% 라 권장하지 않음.
#define VELO_PWM_MAX  205

#define VELO_PID_PERIOD_US  10000UL
#define SERIAL_TX_PERIOD_US 10000UL

long          Curr_val = 0, old_val = 0;
signed long   d_val = 0;
signed long   d_val_raw = 0;
unsigned long last_pid_us = 0;
unsigned long last_dt_us  = VELO_PID_PERIOD_US;

int           velo_val = 0;
float         Error = 0, d_Error = 0, old_Error = 0, Sum_Error = 0;
int           PWM = 0;

float         d_resid = 0.0f;   // [FIX-1] 반올림에서 버린 소수부를 다음 주기로 이월

void Velo_PID_Control() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - last_pid_us;

  if (elapsed_us < VELO_PID_PERIOD_US) return;

  Curr_val  = -1L * readEncoder(1);
  d_val_raw = Curr_val - old_val;
  old_val   = Curr_val;

  float scale = (float)VELO_PID_PERIOD_US / (float)elapsed_us;

  // [FIX-1] 기존: d_val = (signed long)((float)d_val_raw * scale);   ← 항상 내림(절삭)
  //   정수 캐스팅이 소수부를 통째로 버려 매 10ms 계통적으로 거리를 잃었다.
  //   반올림으로 편향을 없애고, 남은 소수는 d_resid 에 담아 다음 주기에 더한다.
  //   → 한 주기 오차는 ±0.5틱이지만 장기 적산 오차는 0 으로 수렴한다.
  float d_exact = (float)d_val_raw * scale + d_resid;
  d_val   = (signed long)lroundf(d_exact);
  d_resid = d_exact - (float)d_val;

  last_dt_us = elapsed_us;

  Error = (float)velo_val - (float)d_val;

  float dt_ratio = (float)elapsed_us / (float)VELO_PID_PERIOD_US;
  Sum_Error += Error * dt_ratio;
  d_Error = (Error - old_Error) / dt_ratio;

  if (velo_val == 0) {
    PWM = 0;
    Sum_Error = 0;
    old_Error = 0;
    d_resid = 0.0f;          // [FIX-1] 정지 중 잔차가 남아 재출발 시 튀는 것 방지
  } else {
    PWM = (int)(velo_Kp * Error + velo_Ki * Sum_Error + velo_Kd * d_Error);
    if (PWM > VELO_PWM_MAX) {
      PWM = VELO_PWM_MAX;
      Sum_Error -= Error * dt_ratio;
    }
    if (PWM < -VELO_PWM_MAX) {
      PWM = -VELO_PWM_MAX;
      Sum_Error -= Error * dt_ratio;
    }
  }

  motor_control(PWM);

  if (elapsed_us > 2 * VELO_PID_PERIOD_US) {
    last_pid_us = now_us;
  } else {
    last_pid_us += VELO_PID_PERIOD_US;
  }

  old_Error = Error;
}

// ── 조향 PID (MsTimer2 20ms - 기존 유지) ────────────────────────────────
#define Steering_Sensor  A15
#define NEURAL_ANGLE     0
#define LEFT_STEER_ANGLE  -21
#define RIGHT_STEER_ANGLE  21
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

const int   AD_MIN = -460;
const int   AD_MAX =  423;

float Kp = 5.0, Ki_s = 1.8, Kd_s = 2.5;

// [FIX-2] 1.0 → 0.4.  ROS 조향 명령 변화가 평균 0.28° 라 1.0° 는 97% 를 삼켰다.
//   ⚠️ 정지 중 조향모터가 미세하게 떨면 0.5~0.6 으로 올릴 것.
const float STEER_DEADBAND = 0.4;

const float I_CLAMP   = 40.0;
const int   PWM_LIMIT = 255;
const int   PWM_SLEW  = 11;
const float STEER_LPF_FC = 12.0f;

double error_s = 0.0, error_old_s = 0.0, error_i_s = 0.0;
int    pwm_prev_s = 0, sensorValue = 0;
float  sensorValue_f = 0.0f;
int    Steer_Angle_Measure = 0, Steering_Angle = NEURAL_ANGLE;

void steer_motor_control(int pwm) {
  if (sensorValue >= AD_MAX || sensorValue <= AD_MIN) {
    digitalWrite(MOTOR3_ENA, LOW); digitalWrite(MOTOR3_ENB, LOW); analogWrite(MOTOR3_PWM, 0);
    return;
  }
  if (pwm > 0) {
    digitalWrite(MOTOR3_ENA, LOW);  digitalWrite(MOTOR3_ENB, HIGH); analogWrite(MOTOR3_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(MOTOR3_ENA, HIGH); digitalWrite(MOTOR3_ENB, LOW);  analogWrite(MOTOR3_PWM, -pwm);
  } else {
    digitalWrite(MOTOR3_ENA, LOW);  digitalWrite(MOTOR3_ENB, LOW);  analogWrite(MOTOR3_PWM, 0);
  }
}

void Steer_PID_Control(float dt_s) {
  error_s = (double)Steering_Angle - (double)Steer_Angle_Measure;
  if (fabs(error_s) <= STEER_DEADBAND) {
    // [FIX-3] 여기서 error_i_s = 0.0 으로 적분을 버리던 것을 제거했다.
    //   데드밴드에 들어갈 때마다 리셋하면 벗어날 때마다 적분을 0부터 다시 쌓아
    //   응답이 늦어지고 리밋사이클이 생긴다. 아래 return 때문에 적분 갱신 코드에는
    //   도달하지 않으므로, 지워도 적분이 커지지 않고 그대로 멈춰 있을 뿐이다.
    //   (I_CLAMP 40 상한도 그대로 살아 있다)
    //   ※ 이 적분은 '조향각 오차'(A15 포텐셔미터)의 적분이며 IMU 와 무관하다.
    //   불안하면 아래 감쇠로 대체 가능:  error_i_s *= 0.98;
    steer_motor_control(0); pwm_prev_s = 0; error_old_s = error_s; return;
  }
  double p  = Kp * error_s;
  double d  = Kd_s * ((error_s - error_old_s) / dt_s);
  double ic = error_i_s + (Ki_s * error_s * dt_s);
  double u  = p + ic + d;
  double u0 = u;
  if (u >  PWM_LIMIT) u =  PWM_LIMIT;
  if (u < -PWM_LIMIT) u = -PWM_LIMIT;
  bool sat = (u != u0);
  if (!sat || ((u>0&&error_s<0)||(u<0&&error_s>0))) {
    error_i_s = ic;
    if (error_i_s >  I_CLAMP) error_i_s =  I_CLAMP;
    if (error_i_s < -I_CLAMP) error_i_s = -I_CLAMP;
  }
  int uc = (int)round(u);
  int du = uc - pwm_prev_s;
  if (du >  PWM_SLEW) uc = pwm_prev_s + PWM_SLEW;
  if (du < -PWM_SLEW) uc = pwm_prev_s - PWM_SLEW;
  steer_motor_control(uc);
  pwm_prev_s = uc; error_old_s = error_s;
}

void control_callback() {
  static unsigned long last_ms = 0;
  unsigned long now = millis();
  float dt_s = (now - last_ms) * 0.001f;
  if (dt_s <= 0.0f) dt_s = 0.02f;
  last_ms = now;

  int raw = analogRead(Steering_Sensor) - 512;
  float RC    = 1.0f / (6.283185f * STEER_LPF_FC);
  float alpha = dt_s / (RC + dt_s);
  sensorValue_f += alpha * ((float)raw - sensorValue_f);
  sensorValue    = (int)round(sensorValue_f);

  int sens = sensorValue;
  if (sens < AD_MIN) sens = AD_MIN;
  if (sens > AD_MAX) sens = AD_MAX;
  Steer_Angle_Measure = (int)round(
    ((double)(sens-AD_MIN)*(RIGHT_STEER_ANGLE-LEFT_STEER_ANGLE)/(double)(AD_MAX-AD_MIN))
    + LEFT_STEER_ANGLE);

  Steering_Angle = NEURAL_ANGLE + steer_angle;
  if (Steering_Angle < LEFT_STEER_ANGLE)  Steering_Angle = LEFT_STEER_ANGLE;
  if (Steering_Angle > RIGHT_STEER_ANGLE) Steering_Angle = RIGHT_STEER_ANGLE;
  Steer_PID_Control(dt_s);
}

// ══════════════════════════════════════════════════════════════════════════
// PS2 컨트롤러 - 우선권 보장 핵심 영역
// ══════════════════════════════════════════════════════════════════════════
#define PS2_DAT 17
#define PS2_CMD 16
#define PS2_SEL 15
#define PS2_CLK 14
PS2X ps2x;
int  con_error = 1;
bool controller_true = false;
int  con_sp = 0;
#define pressures false
#define rumble    false

#define PS2_POLL_PERIOD_US 30000UL
unsigned long last_ps2_us = 0;

#define PS2_RECONFIG_INTERVAL_MS 5000UL
unsigned long last_ps2_recfg_ms = 0;

#define OVERRIDE_DURATION_MS 300UL
#define STICK_ACTIVE_THRESHOLD 30
unsigned long remote_override_until_ms = 0;
int last_remote_sp    = 0;
int last_remote_steer = 0;

unsigned long last_ps2_valid_ms = 0;

// ── PS2 read 동안 Serial RX 인터럽트만 임시 차단 ───────────────────────
static inline void ps2_safe_read() {
  uint8_t saved = UCSR0B;
  UCSR0B &= ~(1 << RXCIE0);   // RX 인터럽트 OFF
  ps2x.read_gamepad(false, 0);
  UCSR0B = saved;             // 복원
}

void controller() {
  unsigned long now_ms = millis();
  unsigned long now_us = micros();

  // ── PS2 미연결 시 주기적으로 재설정 시도 ──
  if (con_error != 0) {
    if (now_ms - last_ps2_recfg_ms >= PS2_RECONFIG_INTERVAL_MS) {
      con_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
      last_ps2_recfg_ms = now_ms;
      if (con_error == 0) {
        last_ps2_valid_ms = now_ms;
      }
    }
    controller_true = false;
    return;
  }

  // ── 폴링 주기 도달했을 때만 ──
  if (now_us - last_ps2_us < PS2_POLL_PERIOD_US) {
    if (now_ms < remote_override_until_ms) {
      controller_true = true;
    }
    return;
  }
  last_ps2_us = now_us;

  // ── PS2 read (Serial RX 인터럽트만 임시 차단) ──
  ps2_safe_read();

  // ── 입력 분석 ──
  bool l1r1     = ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1);
  int  ly       = ps2x.Analog(PSS_LY);
  int  rx_stick = ps2x.Analog(PSS_RX);

  bool stick_active =
      (abs(ly - 128) >= STICK_ACTIVE_THRESHOLD) ||
      (abs(rx_stick - 128) >= STICK_ACTIVE_THRESHOLD);

  bool ps2_likely_alive =
      l1r1 ||
      ps2x.Button(PSB_PAD_UP)    || ps2x.Button(PSB_PAD_DOWN) ||
      ps2x.Button(PSB_PAD_LEFT)  || ps2x.Button(PSB_PAD_RIGHT) ||
      ps2x.Button(PSB_CROSS)     || ps2x.Button(PSB_CIRCLE) ||
      ps2x.Button(PSB_SQUARE)    || ps2x.Button(PSB_TRIANGLE) ||
      (ly       > 5 && ly       < 250) ||
      (rx_stick > 5 && rx_stick < 250);

  if (ps2_likely_alive) {
    last_ps2_valid_ms = now_ms;
  } else if (now_ms - last_ps2_valid_ms > 2000) {
    con_error = 1;
    last_ps2_recfg_ms = now_ms - PS2_RECONFIG_INTERVAL_MS;
    controller_true = false;
    return;
  }

  // ── 리모컨 활성 판정 ──
  if (l1r1 || stick_active) {
    last_remote_sp    = -(map(ly, 0, 255, -10, 10)) * 10;
    last_remote_steer = -map(rx_stick, 0, 255, -21, 21);
    controller_true   = true;
    remote_override_until_ms = now_ms + OVERRIDE_DURATION_MS;
  } else {
    if (now_ms >= remote_override_until_ms) {
      controller_true   = false;
      last_remote_sp    = 0;
      last_remote_steer = 0;
    }
  }
}

// ── Serial 명령 파싱 ─────────────────────────────────────────────────────────
void process_rx_line(const char* line) {
  // 유효한 C/S 명령 라인이 들어오면 통신 살아있음으로 표시.
  //   (C 파싱이 뒤에서 실패하더라도, 명령 라인이 도착했다는 것 자체가 통신 정상)
  if ((line[0] == 'C' || line[0] == 'S') && line[1] == ',') {
    last_cmd_rx_ms = millis();
  }

  if (line[0]=='C' && line[1]==',') {
    const char* p  = line + 2;
    int v = atoi(p);
    const char* c2 = strchr(p, ',');
    if (!c2) return;
    int s = atoi(c2 + 1);

    if (v >  MAX_VEL) v =  MAX_VEL;
    if (v < -MAX_VEL) v = -MAX_VEL;
    if (s >  21) s =  21;
    if (s < -21) s = -21;

    velocity    = v;
    if (millis() >= remote_override_until_ms) {
      steer_angle = s;
    }
    return;
  }

  if (line[0]=='S' && line[1]==',') {
    int st = atoi(line + 2);
    state = (st != 0);
    if (!state) {
      velocity  = 0;
      Sum_Error = 0;
      motor_control(0);
    }
    return;
  }
}

void serial_rx_poll() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rx_line[rx_idx] = '\0';
      if (rx_idx > 0) process_rx_line(rx_line);
      rx_idx = 0;
      continue;
    }
    if (rx_idx < sizeof(rx_line)-1) rx_line[rx_idx++] = ch;
    else rx_idx = 0;
  }
}

// ── Serial 송신 ─────────────────────────────────────────────────────────
unsigned long last_tx_us = 0;

void serial_tx_poll() {
  unsigned long now_us = micros();
  if (now_us - last_tx_us < SERIAL_TX_PERIOD_US) return;
  if (Serial.availableForWrite() < 56) return;   // [FIX-6] D 프레임 추가분 여유(48→56)

  last_tx_us = now_us;

  Serial.print("E,");
#if DIAG_DT
  Serial.print(d_val);
  Serial.print(",");
  Serial.println(last_dt_us);
#else
  Serial.println(d_val);
#endif

  // [FIX-4] 속도PID 의 실제 PWM 출력. E 프레임과 독립된 별도 프레임이라
  //   기존 파싱 규약(motor.py 의 'E' 처리)에 영향이 없다.
  //   ROS: motor.py 가 'P,pwm' → /motor_pwm 발행, rec.py 가 로스백에 기록.
  Serial.print("P,");
  Serial.println(PWM);

  // [FIX-5] A15 포텐셔미터로 실측한 조향각(LEFT_STEER_ANGLE~RIGHT_STEER_ANGLE, deg).
  //   ROS 의 clamped_steer(같은 deg 단위)와 직접 대조하면 명령→실현 지연·비율을
  //   추정이 아니라 측정으로 구할 수 있다. ROS: motor.py 가 'A,angle' →
  //   /steer_angle_measured 발행.
  Serial.print("A,");
  Serial.println(Steer_Angle_Measure);

  // [FIX-6, 2026-07-30] 조향 PID 의 실제 PWM 출력(pwm_prev_s, ±PWM_LIMIT).
  //   목적: A 프레임에서 실측 조향각이 몇 초씩 안 움직이는 구간(로스백 18_36_42,
  //   0.5s+ 고착이 42초 중 82%)을 발견했는데, 이게 "모터에 전류가 가는데 기구적으로
  //   안 움직이는지" 인지 "PID 가 PWM 자체를 안 내보내는지" 구분이 안 됐다. D 가 0 에
  //   가까운데 각도도 안 변하면 기구/전원 쪽, D 가 큰데 각도가 안 변하면 기구 고착
  //   (기어 슬립·바인딩) 쪽으로 좁혀진다. ROS: motor.py 가 'D,pwm' → /steer_pwm 발행.
  Serial.print("D,");
  Serial.println(pwm_prev_s);
}

// ── 감속 슬루율 ─────────────────────────────────────────────────────────
#define DECEL_STEP_PERIOD_US 50000UL
unsigned long last_decel_us = 0;

void decel_slew_update() {
  if (controller_true || state) {
    last_decel_us = micros();
    return;
  }
  unsigned long now_us = micros();
  if (now_us - last_decel_us < DECEL_STEP_PERIOD_US) return;
  last_decel_us = now_us;

  if (velo_val > 0) {
    velo_val--;
  } else if (velo_val < 0) {
    velo_val++;
  }
  if (velo_val == 0) {
    Sum_Error = 0;
  }
}

// ── setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) {
    con_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    if (con_error == 0) break;
    delay(200);   // setup 단계 허용
  }

  pinMode(13, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT); pinMode(MOTOR1_ENA, OUTPUT); pinMode(MOTOR1_ENB, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT); pinMode(MOTOR2_ENA, OUTPUT); pinMode(MOTOR2_ENB, OUTPUT);
  initEncoders(); clearEncoderCount(1); clearEncoderCount(2);
  pinMode(MOTOR3_PWM, OUTPUT); pinMode(MOTOR3_ENA, OUTPUT); pinMode(MOTOR3_ENB, OUTPUT);

  Error = Sum_Error = d_Error = 0;
  old_val = 0; error_s = error_i_s = error_old_s = 0.0; pwm_prev_s = 0;
  d_resid = 0.0f;                                     // [FIX-1] 잔차 초기화

  MsTimer2::set(20, control_callback);
  MsTimer2::start();

  unsigned long now_us = micros();
  last_pid_us   = now_us;
  last_tx_us    = now_us;
  last_ps2_us   = now_us;
  last_decel_us = now_us;

  unsigned long now_ms = millis();
  last_ps2_recfg_ms        = now_ms;
  last_ps2_valid_ms        = now_ms;
  remote_override_until_ms = 0;
  last_cmd_rx_ms           = now_ms;   // 워치독 초기화 (부팅 직후 오발동 방지)

  Serial.println("Arduino Ready. PS2 priority + comms watchdog 500ms failsafe");
  Serial.println("  [FIX] d_val round+resid / deadband 0.4 / no I-reset / P,pwm telemetry");
}

// ── 보드 LED(13번) 상태 표시 ──────────────────────────────────────────
//   - 리모컨 override 중      : LED 계속 켜짐(solid)
//   - 통신두절 페일세이프 중  : LED 빠른 깜빡임(~5Hz)
//   - 자율주행(정상)          : LED 느린 하트비트(~1Hz)
//   - 대기(정지)              : LED 꺼짐
void update_status_led(bool override_active, bool comms_failsafe, bool autonomous) {
  unsigned long t = millis();
  bool on;
  if (override_active) {
    on = true;                                   // solid
  } else if (comms_failsafe) {
    on = ((t / 100) & 1);                         // 5Hz blink
  } else if (autonomous) {
    on = ((t % 1000) < 60);                       // 1Hz 짧은 하트비트
  } else {
    on = false;                                   // off
  }
  digitalWrite(13, on ? HIGH : LOW);
}

// ── loop ────────────────────────────────────────────────────────────────────
void loop() {
  // 1) PS2 컨트롤러 폴링 (30ms 주기)
  controller();

  // 2) Serial 수신 처리
  serial_rx_poll();

  // 3) velo_val 결정 - 리모컨 우선
  unsigned long now_ms = millis();
  bool override_active = (now_ms < remote_override_until_ms);
  // 통신 두절 판정 (자율주행 중에만 의미 있음)
  bool comms_lost = (now_ms - last_cmd_rx_ms > COMM_TIMEOUT_MS);

  if (override_active || controller_true) {
    // ── 리모컨 우선권 발동 (사람 우선, 통신 워치독보다 우선) ──
    motor_control(last_remote_sp);
    velo_val    = 0;
    Sum_Error   = 0;
    steer_angle = last_remote_steer;
  } else if (state && !comms_lost) {
    // ── ROS2 자율주행 (정상) ──
    velo_val = velocity;
  } else if (state && comms_lost) {
    // ── [페일세이프] 자율주행 중 통신 두절 → 즉시 정지 ──
    //   state는 유지 → 통신 복구(C 재수신) 시 다음 루프부터 자동 재개.
    velocity  = 0;
    velo_val  = 0;
    Sum_Error = 0;
    motor_control(0);
  } else {
    // 정지 명령: 시간 기반 슬루 감속
    decel_slew_update();
  }

  // 4) 속도 PID 발동 (10ms 주기)
  Velo_PID_Control();

  // 5) Serial 송신 (10ms 주기)
  serial_tx_poll();

  // 6) 보드 LED 상태 표시 (override / 페일세이프 / 자율 / 대기)
  update_status_led(override_active || controller_true,
                    (state && comms_lost),
                    (state && !comms_lost));
}
