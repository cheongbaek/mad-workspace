// Arduino motor/steering controller [PS2 우선권 + 통신 워치독 페일세이프판]
//
// 변경 요약 (PS2 우선권 보장판 대비)
//  ★ [추가] ROS 통신 워치독 페일세이프
//    - state=true(자율주행)인데 일정 시간(COMM_TIMEOUT_MS) ROS의 C 명령이
//      안 들어오면(USB 분리 / 상위 노드 다운 등) 마지막 속도로 폭주하지 않도록
//      구동모터를 즉시 정지시킨다.
//    - state는 그대로 유지하므로, 통신이 복구되면(C 명령 재수신) 자동으로 재개된다.
//    - 리모컨 override 중에는 절대 개입하지 않는다(사람 우선권 유지).
//    - 정상 자율주행(20Hz, 50ms 주기로 C 수신)에서는 절대 발동하지 않는다.
//
// 기존 유지 사항
// 1. PS2 read 동안 Serial RX 인터럽트만 임시 차단 (UCSR0B의 RXCIE0 토글)
// 2. PS2 폴링 주기 30ms
// 3. PS2 통신 stale 자동 감지 + 재설정
// 4. 리모컨 Override 윈도우 300ms
//
// 단위/통신 체계 (이전과 동일)
// - Serial 수신: C,velocity,steer  /  S,0|1
// - velocity 단위: tick/10ms, 부호 포함
// - Serial 송신: E,d_val  (DIAG_DT=1일 때 E,d_val,dt_us)

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

// ★ [추가] ROS 통신 워치독 ─────────────────────────────────────────────────
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

void Velo_PID_Control() {
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - last_pid_us;

  if (elapsed_us < VELO_PID_PERIOD_US) return;

  Curr_val  = -1L * readEncoder(1);
  d_val_raw = Curr_val - old_val;
  old_val   = Curr_val;

  float scale = (float)VELO_PID_PERIOD_US / (float)elapsed_us;
  d_val = (signed long)((float)d_val_raw * scale);

  last_dt_us = elapsed_us;

  Error = (float)velo_val - (float)d_val;

  float dt_ratio = (float)elapsed_us / (float)VELO_PID_PERIOD_US;
  Sum_Error += Error * dt_ratio;
  d_Error = (Error - old_Error) / dt_ratio;

  if (velo_val == 0) {
    PWM = 0;
    Sum_Error = 0;
    old_Error = 0;
  } else {
    PWM = (int)(velo_Kp * Error + velo_Ki * Sum_Error + velo_Kd * d_Error);
    if (PWM > 205) {
      PWM = 205;
      Sum_Error -= Error * dt_ratio;
    }
    if (PWM < -205) {
      PWM = -205;
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
const float STEER_DEADBAND = 1.0;
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
    steer_motor_control(0); error_i_s = 0.0; pwm_prev_s = 0; error_old_s = error_s; return;
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
