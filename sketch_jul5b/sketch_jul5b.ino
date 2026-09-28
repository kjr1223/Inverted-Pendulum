/*
  역진자 스윙업 제어 (Arduino Mega) - 처음부터 새로 작성
  ─────────────────────────────────────────────────────────
  [하드웨어]
  - 카트: MG513P20 12V DC 기어드 모터 + BTS7960 드라이버
  - 엔코더: A=2번핀, B=3번핀 (외부 인터럽트, x4 쿼드러처)
  - 각도 센서: WDD35D4-5K 포텐셔미터 (A0)
  - 좌표계: theta=0 직립(위), theta=±180 아래. u>0 → 카트 오른쪽.

  [동작 순서]
  1. SENSOR_CHECK (5초): 모터 꺼둔 채 raw/theta 출력. 캘리브레이션 확인용.
  2. KICK: 오른쪽 600ms → 왼쪽 500ms. 봉에 초기 각속도 부여.
  3. SWINGUP: 에너지 기반 bang-bang. E < E_target이면 최대 PWM으로 에너지 주입.
  4. BALANCE: |theta| < 30°이면 PD 제어. 봉이 기우는 방향으로 카트가 따라가며 직립 유지.
*/

// ── 핀 ───────────────────────────────────────────────────────────────────────
const int PWM_PIN = 9;   // 속도 (analogWrite)
const int DIR_PIN = 8;   // 방향 (HIGH=오른쪽, LOW=왼쪽)
const int ENC_A   = 2;   // 엔코더 A (INT0)
const int ENC_B   = 3;   // 엔코더 B (INT1)
const int POT_PIN = A0;  // 포텐셔미터

// ── 각도 캘리브레이션 ─────────────────────────────────────────────────────────
// 실측 방법: 봉을 직립으로 세우고 Serial에서 raw= 값 확인 → POT_VAL_UP에 입력.
//            봉을 아래로 내리고 raw= 값 확인 → POT_VAL_DOWN에 입력.
const int   POT_VAL_UP        = 570;   // 직립(0°)일 때 analogRead 측정값
const int   POT_VAL_DOWN      = 40;    // 아래(±180°)일 때 analogRead 측정값
const float POT_DEG_PER_COUNT = 180.0 / (float)(POT_VAL_UP - POT_VAL_DOWN);  // 현재 각도 출력 

// ── 봉 물리 파라미터 ──────────────────────────────────────────────────────────
const float MASS   = 0.10;   // 봉 질량 [kg]
const float LEN    = 0.40;   // 봉 전체 길이 [m]
const float LC     = LEN / 2.0;                        // 무게중심 (균일봉: L/2)
const float INERT  = (1.0/3.0) * MASS * LEN * LEN;    // 관성모멘트 (균일봉: mL²/3)
const float G      = 9.81;

// ── KICK 파라미터 ─────────────────────────────────────────────────────────────
// 정지 상태에서는 thetaDot=0이라 pump 방향이 결정 안 됨 → 채찍으로 초기 각속도 부여
const unsigned long KICK_R1_MS = 300;  // 1단계: 오른쪽
const unsigned long KICK_L_MS  = 500;  // 2단계: 왼쪽 (채찍) — 700→500 (과도한 에너지 주입 방지)
const unsigned long KICK_R2_MS = 100;  // 3단계: 오른쪽 재가속
const int           KICK_PWM   = 235;

// KICK 중 카트가 레일 끝에 부딪히지 않도록. 초과 시 SWINGUP으로 조기 전환.
const long KICK_CART_LIMIT = 1100;  // KICK중에 카트가 ≈ 13.8cm이상 동작하면 벽에 부딪힐 수 있으니 그 전에 스윙업으로 넘어가는 장치  

// ── SWINGUP 파라미터 ──────────────────────────────────────────────────────────
// bang-bang 선택 이유: PD 비례 출력은 ±120° 근방에서 마찰과 평형을 이뤄 봉이 멈춤.
//                     항상 최대 출력으로 마찰을 이기고 계속 펌핑.
const int SWINGUP_PWM = 110;

// ── BALANCE 파라미터 ──────────────────────────────────────────────────────────
const float SWITCH_ANGLE = 30.0;  // |theta| < 이 값이면 SWINGUP → BALANCE 진입 [°]
const float HYSTERESIS   = 10.0;  // BALANCE → SWINGUP 복귀 기준: SWITCH_ANGLE + HYSTERESIS [°]
const int   BALANCE_PWM  = 180;   // BALANCE 최대 출력

// PD 게인: u = KP_BAL*thetaRad + KD_BAL*thetaDotRad
// 동작 확인법: 봉이 오른쪽으로 기울 때 카트가 오른쪽으로 따라가면 정상.
//             반대로 가면 KP_BAL, KD_BAL 부호를 양수로 바꿀 것.
const float KP_BAL = 50.0;  // 각도 비례 게인 (기울기에 비례해서 카트 이동)
const float KD_BAL = 100.0;  // 각속도 미분 게인 (너무 빠르게 기울면 감쇠)

// ── 모터 공통 ─────────────────────────────────────────────────────────────────
const int   MIN_PWM   = 120;  // 정지마찰 극복 최소값 (실측: 120 미만이면 안 움직임)
const float DEADBAND  = 3.0;  // |u| < 3이면 모터 정지

// ── 필터 ──────────────────────────────────────────────────────────────────────
// EMA: new = alpha*raw + (1-alpha)*prev. alpha=1이면 필터 없음, 작을수록 더 부드러움.
const float ALPHA_THETA    = 0.5;  // theta: 중간 강도
const float ALPHA_THETADOT = 0.3;  // thetaDot: 미분값이라 노이즈가 더 심해 더 강하게

// ── 루프 주기 ─────────────────────────────────────────────────────────────────
// 5ms로 하면 ADC 1 step 노이즈(0.3°)가 thetaDot에서 ±64 deg/s로 증폭됨 → 30ms 사용
const unsigned long LOOP_US = 30000;  // 30ms

// ── 시리얼 출력 주기 ──────────────────────────────────────────────────────────
// 루프(30ms)마다 출력하면 너무 빠름. N 루프에 1번만 출력.
// 예: 10 → 300ms마다 출력 (초당 약 3줄)
const int PRINT_EVERY = 20; // 300ms 마다 1줄 숫자 클수록 느림 

// ── 전역 변수 ─────────────────────────────────────────────────────────────────
volatile long encCount = 0;  // ISR에서 갱신. 반드시 noInterrupts()로 읽을 것.

float theta     = 0.0;
float thetaPrev = 0.0;
float thetaDot  = 0.0;

unsigned long lastUs    = 0;
unsigned long kickStart = 0;

bool targetReached = false;  // ±135° 1회 감지 플래그

enum Mode { SENSOR_CHECK, KICK, SWINGUP, BALANCE };
Mode mode = SENSOR_CHECK;

// ── ISR: x4 쿼드러처 디코딩 ──────────────────────────────────────────────────
void isrA() { encCount += (digitalRead(ENC_A) == digitalRead(ENC_B)) ? 1 : -1; }
void isrB() { encCount += (digitalRead(ENC_A) != digitalRead(ENC_B)) ? 1 : -1; }

long getEnc() {
  noInterrupts(); long v = encCount; interrupts(); return v;
}

// ── 각도 읽기 ─────────────────────────────────────────────────────────────────
// (POT_VAL_UP - raw) * DEG_PER_COUNT: 직립 기준 0°, 아래로 갈수록 ±방향 증가
float readTheta() {
  int raw = analogRead(POT_PIN);
  float a = (POT_VAL_UP - raw) * POT_DEG_PER_COUNT;
  while (a < -180.0) a += 360.0;
  while (a >  180.0) a -= 360.0;
  return a;
}

// ── 모터 출력 ─────────────────────────────────────────────────────────────────
void motorOut(float u, int maxPwm) {
  if (fabs(u) < DEADBAND) { analogWrite(PWM_PIN, 0); return; }
  int pwm = (int)constrain(fabs(u), MIN_PWM, maxPwm);
  digitalWrite(DIR_PIN, u > 0 ? HIGH : LOW);
  analogWrite(PWM_PIN, pwm);
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);
  analogWrite(PWM_PIN, 0);

  theta = readTheta();
  thetaPrev = theta;
  lastUs = micros();

  Serial.println("=== SENSOR_CHECK (5초) ===");
  Serial.println("봉을 직립/아래로 움직여서 raw값과 theta가 맞는지 확인하세요.");
  Serial.println("  직립 → theta ≈ 0°  / 아래 → theta ≈ ±180°");
  Serial.println("캘리브레이션이 맞으면 5초 후 자동으로 KICK 시작합니다.");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  if (now - lastUs < LOOP_US) return;
  float dt = (now - lastUs) / 1e6;  // lastUs 갱신 전에 계산 (순서 바꾸면 dt=0)
  lastUs = now;

  int   raw = analogRead(POT_PIN);
  float rawTheta = readTheta();
  // wrap-around 감지: ±180° 경계를 넘으면 EMA가 0°를 가짜로 만들어냄
  // 예) 직전 theta=+179°, rawTheta=-179° → EMA = 0° (가짜 직립) → BALANCE 오진입
  // 차이가 180° 이상이면 wrap이므로 필터 없이 raw 값 바로 사용
  if (fabs(rawTheta - theta) > 180.0) {
    theta = rawTheta;
  } else {
    theta = ALPHA_THETA * rawTheta + (1.0 - ALPHA_THETA) * theta;
  }

  // ────────────────────────────────────────────────────────────────────────────
  //  SENSOR_CHECK: 모터 완전 정지. raw + theta 출력. 5초 후 KICK 자동 진입.
  // ────────────────────────────────────────────────────────────────────────────
  if (mode == SENSOR_CHECK) {
    thetaPrev = theta;  // SENSOR_CHECK 동안 thetaPrev 유지 (KICK 진입 시 스파이크 방지)
    static unsigned long checkStart = 0;
    static bool checkStarted = false;
    if (!checkStarted) { checkStart = millis(); checkStarted = true; }

    analogWrite(PWM_PIN, 0);  // 모터 정지

    // 남은 시간 계산해서 출력
    long remaining = 5 - (long)((millis() - checkStart) / 1000);
    Serial.print("raw="); Serial.print(raw);
    Serial.print("  theta="); Serial.print(theta, 1);
    Serial.print("°  (KICK까지 "); Serial.print(remaining); Serial.println("초)");

    if (millis() - checkStart >= 5000) {
      mode = KICK;
      kickStart = millis();
      Serial.println("=== KICK 시작 ===");
    }
    return;
  }

  // ────────────────────────────────────────────────────────────────────────────
  //  KICK: 오른쪽 → 왼쪽 → 오른쪽 3단계 채찍. 봉에 초기 각속도 부여.
  // ────────────────────────────────────────────────────────────────────────────
  if (mode == KICK) {
    thetaPrev = theta;  // SWINGUP 첫 루프 thetaDot 스파이크 방지

    // 카트 레일 안전 체크
    if (labs(getEnc()) > KICK_CART_LIMIT) {
      analogWrite(PWM_PIN, 0);
      Serial.println("KICK 중 레일 리밋 → 모터 정지 (SWINGUP 비활성화 중)");
      return;
    }

    unsigned long el = millis() - kickStart;
    if (el < KICK_R1_MS) {
      digitalWrite(DIR_PIN, HIGH); analogWrite(PWM_PIN, KICK_PWM);  // 1단계: 오른쪽
      Serial.print("KICK 1단계(오른쪽)  theta="); Serial.print(theta, 1);
      Serial.print("  raw="); Serial.println(raw);
    } else if (el < KICK_R1_MS + KICK_L_MS) {
      digitalWrite(DIR_PIN, LOW);  analogWrite(PWM_PIN, KICK_PWM);  // 2단계: 왼쪽
      Serial.print("KICK 2단계(왼쪽)    theta="); Serial.print(theta, 1);
      Serial.print("  raw="); Serial.println(raw);
    } else if (el < KICK_R1_MS + KICK_L_MS + KICK_R2_MS) {
      digitalWrite(DIR_PIN, HIGH); analogWrite(PWM_PIN, KICK_PWM);  // 3단계: 오른쪽 재가속
      Serial.print("KICK 3단계(오른쪽)  theta="); Serial.print(theta, 1);
      Serial.print("  raw="); Serial.println(raw);
    } else {
      analogWrite(PWM_PIN, 0);
      mode = SWINGUP;
      Serial.println("KICK 완료 → SWINGUP 진입");
    }
    return;
  }

  // thetaDot 계산 (SWINGUP, BALANCE 공통)
  // ±180° wrap 보정: 360° 회전 시 theta가 +170→-170으로 뛰면 dTheta=-340 → 스파이크 발생
  // 보정 후: -340+360 = +20 (실제 이동량) → thetaDot 정상
  float dTheta = theta - thetaPrev;
  if (dTheta >  180.0f) dTheta -= 360.0f;
  if (dTheta < -180.0f) dTheta += 360.0f;
  float rawDot = dTheta / dt;
  thetaDot = ALPHA_THETADOT * rawDot + (1.0 - ALPHA_THETADOT) * thetaDot;
  thetaPrev = theta;

  float tR = theta    * PI / 180.0;  // [rad]
  float wR = thetaDot * PI / 180.0;  // [rad/s]

  // ── 모드 전환 ─────────────────────────────────────────────────────────────────
  // SWINGUP → BALANCE: 각도 조건 + 속도 조건 둘 다 만족해야 진입
  // 속도 조건이 없으면 하반부를 빠르게 지나갈 때 노이즈로 theta≈0°가 찰나 찍혀 오진입함
  // fabs(thetaDot) < 400: 봉이 진짜로 직립 근처에서 느려진 상태일 때만 진입
  if (mode == SWINGUP && fabs(theta) < SWITCH_ANGLE && fabs(thetaDot) < 1200.0) {
    mode = BALANCE;
    Serial.print("→ BALANCE 진입 (th="); Serial.print(theta,1);
    Serial.print(" w="); Serial.print(thetaDot,1); Serial.println(")");
  }
  // BALANCE → SWINGUP: 봉이 40° 이상 벗어나면 다시 스윙업으로 복귀
  if (mode == BALANCE && fabs(theta) > SWITCH_ANGLE + HYSTERESIS) {
    mode = SWINGUP;
    Serial.println("→ SWINGUP 복귀");
  }

  float u = 0.0;

  if (mode == SWINGUP) {
    // ── SWINGUP: 에너지 기반 bang-bang ─────────────────────────────────────────
    // 비례 에너지 제어: E < E_target → 펌핑, E > E_target → 자동 제동
    // 기존 bang-bang과 달리 에너지 오차에 비례해서 출력이 부드럽게 바뀜 → 360° 과회전 방지
    float E        = MASS * G * LC * cos(tR) + 0.5 * INERT * wR * wR;
    float E_target = MASS * G * LC;
    float pump = (wR * cos(tR) >= 0) ? 1.0 : -1.0;
    float eDrive = 2000.0f * (E - E_target) * pump;
    // 제동(E>E_target)은 255까지 허용, 펌핑(E<E_target)은 SWINGUP_PWM로 제한
    // → 360° 과회전 시 최대 출력으로 제동 가능
    if (E > E_target) {
      u = constrain(eDrive, -255.0f, 255.0f);
    } else {
      u = constrain(eDrive, -(float)SWINGUP_PWM, (float)SWINGUP_PWM);
    }
    motorOut(u, 255);

  } else {  // BALANCE
    // ── BALANCE PD 제어 ───────────────────────────────────────────────────────
    // 확인 포인트: 봉이 오른쪽으로 기울 때(theta>0) 카트가 오른쪽으로 가면 정상.
    //             반대로 가면 KP_BAL, KD_BAL을 양수로 뒤집을 것.
    u = KP_BAL * tR + KD_BAL * wR;
    motorOut(u, BALANCE_PWM);
  }

  // 시리얼 출력: PRINT_EVERY 루프마다 1회
  static int printCount = 0;
  if (++printCount >= PRINT_EVERY) {
    printCount = 0;
    Serial.print(mode == BALANCE ? "BAL" : "SWG");
    Serial.print(" th="); Serial.print(theta, 1);
    Serial.print(" w="); Serial.print(thetaDot, 1);
    Serial.print(" enc="); Serial.print(getEnc());
    Serial.print(" u="); Serial.println(u, 0);
  }
}
