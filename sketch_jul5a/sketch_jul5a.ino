/*
  역진자 직립 균형 제어 (Arduino Mega)
  ─────────────────────────────────────
  좌표계: theta = 0° 직립, theta = ±180° 아래
          u > 0 → 카트 오른쪽

  동작: SENSOR_CHECK(5초) → BALANCE
  KICK/SWINGUP 없음. 손으로 직립 후 손 떼면 균형 유지 시험.

  튜닝 순서:
    1. SENSOR_CHECK에서 theta ≈ 0° 확인 (아니면 POT_VAL_UP 조정)
    2. 봉을 살짝 밀었을 때 카트가 같은 방향으로 가는지 확인
       반대면 KP_BAL 을 음수로 뒤집기
    3. KP_BAL 키우기 → 버티는 힘 증가
    4. KD_BAL 키우기 → 진동 감쇠
*/


// ── 핀 ──────────────────────────────────────────────────────────────────────
const int PWM_PIN = 9;    // 속도 (analogWrite)
const int DIR_PIN = 8;    // 방향 (HIGH = 오른쪽, LOW = 왼쪽)
const int ENC_A   = 2;    // 엔코더 A (INT0)
const int ENC_B   = 3;    // 엔코더 B (INT1)
const int POT_PIN = A0;   // 포텐셔미터


// ── 각도 캘리브레이션 ────────────────────────────────────────────────────────
// 봉을 직립으로 세운 뒤 SENSOR_CHECK에서 raw= 값 확인 → POT_VAL_UP에 입력
// 봉을 아래로 내린 뒤 raw= 값 확인 → POT_VAL_DOWN에 입력
const int   POT_VAL_UP        = 576;
const int   POT_VAL_DOWN      = 40;
const float POT_DEG_PER_COUNT = 180.0f / (float)(POT_VAL_UP - POT_VAL_DOWN);


// ── BALANCE 게인 ─────────────────────────────────────────────────────────────
// u = KP_BAL * theta[rad] + KD_BAL * thetaDot[rad/s]
// KP_BAL: 기울기에 비례해 카트 이동. MIN_PWM=100 넘으려면 최소 700 이상 필요.
// KD_BAL: 각속도 감쇠. 너무 크면 노이즈 증폭 → 요동.
const float KP_BAL = 2000.0f;
const float KD_BAL = 260.0f;


// ── 모터 출력 설정 ───────────────────────────────────────────────────────────
// MIN_PWM: 이 값 미만이면 정지마찰로 모터가 안 움직임 (실측값)
// DEADBAND: |u| < DEADBAND이면 모터 정지 (노이즈 차단)
const int   MIN_PWM  = 100;
const int   MAX_PWM  = 255;
const float DEADBAND = 25.0f;


// ── 필터 ─────────────────────────────────────────────────────────────────────
// EMA: new = alpha * raw + (1-alpha) * prev
// alpha = 1 → 필터 없음, 작을수록 부드럽지만 응답 느려짐
const float ALPHA_THETA    = 0.5f;
const float ALPHA_THETADOT = 0.3f;   // 응답속도 vs 노이즈 균형 (0.1은 47ms 지연으로 너무 느림)


// ── 루프 주기 및 출력 ────────────────────────────────────────────────────────
const unsigned long LOOP_US = 5000;  // 5ms (포텐셔미터 8샘플 평균으로 노이즈 보완)
const int PRINT_EVERY       = 10;    // 10루프 = 50ms마다 시리얼 출력


// ── BALANCE 유지 범위 ────────────────────────────────────────────────────────
// |theta| > 이 값이면 복구 불가 → 모터 정지
const float BALANCE_LIMIT = 30.0f;


// ── 전역 변수 ─────────────────────────────────────────────────────────────────
volatile long encCount = 0;

float theta     = 0.0f;
float thetaPrev = 0.0f;
float thetaDot  = 0.0f;

unsigned long lastUs = 0;

enum Mode { SENSOR_CHECK, BALANCE };
Mode mode = SENSOR_CHECK;


// ── ISR: 엔코더 x4 디코딩 ───────────────────────────────────────────────────
void isrA() { encCount += (digitalRead(ENC_A) == digitalRead(ENC_B)) ?  1 : -1; }
void isrB() { encCount += (digitalRead(ENC_A) != digitalRead(ENC_B)) ?  1 : -1; }


// ── 포텐셔미터 읽기 (8샘플 평균) ────────────────────────────────────────────
float readTheta() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(POT_PIN);
  int raw = (int)(sum / 8);
  float a = (POT_VAL_UP - raw) * POT_DEG_PER_COUNT;
  if (a >  180.0f) a -= 360.0f;
  if (a < -180.0f) a += 360.0f;
  return a;
}


// ── 모터 출력 ────────────────────────────────────────────────────────────────
void motorOut(float u) {
  if (fabsf(u) < DEADBAND) { analogWrite(PWM_PIN, 0); return; }
  // DEADBAND~MAX_PWM 구간을 MIN_PWM~MAX_PWM으로 선형 매핑
  // u=DEADBAND → PWM=MIN_PWM, u=MAX_PWM → PWM=MAX_PWM (절벽 없음)
  float t = (fabsf(u) - DEADBAND) / (MAX_PWM - DEADBAND);
  int pwm = (int)(MIN_PWM + t * (MAX_PWM - MIN_PWM));
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  digitalWrite(DIR_PIN, u > 0 ? HIGH : LOW);
  analogWrite(PWM_PIN, pwm);
}


// ── setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);
  analogWrite(PWM_PIN, 0);

  theta = readTheta();
  thetaPrev = theta;
  lastUs = micros();

  Serial.println("=== SENSOR_CHECK (5초) ===");
  Serial.println("봉을 직립으로 세우고 theta ≈ 0 확인. 5초 후 BALANCE 시작.");
}


// ── loop ─────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  if (now - lastUs < LOOP_US) return;
  float dt = (now - lastUs) / 1e6f;
  lastUs = now;

  // 각도 읽기 + EMA (±180° wrap 보정 포함)
  float rawTheta = readTheta();
  if (fabsf(rawTheta - theta) > 180.0f) theta = rawTheta;
  else theta = ALPHA_THETA * rawTheta + (1.0f - ALPHA_THETA) * theta;

  // thetaDot 계산 (±180° wrap 보정)
  float dTheta = theta - thetaPrev;
  if (dTheta >  180.0f) dTheta -= 360.0f;
  if (dTheta < -180.0f) dTheta += 360.0f;
  thetaDot = ALPHA_THETADOT * (dTheta / dt) + (1.0f - ALPHA_THETADOT) * thetaDot;
  thetaPrev = theta;

  // ── SENSOR_CHECK ──────────────────────────────────────────────────────────
  if (mode == SENSOR_CHECK) {
    static unsigned long start = 0;
    static bool ready = false;
    if (!ready) { start = millis(); ready = true; }

    analogWrite(PWM_PIN, 0);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      long sum = 0;
      for (int i = 0; i < 8; i++) sum += analogRead(POT_PIN);
      noInterrupts(); long encNow = encCount; interrupts();
      Serial.print("raw="); Serial.print((int)(sum / 8));
      Serial.print("  theta="); Serial.print(theta, 3);
      Serial.print("°  enc="); Serial.print(encNow);
      Serial.print("  (BALANCE까지 ");
      Serial.print(5 - (int)((millis() - start) / 1000));
      Serial.println("초)");
    }

    if (millis() - start >= 5000) {
      mode      = BALANCE;
      thetaPrev = theta;
      thetaDot  = 0.0f;
      Serial.println("=== BALANCE 시작 — 손 떼세요 ===");
    }
    return;
  }

  // ── BALANCE ───────────────────────────────────────────────────────────────
  if (fabsf(theta) > BALANCE_LIMIT) {
    analogWrite(PWM_PIN, 0);  // 복구 불가 범위 → 정지
  } else {
    float tR = theta    * PI / 180.0f;
    float wR = thetaDot * PI / 180.0f;
    float u  = KP_BAL * tR + KD_BAL * wR;
    motorOut(u);
  }

  // 시리얼 출력 (50ms마다)
  static int cnt = 0;
  if (++cnt >= PRINT_EVERY) {
    cnt = 0;
    noInterrupts(); long enc = encCount; interrupts();
    float tR = theta    * PI / 180.0f;
    float wR = thetaDot * PI / 180.0f;
    Serial.print("th=");   Serial.print(theta, 3);
    Serial.print(" w=");   Serial.print(thetaDot, 3);
    Serial.print(" u=");   Serial.print(KP_BAL*tR + KD_BAL*wR, 0);
    Serial.print(" enc="); Serial.println(enc);
  }
}
