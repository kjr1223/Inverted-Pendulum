/*
  pendulum_sysid.ino — 하드웨어 검증 / 시스템 식별 전용 (Arduino Mega)
  ─────────────────────────────────────────────────────────────────────
  이 스케치는 자동 balancing / swing-up / MPC / RL 을 전혀 실행하지 않는다.
  모터는 TEST 5 에서 사용자가 명령한 저출력 펄스로만 움직인다.

  핀 / 엔코더 디코딩 / 각도 부호는 현재 장비의 학부생 baseline 코드와 동일하게 맞춘다
  (같은 장비에서 측정한 결과를 baseline 제어기의 부호 convention 에 그대로 대응시키기 위해).
    PWM    D9  analogWrite (Mega Timer2, 약 490 Hz)
    DIR    D8  digitalWrite
    BRAKE  드라이버 BRAKE 는 GND 에 직접 연결 (D7 은 사용하지 않음)
    ENC    A=D3, B=D2  CHANGE 인터럽트 x4  (오른쪽 이동 시 count 증가, 실측 확인)
    POT    A0  WDD35D4-5K (양 끝 5V/GND, 가운데 A0)
    theta = ANGLE_SIGN * (raw - potUp) * 180 / |potDown - potUp|, ±180 wrap
            0 = 직립, ±180 = 하향

  모터 드라이버: L298N 기반 PWM / DIR / BRAKE 방식 보드.
  PWM=0 이면 DIR 과 무관하게 정지이므로 motorOff() 는 두 핀을 모두 LOW 로 둔다.

  카트 거리 환산 (참고, 이 스케치는 count 만 출력한다):
    nominal 1.11768e-4 m/count (13x20x4 CPR, 풀리 r=0.0185 m) x 실측 보정 0.440/0.465
    = 약 1.0576e-4 m/count. 실제 레일 전체 이동 길이 0.440 m.

  Serial 250000 baud, 줄 단위 명령 (Newline). '?' 입력 시 도움말.
    0: IDLE   1: 센서 캘리브레이션   2: 엔코더 캘리브레이션
    3: 루프 타이밍   4: 진자 자유진동   5: 모터 방향/데드존 (수동)

  출력 형식: '#' 로 시작하면 정보/요약, 그 외는 "태그,값,값..." CSV.
*/

#include <math.h>
#include <avr/wdt.h>

// ── TEST 5 (모터 구동) 컴파일 스위치 ─────────────────────────────────────────
// 0: 모터 구동 코드가 바이너리에 존재하지 않음. D8/D9 에는 LOW 만 쓴다.
//    TEST 1~4 는 반드시 0 으로 빌드해서 사용.
// 1: TEST 5 사용 가능 (TEST 1~4 결과 확인 후에만 변경).
#define ENABLE_MOTOR_TEST 0

// ── 핀 (현재 장비 배선, 학부생 baseline 코드와 동일) ─────────────────────────
const uint8_t PWM_PIN = 9;   // Mega: PH6 / OC2B (Timer2)
const uint8_t DIR_PIN = 8;   // Mega: PH5 / OC4C (Timer4)
static_assert(PWM_PIN == 9 && DIR_PIN == 8, "motorOff()/earlyMotorSafe() 는 D8=PH5, D9=PH6 기준");
const uint8_t ENC_A   = 3;   // 엔코더 A 선 → D3
const uint8_t ENC_B   = 2;   // 엔코더 B 선 → D2
const uint8_t POT_PIN = A0;

const long BAUD = 250000;

// ── 각도 캘리브레이션 (잠정값, TEST 1 에서 갱신) ─────────────────────────────
// 현재 장비 관찰값: 직립 raw ≈ 47~52, 하향 안정 raw ≈ 570 (TEST 1 capture 전 대략값)
int potUp   = 50;
int potDown = 570;
// 학부생 baseline 과 같은 부호: raw 가 커지면 theta 감소 (하향 → 약 -180°).
// theta 의 +방향과 카트 +x 방향의 관계는 아직 확인하지 않음 (TEST 1 cright/cleft).
const int ANGLE_SIGN = -1;

// ── 모터 안전 설정 (TEST 5) ──────────────────────────────────────────────────
const int           HARD_PWM_CAP   = 150;   // 명령으로도 넘을 수 없는 절대 상한
int                 softPwmMax     = 60;    // 'm' 명령으로 조정 (≤ HARD_PWM_CAP)
long                posLimit       = 600;   // |enc| 한계 [count] (≈ 6.3 cm, 보정 환산 기준), 'k' 로 조정
long                velLimit       = 2000;  // |속도| 한계 [count/s] (≈ 0.21 m/s), 'v' 로 조정
unsigned int        pulseMs        = 400;   // 펄스 길이, 't' 로 조정
const unsigned int  PULSE_MS_MAX   = 3000;
const unsigned int  PROBE_MS       = 30;    // 배선 probe (DIR=HIGH, PWM=0)
const unsigned int  RAMP_STEP_MS   = 100;   // ramp: 100 ms 마다 PWM +1
long                moveThresh     = 8;     // ramp 가 "움직였다" 판정 [count]
const float         ANGLE_LIMIT_DN = 60.0f; // 모터 테스트 중 진자 |phi_down| 한계 [deg]
const long          START_MARGIN   = 20;    // 시작 시 posLimit 여유 [count]

// ── 전역 ─────────────────────────────────────────────────────────────────────
enum Mode : uint8_t { M_IDLE, M_T1, M_T2, M_T3, M_T4, M_T5 };
Mode mode = M_IDLE;
enum RunKind : uint8_t { RK_PULSE, RK_RAMP, RK_PROBE };   // TEST 5 구동 종류

volatile long encCount = 0;

void isrA() { encCount += (digitalRead(ENC_A) == digitalRead(ENC_B)) ? 1 : -1; }
void isrB() { encCount += (digitalRead(ENC_A) != digitalRead(ENC_B)) ? 1 : -1; }

long getEnc() { noInterrupts(); long v = encCount; interrupts(); return v; }
void setEnc(long v) { noInterrupts(); encCount = v; interrupts(); }

// ── 모터 ─────────────────────────────────────────────────────────────────────
// 리셋 직후 main()/init()/setup() 보다 먼저 실행 (.init3).
// - watchdog 리셋 뒤 AVR 은 WDT 가 켜진 채 시작하므로 MCUSR 클리어 + WDT 끔
// - D8/D9 를 즉시 출력 LOW 로.
//   단, 리셋 후 부트로더가 먼저 실행되는 동안에는 D8/D9 가 high-Z(입력) 이다.
//   이 구간은 소프트웨어로 막을 수 없으므로 드라이버 입력의 풀다운 저항으로 막아야 한다.
void earlyMotorSafe() __attribute__((naked, used, section(".init3")));
void earlyMotorSafe() {
  MCUSR = 0;
  wdt_disable();
  PORTH &= ~(_BV(PH5) | _BV(PH6));
  DDRH  |=  (_BV(PH5) | _BV(PH6));
}

// 레지스터 직접 조작: 타이머 PWM 연결 해제 + 핀 LOW. 수 사이클이라 루프마다 호출 가능.
void motorOff() {
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));   // D9 PWM 분리
  TCCR4A &= ~(_BV(COM4C1) | _BV(COM4C0));   // D8 PWM 분리 (사용하지 않지만 확실히)
  PORTH  &= ~(_BV(PH5) | _BV(PH6));
  DDRH   |=  (_BV(PH5) | _BV(PH6));
}

#if ENABLE_MOTOR_TEST
bool motorAllowed();   // TEST 5 에서 arm 된 경우만 true (아래 정의)

// pwm 은 호출 전에 이미 제한되어 있어야 하지만 여기서 한 번 더 자른다.
void motorApply(bool dirHigh, int pwm) {
  if (!motorAllowed()) { motorOff(); return; }
  pwm = constrain(pwm, 0, min(softPwmMax, HARD_PWM_CAP));
  analogWrite(PWM_PIN, 0);
  digitalWrite(DIR_PIN, dirHigh ? HIGH : LOW);
  analogWrite(PWM_PIN, pwm);
}
#endif

// ── 논블로킹 출력 링버퍼 ─────────────────────────────────────────────────────
// 측정 루프는 링버퍼에 넣기만 하고, 실제 전송은 drainOut() 이
// Serial.availableForWrite() 만큼만 보내므로 Serial 이 루프를 막지 않는다.
const uint16_t OUT_SIZE = 2048;          // 2의 거듭제곱
char     outBuf[OUT_SIZE];
uint16_t outHead = 0, outTail = 0;
unsigned long droppedLines = 0;

uint16_t outUsed() { return (outHead - outTail) & (OUT_SIZE - 1); }

void outWrite(const char* s, uint8_t n) {
  if ((uint16_t)(OUT_SIZE - 1 - outUsed()) < n) { droppedLines++; return; }
  for (uint8_t i = 0; i < n; i++) { outBuf[outHead] = s[i]; outHead = (outHead + 1) & (OUT_SIZE - 1); }
}

void drainOut() {
  int n = Serial.availableForWrite();
  while (n-- > 0 && outTail != outHead) {
    Serial.write(outBuf[outTail]);
    outTail = (outTail + 1) & (OUT_SIZE - 1);
  }
}

void flushOut() {  // 블로킹. 모드 전환/도움말 등 타이밍 무관한 곳에서만 사용
  while (outTail != outHead) { Serial.write(outBuf[outTail]); outTail = (outTail + 1) & (OUT_SIZE - 1); }
}

// ── 줄 조립 (AVR snprintf 는 float 미지원) ──────────────────────────────────
char    line[100];
uint8_t lp = 0;

void lbStart()            { lp = 0; }
void lbStr(const char* s) { while (*s && lp < sizeof(line) - 2) line[lp++] = *s++; }
void lbL(long v)          { char b[12]; ltoa(v, b, 10); lbStr(b); }
void lbUL(unsigned long v){ char b[12]; ultoa(v, b, 10); lbStr(b); }
void lbC()                { lbStr(","); }
void lbFix(float v, uint8_t dec) {
  long scale = 1;
  for (uint8_t i = 0; i < dec; i++) scale *= 10;
  long c = (long)(v * scale + (v >= 0 ? 0.5f : -0.5f));
  if (c < 0) { lbStr("-"); c = -c; }
  lbUL(c / scale);
  if (dec) {
    lbStr(".");
    long f = c % scale;
    for (long s = scale / 10; s > 0; s /= 10) {
      char d[2] = { (char)('0' + (f / s) % 10), 0 };
      lbStr(d);
    }
  }
}
void lbEnd() { line[lp++] = '\n'; outWrite(line, lp); }

// ── 각도 ─────────────────────────────────────────────────────────────────────
float wrap180(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float thetaFromRaw(float raw) {
  if (potUp == potDown) return 0.0f;
  return wrap180(ANGLE_SIGN * (raw - potUp) * 180.0f / fabsf((float)(potDown - potUp)));
}

// downward 기준 편차 (theta=±180 → 0)
float phiDown(float theta) { return wrap180(theta - 180.0f); }

int readAdcAvg(uint8_t n) {
  long s = 0;
  for (uint8_t i = 0; i < n; i++) s += analogRead(POT_PIN);
  return (int)((s + n / 2) / n);
}

// ═════════════════════════════════════════════════════════════════════════════
// TEST 1 — 진자 센서 캘리브레이션 (모터 OFF)
// ═════════════════════════════════════════════════════════════════════════════
unsigned long t1DispNextUs;
bool          capActive = false;
unsigned long capNextUs;
int           capN = 2000, capI = 0, capRef = 0, capMin = 0, capMax = 0;
long          capSum = 0, capSumSq = 0;
char          capLabel[12] = "";
float         lastCapMean = -1.0f;

void t1StartCapture(const char* label) {
  strncpy(capLabel, label[0] ? label : "-", sizeof(capLabel) - 1);
  capLabel[sizeof(capLabel) - 1] = 0;
  capI = 0; capSum = 0; capSumSq = 0;
  capActive = true;
  capNextUs = micros();
  lbStart(); lbStr("#CAP_START,"); lbStr(capLabel); lbC(); lbL(capN); lbStr(",period_us=1000"); lbEnd();
  lbStart(); lbStr("#C,i,raw"); lbEnd();
}

void runT1() {
  unsigned long now = micros();
  if (capActive) {
    if ((long)(now - capNextUs) < 0) return;
    capNextUs += 1000;
    int raw = analogRead(POT_PIN);
    if (capI == 0) { capRef = raw; capMin = raw; capMax = raw; }
    long d = raw - capRef;
    capSum += d; capSumSq += d * d;
    if (raw < capMin) capMin = raw;
    if (raw > capMax) capMax = raw;
    lbStart(); lbStr("C,"); lbL(capI); lbC(); lbL(raw); lbEnd();
    if (++capI >= capN) {
      capActive = false;
      float m   = (float)capSum / capN;
      float var = (float)capSumSq / capN - m * m;
      float mean = capRef + m;
      lastCapMean = mean;
      lbStart(); lbStr("#CAP,"); lbStr(capLabel); lbC(); lbL(capN); lbC();
      lbStr("mean="); lbFix(mean, 2); lbStr(",std="); lbFix(sqrtf(var > 0 ? var : 0), 3);
      lbStr(",min="); lbL(capMin); lbStr(",max="); lbL(capMax);
      lbStr(",theta_deg="); lbFix(thetaFromRaw(mean), 2); lbEnd();
      t1DispNextUs = micros() + 300000UL;
    }
    return;
  }
  if ((long)(now - t1DispNextUs) < 0) return;
  t1DispNextUs = now + 50000UL;  // 20 Hz 표시
  int raw   = analogRead(POT_PIN);
  int raw16 = readAdcAvg(16);
  lbStart(); lbStr("T1,"); lbUL(millis()); lbC(); lbL(raw); lbC(); lbL(raw16); lbC();
  lbFix(thetaFromRaw(raw16), 2); lbEnd();
}

void cmdT1(const char* s) {
  if (s[0] == 'c') { t1StartCapture(s + 1); return; }
  if (s[0] == 'N') { capN = constrain(atoi(s + 1), 100, 10000); Serial.print(F("# capture N=")); Serial.println(capN); return; }
  if (s[0] == 'U' || s[0] == 'D') {
    if (lastCapMean < 0) { Serial.println(F("# 먼저 c 로 capture 하세요")); return; }
    int v = (int)(lastCapMean + 0.5f);
    if (s[0] == 'U') potUp = v; else potDown = v;
    Serial.print(F("# potUp=")); Serial.print(potUp); Serial.print(F(" potDown=")); Serial.println(potDown);
    return;
  }
  Serial.println(F("# T1 명령: c<label>, N<n>, U, D"));
}

// ═════════════════════════════════════════════════════════════════════════════
// TEST 2 — 카트 엔코더 캘리브레이션 (모터 OFF)
// ═════════════════════════════════════════════════════════════════════════════
unsigned long t2NextUs, t2LastPrintMs;
long          t2LastEnc = 0x7FFFFFFF;

void runT2() {
  unsigned long now = micros();
  if ((long)(now - t2NextUs) < 0) return;
  t2NextUs = now + 20000UL;
  long enc = getEnc();
  if (enc != t2LastEnc || millis() - t2LastPrintMs >= 500) {
    t2LastEnc = enc; t2LastPrintMs = millis();
    lbStart(); lbStr("T2,"); lbUL(millis()); lbC(); lbL(enc); lbEnd();
  }
}

void cmdT2(const char* s) {
  if (s[0] == 'z') { setEnc(0); lbStart(); lbStr("#ZERO,"); lbUL(millis()); lbEnd(); return; }
  if (s[0] == 'm') {
    lbStart(); lbStr("#MARK,"); lbStr(s[1] ? s + 1 : "-"); lbC(); lbUL(millis()); lbC(); lbL(getEnc()); lbEnd();
    return;
  }
  Serial.println(F("# T2 명령: z, m<label>"));
}

// ═════════════════════════════════════════════════════════════════════════════
// TEST 3 — 제어 루프 타이밍 (모터 OFF, 제어기와 같은 부하만 계산)
// ═════════════════════════════════════════════════════════════════════════════
unsigned long t3Period = 5000, t3NextUs, t3LastStart, t3WinStartMs;
uint8_t       t3NAvg = 8, t3PrintMode = 0;
bool          t3First = true;
unsigned long t3N, t3DtSum, t3DtMin, t3DtMax, t3ExecSum, t3ExecMax, t3LateMax, t3LastExec;
unsigned long t3OverWin, t3OverTotal, t3TotalN;
float         t3Th, t3ThPrev, t3ThDot;
long          t3EncPrev;
volatile float t3Sink;  // 계산 결과 폐기용 (최적화 방지). 모터로 나가지 않음.

void t3ResetWin() {
  t3N = 0; t3DtSum = 0; t3DtMin = 0xFFFFFFFF; t3DtMax = 0;
  t3ExecSum = 0; t3ExecMax = 0; t3LateMax = 0; t3OverWin = 0;
  t3WinStartMs = millis();
}

void t3Reset() {
  t3ResetWin(); t3OverTotal = 0; t3TotalN = 0; t3First = true;
  t3NextUs = micros(); t3LastStart = t3NextUs;
  t3Th = thetaFromRaw(readAdcAvg(t3NAvg)); t3ThPrev = t3Th; t3ThDot = 0; t3EncPrev = getEnc();
}

void runT3() {
  unsigned long now = micros();
  long late = (long)(now - t3NextUs);
  if (late < 0) return;
  if (late >= (long)t3Period) { t3OverWin++; t3OverTotal++; t3NextUs = now; }  // 한 주기 이상 밀림
  t3NextUs += t3Period;
  unsigned long dt = now - t3LastStart;
  t3LastStart = now;

  // ── 제어기와 동등한 부하: ADC 평균, 엔코더, 필터, 4상태 내적 ──
  int   raw  = readAdcAvg(t3NAvg);
  long  enc  = getEnc();
  float dtS  = dt > 0 ? dt * 1e-6f : t3Period * 1e-6f;
  float thR  = thetaFromRaw(raw);
  t3Th       = 0.5f * thR + 0.5f * t3Th;
  float dth  = wrap180(t3Th - t3ThPrev); t3ThPrev = t3Th;
  t3ThDot    = 0.3f * (dth / dtS) + 0.7f * t3ThDot;
  float xs[4] = { enc * 1e-4f, (enc - t3EncPrev) / dtS * 1e-4f, t3Th * 0.0174533f, t3ThDot * 0.0174533f };
  t3EncPrev  = enc;
  float acc = 0;
  for (uint8_t i = 0; i < 4; i++) acc += xs[i];
  t3Sink = acc;

  if (t3PrintMode == 1) {
    lbStart(); lbStr("L3,"); lbUL(now); lbC(); lbUL(dt); lbC(); lbUL(t3LastExec); lbEnd();
  } else if (t3PrintMode == 2) {  // 기존 스케치 방식: 루프 안에서 직접 Serial.print (비교용)
    flushOut();
    Serial.print("L3 t="); Serial.print(now); Serial.print(" dt="); Serial.print(dt);
    Serial.print(" th="); Serial.print(t3Th, 3); Serial.print(" w="); Serial.println(t3ThDot, 3);
  }

  unsigned long exec = micros() - now;
  t3LastExec = exec;
  if (t3First) { t3First = false; return; }  // 첫 dt 는 모드 진입 지연이 섞이므로 제외
  t3N++; t3TotalN++;
  t3DtSum += dt; if (dt < t3DtMin) t3DtMin = dt; if (dt > t3DtMax) t3DtMax = dt;
  t3ExecSum += exec; if (exec > t3ExecMax) t3ExecMax = exec;
  if (late > (long)t3LateMax) t3LateMax = late;

  if (millis() - t3WinStartMs >= 1000 && t3N > 0) {
    lbStart(); lbStr("T3S,"); lbUL(millis()); lbC(); lbUL(t3N); lbC();
    lbUL(t3DtSum / t3N); lbC(); lbUL(t3DtMin); lbC(); lbUL(t3DtMax); lbC();
    lbUL(t3ExecSum / t3N); lbC(); lbUL(t3ExecMax); lbC(); lbUL(t3LateMax); lbC();
    lbUL(t3OverWin); lbC(); lbUL(t3OverTotal); lbC(); lbUL(t3TotalN); lbC(); lbUL(droppedLines); lbEnd();
    t3ResetWin();
  }
}

void t3PrintHeader() {
  Serial.print(F("# T3 period_us=")); Serial.print(t3Period);
  Serial.print(F(" adc_avg=")); Serial.print(t3NAvg);
  Serial.print(F(" print_mode=")); Serial.println(t3PrintMode);
  Serial.println(F("#T3S,t_ms,n,dt_mean,dt_min,dt_max,exec_mean,exec_max,late_max,overrun_win,overrun_total,n_total,dropped_lines"));
}

void cmdT3(const char* s) {
  if (s[0] == 'a') t3NAvg = constrain(atoi(s + 1), 1, 32);
  else if (s[0] == 'p') t3PrintMode = constrain(atoi(s + 1), 0, 2);
  else if (s[0] == 'T') t3Period = constrain(atol(s + 1), 1000L, 30000L);
  else if (s[0] != 'r') { Serial.println(F("# T3 명령: a<n>, p<0|1|2>, T<us>, r")); return; }
  flushOut(); t3PrintHeader(); t3Reset();
}

// ═════════════════════════════════════════════════════════════════════════════
// TEST 4 — 진자 자유진동 (모터 OFF)
// ═════════════════════════════════════════════════════════════════════════════
unsigned long t4Period = 2000, t4DurS = 30;
bool          t4Active = false;
unsigned long t4NextUs, t4StartUs, t4N, t4Over, t4DeadZone, t4DropStart;

void t4Stop() {
  t4Active = false;
  lbStart(); lbStr("#T4_END,n="); lbUL(t4N); lbStr(",overruns="); lbUL(t4Over);
  lbStr(",deadzone_hits="); lbUL(t4DeadZone); lbStr(",dropped_lines="); lbUL(droppedLines - t4DropStart); lbEnd();
}

void runT4() {
  if (!t4Active) return;
  unsigned long now = micros();
  long late = (long)(now - t4NextUs);
  if (late < 0) return;
  if (late >= (long)t4Period) { t4Over++; t4NextUs = now; }
  t4NextUs += t4Period;
  int raw = analogRead(POT_PIN);
  if (raw <= 3 || raw >= 1020) t4DeadZone++;
  float th = thetaFromRaw(raw);
  lbStart(); lbStr("F,"); lbUL(now - t4StartUs); lbC(); lbL(raw); lbC(); lbFix(th, 2); lbC(); lbFix(phiDown(th), 2); lbEnd();
  t4N++;
  if (now - t4StartUs >= t4DurS * 1000000UL) t4Stop();
}

void cmdT4(const char* s) {
  if (s[0] == 's') {
    if (t4Active) return;
    t4N = 0; t4Over = 0; t4DeadZone = 0; t4DropStart = droppedLines;
    flushOut();
    Serial.print(F("#T4_START,period_us=")); Serial.print(t4Period);
    Serial.print(F(",dur_s=")); Serial.print(t4DurS);
    Serial.print(F(",potUp=")); Serial.print(potUp); Serial.print(F(",potDown=")); Serial.println(potDown);
    Serial.println(F("#F,t_us,raw,theta_deg,phi_down_deg"));
    t4StartUs = micros(); t4NextUs = t4StartUs; t4Active = true;
    return;
  }
  if (s[0] == 'x') { if (t4Active) t4Stop(); return; }
  if (s[0] == 'T') { t4Period = constrain(atol(s + 1), 1000L, 20000L); Serial.print(F("# period_us=")); Serial.println(t4Period); return; }
  if (s[0] == 'd') { t4DurS = constrain(atol(s + 1), 1L, 120L); Serial.print(F("# dur_s=")); Serial.println(t4DurS); return; }
  Serial.println(F("# T4 명령: s, x, T<us>, d<sec>"));
}

// ═════════════════════════════════════════════════════════════════════════════
// TEST 5 — 모터 방향 / 데드존 (수동, 저출력, 시간제한 펄스만)
// ═════════════════════════════════════════════════════════════════════════════
#if ENABLE_MOTOR_TEST
bool        t5Armed = false, t5Running = false, t5DirHigh = false;
bool          t5HighUnlocked = false;   // probe 통과 전에는 DIR=HIGH 금지
bool          t5ProbePrevDir = false;
RunKind       t5Kind;
int           t5Pwm = 0;
unsigned long t5NextUs, t5StartMs, t5RampLastMs, t5Tick;
unsigned int  t5DurMs;
long          t5EncStart, t5VelRef, t5Vel, t5PeakVel;
bool          discardLine = false;

bool motorAllowed() { return mode == M_T5 && t5Armed; }

const char* kindName(RunKind k) { return k == RK_PULSE ? "PULSE" : (k == RK_RAMP ? "RAMP" : "PROBE"); }

void t5Stop(const char* reason) {
  motorOff();
  if (!t5Running) return;
  t5Running = false;
  long enc = getEnc();
  lbStart(); lbStr("#RUN_END,"); lbStr(kindName(t5Kind)); lbC(); lbStr(reason);
  lbStr(",dir="); lbStr(t5DirHigh ? "HIGH" : "LOW"); lbStr(",pwm="); lbL(t5Kind == RK_PROBE ? 0 : t5Pwm);
  lbStr(",dur_ms="); lbUL(millis() - t5StartMs); lbStr(",enc_start="); lbL(t5EncStart);
  lbStr(",enc_end="); lbL(enc); lbStr(",delta="); lbL(enc - t5EncStart); lbStr(",peak_vel_cps="); lbL(t5PeakVel); lbEnd();

  if (t5Kind == RK_PROBE) {
    t5DirHigh = t5ProbePrevDir;
    t5HighUnlocked = labs(enc - t5EncStart) < 5;
    lbStart();
    lbStr(t5HighUnlocked ? "#PROBE_OK: cart stayed at DIR=HIGH,PWM=0 -> dh allowed"
                         : "#PROBE_FAIL: cart MOVED at DIR=HIGH,PWM=0 -> wiring suspect, dh blocked");
    lbEnd();
  }
}

bool t5CanStart() {
  if (!t5Armed)  { Serial.println(F("# 거부: arm 먼저")); return false; }
  if (t5Running) return false;
  if (labs(getEnc()) >= posLimit - START_MARGIN) { Serial.println(F("# 거부: 카트가 위치 한계 근처. 손으로 중앙에 두고 z")); return false; }
  return true;
}

void t5Begin(RunKind k, unsigned int durMs) {
  t5Kind = k; t5DurMs = durMs;
  t5EncStart = getEnc(); t5VelRef = t5EncStart; t5Vel = 0; t5PeakVel = 0;
  t5StartMs = millis(); t5RampLastMs = t5StartMs; t5Tick = 0;
  t5Running = true;
  lbStart(); lbStr("#RUN_START,"); lbStr(kindName(k)); lbStr(",dir="); lbStr(t5DirHigh ? "HIGH" : "LOW");
  lbStr(",pwm="); lbL(k == RK_PROBE ? 0 : t5Pwm); lbStr(",dur_ms="); lbUL(durMs); lbEnd();
  if (!motorAllowed()) { t5Stop("NOT_ALLOWED"); return; }
  if (k == RK_PROBE) { analogWrite(PWM_PIN, 0); digitalWrite(DIR_PIN, HIGH); }  // 의도적으로 PWM=0 + DIR=HIGH
  else motorApply(t5DirHigh, t5Pwm);
}

void runT5() {
  unsigned long now = micros();
  if ((long)(now - t5NextUs) < 0) return;
  t5NextUs += 1000;                                   // 1 kHz 안전 감시
  if ((long)(now - t5NextUs) > 1000) t5NextUs = now + 1000;
  t5Tick++;
  long enc = getEnc();
  if (t5Tick % 10 == 0) { t5Vel = (enc - t5VelRef) * 100; t5VelRef = enc; }  // count/s (10 ms 창)

  if (t5Running) {
    long av = labs(t5Vel);
    if (av > t5PeakVel) t5PeakVel = av;
    float phi = phiDown(thetaFromRaw(analogRead(POT_PIN)));
    unsigned long el = millis() - t5StartMs;
    if (labs(enc) >= posLimit)               t5Stop("POS_LIMIT");
    else if (av > velLimit)                  t5Stop("VEL_LIMIT");
    else if (fabsf(phi) > ANGLE_LIMIT_DN)    t5Stop("ANGLE_LIMIT");
    else if (t5Kind != RK_RAMP && el >= t5DurMs) t5Stop("DONE");
    else if (t5Kind == RK_RAMP) {
      if (labs(enc - t5EncStart) >= moveThresh) {
        t5Stop("MOVED");
        lbStart(); lbStr("#BREAKAWAY,dir="); lbStr(t5DirHigh ? "HIGH" : "LOW"); lbStr(",pwm="); lbL(t5Pwm); lbEnd();
      } else if (millis() - t5RampLastMs >= RAMP_STEP_MS) {
        t5RampLastMs = millis();
        if (t5Pwm >= softPwmMax) t5Stop("RAMP_MAX");
        else motorApply(t5DirHigh, ++t5Pwm);
      }
    }
  }

  // 로그: 구동 중 5 ms, 정지 중 200 ms
  if (t5Tick % (t5Running ? 5 : 200) == 0) {
    int pwmOut = (t5Running && t5Kind != RK_PROBE) ? t5Pwm : 0;
    lbStart(); lbStr("M,"); lbUL(now); lbC(); lbL(pwmOut); lbC(); lbL(t5DirHigh ? 1 : 0); lbC(); lbL(enc); lbEnd();
  }
}

void t5PrintStatus() {
  Serial.print(F("# armed=")); Serial.print(t5Armed);
  Serial.print(F(" dir=")); Serial.print(t5DirHigh ? F("HIGH") : F("LOW"));
  Serial.print(F(" pwm=")); Serial.print(t5Pwm);
  Serial.print(F(" softMax=")); Serial.print(softPwmMax);
  Serial.print(F(" pulse_ms=")); Serial.print(pulseMs);
  Serial.print(F(" posLimit=")); Serial.print(posLimit);
  Serial.print(F(" velLimit=")); Serial.print(velLimit);
  Serial.print(F(" moveThresh=")); Serial.print(moveThresh);
  Serial.print(F(" highUnlocked=")); Serial.print(t5HighUnlocked);
  Serial.print(F(" enc=")); Serial.println(getEnc());
}

void cmdT5(const char* s) {
  if (!strcmp(s, "arm"))    { t5Armed = true;  Serial.println(F("# ARMED")); return; }
  if (!strcmp(s, "disarm")) { t5Stop("DISARM"); t5Armed = false; Serial.println(F("# DISARMED")); return; }
  if (!strcmp(s, "x"))      { t5Stop("USER_X"); return; }
  if (t5Running) return;   // 구동 중 명령은 정지만 유발 (pollSerial 에서 처리)

  if (!strcmp(s, "dl")) { t5DirHigh = false; }
  else if (!strcmp(s, "dh")) {
    if (!t5HighUnlocked) { Serial.println(F("# 거부: DIR=HIGH 는 w (probe) 통과 후 사용 가능")); return; }
    t5DirHigh = true;
  }
  else if (s[0] == 'p') t5Pwm = constrain(atoi(s + 1), 0, softPwmMax);
  else if (!strcmp(s, "+")) t5Pwm = min(t5Pwm + 5, softPwmMax);
  else if (!strcmp(s, "-")) t5Pwm = max(t5Pwm - 5, 0);
  else if (s[0] == 't') pulseMs = constrain(atol(s + 1), 20L, (long)PULSE_MS_MAX);
  else if (s[0] == 'm') { softPwmMax = constrain(atoi(s + 1), 0, HARD_PWM_CAP); t5Pwm = min(t5Pwm, softPwmMax); }
  else if (s[0] == 'k') posLimit = constrain(atol(s + 1), 50L, 20000L);
  else if (s[0] == 'v') velLimit = constrain(atol(s + 1), 100L, 50000L);
  else if (s[0] == 'n') moveThresh = constrain(atol(s + 1), 1L, 200L);
  else if (!strcmp(s, "z")) { setEnc(0); }
  else if (!strcmp(s, "g")) {
    if (!t5CanStart()) return;
    if (t5Pwm == 0) { Serial.println(F("# 거부: pwm=0 (배선 확인은 w)")); return; }
    t5Begin(RK_PULSE, pulseMs); return;
  }
  else if (!strcmp(s, "u")) { if (t5CanStart()) t5Begin(RK_RAMP, 0); return; }
  else if (!strcmp(s, "w")) {
    if (!t5CanStart()) return;
    t5ProbePrevDir = t5DirHigh;
    t5DirHigh = true;   // probe 동안 로그에 HIGH 로 기록, 종료 시 복원
    t5Begin(RK_PROBE, PROBE_MS);
    return;
  }
  else if (!strcmp(s, "s")) { }
  else { Serial.println(F("# T5 명령: arm disarm x dl dh p<n> + - g u w t<ms> m<n> k<n> v<n> n<n> z s")); return; }
  t5PrintStatus();
}

#else  // !ENABLE_MOTOR_TEST — 모터 구동 코드 없음
const bool t5Running = false;
bool       discardLine = false;
void t5Stop(const char*) { motorOff(); }
#endif

// ═════════════════════════════════════════════════════════════════════════════
// 명령 처리 / 모드 전환
// ═════════════════════════════════════════════════════════════════════════════
void printHelp() {
  flushOut();
  Serial.println(F("# ── pendulum_sysid ── 명령은 줄 단위 (Newline)"));
  Serial.println(F("#  0 IDLE | 1 센서 | 2 엔코더 | 3 타이밍 | 4 자유진동 | 5 모터(수동) | st 상태 | ? 도움말"));
  switch (mode) {
    case M_T1:
      Serial.println(F("# T1 (모터 OFF): T1,t_ms,raw,raw16,theta_deg 를 20 Hz 로 출력"));
      Serial.println(F("#  c<label>  : 1 kHz 로 N 샘플 raw 수집 → #CAP mean/std (예: cup, cright, cleft, cdown)"));
      Serial.println(F("#  N<n>      : capture 샘플 수 (기본 2000)"));
      Serial.println(F("#  U / D     : 직전 capture 평균을 potUp / potDown 으로 설정 (이 세션에만 적용)"));
      break;
    case M_T2:
      Serial.println(F("# T2 (모터 OFF): T2,t_ms,enc (변화 시 / 500 ms 마다)"));
      Serial.println(F("#  z : enc=0   m<label> : #MARK 기록 (예: m0, mR10, mL10)"));
      break;
    case M_T3:
      Serial.println(F("# T3 (모터 OFF): 제어기와 같은 계산 부하로 고정주기 루프, 1초마다 T3S 요약"));
      Serial.println(F("#  a<n> ADC 평균 수 | p0 요약만 | p1 매 루프 링버퍼 출력 | p2 매 루프 직접 Serial.print | T<us> 주기 | r 리셋"));
      t3PrintHeader();
      break;
    case M_T4:
      Serial.println(F("# T4 (모터 OFF): 고정주기 각도 기록  F,t_us,raw,theta_deg,phi_down_deg"));
      Serial.println(F("#  s 시작 | x 중지 | T<us> 주기(기본 2000) | d<sec> 길이(기본 30)"));
      Serial.println(F("#  하향 raw≈570 은 ADC 중간 영역. 직립 raw≈50 쪽이 포텐셔미터 dead zone(raw 0) 근처."));
      Serial.println(F("#  자유진동은 진폭 약 10° 이내로 (선형 영역 감쇠 측정)."));
      break;
#if ENABLE_MOTOR_TEST
    case M_T5:
      Serial.println(F("# T5 모터 수동 테스트. TEST 1~4 정상 확인 후에만 사용."));
      Serial.println(F("#  카트를 레일 중앙에 두고 z → arm. 한 손은 모터 전원 스위치에."));
      Serial.println(F("#  어떤 입력이든 구동 중이면 즉시 정지 (그 줄은 명령으로 처리 안 됨)."));
      Serial.println(F("#  dl/dh 방향(dh 는 w 통과 후) | p<n> pwm | +/- 5씩 | g 펄스 | u ramp(+1/100ms, 움직이면 정지)"));
      Serial.println(F("#  w 배선 probe (DIR=HIGH,PWM=0, 30 ms) | x 정지 | t<ms> 펄스길이 | m<n> soft PWM 상한(≤150)"));
      Serial.println(F("#  k<n> |enc| 한계 | v<n> 속도 한계 count/s | n<n> ramp 이동판정 | z enc=0 | s 상태 | arm/disarm"));
      Serial.println(F("#M,t_us,pwm,dir(1=HIGH),enc"));
      t5PrintStatus();
      break;
#endif
    default:
      Serial.println(F("# IDLE: 모터 OFF. 1~4 로 테스트 선택 (5 는 ENABLE_MOTOR_TEST=1 빌드에서만)."));
  }
}

void printStatus() {
  flushOut();
  Serial.print(F("# mode=")); Serial.print((int)mode);
  Serial.print(F(" potUp=")); Serial.print(potUp); Serial.print(F(" potDown=")); Serial.print(potDown);
  Serial.print(F(" enc=")); Serial.print(getEnc());
  Serial.print(F(" raw=")); Serial.print(analogRead(POT_PIN));
  Serial.print(F(" dropped_lines=")); Serial.print(droppedLines);
  Serial.print(F(" motor_test_build=")); Serial.println(ENABLE_MOTOR_TEST);
#if ENABLE_MOTOR_TEST
  if (mode == M_T5) t5PrintStatus();
#endif
}

void enterMode(uint8_t m) {
  t5Stop("MODE_CHANGE");
  motorOff();
#if !ENABLE_MOTOR_TEST
  if (m == M_T5) {
    Serial.println(F("# TEST 5 비활성 빌드 (ENABLE_MOTOR_TEST=0). 모드 변경 없음, 모터 OFF."));
    return;
  }
#endif
  t4Active = false; capActive = false;
  mode = (Mode)m;
  unsigned long now = micros();
  t1DispNextUs = now; t2NextUs = now; t2LastEnc = 0x7FFFFFFF;
#if ENABLE_MOTOR_TEST
  t5Armed = false; t5NextUs = now; t5Tick = 0;
#endif
  printHelp();
  if (mode == M_T3) t3Reset();
}

void handleCommand(char* s) {
  if (!strcmp(s, "?"))  { printHelp(); return; }
  if (!strcmp(s, "st")) { printStatus(); return; }
  if (s[1] == 0 && s[0] >= '0' && s[0] <= '5') { enterMode(s[0] - '0'); return; }
  switch (mode) {
    case M_T1: cmdT1(s); break;
    case M_T2: cmdT2(s); break;
    case M_T3: cmdT3(s); break;
    case M_T4: cmdT4(s); break;
#if ENABLE_MOTOR_TEST
    case M_T5: cmdT5(s); break;
#endif
    default:   Serial.println(F("# 1~4 로 테스트 선택, ? 도움말"));
  }
}

char    cmdBuf[32];
uint8_t cmdLen = 0;

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (mode == M_T5 && t5Running) { t5Stop("USER_INPUT"); discardLine = true; }
    if (c == '\n' || c == '\r') {
      if (cmdLen && !discardLine) { cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); }
      cmdLen = 0; discardLine = false;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  // earlyMotorSafe() 에서 이미 LOW. Arduino API 상태도 맞춰 둔다.
  motorOff();
  digitalWrite(PWM_PIN, LOW); digitalWrite(DIR_PIN, LOW);
  pinMode(PWM_PIN, OUTPUT);   pinMode(DIR_PIN, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  Serial.begin(BAUD);
  Serial.println();
  Serial.println(F("# pendulum_sysid ready. 자동 제어 없음. 모터 OFF."));
  printHelp();
}

void loop() {
  pollSerial();
#if ENABLE_MOTOR_TEST
  if (mode != M_T5) motorOff();   // TEST 5 외에는 매 반복 강제 OFF (레지스터 몇 개, 1 µs 미만)
#else
  motorOff();
#endif
  switch (mode) {
    case M_T1: runT1(); break;
    case M_T2: runT2(); break;
    case M_T3: runT3(); break;
    case M_T4: runT4(); break;
#if ENABLE_MOTOR_TEST
    case M_T5: runT5(); break;
#endif
    default:   break;
  }
  drainOut();
}
