"""Cart-pendulum 동역학 모델 (DC 기어드 모터 + 벨트 카트 + 봉 진자).

좌표 / 부호 (sketch_jul5a/b 와 동일한 convention 을 가정):
    x      카트 위치 [m], +x = DIR=HIGH 방향("오른쪽")
    theta  진자 각도 [rad], 0 = upright, +theta = 봉이 +x 쪽으로 기울어짐
    u      모터 명령 [PWM count], +u = +x 방향 구동, 하드웨어 범위 ±255
    (실제 부호 관계는 TEST 1/2/5 측정 전까지 미확인)

비선형 운동방정식 (Lagrange, 진자 질점 위치 = (x + l sinθ, l cosθ)):
    (M + m) ẍ + m l cosθ θ̈ − m l sinθ θ̇² = F − b ẋ
    m l cosθ ẍ + J θ̈ − m g l sinθ       = −c θ̇
    J = I_c + m l²  (피벗 기준 관성모멘트)

모터 (전압 구동 DC 모터의 선형 모델을 카트 좌표로 표현):
    F = (M / τ_m) · (K_v · u − ẋ)
    → 진자가 없을 때  ẍ = (K_v·u − ẋ)/τ_m  (PWM 스텝 → 1차 속도응답)
    K_v, τ_m 은 모터 스텝응답(TEST 5 이후)으로 직접 식별할 수 있는 양이다.
    M 은 카트 질량 + 모터 로터/기어의 반사관성을 포함한 등가질량.
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields

import numpy as np

# 파라미터 출처 태그
CODE = "CODE"          # 기존 Arduino 코드/주석에 있는 값 (하드웨어로 검증되지 않음)
ASSUMED = "ASSUMED"    # 코드에도 없음. 시뮬레이션용 임시 가정값
PHYSICS = "PHYSICS"    # 물리 상수


@dataclass
class CartPoleParams:
    # ── 진자 ──
    m_p: float = 0.10            # [kg]  sketch_jul5b MASS
    L_p: float = 0.40            # [m]   sketch_jul5b LEN (균일봉 가정)
    c_p: float = 5e-4            # [N·m·s/rad] 피벗 점성마찰 → TEST 4 감쇠로 교체
    g: float = 9.81              # [m/s²]
    # ── 카트 + 모터 ──
    M_eq: float = 3.0            # [kg] 카트 + 반사관성 등가질량
    tau_m: float = 0.04          # [s]  PWM→카트속도 1차 시정수
    v_max: float = 0.60          # [m/s] PWM=255 무부하 정상상태 카트 속도
    b_c: float = 0.0             # [N·s/m] 모터 외 추가 카트 점성마찰
    # ── 액추에이터 ──
    pwm_max: float = 255.0       # analogWrite 범위
    deadzone_pwm: float = 0.0    # 플랜트 PWM 데드존 (기본 0 = 이상적)
    # ── 센서 / 기구 ──
    counts_per_m: float = 7971.0  # jul5b 주석 "1100 count ≈ 13.8 cm"
    pot_up: int = 576            # sketch_jul5a POT_VAL_UP
    pot_down: int = 40           # sketch_jul5a POT_VAL_DOWN
    x_limit: float = 0.25        # [m] 중앙 기준 레일 반길이
    # ── 제어 주기 ──
    Ts: float = 0.005            # [s]

    source: dict = field(default_factory=lambda: {
        "m_p": (CODE, "sketch_jul5b MASS=0.10 (반올림값, 실측 여부 불명)"),
        "L_p": (CODE, "sketch_jul5b LEN=0.40, 균일봉 가정"),
        "c_p": (ASSUMED, "피벗 마찰, TEST 4 자유진동 감쇠로 식별 예정"),
        "g": (PHYSICS, ""),
        "M_eq": (ASSUMED, "카트 질량 + 모터 반사관성, 미측정"),
        "tau_m": (ASSUMED, "모터 스텝응답으로 식별 예정"),
        "v_max": (ASSUMED, "PWM 255 무부하 카트 속도, 스텝응답으로 식별 예정"),
        "b_c": (ASSUMED, "추가 카트 마찰"),
        "pwm_max": (CODE, "analogWrite 0..255"),
        "deadzone_pwm": (ASSUMED, "기본 0. 코드 주석의 MIN_PWM=100/120 은 TEST 5 로 확인 예정"),
        "counts_per_m": (CODE, "jul5b 주석 1100 count≈13.8 cm, TEST 2 로 확인 예정"),
        "pot_up": (CODE, "sketch_jul5a POT_VAL_UP, TEST 1 로 확인 예정"),
        "pot_down": (CODE, "sketch_jul5a POT_VAL_DOWN, TEST 1 로 확인 예정"),
        "x_limit": (ASSUMED, "레일 길이 미측정"),
        "Ts": (CODE, "sketch_jul5a LOOP_US=5000, TEST 3 로 실제 dt 확인 예정"),
    }, repr=False)

    # ── 파생량 ──
    @property
    def l_c(self) -> float:          # 피벗–무게중심 거리
        return self.L_p / 2.0

    @property
    def J_p(self) -> float:          # 피벗 기준 관성 (균일봉: mL²/3)
        return self.m_p * self.L_p ** 2 / 3.0

    @property
    def K_v(self) -> float:          # [m/s per PWM]
        return self.v_max / self.pwm_max

    @property
    def k_f(self) -> float:          # [N per (m/s)] 모터 속도오차 → 힘
        return self.M_eq / self.tau_m

    @property
    def omega0(self) -> float:       # 진자 고유진동수 (downward), upright 불안정극과 동일 크기
        return np.sqrt(self.m_p * self.g * self.l_c / self.J_p)

    def table(self) -> list[tuple[str, float, str, str]]:
        rows = []
        for f in fields(self):
            if f.name == "source":
                continue
            tag, note = self.source.get(f.name, ("?", ""))
            rows.append((f.name, getattr(self, f.name), tag, note))
        return rows


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def apply_actuator(u: float, p: CartPoleParams, limit: float | None) -> float:
    """플랜트 측 물리적 PWM: 포화(limit=None 이면 없음) 후 데드존."""
    if limit is not None:
        u = float(np.clip(u, -limit, limit))
    if p.deadzone_pwm > 0:
        u = np.sign(u) * max(abs(u) - p.deadzone_pwm, 0.0)
    return u


def dynamics(X: np.ndarray, u_eff: float, p: CartPoleParams) -> np.ndarray:
    """비선형 연속시간 동역학 Ẋ = f(X, u). u_eff 는 데드존/포화 이후의 PWM."""
    _, xd, th, thd = X
    s, c = np.sin(th), np.cos(th)
    m, l, J, Mt = p.m_p, p.l_c, p.J_p, p.M_eq + p.m_p
    F = p.k_f * (p.K_v * u_eff - xd) - p.b_c * xd
    M = np.array([[Mt, m * l * c],
                  [m * l * c, J]])
    rhs = np.array([F + m * l * s * thd ** 2,
                    m * p.g * l * s - p.c_p * thd])
    xdd, thdd = np.linalg.solve(M, rhs)
    return np.array([xd, xdd, thd, thdd])


def rk4_step(X: np.ndarray, u_eff: float, p: CartPoleParams, h: float) -> np.ndarray:
    k1 = dynamics(X, u_eff, p)
    k2 = dynamics(X + 0.5 * h * k1, u_eff, p)
    k3 = dynamics(X + 0.5 * h * k2, u_eff, p)
    k4 = dynamics(X + h * k3, u_eff, p)
    return X + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def linearize(p: CartPoleParams) -> tuple[np.ndarray, np.ndarray]:
    """upright (X=0, u=0) 에서의 해석적 선형화  Ẋ = A X + B u.

    sinθ≈θ, cosθ≈1, θ̇²≈0 로 두면
        Mt ẍ + m l θ̈ = α u − β ẋ
        m l ẍ + J θ̈  = m g l θ − c θ̇
    α = k_f K_v, β = k_f + b_c, D = Mt J − (m l)².
    """
    m, l, J, g, c = p.m_p, p.l_c, p.J_p, p.g, p.c_p
    Mt = p.M_eq + m
    alpha = p.k_f * p.K_v
    beta = p.k_f + p.b_c
    D = Mt * J - (m * l) ** 2
    A = np.array([
        [0, 1, 0, 0],
        [0, -J * beta / D, -(m * l) ** 2 * g / D, m * l * c / D],
        [0, 0, 0, 1],
        [0, m * l * beta / D, Mt * m * g * l / D, -Mt * c / D],
    ])
    B = np.array([[0], [J * alpha / D], [0], [-m * l * alpha / D]])
    return A, B


def numeric_jacobian(p: CartPoleParams, eps: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    """해석적 선형화 검증용 유한차분 Jacobian."""
    X0 = np.zeros(4)
    A = np.zeros((4, 4))
    for i in range(4):
        d = np.zeros(4)
        d[i] = eps
        A[:, i] = (dynamics(X0 + d, 0.0, p) - dynamics(X0 - d, 0.0, p)) / (2 * eps)
    B = ((dynamics(X0, eps, p) - dynamics(X0, -eps, p)) / (2 * eps)).reshape(4, 1)
    return A, B
