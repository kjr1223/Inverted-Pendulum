"""플랜트 인터페이스: 시뮬레이터와 (향후) 실제 Arduino 를 같은 형태로 다룬다.

    LQR Controller ──u──▶ Plant ──Measurement(t, encoder, pot_raw)──▶ Estimator ──X̂──▶ Controller

Measurement 는 실제 하드웨어가 USB Serial 로 줄 수 있는 값
(timestamp, encoder count, potentiometer ADC raw) 만을 기본으로 가진다.
시뮬레이터는 여기에 참값 true_state 를 추가로 채운다 (하드웨어에서는 None).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from cartpole_model import CartPoleParams, apply_actuator, rk4_step, wrap_angle


@dataclass
class Measurement:
    t: float                          # [s]
    encoder: int                      # 카트 엔코더 count
    pot_raw: int                      # A0 ADC raw 0..1023
    true_state: np.ndarray | None = None
    u_applied: float | None = None    # 플랜트가 실제로 받은 PWM (포화/데드존 후)


class Plant(ABC):
    Ts: float

    @abstractmethod
    def reset(self, X0: np.ndarray | None = None) -> Measurement: ...

    @abstractmethod
    def apply(self, u_pwm: float) -> None:
        """다음 샘플까지 유지할 PWM 명령 (ZOH)."""

    @abstractmethod
    def advance(self) -> Measurement:
        """한 제어주기 Ts 만큼 진행하고 새 측정값을 반환 (하드웨어: 다음 샘플 수신까지 대기)."""

    def close(self) -> None:
        pass


# ── 센서 변환 (시뮬레이터 ↔ Arduino raw 값) ─────────────────────────────────
def state_to_raw(x: float, theta: float, p: CartPoleParams) -> tuple[int, int]:
    enc = int(np.round(x * p.counts_per_m))
    deg_per_count = 180.0 / (p.pot_up - p.pot_down)
    raw = int(np.round(p.pot_up - np.degrees(theta) / deg_per_count))
    return enc, int(np.clip(raw, 0, 1023))


class SimulatedPlant(Plant):
    """비선형 cart-pole 을 RK4 (dt_sim) 로 적분. 센서는 정수 count 로 양자화."""

    def __init__(self, p: CartPoleParams, u_limit: float | None = 255.0,
                 dt_sim: float = 0.0005, pot_noise_counts: float = 0.0, seed: int = 0):
        self.p = p
        self.Ts = p.Ts
        self.u_limit = u_limit
        self.n_sub = max(1, int(round(p.Ts / dt_sim)))
        self.h = p.Ts / self.n_sub
        self.pot_noise = pot_noise_counts
        self.rng = np.random.default_rng(seed)
        self.reset()

    def _measure(self) -> Measurement:
        enc, raw = state_to_raw(self.X[0], self.X[2], self.p)
        if self.pot_noise > 0:
            raw = int(np.clip(raw + np.round(self.rng.normal(0, self.pot_noise)), 0, 1023))
        return Measurement(self.t, enc, raw, self.X.copy(), self.u_eff)

    def reset(self, X0=None) -> Measurement:
        self.X = np.zeros(4) if X0 is None else np.asarray(X0, dtype=float).copy()
        self.t = 0.0
        self.u_eff = 0.0
        return self._measure()

    def apply(self, u_pwm: float) -> None:
        self.u_eff = apply_actuator(u_pwm, self.p, self.u_limit)

    def advance(self) -> Measurement:
        for _ in range(self.n_sub):
            self.X = rk4_step(self.X, self.u_eff, self.p, self.h)
        self.X[2] = wrap_angle(self.X[2])
        self.t += self.Ts
        return self._measure()

    def disturb(self, d_thetadot: float) -> None:
        """시각화에서 진자를 '툭 치는' 외란 (각속도 순간 변화)."""
        self.X[3] += d_thetadot


# ── 향후 실제 장비 연동 (현재 단계에서는 사용하지 않음) ──────────────────────
def parse_hw_line(line: str) -> Measurement | None:
    """Arduino 출력 한 줄 "H,<t_us>,<encoder>,<pot_raw>" → Measurement.

    향후 제어 펌웨어가 이 형식으로 보내도록 맞출 예정. 형식이 다르면 None.
    """
    parts = line.strip().split(",")
    if len(parts) != 4 or parts[0] != "H":
        return None
    try:
        return Measurement(int(parts[1]) * 1e-6, int(parts[2]), int(parts[3]))
    except ValueError:
        return None


class SerialHardwarePlant(Plant):
    """실제 cart-pendulum (Arduino Mega, USB Serial) — 자리표시자.

    계획된 동작:
      advance(): 시리얼에서 다음 "H,t_us,enc,pot" 줄을 읽어 Measurement 반환
      apply(u):  "U,<pwm>\\n" 전송 (펌웨어 측에서 포화/리밋/E-stop 을 다시 적용)
    현재 단계에서는 실제 장비와 연동하지 않으므로 생성 자체를 막는다.
    """

    def __init__(self, *args, **kwargs):
        raise NotImplementedError("현재 단계에서는 실제 장비와 연동하지 않습니다 (simulation only).")

    def reset(self, X0=None): ...
    def apply(self, u_pwm): ...
    def advance(self): ...
