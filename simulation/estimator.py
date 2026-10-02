"""Measurement → 상태추정 X̂ = [x, ẋ, θ, θ̇].

IdealEstimator      : 시뮬레이터의 참값을 그대로 사용 (제어 설계 검증용)
SensorEstimator     : (= FDLPFEstimator) 후진차분 + 1차 저역통과. baseline 'sensor' 시나리오.
RegressionEstimator : 최근 N 샘플의 선형회귀 기울기로 속도 추정.
KalmanEstimator     : 선형 이산모델 (Ad, Bd) 기반 정상상태 Kalman filter.

모두 하드웨어와 같은 정보(encoder count, pot raw, 보낸 PWM 명령)만 사용한다.
폐루프는 매 주기 estimate(meas) 후 제어기가 낸 명령을 notify_command(u) 로 알려준다.
"""
from __future__ import annotations

from collections import deque

import numpy as np
from scipy.linalg import solve_discrete_are

from cartpole_model import CartPoleParams, wrap_angle
from plant import Measurement


def raw_to_pos(meas: Measurement, p: CartPoleParams) -> tuple[float, float]:
    x = meas.encoder / p.counts_per_m
    deg = (p.pot_up - meas.pot_raw) * 180.0 / (p.pot_up - p.pot_down)
    return x, wrap_angle(np.radians(deg))


class IdealEstimator:
    name = "ideal"

    def reset(self, meas: Measurement) -> None:
        pass

    def estimate(self, meas: Measurement) -> np.ndarray:
        if meas.true_state is None:
            raise ValueError("IdealEstimator 는 시뮬레이터 참값이 필요합니다.")
        return meas.true_state.copy()

    def notify_command(self, u: float) -> None:
        pass


class SensorEstimator:
    """후진차분 + 1차 IIR 저역통과: v ← v + α((p_k − p_{k−1})/dt − v)."""

    def __init__(self, p: CartPoleParams, fc_xd: float = 30.0, fc_thd: float = 30.0):
        self.p = p
        self.fc_xd = fc_xd
        self.fc_thd = fc_thd
        self._prev = None

    @property
    def name(self):
        return f"FD+LPF {self.fc_thd:g}Hz"

    def raw_to_pos(self, meas: Measurement) -> tuple[float, float]:
        return raw_to_pos(meas, self.p)

    def reset(self, meas: Measurement) -> None:
        x, th = self.raw_to_pos(meas)
        self._prev = (meas.t, x, th)
        self.xd = 0.0
        self.thd = 0.0

    @staticmethod
    def _alpha(fc: float, dt: float) -> float:
        return dt / (dt + 1.0 / (2 * np.pi * fc))

    def estimate(self, meas: Measurement) -> np.ndarray:
        if self._prev is None:
            self.reset(meas)
        x, th = self.raw_to_pos(meas)
        t0, x0, th0 = self._prev
        dt = meas.t - t0
        if dt > 0:
            self.xd += self._alpha(self.fc_xd, dt) * ((x - x0) / dt - self.xd)
            self.thd += self._alpha(self.fc_thd, dt) * (wrap_angle(th - th0) / dt - self.thd)
        self._prev = (meas.t, x, th)
        return np.array([x, self.xd, th, self.thd])

    def notify_command(self, u: float) -> None:
        pass


FDLPFEstimator = SensorEstimator


class RegressionEstimator:
    """최근 N 샘플 (t_i, p_i) 에 직선을 최소제곱 적합한 기울기를 속도로 사용.

    위치는 최신 측정값 그대로 (FD+LPF 와 같은 조건).
    등간격이면 기울기 잡음 = σ·√(12 / (N(N²−1))) / Ts, 지연 ≈ (N−1)·Ts/2.
    """

    def __init__(self, p: CartPoleParams, n: int = 10):
        self.p = p
        self.n = n
        self.name = f"REG N={n}"

    def reset(self, meas: Measurement) -> None:
        x, th = raw_to_pos(meas, self.p)
        self.buf = deque(maxlen=self.n)
        self.th_unwrapped = th
        self.buf.append((meas.t, x, th))

    def estimate(self, meas: Measurement) -> np.ndarray:
        x, th = raw_to_pos(meas, self.p)
        if meas.t > self.buf[-1][0]:
            self.th_unwrapped += wrap_angle(th - wrap_angle(self.th_unwrapped))
            self.buf.append((meas.t, x, self.th_unwrapped))
        if len(self.buf) < 2:
            return np.array([x, 0.0, th, 0.0])
        a = np.array(self.buf)
        tc = a[:, 0] - a[:, 0].mean()
        den = np.dot(tc, tc)
        xd = np.dot(tc, a[:, 1] - a[:, 1].mean()) / den
        thd = np.dot(tc, a[:, 2] - a[:, 2].mean()) / den
        return np.array([x, xd, th, thd])

    def notify_command(self, u: float) -> None:
        pass


class KalmanEstimator:
    """정상상태 Kalman filter (설계모델 Ad, Bd 사용 — 모델이 틀리면 성능이 떨어진다).

    측정 y = [x, θ] (양자화된 encoder / pot).
    프로세스 잡음 = 카트/진자 가속도 백색잡음 (σ_ax [m/s²], σ_aθ [rad/s²]) — 모델 오차를 흡수하는 튜닝값.
    측정 잡음 = 양자화 (LSB²/12) + pot ADC 잡음 (σ_pot counts).
    delay_steps: 명령→적용 지연 (폐루프와 동일하게). predict=True 면
    이미 보낸(아직 적용 전) 명령으로 X̂_{k+delay|k} 를 반환해 지연을 보상.
    """

    def __init__(self, p: CartPoleParams, Ad: np.ndarray, Bd: np.ndarray,
                 sigma_ax: float = 1.0, sigma_ath: float = 5.0, sigma_pot_counts: float = 0.5,
                 delay_steps: int = 0, predict: bool = False):
        self.p, self.Ad, self.Bd = p, Ad, Bd.reshape(4)
        self.delay_steps, self.predict = delay_steps, predict
        Ts = p.Ts
        self.C = np.array([[1.0, 0, 0, 0], [0, 0, 1.0, 0]])
        G = np.array([[0.5 * Ts ** 2, 0], [Ts, 0], [0, 0.5 * Ts ** 2], [0, Ts]])
        Qw = G @ np.diag([sigma_ax ** 2, sigma_ath ** 2]) @ G.T
        q_x = 1.0 / p.counts_per_m
        q_th = np.radians(180.0 / (p.pot_up - p.pot_down))
        Rv = np.diag([q_x ** 2 / 12, q_th ** 2 * (1 / 12 + sigma_pot_counts ** 2)])
        P = solve_discrete_are(Ad.T, self.C.T, Qw, Rv)       # 예측 공분산
        self.L = P @ self.C.T @ np.linalg.inv(self.C @ P @ self.C.T + Rv)
        self.name = (f"KF σa=({sigma_ax:g},{sigma_ath:g})" + (" +pred" if predict else ""))

    def reset(self, meas: Measurement) -> None:
        x, th = raw_to_pos(meas, self.p)
        self.xh = np.array([x, 0.0, th, 0.0])
        self.queue = deque([0.0] * self.delay_steps)   # 보냈지만 아직 적용 전인 명령
        self.u_applied = 0.0                           # 직전 주기 동안 적용된 PWM
        self.first = True

    def estimate(self, meas: Measurement) -> np.ndarray:
        if not self.first:
            self.xh = self.Ad @ self.xh + self.Bd * self.u_applied
        self.first = False
        x, th = raw_to_pos(meas, self.p)
        innov = np.array([x - self.xh[0], wrap_angle(th - self.xh[2])])
        self.xh = self.xh + self.L @ innov
        self.xh[2] = wrap_angle(self.xh[2])
        out = self.xh.copy()
        if self.predict:
            for u in list(self.queue):
                out = self.Ad @ out + self.Bd * u
        return out

    def notify_command(self, u: float) -> None:
        self.queue.append(u)
        self.u_applied = self.queue.popleft()
