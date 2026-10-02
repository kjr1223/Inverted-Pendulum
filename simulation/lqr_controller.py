"""이산시간 LQR 설계와 제어기.

제어기는 상태추정값 X̂ 만 입력으로 받는다. 플랜트가 시뮬레이터인지
실제 Arduino 인지는 알지 못한다 (plant.py / estimator.py 참고).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

STATE_NAMES = ["x [m]", "xdot [m/s]", "theta [rad]", "thetadot [rad/s]"]


def discretize(A: np.ndarray, B: np.ndarray, Ts: float) -> tuple[np.ndarray, np.ndarray]:
    """ZOH 이산화 (제어 입력이 Ts 동안 일정 = Arduino analogWrite 와 동일)."""
    Ad, Bd, *_ = cont2discrete((A, B, np.eye(4), np.zeros((4, 1))), Ts, method="zoh")
    return Ad, Bd


def controllability(A: np.ndarray, B: np.ndarray) -> tuple[int, np.ndarray, np.ndarray]:
    n = A.shape[0]
    C = np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(n)])
    sv = np.linalg.svd(C, compute_uv=False)
    return np.linalg.matrix_rank(C), C, sv


def bryson(x_max: float, xd_max: float, th_max: float, thd_max: float, u_max: float):
    """Bryson 규칙: 각 상태/입력의 '허용 최대값' 제곱의 역수를 가중치로."""
    Q = np.diag([1 / x_max ** 2, 1 / xd_max ** 2, 1 / th_max ** 2, 1 / thd_max ** 2])
    R = np.array([[1 / u_max ** 2]])
    return Q, R


def dlqr(Ad, Bd, Q, R):
    """J = Σ X_kᵀ Q X_k + u_kᵀ R u_k 를 최소화하는 u_k = −K X_k."""
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K, P


@dataclass
class ControlOutput:
    u_raw: float        # −K X̂ (+ 데드존 보상) : 포화 전
    u_cmd: float        # 실제로 보내는 PWM 명령 (포화 후)
    saturated: bool


class LQRController:
    """u = −K X̂, 선택적으로 PWM 포화와 데드존 보상.

    u_limit=None 이면 포화 없음 (이상적 비교용).
    deadzone_comp>0 이면 |u|>deadzone_eps 일 때 sign(u)·deadzone_comp 를 더한다.
    """

    def __init__(self, K: np.ndarray, u_limit: float | None = 255.0,
                 deadzone_comp: float = 0.0, deadzone_eps: float = 1.0):
        self.K = np.asarray(K, dtype=float).reshape(1, 4)
        self.u_limit = u_limit
        self.deadzone_comp = deadzone_comp
        self.deadzone_eps = deadzone_eps

    def compute(self, X_hat: np.ndarray) -> ControlOutput:
        u = float(-(self.K @ X_hat)[0])
        if self.deadzone_comp > 0 and abs(u) > self.deadzone_eps:
            u += np.sign(u) * self.deadzone_comp
        if self.u_limit is None:
            return ControlOutput(u, u, False)
        u_cmd = float(np.clip(u, -self.u_limit, self.u_limit))
        return ControlOutput(u, u_cmd, abs(u) > self.u_limit)
