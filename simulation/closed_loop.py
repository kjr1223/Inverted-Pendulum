"""폐루프 실행, 시나리오 정의, 수치 검증 지표.

루프는 Plant / Estimator / Controller 인터페이스만 사용하므로
SimulatedPlant 를 향후 SerialHardwarePlant 로 바꿔도 그대로 동작한다.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from cartpole_model import CartPoleParams, linearize
from estimator import IdealEstimator, SensorEstimator
from lqr_controller import LQRController, bryson, discretize, dlqr
from plant import SimulatedPlant

# Bryson 규칙 기본값 (각 항목의 "허용 최대 크기")
DEFAULT_BRYSON = dict(x_max=0.10, xd_max=0.50, th_max=np.radians(5.0),
                      thd_max=1.0, u_max=255.0)

# 시나리오
#   ideal     : 포화 없음, 참값 상태 → 선형 설계가 비선형 플랜트에서 동작하는지
#   saturated : PWM ±255 포화, 참값 상태 → 액추에이터 한계의 영향
#   sensor    : 포화 + 센서 양자화(엔코더/10bit pot) + ADC 잡음 + 필터 미분 + 1샘플 지연
SCENARIOS = {
    "ideal":     dict(u_limit=None,  estimator="ideal",  delay=0, pot_noise=0.0),
    "saturated": dict(u_limit=255.0, estimator="ideal",  delay=0, pot_noise=0.0),
    "sensor":    dict(u_limit=255.0, estimator="sensor", delay=1, pot_noise=0.5),
}


@dataclass
class Design:
    A: np.ndarray
    B: np.ndarray
    Ad: np.ndarray
    Bd: np.ndarray
    Q: np.ndarray
    R: np.ndarray
    K: np.ndarray
    P: np.ndarray


def design_lqr(p: CartPoleParams, Q=None, R=None) -> Design:
    A, B = linearize(p)
    Ad, Bd = discretize(A, B, p.Ts)
    if Q is None or R is None:
        Q0, R0 = bryson(**DEFAULT_BRYSON)
        Q = Q0 if Q is None else Q
        R = R0 if R is None else R
    K, P = dlqr(Ad, Bd, Q, R)
    return Design(A, B, Ad, Bd, Q, R, K, P)


def build(p: CartPoleParams, design: Design, scenario: str, seed: int = 0):
    sc = SCENARIOS[scenario]
    plant = SimulatedPlant(p, u_limit=sc["u_limit"], pot_noise_counts=sc["pot_noise"], seed=seed)
    est = IdealEstimator() if sc["estimator"] == "ideal" else SensorEstimator(p)
    ctrl = LQRController(design.K, u_limit=sc["u_limit"])
    return plant, est, ctrl, sc["delay"]


@dataclass
class Log:
    t: np.ndarray
    X: np.ndarray          # 참값 (시뮬레이터). 하드웨어에서는 추정값으로 대체
    X_hat: np.ndarray
    u_raw: np.ndarray
    u_cmd: np.ndarray
    u_eff: np.ndarray
    sat: np.ndarray
    fell: bool
    meta: dict = field(default_factory=dict)

    def save_csv(self, path: str) -> None:
        data = np.column_stack([self.t, self.X, self.X_hat, self.u_raw, self.u_cmd,
                                self.u_eff, self.sat.astype(int)])
        header = ("t,x,xd,theta,thetad,x_hat,xd_hat,theta_hat,thetad_hat,"
                  "u_raw,u_cmd,u_eff,saturated")
        np.savetxt(path, data, delimiter=",", header=header, comments="", fmt="%.6g")


def run_closed_loop(plant, estimator, controller, X0, t_final: float,
                    delay_steps: int = 0, fall_angle: float = np.radians(60)) -> Log:
    meas = plant.reset(X0)
    estimator.reset(meas)
    pending = deque([0.0] * delay_steps)
    rows = []
    fell = False
    while meas.t < t_final - 1e-9:
        X_hat = estimator.estimate(meas)
        out = controller.compute(X_hat)
        estimator.notify_command(out.u_cmd)
        pending.append(out.u_cmd)
        u_now = pending.popleft()
        plant.apply(u_now)
        X_true = meas.true_state if meas.true_state is not None else X_hat
        rows.append((meas.t, *X_true, *X_hat, out.u_raw, u_now,
                     getattr(plant, "u_eff", np.nan), out.saturated))
        meas = plant.advance()
        th = meas.true_state[2] if meas.true_state is not None else X_hat[2]
        if abs(th) > fall_angle:
            fell = True
            break
    a = np.array(rows, dtype=float)
    return Log(a[:, 0], a[:, 1:5], a[:, 5:9], a[:, 9], a[:, 10], a[:, 11],
               a[:, 12].astype(bool), fell)


def evaluate(log: Log, p: CartPoleParams, scenario: str, tail: float = 1.0) -> dict:
    """수치 검증. 애니메이션이 아니라 이 값으로 성공 여부를 판단한다."""
    t, X = log.t, log.X
    th_deg = np.degrees(X[:, 2])
    tail_mask = t >= t[-1] - tail
    head_mask = t <= t[0] + tail

    def settle_time(sig, tol):
        out = np.nonzero(np.abs(sig) > tol)[0]
        if len(out) == 0:
            return 0.0
        return float(t[out[-1]]) if out[-1] < len(t) - 1 else float("inf")

    # 센서 시나리오는 양자화 때문에 작은 limit cycle 이 남으므로 허용 오차를 넓힌다
    tol_th, tol_x = (0.05, 0.001) if scenario != "sensor" else (0.5, 0.01)
    th_final = float(np.max(np.abs(th_deg[tail_mask])))
    x_final = float(np.max(np.abs(X[tail_mask, 0])))
    norm = np.linalg.norm(X / np.array([0.1, 0.5, np.radians(5), 1.0]), axis=1)
    m = {
        "t_end": float(t[-1]),
        "fell": bool(log.fell),
        "theta_max_deg": float(np.max(np.abs(th_deg))),
        "x_max_m": float(np.max(np.abs(X[:, 0]))),
        "xd_max_mps": float(np.max(np.abs(X[:, 1]))),
        "thd_max_radps": float(np.max(np.abs(X[:, 3]))),
        "u_raw_max": float(np.max(np.abs(log.u_raw))),
        "u_cmd_max": float(np.max(np.abs(log.u_cmd))),
        "sat_fraction": float(np.mean(log.sat)),
        "sat_time_s": float(np.sum(log.sat) * p.Ts),
        "theta_final_deg": th_final,
        "x_final_m": x_final,
        "theta_settle_s(0.5deg)": settle_time(th_deg, 0.5),
        "x_settle_s(5mm)": settle_time(X[:, 0], 0.005),
        "norm_head": float(np.max(norm[head_mask])),
        "norm_tail": float(np.max(norm[tail_mask])),
    }
    m["check_theta_to_0"] = (not log.fell) and th_final < tol_th
    m["check_x_to_0"] = (not log.fell) and x_final < tol_x
    m["check_not_diverging"] = (not log.fell) and m["norm_tail"] < m["norm_head"]
    m["check_rail_ok"] = m["x_max_m"] < p.x_limit
    m["saturation_hit"] = m["sat_fraction"] > 0
    m["PASS"] = all(m[k] for k in ("check_theta_to_0", "check_x_to_0",
                                   "check_not_diverging", "check_rail_ok"))
    return m
