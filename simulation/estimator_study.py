#!/usr/bin/env python3
"""sensor 시나리오 chattering 원인 분석 + 상태추정기 비교.

    python3 simulation/estimator_study.py            # 결과: simulation/results/estimator_study/

고정 조건 (모든 추정기 동일):
    plant    : CartPoleParams() 기본값 (ASSUMED), PWM ±255 포화, pot ADC 잡음 0.5 count, 1샘플 지연
    제어기   : baseline LQR K (Q/R 변경 없음)
    초기조건 : θ0 = 2, 5, 10 deg,  seed = 0..4 (추정기마다 같은 seed 사용)
바뀌는 것은 추정기 하나뿐이다.

※ ASSUMED plant 파라미터 시뮬레이션. 실제 하드웨어 검증 전.
"""
from __future__ import annotations

import csv
import os
from concurrent.futures import ProcessPoolExecutor

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from cartpole_model import CartPoleParams
from closed_loop import SCENARIOS, design_lqr, run_closed_loop
from estimator import IdealEstimator, KalmanEstimator, RegressionEstimator, SensorEstimator
from lqr_controller import LQRController
from plant import Measurement, SimulatedPlant, state_to_raw

BANNER = "SIMULATION ONLY - ASSUMED PLANT PARAMETERS - NOT VALIDATED ON HARDWARE"
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results", "estimator_study")
BASELINE = os.path.join(HERE, "results", "baseline_lqr_v1")

THETA0 = [2.0, 5.0, 10.0]
SEEDS = [0, 1, 2, 3, 4]
T_FINAL = 5.0
T_SS = 2.0                      # 정상상태 구간 시작 [s]
SC = SCENARIOS["sensor"]        # u_limit=255, delay=1, pot_noise=0.5

# 비교할 추정기 (kind, 파라미터)
SPECS = (
    [("lpf", fc) for fc in (30, 20, 15, 10, 7, 5, 3, 2, 1, 0.5)] +
    [("reg", n) for n in (3, 5, 8, 12, 16, 24, 32)] +
    [("kf", (0.5, 2.0, False)), ("kf", (1.0, 5.0, False)), ("kf", (3.0, 20.0, False)),
     ("kf", (1.0, 5.0, True))]
)


# 모델 불일치 강건성: 플랜트만 바꾸고 추정기/제어기는 nominal 모델로 설계된 그대로 둔다.
# (KF 는 모델을 쓰므로 nominal 비교에서 유리 → 실제 장비의 모델 오차에서 어떻게 되는지 확인)
MISMATCH = {
    "nominal": {},
    "motor weak/slow (v_max×0.7, τ×1.5)": dict(v_max=0.42, tau_m=0.06),
    "motor strong/fast (v_max×1.3, τ×0.6)": dict(v_max=0.78, tau_m=0.024),
    "light cart (M_eq×0.5)": dict(M_eq=1.5),
    "pendulum (L×1.2, m×1.3)": dict(L_p=0.48, m_p=0.13),
    "PWM deadzone 30": dict(deadzone_pwm=30.0),
}
ROBUST_SPECS = [("ideal", None), ("lpf", 30), ("lpf", 5), ("lpf", 3), ("lpf", 2), ("reg", 8),
                ("kf", (0.5, 2.0, False)), ("kf", (1.0, 5.0, False)), ("kf", (3.0, 20.0, False))]


def make_estimator(spec, p, d, delay):
    kind, arg = spec
    if kind == "ideal":
        return IdealEstimator()
    if kind == "lpf":
        return SensorEstimator(p, fc_xd=arg, fc_thd=arg)
    if kind == "reg":
        return RegressionEstimator(p, n=arg)
    sax, sath, pred = arg
    return KalmanEstimator(p, d.Ad, d.Bd, sigma_ax=sax, sigma_ath=sath,
                           sigma_pot_counts=SC["pot_noise"], delay_steps=delay, predict=pred)


def spec_name(spec):
    p = CartPoleParams()
    return make_estimator(spec, p, design_lqr(p), 1).name


# ── 1. 해석적 잡음 증폭 ─────────────────────────────────────────────────────
def analytic(p, K):
    Ts = p.Ts
    q = np.radians(180.0 / (p.pot_up - p.pot_down))
    s_quant = q / np.sqrt(12)
    s_tot = q * np.sqrt(1 / 12 + SC["pot_noise"] ** 2)
    qx = 1.0 / p.counts_per_m
    sx = qx / np.sqrt(12)
    K = np.abs(K.reshape(4))
    res = dict(q_deg=np.degrees(q), sigma_quant_deg=np.degrees(s_quant), sigma_tot_deg=np.degrees(s_tot),
               fd_one_count_step_radps=q / Ts,
               fd_std_quant_radps=np.sqrt(2) * s_quant / Ts,
               fd_std_tot_radps=np.sqrt(2) * s_tot / Ts,
               u_from_theta_pos=K[2] * s_tot,
               u_from_fd_thd=K[3] * np.sqrt(2) * s_tot / Ts,
               u_from_one_count_thd=K[3] * q / Ts,
               enc_fd_one_count_mps=qx / Ts,
               u_from_enc_one_count_xd=K[1] * qx / Ts)
    rows = []
    for fc in (30, 20, 15, 10, 7, 5, 3, 2, 1, 0.5):
        a = Ts / (Ts + 1 / (2 * np.pi * fc))
        gain = a / Ts * np.sqrt(2 / (2 - a))                 # FD+EMA 백색잡음 이득
        rows.append(("lpf", fc, gain * s_tot, K[3] * gain * s_tot, delay_lpf(a, Ts)))
    for n in (3, 5, 8, 12, 16, 24, 32):
        gain = np.sqrt(12 / (n * (n ** 2 - 1))) / Ts
        rows.append(("reg", n, gain * s_tot, K[3] * gain * s_tot, (n - 1) / 2 * Ts))
    return res, rows


def delay_lpf(a, Ts, w=2 * np.pi * 1.0):
    """FD+EMA 의 1 Hz 에서의 등가 시간지연 (이상 미분 jω 대비 위상지연 / ω)."""
    z = np.exp(1j * w * Ts)
    H = a * (1 - 1 / z) / Ts / (1 - (1 - a) / z)
    return -(np.angle(H) - np.pi / 2) / w


# ── 2. 폐루프 실험 ──────────────────────────────────────────────────────────
def run_one(args):
    spec, th0, seed = args[:3]
    case = args[3] if len(args) > 3 else "nominal"
    p = CartPoleParams()
    d = design_lqr(p)                                   # 제어기/추정기: 항상 nominal 모델
    p_plant = CartPoleParams(**MISMATCH[case])          # 플랜트: 불일치 케이스
    plant = SimulatedPlant(p_plant, u_limit=SC["u_limit"], pot_noise_counts=SC["pot_noise"], seed=seed)
    est = make_estimator(spec, p, d, SC["delay"])
    ctrl = LQRController(d.K, u_limit=SC["u_limit"])
    lg = run_closed_loop(plant, est, ctrl, [0, 0, np.radians(th0), 0], T_FINAL, SC["delay"])
    return spec, th0, seed, metrics(lg, p), (lg if seed == 0 else None)


def settle(t, sig, tol, t_end):
    out = np.nonzero(np.abs(sig) > tol)[0]
    if len(out) == 0:
        return 0.0
    ts = t[out[-1]] + (t[1] - t[0])
    return float(ts) if ts < t_end - 0.5 else float("inf")


def metrics(lg, p):
    t, X = lg.t, lg.X
    th = np.degrees(X[:, 2])
    du = np.diff(lg.u_cmd)
    ss = t >= T_SS
    ss_du = ss[1:]
    err = lg.X_hat - X
    fell = bool(lg.fell)
    return dict(
        fell=fell,
        rms_theta_deg=float(np.sqrt(np.mean(th ** 2))),
        max_theta_deg=float(np.max(np.abs(th))),
        rms_x_cm=float(np.sqrt(np.mean(X[:, 0] ** 2)) * 100),
        max_x_cm=float(np.max(np.abs(X[:, 0])) * 100),
        rms_u=float(np.sqrt(np.mean(lg.u_cmd ** 2))),
        rms_du=float(np.sqrt(np.mean(du ** 2))),
        sat_ratio=float(np.mean(lg.sat)),
        settle_theta_1deg_s=settle(t, th, 1.0, T_FINAL) if not fell else float("inf"),
        rms_theta_ss_deg=float(np.sqrt(np.mean(th[ss] ** 2))) if not fell else float("nan"),
        rms_x_ss_cm=float(np.sqrt(np.mean(X[ss, 0] ** 2)) * 100) if not fell else float("nan"),
        rms_du_ss=float(np.sqrt(np.mean(du[ss_du] ** 2))) if not fell else float("nan"),
        sat_ratio_ss=float(np.mean(lg.sat[ss])) if not fell else float("nan"),
        rms_thd_err=float(np.sqrt(np.mean(err[:, 3] ** 2))),
        rms_xd_err=float(np.sqrt(np.mean(err[:, 1] ** 2))),
    )


# ── 3. 개루프 추정 테스트 (같은 참 궤적을 모든 추정기에 입력) ────────────────
def openloop(spec, p, d, traj, seed=0):
    t, X, u = traj
    rng = np.random.default_rng(seed)
    est = make_estimator(spec, p, d, 0)      # 기록 궤적은 지연 0 으로 생성됨
    if isinstance(est, KalmanEstimator):
        est.predict = False
    Xh = np.zeros_like(X)
    for k in range(len(t)):
        enc, raw = state_to_raw(X[k, 0], X[k, 2], p)
        raw = int(np.clip(raw + np.round(rng.normal(0, SC["pot_noise"])), 0, 1023))
        m = Measurement(t[k], enc, raw)
        if k == 0:
            est.reset(m)
        Xh[k] = est.estimate(m)
        est.notify_command(u[k])
    e = Xh[:, 3] - X[:, 3]
    tr = t < 1.0
    ss = t >= T_SS
    # 등가지연: 과도구간에서 추정 θ̇ 와 참 θ̇ 의 상호상관 최대 지연
    a, b = X[tr, 3], Xh[tr, 3]
    lags = np.arange(0, 40)
    c = [np.dot(a[: len(a) - L], b[L:]) for L in lags]
    i = int(np.argmax(c))
    if 0 < i < len(c) - 1:
        i = i + 0.5 * (c[i - 1] - c[i + 1]) / (c[i - 1] - 2 * c[i] + c[i + 1])
    u_noise = np.std(((Xh - X)[ss] @ d.K.reshape(4)))
    return dict(thd_err_transient=float(np.sqrt(np.mean(e[tr] ** 2))),
                thd_noise_ss=float(np.sqrt(np.mean(e[ss] ** 2))),
                equiv_delay_ms=float(i * p.Ts * 1e3),
                u_noise_ss=float(u_noise)), Xh


def nanmean(v):
    v = np.asarray(v, dtype=float)
    return float(np.mean(v[np.isfinite(v)])) if np.any(np.isfinite(v)) else float("nan")


def load_baseline(name):
    a = np.loadtxt(os.path.join(BASELINE, name), delimiter=",", skiprows=1)
    return a


def main():
    os.makedirs(OUT, exist_ok=True)
    p = CartPoleParams()
    d = design_lqr(p)
    names = {s: spec_name(s) for s in SPECS + ROBUST_SPECS}

    # 1. 해석
    res, rows = analytic(p, d.K)
    sens = load_baseline("log_sensor_theta10.csv")         # baseline sensor (FD+LPF 30 Hz) 로그
    t, th_true, thd_true, th_meas, thd_hat, u = (sens[:, 0], sens[:, 3], sens[:, 4], sens[:, 7],
                                                 sens[:, 8], sens[:, 10])
    ss = t >= T_SS
    fd = np.r_[0, np.diff(th_meas) / p.Ts]
    meas = dict(fd_err_std=float(np.std((fd - thd_true)[ss])),
                lpf30_err_std=float(np.std((thd_hat - thd_true)[ss])),
                theta_meas_err_std_deg=float(np.degrees(np.std((th_meas - th_true)[ss]))),
                rms_du_ss=float(np.sqrt(np.mean(np.diff(u)[ss[1:]] ** 2))),
                u_std_ss=float(np.std(u[ss])))

    # 2. 폐루프
    jobs = [(s, th0, sd) for s in SPECS for th0 in THETA0 for sd in SEEDS]
    with ProcessPoolExecutor() as ex:
        results = list(ex.map(run_one, jobs, chunksize=4))
    logs = {(r[0], r[1]): r[4] for r in results if r[4] is not None}
    keys = list(results[0][3].keys())
    with open(os.path.join(OUT, "metrics_runs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["# " + BANNER])
        w.writerow(["estimator", "theta0_deg", "seed"] + keys)
        for s, th0, sd, m, _ in results:
            w.writerow([names[s], th0, sd] + [m[k] for k in keys])

    def agg(spec, th0s=THETA0):
        ms = [m for s, th0, sd, m, _ in results if s == spec and th0 in th0s]
        out = {"falls": sum(m["fell"] for m in ms), "n": len(ms)}
        for k in keys:
            if k == "fell":
                continue
            v = np.array([m[k] for m in ms], dtype=float)
            out[k] = float(np.max(v)) if k.startswith(("max", "settle")) else float(np.nanmean(v))
        return out

    # 3. 개루프
    sat = load_baseline("log_saturated_theta10.csv")
    traj = (sat[:, 0], sat[:, 1:5], sat[:, 10])
    ol, ol_x = {}, {}
    for s in SPECS:
        ol[s], ol_x[s] = openloop(s, p, d, traj)

    summ = {s: agg(s) for s in SPECS}
    summ10 = {s: agg(s, [10.0]) for s in SPECS}

    # 4. 모델 불일치 강건성
    rjobs = [(s, th0, sd, c) for c in MISMATCH for s in ROBUST_SPECS
             if not (c == "nominal" and s in SPECS) for th0 in THETA0 for sd in SEEDS]
    with ProcessPoolExecutor() as ex:
        rres = list(ex.map(run_one, rjobs, chunksize=4))
    robust = {}
    for c in MISMATCH:
        for s in ROBUST_SPECS:
            if c == "nominal" and s in SPECS:
                ms = [m for s2, th0, sd, m, _ in results if s2 == s]
            else:
                ms = [r[3] for r, j in zip(rres, rjobs) if j[0] == s and j[3] == c]
            ms10 = [m for m, j in zip(ms, [(th0,) for th0 in THETA0 for _ in SEEDS]) if j[0] == 10.0]
            robust[(s, c)] = dict(
                falls=sum(m["fell"] for m in ms), n=len(ms),
                rms_du_ss=nanmean([m["rms_du_ss"] for m in ms]),
                rms_theta_deg=float(np.mean([m["rms_theta_deg"] for m in ms])),
                max_x_cm=float(np.max([m["max_x_cm"] for m in ms])),
                sat_ratio=float(np.mean([m["sat_ratio"] for m in ms])),
                settle10=float(np.max([m["settle_theta_1deg_s"] for m in ms10])))
    # 강건 판정은 "참값 상태(ideal)로도 성공하는 케이스"에서만 한다.
    # ideal 도 실패하는 케이스는 추정기가 아니라 K/액추에이터 한계 문제 → Q/R 실험에서 다룸.
    ref = ("ideal", None)
    fair = [c for c in MISMATCH if robust[(ref, c)]["falls"] == 0]
    cand = [s for s in ROBUST_SPECS if s != ref]
    rok = [s for s in cand if all(robust[(s, c)]["falls"] == 0 and
                                  robust[(s, c)]["rms_theta_deg"] <= 1.5 * robust[(ref, c)]["rms_theta_deg"]
                                  for c in fair)]
    robust_best = min(rok, key=lambda s: max(robust[(s, c)]["rms_du_ss"] for c in fair)) if rok else None
    # 차선: 모든 케이스 총 넘어짐 최소 → 동률이면 최악 RMS Δu ss 최소
    fewest = min(cand, key=lambda s: (sum(robust[(s, c)]["falls"] for c in MISMATCH),
                                      max(robust[(s, c)]["rms_du_ss"] for c in fair)))

    # 선정 규칙 (사전 정의): 모든 run 에서 넘어짐 0, θ0=10° 정착시간 유한 → 정상상태 RMS Δu 최소
    ok = [s for s in SPECS if summ[s]["falls"] == 0 and np.isfinite(summ[s]["settle_theta_1deg_s"])]
    best = min(ok, key=lambda s: summ[s]["rms_du_ss"]) if ok else None
    fam_best = {}
    for fam in ("lpf", "reg", "kf"):
        c = [s for s in ok if s[0] == fam]
        fam_best[fam] = min(c, key=lambda s: summ[s]["rms_du_ss"]) if c else None

    write_report(p, d, res, rows, meas, names, summ, summ10, ol, best, fam_best, robust, robust_best, fewest)
    plot_noise(p, sens, res, meas)
    plot_tradeoff(names, summ, summ10, ol, best)
    plot_robust(names, robust)
    plot_timeseries(names, logs, ol_x, traj)
    print(open(os.path.join(OUT, "report.md"), encoding="utf-8").read())


# ── 출력 ────────────────────────────────────────────────────────────────────
def write_report(p, d, res, rows, meas, names, summ, summ10, ol, best, fam_best, robust, robust_best, fewest):
    L = [f"# Estimator study — sensor scenario\n", f"**{BANNER}**\n",
         f"고정: baseline LQR K = {np.array2string(d.K[0], precision=1)}, PWM ±255, pot 잡음 "
         f"{SC['pot_noise']} count, 1샘플 지연, Ts = {p.Ts * 1e3:g} ms, θ0 = {THETA0}°, seed = {SEEDS}\n",
         "## 1. 양자화 → θ̇ 잡음 증폭 (해석)\n",
         f"- pot 분해능 q = {res['q_deg']:.4f}°/count; 양자화 잡음 σ = q/√12 = {res['sigma_quant_deg']:.4f}°; "
         f"ADC 잡음 0.5 count 포함 σθ = {res['sigma_tot_deg']:.4f}°",
         f"- 1 count 변화의 후진차분: q/Ts = **{res['fd_one_count_step_radps']:.3f} rad/s "
         f"({np.degrees(res['fd_one_count_step_radps']):.0f}°/s)** → |K_θ̇|·q/Ts = "
         f"**{res['u_from_one_count_thd']:.0f} PWM**",
         f"- 백색잡음 가정 FD 표준편차 √2·σθ/Ts = {res['fd_std_tot_radps']:.3f} rad/s "
         f"(양자화만: {res['fd_std_quant_radps']:.3f}) → u 잡음 ≈ **{res['u_from_fd_thd']:.0f} PWM rms**"
         f" (θ 위치 잡음 자체는 |K_θ|·σθ = {res['u_from_theta_pos']:.1f} PWM)",
         f"- 엔코더 1 count 의 FD: {res['enc_fd_one_count_mps'] * 1e3:.1f} mm/s → |K_ẋ|·… = "
         f"{res['u_from_enc_one_count_xd']:.0f} PWM",
         "\n### baseline sensor 로그에서 실측 (θ0=10°, t ≥ 2 s)\n",
         f"- 필터 없는 FD θ̇ 오차 std = **{meas['fd_err_std']:.3f} rad/s** (해석 {res['fd_std_tot_radps']:.3f})",
         f"- FD+LPF 30 Hz θ̇ 오차 std = {meas['lpf30_err_std']:.3f} rad/s",
         f"- θ 측정 오차 std = {meas['theta_meas_err_std_deg']:.3f}°",
         f"- u std = {meas['u_std_ss']:.0f} PWM, RMS Δu = {meas['rms_du_ss']:.0f} PWM/step\n",
         "### 선형 필터의 잡음–지연 (해석, θ̇ 채널)\n",
         "| 필터 | 파라미터 | θ̇ 잡음 std [rad/s] | → u 잡음 [PWM] | 등가지연 [ms] |", "|---|---|---|---|---|"]
    for kind, par, s, un, dl in rows:
        L.append(f"| {'FD+LPF' if kind == 'lpf' else 'REG'} | {'fc=' + str(par) + ' Hz' if kind == 'lpf' else 'N=' + str(par)} "
                 f"| {s:.3f} | {un:.0f} | {dl * 1e3:.1f} |")

    L += ["\n## 2. 폐루프 비교 (θ0 = 2/5/10°, seed 5개 → 평균; max·settle 은 최악값)\n",
          "| estimator | falls | RMS θ° | max θ° | RMS x cm | max x cm | RMS u | RMS Δu | sat % | settle θ<1° s (θ0=10) | "
          "RMS Δu ss | sat % ss | RMS θ ss° |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for s in SPECS:
        m, m10 = summ[s], summ10[s]
        mark = " **←선정**" if s == best else ""
        L.append(f"| {names[s]}{mark} | {m['falls']}/{m['n']} | {m['rms_theta_deg']:.2f} | {m['max_theta_deg']:.1f} | "
                 f"{m['rms_x_cm']:.2f} | {m['max_x_cm']:.1f} | {m['rms_u']:.0f} | {m['rms_du']:.0f} | "
                 f"{m['sat_ratio'] * 100:.1f} | {m10['settle_theta_1deg_s']:.2f} | {m['rms_du_ss']:.0f} | "
                 f"{m['sat_ratio_ss'] * 100:.1f} | {m['rms_theta_ss_deg']:.2f} |")

    L += ["\n## 3. 개루프 추정 테스트 (baseline saturated θ0=10° 참 궤적을 양자화+잡음 후 입력)\n",
          "| estimator | 등가지연 ms (xcorr) | θ̇ 오차 과도 (t<1 s) | θ̇ 잡음 ss | u 잡음 ss [PWM] |", "|---|---|---|---|---|"]
    for s in SPECS:
        o = ol[s]
        L.append(f"| {names[s]} | {o['equiv_delay_ms']:.1f} | {o['thd_err_transient']:.3f} | "
                 f"{o['thd_noise_ss']:.3f} | {o['u_noise_ss']:.1f} |")
    L += ["\n## 4. 모델 불일치 강건성 (플랜트만 변경, 추정기·K 는 nominal 설계 그대로)\n",
          "셀 = falls/15 · RMS Δu ss [PWM/step] · RMS θ [deg] · θ0=10° settle(θ<1°) [s]\n",
          "| estimator | " + " | ".join(MISMATCH) + " |", "|---|" + "---|" * len(MISMATCH)]
    for s in ROBUST_SPECS:
        cells = []
        for c in MISMATCH:
            r = robust[(s, c)]
            cells.append(f"{r['falls']} · {r['rms_du_ss']:.0f} · {r['rms_theta_deg']:.2f} · {r['settle10']:.2f}")
        L.append(f"| {names[s]}{' **←강건 선정**' if s == robust_best else ''} | " + " | ".join(cells) + " |")

    L += ["\n## 5. 선정\n",
          "규칙(사전 정의):",
          "- nominal: 15 run 넘어짐 0, θ0=10° 정착시간 유한 → 정상상태 RMS Δu 최소",
          "- 강건: 참값 상태(ideal)로 성공하는 불일치 케이스 전부에서 넘어짐 0 이고 "
          "RMS θ ≤ 1.5 × ideal 의 RMS θ → 그 케이스들 중 최악 RMS Δu ss 최소\n",
          f"- nominal 최적: **{names[best] if best else '없음'}**",
          f"- 계열별 nominal 최적: LPF = {names.get(fam_best['lpf'], '-')}, REG = {names.get(fam_best['reg'], '-')}, "
          f"KF = {names.get(fam_best['kf'], '-')}",
          f"- 모델 불일치 포함 강건 최적: **{names[robust_best] if robust_best else '없음 (규칙을 만족하는 추정기 없음)'}**",
          f"- 차선 (전체 케이스 넘어짐 합 최소 → 최악 RMS Δu ss 최소): **{names[fewest]}** "
          f"(넘어짐 합 {sum(robust[(fewest, c)]['falls'] for c in MISMATCH)}/{15 * len(MISMATCH)})",
          f"\n**{BANNER}**"]
    with open(os.path.join(OUT, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


def plot_noise(p, sens, res, meas):
    t, thd_true, th_meas, thd_hat = sens[:, 0], sens[:, 4], sens[:, 7], sens[:, 8]
    fd = np.r_[0, np.diff(th_meas) / p.Ts]
    w = (t >= 2.0) & (t <= 2.6)
    fig, ax = plt.subplots(2, 1, figsize=(10, 7))
    ax[0].step(t[w], fd[w], where="post", lw=0.8, label="raw finite difference")
    ax[0].plot(t[w], thd_hat[w], lw=1.5, label="FD + LPF 30 Hz (baseline)")
    ax[0].plot(t[w], thd_true[w], "k", lw=2, label="true dθ/dt")
    ax[0].set_ylabel("dθ/dt [rad/s]")
    ax[0].set_xlabel("t [s]")
    ax[0].legend(fontsize=8)
    ax[0].set_title(f"one pot count = {res['q_deg']:.3f}° → FD step = {res['fd_one_count_step_radps']:.2f} rad/s "
                    f"→ {res['u_from_one_count_thd']:.0f} PWM via K_θ̇", fontsize=9)
    ax[1].hist((fd - thd_true)[t >= 2], bins=41, alpha=0.6, label=f"raw FD err, std={meas['fd_err_std']:.2f}")
    ax[1].hist((thd_hat - thd_true)[t >= 2], bins=41, alpha=0.6,
               label=f"LPF30 err, std={meas['lpf30_err_std']:.2f}")
    ax[1].set_xlabel("dθ/dt estimation error [rad/s]  (baseline sensor log, θ0=10°, t≥2 s)")
    ax[1].legend(fontsize=8)
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(f"Noise amplification of finite difference\n{BANNER}", fontsize=10, color="darkred")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "noise_amplification.png"), dpi=110)
    plt.close(fig)


def plot_tradeoff(names, summ, summ10, ol, best):
    fig, ax = plt.subplots(2, 3, figsize=(15, 8))
    ax = ax.ravel()
    panels = [("rms_du_ss", "RMS Δu steady [PWM/step]", summ),
              ("sat_ratio", "saturation ratio", summ),
              ("rms_theta_deg", "RMS θ [deg] (whole run)", summ),
              ("rms_x_cm", "RMS x [cm]", summ),
              ("settle_theta_1deg_s", "settle θ<1° [s] (θ0=10, worst)", summ10)]
    styles = {"lpf": ("tab:blue", "o-", "FD+LPF (fc sweep)"), "reg": ("tab:orange", "s-", "regression (N sweep)"),
              "kf": ("tab:green", "D", "Kalman")}
    for i, (k, lab, src) in enumerate(panels):
        for fam, (c, st, fl) in styles.items():
            ss = [s for s in SPECS if s[0] == fam]
            xs = [ol[s]["equiv_delay_ms"] for s in ss]
            ys = [src[s][k] if np.isfinite(src[s][k]) else np.nan for s in ss]
            ax[i].plot(xs, ys, st, color=c, label=fl, ms=5, lw=1 if fam != "kf" else 0)
            for s, x_, y_ in zip(ss, xs, ys):
                if fam == "kf" or s[1] in (30, 10, 3, 5, 16, 32):
                    ax[i].annotate(names[s].replace("FD+LPF ", "").replace("REG ", "")
                                   .replace("KF σa=", "KF"), (x_, y_), fontsize=6)
            fell = [(ol[s]["equiv_delay_ms"]) for s in ss if summ[s]["falls"] > 0]
            if fell and i == 0:
                ax[i].plot(fell, [0] * len(fell), "x", color="r", ms=8)
        ax[i].set_xlabel("equivalent estimator delay [ms] (open-loop xcorr)")
        ax[i].set_ylabel(lab)
        ax[i].grid(alpha=0.3)
    ax[0].legend(fontsize=8)
    b = best
    ax[5].axis("off")
    ax[5].text(0, 0.9, "Trade-off: more filtering → smoother u\nbut more phase delay → larger θ/x excursions,\n"
               "slower settling, eventually falls (red x).\n\n"
               f"nominal best: {names[b] if b else '-'}\n"
               "(but see robustness.png: low-noise KF is fragile\n to model mismatch)", fontsize=10, va="top")
    fig.suptitle(f"Estimator trade-off (closed loop, θ0=2/5/10°, 5 seeds, same K)\n{BANNER}",
                 fontsize=10, color="darkred")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "tradeoff_closedloop.png"), dpi=110)
    plt.close(fig)

    fig, a = plt.subplots(figsize=(8, 6))
    for fam, (c, st, fl) in styles.items():
        ss = [s for s in SPECS if s[0] == fam]
        a.plot([ol[s]["thd_noise_ss"] for s in ss], [ol[s]["thd_err_transient"] for s in ss], st,
               color=c, label=fl, lw=1 if fam != "kf" else 0)
        for s in ss:
            a.annotate(names[s], (ol[s]["thd_noise_ss"], ol[s]["thd_err_transient"]), fontsize=6)
    a.set_xlabel("dθ/dt noise at rest (RMS error, t≥2 s) [rad/s]")
    a.set_ylabel("dθ/dt error during transient (RMS, t<1 s) [rad/s]  ← delay dominated")
    a.set_xscale("log")
    a.grid(alpha=0.3, which="both")
    a.legend()
    a.set_title(f"Open-loop estimator test (same true trajectory)\n{BANNER}", fontsize=9, color="darkred")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "tradeoff_openloop.png"), dpi=110)
    plt.close(fig)


def plot_robust(names, robust):
    cases = list(MISMATCH)
    fig, ax = plt.subplots(1, 3, figsize=(16, 5.5))
    w = 0.8 / len(ROBUST_SPECS)
    for i, s in enumerate(ROBUST_SPECS):
        xs = np.arange(len(cases)) + (i - len(ROBUST_SPECS) / 2) * w
        for a, k in zip(ax, ("rms_du_ss", "rms_theta_deg", "falls")):
            ys = [robust[(s, c)][k] for c in cases]
            a.bar(xs, np.nan_to_num(ys, nan=0), w, label=names[s])
    for a, lb in zip(ax, ("RMS Δu steady [PWM/step]", "RMS θ [deg]", "falls (of 15)")):
        a.set_xticks(np.arange(len(cases)))
        a.set_xticklabels(cases, rotation=25, ha="right", fontsize=7)
        a.set_ylabel(lb)
        a.grid(alpha=0.3, axis="y")
    ax[0].legend(fontsize=7)
    fig.suptitle(f"Robustness to plant/model mismatch (estimator & K designed on nominal)\n{BANNER}",
                 fontsize=10, color="darkred")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "robustness.png"), dpi=110)
    plt.close(fig)


def plot_timeseries(names, logs, ol_x, traj):
    sel = [("lpf", 30), ("lpf", 5), ("reg", 8), ("kf", (0.5, 2.0, False)), ("kf", (3.0, 20.0, False))]
    fig, ax = plt.subplots(4, 1, figsize=(11, 11))
    for a in ax[1:3]:
        a.sharex(ax[0])
    for s in sel:
        lg = logs[(s, 10.0)]
        ax[0].plot(lg.t, np.degrees(lg.X[:, 2]), label=names[s])
        ax[1].plot(lg.t, lg.X[:, 0] * 100)
        ax[2].plot(lg.t, lg.u_cmd, lw=0.8)
    t, X, _ = traj
    ax[3].plot(t, X[:, 3], "k", lw=2, label="true (open-loop test)")
    for s in sel:
        ax[3].plot(t, ol_x[s][:, 3], lw=0.8, label=names[s])
    ax[3].set_xlim(0, 1.5)
    for a, lb in zip(ax, ["θ [deg]", "x [cm]", "u_cmd [PWM]", "dθ/dt est [rad/s]"]):
        a.set_ylabel(lb)
        a.grid(alpha=0.3)
    ax[2].axhline(255, color="r", ls=":")
    ax[2].axhline(-255, color="r", ls=":")
    ax[0].legend(fontsize=8)
    ax[3].legend(fontsize=7)
    ax[3].set_xlabel("t [s]  (top 3: closed loop θ0=10°, seed 0;  bottom: open-loop estimate, zoom 0–1.5 s)")
    fig.suptitle(f"Baseline (FD+LPF 30Hz) vs candidate estimators\n{BANNER}", fontsize=10, color="darkred")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "timeseries_theta10.png"), dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
