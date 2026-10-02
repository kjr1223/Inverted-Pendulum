#!/usr/bin/env python3
"""LQR 설계 + 비선형 cart-pole 시뮬레이션 + 수치 검증 + 그래프 저장.

    python3 simulation/run_lqr.py                       # 기본: θ0=2,5,10°, 시나리오 3종
    python3 simulation/run_lqr.py --theta0 2 5 10 15
    python3 simulation/run_lqr.py --bryson 0.1,0.5,5,1.0,255     # x,xd,θ[deg],θd,u 허용 최대값
    python3 simulation/run_lqr.py --q 100,4,1300,1 --r 1.5e-5    # Q 대각, R 직접 지정

※ 모든 결과는 ASSUMED 파라미터 기반 시뮬레이션이며 실제 하드웨어로 검증되지 않았다.
"""
from __future__ import annotations

import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from cartpole_model import CartPoleParams, numeric_jacobian
from closed_loop import DEFAULT_BRYSON, SCENARIOS, build, design_lqr, evaluate, run_closed_loop
from estimator import SensorEstimator
from lqr_controller import STATE_NAMES, bryson, controllability

BANNER = "SIMULATION ONLY - ASSUMED PARAMETERS - NOT VALIDATED ON HARDWARE"


def fmt(M):
    return np.array2string(np.atleast_2d(M), precision=5, suppress_small=False,
                           max_line_width=140, floatmode="maxprec")


def print_design(p, d):
    print("=" * 78)
    print(BANNER)
    print("=" * 78)
    print("\n[파라미터]  (CODE=기존 Arduino 코드 값·미검증, ASSUMED=가정값)")
    for name, val, tag, note in p.table():
        print(f"  {name:13s} = {val:<10.6g} {tag:8s} {note}")
    print(f"  (파생) l_c={p.l_c:.3f} m, J_p={p.J_p:.3e} kg·m², K_v={p.K_v:.3e} m/s/PWM, "
          f"k_f=M_eq/τ={p.k_f:.1f} N·s/m, ω0={p.omega0:.3f} rad/s")

    print("\n[연속시간 선형화]  Ẋ = A X + B u,  X=[x, ẋ, θ, θ̇], u=PWM")
    print("A =\n" + fmt(d.A))
    print("B =\n" + fmt(d.B))
    An, Bn = numeric_jacobian(p)
    print(f"  해석식 vs 유한차분 Jacobian 최대오차: A {np.max(np.abs(An - d.A)):.2e}, "
          f"B {np.max(np.abs(Bn - d.B)):.2e}")
    ev = np.linalg.eigvals(d.A)
    print("  eig(A) =", np.array2string(np.sort_complex(ev), precision=4),
          " ← 양의 실근 = upright 불안정극")

    rank, _, sv = controllability(d.A, d.B)
    print(f"\n[Controllability]  rank[B AB A²B A³B] = {rank} / 4 → "
          f"{'가제어' if rank == 4 else '불가제어'}")
    print("  singular values =", np.array2string(sv, precision=4))
    rank_d, _, _ = controllability(d.Ad, d.Bd)
    print(f"  이산모델 rank = {rank_d} / 4")

    print(f"\n[이산화]  ZOH, Ts = {p.Ts * 1e3:.1f} ms")
    print("Ad =\n" + fmt(d.Ad))
    print("Bd =\n" + fmt(d.Bd))

    print("\n[가중치]  J = Σ Xᵀ Q X + uᵀ R u")
    print("Q = diag(" + ", ".join(f"{q:.4g}" for q in np.diag(d.Q)) + ")")
    print(f"R = {d.R[0, 0]:.4g}")
    print("\n[LQR 게인]  u[PWM] = −K X")
    for n, k in zip(STATE_NAMES, d.K[0]):
        print(f"  K[{n:17s}] = {k:12.4f}")
    cl = np.linalg.eigvals(d.Ad - d.Bd @ d.K)
    print("  폐루프 이산 고유값 |z| =", np.array2string(np.abs(cl), precision=4),
          "(모두 < 1 이면 안정)")
    print("  등가 연속 극 s = ln(z)/Ts =",
          np.array2string(np.log(cl.astype(complex)) / p.Ts, precision=3))
    print("  감도 참고: θ 1° → u = %.1f PWM,  pot 1 count(%.3f°) → %.1f PWM" % (
        abs(d.K[0, 2]) * np.radians(1), 180 / (p.pot_up - p.pot_down),
        abs(d.K[0, 2]) * np.radians(180 / (p.pot_up - p.pot_down))))


def plot_scenario(logs, p, scenario, path):
    fig, ax = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
    for th0, lg in logs.items():
        ax[0].plot(lg.t, lg.X[:, 0] * 100, label=f"θ0={th0}°")
        ax[1].plot(lg.t, lg.X[:, 1])
        ax[2].plot(lg.t, np.degrees(lg.X[:, 2]))
        ax[3].plot(lg.t, lg.X[:, 3])
        line, = ax[4].plot(lg.t, lg.u_cmd)
        if SCENARIOS[scenario]["u_limit"] is not None:
            ax[4].plot(lg.t, lg.u_raw, "--", color=line.get_color(), alpha=0.4, lw=0.8)
    ax[0].axhline(p.x_limit * 100, color="r", ls=":", lw=1)
    ax[0].axhline(-p.x_limit * 100, color="r", ls=":", lw=1, label="rail (ASSUMED)")
    for lim in (255, -255):
        ax[4].axhline(lim, color="r", ls=":", lw=1)
    labels = ["x [cm]", "dx/dt [m/s]", "θ [deg]", "dθ/dt [rad/s]", "u [PWM]\n(dashed: before sat.)"]
    for a, lb in zip(ax, labels):
        a.set_ylabel(lb)
        a.grid(alpha=0.3)
    ax[0].legend(loc="upper right", fontsize=8)
    ax[-1].set_xlabel("t [s]")
    fig.suptitle(f"LQR – scenario '{scenario}'\n{BANNER}", fontsize=10, color="darkred")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_compare(all_logs, th0, path):
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for sc, logs in all_logs.items():
        lg = logs[th0]
        ax[0].plot(lg.t, np.degrees(lg.X[:, 2]), label=sc)
        ax[1].plot(lg.t, lg.X[:, 0] * 100)
        ax[2].plot(lg.t, lg.u_cmd)
    for lim in (255, -255):
        ax[2].axhline(lim, color="r", ls=":", lw=1)
    for a, lb in zip(ax, ["θ [deg]", "x [cm]", "u_cmd [PWM]"]):
        a.set_ylabel(lb)
        a.grid(alpha=0.3)
    ax[0].legend()
    ax[-1].set_xlabel("t [s]")
    fig.suptitle(f"θ0 = {th0}° : ideal vs saturated vs sensor\n{BANNER}", fontsize=10, color="darkred")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--theta0", type=float, nargs="+", default=[2.0, 5.0, 10.0], help="초기 θ [deg]")
    ap.add_argument("--t-final", type=float, default=5.0)
    ap.add_argument("--scenarios", nargs="+", default=list(SCENARIOS), choices=list(SCENARIOS))
    ap.add_argument("--bryson", type=str, help="x_max,xd_max,th_max_deg,thd_max,u_max")
    ap.add_argument("--q", type=str, help="Q 대각 4개 (쉼표)")
    ap.add_argument("--r", type=float, help="R 스칼라")
    ap.add_argument("--fc", type=float, default=30.0, help="sensor 시나리오 속도필터 차단주파수 [Hz]")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
    ap.add_argument("--overwrite", action="store_true", help="기존 결과 폴더 덮어쓰기 허용")
    args = ap.parse_args()

    p = CartPoleParams()
    Q = R = None
    if args.bryson:
        xm, xdm, thm, thdm, um = map(float, args.bryson.split(","))
        Q, R = bryson(xm, xdm, np.radians(thm), thdm, um)
    if args.q:
        Q = np.diag(list(map(float, args.q.split(","))))
    if args.r is not None:
        R = np.array([[args.r]])
    d = design_lqr(p, Q, R)
    print_design(p, d)

    if os.path.exists(os.path.join(args.out, "summary.json")) and not args.overwrite:
        raise SystemExit(f"\n{args.out}/summary.json 이 이미 있습니다. 기존 결과 보존을 위해 중단합니다. "
                         f"--out 으로 다른 폴더를 지정하거나 --overwrite 를 주세요.")
    os.makedirs(args.out, exist_ok=True)
    all_logs, summary = {}, {}
    print("\n[비선형 시뮬레이션 결과]  (검증 기준: 마지막 1초 동안 |θ|,|x| 가 허용오차 이내, "
          "발산 없음, 레일 이내)")
    hdr = (f"  {'scenario':10s} {'θ0':>4s} {'PASS':>5s} {'fell':>5s} {'|θ|max°':>8s} "
           f"{'|x|max cm':>9s} {'θ_fin°':>7s} {'x_fin mm':>8s} {'t_θ<.5°':>7s} "
           f"{'|u|raw max':>10s} {'sat%':>6s}")
    print(hdr)
    for sc in args.scenarios:
        all_logs[sc] = {}
        for th0 in args.theta0:
            plant, est, ctrl, delay = build(p, d, sc)
            if isinstance(est, SensorEstimator):
                est.fc_xd = est.fc_thd = args.fc
            lg = run_closed_loop(plant, est, ctrl, [0, 0, np.radians(th0), 0], args.t_final, delay)
            m = evaluate(lg, p, sc)
            all_logs[sc][th0] = lg
            summary[f"{sc}_theta{th0:g}"] = m
            lg.save_csv(os.path.join(args.out, f"log_{sc}_theta{th0:g}.csv"))
            print(f"  {sc:10s} {th0:4.0f} {str(m['PASS']):>5s} {str(m['fell']):>5s} "
                  f"{m['theta_max_deg']:8.2f} {m['x_max_m'] * 100:9.2f} {m['theta_final_deg']:7.3f} "
                  f"{m['x_final_m'] * 1e3:8.2f} {m['theta_settle_s(0.5deg)']:7.2f} "
                  f"{m['u_raw_max']:10.0f} {m['sat_fraction'] * 100:6.1f}")
        plot_scenario(all_logs[sc], p, sc, os.path.join(args.out, f"states_{sc}.png"))
    if len(args.scenarios) > 1:
        for th0 in args.theta0:
            plot_compare(all_logs, th0, os.path.join(args.out, f"compare_theta{th0:g}.png"))

    with open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({
            "WARNING": BANNER,
            "params": {n: {"value": v, "source": s, "note": note} for n, v, s, note in p.table()},
            "A": d.A.tolist(), "B": d.B.tolist(), "Ad": d.Ad.tolist(), "Bd": d.Bd.tolist(),
            "Q": d.Q.tolist(), "R": d.R.tolist(), "K": d.K.tolist(),
            "bryson_default": {k: float(v) for k, v in DEFAULT_BRYSON.items()},
            "results": summary,
        }, f, indent=2, ensure_ascii=False)
    print(f"\n결과 저장: {args.out}/ (states_*.png, compare_*.png, log_*.csv, summary.json)")
    print(BANNER)


if __name__ == "__main__":
    main()
