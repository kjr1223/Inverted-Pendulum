#!/usr/bin/env python3
"""Inverted Pendulum - LQR Simulation (실시간 시연용 GUI).

    python3 simulation/visualizer.py              # 옵션 없이 바로 시연 (θ0=10°, PWM ±255, 1× 실시간, 자동 반복)

키:  R 리셋 · Space 일시정지/재개 · ← / → 외란 · Esc / Q 종료

기타 옵션:
    --theta0 5            초기 각도 [deg]
    --scenario sensor     추정기/센서 시나리오 (ideal | saturated | sensor)
    --speed 0.5           재생 배속
    --no-loop             자동 반복 끄기
    --save demo.mp4       같은 프레임 함수로 영상 저장 (화면 불필요)
    --selftest 12         GUI 를 N 초 실행 후 자동 종료하며 루프 동작/실시간성 통계 출력 (키 입력도 자동 주입)

구조: 각 track 은 (Plant, Estimator, Controller) 한 벌을 시간에 맞춰 진행하는 LiveSession.
향후 SerialHardwarePlant 로 만든 LiveSession 을 추가하면 실제 장비와 시뮬레이션을 같은 화면에 표시할 수 있다.
LQR 설계(A, B, Q, R, K)는 closed_loop.design_lqr() 의 baseline 을 그대로 사용한다.
"""
from __future__ import annotations

import argparse
import time
from collections import deque

import numpy as np

from cartpole_model import CartPoleParams
from closed_loop import SCENARIOS, build, design_lqr

WINDOW_TITLE = "Inverted Pendulum - LQR Simulation"
FOOTER = "Simulation only — assumed plant parameters, not hardware validated"

# 시연 사이클
SHOW_S = 5.0            # 안정화 과정 표시
HOLD_S = 2.0            # 안정화 상태 유지 후 자동 리셋
MAX_CYCLE_S = 20.0      # 외란 등으로 안정화가 늦어져도 이 시간이 지나면 리셋
FELL_RESET_S = 1.5      # 넘어지면 이만큼 보여준 뒤 리셋
# STABILIZED 판정
STAB_THETA_DEG = 1.0
STAB_THD = 0.2          # rad/s
STAB_X = 0.02           # m
STAB_HOLD = 0.3         # s 연속 만족
HISTORY_S = 3.0


class LiveSession:
    """Plant/Estimator/Controller 를 목표 시각까지 제어주기(Ts) 단위로 진행."""

    def __init__(self, plant, estimator, controller, X0, delay_steps=0,
                 fall_angle=np.radians(60)):
        self.plant, self.est, self.ctrl = plant, estimator, controller
        self.X0 = np.asarray(X0, dtype=float)
        self.delay_steps = delay_steps
        self.fall_angle = fall_angle
        self.history = deque(maxlen=int(HISTORY_S / plant.Ts) + 10)
        self.reset()

    def reset(self):
        self.meas = self.plant.reset(self.X0)
        self.est.reset(self.meas)
        self.pending = deque([0.0] * self.delay_steps)
        self.u_cmd, self.sat, self.fell = 0.0, False, False
        self.t_fell = None
        self.stable_since = None
        self.X_hat = self.est.estimate(self.meas)
        self.history.clear()
        self.history.append((0.0, np.degrees(self.X[2]), 0.0))

    @property
    def t(self):
        return self.meas.t

    @property
    def X(self):
        return self.meas.true_state if self.meas.true_state is not None else self.X_hat

    def step(self):
        self.X_hat = self.est.estimate(self.meas)
        out = self.ctrl.compute(self.X_hat)
        self.est.notify_command(out.u_cmd)
        self.pending.append(out.u_cmd)
        self.u_cmd = self.pending.popleft()
        self.sat = out.saturated
        self.plant.apply(self.u_cmd)
        self.meas = self.plant.advance()
        X = self.X
        self.history.append((self.t, np.degrees(X[2]), self.u_cmd))
        stable = (abs(np.degrees(X[2])) < STAB_THETA_DEG and abs(X[3]) < STAB_THD and abs(X[0]) < STAB_X)
        if stable:
            if self.stable_since is None:
                self.stable_since = self.t
        else:
            self.stable_since = None
        if abs(X[2]) > self.fall_angle:
            self.fell, self.t_fell = True, self.t
            self.plant.apply(0.0)
            self.u_cmd = 0.0

    def run_until(self, t_target: float, max_steps: int = 400):
        n = 0
        while not self.fell and self.meas.t < t_target - 1e-9 and n < max_steps:
            self.step()
            n += 1

    @property
    def stabilized(self):
        return self.stable_since is not None and self.t - self.stable_since >= STAB_HOLD

    def stable_duration(self):
        return 0.0 if self.stable_since is None else self.t - self.stable_since


class DemoView:
    """중앙: 레일 위 cart + 회전하는 pendulum.  우측: 최근 3초 θ(t), PWM(t)."""

    def __init__(self, p: CartPoleParams, theta0: float, K):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
        self.p = p
        self.fig = plt.figure(figsize=(14, 7.5), facecolor="white")
        try:
            self.fig.canvas.manager.set_window_title(WINDOW_TITLE)
        except AttributeError:
            pass
        gs = self.fig.add_gridspec(2, 2, width_ratios=[2.1, 1], left=0.04, right=0.97,
                                   top=0.93, bottom=0.1, wspace=0.18, hspace=0.35)
        ax = self.ax = self.fig.add_subplot(gs[:, 0])
        self.ax_th = self.fig.add_subplot(gs[0, 1])
        self.ax_u = self.fig.add_subplot(gs[1, 1])
        self.fig.suptitle(WINDOW_TITLE, fontsize=16, fontweight="bold")

        # ── 장면 ──
        half = p.x_limit + 0.12
        ax.set_xlim(-half, half)
        ax.set_ylim(-0.13, p.L_p + 0.14)
        ax.set_aspect("equal")
        ax.set_yticks([])
        ax.set_xlabel("cart position x [m]", fontsize=10)
        for s in ("top", "right", "left"):
            ax.spines[s].set_visible(False)
        ax.add_patch(Rectangle((-half, -0.052), 2 * half, 0.012, fc="0.35", ec="none"))     # 레일
        for s in (-1, 1):                                                                    # 엔드스톱
            ax.add_patch(Rectangle((s * p.x_limit - 0.006, -0.052), 0.012, 0.05, fc="firebrick"))
        ax.text(p.x_limit, -0.1, "rail limit (assumed)", color="firebrick", fontsize=7, ha="center")
        ax.axvline(0, color="0.8", lw=1, ls=":", zorder=0)
        self.ref, = ax.plot([], [], color="0.75", lw=1.5, ls="--", zorder=1)                 # upright 목표선

        self.cw, self.ch = 0.11, 0.05
        self.cart = FancyBboxPatch((0, 0), self.cw, self.ch, boxstyle="round,pad=0,rounding_size=0.008",
                                   fc="#2f6db5", ec="#1b3f6b", lw=2, zorder=3)
        ax.add_patch(self.cart)
        self.wheels = [Circle((0, 0), 0.011, fc="0.15", zorder=4) for _ in range(2)]
        for w in self.wheels:
            ax.add_patch(w)
        self.pole, = ax.plot([], [], color="#d9822b", lw=7, solid_capstyle="round", zorder=5)
        self.bob = Circle((0, 0), 0.02, fc="#b35a00", ec="k", zorder=6)
        ax.add_patch(self.bob)
        self.pivot = Circle((0, 0), 0.009, fc="white", ec="k", zorder=7)
        ax.add_patch(self.pivot)

        self.info = ax.text(0.015, 0.975, "", transform=ax.transAxes, va="top", ha="left",
                            family="monospace", fontsize=15,
                            bbox=dict(boxstyle="round", fc="white", ec="0.6", alpha=0.92), zorder=10)
        self.status = ax.text(0.985, 0.975, "", transform=ax.transAxes, va="top", ha="right",
                              fontsize=20, fontweight="bold", zorder=10,
                              bbox=dict(boxstyle="round", fc="white", ec="0.6"))
        self.sub = ax.text(0.985, 0.84, "", transform=ax.transAxes, va="top", ha="right",
                           fontsize=10, color="0.35", zorder=10)
        ax.text(0.015, 0.02, f"x0 = 0 m, θ0 = {theta0:g}°   ·   discrete LQR, Ts = {p.Ts * 1e3:g} ms"
                f"   ·   PWM limit ±{p.pwm_max:g}   ·   nonlinear plant\n"
                f"K = [{', '.join(f'{k:.0f}' for k in np.ravel(K))}]", transform=ax.transAxes,
                fontsize=8, color="0.4", va="bottom")

        # ── 히스토리 ──
        for a, lab, lim in ((self.ax_th, "θ [deg]", 12.5), (self.ax_u, "PWM", 300)):
            a.set_xlim(-HISTORY_S, 0)
            a.set_ylim(-lim, lim)
            a.set_ylabel(lab)
            a.grid(alpha=0.3)
            a.axhline(0, color="0.5", lw=0.8)
        self.ax_th.set_title("θ(t) — last 3 s", fontsize=11)
        self.ax_u.set_title("PWM(t) — last 3 s", fontsize=11)
        self.ax_u.set_xlabel("time relative to now [s]")
        for s in (1, -1):
            self.ax_u.axhline(s * p.pwm_max, color="r", ls="--", lw=1)
            self.ax_th.axhline(s * STAB_THETA_DEG, color="green", ls=":", lw=1)
        self.ax_u.text(-HISTORY_S + 0.05, p.pwm_max + 8, "saturation ±255", color="r", fontsize=8)
        self.l_th, = self.ax_th.plot([], [], color="#d9822b", lw=2)
        self.l_u, = self.ax_u.plot([], [], color="#2f6db5", lw=1.5)

        self.fig.text(0.5, 0.012, FOOTER, ha="center", fontsize=9, color="0.45", style="italic")
        self.fig.text(0.99, 0.012, "R reset · Space pause · ←/→ disturb · Q/Esc quit",
                      ha="right", fontsize=8, color="0.55")

        self.dynamic = [self.ref, self.cart, *self.wheels, self.pole, self.bob, self.pivot,
                        self.info, self.status, self.sub, self.l_th, self.l_u]
        for a in self.dynamic:
            a.set_animated(True)

    def draw_state(self, s: LiveSession, paused: bool, auto_msg: str):
        p, X = self.p, s.X
        x, th = X[0], X[2]
        self.cart.set_x(x - self.cw / 2)
        self.cart.set_y(-0.04)
        for w, dx in zip(self.wheels, (-0.032, 0.032)):
            w.center = (x + dx, -0.04)
        py = -0.04 + self.ch
        tip = (x + p.L_p * np.sin(th), py + p.L_p * np.cos(th))
        self.pole.set_data([x, tip[0]], [py, tip[1]])
        self.bob.center = tip
        self.pivot.center = (x, py)
        self.ref.set_data([x, x], [py, py + p.L_p + 0.03])

        self.info.set_text(f"Controller: LQR\n"
                           f"θ    = {np.degrees(th):+6.2f} deg\n"
                           f"x    = {x:+6.3f} m\n"
                           f"PWM  = {s.u_cmd:+4.0f}\n"
                           f"Time = {s.t:5.2f} s")
        if paused:
            txt, col = "PAUSED", "0.3"
        elif s.fell:
            txt, col = "FELL", "firebrick"
        elif s.stabilized:
            txt, col = "STABILIZED", "green"
        else:
            txt, col = "STABILIZING…", "darkorange"
        self.status.set_text(txt)
        self.status.set_color(col)
        self.status.get_bbox_patch().set_edgecolor(col)
        self.sub.set_text(("PWM SATURATED  ·  " if s.sat and not s.fell else "") + auto_msg)

        if s.history:
            h = np.array(s.history)
            self.l_th.set_data(h[:, 0] - s.t, h[:, 1])
            self.l_u.set_data(h[:, 0] - s.t, h[:, 2])
        return self.dynamic


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--theta0", type=float, default=10.0, help="초기 θ [deg] (기본 10)")
    ap.add_argument("--scenario", default="saturated", choices=list(SCENARIOS),
                    help="기본 saturated = 참값 상태 + PWM ±255")
    ap.add_argument("--speed", type=float, default=1.0, help="재생 배속 (기본 1.0 = 실시간)")
    ap.add_argument("--kick", type=float, default=0.8, help="←/→ 외란: 진자 각속도 변화 [rad/s]")
    ap.add_argument("--no-loop", action="store_true", help="자동 리셋 반복 끄기")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--save", type=str, help="mp4/gif 저장 (화면 표시 없음)")
    ap.add_argument("--duration", type=float, default=10.0, help="--save 길이 [s]")
    ap.add_argument("--selftest", type=float, default=0.0, help="N 초 GUI 실행 후 자동 종료 + 통계")
    ap.add_argument("--screenshot", type=str, help="--selftest 중 창 화면 캡처 저장 경로")
    args = ap.parse_args()

    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    for k in list(matplotlib.rcParams):            # 기본 단축키(R, ←/→, L, K 등) 충돌 제거
        if k.startswith("keymap."):
            matplotlib.rcParams[k] = []
    matplotlib.rcParams["toolbar"] = "None"
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    p = CartPoleParams()
    d = design_lqr(p)                              # baseline LQR (A, B, Q, R, K 변경 없음)
    plant, est, ctrl, delay = build(p, d, args.scenario)
    X0 = [0.0, 0.0, np.radians(args.theta0), 0.0]
    sess = LiveSession(plant, est, ctrl, X0, delay)
    # 향후: hw = LiveSession(SerialHardwarePlant(...), SensorEstimator(p), ctrl, X0) 를 같은 화면에 추가

    view = DemoView(p, args.theta0, d.K)
    st = dict(t=0.0, last=time.perf_counter(), paused=False, resets=0, frames=0,
              frame_ms=[], wall0=time.perf_counter(), sim_total=0.0, keys=[])

    def do_reset():
        st["sim_total"] += sess.t
        sess.reset()
        st["t"] = 0.0
        st["resets"] += 1

    def advance_clock(frame):
        if args.save:                                   # 영상: 프레임당 1/fps 초씩 누적 (리셋 시 0 으로)
            if not st["paused"]:
                st["t"] += args.speed / args.fps
            return st["t"]
        now = time.perf_counter()
        if not st["paused"]:
            st["t"] += min(now - st["last"], 0.1) * args.speed   # 창 드래그 등으로 멈춘 시간은 건너뜀
        st["last"] = now
        return st["t"]

    def auto_cycle():
        """자동 반복: 5초 보여주고 2초 유지 → 리셋. 남은 시간 메시지 반환."""
        if args.no_loop:
            return ""
        if sess.fell:
            if sess.t - sess.t_fell >= FELL_RESET_S or st["t"] - sess.t_fell >= FELL_RESET_S:
                do_reset()
            return "auto reset…"
        if sess.t >= SHOW_S + HOLD_S and (sess.stable_duration() >= HOLD_S or sess.t >= MAX_CYCLE_S):
            do_reset()
            return ""
        if sess.t >= SHOW_S and sess.stabilized:
            remain = max(SHOW_S + HOLD_S - sess.t, HOLD_S - sess.stable_duration(), 0.0)
            return f"auto reset in {remain:.1f} s"
        return ""

    def on_key(ev):
        k = (ev.key or "").lower()
        st["keys"].append(k)
        if k == "r":
            do_reset()
        elif k == " ":
            st["paused"] = not st["paused"]
        elif k in ("left", "right"):
            if hasattr(sess.plant, "disturb") and not sess.fell:
                sess.plant.disturb(args.kick if k == "right" else -args.kick)
        elif k in ("escape", "q"):
            plt.close(view.fig)

    view.fig.canvas.mpl_connect("key_press_event", on_key)

    def frame_fn(frame):
        t0 = time.perf_counter()
        t = advance_clock(frame)
        sess.run_until(t)
        if sess.fell:
            st["t"] = min(st["t"], sess.t + FELL_RESET_S)
        msg = auto_cycle()
        arts = view.draw_state(sess, st["paused"], msg)
        st["frames"] += 1
        st["frame_ms"].append((time.perf_counter() - t0) * 1e3)
        return arts

    if args.save:
        n = int(args.duration * args.fps)
        anim = FuncAnimation(view.fig, frame_fn, frames=n, interval=1000 / args.fps, blit=False)
        anim.save(args.save, fps=args.fps, dpi=80)
        print(f"saved {args.save}: {n} frames, resets={st['resets']}")
        return

    # Tk 타이머는 콜백 종료 후 interval 을 다시 세므로, 처리시간(~20 ms)을 빼고 짧게 잡아 목표 fps 에 맞춘다.
    # 시뮬레이션 시간은 프레임 수가 아니라 벽시계로 진행하므로 fps 와 무관하게 1× 실시간이다.
    anim = FuncAnimation(view.fig, frame_fn, interval=max(1, int(1000 / args.fps) - 22), blit=True,
                         cache_frame_data=False)

    if args.selftest > 0:
        from matplotlib.backend_bases import KeyEvent
        T = args.selftest
        script = [(0.35 * T, "right"), (0.45 * T, " "), (0.5 * T, " "), (0.6 * T, "left"), (0.75 * T, "r")]

        def inject(key):
            view.fig.canvas.callbacks.process("key_press_event",
                                              KeyEvent("key_press_event", view.fig.canvas, key))
        timers = []
        for at, key in script:
            tm = view.fig.canvas.new_timer(interval=int(at * 1000))
            tm.single_shot = True
            tm.add_callback(inject, key)
            tm.start()
            timers.append(tm)
        if args.screenshot:
            def grab():
                from PIL import ImageGrab
                w = view.fig.canvas.get_tk_widget().winfo_toplevel()
                x0, y0 = w.winfo_rootx(), w.winfo_rooty()
                ImageGrab.grab(bbox=(x0, y0, x0 + w.winfo_width(), y0 + w.winfo_height())).save(args.screenshot)
            tg = view.fig.canvas.new_timer(interval=int(0.18 * T * 1000))
            tg.single_shot = True
            tg.add_callback(grab)
            tg.start()
            timers.append(tg)
        tq = view.fig.canvas.new_timer(interval=int(T * 1000))
        tq.single_shot = True
        tq.add_callback(inject, "q")
        tq.start()
        timers.append(tq)

    plt.show()

    if args.selftest > 0:
        wall = time.perf_counter() - st["wall0"]
        fm = np.array(st["frame_ms"]) if st["frame_ms"] else np.array([np.nan])
        sim = st["sim_total"] + sess.t
        print(f"[selftest] dpi={view.fig.dpi} device_pixel_ratio={getattr(view.fig.canvas, 'device_pixel_ratio', 1)} "
              f"canvas_px={view.fig.canvas.get_width_height(physical=True)}")
        print(f"[selftest] backend={matplotlib.get_backend()} wall={wall:.2f}s frames={st['frames']} "
              f"fps={st['frames'] / wall:.1f}")
        print(f"[selftest] frame compute+draw: mean={np.mean(fm):.1f} ms, p95={np.percentile(fm, 95):.1f} ms, "
              f"max={np.max(fm):.1f} ms")
        print(f"[selftest] simulated time (all cycles)={sim:.2f}s  (paused ~{0.05 * args.selftest:.2f}s), "
              f"resets={st['resets']}, keys={st['keys']}")
        print(f"[selftest] final: t={sess.t:.2f}s θ={np.degrees(sess.X[2]):+.3f}° x={sess.X[0]:+.4f} m "
              f"stabilized={sess.stabilized} fell={sess.fell}")


if __name__ == "__main__":
    main()
