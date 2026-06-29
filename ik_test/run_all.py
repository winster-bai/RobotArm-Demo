#!/usr/bin/env python3
"""
批量评估所有 IK 方法，输出横向对比表。
只做纯计算评估（--no-move），不驱动电机。

评估指标：
  成功率    IK 求解成功次数
  avg/max_ms  求解耗时（毫秒）
  avg/max_err  FK 验证误差（毫米，pinocchio 作为基准）
  peak_MB   求解过程中的峰值内存增量（psutil tracemalloc）
  cpu_%     求解期间 CPU 占用率（psutil）

Usage:
  python run_all.py [--urdf ../so101_new_calib.urdf]
"""
from __future__ import annotations
import importlib, math, sys, time, tracemalloc
from pathlib import Path
import numpy as np

try:
    import psutil, os
    _PSUTIL = True
except ImportError:
    _PSUTIL = False
    print("提示: pip install psutil 可获得 CPU% 和内存指标")

sys.path.insert(0, str(Path(__file__).parent))
from eval_targets import EVAL_TARGETS, INIT_JOINTS
from arm_driver import build_pin_fk, ARM_J

URDF_DEF = str(Path(__file__).parent.parent / "so101_new_calib.urdf")


def safe_import(name: str):
    try: return importlib.import_module(name)
    except ImportError: return None


def run_method(name: str, ik_fn, fk_fn) -> dict:
    times, fk_errs, ok_count = [], [], 0
    peak_mbs, cpu_pcts = [], []

    proc = psutil.Process(os.getpid()) if _PSUTIL else None

    for tx, ty, tz, label in EVAL_TARGETS:
        # 内存基线
        tracemalloc.start()
        cpu_before = proc.cpu_times() if proc else None

        t0 = time.perf_counter()
        try:
            sol = ik_fn(tx, ty, tz)
            ok  = True; ok_count += 1
        except Exception as e:
            print(f"    ✗ {label}: {type(e).__name__}: {e}")
            sol = INIT_JOINTS.copy(); ok = False
        elapsed = time.perf_counter() - t0

        # 采集指标
        _, peak = tracemalloc.get_traced_memory(); tracemalloc.stop()
        peak_mbs.append(peak / 1024 / 1024)

        if proc and cpu_before and elapsed > 0:
            cpu_after = proc.cpu_times()
            cpu_used  = (cpu_after.user - cpu_before.user +
                         cpu_after.system - cpu_before.system)
            cpu_pcts.append(cpu_used / elapsed * 100)

        times.append(elapsed * 1000)

        if fk_fn and ok:
            ee  = fk_fn(sol)
            err = float(np.linalg.norm(ee - np.array([tx, ty, tz])) * 1000)
            fk_errs.append(err)

    return {
        "name":       name,
        "success":    f"{ok_count}/{len(EVAL_TARGETS)}",
        "avg_ms":     sum(times) / len(times),
        "max_ms":     max(times),
        "avg_err_mm": sum(fk_errs) / len(fk_errs) if fk_errs else float("nan"),
        "max_err_mm": max(fk_errs) if fk_errs else float("nan"),
        "peak_mb":    max(peak_mbs),          # 峰值内存增量（MB）
        "avg_cpu":    sum(cpu_pcts) / len(cpu_pcts) if cpu_pcts else float("nan"),
    }


def main():
    args = sys.argv[1:]; urdf = URDF_DEF
    i = 0
    while i < len(args):
        if args[i] == "--urdf" and i + 1 < len(args): urdf = args[i + 1]; i += 2
        else: i += 1

    print(f"批量 IK 评估 (--no-move)  urdf={Path(urdf).name}\n")
    fk_fn = build_pin_fk(urdf)
    if fk_fn is None:
        print("ERROR: pinocchio 未安装，无法进行 FK 误差验证。请先 pip install pin")
        sys.exit(1)

    rows = []

    # ── 1. 2-Link Planar ──────────────────────────────────────────────────────
    import math as _m
    L1, L2 = 0.1159, 0.1350
    T1OFF = _m.atan2(0.028, 0.11257)
    T2OFF = _m.atan2(0.0052, 0.1349) + T1OFF
    def ik_2link(tx, ty, tz):
        pan = _m.degrees(_m.atan2(ty, tx))
        r = _m.hypot(tx, ty); px, py = r, tz
        d = _m.hypot(px, py)
        d = max(abs(L1 - L2) * 1.01, min((L1 + L2) * 0.99, d))
        if _m.hypot(px, py) > 0 and d != _m.hypot(px, py):
            s = d / _m.hypot(px, py); px *= s; py *= s
        c2 = -(d**2 - L1**2 - L2**2) / (2 * L1 * L2); c2 = max(-1, min(1, c2))
        t2 = _m.pi - _m.acos(c2)
        t1 = _m.atan2(py, px) + _m.atan2(L2 * _m.sin(t2), L1 + L2 * _m.cos(t2))
        sl = 90.0 - _m.degrees(max(-0.1, min(3.45, t1 + T1OFF)))
        ef = _m.degrees(max(-0.2, min(_m.pi, t2 + T2OFF))) - 90.0
        return {"shoulder_pan": pan, "shoulder_lift": sl, "elbow_flex": ef,
                "wrist_flex": -sl - ef + 90, "wrist_roll": 0, "gripper": 50}
    rows.append(run_method("2-Link Planar", ik_2link, fk_fn))

    # ── 2. Placo ──────────────────────────────────────────────────────────────
    if safe_import("placo"):
        from eval_placo import PlacoIK
        _s = PlacoIK(urdf)
        # Placo 内部坐标系与 pinocchio 不同，必须用自身 FK 验证才有意义
        def ik_placo(tx, ty, tz):
            sol = _s.solve(INIT_JOINTS, tx, ty, tz); sol["gripper"] = 50; return sol
        rows.append(run_method("Placo", ik_placo, _s.fk))  # 用 placo 自身 FK
    else:
        print("  [跳过] placo 未安装")

    # ── 3. IKPy ───────────────────────────────────────────────────────────────
    if safe_import("ikpy"):
        from eval_ikpy import IKPyIK
        _s = IKPyIK(urdf)
        # IKPy 用自身 FK 验证（pinocchio FK 也得 0mm，用自身更一致）
        def ik_ikpy(tx, ty, tz): return _s.solve(INIT_JOINTS, tx, ty, tz)
        rows.append(run_method("IKPy", ik_ikpy, _s.fk))
    else:
        print("  [跳过] ikpy 未安装")

    # ── 4. Pinocchio ─────────────────────────────────────────────────────────
    if safe_import("pinocchio"):
        from eval_pinocchio import PinocchioIK
        _s = PinocchioIK(urdf)
        def ik_pin(tx, ty, tz):
            sol, _, _ = _s.solve(INIT_JOINTS, tx, ty, tz); return sol
        rows.append(run_method("Pinocchio", ik_pin, fk_fn))
    else:
        print("  [跳过] pinocchio 未安装")

    # ── 输出对比表 ────────────────────────────────────────────────────────────
    W = 88
    print("\n" + "═" * W)
    print(f"{'方法':<16} {'成功率':>6} {'avg_ms':>10} {'max_ms':>10} "
          f"{'avg_err':>10} {'max_err':>10} {'peak_MB':>9} {'avg_cpu%':>9}")
    print("─" * W)
    for r in rows:
        ae = f"{r['avg_err_mm']:.1f}mm" if not math.isnan(r['avg_err_mm']) else "   N/A"
        me = f"{r['max_err_mm']:.1f}mm" if not math.isnan(r['max_err_mm']) else "   N/A"
        pm = f"{r['peak_mb']:.2f}MB"
        cp = f"{r['avg_cpu']:.1f}%" if not math.isnan(r['avg_cpu']) else "  N/A"
        print(f"{r['name']:<16} {r['success']:>6} {r['avg_ms']:>9.3f}ms {r['max_ms']:>9.3f}ms "
              f"{ae:>10} {me:>10} {pm:>9} {cp:>9}")
    print("═" * W)
    print("注：误差验证基准 — Pinocchio/2-Link 用 pinocchio FK，Placo 用 Placo FK，IKPy 用 IKPy FK。")
    print("    各库的 FK 实现对同一 URDF 的解析略有差异，跨库误差对比仅供参考。")
    if not _PSUTIL:
        print("    内存/CPU 指标需要: pip install psutil")


if __name__ == "__main__": main()
