#!/usr/bin/env python3
"""Phase 4C closed-loop visual nudge -- the FK-independent grasp-aim corrector.

Root problem (see PHASE4C_VISUAL_SERVO_PLAN.md): the open-loop table->arm map leaves a
~14 mm, POSITION-DEPENDENT residual (the real arm over-reaches the URDF model and 6 touch
points cannot identify a better one), which exceeds the 40 mm box's +-7.5 mm grasp
tolerance everywhere except a ~4 cm patch.

Fix: before the jaws descend, the SAME overhead camera that perceives the object also sees
an ArUco marker on the gripper. We iterate, in the TABLE frame,
    err = obj_xy - (gripper_marker_xy + tip_offset)
    command_target += K * err
until |err| < thresh. Because ``err`` is a DIFFERENCE of two points measured through the
SAME camera in the SAME frame, the wrong FK and the global scale CANCEL as common-mode
bias -- they only need to be good enough for the loop to converge. Grasp accuracy then
becomes camera-bound (~1-4 mm), workspace-wide, INDEPENDENT of the kinematic model.

This module is pure + hardware-free. ``servo_align`` takes an ``observe`` callable (real =
capture+detect the gripper marker; test = a synthetic biased model) and a ``command``
callable (real = table->arm->IK->move; test = record the target). The __main__ self-test
proves convergence against a position-dependent bias with NO hardware.
"""
from __future__ import annotations

import math
from typing import Callable, Dict, List, Tuple

XY = Tuple[float, float]


def servo_align(obj_xy: XY,
                observe: Callable[[], XY],
                command: Callable[[XY], None],
                *,
                tip_offset: XY = (0.0, 0.0),
                gain: float = 0.6,
                thresh_m: float = 0.003,
                max_iter: int = 4) -> Dict:
    """Closed-loop visual nudge in the TABLE frame.

    obj_xy      target object centre (table frame, metres).
    observe()   -> measured gripper-marker centre (table frame, metres), or None on a
                   missed/failed measurement (the loop then HOLDS AIM and retries; a miss
                   is NEVER counted as convergence).
    command(t)  move the arm so the gripper AIMS at table-frame point ``t``.
    tip_offset  fixed marker-centre -> fingertip-contact offset (table frame, m).
    gain        nudge fraction K (0<K<2; 0.6 ~= 0.4 contraction/step).
    thresh_m    convergence radius (3 mm default, well inside the box tolerance).
    max_iter    iteration cap (each step is one camera grab -- keep it small).

    Returns {converged, iters, final_target, history:[{target, observed, err_mm}]}.
    The caller descends + closes at ``final_target`` (the most-corrected aim).
    """
    target: XY = (float(obj_xy[0]), float(obj_xy[1]))      # naive aim = the object itself
    history: List[Dict] = []
    converged = False
    measured = 0
    misses = 0
    for _ in range(max_iter):
        command(target)
        obs = observe()
        if obs is None:                                    # no measurement (e.g. capture failed)
            misses += 1                                    # HOLD AIM + retry; never 'converged'
            history.append({"target": (round(target[0], 5), round(target[1], 5)),
                            "observed": None, "err_mm": None, "miss": True})
            continue
        gx, gy = obs
        ex = obj_xy[0] - (gx + tip_offset[0])
        ey = obj_xy[1] - (gy + tip_offset[1])
        err_mm = math.hypot(ex, ey) * 1000.0
        measured += 1
        history.append({"target": (round(target[0], 5), round(target[1], 5)),
                        "observed": (round(gx, 5), round(gy, 5)),
                        "err_mm": round(err_mm, 2)})
        if err_mm <= thresh_m * 1000.0:
            converged = True
            break
        target = (target[0] + gain * ex, target[1] + gain * ey)
    return {"converged": converged, "iters": len(history),
            "measured": measured, "misses": misses,
            "final_target": target, "history": history}


# --------------------------------------------------------------------------- #
#  Offline self-test: synthetic position-dependent bias, no hardware/camera.   #
# --------------------------------------------------------------------------- #
def _selftest() -> int:
    import random
    random.seed(1)

    def make_world(obj_xy):
        """A synthetic 'real arm + camera': the gripper marker LANDS at a biased
        transform of the commanded target. Mimics the MEASURED fault -- a constant
        over-reach component (~up to 14 mm) PLUS a position-dependent linear term
        (the ~11-15% scatter a single global scale cannot capture) PLUS ~1 mm camera
        noise:  actual_marker = commanded + b0 + B @ (commanded - obj) + noise."""
        b0 = (random.uniform(-0.012, 0.012), random.uniform(-0.012, 0.012))
        B = [[random.uniform(-0.15, 0.15), random.uniform(-0.15, 0.15)],
             [random.uniform(-0.15, 0.15), random.uniform(-0.15, 0.15)]]
        state = {"cmd": obj_xy}

        def command(t):
            state["cmd"] = (float(t[0]), float(t[1]))

        def observe():
            cx, cy = state["cmd"]
            dx, dy = cx - obj_xy[0], cy - obj_xy[1]
            ax = cx + b0[0] + B[0][0] * dx + B[0][1] * dy + random.gauss(0, 0.001)
            ay = cy + b0[1] + B[1][0] * dx + B[1][1] * dy + random.gauss(0, 0.001)
            return (ax, ay)
        return command, observe

    N = 500
    reached = 0
    start_errs, final_errs, iters_used = [], [], []
    for _ in range(N):
        obj = (random.uniform(0.13, 0.32), random.uniform(0.10, 0.40))
        command, observe = make_world(obj)
        # open-loop baseline: aim naively at the object, see where the gripper lands
        command(obj)
        gx, gy = observe()
        start_errs.append(math.hypot(obj[0] - gx, obj[1] - gy) * 1000.0)
        r = servo_align(obj, observe, command, gain=0.6, thresh_m=0.003, max_iter=4)
        final_errs.append(r["history"][-1]["err_mm"])
        iters_used.append(r["iters"])
        if r["history"][-1]["err_mm"] <= 4.0:        # <=4 mm allows ~1 mm camera noise
            reached += 1

    def stats(a):
        a = sorted(a)
        n = len(a)
        return sum(a) / n, a[n // 2], a[int(0.95 * n)], max(a)

    sm, smed, s95, smax = stats(start_errs)
    fm, fmed, f95, fmax = stats(final_errs)
    print("[selftest] %d random objects | position-dependent bias + 1mm cam noise | K=0.6 cap=4" % N)
    print("  open-loop  start err mm: mean %.1f median %.1f p95 %.1f max %.1f" % (sm, smed, s95, smax))
    print("  closed-loop final err mm: mean %.2f median %.2f p95 %.2f max %.2f" % (fm, fmed, f95, fmax))
    print("  reached <=4mm: %d/%d (%.1f%%) | mean iters %.1f" % (reached, N, 100.0 * reached / N, sum(iters_used) / N))
    ok = reached >= 0.95 * N and fmed < 3.0
    print("  RESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
