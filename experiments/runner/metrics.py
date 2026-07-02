# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Trial metric computation (protocol §4) — pure functions over recorded streams.

A trial produces a time series of samples (sim time, robot pose, ground-truth
obstacle poses, per-cycle compute time, commanded velocity). These functions
turn that series into the paper's metrics. They are deliberately free of ROS so
they can be unit-tested with synthetic logs and re-run on saved trial JSON.

Sample schema (one dict per control cycle / odom tick):
    {
      "t": float,                 # sim clock seconds (required)
      "x": float, "y": float,     # robot pose in map frame (required)
      "yaw": float,               # optional
      "v": float, "w": float,     # commanded twist (optional, for smoothness)
      "loop_ms": float,           # controller compute time this cycle (optional)
      "obstacles": [              # ground-truth obstacles this instant (optional)
         {"x": float, "y": float, "r": float}, ...
      ],
    }
Obstacles may instead be supplied once as a static list (see compute_metrics).
"""

from __future__ import annotations

import math
from typing import Sequence

# Trial outcome categories (design §2 "[CLASSIFY]").
SUCCESS = 'SUCCESS'
COLLISION = 'COLLISION'
TIMEOUT = 'TIMEOUT'
STUCK = 'STUCK'
SETUP_FAIL = 'SETUP_FAIL'


def _dist(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def path_length(samples: Sequence[dict]) -> float:
    """Arc length of the executed trajectory (integral of |Δpose|)."""
    total = 0.0
    for a, b in zip(samples, samples[1:]):
        total += _dist(a['x'], a['y'], b['x'], b['y'])
    return total


def clearance_series(samples: Sequence[dict], robot_radius: float,
                     static_obstacles: Sequence[dict] | None = None) -> list:
    """Per-sample signed clearance to the nearest obstacle surface.

    clearance = dist(center, obstacle_center) − obstacle_r − robot_radius.
    Negative means the footprints overlap (a collision instant). Obstacles are
    taken per-sample from ``sample['obstacles']`` when present, else from the
    shared ``static_obstacles`` list. Samples with no obstacles yield +inf.
    """
    out = []
    for s in samples:
        obs = s.get('obstacles')
        if obs is None:
            obs = static_obstacles or []
        if not obs:
            out.append(math.inf)
            continue
        nearest = min(
            _dist(s['x'], s['y'], o['x'], o['y']) - o.get('r', 0.0) - robot_radius
            for o in obs
        )
        out.append(nearest)
    return out


def reached_goal(samples: Sequence[dict], goal: Sequence[float],
                 goal_tol: float) -> bool:
    """True if the final pose is within ``goal_tol`` of the goal position."""
    if not samples:
        return False
    last = samples[-1]
    return _dist(last['x'], last['y'], goal[0], goal[1]) <= goal_tol


def time_to_goal(samples: Sequence[dict]) -> float:
    """Elapsed sim time from first to last sample."""
    if len(samples) < 2:
        return 0.0
    return samples[-1]['t'] - samples[0]['t']


def spl(success: bool, optimal_length: float, actual_length: float) -> float:
    """Success weighted by (normalized inverse) path length (Anderson et al.)."""
    if not success or actual_length <= 0.0:
        return 0.0
    return optimal_length / max(actual_length, optimal_length)


def barn_score(success: bool, optimal_time: float, actual_time: float) -> float:
    """BARN navigation score: 1_success · OT / clip(AT, 2·OT, 8·OT)."""
    if not success or optimal_time <= 0.0:
        return 0.0
    lo, hi = 2.0 * optimal_time, 8.0 * optimal_time
    at = min(max(actual_time, lo), hi)
    return optimal_time / at


def compute_time_stats(samples: Sequence[dict]) -> dict:
    """Aggregate per-cycle controller compute time (ms): mean/median/p95/max."""
    vals = [s['loop_ms'] for s in samples if s.get('loop_ms') is not None]
    if not vals:
        return {'n': 0, 'mean_ms': None, 'median_ms': None,
                'p95_ms': None, 'max_ms': None}
    sv = sorted(vals)
    return {
        'n': len(sv),
        'mean_ms': sum(sv) / len(sv),
        'median_ms': _percentile(sv, 50),
        'p95_ms': _percentile(sv, 95),
        'max_ms': sv[-1],
    }


def _percentile(sorted_vals: Sequence[float], pct: float) -> float:
    """Linear-interpolated percentile of a pre-sorted sequence."""
    if not sorted_vals:
        return math.nan
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    rank = (pct / 100.0) * (len(sorted_vals) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return sorted_vals[lo]
    frac = rank - lo
    return sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac


def smoothness(samples: Sequence[dict]) -> dict:
    """Mean absolute linear/angular jerk from the commanded twist series.

    Falls back gracefully: needs ``v``/``w`` and ``t`` on the samples. Jerk is
    the time derivative of acceleration (second difference of velocity / dt²),
    averaged in magnitude — a standard control-smoothness proxy.
    """
    ts = [s['t'] for s in samples if 'v' in s and 'w' in s]
    vs = [s['v'] for s in samples if 'v' in s and 'w' in s]
    ws = [s['w'] for s in samples if 'v' in s and 'w' in s]
    return {
        'lin_jerk_mean': _mean_abs_jerk(ts, vs),
        'ang_jerk_mean': _mean_abs_jerk(ts, ws),
    }


def _mean_abs_jerk(ts: Sequence[float], xs: Sequence[float]) -> float | None:
    if len(xs) < 3:
        return None
    acc = []
    for i in range(1, len(xs)):
        dt = ts[i] - ts[i - 1]
        if dt <= 0:
            continue
        acc.append(((xs[i] - xs[i - 1]) / dt, 0.5 * (ts[i] + ts[i - 1])))
    if len(acc) < 2:
        return None
    jerks = []
    for i in range(1, len(acc)):
        dt = acc[i][1] - acc[i - 1][1]
        if dt <= 0:
            continue
        jerks.append(abs((acc[i][0] - acc[i - 1][0]) / dt))
    return sum(jerks) / len(jerks) if jerks else None


def oscillation_ratio(samples: Sequence[dict], v_eps: float = 0.05,
                      w_eps: float = 0.2) -> float | None:
    """Fraction of cycles spinning in place (|v|<v_eps and |w|>w_eps).

    A proxy for the local-minima thrashing that escape is meant to remove
    (protocol §4 "진동/제자리회전 시간 비율"). Needs ``v``/``w`` on samples.
    """
    cmd = [(s['v'], s['w']) for s in samples if 'v' in s and 'w' in s]
    if not cmd:
        return None
    spin = sum(1 for v, w in cmd if abs(v) < v_eps and abs(w) > w_eps)
    return spin / len(cmd)


def compute_metrics(samples: Sequence[dict], *, goal: Sequence[float],
                    goal_tol: float, robot_radius: float,
                    static_obstacles: Sequence[dict] | None = None,
                    optimal_length: float | None = None,
                    optimal_time: float | None = None) -> dict:
    """Compute the full metric dict for one trial from its sample stream.

    ``optimal_length``/``optimal_time`` default to the straight-line start→goal
    distance and (distance / nominal 0.5 m·s⁻¹) when not supplied, so SPL/BARN
    are always defined; callers should pass planner-optimal values when known.
    """
    n = len(samples)
    reached = reached_goal(samples, goal, goal_tol)
    clr = clearance_series(samples, robot_radius, static_obstacles)
    finite = [c for c in clr if math.isfinite(c)]
    min_clr = min(finite) if finite else None
    collided = min_clr is not None and min_clr < 0.0
    plen = path_length(samples)
    ttg = time_to_goal(samples)

    if optimal_length is None and n:
        optimal_length = _dist(samples[0]['x'], samples[0]['y'], goal[0], goal[1])
    if optimal_time is None and optimal_length is not None:
        optimal_time = optimal_length / 0.5  # nominal cruise speed

    success = bool(reached and not collided)
    return {
        'n_samples': n,
        'reached_goal': reached,
        'collided': collided,
        'success': success,
        'min_clearance': min_clr,
        'time_to_goal': ttg,
        'path_length': plen,
        'spl': spl(success, optimal_length or 0.0, plen),
        'barn_score': barn_score(success, optimal_time or 0.0, ttg),
        'optimal_length': optimal_length,
        'optimal_time': optimal_time,
        'compute': compute_time_stats(samples),
        'smoothness': smoothness(samples),
        'oscillation_ratio': oscillation_ratio(samples),
    }


def classify(metrics: dict, *, setup_failed: bool = False, timeout: bool = False,
             stuck: bool = False) -> str:
    """Map a metric dict + run flags to one outcome category (design §2).

    Precedence: SETUP_FAIL > COLLISION > SUCCESS > TIMEOUT > STUCK. A collision
    dominates a timeout (an unsafe run is not merely slow); a clean goal arrival
    is SUCCESS even if the watchdog also tripped late.
    """
    if setup_failed:
        return SETUP_FAIL
    if metrics.get('collided'):
        return COLLISION
    if metrics.get('success'):
        return SUCCESS
    if timeout:
        return TIMEOUT
    # Not reached, no collision, not timed out: the robot failed to make
    # progress (controller abort, recovery loop, or the watchdog's stuck flag).
    return STUCK
