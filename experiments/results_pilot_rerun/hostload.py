#!/usr/bin/env python3
# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Host-load admission gate and per-trial validity rule (PREREG.md, amendment 5).

The question of the re-run is whether the host or the MPPI binary explains the
July pilot, so load from OTHER jobs on the host is the confound to keep out.

* Admission: a trial starts only when the 1-minute load average is below
  ``ADMIT_LOAD1`` at two checks ``ADMIT_SPACING_S`` apart.
* Validity: during the drive, CPU time used by processes that do not belong to
  the trial ("foreign" CPU, in cores) must stay at or below ``VALID_MEAN_CORES``
  on average and ``VALID_WINDOW_CORES`` in every ``WINDOW_S`` window.

Foreign CPU = host busy CPU (``/proc/stat``) minus the CPU time of the trial's
own processes (those that carry the trial's environment tag). The rule uses
nothing the controller can influence (not the real-time factor, not missed
control cycles): those are outcomes.

Standard library only; used by run_rerun.py (sampling, decision to repeat) and
by analyze.py (the reported verdict).
"""

from __future__ import annotations

import csv
import glob
import os
import threading
import time

ADMIT_LOAD1 = 2.0
ADMIT_SPACING_S = 30.0
ADMIT_BUSY_CORES = 0.5       # host busy CPU over the 30 s before a start (amendment 8)
VALID_MEAN_CORES = 1.0
VALID_WINDOW_CORES = 3.0
WINDOW_S = 30.0
SAMPLE_S = 5.0
CLK_TCK = os.sysconf('SC_CLK_TCK')
FIELDS = ['wall_s', 'busy_cpu_s', 'own_cpu_s', 'load1', 'load5', 'ncpu']


def loadavg() -> tuple:
    with open('/proc/loadavg') as f:
        a, b, c = f.read().split()[:3]
    return float(a), float(b), float(c)


def host_busy_cpu_s() -> float:
    """CPU-seconds the host has spent not idle since boot."""
    with open('/proc/stat') as f:
        v = [float(x) for x in f.readline().split()[1:]]
    idle = v[3] + (v[4] if len(v) > 4 else 0.0)        # idle + iowait
    return (sum(v[:8]) - idle) / CLK_TCK


def top_consumers(n: int = 8) -> list:
    """The ``n`` busiest processes right now, as 'pcpu comm' strings."""
    import subprocess
    r = subprocess.run(['ps', '-eo', 'pcpu,comm', '--sort=-pcpu', '--no-headers'],
                       capture_output=True, text=True)
    return [' '.join(ln.split()) for ln in r.stdout.splitlines()[:n]]


def wait_for_quiet(log=print, poll_s: float = 15.0) -> dict:
    """Block until the admission rule holds. Returns what was seen."""
    t0 = time.time()
    first_ok = None
    while True:
        l1, l5, _ = loadavg()
        now = time.time()
        if l1 < ADMIT_LOAD1:
            if first_ok is None:
                first_ok, busy0 = now, host_busy_cpu_s()
            elif now - first_ok >= ADMIT_SPACING_S:
                # No trial is running here, so all busy CPU is foreign.
                busy = (host_busy_cpu_s() - busy0) / (now - first_ok)
                if busy < ADMIT_BUSY_CORES:
                    return {'load1_at_start': l1, 'load5_at_start': l5,
                            'busy_cores_before_start': round(busy, 3),
                            'load_gate_waited_s': round(now - t0, 1),
                            'top_before': top_consumers()}
                first_ok = None
        else:
            first_ok = None
        if int(now - t0) % 300 < poll_s:
            log(f'[gate] waiting for a quiet host: load1={l1:.2f} '
                f'(need < {ADMIT_LOAD1} and busy CPU < {ADMIT_BUSY_CORES} cores), '
                f'waited {now - t0:.0f} s')
        time.sleep(poll_s)


class Sampler(threading.Thread):
    """Writes host and own CPU time every ``SAMPLE_S`` seconds to a CSV."""

    def __init__(self, path: str, tag_var: str, tag: str):
        super().__init__(daemon=True)
        self.path, self.needle = path, f'{tag_var}={tag}'.encode()
        self._stop_evt = threading.Event()
        self._own = {}                       # pid -> last utime+stime (s)

    def _own_cpu_s(self) -> float:
        for p in glob.glob('/proc/[0-9]*/environ'):
            pid = p.split('/')[2]
            try:
                with open(p, 'rb') as f:
                    if self.needle not in f.read().split(b'\0'):
                        continue
                with open(f'/proc/{pid}/stat') as f:
                    rest = f.read().rsplit(')', 1)[1].split()
                self._own[pid] = (float(rest[11]) + float(rest[12])) / CLK_TCK
            except (OSError, IndexError, ValueError):
                continue
        return sum(self._own.values())       # exited processes keep their last value

    def run(self):
        with open(self.path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(FIELDS)
            while not self._stop_evt.is_set():
                l1, l5, _ = loadavg()
                w.writerow([f'{time.time():.3f}', f'{host_busy_cpu_s():.3f}',
                            f'{self._own_cpu_s():.3f}', l1, l5, os.cpu_count()])
                f.flush()
                self._stop_evt.wait(SAMPLE_S)

    def stop(self):
        self._stop_evt.set()
        self.join(timeout=SAMPLE_S + 5)


def verdict(rows: list, t_start: float, t_end: float) -> dict:
    """Validity of the interval ``[t_start, t_end]`` from sampler rows
    (dicts of floats with the ``FIELDS`` keys)."""
    rs = [r for r in rows if t_start - SAMPLE_S <= r['wall_s'] <= t_end + SAMPLE_S]
    if len(rs) < 2:
        return {'valid': False, 'reason': 'no host-load samples over the drive'}

    def foreign(a, b):
        dt = b['wall_s'] - a['wall_s']
        return ((b['busy_cpu_s'] - a['busy_cpu_s']) - (b['own_cpu_s'] - a['own_cpu_s'])) / dt

    mean = foreign(rs[0], rs[-1])
    worst = mean
    i = 0
    for j in range(len(rs)):
        while rs[j]['wall_s'] - rs[i]['wall_s'] > WINDOW_S:
            i += 1
        if rs[j]['wall_s'] - rs[i]['wall_s'] >= WINDOW_S - SAMPLE_S and j > i:
            worst = max(worst, foreign(rs[i], rs[j]))
    own = (rs[-1]['own_cpu_s'] - rs[0]['own_cpu_s']) / (rs[-1]['wall_s'] - rs[0]['wall_s'])
    out = {
        'foreign_cores_mean': round(mean, 3), 'foreign_cores_worst_window': round(worst, 3),
        'own_cores_mean': round(own, 3),
        'load1_min': min(r['load1'] for r in rs), 'load1_max': max(r['load1'] for r in rs),
        'valid': mean <= VALID_MEAN_CORES and worst <= VALID_WINDOW_CORES,
    }
    if not out['valid']:
        out['reason'] = (f'foreign CPU {mean:.2f} cores mean, {worst:.2f} worst '
                         f'{WINDOW_S:.0f} s window (limits {VALID_MEAN_CORES}, '
                         f'{VALID_WINDOW_CORES})')
    return out


def read_rows(path: str) -> list:
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
