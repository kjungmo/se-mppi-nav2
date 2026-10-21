#!/usr/bin/env python3
"""Post-hoc check (not part of the frozen analysis): CBF obstacle admissions, control-loop rate
and host load of campaign 2.

Three things, read from committed files only:

* ``admissions``: for every SE-MPPI trial (C2, C4; evidence trials included), whether the
  diagnostics field ``cbf_obstacles`` (the number of obstacles the tracker passed to the CBF in
  the cycle of the message) was above zero in any diagnostics message, and its maximum. The
  controller publishes diagnostics once per wall-clock second, so this samples about one cycle in
  ten: a trial counted as zero may still have had admissions between samples. The counts are
  therefore lower bounds.
* ``loop_rate``: control cycles per wall-clock second and per simulated second, from the first to
  the last diagnostics message with ``cycles`` above zero (cumulative ``cycles`` field; ``wall_s``
  and ``sim_s`` of the message).
* ``collision_monitor_log``: lines of the ``collision_monitor`` node in each trial's
  ``launch.log``, and how many are not at INFO level (a source-invalid or stop warning would be).
* ``host``: own CPU (``host_load.own_cores_mean``), ``host_load.load1_max``,
  ``missed_control_cycles`` and ``rtf_min_10s`` from ``results.json.gz``.

Inputs: ``raw_trials.tar.gz`` (committed; ``--archive``) and ``results.json.gz``. Run from the
repository root:

    python3 experiments/campaign2/cbf_admissions.py [--out-dir DIR]

It writes ``CBF_ADMISSIONS.json`` and ``CBF_ADMISSIONS.md`` into ``--out-dir`` (default: the
directory of this file). The paper quotes the output (``scripts/check_paper_numbers.py`` re-reads
``CBF_ADMISSIONS.json``).
"""
import argparse
import gzip
import json
import os
import tarfile

HERE = os.path.dirname(os.path.abspath(__file__))
SE = ('C2', 'C4')


def rng(vals, d=3):
    return {'n': len(vals), 'min': round(min(vals), d), 'max': round(max(vals), d)} if vals else {'n': 0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--archive', default=os.path.join(HERE, 'raw_trials.tar.gz'))
    ap.add_argument('--results', default=os.path.join(HERE, 'results.json.gz'))
    ap.add_argument('--out-dir', default=HERE)
    a = ap.parse_args()

    res = json.loads(gzip.open(a.results).read())
    trials = {t['trial_id']: t for t in res['trials']}

    diag = {}
    cm_logs = {}
    with tarfile.open(a.archive) as tf:
        for m in tf.getmembers():
            p = m.name.split('/')
            if len(p) == 4 and p[0] == 'campaign2_raw' and p[1] == 'trials' and p[3] == 'diagnostics.jsonl':
                rows = [json.loads(ln) for ln in tf.extractfile(m).read().decode().splitlines() if ln.strip()]
                diag[p[2]] = rows
            if len(p) == 4 and p[0] == 'campaign2_raw' and p[1] == 'trials' and p[3] == 'launch.log':
                lines = tf.extractfile(m).read().decode(errors='replace').splitlines()
                cm = [ln for ln in lines if '[collision_monitor' in ln]
                cm_logs[p[2]] = {'lines': len(cm), 'not_info': sum('[INFO]' not in ln for ln in cm)}
    assert set(diag) == set(trials), 'archive trials differ from results.json.gz'

    rows = []
    for tid in sorted(trials):
        t = trials[tid]
        d = diag[tid]
        cbf = [int(r['values'].get('cbf_obstacles', '0')) for r in d]
        run = [r for r in d if int(r['values'].get('cycles', '0')) > 0]
        rate_wall = rate_sim = None
        if len(run) >= 2:
            dc = int(run[-1]['values']['cycles']) - int(run[0]['values']['cycles'])
            rate_wall = dc / (run[-1]['wall_s'] - run[0]['wall_s'])
            rate_sim = dc / (run[-1]['sim_s'] - run[0]['sim_s'])
        hl = t['host_load']
        rows.append({
            'trial_id': tid, 'scenario': t['scenario'], 'config': t['config'],
            'evidence': t['scenario'].endswith('evidence'),
            'diag_messages': len(d),
            'cbf_obstacles_positive_messages': sum(c > 0 for c in cbf),
            'cbf_obstacles_max': max(cbf) if cbf else 0,
            'forced_stops_last': int(t['diagnostics_last'].get('forced_stops', '0')),
            'cycles_per_wall_s': rate_wall, 'cycles_per_sim_s': rate_sim,
            'own_cores_mean': hl['own_cores_mean'], 'load1_max': hl['load1_max'],
            'missed_control_cycles': t['missed_control_cycles'], 'rtf_min_10s': t['rtf_min_10s'],
        })

    se = [r for r in rows if r['config'] in SE]
    adm = [r for r in se if r['cbf_obstacles_positive_messages'] > 0]
    by_scen = {}
    for r in se:
        k = r['scenario']
        b = by_scen.setdefault(k, {'n': 0, 'with_admission': 0, 'max_obstacles': 0, 'with_forced_stop': 0})
        b['n'] += 1
        b['with_admission'] += r['cbf_obstacles_positive_messages'] > 0
        b['max_obstacles'] = max(b['max_obstacles'], r['cbf_obstacles_max'])
        b['with_forced_stop'] += r['forced_stops_last'] > 0
    se_rates_w = [r['cycles_per_wall_s'] for r in se if r['cycles_per_wall_s'] is not None]
    se_rates_s = [r['cycles_per_sim_s'] for r in se if r['cycles_per_sim_s'] is not None]
    missed = {}
    for r in rows:
        if r['missed_control_cycles']:
            k = f"{r['scenario']}/{r['config']}"
            missed.setdefault(k, []).append(r['missed_control_cycles'])
    se_pocket = [r for r in se if r['scenario'] != 's0_open']
    out = {
        'note': 'post hoc; diagnostics sampled once per wall-clock second; counts are lower bounds',
        'admissions': {
            'se_trials': len(se), 'se_trials_with_admission': len(adm),
            'max_obstacles_in_a_message': max(r['cbf_obstacles_max'] for r in se),
            'by_scenario': by_scen,
        },
        'loop_rate': {
            'se_trials': len(se_rates_w),
            'cycles_per_wall_s': rng(se_rates_w, 2), 'cycles_per_sim_s': rng(se_rates_s, 2),
        },
        'host': {
            'all_trials': len(rows),
            'own_cores_mean': rng([r['own_cores_mean'] for r in rows], 1),
            'load1_max': rng([r['load1_max'] for r in rows], 2),
            'rtf_min_10s_all': rng([r['rtf_min_10s'] for r in rows], 3),
            'rtf_min_10s_se_pocket': rng([r['rtf_min_10s'] for r in se_pocket], 3),
            'missed_control_cycles_nonzero': missed,
        },
        'collision_monitor_log': {
            'launch_logs': len(cm_logs),
            'lines': sum(v['lines'] for v in cm_logs.values()),
            'lines_not_info': sum(v['not_info'] for v in cm_logs.values()),
        },
        'rows': rows,
    }
    os.makedirs(a.out_dir, exist_ok=True)
    with open(os.path.join(a.out_dir, 'CBF_ADMISSIONS.json'), 'w') as f:
        json.dump(out, f, indent=1, sort_keys=True)
        f.write('\n')
    L = ['# CBF admissions, loop rate and host load (campaign 2, post hoc)', '',
         'Generated by `experiments/campaign2/cbf_admissions.py` from `raw_trials.tar.gz` and',
         '`results.json.gz`. Diagnostics are sampled once per wall-clock second, so the admission',
         'counts are lower bounds.', '',
         f"SE-MPPI trials (C2, C4, evidence included) with `cbf_obstacles` > 0 in at least one message: "
         f"{len(adm)} of {len(se)}; largest value in a message: {out['admissions']['max_obstacles_in_a_message']}.", '',
         '| scenario | SE trials | with admission | max obstacles | with forced stop |', '|---|---|---|---|---|']
    for k in sorted(by_scen):
        b = by_scen[k]
        L.append(f"| {k} | {b['n']} | {b['with_admission']} | {b['max_obstacles']} | {b['with_forced_stop']} |")
    lr = out['loop_rate']
    h = out['host']
    L += ['', f"Control cycles per wall-clock second (SE trials, n={lr['se_trials']}): "
          f"{lr['cycles_per_wall_s']['min']:.2f} to {lr['cycles_per_wall_s']['max']:.2f}; per simulated second: "
          f"{lr['cycles_per_sim_s']['min']:.2f} to {lr['cycles_per_sim_s']['max']:.2f}.", '',
          f"Host, all {h['all_trials']} trials: own CPU {h['own_cores_mean']['min']:.1f} to "
          f"{h['own_cores_mean']['max']:.1f} cores (mean per trial); load1 peak {h['load1_max']['min']:.2f} to "
          f"{h['load1_max']['max']:.2f}; lowest 10 s real-time factor {h['rtf_min_10s_all']['min']:.3f} "
          f"(SE pocket trials: {h['rtf_min_10s_se_pocket']['min']:.3f}).", '',
          'Trials with missed control cycles (Nav2 warning count):', '']
    for k in sorted(missed):
        L.append(f"- {k}: {', '.join(str(v) for v in missed[k])}")
    if not missed:
        L.append('- none')
    cml = out['collision_monitor_log']
    L += ['', f"collision_monitor in the launch logs ({cml['launch_logs']} trials): {cml['lines']} lines, "
          f"{cml['lines_not_info']} of them above INFO level."]
    with open(os.path.join(a.out_dir, 'CBF_ADMISSIONS.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')


if __name__ == '__main__':
    main()
