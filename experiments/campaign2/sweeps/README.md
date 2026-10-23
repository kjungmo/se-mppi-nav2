# Headless development sweeps: per-geometry rows of confirmation 2

Source: the logs `sweep_*.log` of the confirmation-2 sweeps, which ran on 2026-10-18 at commit
`b7e012a` (controller sources of `507173a`; see `../PREREG.md`), from 09:44 to 09:57 KST, on a
shared host. Each file here holds the lines that start with `UTRAP ` of one sweep: one row per
geometry (56 valid geometries, one run each) and the summary line `UTRAP sweep passed N of 56`.
Rows of runs that did not pass (for example a run that never entered the pocket) are included.

| File | Additions | Passed of 56 |
|---|---|---|
| `confirmation2_on1.txt` | on | 55 |
| `confirmation2_on2.txt` | on | 56 |
| `confirmation2_on3.txt` | on | 56 |
| `confirmation2_off1.txt` | off | 16 |
| `confirmation2_off2.txt` | off | 18 |
| `confirmation2_off3b.txt` | off | 14 |

A run passes if it enters the pocket, leaves it within 60 s, reaches within 0.5 m of the goal
within 150 s, and has no contact and no `NoValidControl` throw. `reached=1` alone is not a pass
(`on1` has 56 rows with `reached=1` and 55 passes: one run never entered the pocket).

Not archived: the sweeps of the implementer and of the reviewer, and the stalled `off3` (3 rows).
The sweeps cannot be seeded: MPPI's noise is not seedable from the test, so a rerun gives
different counts. Commands: Appendix A of the paper (Headless development test) and `../PREREG.md`
(the switch-off string and the sweep setting, lines 198-204). `robot_radius_lines.txt` holds the controller start lines of
the raw logs (the rig logs `robot_radius=0.114`).
