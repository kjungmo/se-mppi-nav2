#!/usr/bin/env python3
"""Check that this package's libraries use the same xtensor storage type as libmppi_controller.

EscapeCritic writes MPPI's xt::xtensor buffers (CriticData::costs, trajectories)
across the pluginlib boundary. The installed MPPI library is built with
XTENSOR_USE_XSIMD (+ AVX2), so its tensors are backed by
xt::uvector<float, xsimd::aligned_allocator<float, N>>. A critic compiled with
other flags reinterprets them with a different storage type, which crashed the
whole Nav2 container live (see the note at the top of
src/nav2_se_controller/CMakeLists.txt). The mangled symbols carry the
allocator, so comparing them catches the mismatch statically.

libmppi_rollout_check (the configure-time rollout self-check) sizes and reads
the tensors of an mppi::Optimizer the same way, so it is checked too. Its
tensor code is all inline and hidden from the dynamic symbol table, so for a
library without xt::uvector symbols there the full symbol table is read.

    python3 scripts/check_simd_abi.py \
        --mppi $CONDA_PREFIX/lib/libmppi_controller.so \
        --critic install/nav2_se_controller/lib/libescape_critic.so \
                 install/nav2_se_controller/lib/libmppi_rollout_check.so
"""

import argparse
import os
import subprocess
import sys


def uvector_types(lib, dynamic=True):
    out = subprocess.run(['nm', '-DC' if dynamic else '-C', lib],
                         check=True, capture_output=True, text=True).stdout
    found = set()
    key = 'xt::uvector<'
    for line in out.splitlines():
        start = line.find(key)
        while start != -1:
            i, depth = start + len(key), 1
            while i < len(line) and depth:
                depth += {'<': 1, '>': -1}.get(line[i], 0)
                i += 1
            found.add(line[start:i])
            start = line.find(key, i)
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mppi', required=True, nargs='+', help='installed MPPI shared libraries')
    ap.add_argument('--critic', required=True, nargs='+',
                    help='libraries of this package to check (libescape_critic.so, '
                         'libmppi_rollout_check.so)')
    args = ap.parse_args()

    mppi = set().union(*(uvector_types(lib) for lib in args.mppi))
    print('MPPI storage types: ' + (', '.join(sorted(mppi)) or '(none)'))
    if not mppi:
        sys.exit('FAIL: no xt::uvector symbols in the MPPI libraries; cannot compare')
    for lib in args.critic:
        name = os.path.basename(lib)
        ours = uvector_types(lib) or uvector_types(lib, dynamic=False)
        print(f'{name} storage types: ' + (', '.join(sorted(ours)) or '(none)'))
        if not ours:
            sys.exit(f'FAIL: no xt::uvector symbols in {name}; the check would prove nothing')
        foreign = sorted(ours - mppi)
        if foreign:
            sys.exit(f'FAIL: {name} uses xtensor storage the MPPI libraries do not: '
                     + ', '.join(foreign))
    print('PASS: ' + ', '.join(os.path.basename(lib) for lib in args.critic)
          + ' use the same xtensor storage type as libmppi')


if __name__ == '__main__':
    main()
