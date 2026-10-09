#!/usr/bin/env python3
"""Check that libescape_critic uses the same xtensor storage type as libmppi_controller.

EscapeCritic writes MPPI's xt::xtensor buffers (CriticData::costs, trajectories)
across the pluginlib boundary. The installed MPPI library is built with
XTENSOR_USE_XSIMD (+ AVX2), so its tensors are backed by
xt::uvector<float, xsimd::aligned_allocator<float, N>>. A critic compiled with
other flags reinterprets them with a different storage type, which crashed the
whole Nav2 container live (see the note at the top of
src/nav2_se_controller/CMakeLists.txt). The mangled symbols carry the
allocator, so comparing them catches the mismatch statically.

    python3 scripts/check_simd_abi.py \
        --mppi $CONDA_PREFIX/lib/libmppi_controller.so \
        --critic install/nav2_se_controller/lib/libescape_critic.so
"""

import argparse
import subprocess
import sys


def uvector_types(lib):
    out = subprocess.run(['nm', '-DC', lib], check=True, capture_output=True, text=True).stdout
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
    ap.add_argument('--critic', required=True, help='libescape_critic.so to check')
    args = ap.parse_args()

    mppi = set().union(*(uvector_types(lib) for lib in args.mppi))
    critic = uvector_types(args.critic)
    print('MPPI storage types:   ' + (', '.join(sorted(mppi)) or '(none)'))
    print('critic storage types: ' + (', '.join(sorted(critic)) or '(none)'))
    if not mppi:
        sys.exit('FAIL: no xt::uvector symbols in the MPPI libraries; cannot compare')
    if not critic:
        sys.exit('FAIL: no xt::uvector symbols in the critic; the check would prove nothing')
    foreign = sorted(critic - mppi)
    if foreign:
        sys.exit('FAIL: critic uses xtensor storage the MPPI libraries do not: '
                 + ', '.join(foreign))
    print('PASS: escape_critic uses the same xtensor storage type as libmppi')


if __name__ == '__main__':
    main()
