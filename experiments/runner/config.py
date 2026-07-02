# Copyright (c) 2026 Jungmo Kang. Licensed under the Apache License, Version 2.0.
"""Ablation config resolution (protocol §3).

Each ablation (A–F, plus the F-variants) is expressed as a *param overlay* over
a base Nav2 params YAML, rather than a full copy. This module loads the compact
``configs/ablations.yaml`` spec, expands ``inherit`` chains, and applies each
overlay (dotted-path ``set`` edits + ``drop_critics``) onto the base params to
yield the final, ready-to-launch parameter dict for one controller config.

Keeping configs as diffs means a base-param change propagates to every ablation,
and the 9 cells of the matrix stay legible next to the protocol table.
"""

from __future__ import annotations

import copy
import os
from dataclasses import dataclass, field
from typing import Any

import yaml


# --------------------------------------------------------------------------- #
# Dotted-path helpers (operate on the nested dict parsed from a Nav2 yaml)
# --------------------------------------------------------------------------- #
def get_by_path(tree: dict, path: str) -> Any:
    """Return the value at a dotted ``path``; raise KeyError if absent."""
    node: Any = tree
    for key in path.split('.'):
        if not isinstance(node, dict) or key not in node:
            raise KeyError(f'path not found: {path!r} (at {key!r})')
        node = node[key]
    return node


def set_by_path(tree: dict, path: str, value: Any) -> None:
    """Set ``value`` at a dotted ``path``, creating intermediate dicts."""
    keys = path.split('.')
    node = tree
    for key in keys[:-1]:
        nxt = node.get(key)
        if not isinstance(nxt, dict):
            nxt = {}
            node[key] = nxt
        node = nxt
    node[keys[-1]] = value


# --------------------------------------------------------------------------- #
# Spec model
# --------------------------------------------------------------------------- #
@dataclass
class AblationSpec:
    """One resolved ablation entry from ``ablations.yaml`` (after inherit)."""

    name: str
    desc: str
    set: dict = field(default_factory=dict)        # FP-relative dotted -> value
    drop_critics: list = field(default_factory=list)


@dataclass
class AblationSuite:
    base_path: str          # base params yaml, resolved to an absolute path
    fp_prefix: str          # dotted prefix of the FollowPath controller namespace
    specs: dict             # name -> AblationSpec, inherit already flattened


def load_ablations(path: str) -> AblationSuite:
    """Load and flatten ``ablations.yaml`` into an :class:`AblationSuite`."""
    with open(path) as f:
        doc = yaml.safe_load(f)
    base_dir = os.path.dirname(os.path.abspath(path))
    base_path = os.path.normpath(os.path.join(base_dir, doc['base']))
    fp_prefix = doc['fp_prefix']
    raw = doc['configs']

    def flatten(name: str, seen: tuple = ()) -> AblationSpec:
        if name in seen:
            raise ValueError(f'inherit cycle: {" -> ".join(seen + (name,))}')
        entry = raw[name]
        parent = entry.get('inherit')
        merged_set: dict = {}
        merged_drop: list = []
        if parent:
            pspec = flatten(parent, seen + (name,))
            merged_set.update(pspec.set)
            merged_drop.extend(pspec.drop_critics)
        merged_set.update(entry.get('set', {}) or {})
        for c in entry.get('drop_critics', []) or []:
            if c not in merged_drop:
                merged_drop.append(c)
        return AblationSpec(
            name=name,
            desc=entry.get('desc', ''),
            set=merged_set,
            drop_critics=merged_drop,
        )

    specs = {name: flatten(name) for name in raw}
    return AblationSuite(base_path=base_path, fp_prefix=fp_prefix, specs=specs)


def resolve_params(suite: AblationSuite, name: str,
                   base_tree: dict | None = None) -> dict:
    """Apply ablation ``name`` onto the base params, returning a new dict.

    ``base_tree`` may be supplied (already-parsed base yaml) to avoid disk I/O
    in tests; otherwise it is read from ``suite.base_path``.
    """
    if name not in suite.specs:
        raise KeyError(f'unknown ablation: {name!r}')
    spec = suite.specs[name]
    if base_tree is None:
        with open(suite.base_path) as f:
            base_tree = yaml.safe_load(f)
    tree = copy.deepcopy(base_tree)
    fp = suite.fp_prefix

    # 1. drop_critics: remove names from the FollowPath critics list (if present).
    if spec.drop_critics:
        try:
            critics = list(get_by_path(tree, f'{fp}.critics'))
        except KeyError:
            critics = []
        critics = [c for c in critics if c not in spec.drop_critics]
        set_by_path(tree, f'{fp}.critics', critics)

    # 2. set: apply each FP-relative dotted edit.
    for rel_path, value in spec.set.items():
        set_by_path(tree, f'{fp}.{rel_path}', value)

    return tree


def config_names(suite: AblationSuite) -> list:
    """Ablation names in the file's declared order."""
    return list(suite.specs.keys())
