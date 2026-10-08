"""Experiment-matrix expansion and run bookkeeping (phase 2 R-04). No ROS."""

import json
import random
from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class RunSpec:
    run_id: str
    scene: str
    mode: str
    seed: int
    num_agents: int
    launch_args: dict = field(default_factory=dict)


def expand_matrix(cfg):
    """All (scene, mode, seed) runs, in a fixed pseudo-random order.

    The order is shuffled with cfg['shuffle_seed'] so slow drift in machine
    load is not aligned with a single cell; it is deterministic, so an
    interrupted matrix resumes in the same order.
    """
    runs = []
    for scene, sc in cfg['scenes'].items():
        for mode, mc in cfg['modes'].items():
            for seed in sc['seeds']:
                runs.append(RunSpec(
                    run_id=f'{scene}-{mode}-seed{seed}', scene=scene,
                    mode=mode, seed=int(seed),
                    num_agents=int(sc['num_agents']),
                    launch_args={**sc.get('launch_args', {}), **mc}))
    random.Random(cfg.get('shuffle_seed', 0)).shuffle(runs)
    return runs


def is_done(run_dir):
    """A run is finished iff its run_config.json was written (valid or
    invalid). Partial directories are not done and get re-run."""
    cfg = Path(run_dir) / 'run_config.json'
    if not cfg.is_file():
        return False
    try:
        json.loads(cfg.read_text())
    except ValueError:
        return False
    return True


def classify_validity(exit_code, result_found, wall_timeout):
    """('valid', None) or ('invalid', reason). A simulated-time timeout
    (no capture within 30 s) is a valid not-captured outcome; invalid means
    the run itself failed (hang, crash, no episode result)."""
    if wall_timeout:
        return 'invalid', 'wall_timeout'
    if exit_code != 0:
        return 'invalid', f'exit_code_{exit_code}'
    if not result_found:
        return 'invalid', 'no_episode_result'
    return 'valid', None


def cell_reliability(valid, invalid):
    total = valid + invalid
    if total and invalid / total > 0.10:
        return 'unreliable'
    return 'ok'
