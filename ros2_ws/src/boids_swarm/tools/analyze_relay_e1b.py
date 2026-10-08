#!/usr/bin/env python3
"""Offline analysis of the E1b follow-up diagnostic (POST-HOC, not pre-registered).
usage: analyze_relay_e1b.py RUNS_DIR --summary summary.json [--media-dir docs/media]
"""
import argparse, collections, gzip, json, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import analyze_relay_e1 as e1
from boids_swarm import relay_stats as rs

CELLS = [('S12ff', 'ros-d1'), ('S12ff', 'ros-d10'), ('S12rt', 'ros-d1'), ('S12rt', 'ros-d10')]
SIGHTING_TIMEOUT = 0.6
# 'stale' = age above sighting_timeout (the delay-induced rejection); 'late_observation' =
# observation not newer than one the controller already applied (supersession, not delay).
MAIN_REASONS = ['accepted', 'stale', 'late_observation', 'out_of_range']


def quantile(vals, q):
    v = sorted(vals)
    if not v:
        return None
    k = (len(v) - 1) * q
    f = int(k)
    c = min(f + 1, len(v) - 1)
    return v[f] + (v[c] - v[f]) * (k - f)


def r(x, nd=4):
    return None if x is None else round(x, nd)


def load_events(d):
    with gzip.open(d / 'relay_events.jsonl.gz', 'rt') as f:
        return [json.loads(l) for l in f if l.strip()]


def reason_table(events):
    n = len(events)
    c = collections.Counter(e['reason'] for e in events)
    out = {k: [c.get(k, 0), r(c.get(k, 0) / n) if n else None] for k in MAIN_REASONS}
    other = n - sum(c.get(k, 0) for k in MAIN_REASONS)
    out['other'] = [other, r(other / n) if n else None]
    out['other_breakdown'] = {k: v for k, v in sorted(c.items()) if k not in MAIN_REASONS}
    out['n'] = n
    return out


def analyze(runs_dir):
    cells = {}
    ages_by_cell = {}
    for scene, mode in CELLS:
        runs = []
        for d in sorted(runs_dir.glob(f'{scene}-{mode}-seed*')):
            if not (d / 'run_config.json').exists():
                continue
            cfg = json.loads((d / 'run_config.json').read_text())
            runs.append((d, cfg))
        valid = [(d, c) for d, c in runs if c['validity'] == 'valid']
        drops = [e1.drop_for_run(d, c) for d, c in valid]
        drops = [x for x in drops if x]
        E = sum(x['run_expected_total'] for x in drops)
        R = sum(x['run_received_total'] for x in drops)
        boot = rs.bootstrap_pooled_drop_ci([(x['run_expected_total'], x['run_received_total']) for x in drops], 10000, 20261011)
        events = [ev for d, c in valid for ev in load_events(d)]
        per_run_events = {d.name: load_events(d) for d, c in valid}
        nonself = [e for e in events if e['reason'] != 'self_originated']
        in_range = [e for e in nonself if e['reason'] not in ('out_of_range', 'awaiting_episode')]
        # age: all callback-delivered, non-self events once the episode is known
        ages_all = [e['age'] for e in nonself if e['reason'] != 'awaiting_episode']
        ages_acc = [e['age'] for e in nonself if e['reason'] == 'accepted']
        per_run_p95 = [quantile([e['age'] for e in ev if e['reason'] not in ('self_originated', 'awaiting_episode')], .95)
                       for ev in per_run_events.values()]
        per_run_p95 = [x for x in per_run_p95 if x is not None]
        ages_by_cell[f'{scene}/{mode}'] = ages_all
        cells[f'{scene}/{mode}'] = {
            'runs': len(runs), 'valid': len(valid), 'captures': sum(c['captured'] for _, c in valid),
            'seeds': sorted(c['seed'] for _, c in valid),
            'wall_sec_of_30s_uncaptured_runs': sorted(round(c['wall_sec'], 1) for _, c in valid if not c['captured']),
            'wall_sec_note': 'includes about 3 s launch overhead; time_scale 4 would give ~10 s, time_scale 1 ~33 s',
            'loadavg_before_range': [min(float(c['loadavg_before'][0]) for _, c in valid), max(float(c['loadavg_before'][0]) for _, c in valid)],
            'drop_pooled': r(1 - R / E) if E else None,
            'drop_pooled_bootstrap95': [r(boot[1]), r(boot[2])] if boot else None,
            'drop_per_run': [r(x['pooled']) for x in drops if x['pooled'] is not None],
            'in_range_expected_total': E, 'in_range_received_total': R,
            'receivers_without_status': sum(x['receivers_without_status'] for x in drops),
            'age_events_n': len(ages_all),
            'age_median_s': r(quantile(ages_all, .5)), 'age_p95_s': r(quantile(ages_all, .95)),
            'age_p99_s': r(quantile(ages_all, .99)), 'age_max_s': r(max(ages_all) if ages_all else None),
            'age_p95_per_run_range_s': [r(min(per_run_p95)), r(max(per_run_p95))] if per_run_p95 else None,
            'age_frac_gt_sighting_timeout': r(sum(a > SIGHTING_TIMEOUT for a in ages_all) / len(ages_all)) if ages_all else None,
            'age_accepted_median_s': r(quantile(ages_acc, .5)), 'age_accepted_p95_s': r(quantile(ages_acc, .95)),
            'reasons_all_nonself': reason_table(nonself),
            'reasons_in_range_only': reason_table(in_range),
            'event_rows_total': len(events),
        }
    return {'name': 'E1b', 'status': 'POST-HOC follow-up diagnostic, not pre-registered',
            'sighting_timeout_s': SIGHTING_TIMEOUT, 'cells': cells,
            'git_commits': sorted({json.loads(p.read_text())['git_commit'] for p in runs_dir.glob('*/run_config.json')}),
            'fingerprints': sorted({json.loads(p.read_text())['source_fingerprint_sha256'] for p in runs_dir.glob('*/run_config.json')}),
            'invalid_runs': [p.parent.name for p in runs_dir.glob('*/run_config.json')
                             if json.loads(p.read_text())['validity'] != 'valid']}, ages_by_cell


def plot(ages, media):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    media.mkdir(parents=True, exist_ok=True)
    f, axs = plt.subplots(1, 2, figsize=(11, 4.3), sharey=True)
    for ax, scene, title in zip(axs, ('S12ff', 'S12rt'), ('fast-forward (time_scale 4), 8 runs per mode', 'real time (time_scale 1), 4 runs per mode')):
        for mode, col in (('ros-d1', '#D55E00'), ('ros-d10', '#E69F00')):
            a = sorted(ages[f'{scene}/{mode}'])
            n = len(a)
            if not n:
                continue
            ccdf = [1 - (i + 1) / n for i in range(n)]
            ax.step([max(x, 0.0) + 1e-3 for x in a], [max(c, 0.5 / n) for c in ccdf], where='post', color=col, lw=2, label=f'{mode} (n={n:,})')
        ax.axvline(SIGHTING_TIMEOUT, color='#555', ls='--', lw=1)
        ax.text(SIGHTING_TIMEOUT * 1.05, 0.3, 'sighting_timeout\n0.6 s', fontsize=8, color='#555')
        ax.set_xscale('log'); ax.set_yscale('log')
        ax.set_xlabel('sighting age at receive (sim s, +1 ms offset for log axis)')
        ax.set_title(title, fontsize=10); ax.legend(frameon=False, loc='lower left')
        ax.spines[['top', 'right']].set_visible(False)
    axs[0].set_ylabel('fraction of delivered sightings older than x')
    f.suptitle('E1b (post-hoc diagnostic): age of delivered, non-self sightings, S12', fontsize=11)
    f.tight_layout(); f.savefig(media / 'relay_e1b_age.png', dpi=140); plt.close(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', type=Path)
    ap.add_argument('--summary', type=Path, required=True)
    ap.add_argument('--media-dir', type=Path, default=None)
    a = ap.parse_args()
    s, ages = analyze(a.runs)
    a.summary.write_text(json.dumps(s, indent=2, sort_keys=True))
    if a.media_dir:
        plot(ages, a.media_dir)
    for k, v in s['cells'].items():
        print(k, {x: v[x] for x in ('valid', 'captures', 'drop_pooled', 'drop_pooled_bootstrap95', 'age_median_s', 'age_p95_s', 'age_max_s', 'age_frac_gt_sighting_timeout')})
        print('   reasons(all nonself)', {k2: v2[1] if isinstance(v2, list) else v2 for k2, v2 in v['reasons_all_nonself'].items()})


if __name__ == '__main__':
    main()
