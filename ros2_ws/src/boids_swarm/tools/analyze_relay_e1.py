#!/usr/bin/env python3
"""Offline analysis for the phase-2 experiment (R-05). Raw run dirs only.
usage: analyze_relay_e1.py RUNS_DIR --summary summary.json [--media-dir docs/media]
"""
import argparse, collections, gzip, json, math, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from boids_swarm import relay_stats as rs
from boids_swarm import experiment_matrix as em

MODES = ['off', 'oracle-mh', 'oracle-1h', 'ros-d1', 'ros-d10']
SCENES = ['S12', 'S4']
TV_WINDOW = 5.0      # seconds; each run uses min(episode length, 5 s)
BOOT = 10000


def load_status(d):
    with gzip.open(d / 'track_status.jsonl.gz', 'rt') as f:
        return [json.loads(l) for l in f if l.strip()]


def drop_for_run(d, cfg):
    """Per-receiver drop for one ros run, or None."""
    exp_file = d / 'relay' / 'expected_receivers.jsonl'
    if not exp_file.exists():
        return None
    published = set()
    for p in (d / 'relay').glob('published_agent*.jsonl'):
        for l in p.read_text().splitlines():
            if l:
                ep, sender, seq, _ = json.loads(l)
                published.add((ep, sender, seq))
    events = []
    for l in exp_file.read_text().splitlines():
        if l:
            r = json.loads(l)
            if tuple(r['k']) in published:       # only keys that reached the wire
                events.append(r)
    st = load_status(d)
    last = {}
    for s in st:
        a = s['agent']
        if a not in last or s['control_cycle_sequence'] > last[a]['control_cycle_sequence']:
            last[a] = s
    cutoff = {a: s['stamp'] for a, s in last.items()}
    received = {a: s['sightings_received_in_range'] for a, s in last.items()}
    expected = rs.expected_in_range_counts(events, cutoff)
    all_receivers = {r for ev in events for r in ev['exp']}
    missing = sorted(all_receivers - set(last))
    missing_expected = sum(1 for ev in events for r in ev['exp'] if r in missing)
    pos = {a: v for a, v in expected.items() if v > 0}
    return {'run_expected_total': sum(pos.values()),
            'run_received_total': sum(received.get(a, 0) for a in pos),
            'receivers_without_status': len(missing),
            'expected_events_for_missing_receivers': missing_expected,'expected': expected, 'received': received,
            'per_receiver': rs.per_receiver_drop(expected, received),
            'pooled': rs.pooled_drop(expected, received),
            'published_keys': len(published), 'expected_events_keys': len(events),
            'self_received': sum(s['sightings_received_self'] for s in last.values()),
            'out_of_range_received': sum(s['sightings_received_out_of_range'] for s in last.values()),
            'unclassified_received': sum(s['sightings_received_unclassified'] for s in last.values()),
            'excess_agents': [a for a in expected if received.get(a, 0) > expected[a]]}


def fmt(x, nd=3):
    return None if x is None else round(x, nd)


def analyze(runs_dir):
    runs = []
    for d in sorted(runs_dir.iterdir()):
        if not (d / 'run_config.json').exists():
            continue
        cfg = json.loads((d / 'run_config.json').read_text())
        rows = load_status(d)
        L = cfg['capture_time_sec'] if cfg['captured'] else cfg['common']['time_limit']
        cfg['_L'] = L
        cfg['_tv_win'], cfg['_tv_win_den'] = rs.window_fraction(rows, min(L, TV_WINDOW))
        cfg['_drop'] = drop_for_run(d, cfg) if cfg['mode'].startswith('ros') and cfg['validity'] == 'valid' else None
        runs.append(cfg)
    cells = {}
    for scene in SCENES:
        for mode in MODES:
            rr = [r for r in runs if r['scene'] == scene and r['mode'] == mode]
            if not rr:
                continue
            valid = [r for r in rr if r['validity'] == 'valid']
            k = sum(r['captured'] for r in valid)
            times = [r['capture_time_sec'] for r in valid if r['captured']]
            twin = [r['_tv_win'] for r in valid if r['_tv_win'] is not None]
            tv = [r['track_valid_fraction'] for r in valid if r['track_valid_fraction'] is not None]
            cell = {'runs': len(rr), 'valid': len(valid), 'invalid': len(rr) - len(valid),
                    'invalid_reasons': dict(collections.Counter(r['invalid_reason'] for r in rr if r['validity'] != 'valid')),
                    'reliability': em.cell_reliability(len(valid), len(rr) - len(valid)),
                    'captures': k, 'capture_rate': fmt(k / len(valid)) if valid else None,
                    'wilson95': [fmt(x) for x in rs.wilson(k, len(valid))] if valid else None,
                    'capture_time_median': fmt(rs.median(times), 2), 'capture_times': sorted(round(t, 2) for t in times),
                    # full-episode value kept for continuity; its denominator is
                    # the episode length, so it is mechanically tied to capture time
                    'track_valid_full_episode_median': fmt(rs.median(tv)),
                    'track_valid_full_episode_per_run': [fmt(t) for t in tv],
                    # primary: fixed window = first min(episode length, 5 s)
                    'track_valid_median': fmt(rs.median(twin)), 'track_valid_per_run': [fmt(t) for t in twin],
                    'track_valid_window_sec': TV_WINDOW,
                    'km_times': [round(r['_L'], 2) for r in valid],
                    'km_observed': [bool(r['captured']) for r in valid],
                    'seeds_captured': sorted(r['seed'] for r in valid if r['captured'])}
            drops = [r['_drop'] for r in valid if r['_drop']]
            if mode.startswith('ros'):
                E = collections.Counter(); R = collections.Counter()
                for dd in drops:
                    for a, v in dd['expected'].items():
                        E[(id(dd), a)] += v; R[(id(dd), a)] += dd['received'].get(a, 0)
                pooled = rs.pooled_drop(E, R)
                boot = rs.bootstrap_pooled_drop_ci(
                    [(dd['run_expected_total'], dd['run_received_total']) for dd in drops], BOOT, 20261010)
                per_run = [dd['pooled'] for dd in drops if dd['pooled'] is not None]
                cell.update({'drop_pooled': fmt(pooled, 4), 'drop_per_run_median': fmt(rs.median(per_run), 4),
                             'drop_per_run_min': fmt(min(per_run), 4) if per_run else None,
                             'drop_per_run_max': fmt(max(per_run), 4) if per_run else None,
                             'drop_per_run': [fmt(x, 4) for x in per_run],
                             'in_range_expected_total': sum(E.values()), 'in_range_received_total': sum(R.values()),
                             'receivers_without_status': sum(dd['receivers_without_status'] for dd in drops),
                             'expected_events_for_receivers_without_status': sum(dd['expected_events_for_missing_receivers'] for dd in drops),
                             'drop_pooled_bootstrap95': [fmt(x, 4) for x in boot[1:]] if boot else None,
                             'bootstrap': {'unit': 'run', 'resamples': BOOT, 'seed': 20261010, 'method': 'percentile'},
                             'receiver_agents_received_gt_expected': sum(len(dd['excess_agents']) for dd in drops),
                             'self_received_total': sum(dd['self_received'] for dd in drops),
                             'out_of_range_received_total': sum(dd['out_of_range_received'] for dd in drops),
                             'unclassified_received_total': sum(dd['unclassified_received'] for dd in drops),
                             'runs_with_zero_expected': sum(1 for r in valid if mode.startswith('ros') and (r['_drop'] is None or r['_drop']['pooled'] is None))})
            cells[f'{scene}/{mode}'] = cell
    fps = sorted({r['source_fingerprint_sha256'] for r in runs})
    summary = {'cells': cells, 'n_runs': len(runs), 'n_invalid': sum(r['validity'] != 'valid' for r in runs),
               'fingerprints': fps, 'git_commits': sorted({r['git_commit'] for r in runs}),
               'dirty_runs': [r['run_id'] for r in runs if r['working_tree_dirty']],
               'traceback_runs': [r['run_id'] for r in runs if r['traceback_in_log']],
               'hypotheses': hypotheses(cells),
               'km_exploratory': km_summary(cells),
               'power': power_summary()}
    return summary


def km_summary(cells):
    out = {'note': 'exploratory, not pre-registered; Kaplan-Meier, right-censored at 30 s; Greenwood 95% band', 'S12': {}}
    for m in MODES:
        c = cells.get(f'S12/{m}')
        if not c:
            continue
        curve = rs.km_curve(c['km_times'], c['km_observed'])
        row = {'curve': [[round(x, 4) for x in r] for r in curve]}
        for t in (10, 20, 30):          # shown at descriptive checkpoints only; no test, no p value
            sv, lo, hi = rs.km_at(curve, t)
            row[f'cum_capture_at_{t}s'] = [round(1 - sv, 3), round(1 - hi, 3), round(1 - lo, 3)]
        out['S12'][m] = row
    return out


def power_summary():
    res = {'method': 'two-sided two-proportion z test, alpha 0.05, power 0.80, equal n per cell; '
                     'n = (z_a*sqrt(2*pb*(1-pb)) + z_b*sqrt(p1(1-p1)+p2(1-p2)))^2 / (p1-p2)^2, pb = (p1+p2)/2; '
                     'cc = Fleiss continuity-corrected. Planning numbers from the E1 point estimates, not a post-hoc power of the observed result.'}
    for name, (p1, p2) in {'0.25_vs_0.45 (off vs ros-d10 point estimates)': (0.25, 0.45),
                           '0.35_vs_0.45 (ros-d1 vs ros-d10 point estimates)': (0.35, 0.45)}.items():
        n = rs.n_per_group_two_proportions(p1, p2)
        ncc = rs.n_per_group_two_proportions(p1, p2, continuity=True)
        res[name] = {'n_per_cell': math.ceil(n), 'n_per_cell_continuity_corrected': math.ceil(ncc)}
    return res


def overlap(a, b):
    return not (a[1] < b[0] or b[1] < a[0])


def hypotheses(c):
    out = {}
    for sc in SCENES:
        d1, d10 = c.get(f'{sc}/ros-d1'), c.get(f'{sc}/ros-d10')
        if d1 and d10 and d1.get('drop_pooled') is not None and d10.get('drop_pooled') is not None:
            diff = d1['drop_pooled'] - d10['drop_pooled']
            out[f'H1_{sc}'] = {'drop_d1': d1['drop_pooled'], 'drop_d10': d10['drop_pooled'],
                               'diff_pp': round(diff * 100, 1), 'criterion': 'd1 - d10 > 10 pp',
                               'supported': diff > 0.10}
    sc = 'S12'
    chain = ['off', 'ros-d1', 'oracle-1h', 'oracle-mh']
    chain10 = ['off', 'ros-d10', 'oracle-1h', 'oracle-mh']
    for name, ch in (('with_ros_d1', chain), ('with_ros_d10', chain10)):
        cs = [c.get(f'{sc}/{m}') for m in ch]
        if all(cs):
            rates = [x['capture_rate'] for x in cs]
            monotone = all(rates[i] <= rates[i + 1] for i in range(3))
            pairs = {f'{ch[i]}-vs-{ch[j]}': ('distinguishable' if not overlap(cs[i]['wilson95'], cs[j]['wilson95']) else 'indistinguishable')
                     for i in range(4) for j in range(i + 1, 4)}
            out[f'H2_{sc}_{name}'] = {'order': ch, 'rates': rates, 'point_estimates_monotone': monotone, 'pairs_by_wilson_overlap': pairs}
    for sc in SCENES:
        o, d1, d10 = (c.get(f'{sc}/{m}') for m in ('oracle-1h', 'ros-d1', 'ros-d10'))
        if o and d1 and d10:
            g1 = abs(o['capture_rate'] - d1['capture_rate']); g10 = abs(o['capture_rate'] - d10['capture_rate'])
            out[f'H3_{sc}'] = {'gap_d1_vs_1h': round(g1, 3), 'gap_d10_vs_1h': round(g10, 3),
                               'd10_gap_smaller': g10 < g1,
                               'wilson_overlap_d10_vs_1h': overlap(d10['wilson95'], o['wilson95']),
                               'wilson_overlap_d1_vs_1h': overlap(d1['wilson95'], o['wilson95'])}
    return out


COL = {'off': '#7a7a7a', 'oracle-mh': '#0072B2', 'oracle-1h': '#56B4E9', 'ros-d1': '#D55E00', 'ros-d10': '#E69F00'}


def plots(summary, media):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    media.mkdir(parents=True, exist_ok=True)
    c = summary['cells']
    def fig(ax_n=2):
        f, axs = plt.subplots(1, ax_n, figsize=(11, 4.2)); return f, axs
    # capture rate
    f, axs = fig()
    for ax, sc in zip(axs, SCENES):
        for i, m in enumerate(MODES):
            x = c.get(f'{sc}/{m}')
            if not x or x['capture_rate'] is None: continue
            lo, hi = x['wilson95']; r = x['capture_rate']
            ax.bar(i, r, color=COL[m], width=0.7)
            ax.errorbar(i, r, yerr=[[r - lo], [hi - r]], color='#222', capsize=4, lw=1.5)
            ax.text(i, min(hi + 0.03, 1.02), f"{x['captures']}/{x['valid']}", ha='center', fontsize=9)
        ax.set_xticks(range(len(MODES))); ax.set_xticklabels(MODES, rotation=20)
        ax.set_ylim(0, 1.12); ax.set_ylabel('capture rate (Wilson 95%)')
        ax.set_title(f'{sc} ({12 if sc=="S12" else 4} agents, 30 s)')
        ax.spines[['top', 'right']].set_visible(False)
    f.tight_layout(); f.savefig(media / 'relay_e1_capture_rate.png', dpi=140); plt.close(f)
    # drop rate
    f, axs = fig()
    for ax, sc in zip(axs, SCENES):
        for i, m in enumerate(['ros-d1', 'ros-d10']):
            x = c.get(f'{sc}/{m}')
            if not x or x.get('drop_pooled') is None: continue
            ax.bar(i, x['drop_pooled'], color=COL[m], width=0.6)
            pr = x['drop_per_run']
            ax.scatter([i + (j % 7 - 3) * 0.04 for j in range(len(pr))], pr, color='#222', s=12, zorder=3)
            ax.text(i, max(max(pr), x['drop_pooled']) + 0.04, f"{x['drop_pooled']*100:.1f}%", ha='center')
        ax.set_xticks([0, 1]); ax.set_xticklabels(['ros-d1', 'ros-d10'])
        ax.set_ylim(0, 1.05); ax.set_ylabel('per-receiver drop rate (in range)')
        ax.set_title(f'{sc}: bars = pooled, dots = per run'); ax.spines[['top', 'right']].set_visible(False)
    f.tight_layout(); f.savefig(media / 'relay_e1_drop_rate.png', dpi=140); plt.close(f)
    # capture time
    f, axs = fig()
    import random
    rnd = random.Random(1)
    for ax, sc in zip(axs, SCENES):
        for i, m in enumerate(MODES):
            x = c.get(f'{sc}/{m}')
            if not x: continue
            ts = x['capture_times']
            ax.scatter([i + rnd.uniform(-.15, .15) for _ in ts], ts, color=COL[m], s=28, zorder=3)
            if x['capture_time_median'] is not None:
                ax.hlines(x['capture_time_median'], i - .3, i + .3, color='#222', lw=2)
            ax.text(i, 31.5, f"{x['captures']}/{x['valid']}", ha='center', fontsize=9)
        ax.axhline(30, color='#bbb', lw=1, ls='--')
        ax.set_xticks(range(len(MODES))); ax.set_xticklabels(MODES, rotation=20)
        ax.set_ylim(0, 34); ax.set_ylabel('capture time of captured runs (s); bar = median')
        ax.set_title(f'{sc}'); ax.spines[['top', 'right']].set_visible(False)
    f.tight_layout(); f.savefig(media / 'relay_e1_capture_time.png', dpi=140); plt.close(f)
    # track valid
    f, axs = fig()
    for ax, sc in zip(axs, SCENES):
        for i, m in enumerate(MODES):
            x = c.get(f'{sc}/{m}')
            if not x: continue
            v = x['track_valid_per_run']
            ax.scatter([i + rnd.uniform(-.15, .15) for _ in v], v, color=COL[m], s=22)
            if x['track_valid_median'] is not None:
                ax.hlines(x['track_valid_median'], i - .3, i + .3, color='#222', lw=2)
        ax.set_xticks(range(len(MODES))); ax.set_xticklabels(MODES, rotation=20)
        ax.set_ylim(0, 1.05); ax.set_ylabel(f'track-valid, first min(episode, {TV_WINDOW:g} s); bar = median'); ax.set_title(sc)
        ax.spines[['top', 'right']].set_visible(False)
    f.tight_layout(); f.savefig(media / 'relay_e1_track_valid.png', dpi=140); plt.close(f)
    # exploratory cumulative capture (Kaplan-Meier, right-censored at 30 s)
    f, ax = plt.subplots(figsize=(7.4, 4.8))
    for m in MODES:
        x = c.get(f'S12/{m}')
        if not x:
            continue
        curve = rs.km_curve(x['km_times'], x['km_observed'])
        ts = [r[0] for r in curve] + [30.0]
        cum = [1 - r[1] for r in curve] + [1 - curve[-1][1]]
        lo = [1 - r[3] for r in curve] + [1 - curve[-1][3]]
        hi = [1 - r[2] for r in curve] + [1 - curve[-1][2]]
        ax.step(ts, cum, where='post', color=COL[m], lw=2, label=f"{m} ({x['captures']}/{x['valid']})")
        ax.fill_between(ts, lo, hi, step='post', color=COL[m], alpha=0.10, lw=0)
    ax.set_xlim(0, 30); ax.set_ylim(0, 1.0)
    ax.set_xlabel('sim time (s); runs without capture are censored at 30 s')
    ax.set_ylabel('cumulative capture probability (Kaplan-Meier)')
    ax.set_title('S12, 20 runs per mode. Exploratory (not pre-registered).\nBands: Greenwood 95%, heavily overlapping', fontsize=10)
    ax.legend(loc='upper left', frameon=False); ax.spines[['top', 'right']].set_visible(False)
    f.tight_layout(); f.savefig(media / 'relay_e1_cumulative_capture.png', dpi=140); plt.close(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', type=Path)
    ap.add_argument('--summary', type=Path, required=True)
    ap.add_argument('--media-dir', type=Path, default=None)
    a = ap.parse_args()
    s = analyze(a.runs)
    a.summary.write_text(json.dumps(s, indent=2, sort_keys=True))
    if a.media_dir:
        plots(s, a.media_dir)
    for k, v in s['cells'].items():
        print(k, {x: v.get(x) for x in ('valid', 'invalid', 'captures', 'wilson95', 'capture_time_median', 'track_valid_median', 'drop_pooled', 'in_range_expected_total', 'in_range_received_total')})
    print(json.dumps(s['hypotheses'], indent=1))


if __name__ == '__main__':
    main()
