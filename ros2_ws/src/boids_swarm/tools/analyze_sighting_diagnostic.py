#!/usr/bin/env python3
"""Offline analysis of run_sighting_diagnostic.py output (raw JSONL only).
usage: analyze_sighting_diagnostic.py RUNS_DIR OUT_SUMMARY_JSON"""
import collections, json, math, sys
from pathlib import Path


def wilson(k, n, z=1.96):
    if n == 0:
        return None
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [round(max(0, c - h), 3), round(min(1, c + h), 3)]


def rows(path):
    return [json.loads(l) for l in path.read_text().splitlines() if l]


key = lambda r: (r['episode_id'], r['sender_id'], r['sequence'])


def analyze(d):
    cfg = json.loads((d / 'run_config.json').read_text())
    n, mode = cfg['num_agents'], cfg['mode']
    st = rows(d / 'track_status.jsonl')
    cyc = [s for s in st if s.get('event') == 'cycle' and s.get('own_pose_fresh')]
    out = {
        'run': d.name, 'mode': mode, 'num_agents': n, 'seed': cfg['seed'],
        'domain': cfg['ros_domain_id'], 'outcome': cfg['capture_result'],
        'capture_time_sec': cfg['capture_time_sec'], 'wall_sec': cfg['wall_sec'],
        'cpu_sec': cfg['children_cpu_sec'],
        'cpu_cores_avg': round(cfg['children_cpu_sec'] / cfg['wall_sec'], 2),
        'loadavg_before': cfg['loadavg_before'], 'loadavg_after': cfg['loadavg_after'],
        'exit_code': cfg['exit_code'], 'traceback_in_log': cfg['traceback_in_log'],
        'track_valid_fraction': (round(sum(bool(s['belief_valid']) for s in cyc) / len(cyc), 4) if cyc else None),
        'track_valid_denominator': len(cyc),
    }
    # status-channel completeness (reliable depth-20 pub, collector depth 300)
    per = collections.defaultdict(list)
    for s in st:
        per[s['agent']].append(s['control_cycle_sequence'])
    miss = sum(max(v) - len(set(v)) for v in per.values()) if per else 0
    tot = sum(max(v) for v in per.values()) if per else 0
    out['status_channel_missing'] = f'{miss}/{tot}'
    if mode != 'ros':
        out['local_sightings'] = len(rows(d / 'local_sightings.jsonl'))
        return out
    loc = {key(r) for r in rows(d / 'local_sightings.jsonl')}
    d1 = {key(r) for r in rows(d / 'shared_sightings_depth1.jsonl')}
    d3 = {key(r) for r in rows(d / 'shared_sightings_depth300.jsonl')}
    ev = rows(d / 'relay_events.jsonl')
    ek = collections.Counter(key(e) for e in ev)
    pub_low = d3 | set(ek)            # published, seen by collector deep or any receiver
    pub_up = pub_low | loc            # + every sim sighting (direct inbox rejects would not be published)
    E = len(ev)
    self_ev = sum(e['reason'] == 'self_originated' for e in ev)
    reasons = collections.Counter(e['reason'] for e in ev)
    recv_pairs = collections.Counter((e['receiver_id'], key(e)) for e in ev)
    dup_events = sum(v - 1 for v in recv_pairs.values())
    nonself = E - self_ev
    inrange = sum(1 for e in ev if e['reason'] != 'self_originated'
                  and e['radio_distance'] is not None and e['radio_distance'] <= cfg['radio_range'])
    out.update({
        'sim_sightings_local': len(loc), 'shared_keys_depth300': len(d3),
        'shared_keys_depth1': len(d1), 'shared_keys_with_any_event': len(ek),
        'published_keys_low': len(pub_low), 'published_keys_up': len(pub_up),
        'relay_events_total': E, 'relay_events_self': self_ev, 'relay_events_nonself': nonself,
        'duplicate_receiver_events': dup_events,
        'expected_receipts_low_denominator': n * len(pub_low),
        'delivery_all_receivers': round(E / (n * len(pub_low)), 4),
        'loss_all_receivers_upper_published': round(1 - E / (n * len(pub_up)), 4) if pub_up else None,
        'loss_all_receivers': round(1 - E / (n * len(pub_low)), 4),
        'loss_nonself_receivers': round(1 - nonself / ((n - 1) * len(pub_low)), 4),
        'loss_self_receiver': round(1 - self_ev / len(pub_low), 4),
        'depth1_collector_key_coverage': round(len(d1) / len(d3), 4) if d3 else None,
        'event_multiplicity': dict(sorted(collections.Counter(ek.values()).items())),
        'reasons': dict(reasons),
        'nonself_events_within_radio_range': inrange,
        'accepted_events': reasons.get('accepted', 0),
    })
    return out


def main():
    runs = Path(sys.argv[1])
    res = [analyze(d) for d in sorted(runs.iterdir()) if (d / 'run_config.json').exists()]
    groups = collections.defaultdict(list)
    for r in res:
        groups[(r['mode'], r['num_agents'])].append(r)
    agg = {}
    for (m, n), rs in sorted(groups.items()):
        k = sum(r['outcome'] == 'captured' or (r['outcome'] not in (None, 'timeout')) for r in rs)
        agg[f'{m}-n{n}'] = {
            'runs': len(rs), 'captured': k, 'wilson95': wilson(k, len(rs)),
            'outcomes': collections.Counter(r['outcome'] for r in rs),
            'mean_wall_sec': round(sum(r['wall_sec'] for r in rs) / len(rs), 2),
            'mean_cpu_cores': round(sum(r['cpu_cores_avg'] for r in rs) / len(rs), 2)}
    Path(sys.argv[2]).write_text(json.dumps({'by_group': agg, 'runs': res}, indent=2, sort_keys=True, default=dict))
    for r in res:
        print({k: r[k] for k in ('run', 'outcome', 'capture_time_sec', 'wall_sec', 'cpu_cores_avg', 'track_valid_fraction') } |
              ({k: r[k] for k in ('published_keys_low', 'relay_events_total', 'loss_all_receivers', 'loss_nonself_receivers', 'loss_self_receiver', 'depth1_collector_key_coverage')} if r['mode'] == 'ros' else {}))
    print(json.dumps(agg, default=dict))


if __name__ == '__main__':
    main()
