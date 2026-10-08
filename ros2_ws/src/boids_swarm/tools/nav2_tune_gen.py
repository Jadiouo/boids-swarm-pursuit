#!/usr/bin/env python3
"""Write a tuning variant of config/nav2_target.yaml.

    nav2_tune_gen.py OUT.yaml 'FollowPath.vx_std=1.5' \\
        'FollowPath.PathFollowCritic.cost_weight=10' \\
        'local_costmap.local_costmap.inflation_layer.inflation_radius=0.5'

Keys are dotted paths below each top-level node's `ros__parameters`
(controller_server / planner_server / local_costmap.local_costmap / ...);
the first segment is the top-level node, then (for the two costmaps) the
inner node name. Values are YAML-parsed.
"""
import os
import sys

import yaml

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                    'config', 'nav2_target.yaml')


def main():
    out, *sets = sys.argv[1:]
    doc = yaml.safe_load(open(BASE))
    for item in sets:
        if item.startswith('REPLACE_FOLLOWPATH='):    # swap the whole controller
            doc['controller_server']['ros__parameters']['FollowPath'] = \
                yaml.safe_load(open(item.split('=', 1)[1]))
            continue
        key, val = item.split('=', 1)
        parts = key.split('.')
        if parts[0] == 'FollowPath':        # shorthand
            parts = ['controller_server'] + parts
        top = parts[0]
        node = doc[top]
        if top in ('local_costmap', 'global_costmap'):
            node = node[parts[1]]
            parts = parts[1:]
        node = node['ros__parameters']
        for p in parts[1:-1]:
            node = node.setdefault(p, {})
        new = yaml.safe_load(val)
        old = node.get(parts[-1])
        if isinstance(old, float) and isinstance(new, int):
            new = float(new)        # Nav2 rejects int for a double param
        node[parts[-1]] = new
    # drop the placeholder-doc: plain yaml dump
    with open(out, 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False)


if __name__ == '__main__':
    main()
