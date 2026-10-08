"""Regenerate golden/pursuit_launch.json (only when the *intended*
headless / ui:=false behaviour of pursuit.launch.py changes):

    python3 test/gen_golden.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from launch_recorder import dumps, record  # noqa: E402
from golden_cases import CASES  # noqa: E402

if __name__ == '__main__':
    out = {name: record('pursuit.launch.py', args)
           for name, args in CASES.items()}
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'golden', 'pursuit_launch.json')
    with open(path, 'w') as f:
        f.write(dumps(out) + '\n')
    print('wrote', path, len(out), 'cases')
