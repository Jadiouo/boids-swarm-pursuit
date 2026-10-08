"""Frame-capture hook for the demo GIF (no source changes to the package).

Put this directory on PYTHONPATH and set DEMO_FRAME_DIR. Python imports
`sitecustomize` at startup of every process; this one only acts inside the
`pygame_sim` process, where it wraps pygame.display.flip so each Nth
presented frame is also saved as a PNG. Used by tools/record_demo.sh.
"""
import os
import sys

_dir = os.environ.get('DEMO_FRAME_DIR')
if _dir and 'pygame_sim' in os.path.basename(sys.argv[0] if sys.argv else ''):
    import pygame
    os.makedirs(_dir, exist_ok=True)
    _every = int(os.environ.get('DEMO_EVERY', '4'))     # 60 fps / 4 = 15 fps
    _max = int(os.environ.get('DEMO_MAX_FRAMES', '400'))
    _orig = pygame.display.flip
    _n = {'flip': 0, 'saved': 0}

    def _flip(*a, **k):
        r = _orig(*a, **k)
        _n['flip'] += 1
        if _n['flip'] % _every == 0 and _n['saved'] < _max:
            surf = pygame.display.get_surface()
            if surf is not None:
                pygame.image.save(surf, os.path.join(
                    _dir, f'f_{_n["saved"]:05d}.png'))
                _n['saved'] += 1
        return r

    pygame.display.flip = _flip
