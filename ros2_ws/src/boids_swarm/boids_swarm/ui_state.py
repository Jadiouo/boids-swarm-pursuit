"""What a panel click *means* — pure decision logic, no ROS, no pygame.

The panel produces `(scope, key, value)` changes. Most are live parameter
edits that go straight to the nodes. Fields marked "restart" must not: they
are only **staged** and take effect when the user presses *Apply & restart*
(or picks a mode, which applies at once). `PanelState` owns the applied /
staged split and turns a change into a decision the sim executes:

    ('send', scope, key, value)   live parameter edit -> ParamBridge
    ('stage', key)                remembered, nothing sent
    ('restart', overrides)        rebuild the stack with these changes
    ('action', name)              reset / pause ...
    ('notice', text)              refused, show the reason
"""

from . import stack_config as sc
from . import ui

# Panel scope for fields that need a restart (staged, not sent).
RESTART_SCOPE = ui.RESTART


class PanelState:

    def __init__(self, applied, available=sc.ALL_EVADERS, managed=True):
        self.applied = dict(applied)     # what the running stack uses
        self.staged = {}                 # edits waiting for Apply
        self.available = tuple(available)
        self.managed = managed
        self.notice = ''

    # --- values -----------------------------------------------------------
    def value(self, key):
        return self.staged.get(key, self.applied.get(key))

    def is_pending(self, key):
        return key in self.staged

    @property
    def pending_count(self):
        return len(self.staged)

    @property
    def mode(self):
        return sc.mode_of(self.applied)

    def target_cfg(self, overrides=None):
        cfg = dict(self.applied)
        cfg.update(self.staged)
        cfg.update(overrides or {})
        return cfg

    def commit(self, cfg):
        """The stack now runs `cfg`."""
        self.applied = dict(cfg)
        self.staged.clear()

    # --- staging ----------------------------------------------------------
    def stage(self, key, value):
        """Remember an edit; undo it when it equals what is running.
        Keeps `sharing_mode=ros` and `perception=sensor` consistent by
        moving the partner field (and saying so)."""
        self._set(key, value)
        eff = self.target_cfg()
        if key == 'sharing_mode' and value == 'ros' \
                and eff['perception'] != 'sensor':
            self._set('perception', 'sensor')
            self.notice = 'sharing ros needs sensor perception: set it too'
        elif key == 'perception' and value == 'perfect' \
                and eff['sharing_mode'] == 'ros':
            self._set('sharing_mode', 'legacy')
            self.notice = 'perfect perception cannot use ros sharing: ' \
                          'sharing set to legacy'

    def _set(self, key, value):
        if value == self.applied.get(key):
            self.staged.pop(key, None)
        else:
            self.staged[key] = value

    # --- decisions --------------------------------------------------------
    def handle(self, change):
        scope, key, value = change
        self.notice = ''
        if scope == ui.RESTART:
            self.stage(key, value)
            return ('stage', key)
        if scope == ui.ACTION:
            return self._action(key, value)
        if scope == ui.TARGET and key == 'evader':
            return self._evader(value)
        return ('send', scope, key, value)

    def _action(self, key, value):
        if key == 'mode':
            if not self.managed:
                return self._unmanaged()
            return ('restart', sc.mode_preset(value, self.available))
        if key == 'apply':
            if not self.managed:
                return self._unmanaged()
            if not self.staged:
                self.notice = 'nothing to apply'
                return ('notice', self.notice)
            return ('restart', {})
        return ('action', key)

    def _evader(self, value):
        """Brains that only need a parameter swap change live; anything
        involving Nav2 changes which processes exist, so it restarts."""
        involves_nav2 = value == 'nav2' or self.applied.get('evader') == 'nav2'
        if not involves_nav2:
            return ('send', ui.TARGET, 'evader', value)
        if not self.managed:
            return self._unmanaged()
        if value == 'nav2':
            return ('restart', sc.mode_preset(sc.NAV2, self.available))
        return ('restart', {'evader': value})

    def _unmanaged(self):
        self.notice = ('stack was not started by the sim: restart-type '
                       'changes need a re-launch')
        return ('notice', self.notice)


def status_view(sup, managed, notice='', mode_label=''):
    """What the StatusBar shows, from a StackSupervisor.status() dict."""
    if not managed:
        return {'state': 'unmanaged',
                'text': 'Stack: started externally',
                'note': notice or 'mode switching needs the sim to own it',
                'frac': None}
    st = sup.get('state', 'idle')
    left, eta = sup.get('warm_left', 0.0), sup.get('eta', 0.0)
    frac = None
    note = notice
    if st == 'starting':
        text = f'Starting stack... ~{left:.0f}s' if eta else 'Starting stack...'
        if sup.get('label') == 'nav2':
            text = f'Warming up Nav2... ~{left:.0f}s'
        frac = (1.0 - left / eta) if eta > 0 else None
    elif st == 'warming':
        text, frac = 'Stack up, settling...', 1.0
    elif st == 'running':
        text = f'Running: {mode_label}' if mode_label else 'Running'
    elif st == 'stopping':
        text = 'Stopping previous stack...'
    elif st == 'failed':
        first = (sup.get('detail') or '').splitlines()
        text = 'FAILED: ' + (first[0] if first else 'unknown')
        note = (first[-1] if len(first) > 1 else '') or notice
    else:
        text = 'Stack idle'
    return {'state': st, 'text': text, 'note': note, 'frac': frac}
