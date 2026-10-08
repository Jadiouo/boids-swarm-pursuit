"""StackSupervisor — owns the restartable half of the game (control panel).

The pygame sim keeps its window for the whole session; the processes that
define a *mode* (boid controllers, target controller, Nav2) are started as
one `ros2 launch swarm_stack.launch.py` in their **own session / process
group**, so the supervisor can signal the entire tree at once and nothing
the launch spawns can escape it.

State machine (``state``)::

    idle ──start──▶ starting ──probe ok──▶ warming ──settle──▶ running
      ▲                │  ▲                    │                  │
      │      exit/timeout│                    │ exit             │ exit
      │                ▼  │                   ▼                  ▼
      └──stopped── stopping ◀──switch/stop── failed ◀────────────┘

* ``starting``  process spawned, waiting for the readiness ``probe``
                (controller nodes visible; Nav2 lifecycle active for nav2)
* ``warming``   probe passed, short settle delay
* ``running``   stack up; a dying launch process flips this to ``failed``
* ``stopping``  SIGINT sent to the group; SIGKILL after ``stop_grace``;
                a ``switch`` starts its replacement only when the old group
                is completely gone (one live stack at any time)
* ``failed``    ``detail`` carries the reason (+ tail of the stack log)

Everything is driven by ``tick()`` (called every frame from the sim loop),
so nothing blocks the render thread. Only ``shutdown()`` blocks, and only
for the atexit / signal-handler path. Popen, killpg, clock and sleep are
injectable so the whole machine is unit-tested without spawning anything.
All calls must come from one thread (the sim's main thread).
"""

import os
import signal
import subprocess
import tempfile
import time

IDLE, STARTING, WARMING = 'idle', 'starting', 'warming'
RUNNING, STOPPING, FAILED = 'running', 'stopping', 'failed'

LOG_TAIL_LINES = 8


class StackSupervisor:

    def __init__(self, popen=subprocess.Popen, killpg=os.killpg,
                 clock=time.monotonic, sleep=time.sleep, stop_grace=6.0,
                 kill_wait=3.0, ready_timeout=60.0, log_dir=None, env=None):
        self._popen, self._killpg = popen, killpg
        self._clock, self._sleep = clock, sleep
        self.stop_grace, self.kill_wait = stop_grace, kill_wait
        self.ready_timeout = ready_timeout
        self._log_dir = log_dir
        self._env = env
        self.state = IDLE
        self.detail = ''
        self.label = ''
        self.log_path = ''
        self._proc = None
        self._log = None
        self._pgid = None
        self._probe = None
        self._settle = 0.0
        self._eta = 0.0
        self._t_start = self._t_ready = self._t_stop = self._t_kill = 0.0
        self._killed = False
        self._pending = None        # (cmd, kwargs) to start once stopped
        self._after_stop = None     # ('failed', reason) once stopped
        self._probe_note = ''
        self._n = 0
        self.stacks_started = 0

    # --- public -----------------------------------------------------------
    @property
    def is_running(self):
        return self.state == RUNNING

    @property
    def busy(self):
        return self.state in (STARTING, WARMING, STOPPING)

    @property
    def warm_left(self):
        """Seconds until the stack is expected to be ready (UI countdown)."""
        if self.state in (RUNNING, IDLE, FAILED):
            return 0.0
        if self.state == STOPPING:
            return self._eta
        return max(0.0, self._eta - (self._clock() - self._t_start))

    def status(self):
        return {'state': self.state, 'detail': self.detail,
                'label': self.label, 'warm_left': self.warm_left,
                'eta': self._eta, 'log': self.log_path,
                'pid': self._proc.pid if self._proc else 0,
                'generation': self.stacks_started}

    def start(self, cmd, probe=None, settle_s=0.0, eta_s=0.0, label=''):
        """Spawn a stack. Refuses (via switch semantics) if one is alive."""
        if self._proc is not None and self._group_alive():
            return self.switch(cmd, probe=probe, settle_s=settle_s,
                               eta_s=eta_s, label=label)
        self._spawn(cmd, dict(probe=probe, settle_s=settle_s, eta_s=eta_s,
                              label=label))

    def switch(self, cmd, **kw):
        """Replace the running stack. The old one is stopped completely
        first; rapid repeated switches keep only the latest request."""
        kw.setdefault('probe', None)
        kw.setdefault('settle_s', 0.0)
        kw.setdefault('eta_s', 0.0)
        kw.setdefault('label', '')
        self._pending = (cmd, kw)
        self._after_stop = None
        if self.state == STOPPING:
            self._eta = kw['eta_s']
            return
        if self._proc is None or not self._group_alive():
            self._reap()
            self._spawn(*self._pending_pop())
            return
        self._begin_stop()

    def stop(self):
        self._pending = None
        if self._proc is None or not self._group_alive():
            self._reap()
            self.state = IDLE
            return
        self._begin_stop()

    def tick(self):
        now = self._clock()
        if self.state in (STARTING, WARMING, RUNNING):
            self._tick_alive(now)
        elif self.state == STOPPING:
            self._tick_stopping(now)

    def shutdown(self, timeout=12.0):
        """Blocking, idempotent: leave no process of ours behind."""
        self._pending = None
        self._after_stop = None
        if self._proc is not None and self._group_alive():
            self._signal(signal.SIGINT)
            deadline = self._clock() + min(timeout, self.stop_grace)
            while self._clock() < deadline and self._group_alive_poll():
                self._sleep(0.1)
            if self._group_alive_poll():
                self._signal(signal.SIGKILL)
                deadline = self._clock() + self.kill_wait
                while self._clock() < deadline and self._group_alive_poll():
                    self._sleep(0.05)
        self._reap()
        self.state = IDLE
        self.detail = ''

    # --- internals --------------------------------------------------------
    def _pending_pop(self):
        cmd, kw = self._pending
        self._pending = None
        return cmd, kw

    def _spawn(self, cmd, kw):
        self.label = kw['label']
        self._probe, self._settle = kw['probe'], kw['settle_s']
        self._eta = kw['eta_s']
        self.detail = ''
        self._probe_note = ''
        self._killed = False
        self._n += 1
        try:
            self._open_log()
            env = dict(os.environ if self._env is None else self._env)
            env['BOIDS_STACK_PARENT_PID'] = str(os.getpid())
            self._proc = self._popen(
                cmd, stdin=subprocess.DEVNULL, stdout=self._log,
                stderr=subprocess.STDOUT, start_new_session=True, env=env)
        except Exception as exc:                       # no ros2, no fds ...
            self._proc = None
            self._fail(f'could not start stack: {exc}')
            return
        self._pgid = self._proc.pid                    # session leader
        self._t_start = self._clock()
        self.stacks_started += 1
        self.state = STARTING

    def _open_log(self):
        self._close_log()
        d = self._log_dir or tempfile.gettempdir()
        os.makedirs(d, exist_ok=True)
        self.log_path = os.path.join(
            d, f'swarm_stack_{os.getpid()}_{self._n}.log')
        self._log = open(self.log_path, 'wb')

    def _close_log(self):
        if self._log is not None:
            try:
                self._log.close()
            except OSError:
                pass
            self._log = None

    def _tail(self):
        try:
            with open(self.log_path, 'rb') as f:
                lines = f.read().decode('utf-8', 'replace').splitlines()
        except OSError:
            return ''
        lines = [ln for ln in lines if ln.strip()]
        return '\n'.join(lines[-LOG_TAIL_LINES:])

    def _group_alive(self):
        if self._pgid is None:
            return False
        try:
            self._killpg(self._pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return False

    def _group_alive_poll(self):
        """Reap the leader (so it stops being a zombie) then test the group."""
        if self._proc is not None:
            self._proc.poll()
        return self._group_alive()

    def _signal(self, sig):
        try:
            self._killpg(self._pgid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _reap(self):
        self._close_log()
        self._proc = None
        self._pgid = None

    def _begin_stop(self):
        self.state = STOPPING
        self._t_stop = self._clock()
        self._killed = False
        self._signal(signal.SIGINT)

    def _fail(self, reason):
        if self._pgid is not None and self._group_alive():
            self._signal(signal.SIGKILL)
        tail = self._tail() if self.log_path and self._n else ''
        self.detail = reason + (f'\n{tail}' if tail else '')
        self._reap()
        self.state = FAILED

    def _tick_alive(self, now):
        code = self._proc.poll()
        if code is not None:
            self._fail(f'stack exited with code {code}')
            return
        if self.state == STARTING:
            ok = True
            if self._probe is not None:
                try:
                    ok = bool(self._probe())
                except Exception as exc:               # probe must not kill us
                    ok, self._probe_note = False, str(exc)
            if ok:
                self.state, self._t_ready = WARMING, now
            elif now - self._t_start > self.ready_timeout:
                self._after_stop = (
                    'failed', f'timed out after {self.ready_timeout:.0f}s '
                              f'waiting for the stack to come up')
                self._begin_stop()
                return
        if self.state == WARMING and now - self._t_ready >= self._settle:
            self.state = RUNNING

    def _tick_stopping(self, now):
        if not self._group_alive_poll():
            self._reap()
            if self._pending is not None:
                self._spawn(*self._pending_pop())
            elif self._after_stop is not None:
                _, reason = self._after_stop
                self._after_stop = None
                self.detail, self.state = reason, FAILED
            else:
                self.state = IDLE
            return
        if not self._killed and now - self._t_stop >= self.stop_grace:
            self._signal(signal.SIGKILL)
            self._killed, self._t_kill = True, now
        elif self._killed and now - self._t_kill >= self.kill_wait:
            self._pending = None
            self._fail('previous stack could not be stopped (SIGKILL '
                       'ignored)')


def parent_alive(pid, kill=os.kill):
    """True while process `pid` exists (signal 0 probe)."""
    try:
        kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def watch_parent(period=1.0, environ=None, kill=os.kill, interrupt=None,
                 sleep=time.sleep, start=True):
    """Stack-side safety net for a sim that dies without cleaning up
    (SIGKILL, OOM): the supervisor exports BOIDS_STACK_PARENT_PID, and this
    thread inside the stack's launch process SIGINTs the launch itself
    once that pid is gone, which shuts every node down gracefully.

    Returns the thread (or None when the stack was started by hand)."""
    import threading
    pid = int((environ if environ is not None else os.environ).get(
        'BOIDS_STACK_PARENT_PID', '0') or 0)
    if pid <= 0:
        return None
    if interrupt is None:
        def interrupt():
            kill(os.getpid(), signal.SIGINT)

    def loop():
        while parent_alive(pid, kill):
            sleep(period)
        interrupt()

    th = threading.Thread(target=loop, daemon=True, name='parent-watch')
    if start:
        th.start()
    return th
