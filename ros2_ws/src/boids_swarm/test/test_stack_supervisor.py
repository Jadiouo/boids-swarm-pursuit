"""StackSupervisor state machine, driven by a fake Popen / killpg / clock.

No real process is ever started: the fake world records every signal sent
to a process group and lets each test decide who obeys SIGINT and who needs
SIGKILL, which is what the timeout path is about.
"""

import signal

import pytest

from boids_swarm.stack_supervisor import StackSupervisor


class World:
    """Fake clock + fake process table."""

    def __init__(self):
        self.t = 0.0
        self.procs = []
        self.signals = []            # (pgid, sig)
        self.spawn_kwargs = []
        self.obey_sigint = True      # exit on SIGINT?
        self.group_leftover = False  # survivors after the leader exits

    def clock(self):
        return self.t

    def sleep(self, dt):
        self.t += dt

    def popen(self, cmd, **kw):
        self.spawn_kwargs.append(dict(kw, cmd=cmd))
        p = FakeProc(self, 1000 + len(self.procs))
        self.procs.append(p)
        return p

    def killpg(self, pgid, sig):
        proc = next(p for p in self.procs if p.pid == pgid)
        if sig == 0:
            if not proc.group_alive:
                raise ProcessLookupError()
            return
        self.signals.append((pgid, sig))
        if sig == signal.SIGKILL:
            proc.die(-9, leftover=False)
        elif sig in (signal.SIGINT, signal.SIGTERM) and self.obey_sigint:
            proc.die(0, leftover=self.group_leftover)


class FakeProc:
    def __init__(self, world, pid):
        self.w, self.pid = world, pid
        self.returncode = None
        self.group_alive = True

    def poll(self):
        return self.returncode

    def die(self, code, leftover=False):
        self.returncode = code
        self.group_alive = leftover

    def wait(self, timeout=None):
        return self.returncode


@pytest.fixture
def world():
    return World()


@pytest.fixture
def sup(world, tmp_path):
    return StackSupervisor(popen=world.popen, killpg=world.killpg,
                           clock=world.clock, sleep=world.sleep,
                           stop_grace=5.0, kill_wait=2.0,
                           log_dir=str(tmp_path))


def run(world, sup, seconds, step=0.1):
    for _ in range(int(seconds / step)):
        world.t += step
        sup.tick()


def test_start_spawns_in_a_new_session_and_reports_starting(world, sup):
    sup.start(['ros2', 'launch', 'x'], eta_s=3.0)
    kw = world.spawn_kwargs[0]
    assert kw['cmd'] == ['ros2', 'launch', 'x']
    assert kw['start_new_session'] is True      # own process group
    assert sup.state == 'starting'


def test_becomes_running_once_probe_passes_and_settle_elapses(world, sup):
    ready = {'v': False}
    sup.start(['x'], probe=lambda: ready['v'], settle_s=1.0, eta_s=2.0)
    run(world, sup, 2.0)
    assert sup.state == 'starting'               # probe not satisfied
    ready['v'] = True
    run(world, sup, 0.2)
    assert sup.state == 'warming'
    run(world, sup, 1.0)
    assert sup.state == 'running'
    assert sup.is_running


def test_no_probe_means_running_after_the_settle_time(world, sup):
    sup.start(['x'], settle_s=0.5)
    run(world, sup, 1.0)
    assert sup.state == 'running'


def test_warm_progress_counts_down_towards_eta(world, sup):
    sup.start(['x'], probe=lambda: False, eta_s=8.0)
    run(world, sup, 3.0)
    assert sup.warm_left == pytest.approx(5.0, abs=0.2)
    run(world, sup, 20.0)
    assert sup.warm_left == 0.0


def test_ready_timeout_fails_and_kills_the_stack(world, sup):
    sup.ready_timeout = 10.0
    sup.start(['x'], probe=lambda: False)
    run(world, sup, 11.0)
    run(world, sup, 1.0)
    assert sup.state == 'failed'
    assert 'timed out' in sup.detail
    assert any(s == signal.SIGINT for _, s in world.signals)


def test_unexpected_exit_while_starting_is_failed_with_code_and_log(
        world, sup, tmp_path):
    sup.start(['x'], probe=lambda: False)
    log = world.spawn_kwargs[0]['stdout']
    log.write(b'line1\nRuntimeError: unknown evader smart\n')
    log.flush()
    world.procs[0].die(3, leftover=False)
    run(world, sup, 0.2)
    assert sup.state == 'failed'
    assert 'code 3' in sup.detail
    assert 'unknown evader smart' in sup.detail


def test_unexpected_exit_while_running_is_failed(world, sup):
    sup.start(['x'], settle_s=0.1)
    run(world, sup, 0.5)
    assert sup.state == 'running'
    world.procs[0].die(1, leftover=False)
    run(world, sup, 0.2)
    assert sup.state == 'failed'


def test_failure_sweeps_leftover_group_members(world, sup):
    sup.start(['x'], settle_s=0.1)
    run(world, sup, 0.5)
    world.procs[0].die(1, leftover=True)         # launch died, nodes live
    run(world, sup, 0.2)
    assert (1000, signal.SIGKILL) in world.signals
    assert not world.procs[0].group_alive


def test_stop_sends_sigint_to_the_group_then_goes_idle(world, sup):
    sup.start(['x'], settle_s=0.1)
    run(world, sup, 0.5)
    sup.stop()
    assert sup.state == 'stopping'
    assert world.signals == [(1000, signal.SIGINT)]
    run(world, sup, 0.3)
    assert sup.state == 'idle'


def test_stop_escalates_to_sigkill_after_the_grace_period(world, sup):
    world.obey_sigint = False
    sup.start(['x'], settle_s=0.1)
    run(world, sup, 0.5)
    sup.stop()
    run(world, sup, 4.0)
    assert (1000, signal.SIGKILL) not in world.signals   # still polite
    run(world, sup, 1.5)
    assert (1000, signal.SIGKILL) in world.signals
    run(world, sup, 0.3)
    assert sup.state == 'idle'


def test_stop_waits_for_group_members_not_just_the_leader(world, sup):
    """The launch process exiting is not enough: a node that outlived it
    would be an orphan. Idle only once the whole group is gone."""
    world.group_leftover = True
    sup.start(['x'], settle_s=0.1)
    run(world, sup, 0.5)
    sup.stop()
    run(world, sup, 1.0)
    assert sup.state == 'stopping'
    run(world, sup, 5.0)                          # grace over -> SIGKILL
    assert (1000, signal.SIGKILL) in world.signals
    run(world, sup, 0.3)
    assert sup.state == 'idle'


def test_unkillable_group_is_reported_failed(world, sup):
    world.obey_sigint = False
    sup.start(['x'], settle_s=0.1)
    run(world, sup, 0.5)
    # SIGKILL "fails": process refuses to die
    orig = world.killpg

    def stubborn(pgid, sig):
        if sig == signal.SIGKILL:
            world.signals.append((pgid, sig))
            return
        return orig(pgid, sig)
    sup._killpg = stubborn
    sup.stop()
    run(world, sup, 12.0)
    assert sup.state == 'failed'
    assert 'could not be stopped' in sup.detail


def test_switch_stops_the_old_stack_before_starting_the_new_one(world, sup):
    sup.start(['old'], settle_s=0.1)
    run(world, sup, 0.5)
    sup.switch(['new'], settle_s=0.1)
    # old group is signalled, new one is NOT spawned until the old one is gone
    assert len(world.procs) == 1
    assert sup.state == 'stopping'
    run(world, sup, 0.3)
    assert len(world.procs) == 2
    assert world.spawn_kwargs[1]['cmd'] == ['new']
    assert sup.state in ('starting', 'warming')
    assert world.procs[0].returncode is not None   # old process is gone
    run(world, sup, 0.5)
    assert sup.state == 'running'


def test_switch_with_a_stuck_old_stack_kills_it_then_starts(world, sup):
    world.obey_sigint = False
    sup.start(['old'], settle_s=0.1)
    run(world, sup, 0.5)
    sup.switch(['new'], settle_s=0.1)
    run(world, sup, 6.0)
    assert (1000, signal.SIGKILL) in world.signals
    assert len(world.procs) == 2


def test_switch_when_idle_just_starts(world, sup):
    sup.switch(['new'], settle_s=0.1)
    assert len(world.procs) == 1 and sup.state == 'starting'


def test_switch_after_failure_starts_a_fresh_stack(world, sup):
    sup.start(['bad'], probe=lambda: False)
    world.procs[0].die(2, leftover=False)
    run(world, sup, 0.2)
    assert sup.state == 'failed'
    sup.switch(['good'], settle_s=0.1)
    run(world, sup, 0.5)
    assert sup.state == 'running'
    assert world.spawn_kwargs[-1]['cmd'] == ['good']


def test_only_one_stack_ever_alive(world, sup):
    sup.start(['a'], settle_s=0.1)
    run(world, sup, 0.5)
    for cmd in (['b'], ['c'], ['d']):
        sup.switch(cmd, settle_s=0.1)       # rapid-fire clicks
        run(world, sup, 0.05)
    run(world, sup, 2.0)
    alive = [p for p in world.procs if p.group_alive]
    assert len(alive) == 1
    assert world.spawn_kwargs[-1]['cmd'] == ['d']      # latest request wins
    assert sup.state == 'running'


def test_shutdown_kills_everything_synchronously(world, sup):
    sup.start(['x'], settle_s=0.1)
    run(world, sup, 0.5)
    sup.shutdown()
    assert all(not p.group_alive for p in world.procs)
    assert sup.state == 'idle'


def test_shutdown_escalates_when_the_group_ignores_sigint(world, sup):
    world.obey_sigint = False
    sup.start(['x'], settle_s=0.1)
    run(world, sup, 0.5)
    sup.shutdown(timeout=8.0)
    assert (1000, signal.SIGKILL) in world.signals
    assert not world.procs[0].group_alive


def test_shutdown_is_idempotent_and_safe_with_nothing_started(world, sup):
    sup.shutdown()
    sup.start(['x'])
    sup.shutdown()
    sup.shutdown()


def test_shutdown_cancels_a_pending_switch(world, sup):
    sup.start(['old'], settle_s=0.1)
    run(world, sup, 0.5)
    sup.switch(['new'], settle_s=0.1)
    sup.shutdown()
    assert len(world.procs) == 1            # 'new' never spawned


def test_status_dict_for_the_panel_and_topic(world, sup):
    sup.start(['x'], probe=lambda: False, eta_s=6.0, label='nav2')
    st = sup.status()
    assert st['state'] == 'starting' and st['label'] == 'nav2'
    assert st['warm_left'] == pytest.approx(6.0, abs=0.01)
    assert 'detail' in st and 'log' in st


def test_spawn_failure_is_reported_not_raised(world, tmp_path):
    def boom(cmd, **kw):
        raise FileNotFoundError('ros2')
    s = StackSupervisor(popen=boom, killpg=world.killpg, clock=world.clock,
                        sleep=world.sleep, log_dir=str(tmp_path))
    s.start(['ros2'])
    assert s.state == 'failed' and 'ros2' in s.detail


# --- parent watchdog (the stack side of "no orphans") ----------------------

def test_watch_parent_does_nothing_for_a_hand_started_stack():
    from boids_swarm.stack_supervisor import watch_parent
    assert watch_parent(environ={}) is None


def test_watch_parent_interrupts_the_launch_when_the_sim_is_gone():
    from boids_swarm.stack_supervisor import watch_parent
    alive = {'v': True}
    hits = []

    def kill(pid, sig):
        assert (pid, sig) == (4242, 0)
        if not alive['v']:
            raise ProcessLookupError()

    def sleep(_):
        alive['v'] = False              # the sim dies during the wait

    th = watch_parent(environ={'BOIDS_STACK_PARENT_PID': '4242'}, kill=kill,
                      interrupt=lambda: hits.append('int'), sleep=sleep,
                      start=False)
    th.run()
    assert hits == ['int']
