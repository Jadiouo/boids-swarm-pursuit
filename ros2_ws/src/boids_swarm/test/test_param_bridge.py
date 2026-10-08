"""ParamBridge fan-out and mirror write-back (control-panel bug 1 and 2).

Needs only the rcl_interfaces message types (no rclpy.init), so it runs
anywhere ROS is sourced and skips elsewhere. Nodes, clients and futures are
fakes: the point is *when* the sim's local mirror is written (only after the
remote node confirmed) and *which* nodes each scope reaches.
"""

import pytest

pytest.importorskip('rcl_interfaces')

from rcl_interfaces.msg import ParameterType, SetParametersResult  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402

from boids_swarm.param_bridge import ParamBridge  # noqa: E402


class Future:
    def __init__(self):
        self._cbs, self._res, self._done = [], None, False

    def add_done_callback(self, cb):
        if self._done:
            cb(self)
        else:
            self._cbs.append(cb)

    def done(self):
        return self._done

    def result(self):
        if isinstance(self._res, Exception):
            raise self._res
        return self._res

    def finish(self, ok=True, reason=''):
        r = SetParameters.Response()
        r.results = [SetParametersResult(successful=ok, reason=reason)]
        self._res, self._done = r, True
        for cb in self._cbs:
            cb(self)

    def fail(self, exc):
        self._res, self._done = exc, True
        for cb in self._cbs:
            cb(self)


class Client:
    def __init__(self, name, ready=True):
        self.name, self.ready, self.calls = name, ready, []

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        f = Future()
        self.calls.append((req, f))
        return f


class Log:
    def __init__(self):
        self.warns = []

    def warn(self, m):
        self.warns.append(m)


class Node:
    def __init__(self):
        self.clients = {}
        self.local = []                 # set_parameters calls on the sim
        self.log = Log()

    def create_client(self, srv, name):
        c = self.clients[name] = Client(name)
        return c

    def set_parameters(self, params):
        self.local.append([(p.name, p.value) for p in params])

    def get_logger(self):
        return self.log


@pytest.fixture
def node():
    return Node()


@pytest.fixture
def bridge(node):
    return ParamBridge(node, 3)


def finish_all(node, ok=True):
    for c in node.clients.values():
        for _, f in c.calls:
            if not f.done():
                f.finish(ok)


def test_agents_scope_reaches_every_controller_and_not_the_target(
        node, bridge):
    bridge.request('agents', 'w_pursuit', 1.5)
    bridge.pump()
    for i in range(3):
        assert len(node.clients[f'/agent{i}/boid_controller/'
                                f'set_parameters'].calls) == 1
    assert node.clients['/target_controller/set_parameters'].calls == []
    req = node.clients['/agent0/boid_controller/set_parameters'].calls[0][0]
    assert req.parameters[0].name == 'w_pursuit'
    assert req.parameters[0].value.double_value == 1.5


def test_mirror_is_written_back_only_after_every_controller_confirms(
        node, bridge):
    """Regression for panel bug 1: the AGENTS/TARGET rows read the sim's
    local mirror, and `_apply` never wrote it, so the display stayed at the
    launch value and a Cycler restarted from the same entry forever."""
    bridge.request('agents', 'pursuit_strategy', 'pincer')
    bridge.pump()
    assert node.local == []                       # not before the reply
    futs = [c.calls[0][1] for n, c in node.clients.items()
            if n.startswith('/agent')]
    futs[0].finish()
    futs[1].finish()
    assert node.local == []                       # one still outstanding
    futs[2].finish()
    assert len(node.local) == 1
    name, value = node.local[0][0]
    assert name == 'pursuit_strategy' and value == 'pincer'


def test_rejected_value_does_not_touch_the_mirror(node, bridge):
    bridge.request('target', 'evader', 'smart')
    bridge.pump()
    node.clients['/target_controller/set_parameters'].calls[0][1].finish(
        ok=False, reason="unknown evader 'smart'")
    assert node.local == []
    assert 'smart' in bridge.last_error


def test_one_rejecting_controller_keeps_the_old_mirror(node, bridge):
    bridge.request('agents', 'w_pursuit', 9.0)
    bridge.pump()
    futs = [c.calls[0][1] for n, c in node.clients.items()
            if n.startswith('/agent')]
    futs[0].finish()
    futs[1].finish(ok=False, reason='out of range')
    futs[2].finish()
    assert node.local == []


def test_service_exception_is_survived_and_reported(node, bridge):
    bridge.request('target', 'evader', 'adaptive')
    bridge.pump()
    node.clients['/target_controller/set_parameters'].calls[0][1].fail(
        RuntimeError('service died'))
    assert node.local == []
    assert 'service died' in bridge.last_error


def test_unready_controllers_are_skipped_and_nothing_is_mirrored(
        node, bridge):
    for c in node.clients.values():
        c.ready = False
    bridge.request('agents', 'w_pursuit', 1.0)
    bridge.pump()
    assert all(not c.calls for c in node.clients.values())
    assert node.local == []
    assert bridge.skipped == 1


def test_sim_scope_is_set_locally_and_sent_nowhere(node, bridge):
    bridge.request('sim', 'fov', 1.0)
    bridge.pump()
    assert node.local == [[('fov', node.local[0][0][1])]]
    assert all(not c.calls for c in node.clients.values())


def test_target_speed_reaches_sim_and_target_controller(node, bridge):
    """Regression for panel bug 2: target_speed_multiplier / omega were
    sim-only, but target_controller reads same-named parameters to steer."""
    bridge.request('sim+target', 'target_speed_multiplier', 2.4)
    bridge.pump()
    assert node.local[0][0][0] == 'target_speed_multiplier'
    calls = node.clients['/target_controller/set_parameters'].calls
    assert len(calls) == 1
    assert calls[0][0].parameters[0].value.double_value == 2.4


def test_sim_plus_agents_reaches_sim_and_all_controllers(node, bridge):
    bridge.request('sim+agents', 'radio_range', 6.0)
    bridge.pump()
    assert node.local[0][0][0] == 'radio_range'
    assert all(len(node.clients[f'/agent{i}/boid_controller/'
                                f'set_parameters'].calls) == 1
               for i in range(3))


def test_slider_drag_is_coalesced_to_the_newest_value(node, bridge):
    for v in (1.0, 1.1, 1.2, 1.3):
        bridge.request('agents', 'w_pursuit', v)
    bridge.pump()
    calls = node.clients['/agent0/boid_controller/set_parameters'].calls
    assert len(calls) == 1
    assert calls[0][0].parameters[0].value.double_value == 1.3


def test_int_and_bool_keep_their_ros_types(node, bridge):
    bridge.request('agents', 'k', 3)
    bridge.request('target', 'flag', True)
    bridge.pump()
    a = node.clients['/agent0/boid_controller/set_parameters'].calls[0][0]
    t = node.clients['/target_controller/set_parameters'].calls[0][0]
    assert a.parameters[0].value.type == ParameterType.PARAMETER_INTEGER
    assert t.parameters[0].value.type == ParameterType.PARAMETER_BOOL


def test_set_n_resizes_the_controller_fan_out(node, bridge):
    bridge.set_n(5)
    bridge.request('agents', 'w_pursuit', 1.0)
    bridge.pump()
    assert len(node.clients['/agent4/boid_controller/'
                            'set_parameters'].calls) == 1
    bridge.set_n(2)
    bridge.request('agents', 'w_pursuit', 1.5)
    bridge.pump()
    assert len(node.clients['/agent2/boid_controller/'
                            'set_parameters'].calls) == 1       # old call only
    assert len(node.clients['/agent1/boid_controller/'
                            'set_parameters'].calls) == 2


def test_late_confirmation_after_a_restart_does_not_corrupt_the_mirror(
        node, bridge):
    """A reply for a stack that has since been replaced (flush) is dropped."""
    bridge.request('agents', 'w_pursuit', 4.0)
    bridge.pump()
    bridge.flush()                      # mode switch: old calls are stale
    finish_all(node)
    assert node.local == []
