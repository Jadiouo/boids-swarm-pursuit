"""Fan out parameter changes from the GUI to this node and its peers (M15).

The panel can only change the sim's own parameters directly. `pursuit_strategy`
lives on N separate `/agentK/boid_controller` processes and `evader` on
`/target_controller`, so a panel edit becomes N+1 `SetParameters` service
calls.

Scopes (`'a+b'` combines):

    sim      the sim node itself
    agents   every /agentK/boid_controller
    target   /target_controller

The panel is immediate mode and draws the sim's *local mirror* of remote
parameters. The mirror is written back **after every addressed node confirmed
the change**: a rejected or lost request leaves the display on the value the
controllers really have, and a Cycler therefore advances from what is shown
(before this, the mirror was never updated and a Cycler restarted from the
same entry on every click).

**Threading contract.** pygame (and therefore the panel) runs on the main
thread while rclpy callbacks run on a background executor thread (see the
sim's `main`). Issuing service calls straight from the click handler would
put rclpy work on two threads at once. Instead the panel only *enqueues*
`(scope, name, value)` and a ROS timer — which the executor thread owns —
drains the queue and does all the rclpy work. One writer, one reader, no
locks beyond the queue's own. Done-callbacks also run on that thread.

Requests are coalesced by (scope, name): dragging a slider produces a value
every mouse-move, and only the newest one matters. Without this a 2-second
drag would queue ~100 rounds of N service calls.
"""

import queue

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter as RclParameter


def to_parameter_value(value) -> ParameterValue:
    """Python value -> ParameterValue. bool is checked before int on
    purpose: bool is a subclass of int and would otherwise be sent as an
    integer, which the receiving node rejects as a type mismatch."""
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL,
                              bool_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                              integer_value=value)
    if isinstance(value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                              double_value=value)
    return ParameterValue(type=ParameterType.PARAMETER_STRING,
                          string_value=str(value))


class _Fan:
    """Collects the replies of one fan-out; commits the mirror at the end."""

    def __init__(self, bridge, name, value, count, epoch):
        self.bridge, self.name, self.value = bridge, name, value
        self.count, self.epoch = count, epoch
        self.ok, self.errors = 0, []

    def reply(self, fut):
        try:
            res = fut.result()
            bad = [r.reason or 'rejected' for r in res.results
                   if not r.successful]
            if bad:
                self.errors.append(bad[0])
            else:
                self.ok += 1
        except Exception as exc:             # service gone / cancelled
            self.errors.append(str(exc) or type(exc).__name__)
        if self.ok + len(self.errors) == self.count:
            self.bridge._commit(self)


class ParamBridge:
    """Queue-backed parameter fan-out."""

    def __init__(self, node, n_agents, target_enabled=True,
                 agent_node='boid_controller',
                 target_node='/target_controller'):
        self.node = node
        self._q = queue.Queue()
        self._agent_node = agent_node
        self._agent_clients = []
        self.n = 0
        self.set_n(n_agents)
        self._target_client = (
            node.create_client(SetParameters,
                               f'{target_node}/set_parameters')
            if target_enabled else None)
        self._pending = []          # futures kept only until they finish
        self._epoch = 0
        self.last_error = ''
        self.skipped = 0

    def set_n(self, n):
        """Address the first `n` controllers (clients are created lazily
        and kept: a later, smaller n just ignores the surplus)."""
        while len(self._agent_clients) < n:
            i = len(self._agent_clients)
            self._agent_clients.append(self.node.create_client(
                SetParameters, f'/agent{i}/{self._agent_node}/set_parameters'))
        self.n = n

    def flush(self):
        """Forget in-flight requests (the stack they addressed is being
        replaced): their late replies must not write the mirror."""
        self._epoch += 1
        self._pending.clear()
        while not self._q.empty():
            try:
                self._q.get_nowait()
            except queue.Empty:
                break

    # --- producer side (main / render thread) ---------------------------
    def request(self, scope, name, value):
        self._q.put((scope, name, value))

    # --- consumer side (executor thread, via a ROS timer) ---------------
    def pump(self):
        latest = {}
        while True:
            try:
                scope, name, value = self._q.get_nowait()
            except queue.Empty:
                break
            latest[(scope, name)] = value       # coalesce slider drags
        for (scope, name), value in latest.items():
            try:
                self._apply(scope, name, value)
            except Exception as exc:            # never kill the timer
                self.last_error = f'{scope}.{name}={value!r}: {exc}'
                self.node.get_logger().warn(
                    f'param {scope}.{name}={value!r} failed: {exc}')
        self._pending = [f for f in self._pending if not f.done()]

    def _apply(self, scope, name, value):
        parts = scope.split('+')
        if 'sim' in parts:
            self.node.set_parameters([RclParameter(name, value=value)])
        clients = []
        if 'agents' in parts:
            clients += self._agent_clients[:self.n]
        if 'target' in parts and self._target_client is not None:
            clients.append(self._target_client)
        if not clients:
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name,
                                    value=to_parameter_value(value))]
        ready = [c for c in clients if c.service_is_ready()]
        if len(ready) < len(clients):
            # controller not up yet: do not claim the change happened
            self.skipped += 1
            self.last_error = f'{name}: controllers not ready'
            return
        # `sim` already holds the new value; remote-only scopes mirror it
        # once everybody agreed.
        fan = _Fan(self, name, value, len(ready), self._epoch)
        fan.mirror = 'sim' not in parts
        for cli in ready:
            fut = cli.call_async(req)
            self._pending.append(fut)
            fut.add_done_callback(fan.reply)

    def _commit(self, fan):
        if fan.epoch != self._epoch:
            return                               # stack was replaced
        if fan.errors:
            self.last_error = f'{fan.name}: {fan.errors[0]}'
            self.node.get_logger().warn(
                f'param {fan.name}={fan.value!r} not applied: '
                f'{fan.errors[0]}')
            return
        self.last_error = ''
        if fan.mirror:
            try:
                self.node.set_parameters(
                    [RclParameter(fan.name, value=fan.value)])
            except Exception as exc:     # e.g. mirror param type mismatch
                self.node.get_logger().warn(
                    f'mirror {fan.name}={fan.value!r} not stored: {exc}')

    def all_ready(self, include_target=True):
        """Every addressed controller service is discoverable (the panel's
        readiness probe for a freshly started stack)."""
        clients = list(self._agent_clients[:self.n])
        if include_target and self._target_client is not None:
            clients.append(self._target_client)
        return bool(clients) and all(c.service_is_ready() for c in clients)
