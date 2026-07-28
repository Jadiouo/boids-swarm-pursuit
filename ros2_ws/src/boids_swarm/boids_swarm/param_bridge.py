"""Fan out parameter changes from the GUI to this node and its peers (M15).

The panel can only change the sim's own parameters directly. `pursuit_strategy`
lives on N separate `/agentK/boid_controller` processes and `evader` on
`/target_controller`, so a panel edit becomes N+1 `SetParameters` service
calls.

**Threading contract.** pygame (and therefore the panel) runs on the main
thread while rclpy callbacks run on a background executor thread (see the
sim's `main`). Issuing service calls straight from the click handler would
put rclpy work on two threads at once. Instead the panel only *enqueues*
`(scope, name, value)` and a ROS timer — which the executor thread owns —
drains the queue and does all the rclpy work. One writer, one reader, no
locks beyond the queue's own.

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


class ParamBridge:
    """Queue-backed parameter fan-out.

    scope 'sim'    -> set on the owning node directly
    scope 'agents' -> /agent0..N-1/boid_controller/set_parameters
    scope 'target' -> /target_controller/set_parameters
    """

    def __init__(self, node, n_agents, target_enabled=True,
                 agent_node='boid_controller',
                 target_node='/target_controller'):
        self.node = node
        self._q = queue.Queue()
        self._agent_clients = [
            node.create_client(SetParameters,
                               f'/agent{i}/{agent_node}/set_parameters')
            for i in range(n_agents)]
        self._target_client = (
            node.create_client(SetParameters,
                               f'{target_node}/set_parameters')
            if target_enabled else None)
        self._pending = []          # futures we keep only to log failures

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
                self.node.get_logger().warn(
                    f'param {scope}.{name}={value!r} failed: {exc}')
        self._pending = [f for f in self._pending if not f.done()]

    def _apply(self, scope, name, value):
        if scope == 'sim':
            self.node.set_parameters(
                [RclParameter(name, value=value)])
            return
        if scope == 'agents':
            clients = self._agent_clients
        elif scope == 'target':
            clients = [self._target_client] if self._target_client else []
        else:
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name,
                                    value=to_parameter_value(value))]
        for cli in clients:
            if not cli.service_is_ready():
                continue        # controller not up yet; next edit will land
            self._pending.append(cli.call_async(req))
