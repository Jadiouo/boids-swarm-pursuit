"""launch_util.ensure_sigint_deliverable (pure; signal calls are injected)."""
import signal

from boids_swarm import launch_util


def test_installs_a_handler_when_sigint_is_ignored():
    calls = []
    changed = launch_util.ensure_sigint_deliverable(
        getsignal=lambda s: signal.SIG_IGN,
        install=lambda s, h: calls.append((s, h)))
    assert changed and len(calls) == 1
    sig, handler = calls[0]
    assert sig == signal.SIGINT and callable(handler)
    handler(signal.SIGINT, None)                  # does nothing, no raise


def test_leaves_a_normal_sigint_disposition_alone():
    for current in (signal.SIG_DFL, signal.default_int_handler):
        calls = []
        assert not launch_util.ensure_sigint_deliverable(
            getsignal=lambda s, c=current: c,
            install=lambda s, h: calls.append(s))
        assert calls == []
