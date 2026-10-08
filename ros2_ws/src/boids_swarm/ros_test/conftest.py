"""Shared rclpy lifecycle for the ROS integration tests.

Both test modules create their own nodes, but rclpy.init() may only run once
per process unless the previous context was shut down. A module-scoped
autouse fixture therefore hands every module a clean rclpy state (and reads
ROS_DOMAIN_ID afresh) and shuts it down afterwards, whether the module's
tests passed or failed.
"""
import pytest
import rclpy


def _shutdown_if_ok():
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(scope='module', autouse=True)
def _clean_rclpy_context():
    _shutdown_if_ok()
    yield
    _shutdown_if_ok()
