"""Pure validation helpers for the ROS sighting wire protocol.

The domain record deliberately has no ROS imports. ROS callbacks convert
generated messages to this type, validate at receipt, and only then enqueue
them for the controller's control-tick arbitration.
"""

from dataclasses import dataclass
import math
import re


_AGENT_ID = re.compile(r'^agent[0-9]+$')
_FRAME_ID = 'world'
_COVARIANCE_RELATIVE_TOLERANCE = 1e-12


@dataclass(frozen=True)
class SightingRecord:
    episode_id: int
    sequence: int
    sender_id: str
    target_id: str
    frame_id: str
    stamp: float
    sender_xy: tuple[float, float]
    target_xy: tuple[float, float]
    covariance_xy: tuple[float, float, float, float]
    confidence: float
    valid_for_sec: float


def validate_sighting(record, *, expected_episode, now, sighting_timeout,
                      future_tolerance, max_valid_for):
    """Return a stable reject reason, or ``None`` for a valid observation.

    Age uses the source observation stamp, never callback receipt time. The
    caller supplies the authoritative episode already received from the sim.
    """
    if record.frame_id != _FRAME_ID:
        return 'invalid_frame'
    if not _AGENT_ID.fullmatch(record.sender_id or ''):
        return 'invalid_sender'
    if record.target_id != 'target':
        return 'invalid_target'
    if (isinstance(record.episode_id, bool)
            or not isinstance(record.episode_id, int)
            or not 0 <= record.episode_id <= 0xFFFFFFFF):
        return 'invalid_episode'
    if record.episode_id < expected_episode:
        return 'old_episode'
    if record.episode_id > expected_episode:
        return 'future_episode'
    if (isinstance(record.sequence, bool)
            or not isinstance(record.sequence, int)
            or not 1 <= record.sequence <= 0xFFFFFFFFFFFFFFFF):
        return 'invalid_sequence'

    scalar_values = (record.stamp, record.confidence, record.valid_for_sec,
                     now, sighting_timeout, future_tolerance, max_valid_for)
    points = (*record.sender_xy, *record.target_xy)
    covariance = record.covariance_xy
    if (len(record.sender_xy) != 2 or len(record.target_xy) != 2
            or len(covariance) != 4):
        return 'invalid_shape'
    try:
        if not all(math.isfinite(float(v)) for v in scalar_values + points
                   + tuple(covariance)):
            return 'non_finite'
    except (TypeError, ValueError):
        return 'non_finite'

    xx, xy, yx, yy = (float(v) for v in covariance)
    scale = max(abs(xx), abs(xy), abs(yx), abs(yy), 1.0)
    tol = _COVARIANCE_RELATIVE_TOLERANCE * scale
    min_eigenvalue = 0.5 * (
        xx + yy - math.hypot(xx - yy, xy + yx))
    if (xx < -tol or yy < -tol or abs(xy - yx) > tol
            or min_eigenvalue < -tol):
        return 'invalid_covariance'
    if not 0.0 <= record.confidence <= 1.0:
        return 'invalid_confidence'
    if (record.valid_for_sec <= 0.0
            or record.valid_for_sec > max_valid_for
            or sighting_timeout <= 0.0):
        return 'invalid_validity'
    if record.stamp > now + future_tolerance:
        return 'future_stamp'
    if now - record.stamp > min(record.valid_for_sec, sighting_timeout):
        return 'stale'
    return None
