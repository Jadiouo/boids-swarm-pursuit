import pytest

from boids_swarm.sighting_protocol import (
    SightingRecord,
    validate_sighting,
)


def valid_record(**changes):
    values = dict(
        episode_id=2,
        sequence=7,
        sender_id='agent0',
        target_id='target',
        frame_id='world',
        stamp=10.0,
        sender_xy=(1.0, 2.0),
        target_xy=(4.0, 5.0),
        covariance_xy=(0.25, 0.0, 0.0, 0.5),
        confidence=0.9,
        valid_for_sec=0.6,
    )
    values.update(changes)
    return SightingRecord(**values)


def test_valid_current_sighting_passes_domain_validation():
    assert validate_sighting(
        valid_record(), expected_episode=2, now=10.2,
        sighting_timeout=0.5, future_tolerance=0.02,
        max_valid_for=2.0) is None


@pytest.mark.parametrize(
    ('changes', 'now', 'expected'),
    [
        ({'frame_id': 'map'}, 10.2, 'invalid_frame'),
        ({'sequence': 0}, 10.2, 'invalid_sequence'),
        ({'episode_id': 1}, 10.2, 'old_episode'),
        ({'episode_id': 3}, 10.2, 'future_episode'),
        ({'stamp': 10.1}, 10.0, 'future_stamp'),
        ({'stamp': 9.0}, 10.0, 'stale'),
        ({'covariance_xy': (1.0, 2.0, 0.0, 1.0)}, 10.2,
         'invalid_covariance'),
        ({'covariance_xy': (0.0, 1e-5, 1e-5, 0.0)}, 10.2,
         'invalid_covariance'),
        ({'covariance_xy': (1.0, 0.0, 0.0, -0.1)}, 10.2,
         'invalid_covariance'),
        ({'target_xy': (float('nan'), 0.0)}, 10.2, 'non_finite'),
        ({'confidence': 1.1}, 10.2, 'invalid_confidence'),
        ({'valid_for_sec': 0.0}, 10.2, 'invalid_validity'),
    ],
)
def test_invalid_sighting_has_stable_rejection_reason(changes, now, expected):
    reason = validate_sighting(
        valid_record(**changes), expected_episode=2, now=now,
        sighting_timeout=0.5, future_tolerance=0.02,
        max_valid_for=2.0)
    assert reason == expected

