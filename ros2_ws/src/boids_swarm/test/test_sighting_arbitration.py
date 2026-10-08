from boids_swarm.sighting_protocol import SightingRecord
from boids_swarm.sighting_runtime import ObservationInbox


def sighting(sender, seq, stamp, **changes):
    fields = dict(
        episode_id=4, sequence=seq, sender_id=sender, target_id='target',
        frame_id='world', stamp=stamp, sender_xy=(1.0, 1.0),
        target_xy=(5.0, 5.0), covariance_xy=(0.2, 0.0, 0.0, 0.2),
        confidence=0.9, valid_for_sec=0.8)
    fields.update(changes)
    return SightingRecord(**fields)


def inbox():
    return ObservationInbox(
        self_id='agent1', expected_episode=4, sighting_timeout=0.6,
        future_tolerance=0.02, max_valid_for=1.0)


def test_direct_measurement_wins_and_only_one_candidate_is_selected_per_tick():
    pending = inbox()
    relay = sighting('agent0', 1, 10.0)
    direct = sighting('agent1', 1, 10.01)
    assert pending.offer(relay, source='relay', now=10.02) is None
    assert pending.offer(direct, source='direct', now=10.02) is None

    selected = pending.take_for_cycle(now=10.02)
    assert selected.record == direct
    assert selected.source == 'direct'
    assert pending.take_for_cycle(now=10.02) is None


def test_self_loopback_is_rejected_and_direct_measurement_is_applied_once():
    pending = inbox()
    record = sighting('agent1', 5, 10.0)
    assert pending.offer(record, source='relay', now=10.01) == \
        'self_originated'
    assert pending.offer(record, source='direct', now=10.01) is None
    selected = pending.take_for_cycle(now=10.01)
    assert selected.record == record
    assert selected.source == 'direct'
    pending.mark_applied(selected)
    assert pending.offer(record, source='direct', now=10.02) == \
        'late_observation'
    assert pending.take_for_cycle(now=10.02) is None


def test_conflicting_sequence_or_same_sender_stamp_is_discarded():
    pending = inbox()
    first = sighting('agent0', 2, 10.0)
    changed_payload = sighting('agent0', 2, 10.0, target_xy=(6.0, 5.0))
    assert pending.offer(first, source='relay', now=10.01) is None
    assert pending.offer(changed_payload, source='relay', now=10.01) == \
        'sequence_conflict'
    assert pending.take_for_cycle(now=10.01) is None

    next_pending = inbox()
    assert next_pending.offer(first, source='relay', now=10.01) is None
    same_stamp_new_sequence = sighting('agent0', 3, 10.0)
    assert next_pending.offer(same_stamp_new_sequence, source='relay',
                              now=10.01) == 'sender_stamp_conflict'
    assert next_pending.take_for_cycle(now=10.01) is None


def test_late_callback_cannot_apply_same_or_older_observation_again():
    pending = inbox()
    applied = sighting('agent0', 8, 10.0)
    assert pending.offer(applied, source='relay', now=10.01) is None
    candidate = pending.take_for_cycle(now=10.01)
    pending.mark_applied(candidate)

    same_stamp_new_sender = sighting('agent2', 1, 10.0)
    older = sighting('agent3', 1, 9.99)
    assert pending.offer(same_stamp_new_sender, source='relay',
                         now=10.02) == 'late_observation'
    assert pending.offer(older, source='relay', now=10.02) == \
        'late_observation'


def test_episode_is_only_changed_by_explicit_authority_and_clears_pending():
    pending = inbox()
    current = sighting('agent0', 1, 10.0)
    assert pending.offer(current, source='relay', now=10.01) is None
    pending.set_episode(5)
    assert pending.take_for_cycle(now=10.01) is None
    assert pending.offer(current, source='relay', now=10.01) == 'old_episode'
    future = sighting('agent0', 2, 10.01, episode_id=6)
    assert pending.offer(future, source='relay', now=10.02) == 'future_episode'
    new_epoch = sighting('agent0', 1, 10.02, episode_id=5)
    assert pending.offer(new_epoch, source='relay', now=10.02) is None


def test_older_queued_sequence_conflict_is_poisoned_after_newer_packet():
    pending = inbox()
    first = sighting('agent0', 1, 10.0)
    newer = sighting('agent0', 2, 10.1)
    conflict = sighting('agent0', 1, 10.0, target_xy=(8.0, 5.0))
    assert pending.offer(first, source='relay', now=10.11) is None
    assert pending.offer(newer, source='relay', now=10.11) is None
    assert pending.offer(conflict, source='relay', now=10.11) == \
        'sequence_conflict'
    selected = pending.take_for_cycle(now=10.11)
    assert selected.record == newer


def test_clock_rollback_clears_state_until_authoritative_epoch():
    pending = inbox()
    old = sighting('agent0', 1, 10.0)
    assert pending.offer(old, source='relay', now=10.01) is None
    pending.on_clock_rollback()
    assert pending.take_for_cycle(now=1.0) is None
    assert pending.offer(sighting('agent0', 2, 1.0), source='relay',
                         now=1.01) == 'awaiting_episode'
    assert pending.set_episode(5) == 'episode_changed'
    assert pending.offer(sighting('agent0', 1, 1.0, episode_id=5),
                         source='relay', now=1.01) is None
