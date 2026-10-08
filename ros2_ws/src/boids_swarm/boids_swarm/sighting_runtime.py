"""Bounded one-control-cycle arbitration for direct and relayed sightings.

The ROS callbacks feed this public domain boundary. It makes no ROS calls and
never performs tracker updates; the controller takes at most one returned
candidate per control tick and marks it applied after one tracker update.
"""

from dataclasses import dataclass

from .sighting_protocol import validate_sighting


@dataclass(frozen=True)
class SightingCandidate:
    record: object
    source: str


class ObservationInbox:
    """Validate, de-duplicate, and choose one observation per control tick."""

    def __init__(self, *, self_id, expected_episode=None,
                 sighting_timeout=0.6, future_tolerance=0.02,
                 max_valid_for=2.0):
        self.self_id = self_id
        self.expected_episode = expected_episode
        self.sighting_timeout = sighting_timeout
        self.future_tolerance = future_tolerance
        self.max_valid_for = max_valid_for
        self._latest_by_sender = {}
        self._pending = {}
        self._poisoned = set()
        self._conflicted_stamps = set()
        self._last_applied_stamp = float('-inf')

    def set_episode(self, episode_id):
        """Adopt only simulator authority; clear all per-episode state."""
        if self.expected_episode is not None and episode_id < self.expected_episode:
            return 'old_episode'
        if episode_id == self.expected_episode:
            return 'same_episode'
        self.expected_episode = episode_id
        self._latest_by_sender.clear()
        self._pending.clear()
        self._poisoned.clear()
        self._conflicted_stamps.clear()
        self._last_applied_stamp = float('-inf')
        return 'episode_changed'

    def on_clock_rollback(self):
        """Drop all old-clock state until a fresh authority event arrives."""
        self.expected_episode = None
        self._latest_by_sender.clear()
        self._pending.clear()
        self._poisoned.clear()
        self._conflicted_stamps.clear()
        self._last_applied_stamp = float('-inf')

    def offer(self, record, *, source, now):
        """Queue a validated direct or one-hop record; return rejection code."""
        if self.expected_episode is None:
            return 'awaiting_episode'
        if source not in ('direct', 'relay'):
            return 'invalid_source'
        if source == 'direct' and record.sender_id != self.self_id:
            return 'invalid_direct_sender'
        if source == 'relay' and record.sender_id == self.self_id:
            return 'self_originated'

        reason = validate_sighting(
            record,
            expected_episode=self.expected_episode,
            now=now,
            sighting_timeout=self.sighting_timeout,
            future_tolerance=self.future_tolerance,
            max_valid_for=self.max_valid_for,
        )
        if reason:
            return reason
        if record.stamp <= self._last_applied_stamp:
            return 'late_observation'

        key = (record.episode_id, record.sender_id, record.sequence)
        sender_key = (record.episode_id, record.sender_id)
        stamp_key = (record.episode_id, record.sender_id, record.stamp)
        if key in self._poisoned or stamp_key in self._conflicted_stamps:
            return 'sequence_conflict' if key in self._poisoned else \
                'sender_stamp_conflict'

        queued = self._pending.get(key)
        previous = self._latest_by_sender.get(sender_key)
        if queued is not None and queued.record != record:
            self._pending.pop(key, None)
            self._poisoned.add(key)
            return 'sequence_conflict'
        if previous is not None and record.sequence == previous[0]:
            if record == previous[2]:
                if queued is not None and source == 'direct':
                    self._pending[key] = SightingCandidate(record, 'direct')
                return 'duplicate'
            self._pending.pop(key, None)
            self._poisoned.add(key)
            return 'sequence_conflict'
        if record.stamp <= self._last_applied_stamp:
            return 'late_observation'

        if previous is not None:
            prev_seq, prev_stamp, prev_record = previous
            if record.stamp == prev_stamp:
                for pending_key, candidate in tuple(self._pending.items()):
                    if (candidate.record.episode_id == record.episode_id
                            and candidate.record.sender_id == record.sender_id
                            and candidate.record.stamp == record.stamp):
                        self._pending.pop(pending_key, None)
                self._conflicted_stamps.add(stamp_key)
                return 'sender_stamp_conflict'
            if record.sequence < prev_seq or record.stamp < prev_stamp:
                return 'out_of_order'

        self._latest_by_sender[sender_key] = (
            record.sequence, record.stamp, record)
        self._pending[key] = SightingCandidate(record, source)
        return None

    def take_for_cycle(self, *, now):
        """Drain a tick's batch and return at most one still-fresh candidate."""
        ready = []
        for key, candidate in tuple(self._pending.items()):
            if key in self._poisoned:
                self._pending.pop(key, None)
                continue
            reason = validate_sighting(
                candidate.record,
                expected_episode=self.expected_episode,
                now=now,
                sighting_timeout=self.sighting_timeout,
                future_tolerance=self.future_tolerance,
                max_valid_for=self.max_valid_for,
            )
            if reason is None and candidate.record.stamp > self._last_applied_stamp:
                ready.append(candidate)
            self._pending.pop(key, None)

        self._poisoned.clear()
        self._conflicted_stamps.clear()
        if not ready:
            return None
        direct = [c for c in ready if c.source == 'direct']
        if direct:
            return max(direct, key=lambda c: (c.record.stamp,
                                               c.record.sequence))
        return min(ready, key=lambda c: (-c.record.stamp,
                                         c.record.sender_id))

    def mark_applied(self, candidate):
        """Record the source observation stamp after its single tracker use."""
        if candidate is None:
            return False
        if candidate.record.episode_id != self.expected_episode:
            return False
        if candidate.record.stamp <= self._last_applied_stamp:
            return False
        self._last_applied_stamp = candidate.record.stamp
        return True


class SightingCounters:
    """Cumulative per-controller sighting counters (phase 2 R-03).

    Reported inside every track-status message (reliable channel); the last
    message per receiver is the receiver's own account, independent of any
    collector loss. A sighting is "in range" when the receiver's own pose is
    within radio_range of the sender position carried in the message; the
    sender's own loopback is counted separately and never as in-range.
    """

    def __init__(self):
        self.published = 0
        self.total = 0
        self.in_range = 0
        self.out_of_range = 0
        self.self_ = 0
        self.unclassified = 0

    def on_published(self):
        self.published += 1

    def on_received(self, is_self, distance, radio_range):
        self.total += 1
        if is_self:
            self.self_ += 1
        elif distance is None:
            self.unclassified += 1
        elif distance > radio_range:
            self.out_of_range += 1
        else:
            self.in_range += 1

    def snapshot(self):
        return {'sightings_published': self.published,
                'sightings_received_total': self.total,
                'sightings_received_in_range': self.in_range,
                'sightings_received_out_of_range': self.out_of_range,
                'sightings_received_self': self.self_,
                'sightings_received_unclassified': self.unclassified}


def shared_sighting_qos_spec(depth):
    """Phase 2 R-01: shared-sighting QoS as plain data (ROS-free, testable).

    BEST_EFFORT/VOLATILE is unchanged from phase 1; only the history depth
    is configurable (default 1 elsewhere keeps the phase-1 behavior).
    """
    depth = int(depth)
    if depth < 1:
        raise ValueError('shared_sighting_qos_depth must be >= 1')
    return {'depth': depth, 'reliability': 'best_effort',
            'durability': 'volatile'}
