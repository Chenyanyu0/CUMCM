"""Question 3 experiment: seven-station discovery plus active triangulation.

This file is intentionally separate from robot_solver.py.  It uses one probe
at the origin and six probes on a radius-1123 m hexagon for first discovery.
After a channel returns a bearing, a second probe is selected by maximizing
the baseline angle to the first bearing while accounting for travel time.
"""
from __future__ import annotations

import argparse
import json
import math
import sys

from robot_solver import (
    DEG, R, Observation, Simulator, clear_neighborhood,
    refine_location, robust_location,
)

DISCOVERY_RADIUS = 1123.0


def seven_probe_points():
    points = [(0.0, 0.0)]
    for k in range(6):
        a = 2.0 * math.pi * k / 6.0
        points.append((DISCOVERY_RADIUS * math.cos(a),
                       DISCOVERY_RADIUS * math.sin(a)))
    return points


def angle_quality(obs, point):
    """Quality of a second viewpoint for the first bearing observation."""
    baseline = math.degrees(math.atan2(point[1] - obs.y, point[0] - obs.x))
    delta = abs((baseline - obs.bearing + 180.0) % 360.0 - 180.0)
    # 1 at 90 degrees, 0 at a collinear baseline.
    return abs(math.sin(delta * DEG))


def second_probe(sim, first, candidates, tested):
    scored = []
    for p in candidates:
        key = (round(p[0], 6), round(p[1], 6))
        if key in tested:
            continue
        distance = math.hypot(p[0] - sim.pos[0], p[1] - sim.pos[1])
        # Travel time is in seconds; the angle term rewards stable
        # intersections.  The constants are deliberately conservative.
        score = distance / 5.0 + 5.0 - 10.0 * angle_quality(first, p)
        scored.append((score, p))
    return min(scored, default=(None, None))[1]


def run(sim):
    """Seven discovery stations with a shared dynamic task queue."""
    probes = seven_probe_points()
    state = {ch: {'status': '未发现', 'obs': [], 'tested': set(),
                  'estimate': None, 'failed_obs_count': -1,
                  'retry_level': 0}
             for ch in range(1, 21)}
    active = set(state)
    unvisited = set(range(len(probes)))
    pending_second = set()
    clear_queue = []
    actions = 0
    max_actions = 1000

    def key(p):
        return (round(p[0], 6), round(p[1], 6))

    def second_candidates(ch):
        st = state[ch]
        result = list(probes)
        # Generate side-looking points from every observed bearing. If the
        # first two rays are noisy or nearly parallel, later observations can
        # still supply a well-conditioned baseline.
        for obs in st['obs']:
            a = obs.bearing * DEG
            for length in (250.0, 400.0, 600.0, 800.0, 1000.0, 1200.0):
                for side in (-1.0, 1.0):
                    q = (obs.x - side * length * math.sin(a),
                         obs.y + side * length * math.cos(a))
                    if math.hypot(*q) <= R:
                        result.append(q)
            # Local rings keep the second point inside the source's receive
            # disk when the first observation was near its 1000 m boundary.
            for radius in (300.0, 600.0, 900.0):
                for k in range(12):
                    t = 2.0 * math.pi * k / 12.0
                    q = (obs.x + radius * math.cos(t),
                         obs.y + radius * math.sin(t))
                    if math.hypot(*q) <= R:
                        result.append(q)
        return [p for p in result if key(p) not in st['tested']]

    def enqueue_estimates():
        for ch in list(active):
            st = state[ch]
            if len(st['obs']) < 2 or len(st['obs']) <= st['failed_obs_count']:
                continue
            if st['status'] != '有方向':
                continue
            loc = robust_location(st['obs'][-10:])
            if loc is None or math.hypot(*loc) > R + 20:
                continue
            st['estimate'] = refine_location(st['obs'][-10:], loc)
            st['status'] = '已定位'
            if not any(item[0] == ch for item in clear_queue):
                clear_queue.append((ch, st['estimate'], len(st['obs'])))

    def execute_clear(item):
        ch, loc, obs_count = item
        if ch not in active:
            return
        if clear_neighborhood(sim, loc, ch):
            state[ch]['status'] = '已清除'
            active.remove(ch)
            pending_second.discard(ch)
            return
        # A failed clear invalidates this estimate. Keep the channel pending
        # and require a new bearing before trying another clear.
        st = state[ch]
        st['status'] = '有方向'
        st['estimate'] = None
        st['failed_obs_count'] = obs_count
        st['retry_level'] += 1
        st['tested'].clear()
        pending_second.add(ch)

    def task_candidates():
        candidates = []
        for item in clear_queue:
            ch, loc, _ = item
            d = math.hypot(loc[0] - sim.pos[0], loc[1] - sim.pos[1])
            candidates.append((d / 5.0 + 5.0 - 15.0, 'clear', item))
        for ch in sorted(pending_second):
            if ch not in active or not state[ch]['obs']:
                continue
            qs = second_candidates(ch)
            if not qs:
                continue
            # Pick the best candidate for this channel, but let it compete
            # globally with discovery and other channels.
            first = state[ch]['obs'][0]
            q = min(qs, key=lambda p: math.hypot(p[0] - sim.pos[0], p[1] - sim.pos[1])
                    - 900.0 * angle_quality(first, p))
            d = math.hypot(q[0] - sim.pos[0], q[1] - sim.pos[1])
            quality = angle_quality(first, q)
            candidates.append((d / 5.0 + 6.0 - 10.0 * quality,
                               'second', (ch, q)))
        if unvisited:
            for idx in unvisited:
                p = probes[idx]
                d = math.hypot(p[0] - sim.pos[0], p[1] - sim.pos[1])
                need = sum(len(state[ch]['obs']) == 0 for ch in active)
                candidates.append((d / 5.0 + 5.0 * max(1, need) - 4.0 * need,
                                   'discover', idx))
        return candidates

    def finish_directional(max_actions=260):
        """Use the remaining action budget to drain directional channels."""
        exhausted = set()
        recovery_counts = {ch: 0 for ch in active}
        max_channel_attempts = 12
        for _ in range(max_actions):
            enqueue_estimates()
            if clear_queue:
                item = min(clear_queue,
                           key=lambda x: math.hypot(x[1][0] - sim.pos[0],
                                                    x[1][1] - sim.pos[1]))
                clear_queue.remove(item)
                ch = item[0]
                execute_clear(item)
                # A failed clear resets tested, so this channel can be
                # retried in the recovery pass.
                exhausted.discard(ch)
                recovery_counts[ch] = 0
                continue
            pending = [ch for ch in sorted(active)
                       if state[ch]['status'] == '有方向'
                       and ch not in exhausted]
            if not pending:
                return
            # Prefer channels with only one bearing, then channels whose
            # estimate could not be cleared and need a fresh bearing.
            ch = min(pending, key=lambda c: len(state[c]['obs']))
            st = state[ch]
            if recovery_counts.get(ch, 0) >= max_channel_attempts:
                exhausted.add(ch)
                continue
            qs = second_candidates(ch)
            if not qs:
                exhausted.add(ch)
                continue
            first = st['obs'][0]
            p = min(qs, key=lambda q: (
                math.hypot(q[0] - sim.pos[0], q[1] - sim.pos[1])
                - 900.0 * angle_quality(first, q)))
            ans = sim.measure(p, ch)
            recovery_counts[ch] = recovery_counts.get(ch, 0) + 1
            st['tested'].add(key(p))
            result = ans.get('measure_result')
            if result == 'direction':
                st['obs'].append(Observation(p[0], p[1], ch, result,
                                             float(ans['svd_deg']),
                                             ans.get('virtual_time_s', 0)))
                st['status'] = '有方向'
            elif result == 'near':
                st['status'] = '已定位'
                pending_second.discard(ch)
                clear_queue.append((ch, p, len(st['obs'])))
            # no_signal consumes this candidate; after all candidates are
            # exhausted, this channel is left untouched rather than looping
            # over the same measurements.
        enqueue_estimates()
        while clear_queue:
            item = clear_queue.pop(0)
            execute_clear(item)

    while actions < max_actions:
        enqueue_estimates()
        candidates = task_candidates()
        if not candidates:
            break
        _, kind, payload = min(candidates, key=lambda x: x[0])
        actions += 1
        if kind == 'clear':
            clear_queue.remove(payload)
            execute_clear(payload)
            continue
        if kind == 'second':
            ch, p = payload
            st = state[ch]
            ans = sim.measure(p, ch)
            st['tested'].add(key(p))
            result = ans.get('measure_result')
            if result == 'direction':
                st['obs'].append(Observation(p[0], p[1], ch, result,
                                             float(ans['svd_deg']),
                                             ans.get('virtual_time_s', 0)))
                st['status'] = '有方向'
                # Do not remove the channel until the new bearings actually
                # yield a valid in-arena intersection. This prevents a
                # channel from ending in the misleading "有方向" state.
                pending_second.add(ch)
                enqueue_estimates()
                if st['status'] == '已定位':
                    pending_second.discard(ch)
            elif result == 'near':
                st['status'] = '已定位'
                pending_second.discard(ch)
                clear_queue.append((ch, p, len(st['obs'])))
            continue

        idx = payload
        p = probes[idx]
        unvisited.remove(idx)
        # Discovery stations only measure channels with no bearing yet.
        for ch in sorted(active):
            st = state[ch]
            if st['obs']:
                continue
            ans = sim.measure(p, ch)
            st['tested'].add(key(p))
            result = ans.get('measure_result')
            if result == 'direction':
                st['status'] = '有方向'
                st['obs'].append(Observation(p[0], p[1], ch, result,
                                             float(ans['svd_deg']),
                                             ans.get('virtual_time_s', 0)))
                pending_second.add(ch)
            elif result == 'near':
                st['status'] = '已定位'
                clear_queue.append((ch, p, 0))
            else:
                st['status'] = '无信号'
    finish_directional()
    enqueue_estimates()
    # A final localization pass is required because the last action may have
    # produced the second bearing immediately before the action budget ended.
    for ch in list(active):
        st = state[ch]
        if st['status'] == '有方向' and len(st['obs']) >= 2:
            loc = robust_location(st['obs'][-10:])
            if loc is not None and math.hypot(*loc) <= R + 20:
                st['estimate'] = refine_location(st['obs'][-10:], loc)
                st['status'] = '已定位'
                clear_queue.append((ch, st['estimate'], len(st['obs'])))
    while clear_queue:
        item = clear_queue.pop(0)
        execute_clear(item)
    # A failed final clear reopens the channel for one bounded recovery pass.
    finish_directional(max_actions=120)
    return state


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot-id', required=True)
    ap.add_argument('--base', default='http://127.0.0.1:2026')
    ap.add_argument('--log', default='robot_log_q3_seven.jsonl')
    args = ap.parse_args()
    sim = Simulator(args.base, args.robot_id, args.log)
    sim.enter()
    state = run(sim)
    sim.exit()
    sim.log.close()
    cleared = sorted(ch for ch, st in state.items() if st['status'] == '已清除')
    print(json.dumps({'cleared_channels': cleared, 'count': len(cleared),
                      'states': {str(ch): st['status'] for ch, st in state.items()}},
                     ensure_ascii=False))


if __name__ == '__main__':
    main()
