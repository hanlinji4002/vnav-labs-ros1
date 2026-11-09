#!/usr/bin/env python3
"""Add bounded planned-acceleration preview to the preserved prehover publisher.

The original stable-hover gate and reference clock execute unchanged. Only a
separate desired acceleration topic is added, after real RACING release. No
current state, collision, simulator clock or course topic is published here.
"""
import hashlib
import importlib.util
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def preview_sample(reference, plan_time, phase, preview_s, release_boundary, vertical_weight=1.0):
    """Select future planned acceleration, never future odometry or time."""
    values = (plan_time, preview_s, release_boundary, reference.times[-1], vertical_weight)
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in values):
        raise ValueError('Preview times must be finite numbers')
    if (not 0 <= preview_s <= .04 or plan_time < 0 or
            not 0 < release_boundary < reference.times[-1] or not 0 <= vertical_weight <= 1):
        raise ValueError('Preview range/boundary invalid')
    if phase != 'RACING' or plan_time <= release_boundary or preview_s == 0:
        return None
    target_time = min(plan_time+preview_s, reference.times[-1])
    acceleration = list(reference.sample(target_time)[2])
    if len(acceleration) != 3 or not all(math.isfinite(v) for v in acceleration):
        raise ValueError('Invalid planned preview acceleration')
    if vertical_weight != 1.0:
        current = reference.sample(min(plan_time, reference.times[-1]))[2]
        if len(current) != 3 or not all(math.isfinite(v) for v in current):
            raise ValueError('Invalid current planned acceleration')
        acceleration[2] = current[2]+vertical_weight*(acceleration[2]-current[2])
    return target_time, tuple(float(v) for v in acceleration)


class PreviewPublisher:
    def __init__(self, original, preview, reference, phase, preview_s, boundary,
                 message_factory, timestamp, vertical_weight=1.0):
        self.original = original
        self.preview = preview
        self.reference = reference
        self.phase = phase
        self.preview_s = preview_s
        self.boundary = boundary
        self.message_factory = message_factory
        self.timestamp = timestamp
        self.vertical_weight = vertical_weight

    def publish(self, msg):
        selected = preview_sample(self.reference, msg.time_from_start.to_sec(),
                                  self.phase(), self.preview_s, self.boundary, self.vertical_weight)
        if selected is not None:
            sample = self.message_factory()
            sample.header.stamp = self.timestamp()
            sample.header.frame_id = 'world'
            sample.vector.x, sample.vector.y, sample.vector.z = selected[1]
            self.preview.publish(sample)
        # The exact original desired-state object is forwarded unchanged.
        self.original.publish(msg)

    def __getattr__(self, name):
        return getattr(self.original, name)


def main():
    import rospy
    from geometry_msgs.msg import Vector3Stamped

    base_path = HERE.parent/'prehover_reference.py'
    helper_path = HERE.parent.parent/'scripts/plan_reference.py'
    base = load_module('preserved_prehover_publisher', base_path)
    helper = load_module('preserved_hermite_reference', helper_path)
    original_clock, original_publisher = base.ReleaseClock, rospy.Publisher
    phase = {'value': 'PREPARING'}
    owned_publishers = []

    class ObservedReleaseClock(original_clock):
        def advance(self, now, stable_ready):
            result = super().advance(now, stable_ready)
            phase['value'] = self.phase
            return result

    def publisher(name, *args, **kwargs):
        actual = original_publisher(name, *args, **kwargs)
        if name != '/desired_state':
            return actual
        source = Path(rospy.get_param('~plan')).resolve()
        reference = helper.Reference(helper.load_plan(source))
        seconds = float(rospy.get_param('~attitude_preview_s', 0.0))
        boundary = float(rospy.get_param('~hover_release_t', 5.0))
        vertical_weight = float(rospy.get_param('~preview_vertical_weight', 1.0))
        preview_sample(reference, 0.0, 'PREPARING', seconds, boundary, vertical_weight)
        preview = original_publisher('/sprint_acceleration_preview', Vector3Stamped, queue_size=1)
        owned_publishers.append(preview)
        output = Path(rospy.get_param('~dump_file')).resolve().parent/'preview_settings.json'
        evidence = {
            'preview_s': seconds, 'release_boundary_s': boundary,
            'preview_vertical_weight': vertical_weight,
            'enabled_only_phase': 'RACING', 'strictly_after_release_boundary': True,
            'terminal_sample_clamped': True, 'modifies_current_state_or_clock': False,
            'source_hashes': {str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in (Path(__file__), base_path, helper_path, source)}}
        with output.open('x') as stream:
            stream.write(json.dumps(evidence, indent=2)+'\n')
        return PreviewPublisher(actual, preview, reference, lambda:phase['value'],
                                seconds, boundary, Vector3Stamped, rospy.Time.now, vertical_weight)

    base.ReleaseClock = ObservedReleaseClock
    rospy.Publisher = publisher
    try:
        base.main()
    finally:
        rospy.Publisher = original_publisher
        base.ReleaseClock = original_clock
        for publisher in owned_publishers:
            publisher.unregister()


if __name__ == '__main__':
    main()
