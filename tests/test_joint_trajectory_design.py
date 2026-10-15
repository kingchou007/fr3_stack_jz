"""Validate the initial trajectory specification without hardware or transport."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from examples import joint_smoke, joint_sysid


SPEC = json.loads((Path(__file__).resolve().parents[1] /
                   'configs/joint_sysid.initial.json').read_text())
STAGES = SPEC['stages']


def plan_for(stage):
    values = {**SPEC['common_plan'], **stage['plan']}
    for name in ('joints', 'tones'):
        if name in values:
            values[name] = tuple(values[name])
    return joint_sysid.Plan(**values)


@pytest.mark.parametrize('stage', STAGES, ids=lambda stage: stage['id'])
def test_presets_fit_bounds_with_derivative_headroom(stage):
    plan = plan_for(stage)
    peaks = joint_sysid.Trajectory(plan).check(joint_sysid.PREVIEW_Q)
    assert plan.total == stage['expected_duration_s']
    assert max(peaks['offset_rad']) <= np.deg2rad(plan.amplitude_deg) + 1e-12
    for key, limit in (
        ('velocity_rad_s', plan.max_velocity),
        ('acceleration_rad_s2', plan.max_acceleration),
        ('jerk_rad_s3', plan.max_jerk),
    ):
        assert max(peaks[key]) <= .8 * limit


@pytest.mark.parametrize('stage', STAGES, ids=lambda stage: stage['id'])
def test_presets_start_and_end_at_anchor_with_zero_derivatives(stage):
    plan = plan_for(stage)
    times = [0., plan.total]
    for segment in range(plan.segments):
        start = plan.hold + segment * (plan.seconds + plan.gap)
        times.extend([start, start + plan.seconds])
    for value in joint_sysid.Trajectory(plan).evaluate(times):
        np.testing.assert_allclose(value, 0., atol=1e-12)


def test_sequential_chirp_excites_only_one_joint_at_a_time():
    stage = next(stage for stage in STAGES if stage['id'] == '03_chirp_sequential')
    plan = plan_for(stage)
    trajectory = joint_sysid.Trajectory(plan)
    for segment, joint in enumerate(plan.joints):
        start = plan.hold + segment * (plan.seconds + plan.gap)
        values = trajectory.evaluate(np.linspace(start, start + plan.seconds, 2001))[0]
        other = np.delete(values, joint - 1, axis=1)
        assert not other.any()
        assert np.max(np.abs(values[:, joint - 1])) > .99 * np.deg2rad(plan.amplitude_deg)
        if segment < plan.segments - 1:
            for value in trajectory.evaluate([start + plan.seconds + plan.gap / 2]):
                assert not value.any()


def test_smoke_spec_matches_the_existing_checked_response_wrapper():
    stage = next(stage for stage in STAGES if stage['id'] == '01_smoke_j1')
    plan = plan_for(stage)
    assert plan == joint_smoke.smoke_plan()
    assert stage['entrypoint'] == 'examples/joint_smoke.py'
    assert stage['runtime_guards']['tracking_error'] == pytest.approx(.5 * np.deg2rad(plan.amplitude_deg))


def test_multisine_validation_is_distinct_and_reproducible():
    training = plan_for(next(stage for stage in STAGES if stage['id'] == '04_multisine_train'))
    validation = plan_for(next(stage for stage in STAGES if stage['id'] == '05_multisine_validation'))
    assert set(training.tones).isdisjoint(validation.tones)
    assert training.seed != validation.seed
    times = np.linspace(0, training.total, 1001)
    train_values = joint_sysid.Trajectory(training).evaluate(times)[0]
    validation_values = joint_sysid.Trajectory(validation).evaluate(times)[0]
    assert not np.allclose(train_values, validation_values)
    np.testing.assert_array_equal(validation_values, joint_sysid.Trajectory(validation).evaluate(times)[0])


def test_spec_preserves_controller_filter_and_requires_individual_review():
    assert SPEC['profile'] == 'sysid_stiff'
    assert SPEC['required_controller'] == 'joint_impedance'
    assert 'no automatic progression' in SPEC['execution_policy'].lower()
    assert all(stage['gate'] for stage in STAGES)
    for settings in [SPEC['common_plan'], *(stage['plan'] for stage in STAGES)]:
        assert 'filter_alpha' not in settings
        assert set(settings).issubset(joint_sysid.Plan.__dataclass_fields__)
