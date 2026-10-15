"""Joint adaptation of FrankaTwin excitation; no physical robot motion."""
from dataclasses import replace
import json

import numpy as np
import pytest

from examples import joint_sysid as js, joint_trajectory as runner
from fr3_stack.wire import SCHEMA


@pytest.mark.parametrize('name', ['twin-chirp', 'twin-all', 'twin-validation', 'twin-large'])
def test_twin_reference_derivatives_and_boundaries(name):
    _, stage, plan, _, _ = runner.load_stage(name)
    trajectory = js.Trajectory(plan)
    assert plan.segments == 1
    assert plan.total == stage['expected_duration_s']
    times = np.array([3.7, 8.1, 14.2, plan.total - 4.1])
    h = 1e-5
    left, right, exact = trajectory.evaluate(times - h), trajectory.evaluate(times + h), trajectory.evaluate(times)
    for order in range(1, 4):
        np.testing.assert_allclose((right[order - 1] - left[order - 1]) / (2 * h),
                                   exact[order], atol=1e-8, rtol=1e-5)
    for value in trajectory.evaluate([0., plan.hold, plan.hold + plan.seconds, plan.total]):
        np.testing.assert_allclose(value, 0., atol=1e-12)


def test_all_joint_chirp_is_simultaneous_with_fixed_joint_phases():
    _, _, plan, _, _ = runner.load_stage('twin-all')
    trajectory = js.Trajectory(plan)
    offsets = trajectory.evaluate([plan.hold + 10.])[0][0]
    assert np.count_nonzero(offsets) == 7
    for joint in range(1, 8):
        _, _, phases = trajectory.joint_tones(joint)
        assert phases[0] == pytest.approx(2 * np.pi * (joint - 1) / 7)
        _, _, selected, _, _ = runner.load_stage('twin-chirp', joint)
        np.testing.assert_array_equal(js.Trajectory(selected).evaluate([13.])[0][:, joint - 1],
                                      trajectory.evaluate([13.])[0][:, joint - 1])
        assert not np.delete(js.Trajectory(selected).evaluate([13.])[0], joint - 1, axis=1).any()


def test_larger_chirp_keeps_motion_limits_and_explicit_half_amplitude_guard(tmp_path, capsys):
    _, _, baseline, old_guards, _ = runner.load_stage('twin-all')
    _, _, plan, guards, _ = runner.load_stage('twin-large')
    assert plan.amplitude_deg == 5 * baseline.amplitude_deg == 1.
    assert (plan.f0, plan.f1) == (.05, .15)
    assert plan.joints == tuple(range(1, 8))
    assert plan.chirp_layout == 'simultaneous'
    assert (plan.max_velocity, plan.max_acceleration, plan.max_jerk) == (
        baseline.max_velocity, baseline.max_acceleration, baseline.max_jerk)
    assert guards['tracking_error'] == pytest.approx(np.deg2rad(.5))
    assert guards['tracking_error'] < np.deg2rad(plan.amplitude_deg)
    assert guards['stale_after'] == old_guards['stale_after']
    assert guards['max_lateness'] == old_guards['max_lateness']
    output = tmp_path / 'large-preview'
    assert runner.main(['--stage', 'twin-large', '--dry-run', '--output', str(output)]) == 0
    printed = capsys.readouterr().out
    assert '+/-1 deg' in printed and '(0.5 deg)' in printed
    data = json.loads((output / 'metadata.json').read_text())
    assert data['commands_sent'] == 0
    assert data['guard_settings'] == guards
    assert data['joint_impedance_profile'] == 'sysid_stiff'


def test_frankatwin_tones_are_distinct_normalized_and_velocity_tapered():
    _, _, plan, _, _ = runner.load_stage('twin-validation')
    trajectory = js.Trajectory(plan)
    frequencies = []
    for joint in plan.joints:
        f, w, phase = trajectory.joint_tones(joint)
        frequencies.extend(f)
        np.testing.assert_array_equal(f, js.FRANKATWIN_TONES[joint - 1])
        raw = np.minimum(1., .35 / f)
        np.testing.assert_allclose(w, raw / raw.sum())
        np.testing.assert_allclose(phase, ((joint - 1) * np.pi / 3 + np.arange(3) * np.pi / 2) % (2 * np.pi))
        assert w.sum() == pytest.approx(1.)
    assert len(frequencies) == len(set(frequencies)) == 21
    assert max(frequencies) == 2.
    assert np.max(np.abs(trajectory.evaluate(np.linspace(0, plan.total, 10001))[0])) <= np.deg2rad(plan.amplitude_deg)


@pytest.mark.parametrize('changes', [
    dict(chirp_layout='invalid'), dict(chirp_layout='simultaneous'),
    dict(tone_profile='invalid'), dict(tone_profile='frankatwin'),
    dict(mode='multisine', tone_profile='frankatwin', seed=1),
    dict(mode='multisine', tone_profile='frankatwin', tones=(.2, .3)),
    dict(mode='multisine', tone_profile='frankatwin', joints=(1, 2, 3), rate=30),
])
def test_invalid_waveform_settings_fail_before_execution(changes):
    with pytest.raises(ValueError):
        js.Trajectory(replace(js.Plan(), **changes))


def test_recording_reference_and_source_provenance_are_saved(tmp_path):
    output = tmp_path / 'preview'
    assert runner.main(['--stage', 'twin-all', '--dry-run', '--nuc-log', '/recordings/test.csv',
                        '--output', str(output)]) == 0
    metadata = json.loads((output / 'metadata.json').read_text())
    assert metadata['nuc_log_reference'] == '/recordings/test.csv'
    assert not metadata['nuc_log_verified']
    assert metadata['commands_sent'] == 0
    assert metadata['reference_waveform']['chirp_layout'] == 'simultaneous'
    assert len(metadata['reference_waveform']['joints']) == 7
    assert metadata['trajectory_campaign']['reference_design']['revision'] == 'acf16009c09c99c9a2bb86050b57eb248982ba9f'


@pytest.mark.parametrize('mode, extra', [
    ('chirp', ['--chirp-layout', 'simultaneous', '--f1', '.7']),
    ('multisine', ['--tone-profile', 'frankatwin']),
])
def test_twin_waveform_reaches_existing_joint_wire_with_unchanged_gains(
        daemon_streaming, tmp_path, monkeypatch, mode, extra):
    monkeypatch.delenv('FR3_CONFIG_DIR', raising=False)
    output = tmp_path / mode
    with daemon_streaming.publish_loop(controller='joint_impedance', q=js.PREVIEW_Q.tolist()):
        assert js.main(['--execute', '--host', '127.0.0.1', '--robot-id', 'test',
                        '--cmd-port', str(daemon_streaming.cmd_port),
                        '--state-port', str(daemon_streaming.state_port),
                        '--profile', 'sysid_stiff', '--mode', mode,
                        '--joints', '1', '2', '3', '4', '5', '6', '7',
                        '--amplitude-deg', '.00001', '--seconds', '.12', '--ramp', '.05',
                        '--hold', '.02', '--rate', '100', '--max-lateness', '.2',
                        '--output', str(output), *extra]) == 0
    messages = daemon_streaming.drain_commands()
    assert messages
    for payload in messages:
        with SCHEMA.Command.from_bytes(payload) as command:
            assert not command.termination
            assert command.config.which() == 'jointImpedance'
            assert list(command.config.jointImpedance.kJoint) == [700., 700., 700., 700., 300., 180., 60.]
            assert list(command.config.jointImpedance.dJoint) == [55., 55., 55., 55., 32., 27., 16.]
            assert command.config.jointImpedance.filterAlpha == .05
