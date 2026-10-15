"""Small-motion smoke checks with synthetic feedback and localhost transport."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from examples import joint_smoke as smoke
from fr3_stack.state import State


SCRIPT = Path(__file__).resolve().parents[1] / 'examples' / 'joint_smoke.py'


def test_preset_bounds_and_smooth_return():
    plan = smoke.smoke_plan()
    assert plan.total == 26.
    assert plan.joints == (1,)
    assert plan.amplitude_deg == .25
    trajectory = smoke.excitation.Trajectory(plan)
    peaks = trajectory.check(smoke.excitation.PREVIEW_Q)
    assert max(peaks['offset_rad']) <= np.deg2rad(.25)
    for value in trajectory.evaluate([0., plan.hold, plan.hold + plan.seconds, plan.total]):
        np.testing.assert_allclose(value, 0., atol=1e-12)
    assert peaks['velocity_rad_s'][0] < .01
    smoke.excitation.Trajectory(smoke.smoke_plan(7, .5)).check(smoke.excitation.PREVIEW_Q)


@pytest.mark.parametrize('options', [
    ['--amplitude-deg', '0'], ['--amplitude-deg', '.05'],
    ['--amplitude-deg', '.51'], ['--amplitude-deg', 'nan'],
    ['--amplitude-deg', 'inf'], ['--check-only'],
    ['--execute', '--host', 'must-not-connect.invalid'],
    ['--check-only', '--host', 'user@127.0.0.1'],
    ['--cmd-port', '5556'], ['--state-port', '0'],
])
def test_invalid_invocations_do_not_reach_excitation(tmp_path, monkeypatch, options):
    def reject(*args, **kwargs):
        pytest.fail('invalid smoke settings reached the excitation entrypoint')
    monkeypatch.setattr(smoke.excitation, 'main', reject)
    monkeypatch.chdir(tmp_path)
    assert smoke.main(options) == 1
    assert not (tmp_path / 'recordings').exists()


@pytest.mark.parametrize('mode', ['--check-only', '--execute'])
def test_unresolvable_host_fails_before_robot_creation(tmp_path, monkeypatch, capsys, mode):
    import fr3_stack.robot as robot_module

    def unresolved(*args, **kwargs):
        raise smoke.socket.gaierror('name not known')

    def reject_robot(*args, **kwargs):
        pytest.fail('unresolvable host created a Robot')

    monkeypatch.setattr(smoke.socket, 'getaddrinfo', unresolved)
    monkeypatch.setattr(robot_module, 'Robot', reject_robot)
    monkeypatch.chdir(tmp_path)
    assert smoke.main([mode, '--host', 'login-name', '--robot-id', 'test']) == 1
    error = capsys.readouterr().err
    assert 'cannot resolve NUC host' in error
    assert 'actual IP/hostname' in error
    assert not (tmp_path / 'recordings').exists()


def test_connection_timeout_reports_the_state_endpoint(monkeypatch, capsys):
    import fr3_stack.robot as robot_module

    class TimedOut:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def wait_for_state(self, timeout):
            raise TimeoutError('no state from daemon')

    monkeypatch.setattr(robot_module, 'Robot', lambda *args, **kwargs: TimedOut())
    assert smoke.main(['--check-only', '--host', '127.0.0.1', '--state-port', '12345']) == 1
    error = capsys.readouterr().err
    assert 'tcp://127.0.0.1:12345' in error
    assert 'NUC daemon is running' in error


def test_offline_preview_has_no_transport_import_and_preserves_existing_logs(tmp_path):
    output = tmp_path / 'preview'
    code = '''
import runpy, sys
from pathlib import Path
class RejectTransport:
    def find_spec(self, fullname, *args):
        if fullname.startswith(('fr3_stack', 'zmq', 'capnp')):
            raise RuntimeError('preview tried to import hardware transport')
sys.meta_path.insert(0, RejectTransport())
sys.path.insert(0, str(Path(sys.argv[1]).parent))
sys.argv = [sys.argv[1], '--host', 'must-not-connect.invalid', '--output', sys.argv[2]]
runpy.run_path(sys.argv[0], run_name='__main__')
'''
    result = subprocess.run([sys.executable, '-c', code, str(SCRIPT), str(output)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    metadata = json.loads((output / 'metadata.json').read_text())
    assert metadata['status'] == metadata['smoke_test']['status'] == 'preview'
    assert metadata['plan']['amplitude_deg'] == .25
    assert metadata['joint_impedance_profile'] == 'sysid_stiff'
    assert not metadata['motion_command_attempted']
    before = (output / 'metadata.json').read_bytes()
    assert smoke.main(['--output', str(output)]) == 1
    assert (output / 'metadata.json').read_bytes() == before


@pytest.mark.parametrize('controller,expected', [('joint_impedance', 0), ('idle', 1)])
def test_connection_check_sends_no_commands(daemon_streaming, controller, expected, tmp_path, monkeypatch):
    monkeypatch.delenv('FR3_CONFIG_DIR', raising=False)
    monkeypatch.chdir(tmp_path)
    with daemon_streaming.publish_loop(controller=controller, q=smoke.excitation.PREVIEW_Q.tolist()):
        assert smoke.main(['--check-only', '--host', '127.0.0.1',
                           '--cmd-port', str(daemon_streaming.cmd_port),
                           '--state-port', str(daemon_streaming.state_port)]) == expected
    assert not daemon_streaming.drain_commands()
    assert not (tmp_path / 'recordings').exists()


class Clock:
    def __init__(self):
        self.now = 10.

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def test_stationary_feedback_cannot_pass_response_check(tmp_path):
    anchor = smoke.excitation.PREVIEW_Q
    offsets = [0., np.deg2rad(.25), -np.deg2rad(.25), 0.]
    with (tmp_path / 'commands_and_state.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([f'{key}{j}' for key in ('q_target', 'q_measured', 'dq_measured') for j in range(1, 8)])
        for offset in offsets:
            target = anchor.copy()
            target[0] += offset
            writer.writerow([*target, *anchor, *np.zeros(7)])
    result = smoke.check_response(tmp_path, anchor, 1, .25)
    assert result['status'] == 'failed'
    assert 'expected joint motion was not observed' in result['errors']


class TrackingRobot:
    def __init__(self, clock, *, follow=True, fail_send=False, final_error=0., final_speed=0.):
        self.clock = clock
        self.anchor = smoke.excitation.PREVIEW_Q + np.array([.12, 0., .15, 0., -.1, 0., 0.])
        self.q = self.anchor.copy()
        self.follow = follow
        self.fail_send = fail_send
        self.final_error = final_error
        self.final_speed = final_speed
        self.sent = []
        self.terminated = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    @property
    def state(self):
        q, dq = self.q.copy(), np.zeros(7)
        if self.clock() >= 36.:
            q[0] += self.final_error
            dq[0] = self.final_speed
        return State(valid=True, running=True, controller='joint_impedance',
                     q=q, dq=dq, received_at=self.clock(), timestamp=self.clock())

    def wait_for_state(self, timeout):
        return self.state

    def send_joint_impedance(self, target, **gains):
        if self.fail_send:
            raise RuntimeError('disconnected command path')
        self.sent.append((target.copy(), gains))
        if self.follow:
            self.q = target.copy()

    def terminate(self):
        self.terminated = True


@pytest.mark.parametrize('follow,fail_send,final_error,final_speed,expected', [
    (True, False, 0., 0., 0), (False, False, 0., 0., 1),
    (True, True, 0., 0., 1), (True, False, .0021, 0., 1),
    (True, False, 0., .007, 1),
])
def test_full_preset_observes_response_and_records_failures(tmp_path, monkeypatch, follow, fail_send,
                                                          final_error, final_speed, expected):
    import fr3_stack.robot as robot_module

    clock = Clock()
    robot = TrackingRobot(clock, follow=follow, fail_send=fail_send,
                          final_error=final_error, final_speed=final_speed)
    original_stream = smoke.excitation.stream
    def stream(*args, **kwargs):
        return original_stream(*args, **kwargs, clock=clock, sleep=clock.sleep)
    monkeypatch.setattr(robot_module, 'Robot', lambda *args, **kwargs: robot)
    monkeypatch.setattr(smoke.excitation, 'time', SimpleNamespace(monotonic=clock, sleep=clock.sleep))
    monkeypatch.setattr(smoke.excitation, 'stream', stream)
    monkeypatch.delenv('FR3_CONFIG_DIR', raising=False)
    monkeypatch.chdir(tmp_path)
    output = tmp_path / 'smoke'
    assert smoke.main(['--execute', '--host', '127.0.0.1', '--robot-id', 'test',
                       '--output', str(output)]) == expected
    metadata = json.loads((output / 'metadata.json').read_text())
    np.testing.assert_array_equal(metadata['anchor_q_rad'], robot.anchor)
    if fail_send or not follow:
        assert metadata['status'] == 'aborted'
        assert robot.terminated
        if not follow:
            assert 'tracking error' in metadata['error']
        return
    assert not robot.terminated
    assert metadata['smoke_test']['status'] == ('passed' if expected == 0 else 'failed')
    assert metadata['status'] == ('completed' if expected == 0 else 'failed')
    assert metadata['smoke_test']['command_application_acknowledged'] is False
    assert metadata['sysid_ready'] is False
    for target, gains in robot.sent:
        assert abs(target[0] - robot.anchor[0]) <= np.deg2rad(.25) + 1e-12
        np.testing.assert_array_equal(target[1:], robot.anchor[1:])
        assert gains['K_joint'] == [700., 700., 700., 700., 300., 180., 60.]
        assert gains['filter_alpha'] == .05
    np.testing.assert_array_equal(robot.sent[0][0], robot.anchor)
    np.testing.assert_array_equal(robot.sent[-1][0], robot.anchor)
    if final_error or final_speed:
        assert 'feedback did not settle back at the anchor' in metadata['smoke_test']['errors']
