"""One-stage campaign entrypoint: offline previews and fake-daemon preflight."""
from __future__ import annotations

from dataclasses import asdict
import errno
import json
from pathlib import Path
import subprocess
import sys

import pytest

from examples import joint_trajectory as runner


SCRIPT = Path(runner.__file__).resolve()


@pytest.mark.parametrize('name', runner.STAGES)
def test_stage_matches_specification(name):
    spec, stage, plan, guards, checksum = runner.load_stage(name)
    settings = {**spec['common_plan'], **stage['plan']}
    for key in ('joints', 'tones'):
        if key in settings:
            settings[key] = tuple(settings[key])
    assert plan == runner.excitation.Plan(**settings)
    assert plan.total == stage['expected_duration_s']
    assert guards == {**spec['runtime_guards'], **stage.get('runtime_guards', {})}
    assert len(checksum) == 64


@pytest.mark.parametrize('name', runner.STAGES)
def test_dispatches_exactly_one_stage_and_preserves_parameters(tmp_path, monkeypatch, name):
    calls = []

    def core(options):
        calls.append(('core', runner.excitation.parser().parse_args(options)))
        return 1

    def smoke(options):
        calls.append(('smoke', runner.smoke.parser().parse_args(options)))
        return 1

    monkeypatch.setattr(runner.excitation, 'main', core)
    monkeypatch.setattr(runner.smoke, 'main', smoke)
    output = tmp_path / name
    assert runner.main(['--stage', name, '--output', str(output)]) == 1
    assert len(calls) == 1
    entrypoint, args = calls[0]
    spec, stage, plan, guards, checksum = runner.load_stage(name)
    assert not args.execute
    assert args.profile == 'sysid_stiff'
    assert args.output == output
    if name == 'smoke':
        assert entrypoint == 'smoke'
        assert runner.smoke.smoke_plan(args.joint, args.amplitude_deg) == plan
    else:
        assert entrypoint == 'core'
        actual = {key: getattr(args, key) for key in asdict(plan)}
        actual['joints'], actual['tones'] = tuple(actual['joints']), tuple(actual['tones'])
        assert runner.excitation.Plan(**actual) == plan
        assert {key: getattr(args, key) for key in guards} == guards
        assert args.filter_alpha is None
        assert args.kp is args.kd is None
        assert args.finish == 'hold'


@pytest.mark.parametrize('options', [
    ['--execute'], ['--execute', '--host', '127.0.0.1'],
    ['--execute', '--host', 'user@127.0.0.1', '--robot-id', 'test'],
    ['--execute', '--host', ' ', '--robot-id', 'test'],
    ['--cmd-port', '5556'], ['--state-port', '0'],
    ['--stage', 'train', '--joint', '2'], ['--stage', 'hold', '--joint', '1'],
    ['--stage', 'sequential', '--joint', '2'],
])
def test_invalid_settings_do_not_dispatch(tmp_path, monkeypatch, options):
    def reject(*args, **kwargs):
        pytest.fail('invalid invocation reached a trajectory entrypoint')

    monkeypatch.setattr(runner.excitation, 'main', reject)
    monkeypatch.setattr(runner.smoke, 'main', reject)
    monkeypatch.chdir(tmp_path)
    assert runner.main(options) == 1
    assert not (tmp_path / 'recordings').exists()


def test_unresolvable_host_does_not_dispatch(tmp_path, monkeypatch):
    def unresolved(*args, **kwargs):
        raise runner.socket.gaierror('unknown host')

    def reject(*args, **kwargs):
        pytest.fail('unresolvable host reached execution')

    monkeypatch.setattr(runner.socket, 'getaddrinfo', unresolved)
    monkeypatch.setattr(runner.smoke, 'main', reject)
    assert runner.main(['--execute', '--host', 'login-name', '--robot-id', 'test',
                        '--output', str(tmp_path / 'run')]) == 1
    assert not (tmp_path / 'run').exists()


@pytest.mark.parametrize('name', ['smoke', 'chirp'])
def test_single_joint_selection(name):
    spec, stage, plan, guards, checksum = runner.load_stage(name, 7)
    assert plan.joints == (7,)
    assert plan.total == stage['expected_duration_s']


@pytest.mark.parametrize('change', ['schema', 'duration', 'guard', 'smoke'])
def test_inconsistent_specification_rejected_before_dispatch(tmp_path, monkeypatch, change):
    spec = json.loads(runner.SPEC_PATH.read_text())
    if change == 'schema':
        spec['schema'] = 'unsupported'
    elif change == 'duration':
        spec['stages'][1]['expected_duration_s'] += 1
    elif change == 'guard':
        spec['runtime_guards']['stale_after'] = float('nan')
    else:
        spec['stages'][1]['plan']['seconds'] += 1
        spec['stages'][1]['expected_duration_s'] += 1
    path = tmp_path / 'campaign.json'
    path.write_text(json.dumps(spec))
    monkeypatch.setattr(runner, 'SPEC_PATH', path)
    monkeypatch.chdir(tmp_path)
    assert runner.main([]) == 1
    assert not (tmp_path / 'recordings').exists()


def test_default_is_offline_smoke(tmp_path, monkeypatch):
    calls = []

    def smoke(options):
        calls.append(runner.smoke.parser().parse_args(options))
        return 1

    monkeypatch.setattr(runner.smoke, 'main', smoke)
    monkeypatch.chdir(tmp_path)
    assert runner.main([]) == 1
    assert len(calls) == 1
    assert calls[0].dry_run and not calls[0].execute
    assert calls[0].joint == 1
    assert calls[0].amplitude_deg == .25


@pytest.mark.parametrize('name', ['hold', 'smoke'])
def test_default_output_avoids_read_only_checkout_recordings(tmp_path, monkeypatch, name):
    recordings = tmp_path / 'recordings'
    recordings.mkdir()
    recordings.chmod(0o555)
    home = tmp_path / 'home'
    home.mkdir()
    monkeypatch.setenv('HOME', str(home))
    monkeypatch.chdir(tmp_path)
    try:
        code = runner.main(['--stage', name, '--dry-run'])
        assert not list(recordings.iterdir())
    finally:
        recordings.chmod(0o755)
    assert code == 0
    outputs = list((home / 'fr3-client-recordings').iterdir())
    assert len(outputs) == 1
    assert outputs[0].name.startswith(f'trajectory-{name}-')
    metadata = json.loads((outputs[0] / 'metadata.json').read_text())
    assert metadata['status'] == 'preview'
    assert metadata['commands_sent'] == 0
    assert not metadata['motion_command_attempted']
    assert metadata['trajectory_campaign']['stage'] == name
    assert (outputs[0] / 'reference.csv').is_file()
    assert (outputs[0] / 'preview.svg').is_file()


@pytest.mark.parametrize('name', ['hold', 'smoke'])
def test_explicit_unwritable_output_fails_before_connection(tmp_path, monkeypatch, capsys, name):
    output = tmp_path / 'blocked' / 'run'
    mkdir = Path.mkdir

    def deny_output(path, *args, **kwargs):
        if path == output:
            raise PermissionError(errno.EACCES, 'Permission denied', str(path))
        return mkdir(path, *args, **kwargs)

    def reject(*args, **kwargs):
        pytest.fail('unwritable output reached robot connection or gain resolution')

    monkeypatch.setattr(Path, 'mkdir', deny_output)
    monkeypatch.setattr('fr3_stack.robot.Robot', reject)
    monkeypatch.setattr(runner.excitation, 'resolve_joint_gains', reject)
    monkeypatch.setenv('HOME', str(tmp_path / 'home'))
    assert runner.main(['--stage', name, '--execute', '--host', '127.0.0.1',
                        '--robot-id', 'test', '--output', str(output)]) == 1
    assert 'Permission denied' in capsys.readouterr().err
    assert not output.exists()
    assert not (tmp_path / 'home' / 'fr3-client-recordings').exists()


@pytest.mark.parametrize('name', ['smoke', 'chirp'])
def test_live_arguments_and_joint_forwarded_without_expanding_stage(tmp_path, monkeypatch, name):
    calls = []

    def capture(options):
        parser = runner.smoke.parser if name == 'smoke' else runner.excitation.parser
        calls.append(parser().parse_args(options))
        return 130

    module = runner.smoke if name == 'smoke' else runner.excitation
    monkeypatch.setattr(module, 'main', capture)
    assert runner.main(['--stage', name, '--joint', '7', '--execute',
                        '--host', '127.0.0.1', '--robot-id', 'right',
                        '--cmd-port', '12345', '--state-port', '12346',
                        '--output', str(tmp_path / name)]) == 130
    assert len(calls) == 1
    args = calls[0]
    assert args.execute and not args.dry_run
    assert args.host == '127.0.0.1' and args.robot_id == 'right'
    assert args.cmd_port == 12345 and args.state_port == 12346
    assert args.profile == 'sysid_stiff'
    assert (args.joint if name == 'smoke' else args.joints) == (7 if name == 'smoke' else [7])


def test_existing_output_refused_before_dispatch(tmp_path, monkeypatch):
    output = tmp_path / 'existing'
    output.mkdir()
    metadata = output / 'metadata.json'
    metadata.write_text('{"preserve": true}')
    before = metadata.read_bytes()

    def reject(*args, **kwargs):
        pytest.fail('existing output reached an entrypoint')

    monkeypatch.setattr(runner.smoke, 'main', reject)
    assert runner.main(['--output', str(output)]) == 1
    assert metadata.read_bytes() == before


@pytest.mark.parametrize('name', runner.STAGES)
def test_offline_cli_from_other_directory_has_no_transport_import(tmp_path, name):
    output = tmp_path / name
    code = '''
import runpy, sys
from pathlib import Path
class RejectTransport:
    def find_spec(self, fullname, *args):
        if fullname.startswith(('fr3_stack', 'zmq', 'capnp')):
            raise RuntimeError('offline preview imported hardware transport')
sys.meta_path.insert(0, RejectTransport())
sys.path.insert(0, str(Path(sys.argv[1]).parent))
sys.argv = [sys.argv[1], '--stage', sys.argv[2], '--output', sys.argv[3],
            '--host', 'must-not-connect.invalid']
runpy.run_path(sys.argv[0], run_name='__main__')
'''
    result = subprocess.run([sys.executable, '-c', code, str(SCRIPT), name, str(output)],
                            cwd=tmp_path, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    metadata = json.loads((output / 'metadata.json').read_text())
    assert metadata['status'] == 'preview'
    assert metadata['commands_sent'] == 0
    assert not metadata['motion_command_attempted']
    assert not metadata['sysid_ready']
    assert metadata['trajectory_campaign']['stage'] == name
    assert not metadata['trajectory_campaign']['automatic_progression']
    assert not metadata['trajectory_campaign']['human_review_verified']
    if name == 'smoke':
        assert metadata['smoke_test']['status'] == 'preview'
    assert (output / 'reference.csv').is_file()
    assert (output / 'preview.svg').is_file()
    before = {path.name: path.read_bytes() for path in output.iterdir()}
    assert runner.main(['--stage', name, '--output', str(output)]) == 1
    assert {path.name: path.read_bytes() for path in output.iterdir()} == before


@pytest.mark.parametrize('name', ['smoke', 'chirp', 'train', 'twin-all', 'twin-validation'])
def test_idle_daemon_is_rejected_without_commands(daemon_streaming, tmp_path, monkeypatch, name):
    monkeypatch.delenv('FR3_CONFIG_DIR', raising=False)
    output = tmp_path / name
    with daemon_streaming.publish_loop(controller='gravity_compensation',
                                       q=runner.excitation.PREVIEW_Q.tolist()):
        result = runner.main(['--stage', name, '--execute', '--host', '127.0.0.1',
                              '--robot-id', 'test', '--output', str(output),
                              '--cmd-port', str(daemon_streaming.cmd_port),
                              '--state-port', str(daemon_streaming.state_port)])
    assert result == 1
    assert not daemon_streaming.drain_commands()
    metadata = json.loads((output / 'metadata.json').read_text())
    assert metadata['status'] == 'aborted'
    assert metadata['commands_sent'] == 0
    assert not metadata['motion_command_attempted']
    assert metadata['trajectory_campaign']['stage'] == name
