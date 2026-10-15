"""Launcher access tests using fake Docker/sudo; never operate real containers."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
FAKE_COMMAND = r'''
import json
import os
from pathlib import Path
import sys

tool = Path(sys.argv[0]).name
args = sys.argv[1:]
settings = {
    key: value for key, value in os.environ.items()
    if key.startswith(("FR3_", "COMPOSE_", "DOCKER_")) or key in ("UID", "GID")
}
with open(os.environ["FAKE_LAUNCH_EVENTS"], "a") as stream:
    stream.write(json.dumps({"tool": tool, "args": args, "settings": settings}) + "\n")

if tool == "sudo":
    if args == ["-v"]:
        sys.exit(int(os.environ.get("FAKE_LAUNCH_SUDO_STATUS", "0")))
    assert args[0] == "env"
    clean_env = {
        key: value for key, value in os.environ.items()
        if key in ("PATH", "HOME") or key.startswith("FAKE_LAUNCH_")
    }
    os.execve("/usr/bin/env", ["env", *args[1:]], clean_env)

if args[:2] == ["context", "inspect"]:
    if os.environ.get("FAKE_LAUNCH_CONTEXT_ERROR"):
        sys.exit(1)
    print(os.environ["FAKE_LAUNCH_ENDPOINT"])
    sys.exit(0)

assert "compose" in args
if "ps" in args and "-q" in args:
    if os.environ.get("FAKE_LAUNCH_EXISTING_CONTAINER"):
        print("fake-container")
if "up" in args:
    sys.exit(int(os.environ.get("FAKE_LAUNCH_UP_STATUS", "0")))
'''


@pytest.fixture
def launcher(tmp_path):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name in ("docker", "sudo"):
        executable = bin_dir / name
        executable.write_text(f"#!{sys.executable}\n{FAKE_COMMAND}")
        executable.chmod(0o755)

    repo = tmp_path / "repo"
    repo.mkdir()
    shutil.copy2(ROOT / "fr3-stack", repo / "fr3-stack")
    (repo / ".setup").mkdir()
    socket_path = tmp_path / "docker.sock"
    sock = socket.socket(socket.AF_UNIX)
    sock.bind(str(socket_path))
    socket_path.chmod(0o600)
    event_file = tmp_path / "events.jsonl"
    env = {
        key: value for key, value in os.environ.items()
        if not key.startswith(("FR3_", "COMPOSE_", "DOCKER_", "FAKE_LAUNCH_"))
    }
    env.update(
        PATH=str(bin_dir) + os.pathsep + env["PATH"],
        FAKE_LAUNCH_EVENTS=str(event_file),
        FAKE_LAUNCH_ENDPOINT=f"unix://{socket_path}",
    )

    def run(*args, restricted=False, **overrides):
        if restricted:
            if os.geteuid() == 0:
                pytest.skip("root bypasses Unix socket permission checks")
            socket_path.chmod(0)
        else:
            socket_path.chmod(0o600)
        event_file.write_text("")
        result = subprocess.run(
            ["/bin/bash", str(repo / "fr3-stack"), *args],
            env={**env, **overrides}, capture_output=True, text=True, timeout=10,
        )
        events = [json.loads(line) for line in event_file.read_text().splitlines()]
        return result, events

    run.site_env = repo / ".setup/nuc-jz.env"
    yield run, socket_path
    sock.close()


def compose_events(events):
    return [event for event in events if event["tool"] == "docker" and "compose" in event["args"]]


def test_original_cart_command_uses_sudo_and_preserves_settings(launcher):
    run, socket_path = launcher
    result, events = run(
        "up", "cart", restricted=True,
        FR3_ROBOT_IP="10.0.0.27", FR3_CMD_PORT="6501", FR3_STATE_PORT="6502",
        FR3_RECORDING_PORT="6503", FR3_RECORDINGS_DIR="/tmp/records with spaces",
        FR3_FT_CONTROLLERS_RAW="1", COMPOSE_PROJECT_NAME="test-project",
    )
    assert result.returncode == 0, result.stderr
    assert "using sudo for Docker" in result.stderr
    sudo_events = [event for event in events if event["tool"] == "sudo"]
    assert sudo_events[0]["args"] == ["-v"]
    docker_events = compose_events(events)
    assert len(docker_events) == 3  # Existing container check, up, log tail.
    for event in docker_events:
        args = event["args"]
        assert args[args.index("--host") + 1] == f"unix://{socket_path}"
        settings = event["settings"]
        assert settings["FR3_INITIAL_CONTROLLER"] == "cartesian_impedance"
        assert settings["FR3_ROBOT_IP"] == "10.0.0.27"
        assert settings["FR3_CMD_PORT"] == "6501"
        assert settings["FR3_STATE_PORT"] == "6502"
        assert settings["FR3_RECORDING_PORT"] == "6503"
        assert settings["FR3_RECORDINGS_DIR"] == "/tmp/records with spaces"
        assert settings["FR3_FT_CONTROLLERS_RAW"] == "1"
        assert settings["COMPOSE_PROJECT_NAME"] == "test-project"
    assert docker_events[1]["args"][-2:] == ["up", "-d"]
    assert docker_events[2]["args"][-2:] == ["logs", "-f"]


def test_jz_attach_keeps_config_paths_and_clears_ft(launcher):
    run, socket_path = launcher
    result, events = run(
        "--jz", "up", "joint", "--no-ft", "--attach", "--no-build",
        "--pull", "never", "fr3-stack", restricted=True,
        FR3_JZ_IMAGE="test-image:pinned", FR3_JZ_CALIB_DIR="/tmp/calib ; data",
        FR3_JZ_RECORDINGS_DIR="/tmp/jz records", DOCKER_CONFIG="/tmp/client config",
        FR3_FT_SENSOR_KIND="bota", FR3_FT_SENSOR_CONFIG="/tmp/old sensor.json",
        UID="1001", GID="1001", DOCKER_HOST="ssh://unused-host",
        DOCKER_CONTEXT="selected-local-context",
    )
    assert result.returncode == 0, result.stderr
    inspect = events[0]
    assert inspect["args"][:3] == ["context", "inspect", "selected-local-context"]
    event = compose_events(events)[-1]
    args = event["args"]
    assert args[args.index("--host") + 1] == f"unix://{socket_path}"
    assert args[args.index("--config") + 1] == "/tmp/client config"
    assert args[args.index("--project-name") + 1] == "fr3_stack_jz"
    assert "containers/compose.jz.yml" in args
    assert args[-4:] == ["--no-build", "--pull", "never", "fr3-stack"]
    assert args[args.index("up") + 1:args.index("up") + 4] == ["--no-build", "--pull", "never"]
    settings = event["settings"]
    assert settings["FR3_INITIAL_CONTROLLER"] == "joint_impedance"
    assert settings["FR3_JZ_IMAGE"] == "test-image:pinned"
    assert settings["FR3_JZ_CALIB_DIR"] == "/tmp/calib ; data"
    assert settings["FR3_JZ_RECORDINGS_DIR"] == "/tmp/jz records"
    assert settings["FR3_FT_SENSOR_KIND"] == settings["FR3_FT_SENSOR_CONFIG"] == ""
    assert settings["UID"] == settings["GID"] == "1001"
    assert "DOCKER_CONTEXT" not in settings
    assert "DOCKER_HOST" not in settings


@pytest.mark.parametrize("args", [
    ["down"], ["logs", "-f"], ["ps"], ["build", "--no-cache"],
    ["ft", "up", "--attach"], ["ft", "down"], ["ft", "logs"],
    ["ft", "stream"], ["ft", "ps"],
])
def test_all_daemon_commands_use_the_same_fallback(launcher, args):
    run, _ = launcher
    result, events = run(*args, restricted=True)
    assert result.returncode == 0, result.stderr
    assert [event for event in events if event["tool"] == "sudo"][0]["args"] == ["-v"]
    assert compose_events(events)
    assert all("--host" in event["args"] for event in compose_events(events))


def test_accessible_socket_does_not_use_sudo(launcher):
    run, _ = launcher
    result, events = run("up", "joint", "--attach")
    assert result.returncode == 0, result.stderr
    assert all(event["tool"] != "sudo" for event in events)
    event = compose_events(events)[-1]
    assert event["settings"]["FR3_INITIAL_CONTROLLER"] == "joint_impedance"
    assert event["args"][0] == "compose"


@pytest.mark.parametrize("endpoint", ["ssh://user@remote-host", "tcp://remote-host:2376"])
def test_remote_context_is_never_elevated(launcher, endpoint):
    run, _ = launcher
    result, events = run("ps", restricted=True, FAKE_LAUNCH_ENDPOINT=endpoint)
    assert result.returncode == 0, result.stderr
    assert all(event["tool"] != "sudo" for event in events)
    assert compose_events(events)[0]["args"][0] == "compose"


def test_docker_host_is_used_when_no_context_is_explicit(launcher):
    run, _ = launcher
    result, events = run("ps", restricted=True, DOCKER_HOST="ssh://user@remote-host")
    assert result.returncode == 0, result.stderr
    assert len(events) == 1
    assert events[0]["args"][0] == "compose"
    assert events[0]["settings"]["DOCKER_HOST"] == "ssh://user@remote-host"


def test_explicit_context_takes_precedence_over_docker_host(launcher):
    run, socket_path = launcher
    result, events = run(
        "ps", restricted=True, DOCKER_HOST=f"unix://{socket_path}",
        DOCKER_CONTEXT="remote-context", FAKE_LAUNCH_ENDPOINT="ssh://user@remote-host",
    )
    assert result.returncode == 0, result.stderr
    assert events[0]["args"][2] == "remote-context"
    assert all(event["tool"] != "sudo" for event in events)


def test_explicit_local_host_is_pinned_when_elevating(launcher):
    run, socket_path = launcher
    result, events = run("ps", restricted=True, DOCKER_HOST=f"unix://{socket_path}")
    assert result.returncode == 0, result.stderr
    assert all(event["args"][:2] != ["context", "inspect"] for event in events)
    args = compose_events(events)[0]["args"]
    assert args[args.index("--host") + 1] == f"unix://{socket_path}"


def test_failed_authorization_never_replaces_or_starts_a_container(launcher):
    run, _ = launcher
    result, events = run(
        "up", "cart", restricted=True,
        FAKE_LAUNCH_SUDO_STATUS="1", FAKE_LAUNCH_EXISTING_CONTAINER="1",
    )
    assert result.returncode != 0
    assert "no Docker operation was started" in result.stderr
    assert not compose_events(events)


@pytest.mark.parametrize("args", [
    ["help"], ["up"], ["up", "invalid-mode"], ["ft", "invalid-command"],
])
def test_help_and_invalid_commands_do_not_request_docker_access(launcher, args):
    run, _ = launcher
    result, events = run(*args, restricted=True)
    assert result.returncode == (0 if args == ["help"] else 1)
    assert not events


@pytest.mark.parametrize("cmd", ["config", "version"])
def test_daemon_free_commands_remain_unprivileged(launcher, cmd):
    run, _ = launcher
    result, events = run(cmd, restricted=True)
    assert result.returncode == 0, result.stderr
    assert len(events) == 1
    assert events[0]["args"][-1] == cmd
    assert events[0]["args"][0] == "compose"


def test_missing_socket_does_not_use_sudo(launcher):
    run, _ = launcher
    result, events = run("ps", FAKE_LAUNCH_ENDPOINT="unix:///nonexistent/test-docker.sock")
    assert result.returncode == 0, result.stderr
    assert all(event["tool"] != "sudo" for event in events)


def test_context_inspection_failure_does_not_fall_back_to_root(launcher):
    run, _ = launcher
    result, events = run("ps", FAKE_LAUNCH_CONTEXT_ERROR="1")
    assert result.returncode != 0
    assert "cannot inspect" in result.stderr
    assert not compose_events(events)
    assert all(event["tool"] != "sudo" for event in events)


def test_failed_start_is_not_replayed(launcher):
    run, _ = launcher
    result, events = run("up", "cart", restricted=True, FAKE_LAUNCH_UP_STATUS="23")
    assert result.returncode == 23
    docker_events = compose_events(events)
    assert sum("up" in event["args"] for event in docker_events) == 1
    assert not any("logs" in event["args"] for event in docker_events)


SITE_ENV = """FR3_JZ_IMAGE=test-jz:prepared
FR3_JZ_CALIB_DIR='/tmp/external calibration'
FR3_JZ_RECORDINGS_DIR='/tmp/external recordings'
FR3_ROBOT_IP=10.0.0.11
FR3_CMD_PORT=5555
FR3_STATE_PORT=5556
FR3_RECORDING_PORT=5557
FR3_INITIAL_CONTROLLER=idle
FR3_FT_SENSOR_KIND=bota
FR3_FT_SENSOR_CONFIG='/tmp/site sensor.json'
"""


@pytest.mark.parametrize("mode", ["idle", "cart", "joint"])
def test_bare_start_uses_installed_site_image_and_defaults(launcher, mode):
    run, _ = launcher
    run.site_env.write_text(SITE_ENV)
    result, events = run("up", mode, "--attach")
    assert result.returncode == 0, result.stderr
    assert "image      = test-jz:prepared" in result.stderr
    event = compose_events(events)[-1]
    args = event["args"]
    assert args[args.index("--project-name") + 1] == "fr3_stack_jz"
    assert "containers/compose.jz.yml" in args
    assert args[-4:] == ["up", "--no-build", "--pull", "never"]
    settings = event["settings"]
    assert settings["FR3_JZ_IMAGE"] == "test-jz:prepared"
    assert settings["FR3_JZ_CALIB_DIR"] == "/tmp/external calibration"
    assert settings["FR3_JZ_RECORDINGS_DIR"] == "/tmp/external recordings"
    assert settings["FR3_ROBOT_IP"] == "10.0.0.11"
    assert settings["FR3_RECORDING_PORT"] == "5557"
    assert settings["FR3_INITIAL_CONTROLLER"] == {
        "idle": "idle", "cart": "cartesian_impedance", "joint": "joint_impedance",
    }[mode]


def test_exported_overrides_and_flags_take_precedence_over_site_defaults(launcher):
    run, _ = launcher
    run.site_env.write_text(SITE_ENV)
    result, events = run(
        "up", "joint", "--no-ft", "--attach", restricted=True,
        FR3_ROBOT_IP="10.0.0.22", FR3_CMD_PORT="6501", FR3_STATE_PORT="6502",
        FR3_RECORDING_PORT="6503", FR3_JZ_RECORDINGS_DIR="/tmp/custom records",
        FR3_FT_CONTROLLERS_RAW="",
    )
    assert result.returncode == 0, result.stderr
    settings = compose_events(events)[-1]["settings"]
    assert settings["FR3_ROBOT_IP"] == "10.0.0.22"
    assert settings["FR3_CMD_PORT"] == "6501"
    assert settings["FR3_STATE_PORT"] == "6502"
    assert settings["FR3_RECORDING_PORT"] == "6503"
    assert settings["FR3_JZ_RECORDINGS_DIR"] == "/tmp/custom records"
    assert settings["FR3_JZ_CALIB_DIR"] == "/tmp/external calibration"
    assert settings["FR3_FT_SENSOR_KIND"] == settings["FR3_FT_SENSOR_CONFIG"] == ""
    assert settings["FR3_FT_CONTROLLERS_RAW"] == ""


@pytest.mark.parametrize("args", [
    ["down"], ["logs"], ["ps"], ["ft", "ps"], ["config"], ["version"],
])
def test_installed_site_applies_to_service_lifecycle_and_read_only_config(launcher, args):
    run, _ = launcher
    run.site_env.write_text(SITE_ENV)
    result, events = run(*args)
    assert result.returncode == 0, result.stderr
    event = compose_events(events)[-1]
    assert "containers/compose.jz.yml" in event["args"]
    assert event["settings"]["FR3_JZ_IMAGE"] == "test-jz:prepared"


def test_bare_build_does_not_rebuild_the_installed_pinned_image(launcher):
    run, _ = launcher
    run.site_env.write_text(SITE_ENV)
    result, events = run("build")
    assert result.returncode == 0, result.stderr
    event = compose_events(events)[-1]
    assert "containers/compose.jz.yml" not in event["args"]
    assert "FR3_JZ_IMAGE" not in event["settings"]
