# Troubleshooting

Start with the failing layer: workstation connection, NUC daemon, controller,
or optional F/T sensor. Keep the reported error and active configuration together.

## Docker socket permission denied

An error naming `unix:///var/run/docker.sock` refers to Docker on the machine
running the launcher, not the robot's IP address. `fr3-stack` checks the selected
Docker endpoint and, for an inaccessible local Unix socket, requests sudo
authorization before running any Compose operation. Enter your own sudo password
in that terminal. The fallback preserves exported `FR3_`/`COMPOSE_` settings,
the user's Docker client configuration, and the selected local socket; it does
not change group membership, socket permissions, or the execution host.

`./fr3-stack config` and `version` do not need daemon access and remain
unprivileged. Remote SSH/TCP Docker endpoints never receive the sudo fallback;
fix their credentials or permissions on the selected remote host instead.
If sudo authorization fails, no container is stopped or started.

This fixes Docker access only. The control host still needs PREEMPT_RT, robot
network connectivity, compatible firmware, and a reviewed safe activation.
To control through a NUC, run the launcher on that NUC (including through SSH),
then use the NUC address in the workstation's Python client.

## Daemon rejects a startup argument

An `unknown arg: --recording-port` startup error means the selected container
binary is older than the launcher/Compose configuration. Do not remove the flag
to hide the mismatch: recording and protocol support need a compatible daemon.
Inspect the stopped container's actual image ID and the prepared release receipt.

When `.setup/nuc-jz.env` is installed, normal service commands automatically
load its defaults and use the pinned JZ image and external calibration/recording
directories. You can check the selection without starting a controller:

```bash
./fr3-stack config --format json
```

Exported `FR3_` values still override site defaults, and the requested `up` mode
takes precedence over the default controller. JZ startup defaults to
`--no-build --pull never`; a missing image should be prepared through the matched
release workflow, not silently rebuilt on startup. Without installed site
configuration, the launcher retains its original base Compose behavior.
See the [synchronization guide](https://github.com/kingchou007/fr3_stack_jz/blob/main/sync.md).

## No state arrives

1. Use the **NUC address** in `Robot(host)`, not the robot's FCI address.
2. Confirm the daemon is running on that NUC with `./fr3-stack ps` and inspect
   `./fr3-stack logs`.
3. Check that workstation-to-NUC traffic reaches command port 5555 and state
   port 5556, or your configured replacements.
4. Inspect daemon logs for robot connection or startup errors.

`--host` takes a network IP or resolvable hostname, not the NUC's SSH login
username or an alias defined only in SSH configuration. For example, for
`ssh USER@NUC_IP`, pass `NUC_IP` to the client. A successful SSH connection or
a prepared Docker image does not mean the daemon has been started.
`examples/joint_smoke.py --check-only` rejects unresolvable hostnames and reports
the state endpoint on timeout; it never starts a daemon or sends motion commands.

Two different NUCs can use the same port numbers. Two daemons on the same host
need different ports. For two arms, verify each connection independently first.

## A parameter change has no effect

Controller configuration searches `FR3_CONFIG_DIR`, then local `configs/`, then
packaged `fr3_stack/configs/`. Inspect the selected profile with
`robot.get_profile(controller_name)`. Named profiles replace the configuration;
they do not merge with the base YAML. Explicit `send_*` values also persist in
controller caches, so changing a YAML file does not update an existing client.

## F/T data is missing

`state.wrench_ft is None` means no complete F/T reading is available. Check the
sensor backend and configuration on the NUC. Hybrid commands normally require
this reading. A software gate bypass does not create a sensor measurement.
See [F/T setup](quickstart.md#ft-sensor-optional) and the calibration instructions.

## The dual-arm coordinator enters FAULT

Inspect `pair.fault_reason` and `pair.stop_errors`. Common causes are stale
state, excessive reception-time skew, a daemon error, or failed command delivery.
Both stop callbacks are attempted, but network loss can prevent delivery.
Resolve the cause before explicitly calling `arm()` again. See
[dual-arm behavior and limits](dual-arm.md#behavior).

## Build dependencies are missing

Use `containers/Dockerfile` as the dependency/version reference for the NUC build. Native
builds must expose installed dependency prefixes through `CMAKE_PREFIX_PATH`.
The current CMake setup requires libfranka even when building controller mocks.
The [agent guide](https://github.com/Robot-Dexterity-Lab/fr3_stack/blob/main/AGENTS.md)
contains build and test entry points.

## A test fails to bind a local socket

Python wire tests use localhost FakeDaemons, not physical robots. A restricted
sandbox can prevent socket binding. Run them in an environment that permits
loopback networking before attributing the failure to control behavior.

## Report a reproducible problem

[Open a GitHub issue](https://github.com/Robot-Dexterity-Lab/fr3_stack/issues/new)
with the command or minimal script, commit/version, active controller/profile,
expected behavior, actual behavior, and relevant daemon error. State whether
it occurred with a FakeDaemon, one physical arm, or two physical arms. Remove
credentials from logs and configuration before sharing.
