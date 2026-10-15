# Installation & connection

Run the C++ daemon on the NUC connected to your FR3, and install the Python
client on your workstation. Use the NUC address when connecting from Python.

## NUC side

Ships as a Docker image (libfranka 0.17.0, pinocchio 3.x, Eigen 3.4, gcc 13).

!!! warning "Host kernel must be PREEMPT_RT"

### Build & run

```bash
./fr3-stack build              # one time (~5–10 min, cached after)
./fr3-stack up cart            # cartesian impedance, anchored at current pose
./fr3-stack up hybrid --ft     # hybrid force/position with Bota FT sensor
./fr3-stack up idle            # gravity-comp + damping (hand-guidable)
./fr3-stack up cart --attach   # foreground (default is detached)
./fr3-stack logs -f
./fr3-stack down
```

Modes are `idle`, `cart`, `joint`, `adm`, and `hybrid`. For a non-idle initial controller, the daemon anchors its target to the live robot state at activation.

If the selected local Docker socket is inaccessible, the launcher asks for sudo
authorization for Docker only, while retaining your `FR3_`/`COMPOSE_` settings
and selected socket. This does not select a remote NUC or satisfy the RT/network
requirements. See [Docker access troubleshooting](troubleshooting.md#docker-socket-permission-denied).

An installed `.setup/nuc-jz.env` automatically selects the prepared JZ image and
external data directories for service commands, without needing `--jz` or SDK
activation first. Exported `FR3_` settings override site defaults; `up <mode>`
still chooses the controller. JZ startup defaults to no build or pull.

### Daemon flags

| Flag                     | Default      | Notes                                            |
| ------------------------ | ------------ | ------------------------------------------------ |
| `--robot <ip>`           | *required*   | FR3 IP                                           |
| `--cmd-port`             | 5555         | workstation→NUC commands                         |
| `--state-port`           | 5556         | NUC→workstation state (~200 Hz)                  |
| `--initial-controller`   | `idle`       | `idle` \| `cartesian_impedance` \| `hybrid` |
| `--initial-cart-profile` | empty        | Cartesian profile YAML (SDK format) for the `up cart` boot hold; empty = built-in 200/20 |
| `--ft-sensor-kind`       | empty        | currently only `bota`                            |
| `--ft-sensor-config`     | empty        | Bota: path to driver JSON                        |

Shorthands: `--idle`, `--cart`, `--hybrid`.

Without `--initial-cart-profile`, a freshly started `cartesian_impedance`
daemon holds with the built-in K 200/20 until a client sends its own gains. Set
`FR3_INITIAL_CART_PROFILE` to a container path to boot with a profile instead.
In JZ mode the SDK profiles are mounted read-only at `/opt/fr3-stack/profiles`,
for example in `.setup/nuc-jz.env`:

```bash
FR3_INITIAL_CART_PROFILE=/opt/fr3-stack/profiles/cartesian_impedance.sysid_stiff.yaml
```

The daemon validates the file before connecting to the robot and refuses to
start on an invalid profile; the boot log prints the loaded K and D.

### Compose env vars

```bash
FR3_ROBOT_IP=192.168.1.11 \
FR3_INITIAL_CONTROLLER=cartesian_impedance \
FR3_FT_SENSOR_KIND=bota \
FR3_FT_SENSOR_CONFIG=/opt/bota/driver_config/bota_binary.json \
./fr3-stack up cart --ft
```

### F/T sensor (optional)

The Bota driver is bundled under `third_party/`. Select the FT backend on the NUC:

```bash
./fr3-stack up hybrid --ft
```

When publishing, `State.wrench_ft` is a length-6 vector; otherwise `None`. `send_hybrid` refuses to start without one unless `require_ft_sensor=False`.

---

## Workstation side

```bash
pip install -e .
```

### Connection config (`fr3.yaml`)

Read from one of:

1. `$FR3_CONFIG`                   (explicit override)
2. `./fr3.yaml`                    (per-project, gitignored)
3. `~/.config/fr3/config.yaml`     (user-global)

Template (`examples/fr3.example.yaml`):

```yaml
nuc_host:  192.168.1.8       # where fr3-stack listens
robot_ip:  192.168.1.11      # FR3's own IP
desk:
  user:     YOUR_DESK_USER
  password: YOUR_DESK_PASSWORD
```

Env vars override file values: `FR3_NUC_HOST`, `FR3_ROBOT_IP`, `FR3_DESK_USER`, `FR3_DESK_PASS`.

### Two API layers

| Class   | Module               | Use when                                                                        |
| ------- | -------------------- | ------------------------------------------------------------------------------- |
| `Robot` | `fr3_stack.robot`    | Streaming wire — one `send_*` per command. Direct access to every wire field.   |
| `Arm`   | `fr3_stack.client`   | Pose-centric facade for inference / teleop loops. `arm.robot` is the escape hatch. |

### Hello world (`Robot`)

```python
from fr3_stack import Robot
import numpy as np

with Robot("nuc.local") as robot:
    s = robot.wait_for_state()
    robot.send_cartesian_impedance(
        target_pos       = s.pos + np.array([0, 0, 0.05]),
        target_quat_xyzw = s.quat_xyzw,
        K                = [200, 200, 800, 30, 30, 30],
    )
    print(robot.state.pos, robot.state.wrench_ext)
```

Per-call kwargs (`K=`, `D=`, `target_wrench=`, …) update an internal last-sent cache, so subsequent calls without those kwargs reuse the new values.

### Repeatable small-motion check

`examples/joint_smoke.py` provides an offline preview, a read-only readiness
check, and an opt-in joint motion test. Run from the repository root:

```bash
# Preview only; no connection.
python3 examples/joint_smoke.py

# Fresh stationary joint-mode feedback; no commands sent.
python3 examples/joint_smoke.py --check-only --host NUC_HOST

# Small motion from the LIVE joint pose, then return and hold.
python3 examples/joint_smoke.py --execute --host NUC_HOST --robot-id left
```

Replace `NUC_HOST` with the NUC's actual IP or resolvable hostname, not its SSH
login username. A prepared image must be activated separately before state can
arrive; the smoke client does not start the daemon.

The motion preset is J1, at most +/-0.25 degrees, 0.1 Hz, 20 seconds including
5-second fades, with 3-second holds before and after (26 seconds total).
It uses `sysid_stiff` K/D and the profile's unchanged target filter.
`--joint` selects one joint; `--amplitude-deg` is restricted to 0.1..0.5.
The daemon must already be stationary in joint impedance mode, with other
command producers stopped and the arm/hand motion volume clear. It does not
home the robot or switch controller modes.

Each preview/motion run gets a new `recordings/smoke-.../` directory. A motion
`SMOKE PASS` requires observed excursion of at least half the selected amplitude,
tracking error no greater than half that amplitude against the previous sent
target, final joint error <=0.002 rad, and final speed <=0.005 rad/s.
Stale state, motion limits, or timing faults abort through the existing
[joint excitation lifecycle](joint-sysid.md); termination is best-effort and
unconfirmed. The half-amplitude tracking bound is also enforced during motion.
A stationary feedback stream cannot pass the motion test just
because sends returned successfully. The read-only check verifies state
readiness only, not the command path. These checks have no collision model
and have not been validated on the physical arm.

### Pose-centric API (`Arm`)

Stiffness is sticky — set once, then stream `Pose` targets:

```python
from fr3_stack import Arm, Pose

with Arm("nuc.local") as arm:
    arm.set_stiffness(K=[200, 200, 800, 30, 30, 30], damp_ratio=0.9)
    arm.move_to(Pose([0.5, 0.0, 0.4], [0, 0, 0, 1]), duration=2.0)

    obs    = arm.observe()
    target = Pose(obs.pose.pos + [0, 0, 0.05], obs.pose.quat)
    arm.send(target)

    arm.hold()        # lock at current pose with cached gains
    # arm.relax()     # gravity-comp + damping (hand-guidable; sends idle)
```

`arm.move_to(target, duration=T)` blocks for `T` seconds while the daemon's min-jerk generator drives the move; afterwards the controller holds at the goal. `arm.send(...)` is fire-and-forget streaming. Hybrid commands drop through to `arm.robot.send_hybrid(...)`.

### Per-controller defaults

K/D/M defaults live in `fr3_stack/configs/<name>.yaml`, loaded when `Robot` is constructed. Profile variants (`<name>.<profile>.yaml`) let you keep multiple presets:

```python
robot = Robot("nuc.local", profiles={"cartesian_impedance": "stiff"})
robot.set_profile("hybrid", "polish")        # swap at runtime
```

### F/T calibration

If you have a sensor mounted past the wrist, run payload identification before using hybrid:

```bash
./fr3-stack up hybrid --ft

# manual: hand-guide to ≥6 static poses with varied EE tilt
fr3-ft-calibrate 192.168.1.8

# or record once, replay automatically (CLEAR the workspace)
fr3-ft-calibrate-record 192.168.1.8
fr3-ft-calibrate-auto   192.168.1.8
```

The solver writes `fr3_stack/sensors/bota/config/ft_calibration.yaml`, which docker-compose bind-mounts into the daemon. The daemon reads it **once at startup** — restart to pick up a new file:

```bash
./fr3-stack down
./fr3-stack up hybrid --ft
```

After that, `state.wrench_ft` is compensated, `state.wrench_ft_raw` carries the raw stream, and `state.ft_compensated == True`.

### Termination

```python
robot.terminate()    # sends idle with termination=True; daemon stops RT loop
```
