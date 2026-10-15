# Joint excitation and recording

For the selected Cartesian route using FrankaTwin's original reference
generators, see [Cartesian FrankaTwin collection](#cartesian-frankatwin-collection)
below. Joint excitation remains available; it is not the same reference path.

`examples/joint_sysid.py` generates joint-position references and can send them
to **one existing fr3-stack daemon in joint impedance mode**. It supports a
hold, sequential single-joint sine/chirp segments, and simultaneous multisine
excitation. It does not control XHand. Keep the hand in a fixed, recorded pose.

This is a trajectory/collection client, not a complete FrankaTwin identification
pipeline. The simulation repository stays independent and consumes files.

For a repeatable connection and small-motion check before data collection,
use `examples/joint_smoke.py`; see the
[quickstart preset](quickstart.md#repeatable-small-motion-check). It uses the
same trajectory, feedback guards, and fault lifecycle with a smaller excursion.

## Adopted identification route and existing interface

The goal is joint-space **simulation parameter identification**: replay the
same applied joint references through a matched simulation controller, compare
simulated and recorded joint motion, and fit supported dynamics parameters.
Recorded q is the comparison signal; it must not overwrite simulator state
each step. Inverse-dynamics regression is not the primary route in this plan.

`Robot.send_joint_impedance(q_target)` already provides continuous joint
targets in this stack. The missing real-side pieces are fit-ready 1 kHz
recording and export, not a new joint command interface. The reviewed
FrankaTwin revision `3c7b43ee95cc4631915191928dd2648dba8235c6` instead uses
Cartesian task impedance for SI; its `move_to_q()` is a blocking reset motion.
We reuse its replay/search method, not its Cartesian command path.

The independent `simtoolreal-fr3-xhand` repository owns the adopted design in
`docs/joint_space_sysid.md` and implementation checklist in
`docs/frankatwin_sim_todo.md`. Its validator, matched calibration controller,
optimizer, and profile loader remain planned. It must consume portable files
without this stack's imports, source tree, daemon, or network connection.

The current real control law is target EMA followed by
`K*(filtered_target-q)-D*dq+Coriolis+joint_limit_repulsion`, with optional
friction compensation and final stack torque slew limiting. Filter state
resets to measured q. It does not use desired-velocity feedforward. Match the
actual run settings, torque/gravity convention, and downstream processing in
simulation before allowing an optimizer to change physical parameters.

## Preview first

Run commands from the repository root. `uv run` installs the project's declared
Python dependencies into its environment; an existing installation can instead
use `python examples/joint_sysid.py ...`.

```bash
uv run python examples/joint_sysid.py --dry-run --output /tmp/joint-preview
```

The default is a **J1 sine, 1 degree peak offset, 0.1 Hz, 20 seconds**, including
5-second fade-in and fade-out, with 2 seconds of holding before and after.
These are initial exploration settings, not validated parameters for your robot.
The default command rate is 60 Hz. Preview mode never connects to a robot,
including when `--host` is supplied. It only needs NumPy when run directly.

Open `preview.svg` to inspect offset, velocity, acceleration, and jerk.
`reference.csv` contains absolute joint targets and continuous-reference
derivatives in SI units. The default preview anchor is **illustrative**; use
`--q0-rad` followed by seven measured joint angles to preview your actual pose.
Execution never moves to this illustrative pose or accepts a preview anchor.

Each output directory must be new; omit `--output` for a timestamped directory
under `recordings/`. Keep generated recordings out of commits.

## Initial small-range campaign

`configs/joint_sysid.initial.json` specifies a staged pilot using the existing
trajectory generator. `examples/joint_trajectory.py` previews or executes exactly
one selected stage; it never automatically progresses through the campaign.
All amplitudes are peak offsets about each run's live anchor, not peak-to-peak
angles. Keep `sysid_stiff` and its EMA unchanged throughout the campaign.

| Stage | Reference | Peak offset | Frequency | Total time |
| --- | --- | --- | --- | --- |
| Hold | Live anchor only | 0 | None | 20 s |
| Smoke | J1 sine | 0.25 deg | 0.1 Hz | 26 s |
| Pilot | J1 chirp | 0.4 deg | 0.1 to 0.4 Hz | 46 s |
| Sequential | J1 through J7, one moving joint at a time | 0.4 deg | 0.1 to 0.4 Hz | 304 s |
| Training | Simultaneous multisine, seed 0 | At most 0.5 deg per joint | 0.1, 0.23, 0.37 Hz | 66 s |
| Validation | Simultaneous multisine, seed 101 | At most 0.5 deg per joint | 0.13, 0.29, 0.43 Hz | 66 s |

The common initial/final holds are 3 s; sequential segments have 3 s gaps.
The chirp includes 6 s fades in each 40 s segment. Multisine includes 8 s fades
in its 60 s excitation. Seventh-order envelopes bring offset and the first
three reference derivatives to zero at every boundary. These reference choices
follow the smooth-start/end principle in the
[official FCI requirements](https://frankarobotics.github.io/docs/robot_specifications.html),
but this stack uses torque control: reference checks do not certify actual
robot torque, motion derivatives, collision clearance, or controller stability.

Except for the stricter smoke preset, experiment bounds are 0.02 rad/s,
0.08 rad/s^2, and 0.6 rad/s^3. The designed peaks leave at least 20% reference
derivative headroom against these bounds in a 1 kHz offline check. Runtime
guards use state age 0.1 s, previous-target tracking error 0.003 rad, and
loop/send pause 0.05 s. Use the smoke wrapper for its additional observed-motion
and final-settling checks.

Use the one-stage script from the workstation. With no flags it previews the
J1 smoke preset without importing transport or connecting to the NUC:

```bash
python3 examples/joint_trajectory.py
python3 examples/joint_trajectory.py --stage chirp
```

The script runs on the machine where these files are installed; SSH changes
that machine, it does not transfer the workstation's checkout. If the NUC
reports a missing `examples/joint_trajectory.py`, synchronize the entrypoint,
`joint_smoke.py`, `joint_sysid.py` and `configs/joint_sysid.initial.json` using
the guarded example-only procedure in the repository-root `sync.md`. Prefer workstation
execution so the NUC remains dedicated to control.

For diagnostics from an already synchronized NUC checkout, use its existing SDK
environment, not an unconfigured system Python. With the standard `.venv`:

```bash
.venv/bin/python examples/joint_trajectory.py --help
.venv/bin/python examples/joint_trajectory.py --stage hold --dry-run
```

These commands send no motion. A separately reviewed local live run uses the
same interpreter with `--execute --host 127.0.0.1 --robot-id ARM_ID`; it must
not run concurrently with a workstation command producer. Review the profile
on the execution machine: two files named `sysid_stiff` may contain different
K/D values. Example synchronization preserves existing site profiles and never
changes an installed SDK, gain file, controller mode or running daemon image.

`--stage` accepts `hold`, `smoke` (default), `chirp`, `sequential`, `train`,
and `validation`. `--joint 1..7` is supported only for smoke/chirp; the other
stages keep their designed joint selection. Omit `--output` for a new timestamped
directory under `~/fr3-client-recordings/`, independent of the current working
directory. This keeps client diagnostics separate from Docker's recording
mount, which may be root-owned. An explicit `--output` must be a new directory
under a writable parent; it is never silently redirected. Do not run the client
with sudo or change daemon recording permissions to work around a client log
error. Directory creation failure occurs before any robot connection or command.
The lower-level `joint_sysid.py` and `joint_smoke.py` defaults remain under
`recordings/`; use their `--output` option when that directory is not writable.
The one-stage script prints the selected log directory,
the stage's human review prerequisite and records campaign/entrypoint hashes
in `metadata.json`; it does not claim that prerequisite has been verified.

After independently arranging a stationary joint-mode daemon and reviewing the
stage, opt into one supervised run:

```bash
python3 examples/joint_trajectory.py --stage hold --execute \
  --host NUC_HOST --robot-id ARM_ID
python3 examples/joint_trajectory.py --stage smoke --execute \
  --host NUC_HOST --robot-id ARM_ID
```

Run and review each stage separately. The smoke stage delegates to
`joint_smoke.py`, retaining its observed-motion and final-settling checks.
Execution rejects `idle`/gravity-compensation feedback without sending commands;
the script neither starts nor switches a controller. Successful completion
leaves the daemon holding the live anchor. Faults during streaming retain the
existing best-effort termination behavior, not a guaranteed hardware stop.
No controller math or target EMA is changed; the campaign selects `sysid_stiff`.
It does not start 1 kHz recording, and client diagnostics are not fit-ready data.

For a pose-specific preview or extra experiment metadata/NUC-log references,
use the lower-level generator. For example, preview the J1 pilot:

```bash
python3 examples/joint_sysid.py --dry-run --profile sysid_stiff \
  --mode chirp --joints 1 --amplitude-deg 0.4 --f0 0.1 --f1 0.4 \
  --seconds 40 --ramp 6 --hold 3 --gap 3 --rate 60 \
  --max-velocity 0.02 --max-acceleration 0.08 --max-jerk 0.6 \
  --tracking-error 0.003 --stale-after 0.1 --max-lateness 0.05
```

Add seven measured values through `--q0-rad` for a pose-specific preview;
otherwise its anchor is illustrative. A saved preview anchor is never a homing
target. Before real motion, independently arrange a stationary `joint` daemon,
check the payload and full arm/hand swept space, and progress through hold,
smoke and J1 pilot before considering the later stages. Do not restart or switch
an active `idle` daemon merely to generate previews.

For a supervised pilot, replace `--dry-run` with
`--execute --host NUC_HOST --robot-id ARM_ID`, omit `--q0-rad`, and refer to a
separately started 1 kHz recording with `--nuc-log`. Each stage needs its own
review and recording ID; this JSON does not start recording or enforce its
human review gates. See [Recording](recording.md).

Small, low-frequency motion is an initial response/collection experiment,
not proof that inertia, friction, coupling or millisecond delay are identifiable.
Expand pose, amplitude or bandwidth only after examining actual tracking,
torque and 1 kHz data quality. Keep the validation run out of optimizer fitting.

## FrankaTwin-inspired joint trial

The reviewed source is [FrankaTwin acf16009](https://github.com/tsrobcvai/frankatwin/tree/acf16009c09c99c9a2bb86050b57eb248982ba9f/python/frankatwin/excitation).
These presets follow its low-band chirp / held-out multiband design, **not** its
Cartesian references, controller, gain values or collision configuration.
They retain this stack's C3 fades, site `sysid_stiff` and EMA. The larger trial
changes only its reference and explicit tracking window; other guards remain.
No upstream package or new runtime dependency is required.

| Stage | Selected joints | Peak bound | Frequencies | Total |
| --- | --- | --- | --- | --- |
| `twin-chirp` | J1, or `--joint 1..7` | 0.2 deg | 0.1 to 0.7 Hz | 46 s |
| `twin-all` | J1 through J7 simultaneously | 0.2 deg each | 0.1 to 0.7 Hz | 46 s |
| `twin-validation` | J1 through J7 simultaneously | 0.08 deg each | Three distinct tones per joint, 0.15 to 2 Hz | 66 s |
| `twin-large` | J1 through J7 simultaneously | 1 deg each | 0.05 to 0.15 Hz | 46 s |

The chirp uses fixed phase offsets `2*pi*(joint_index-1)/7`, independent of
selection. The held-out tone table follows source axis order x/y/z/rx/ry/yaw/roll
reassigned to J1/J2/J3/J4/J5/J6/J7; this is a frequency assignment, not an IK
conversion or physical Cartesian-axis mapping. Relative weights `min(1, 0.35/f)`
are normalized to sum to one, unlike upstream's unnormalized amplitude scales.
Phases follow its axis/band stagger. Actual frequencies, weights and phases are
recorded in `reference_waveform`; source revision and adaptations are saved in
campaign metadata. Validation is intentionally smaller to preserve the existing
derivative limits at higher frequencies. These are collection trials, not proof
of identifiability or stability. No 0.7 to 3 Hz high chirp is enabled.

`twin-large` is a larger/slower excitation trial after analysing a small-signal
abort, not a high-band SI replacement. It retains 0.02 rad/s, 0.08 rad/s^2 and
0.6 rad/s^3 limits, state-age and scheduling guards, but explicitly changes the
previous-target tracking window from 0.003 rad to 0.00872665 rad (0.5 deg).
That window is half the reference peak bound, not a disabled tracking check:
a joint that remains stationary through a 1 deg peak still aborts. The same
tracking window applies to terminal return-to-anchor checks; it does not prove
a 0.003 rad return accuracy. Confirm actual excursion and settling from the
1 kHz recording before claiming successful excitation. No K/D, EMA, torque-rate,
collision or firmware settings are changed. Review the larger swept volume
before execution. The entrypoint prints amplitude, tracking window and chirp
band explicitly for operator review.

Preview each stage without a robot connection:

```bash
python3 examples/joint_trajectory.py --stage twin-all --dry-run
python3 examples/joint_trajectory.py --stage twin-validation --dry-run
python3 examples/joint_trajectory.py --stage twin-large --dry-run
```

After a current-session hold/smoke check and inspection of the combined arm/hand
swept volume, run exactly one supervised stage with
`--execute --host NUC_HOST --robot-id ARM_ID`. If individual joints have not been
reviewed, use `twin-chirp --joint J` first. A completed all-joint low chirp is a
prerequisite for separately reviewing the higher-frequency validation stage;
there is no automatic progression. Start 1 kHz recording separately before
motion and stop it after the stage, including on failure. The wrapper now
forwards `--nuc-log` and `--experiment-metadata` to the core for non-smoke stages;
these references do not start or verify a recording. Normal completion leaves
joint hold active. Existing local stop procedures remain necessary.

The core also exposes `--chirp-layout simultaneous` and
`--tone-profile frankatwin`; sequential chirps and custom multisine are unchanged
defaults. The fixed tone profile rejects custom `--tones` or a nonzero `--seed`.
The presets use 50 Hz targets and require independent 1 kHz daemon logs.

## Cartesian FrankaTwin collection

`examples/cart_sysid.py` calls the unmodified, Apache-2.0 licensed FrankaTwin
`chirp.py` and `multiband.py` from revision
`acf16009c09c99c9a2bb86050b57eb248982ba9f`. Sources, SHA256 identities, LICENSE
and NOTICE are in `examples/_frankatwin_reference/`. It does not turn the
references into joint-space sine waves. A fixed endpoint is supplied to every
evaluation so upstream's grid-dependent duration does not change during live
single-sample streaming.

| Stage | Original excitation | Excitation time | With default holds |
| --- | --- | --- | --- |
| `hold` | Live pose only | 10 s | 14 s |
| `chirp` | Six Cartesian DOFs, 0.1 to 0.7 Hz, original phases and 2/3 s linear ramps | 8 s | 12 s |
| `validation` | Original held-out multiband tones and 2/2 s half-cosine ramps | 12 s | 16 s |

`--scale` multiplies all translation and rotation amplitudes, not frequencies,
phases or duration. Default 0.1 gives chirp position scales 1/1/1.5 cm and
rotation-vector component scales 0.05/0.025/0.05 rad. At `--scale 1` they are
the original 10/10/15 cm and 0.5/0.25/0.5 rad. Multiband amplitudes are upstream
tone scales, not per-axis peak bounds. Full scale is not hardware-approved for
an XHand or arbitrary starting pose. This client never homes the robot.

Preview, including original full-scale references, without transport imports:

```bash
python3 examples/cart_sysid.py --stage chirp --scale 1 --dry-run
python3 examples/cart_sysid.py --stage validation --scale 1 --dry-run
```

Existing directories are refused. Default output is a new directory under
`~/fr3-client-recordings/`. Optional `--anchor-pos` and `--anchor-quat-xyzw` are
preview-only; execution anchors at the measured live pose. `reference.csv`,
`preview.svg` and metadata preserve the exact reference/provenance. Original
linear ramps have velocity discontinuities; speed sampling is not a jerk,
joint-limit, collision, torque or stability certificate.

The daemon must already be healthy and stationary in `cartesian_impedance`.
An `idle` or joint daemon is rejected before recording or motion; the script
never changes mode. Controller activation is a separately supervised task.
Review Desk payload, fixed XHand pose, full swept volume and local stop first.
Only one command producer may run. After arranging Cartesian mode, review and
run the stages separately:

```bash
python3 examples/cart_sysid.py --stage hold --execute --record \
  --host NUC_HOST --robot-id ARM_ID
python3 examples/cart_sysid.py --stage chirp --scale 0.1 --execute --record \
  --host NUC_HOST --robot-id ARM_ID
# Optional high-dynamics fit run: FrankaTwin's high band, 0.7 -> 3 Hz with
# amplitudes tapered by (f0/f)^0.5. Scale <= 0.4 stays within the default
# 0.6 m/s / 3 rad/s reference-speed bounds.
python3 examples/cart_sysid.py --stage chirp-high --scale 0.2 --execute --record \
  --host NUC_HOST --robot-id ARM_ID
# Independently reviewed and reserved from fitting, never auto-started:
python3 examples/cart_sysid.py --stage validation --scale 0.1 --execute --record \
  --host NUC_HOST --robot-id ARM_ID
```

The pose-tracking guard is off by default. As in FrankaTwin
(`error_delta_pos 0`), impedance lag is expected data: the client records the
peak position/angle error against the previous sent target in
`metadata.json.observed_motion`. Pass `--max-position-error METERS` and/or
`--max-angle-error RAD` to abort when exceeded (0 keeps a check off); size them
well above the expected lag. Joint speed is never checked, as in FrankaTwin
(a scale-0.2 chirp already needs ~0.3 rad/s on J7), and there is no stillness
requirement at the start or end of a run.
Runaway protection is the daemon's per-joint torque clamp and slew limit, and the libfranka collision
reflex and joint limits; the swept volume and local stop remain the operator's.

At the default Cartesian gains (K 200/20) the arm lags the chirp reference by
roughly 300-500 ms. Add `--profile sysid_stiff` (`fr3_stack/configs/cartesian_impedance.sysid_stiff.yaml`)
for FrankaTwin's held-out translation gain, K 500 with D = 2·√K. Rotation uses
K 20 with damping lowered to 5 (20/9 still excited a transient ~31 Hz J7
oscillation at scale 0.4), and the nullspace stabilizer and target
interpolation/EMA stay on: with FrankaTwin's rotational 30/10.95, this FR3 + XHand oscillated at
25 Hz, dominated by J7 (~1 rad/s, ~10 Nm), once J7 moved faster than about
0.1 rad/s, with or without the nullspace term and smoothing. Collect fit and
held-out runs with the same profile and pass the same gains to the simulator
replay; stiffer gains also push harder on contact.

With `--record`, the client starts an acknowledged unique 1 kHz log at
`/recordings/<run-id>.csv` before the first target. It closes that same recording
after normal completion and records the stop counters; a failed/incomplete
recording returns failure even when the reference finished. Runtime motion
faults attempt termination immediately before recording cleanup, never delay
termination to wait for a recording RPC. Stop timeouts remain unconfirmed:
inspect writer completion and the preserved CSV. No automatic controller
restart occurs. Without `--record`, client files alone are not a 1 kHz dataset.

The selected existing Cartesian profile is resolved and sent explicitly,
including K/D, nullspace, clipping, friction, EMA and dispatcher interpolation.
There are no EMA or gain override flags. A nonzero configured feedforward
wrench is refused. Site YAML/daemon settings are not rewritten. The current
stack uses SO(3) log error, EMA/SLERP, optional LERP, nullspace and Coriolis;
these differ from FrankaTwin's simulated pure task-PD controller. Keeping the
original trajectory does not make the controllers or CSV formats identical.
Raw CSV v2 lacks applied Cartesian gains and filtered pose targets; metadata
records commanded settings as unacknowledged. A matched replay/controller and
portable export are still required before fitting. `sysid_ready` remains false.

Default sampled-feedback guards are state age 0.1 s and loop/send pause
0.05 s; position and orientation error checks are off unless set, and joint
speed is not checked.
The final offset from the anchor and whether enough motion was observed are
recorded in `metadata.json.final_offset`, not enforced: at K 500 joint friction
leaves a few-millimetre static offset after a valid run. These are experiment
settings, not robot limits. Normal completion leaves Cartesian hold
active; network termination is not a hardware emergency stop or NUC watchdog.

## First real joint run

Confirm the installed payload in Desk and inspect the full arm/hand motion
volume at the intended starting pose. The script checks joint ranges, but has
no robot/scene collision model. Complete the stack's ordinary hardware setup
and [connection procedure](quickstart.md). The NUC must already publish healthy,
stationary `joint_impedance` state (launch mode `joint`). Stop other command
producers; there is no exclusive ownership lease.

On the workstation, replace `NUC_HOST` with the NUC's hostname or address:

```bash
# Observe holding behavior and logging before introducing an oscillation.
uv run python examples/joint_sysid.py --execute --host NUC_HOST \
  --robot-id left --mode hold --seconds 20

# The default one-joint, low-frequency sine; anchor is the LIVE joint pose.
uv run python examples/joint_sysid.py --execute --host NUC_HOST \
  --robot-id left --mode sine --joints 1 --amplitude-deg 1 --f0 0.1
```

The client rejects missing/stale/invalid state, daemon errors, a different
controller, or initial joint speed above 0.02 rad/s. It checks the anchor again
after saving its preview and refuses to start if the arm has moved more than
0.002 rad while preparing. It does not reset or home the robot.

It explicitly sends K/D and target-filter settings resolved from the installed
`joint_impedance.yaml` on every command; `--profile`, `--kp`, `--kd`, and
`--filter-alpha` override them. Defaults are therefore **configuration
dependent**. Friction compensation is explicitly disabled for these experiments.
Review the selected gains before execution; no gain ramp is implemented. The
script neither reads nor changes Desk's payload or collision settings. A client
send is not an acknowledgement that the NUC applied those settings.

For stiffer joint-level response during small-amplitude identification, use the
bundled higher-gain joint profile:

```bash
uv run python examples/joint_sysid.py --execute --host NUC_HOST \
  --robot-id left --profile sysid_stiff --mode sine --joints 1 \
  --amplitude-deg 1 --f0 0.1
```

`joint_impedance.sysid_stiff.yaml` raises K/D while keeping the default target
EMA setting. If tracking is still too soft after a hold run and a low-frequency
sine, raise `--kp` / `--kd` in small, recorded steps and keep the 1 kHz recorder
active.

Hardware-free regression tests exercise `--profile sysid_stiff` through the CLI,
verify the transmitted K/D/filter settings and execution metadata with a fake
daemon, and check that unknown profiles or invalid gains fail before connection.
This does not establish stability or a faster response on a physical arm.

Normal completion returns the reference smoothly to the starting joint pose,
holds, and leaves the NUC holding that pose in joint impedance after Python
exits. `--finish terminate` instead requests daemon termination at the end.
Ctrl-C, SIGTERM, send failures, logging failures during streaming, stale
feedback, excessive measured speed/tracking error, or a missed deadline abort
the run and attempt termination **if a motion command was attempted**. A fault
does not command a return to the starting pose.

Termination goes over the same network and is recorded as unconfirmed; it is
not a hardware emergency stop. A disconnected/killed workstation cannot
guarantee a stop, and this client adds no NUC watchdog. Keep the existing local
stop procedure available. Preflight failures send no control commands.

## Chirp and multisine

These examples generate **offline previews**. Select amplitudes and frequency
bands from the actual payload, available space, and observed tracking/torque
response before adding `--execute --host NUC_HOST --robot-id left`.

```bash
# J1 through J7 in sequence, each sweeping 0.1 to 0.5 Hz for 40 seconds.
uv run python examples/joint_sysid.py --mode chirp --joints 1 2 3 4 5 6 7 \
  --seconds 40 --ramp 5 --amplitude-deg 1 --f0 0.1 --f1 0.5

# Simultaneous joint motion; seed fixes independent phases for reproducibility.
uv run python examples/joint_sysid.py --mode multisine --joints 1 2 3 4 5 6 7 \
  --seconds 40 --amplitude-deg 1 --tones 0.1 0.23 0.41 --seed 0

# Reserve this complete run for validation; do not use it to tune the fit.
uv run python examples/joint_sysid.py --mode multisine --joints 1 2 3 4 5 6 7 \
  --seconds 40 --amplitude-deg 1 --tones 0.13 0.29 0.47 --seed 101
```

For sine/chirp, `--seconds` is per selected joint, with `--gap` holding between
segments. For multisine it is the complete excitation duration. `--hold` adds
holding at both ends. Seventh-order envelopes make offset and its first three
derivatives zero at segment boundaries. Multisine tone amplitudes are weighted
by inverse frequency and normalized so their **sum**, not each tone, is bounded
by `--amplitude-deg`. Chirp uses constant amplitude within its envelope.

Position bounds conservatively include the full amplitude on every selected
joint and exclude the stack's outer 10% joint-limit repulsion regions.
Analytic reference derivatives, including the envelopes, are sampled at 1 kHz
for preflight checks. Default per-joint bounds are 0.1 rad/s, 0.3 rad/s^2, and
2 rad/s^3; they are configurable experiment limits, not certified hardware
limits. They do not predict torque, collisions, or the derivatives of the
sample-and-hold commands. The daemon still filters those commands at 1 kHz.

During streaming, the default state age bound is 0.2 s, tracking-error bound is
0.05 rad relative to the previous sent target, and loop/send pause bound is
0.05 s. Commands are evaluated at actual wall time. Expired scheduling slots
are skipped, and large pauses abort instead of causing a burst of old commands.
Terminal settling also requires measured speed at or below 0.02 rad/s.

## Recording contract and remaining 1 kHz work

| File | Contents |
| --- | --- |
| `preview.svg` | Continuous reference curves |
| `reference.csv` | Planned absolute targets and derivatives at the requested command rate |
| `commands_and_state.csv` | Successful client send attempts, send-start/return times, and latest measured q/dq sampled before each send |
| `metadata.json` | Schema, script hash, robot ID, anchor, waveform/seed, explicit commanded gains/filter, bounds, run outcome, and supplied experiment metadata |

`commands_and_state.csv` samples the approximately 200 Hz PUB stream at the
client's command rate. Repeated state samples are possible. Client monotonic
times and daemon timestamps are separate clocks; no clock synchronization is
claimed. Failed sends might still have reached the daemon and are recorded as
an aborted run, not assumed absent from the real motion. CSV output contains
no per-tick torque or effective filtered target.

On tracking-guard failure, the exception names the worst joint, error and limit.
`metadata.json` saves `guard_failure` with the failing measured snapshot, previous
target, seven errors, state age and trajectory time before another command is
sent. This does not relax the guard or confirm termination. A termination can
close the daemon's recording service before a subsequent recording `stop` RPC;
that timeout is an unconfirmed RPC result, not proof of lost data. Check service
exit, daemon writer completion and the on-disk CSV without restarting control
just to obtain an acknowledgement. Exclude rows with `joint_target_valid=0`
from target/error analysis; an end-of-control row can contain zero placeholders.

**These files are not a 1 kHz system-identification dataset.** Metadata always
sets `sysid_ready=false` and `nuc_log_verified=false`. `--nuc-log` only records
the path or identifier of a separately collected NUC log; it does not start,
retrieve, synchronize, or validate that log. This repository implements
CSV v2 joint-target logging, writer-error handling, and independent
`Robot.start_recording` / `stop_recording` / `recording_status` plus a
`fr3-recording` CLI on port 5557. See [Recording](recording.md) for
the acknowledged recording lifecycle and persistent container output mount.
These APIs require the updated NUC daemon and Python client; the excitation
script still only records the `--nuc-log` reference.
Full daemon build, hardware acceptance and portable export are still pending.
These changes are not installed on the robot by this script. Do not fit
millisecond delay from this client's command/PUB log alone.

Supply hardware provenance without changing controller behavior:

```bash
uv run python examples/joint_sysid.py --execute --host NUC_HOST --robot-id left \
  --experiment-metadata /path/to/experiment.json \
  --nuc-log /path/on/nuc/to/independent-1khz.csv
```

The optional JSON object can contain robot/Desk versions, mount order, measured
payload mass, flange-frame center of mass and inertia, fixed hand joint pose,
and software revisions. Unknown values should remain null, not fabricated or
copied from the illustrative sim asset. Use distinct robot IDs and output
directories for the two arms, and validate each independently.

## Required handoff to simulation

The planned portable bundle is `joint_sysid_recording/v1` with `metadata.json`,
`samples.csv`, and a conversion/validation report. This is a draft contract, not
an implemented output option in this script. The excitation script still
emits `fr3_joint_excitation/v1` diagnostics with `sysid_ready=false`.

| Producer requirement | Status and acceptance evidence |
| --- | --- |
| Writer checks for write, flush, and close failures | Implemented/tested; failed I/O and drops invalidate completion; NUC acceptance pending |
| Per-tick active pre-EMA and effective post-EMA joint targets | Implemented/tested in CSV v2, with applied K/D/alpha/friction flag and reset count; NUC acceptance pending |
| State/timing/torque semantics | Shared capture tests pass; CSV v2 includes robot state time and actual callback period alongside the existing host-end timestamp; full dispatch/hardware verification pending |
| Applied configuration and initialization | Verified K/D/alpha, compensations, limits, reset state and warm-up; not only client-requested values |
| Payload and frame provenance | Physical assembly and configured loads recorded separately, with source, uncertainty, hand pose, and asset/frame definitions |
| Portable exporter | Explicit joint-name/unit mapping, original hashes, versioned metadata, and rejection of invalid/gapped/aborted runs without sibling imports |

The logger's callback-end `t_s` must not be silently relabeled as a
state timestamp. CSV v2 adds `robot_time_s` and `control_period_s` explicitly.
Host send and NUC application use different clocks. Per-tick
active targets are needed to replay actual holds; delay fitting must not absorb
unknown timestamp alignment or count upstream command delay twice. Preserve raw
files and do not interpolate missing ticks into an apparently complete run.

Physical payload is unresolved for the reported FR3 -> mount -> XHand assembly.
The calibrated URDF's XHand mass sum (`1.10146642 kg`) excludes a separately
modeled mount inertia and is not a measurement of the complete physical load.
Record physical mass/CoM/inertia and configured Desk values independently; a
configured zero additional load does not establish that no hand is installed.

## Delivery order

1. Fix logging and freeze the portable contract; develop synthetic validation
   and controller fixtures while hardware provenance is being established.
2. Accept real hold/small-motion recordings and single-environment nominal
   replay with fixed hand posture and documented payload assumptions.
3. Collect informative chirp/multisine runs and reserve whole independent runs;
   freeze known gains, filtering, compensation, and timing during each fit.
4. Fit only identifiable parameter groups with fixed-scale q/dq error, finite
   justified bounds, and a baseline; inspect repeated seeds and bound hits.
5. Validate on untouched recordings against predeclared task thresholds,
   export a versioned profile, and reproduce its scores from saved artifacts.
6. Evaluate training transfer separately: current sim implicit PD at 120 Hz
   physics / 60 Hz policy is not proven equivalent to real 1 kHz impedance.
   A 3 ms actuator delay is not three steps at either training rate.

No formal fitting should start before complete recordings and controller
parity are established. The root `TODO_FRANKATWIN.md` tracks real-side tasks
and cross-repository acceptance; the sim design owns the detailed loss,
parameter/profile contract, and training-transfer gates.
