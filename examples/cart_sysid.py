#!/usr/bin/env python3
"""FrankaTwin Cartesian references through the existing fr3-stack controller.

Offline preview by default. Execution requires an already running Cartesian
impedance daemon; no homing, controller switching or automatic stage progression.
The original reference generators are pinned, not rewritten as joint motion.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np

if __package__:
    from . import joint_sysid as common
    from ._frankatwin_reference import chirp, multiband
else:
    import joint_sysid as common
    from _frankatwin_reference import chirp, multiband

UPSTREAM_REVISION = 'acf16009c09c99c9a2bb86050b57eb248982ba9f'
PREVIEW_POS = np.array([.465, 0., .5])
PREVIEW_QUAT = np.array([1., 0., 0., 0.])


def finite_vector(value, size, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f'{name} must contain {size} finite values')
    return a


def quaternion(value):
    q = finite_vector(value, 4, 'xyzw quaternion')
    norm = np.linalg.norm(q)
    if abs(norm - 1.) > 1e-3:
        raise ValueError('quaternion must be unit length')
    return q / norm


def angle_between(a, b):
    return 2 * np.arccos(np.clip(np.abs(np.sum(a * b, axis=-1)), 0., 1.))


STAGES = ('hold', 'chirp', 'chirp-high', 'validation')


@dataclass(frozen=True)
class Plan:
    stage: str = 'chirp'
    scale: float = .1
    padding_hold: float = 2.
    hold_seconds: float = 10.
    rate: float = 50.
    stale_after: float = .1
    max_lateness: float = .05
    # Optional pose-tracking aborts; 0 (default) disables them, as in FrankaTwin.
    max_position_error: float = 0.
    max_angle_error: float = 0.
    max_reference_speed: float = .6
    max_reference_angular_speed: float = 3.

    @property
    def duration(self):
        return self.hold_seconds if self.stage == 'hold' else 8. if self.stage.startswith('chirp') else 12.

    @property
    def total(self):
        return self.duration + 2 * self.padding_hold

    def validate(self):
        if self.stage not in STAGES:
            raise ValueError('unknown Cartesian stage')
        optional = ('max_position_error', 'max_angle_error')
        numeric = [v for k, v in asdict(self).items() if k != 'stage' and k not in optional]
        if not np.isfinite(numeric).all() or min(numeric) <= 0:
            raise ValueError('all numeric settings must be finite and positive')
        guards = [getattr(self, k) for k in optional]
        if not np.isfinite(guards).all() or min(guards) < 0:
            raise ValueError('tracking-error limits must be finite and >= 0 (0 disables)')
        if self.scale > 1 or self.total > 120 or self.rate != 50:
            raise ValueError('scale must be <= 1, duration <= 120 s and target rate 50 Hz')


class Reference:
    def __init__(self, plan, pos, quat):
        plan.validate()
        self.plan = plan
        self.pos = finite_vector(pos, 3, 'anchor position')
        self.quat = quaternion(quat)

    def evaluate(self, times):
        t = np.atleast_1d(np.asarray(times, dtype=float))
        p = self.plan
        if p.stage == 'hold':
            return (np.tile(self.pos, (len(t), 1)), np.zeros((len(t), 3)),
                    np.tile(self.quat, (len(t), 1)))
        # Upstream derives the sweep/envelope duration from grid[-1]. Always
        # append the fixed endpoint, including for a single streamed sample.
        s = np.append(np.clip(t - p.padding_hold, 0., p.duration), p.duration)
        if p.stage.startswith('chirp'):
            # chirp = FrankaTwin low band (fit); chirp-high = its high band,
            # 0.7 -> 3 Hz with amplitudes tapered by (f0/f)^0.5.
            band = 'high' if p.stage == 'chirp-high' else 'low'
            pos, vel, quat, _ = chirp.build_chirp_trajectory(
                s, self.pos, self.quat, amp_x=.1*p.scale, amp_y=.1*p.scale,
                amp_z=.15*p.scale, amp_rx=.5*p.scale, amp_ry=.25*p.scale,
                amp_rz=.5*p.scale, **chirp.CHIRP_BANDS[band])
        else:
            options = dict(multiband.MULTIBAND_PROFILES['heldout'])
            for key in ('amp_x', 'amp_y', 'amp_z', 'amp_yaw', 'amp_roll', 'amp_rx', 'amp_ry'):
                options[key] *= p.scale
            pos, vel, quat, *_ = multiband.build_multiband_trajectory(s, self.pos, self.quat, **options)
        vel = vel[:-1].copy()
        outside = (t < p.padding_hold) | (t > p.padding_hold + p.duration)
        vel[outside] = 0.
        return pos[:-1], vel, quat[:-1]

    def check(self):
        t = np.linspace(0., self.plan.total, int(np.ceil(self.plan.total*1000))+1)
        pos, vel, quat = self.evaluate(t)
        if not all(np.isfinite(a).all() for a in (pos, vel, quat)):
            raise ValueError('nonfinite reference')
        if not np.allclose(np.linalg.norm(quat, axis=1), 1., atol=1e-9):
            raise ValueError('reference quaternion is not normalized')
        if (np.max(np.abs(pos[[0, -1]] - self.pos)) > 1e-9 or
                np.max(angle_between(quat[[0, -1]], self.quat)) > 1e-7):
            raise ValueError('reference does not start/end at the anchor')
        peaks = dict(position_offset_m=np.max(np.abs(pos-self.pos), axis=0).tolist(),
                     angle_offset_rad=float(np.max(angle_between(quat, self.quat))),
                     linear_speed_m_s=float(np.max(np.linalg.norm(vel, axis=1))),
                     angular_speed_rad_s=float(np.max(angle_between(quat[1:], quat[:-1])/np.diff(t))))
        if (peaks['linear_speed_m_s'] > self.plan.max_reference_speed or
                peaks['angular_speed_rad_s'] > self.plan.max_reference_angular_speed):
            raise ValueError('reference speed exceeds configured bounds; reduce scale')
        return peaks


def preview(output, ref):
    t = np.linspace(0., ref.plan.total, int(np.ceil(ref.plan.total*ref.plan.rate))+1)
    pos, vel, quat = ref.evaluate(t)
    with (output/'reference.csv').open('x', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz'])
        w.writerows(np.column_stack([t, pos, quat, vel]))
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="650">',
           '<rect width="1000" height="650" fill="white"/>',
           '<g font-family="sans-serif" font-size="13">',
           '<text x="60" y="25">FrankaTwin Cartesian reference; not a collision/torque certificate</text>']
    for panel, (label, values) in enumerate((('Position offset (mm)', 1000*(pos-ref.pos)),
                                            ('Quaternion (xyzw)', quat), ('Linear velocity (m/s)', vel))):
        top = 65 + panel*190
        span = max(float(np.max(np.abs(values))), 1e-6)*1.1
        svg += [f'<text x="60" y="{top-12}">{label}; +/- {span:.4g}</text>',
                f'<rect x="60" y="{top}" width="900" height="130" fill="none" stroke="#cbd5e1"/>']
        for col in range(values.shape[1]):
            points = ' '.join(f'{60+900*s/ref.plan.total:.2f},{top+65-65*y/span:.2f}'
                              for s, y in zip(t, values[:, col]))
            svg.append(f'<polyline points="{points}" fill="none" stroke="{common.COLORS[col]}"/>')
    (output/'preview.svg').write_text('\n'.join([*svg, '</g></svg>']))


def healthy(state, now, plan):
    if not state.valid or state.received_at is None or not np.isfinite(state.received_at):
        raise RuntimeError('no valid timestamped state')
    age = now-state.received_at
    if age < 0 or age > plan.stale_after:
        raise RuntimeError('stale Cartesian feedback')
    if not state.running or state.last_error or state.controller != 'cartesian_impedance':
        raise RuntimeError('daemon must already be healthy in cartesian_impedance; no mode switch sent')
    if not np.isfinite(state.timestamp):
        raise RuntimeError('nonfinite state timestamp')
    finite_vector(state.pos, 3, 'measured position')
    quaternion(state.quat_xyzw)
    q = common.vector(state.q, 'measured q')
    common.vector(state.dq, 'measured dq')
    if np.any(q <= common.Q_LOWER) or np.any(q >= common.Q_UPPER):
        raise RuntimeError('joint enters the repulsion region')
    # Joint speed is never policed, as in FrankaTwin: runaway is left to the
    # daemon torque clamp/slew limit and the libfranka reflexes.
    return state


def resolve_settings(profile):
    from fr3_stack.config import load_controller_config
    from fr3_stack.wire import flat_cart
    cfg = load_controller_config('cartesian_impedance', profile)
    flat = flat_cart(cfg)
    for key, n in (('K', 6), ('D', 6), ('q_null', 7), ('target_wrench', 6), ('max_delta', 6)):
        finite_vector(flat[key], n, key)
    numeric = np.array([flat[k] for k in ('K_null', 'D_null', 'max_tau_null', 'filter_alpha')])
    if (not np.isfinite(numeric).all() or np.any(np.array(flat['K']) <= 0) or
            np.any(np.array(flat['D']) < 0) or np.any(np.array(flat['max_delta']) < 0) or
            np.any(numeric[:3] < 0) or not 0 < flat['filter_alpha'] <= 1):
        raise ValueError('invalid Cartesian controller settings')
    if any(flat['target_wrench']):
        raise ValueError('collection requires zero configured feedforward wrench')
    return flat, cfg.get('_loaded_from')


def stream(robot, ref, settings, writer, metadata, *, clock=time.monotonic, sleep=time.sleep):
    p = ref.plan
    origin = clock()
    deadline = origin
    last_pos, last_quat = ref.pos.copy(), ref.quat.copy()
    seq, slot = 0, 0
    peak_motion, peak_angle = 0., 0.
    peak_error_m, peak_error_rad = 0., 0.
    joint_peaks = np.zeros(7)
    while True:
        sleep(max(0., deadline-clock()))
        snapshot = robot.state
        now = clock()
        if now-deadline > p.max_lateness:
            raise RuntimeError('Cartesian send loop missed its deadline')
        state = healthy(snapshot, now, p)
        # Tracking error is always recorded. Aborting on it is opt-in: like
        # FrankaTwin (error_delta_pos 0), impedance lag is expected data, and
        # runaway is left to joint speed, torque clamp/slew and the reflexes.
        error_m = float(np.linalg.norm(state.pos-last_pos))
        error_rad = float(angle_between(quaternion(state.quat_xyzw), last_quat))
        peak_error_m, peak_error_rad = max(peak_error_m, error_m), max(peak_error_rad, error_rad)
        if ((p.max_position_error > 0 and error_m > p.max_position_error) or
                (p.max_angle_error > 0 and error_rad > p.max_angle_error)):
            metadata['guard_failure'] = dict(kind='pose_tracking', position_error_m=error_m,
                                             angle_error_rad=error_rad, trajectory_t_s=now-origin)
            raise RuntimeError(f'Cartesian tracking error: {error_m:.6g} m, {error_rad:.6g} rad')
        elapsed = min(now-origin, p.total)
        pos, _, quat = ref.evaluate([elapsed])
        metadata['motion_command_attempted'] = True
        robot.send_cartesian_impedance(pos[0], quat[0], **settings)
        returned = clock()
        metadata['commands_sent'] = seq+1
        writer.writerow([seq, elapsed, now, returned, state.timestamp, state.received_at,
                         *pos[0], *quat[0], *state.pos, *state.quat_xyzw, *state.q, *state.dq])
        peak_motion = max(peak_motion, float(np.linalg.norm(state.pos-ref.pos)))
        peak_angle = max(peak_angle, float(angle_between(quaternion(state.quat_xyzw), ref.quat)))
        metadata['observed_motion'] = dict(position_peak_m=peak_motion, angle_peak_rad=peak_angle,
                                           tracking_error_peak_m=peak_error_m,
                                           tracking_error_peak_rad=peak_error_rad)
        if 'anchor_q_rad' in metadata:
            joint_peaks = np.maximum(joint_peaks, np.abs(state.q-np.asarray(metadata['anchor_q_rad'])))
            metadata['observed_motion']['joint_peak_rad'] = joint_peaks.tolist()
        last_pos, last_quat = pos[0], quat[0]
        if returned-now > p.max_lateness:
            raise RuntimeError('Cartesian command send exceeded timing budget')
        if elapsed >= p.total:
            healthy(state, now, p)
            # Recorded, not enforced (as in FrankaTwin): joint friction leaves
            # a few-mm static offset at K 500, and the data is valid anyway.
            end_m = float(np.linalg.norm(state.pos-ref.pos))
            end_rad = float(angle_between(quaternion(state.quat_xyzw), ref.quat))
            metadata['final_offset'] = dict(
                position_m=end_m, angle_rad=end_rad,
                within_5mm_0p03rad=end_m <= .005 and end_rad <= .03,
                excitation_observed=p.stage == 'hold' or (
                    peak_motion >= .002*p.scale and peak_angle >= .05*p.scale))
            if not metadata['final_offset']['within_5mm_0p03rad']:
                print(f'note: run ended {end_m*1e3:.1f} mm / {end_rad:.3f} rad from the anchor',
                      file=sys.stderr)
            if not metadata['final_offset']['excitation_observed']:
                print('note: observed Cartesian motion is below the scale-dependent minimum',
                      file=sys.stderr)
            return
        seq += 1
        slot = max(slot+1, int(np.floor((returned-origin)*p.rate))+1)
        deadline = origin+slot/p.rate


def run_live(robot, plan, settings, output, metadata, args):
    recording_attempted = False
    try:
        state = healthy(robot.wait_for_state(timeout=5), time.monotonic(), plan)
        ref = Reference(plan, state.pos.copy(), state.quat_xyzw.copy())
        metadata.update(anchor_pos_m=ref.pos.tolist(), anchor_quat_xyzw=ref.quat.tolist(),
                        anchor_q_rad=state.q.tolist(), reference_peaks=ref.check())
        preview(output, ref)
        common.write_json(output/'metadata.json', metadata)
        if args.record:
            recording_attempted = True
            status = robot.start_recording(metadata['nuc_log_reference'],
                                          recording_id=metadata['recording_id'], timeout=2.)
            metadata['recording_start'] = asdict(status)
            common.write_json(output/'metadata.json', metadata)
            print(json.dumps(metadata['recording_start']), flush=True)
        state = healthy(robot.state, time.monotonic(), plan)
        if (np.linalg.norm(state.pos-ref.pos) > .002 or
                angle_between(quaternion(state.quat_xyzw), ref.quat) > .01):
            raise RuntimeError('arm moved during preparation')
        with (output/'commands_and_state.csv').open('x', newline='') as f:
            w = csv.writer(f)
            w.writerow(['seq', 'trajectory_t_s', 'send_start_monotonic_s', 'send_return_monotonic_s',
                        'state_daemon_timestamp', 'state_received_monotonic_s']+
                       [f'{key}{i}' for key, n in [('target_pos',3),('target_quat_xyzw',4),
                        ('measured_pos',3),('measured_quat_xyzw',4),('q',7),('dq',7)] for i in range(n)])
            metadata['status'] = 'running'
            print(f'Streaming Cartesian {plan.stage}: {plan.total:g} s; scale={plan.scale:g}', flush=True)
            stream(robot, ref, settings, w, metadata)
        metadata.update(status='completed', end_behavior='cartesian_hold')
    except BaseException as exc:
        metadata.update(status='aborted', error=f'{type(exc).__name__}: {exc}')
        if metadata['motion_command_attempted']:
            common.request_termination(robot, metadata)
        raise
    finally:
        if recording_attempted:
            try:
                status = robot.stop_recording(metadata['recording_id'], timeout=2.)
                metadata['recording_stop'] = asdict(status)
                if not status.complete or status.error or status.discarded or status.dropped or not status.written:
                    metadata['recording_error'] = 'Recording did not confirm a complete loss-free dataset'
            except Exception as exc:
                metadata['recording_stop_unconfirmed'] = str(exc)
        metadata['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        common.write_json(output/'metadata.json', metadata)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--dry-run', action='store_true')
    for name, field in Plan.__dataclass_fields__.items():
        kwargs = dict(default=field.default)
        if name == 'stage':
            kwargs['choices'] = list(STAGES)
        else:
            kwargs['type'] = float
        p.add_argument('--'+name.replace('_', '-'), **kwargs)
    p.add_argument('--host')
    p.add_argument('--robot-id')
    p.add_argument('--cmd-port', type=int, default=5555)
    p.add_argument('--state-port', type=int, default=5556)
    p.add_argument('--recording-port', type=int, default=5557)
    p.add_argument('--record', action='store_true', help='start/stop acknowledged 1 kHz recording for this stage')
    p.add_argument('--profile', help='existing Cartesian profile; filtering/other settings are preserved')
    p.add_argument('--experiment-metadata', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--anchor-pos', type=float, nargs=3, help='preview only')
    p.add_argument('--anchor-quat-xyzw', type=float, nargs=4, help='preview only')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        plan = Plan(**{name:getattr(args,name) for name in Plan.__dataclass_fields__})
        plan.validate()
        ports = (args.cmd_port, args.state_port, args.recording_port)
        if len(set(ports)) != 3 or any(not 1 <= port <= 65535 for port in ports):
            raise ValueError('ports must be valid and distinct')
        if args.execute and (not args.host or not args.host.strip() or '@' in args.host or
                             not args.robot_id or not args.robot_id.strip() or
                             args.anchor_pos is not None or args.anchor_quat_xyzw is not None):
            raise ValueError('execution requires host/robot-id and forbids preview anchors')
        experiment = json.loads(args.experiment_metadata.read_text()) if args.experiment_metadata else {}
        if not isinstance(experiment, dict):
            raise ValueError('experiment metadata must be a JSON object')
        json.dumps(experiment, allow_nan=False)
        run_id = datetime.now(timezone.utc).strftime(f'cart-{plan.stage}-%Y%m%dT%H%M%S-%fZ')
        output = args.output or Path.home()/'fr3-client-recordings'/run_id
        output.mkdir(parents=True, exist_ok=False)
        metadata = dict(schema='fr3_cart_excitation/v1', status='prepared',
                        created_at_utc=datetime.now(timezone.utc).isoformat(), plan=asdict(plan),
                        upstream_revision=UPSTREAM_REVISION,
                        upstream_sources={Path(m.__file__).name:hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
                                          for m in (chirp, multiband)},
                        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                        controller='fr3-stack cartesian_impedance, not upstream FrankaTwin controller',
                        controller_settings_acknowledged=False, sysid_ready=False,
                        robot_id=args.robot_id, host=args.host, experiment=experiment,
                        recording_requested=args.record,
                        recording_id=run_id, nuc_log_reference='/recordings/'+run_id+'.csv',
                        commands_sent=0, motion_command_attempted=False,
                        data_semantics='Client timestamps are not synchronized with NUC time; command sends are not acknowledgements. '
                                       'Cartesian effective gains/filtered targets are not in CSV v2; replay must match this controller.')
        print(f'Cartesian {plan.stage}; scale={plan.scale:g}; total={plan.total:g} s; output={output}', flush=True)
        print('Live run requires reviewed arm/XHand swept space, payload, fixed hand pose and local stop.', flush=True)
        common.write_json(output/'metadata.json', metadata)
        if not args.execute:
            ref = Reference(plan, args.anchor_pos if args.anchor_pos is not None else PREVIEW_POS,
                            args.anchor_quat_xyzw if args.anchor_quat_xyzw is not None else PREVIEW_QUAT)
            metadata.update(status='preview', reference_peaks=ref.check(),
                            anchor_pos_m=ref.pos.tolist(), anchor_quat_xyzw=ref.quat.tolist(),
                            anchor_source='preview_only')
            preview(output, ref)
            common.write_json(output/'metadata.json', metadata)
            print(json.dumps(metadata['reference_peaks'], indent=2))
            return 0
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from fr3_stack.robot import Robot
        settings, source = resolve_settings(args.profile)
        metadata.update(commanded_controller_settings=settings, controller_profile_path=source)
        def interrupted(signum, frame):
            raise KeyboardInterrupt(f'signal {signum}')
        previous = signal.signal(signal.SIGTERM, interrupted)
        try:
            with Robot(args.host, args.cmd_port, args.state_port, recording_port=args.recording_port,
                       command_send_timeout_ms=20) as robot:
                run_live(robot, plan, settings, output, metadata, args)
        finally:
            signal.signal(signal.SIGTERM, previous)
        if metadata.get('recording_error') or metadata.get('recording_stop_unconfirmed'):
            print('Motion completed, but recording is not confirmed complete; inspect metadata.', flush=True)
            return 2
        print('Completed; Cartesian hold remains active. '+str(output), flush=True)
        if args.record:
            print(json.dumps(metadata['recording_stop']), flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        if 'metadata' in locals() and metadata['status'] != 'aborted':
            metadata.update(status='failed', error=f'{type(exc).__name__}: {exc}')
            common.write_json(output/'metadata.json', metadata)
        print(f'CART FAIL: {exc}', file=sys.stderr)
        if 'metadata' in locals():
            print(f'Termination: {metadata.get("termination", "not requested")}; logs: {output}', file=sys.stderr)
            if metadata.get('recording_stop_unconfirmed'):
                print('Recording stop unconfirmed; inspect daemon writer/CSV without restarting just for an ACK.', file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1


if __name__ == '__main__':
    raise SystemExit(main())
