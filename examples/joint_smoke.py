#!/usr/bin/env python3
"""Repeatable single-joint connection and small-motion check.

Default: offline preview. --check-only reads state without sending commands.
--execute uses the live pose, holds, streams a slow sine, then holds the anchor.
The daemon must already be stationary in joint_impedance mode.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import socket
import sys

import numpy as np

if __package__:
    from . import joint_sysid as excitation
else:
    import joint_sysid as excitation


def smoke_plan(joint=1, amplitude_deg=.25):
    if not np.isfinite(amplitude_deg) or not .1 <= amplitude_deg <= .5:
        raise ValueError('smoke amplitude must be in 0.1..0.5 degrees')
    return excitation.Plan(joints=(joint,), amplitude_deg=amplitude_deg,
                           seconds=20., ramp=5., hold=3., f0=.1, rate=60.,
                           max_velocity=.01, max_acceleration=.05, max_jerk=.5)


def check_response(output, anchor, joint, amplitude_deg):
    """Check observed motion, previous-command tracking, and return to anchor."""
    with (output / 'commands_and_state.csv').open(newline='') as file:
        rows = list(csv.DictReader(file))
    if len(rows) < 2:
        raise ValueError('not enough feedback samples to verify motion')
    signals = [np.array([[float(row[f'{key}{j}']) for j in range(1, 8)] for row in rows])
               for key in ('q_target', 'q_measured', 'dq_measured')]
    if not all(np.isfinite(signal).all() for signal in signals):
        raise ValueError('nonfinite motion feedback')
    targets, measured, velocities = signals
    amplitude = np.deg2rad(amplitude_deg)
    excursion = float(np.max(np.abs(measured[:, joint - 1] - anchor[joint - 1])))
    tracking = float(np.max(np.abs(measured[1:] - targets[:-1])))
    final_error = float(np.max(np.abs(measured[-1] - anchor)))
    final_speed = float(np.max(np.abs(velocities[-1])))
    errors = []
    if excursion < .5 * amplitude:
        errors.append('expected joint motion was not observed')
    if tracking > .5 * amplitude:
        errors.append('measured motion did not track the previous sent targets closely enough')
    if final_error > .002 or final_speed > .005:
        errors.append('feedback did not settle back at the anchor')
    return dict(status='failed' if errors else 'passed', errors=errors,
                measured_excursion_deg=float(np.rad2deg(excursion)),
                minimum_excursion_deg=.5 * amplitude_deg,
                max_tracking_error_deg=float(np.rad2deg(tracking)),
                tracking_error_limit_deg=.5 * amplitude_deg,
                final_anchor_error_rad=final_error, final_speed_rad_s=final_speed,
                final_anchor_error_limit_rad=.002, final_speed_limit_rad_s=.005,
                command_application_acknowledged=False)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='run the small motion on the specified NUC')
    mode.add_argument('--check-only', action='store_true', help='check fresh, stationary joint-mode state; no commands')
    mode.add_argument('--dry-run', action='store_true', help='offline preview (the default)')
    p.add_argument('--host', help='NUC host; required for --execute or --check-only')
    p.add_argument('--robot-id', help='arm identifier; required for --execute')
    p.add_argument('--cmd-port', type=int, default=5555)
    p.add_argument('--state-port', type=int, default=5556)
    p.add_argument('--joint', type=int, choices=range(1, 8), default=1)
    p.add_argument('--amplitude-deg', type=float, default=.25, help='peak offset, bounded to 0.1..0.5 degrees')
    p.add_argument('--profile', default='sysid_stiff', help='joint impedance profile; default: sysid_stiff')
    p.add_argument('--output', type=Path, help='new log directory; default: timestamped under recordings/')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        plan = smoke_plan(args.joint, args.amplitude_deg)
        plan.validate()
        if (args.execute or args.check_only) and not args.host:
            raise ValueError('--host is required for a live check')
        if args.execute and not args.robot_id:
            raise ValueError('--robot-id is required for --execute')
        if (args.cmd_port == args.state_port or
                any(not 1 <= port <= 65535 for port in (args.cmd_port, args.state_port))):
            raise ValueError('command/state ports must be distinct and in 1..65535')
        if args.execute or args.check_only:
            if '@' in args.host:
                raise ValueError('--host expects a NUC IP/hostname, not an SSH user@host target')
            try:
                socket.getaddrinfo(args.host, args.state_port, type=socket.SOCK_STREAM)
            except socket.gaierror as exc:
                raise ValueError(f'cannot resolve NUC host {args.host!r}; use its actual IP/hostname, '
                                 'not the SSH login username or an SSH-only alias') from exc
        if args.check_only:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
            from fr3_stack.robot import Robot
            with Robot(args.host, args.cmd_port, args.state_port, command_send_timeout_ms=20) as robot:
                try:
                    initial = robot.wait_for_state(timeout=5)
                except TimeoutError as exc:
                    raise TimeoutError(f'no valid state from tcp://{args.host}:{args.state_port} within 5 s; '
                                       'check that the NUC daemon is running and the configured state port '
                                       'is reachable from this workstation') from exc
                state = excitation.healthy_state(initial, excitation.time.monotonic(),
                                                 stale_after=.1, max_velocity=plan.max_velocity)
                excitation.Trajectory(plan).check(state.q)
                print(f'STATE CHECK PASS: fresh stationary joint_impedance feedback from {args.host}.')
                print('No commands sent; the command path and physical response are not verified.')
            return 0
        output = args.output or Path('recordings') / datetime.now(timezone.utc).strftime('smoke-%Y%m%dT%H%M%S-%fZ')
        options = ['--execute' if args.execute else '--dry-run', '--output', str(output),
                   '--profile', args.profile, '--joints', str(args.joint),
                   '--tracking-error', str(.5 * np.deg2rad(args.amplitude_deg)),
                   '--stale-after', '.1', '--max-lateness', '.05']
        for name in ('amplitude_deg', 'seconds', 'ramp', 'hold', 'f0', 'rate',
                     'max_velocity', 'max_acceleration', 'max_jerk'):
            options += ['--' + name.replace('_', '-'), str(getattr(plan, name))]
        for flag, value in (('--host', args.host), ('--robot-id', args.robot_id),
                            ('--cmd-port', args.cmd_port), ('--state-port', args.state_port)):
            if value is not None:
                options += [flag, str(value)]
        code = excitation.main(options)
        if code:
            return code
        metadata = json.loads((output / 'metadata.json').read_text())
        result = dict(status='preview', command_application_acknowledged=False)
        if args.execute:
            try:
                result = check_response(output, excitation.vector(metadata['anchor_q_rad'], 'anchor'),
                                        args.joint, args.amplitude_deg)
            except (ValueError, OSError, KeyError) as exc:
                result = dict(status='failed', errors=[str(exc)], command_application_acknowledged=False)
            if result['status'] == 'failed':
                metadata.update(excitation_status=metadata['status'], status='failed')
        result['entrypoint_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        metadata['smoke_test'] = result
        excitation.write_json(output / 'metadata.json', metadata)
        if not args.execute:
            print('PREVIEW ONLY: no connection or commands; add --execute for the live check.')
            return 0
        if result['status'] == 'failed':
            print('SMOKE FAIL: ' + '; '.join(result['errors']), file=sys.stderr)
            return 1
        print(f'SMOKE PASS: live feedback observed J{args.joint} motion and return to the anchor; logs: {output}')
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f'SMOKE FAIL: {exc}', file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1


if __name__ == '__main__':
    raise SystemExit(main())
