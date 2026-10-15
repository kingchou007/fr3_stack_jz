#!/usr/bin/env python3
"""Preview or run exactly one small-range joint excitation stage.

Default: offline J1 smoke preview. Live execution requires --execute, --host,
--robot-id and an already stationary joint_impedance daemon. No mode switching,
homing, automatic stage progression, recording startup or EMA override.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import sys

if __package__:
    from . import joint_smoke as smoke
    from . import joint_sysid as excitation
else:
    import joint_smoke as smoke
    import joint_sysid as excitation


SPEC_PATH = Path(__file__).resolve().parents[1] / 'configs/joint_sysid.initial.json'
STAGES = {
    'hold': '00_hold',
    'smoke': '01_smoke_j1',
    'chirp': '02_chirp_j1',
    'sequential': '03_chirp_sequential',
    'train': '04_multisine_train',
    'validation': '05_multisine_validation',
    'twin-chirp': '06_twin_chirp_j1',
    'twin-all': '07_twin_chirp_all',
    'twin-validation': '08_twin_multisine_validation',
    'twin-large': '09_twin_larger_low_chirp',
}


def load_stage(name, joint=None):
    """Resolve the checked-in preset; refuse drift from the smoke wrapper."""
    source = SPEC_PATH.read_bytes()
    spec = json.loads(source)
    if (spec['schema'] != 'fr3_joint_trajectory_campaign/v1' or
            spec['required_controller'] != 'joint_impedance'):
        raise ValueError('unsupported campaign schema or controller')
    matches = [stage for stage in spec['stages'] if stage['id'] == STAGES[name]]
    if len(matches) != 1:
        raise ValueError('campaign must contain exactly one matching stage')
    stage = matches[0]
    if stage.get('reference_design') not in (None, *spec.get('reference_designs', {})):
        raise ValueError('campaign reference design is missing')
    settings = {**spec['common_plan'], **stage['plan']}
    if joint is not None:
        if name not in ('smoke', 'chirp', 'twin-chirp') or joint not in range(1, 8):
            raise ValueError('--joint is only supported for smoke/chirp/twin-chirp, in 1..7')
        settings['joints'] = [joint]
    for key in ('joints', 'tones'):
        if key in settings:
            settings[key] = tuple(settings[key])
    plan = excitation.Plan(**settings)
    plan.validate()
    if plan.total != stage['expected_duration_s']:
        raise ValueError('campaign duration does not match the resolved plan')
    guards = {**spec['runtime_guards'], **stage.get('runtime_guards', {})}
    if (set(guards) != {'stale_after', 'tracking_error', 'max_lateness'} or
            any(not math.isfinite(value) or value <= 0 for value in guards.values())):
        raise ValueError('campaign runtime guards must be finite and positive')
    if name == 'smoke':
        expected_guards = dict(stale_after=.1, tracking_error=.5 * math.radians(plan.amplitude_deg),
                               max_lateness=.05)
        if plan != smoke.smoke_plan(plan.joints[0], plan.amplitude_deg) or guards != expected_guards:
            raise ValueError('campaign smoke preset no longer matches the checked response wrapper')
    return spec, stage, plan, guards, hashlib.sha256(source).hexdigest()


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='send this one stage to the NUC')
    mode.add_argument('--dry-run', action='store_true', help='offline preview (the default)')
    p.add_argument('--stage', choices=STAGES, default='smoke')
    p.add_argument('--joint', type=int, choices=range(1, 8), help='smoke/chirp/twin-chirp only; default J1')
    p.add_argument('--nuc-log', help='reference to separately started 1 kHz recording; not verified')
    p.add_argument('--experiment-metadata', type=Path, help='optional hardware/tool provenance JSON')
    p.add_argument('--host', help='NUC IP/hostname; required for --execute')
    p.add_argument('--robot-id', help='arm identifier; required for --execute')
    p.add_argument('--cmd-port', type=int, default=5555)
    p.add_argument('--state-port', type=int, default=5556)
    p.add_argument('--output', type=Path,
                   help='new directory; default: timestamped under ~/fr3-client-recordings/')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        spec, stage, plan, guards, spec_hash = load_stage(args.stage, args.joint)
        if (args.cmd_port == args.state_port or
                any(not 1 <= port <= 65535 for port in (args.cmd_port, args.state_port))):
            raise ValueError('command/state ports must be distinct and in 1..65535')
        if args.execute:
            if not args.host or not args.robot_id or not args.host.strip() or not args.robot_id.strip():
                raise ValueError('--execute requires --host and --robot-id')
            if '@' in args.host:
                raise ValueError('--host expects a NUC IP/hostname, not an SSH user@host target')
            try:
                socket.getaddrinfo(args.host, args.state_port, type=socket.SOCK_STREAM)
            except socket.gaierror as exc:
                raise ValueError('cannot resolve NUC host; use its actual IP/hostname, '
                                 'not the SSH login username') from exc
        output = args.output or (Path.home() / 'fr3-client-recordings') / datetime.now(timezone.utc).strftime(
            f'trajectory-{args.stage}-%Y%m%dT%H%M%S-%fZ')
        if output.exists() or output.is_symlink():
            raise ValueError(f'output already exists; choose a new directory: {output}')
        options = ['--execute' if args.execute else '--dry-run', '--output', str(output),
                   '--profile', spec['profile'], '--cmd-port', str(args.cmd_port),
                   '--state-port', str(args.state_port)]
        for flag, value in (('--host', args.host), ('--robot-id', args.robot_id)):
            if value is not None:
                options += [flag, value]
        if args.stage == 'smoke':
            # Keep the stricter observed-motion/settling verification, not just streaming success.
            options += ['--joint', str(plan.joints[0]), '--amplitude-deg', str(plan.amplitude_deg)]
            entrypoint = smoke.main
        else:
            for flag, value in (('--nuc-log', args.nuc_log), ('--experiment-metadata', args.experiment_metadata)):
                if value is not None:
                    options += [flag, str(value)]
            for key, value in {**asdict(plan), **guards}.items():
                values = value if isinstance(value, (tuple, list)) else [value]
                options += ['--' + key.replace('_', '-'), *map(str, values)]
            entrypoint = excitation.main
        print(f'Stage: {args.stage}; joints: {plan.joints}; duration: {plan.total:g} s; '
              f'profile: {spec["profile"]}', flush=True)
        print(f'Human review prerequisite (not automatically verified): {stage["gate"]}', flush=True)
        print(f'Client logs: {output}', flush=True)
        if plan.mode != 'hold':
            print(f'Peak offset bound: +/-{plan.amplitude_deg:g} deg per selected joint; '
                  f'tracking guard: {guards["tracking_error"]:.6g} rad '
                  f'({math.degrees(guards["tracking_error"]):g} deg)', flush=True)
        if plan.mode == 'chirp':
            print(f'Chirp: {plan.f0:g} to {plan.f1:g} Hz; layout: {plan.chirp_layout}', flush=True)
        if stage.get('reference_design'):
            print('FrankaTwin-inspired JOINT adaptation; not the upstream Cartesian trajectory.', flush=True)
        if args.stage == 'smoke' and (args.nuc_log or args.experiment_metadata):
            raise ValueError('use a non-smoke stage for --nuc-log/--experiment-metadata')
        if args.execute:
            print('One stage only. Live pose anchor; normal completion leaves joint hold active.', flush=True)
        code = entrypoint(options)
        metadata_path = output / 'metadata.json'
        if metadata_path.is_file():
            metadata = json.loads(metadata_path.read_text())
            metadata['trajectory_campaign'] = dict(
                schema=spec['schema'], stage_id=stage['id'], stage=args.stage,
                selected_joints=list(plan.joints), specification_sha256=spec_hash,
                entrypoint_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                human_review_prerequisite=stage['gate'], human_review_verified=False,
                automatic_progression=False)
            if stage.get('reference_design'):
                metadata['trajectory_campaign']['reference_design'] = spec['reference_designs'][stage['reference_design']]
            excitation.write_json(metadata_path, metadata)
        return code
    except (Exception, KeyboardInterrupt) as exc:
        print(f'TRAJECTORY FAIL: {exc}', file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1


if __name__ == '__main__':
    raise SystemExit(main())
