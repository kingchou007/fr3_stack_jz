"""Pinned Cartesian references, preflight, wire and recording lifecycle."""
from dataclasses import asdict, replace
import csv
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from examples import cart_sysid as cs
from fr3_stack.state import State
from fr3_stack.recording import RecordingStatus, RecordingTimeout
from fr3_stack.wire import SCHEMA


SETTINGS = dict(K=[200.]*3+[20.]*3, D=[28.]*3+[9.]*3, q_null=[0.]*7,
                K_null=100., D_null=0., max_tau_null=0., filter_alpha=.05,
                target_wrench=[0.]*6, max_delta=[0.]*6, use_friction=False,
                linear_interp=True, ema=True)


def test_vendored_files_match_upstream_identity():
    for module, digest in ((cs.chirp, 'fe06cfd24596820efc6d5f19c411ec3a56268dc28f30502049400154d49553a9'),
                           (cs.multiband, 'd64ce458e04a145772afa433353d0e04da951d83f7570fc0d3a40d1310a1a848')):
        assert hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest() == digest


@pytest.mark.parametrize('stage', ['chirp', 'validation'])
@pytest.mark.parametrize('scale', [.2, 1.])
def test_reference_matches_upstream_including_single_sample_duration(stage, scale):
    p = cs.Plan(stage=stage, scale=scale)
    ref = cs.Reference(p, cs.PREVIEW_POS, cs.PREVIEW_QUAT)
    t = np.linspace(0., p.duration, 1001)
    if stage == 'chirp':
        expected = cs.chirp.build_chirp_trajectory(t, ref.pos, ref.quat,
            amp_x=.1*scale, amp_y=.1*scale, amp_z=.15*scale,
            amp_rx=.5*scale, amp_ry=.25*scale, amp_rz=.5*scale, **cs.chirp.CHIRP_BANDS['low'])[:3]
    else:
        options = dict(cs.multiband.MULTIBAND_PROFILES['heldout'])
        for key in ('amp_x', 'amp_y', 'amp_z', 'amp_yaw', 'amp_roll', 'amp_rx', 'amp_ry'):
            options[key] *= scale
        expected = cs.multiband.build_multiband_trajectory(t, ref.pos, ref.quat, **options)[:3]
    actual = ref.evaluate(t+p.padding_hold)
    for a,b in zip(actual,expected):
        np.testing.assert_allclose(a,b,atol=1e-14)
    for k in (123, 400, 678):
        for a,b in zip(ref.evaluate([t[k]+p.padding_hold]),actual):
            np.testing.assert_allclose(a[0],b[k],atol=1e-14)
    ref.check()


@pytest.mark.parametrize('scale', [.2, .4])
def test_high_band_matches_upstream(scale):
    p = cs.Plan(stage='chirp-high', scale=scale)
    ref = cs.Reference(p, cs.PREVIEW_POS, cs.PREVIEW_QUAT)
    t = np.linspace(0., p.duration, 1001)
    expected = cs.chirp.build_chirp_trajectory(t, ref.pos, ref.quat,
        amp_x=.1*scale, amp_y=.1*scale, amp_z=.15*scale, amp_rx=.5*scale,
        amp_ry=.25*scale, amp_rz=.5*scale, **cs.chirp.CHIRP_BANDS['high'])[:3]
    for a, b in zip(ref.evaluate(t+p.padding_hold), expected):
        np.testing.assert_allclose(a, b, atol=1e-14)
    ref.check()


def test_high_band_above_reference_speed_bound_is_refused():
    with pytest.raises(ValueError, match='reference speed'):
        cs.Reference(cs.Plan(stage='chirp-high', scale=.5), cs.PREVIEW_POS, cs.PREVIEW_QUAT).check()


@pytest.mark.parametrize('stage', ['hold', 'chirp', 'chirp-high', 'validation'])
def test_holds_recover_anchor_and_quaternions_are_xyzw(stage):
    p = cs.Plan(stage=stage)
    ref = cs.Reference(p, cs.PREVIEW_POS, cs.PREVIEW_QUAT)
    pos, vel, quat = ref.evaluate([0., p.padding_hold/2, p.total, p.total+1])
    np.testing.assert_allclose(pos, np.tile(ref.pos,(4,1)), atol=1e-12)
    np.testing.assert_allclose(vel, 0., atol=1e-12)
    np.testing.assert_allclose(quat, np.tile(ref.quat,(4,1)), atol=1e-12)


@pytest.mark.parametrize('change', [dict(scale=0),dict(scale=1.1),dict(scale=float('nan')),
    dict(stage='high'),dict(rate=60),dict(max_position_error=-1),dict(padding_hold=-1)])
def test_invalid_plan(change):
    with pytest.raises(ValueError):
        cs.Reference(replace(cs.Plan(),**change), cs.PREVIEW_POS, cs.PREVIEW_QUAT)


def test_offline_preview_never_imports_transport_and_preserves_existing_output(tmp_path):
    output=tmp_path/'preview'
    code='''
import runpy,sys
class RejectTransport:
    def find_spec(self,fullname,*args):
        if fullname.startswith(('fr3_stack','zmq','capnp')):
            raise RuntimeError('offline preview imported transport')
sys.meta_path.insert(0,RejectTransport())
sys.path.insert(0,sys.argv[1])
sys.argv=[sys.argv[1]+'/cart_sysid.py','--scale','.2','--output',sys.argv[2]]
runpy.run_path(sys.argv[0],run_name='__main__')
'''
    r=subprocess.run([sys.executable,'-c',code,str(Path(cs.__file__).parent),str(output)],
                     text=True,capture_output=True,timeout=20)
    assert r.returncode==0,r.stderr
    data=json.loads((output/'metadata.json').read_text())
    assert data['status']=='preview' and data['commands_sent']==0
    assert data['sysid_ready'] is False
    assert (output/'reference.csv').is_file() and (output/'preview.svg').is_file()
    before=(output/'metadata.json').read_bytes()
    assert cs.main(['--output',str(output)])==1
    assert (output/'metadata.json').read_bytes()==before


class Clock:
    now=10.
    def __call__(self):return self.now
    def sleep(self,seconds):self.now+=seconds


class StubRobot:
    def __init__(self,clock):
        self.clock=clock
        self.pos=cs.PREVIEW_POS.copy()
        self.quat=cs.PREVIEW_QUAT.copy()
        self.sent=[]
        self.changes={}
        self.terminated=False
        self.recording_calls=[]
        self.record_timeout=False
    @property
    def state(self):
        fields=dict(controller='cartesian_impedance',pos=self.pos,quat_xyzw=self.quat,
                    q=cs.common.PREVIEW_Q.copy(),dq=np.zeros(7),timestamp=1.,
                    received_at=self.clock(),running=True,valid=True)
        fields.update(self.changes)
        return State(**fields)
    def wait_for_state(self,timeout):return self.state
    def send_cartesian_impedance(self,pos,quat,**settings):
        self.sent.append((pos.copy(),quat.copy(),settings))
        self.pos,self.quat=pos.copy(),quat.copy()
    def terminate(self):self.terminated=True
    def start_recording(self,path,recording_id,timeout):
        self.recording_calls.append(('start',recording_id,path))
        return RecordingStatus(recording_id,path,True,True,False,False,0,0,0,'',0)
    def stop_recording(self,recording_id,timeout):
        self.recording_calls.append(('stop',recording_id))
        if self.record_timeout:raise RecordingTimeout(recording_id)
        return RecordingStatus(recording_id,'/recordings/test.csv',False,True,True,True,200,0,0,'',0)


@pytest.mark.parametrize('stage',['hold','chirp','chirp-high','validation'])
def test_stream_preserves_settings_and_finishes_holding_anchor(stage):
    clock=Clock(); robot=StubRobot(clock)
    p=cs.Plan(stage=stage,scale=.2,max_lateness=.2)
    ref=cs.Reference(p,robot.pos,robot.quat)
    metadata={'commands_sent':0,'motion_command_attempted':False}
    cs.stream(robot,ref,SETTINGS,csv.writer(io.StringIO()),metadata,clock=clock,sleep=clock.sleep)
    assert metadata['commands_sent']==len(robot.sent)
    assert all(settings==SETTINGS for _,_,settings in robot.sent)
    np.testing.assert_allclose(robot.sent[-1][0],ref.pos,atol=1e-12)
    np.testing.assert_allclose(robot.sent[-1][1],ref.quat,atol=1e-12)
    assert not robot.terminated


class LaggingRobot(StubRobot):
    """Follows each target 20 sends (400 ms at 50 Hz) late, like the real arm."""
    def __init__(self,clock):
        super().__init__(clock); self.queue=[]
    def send_cartesian_impedance(self,pos,quat,**settings):
        self.sent.append((pos.copy(),quat.copy(),settings))
        self.queue.append((pos.copy(),quat.copy()))
        if len(self.queue)>20:self.pos,self.quat=self.queue.pop(0)


class OffsetRobot(StubRobot):
    """Comes to rest 8 mm from each target, like friction at K 500."""
    def send_cartesian_impedance(self,pos,quat,**settings):
        self.sent.append((pos.copy(),quat.copy(),settings))
        self.pos,self.quat=pos.copy()+np.array([.008,0.,0.]),quat.copy()


def test_final_offset_is_recorded_not_aborted():
    clock=Clock(); robot=OffsetRobot(clock)
    p=cs.Plan(stage='chirp',scale=.2,max_lateness=.2)
    ref=cs.Reference(p,robot.pos,robot.quat)
    metadata={'commands_sent':0,'motion_command_attempted':False}
    cs.stream(robot,ref,SETTINGS,csv.writer(io.StringIO()),metadata,clock=clock,sleep=clock.sleep)
    end=metadata['final_offset']
    assert abs(end['position_m']-.008)<1e-9 and end['within_5mm_0p03rad'] is False
    assert not robot.terminated


def test_large_tracking_lag_is_recorded_not_aborted():
    clock=Clock(); robot=LaggingRobot(clock)
    p=cs.Plan(stage='chirp',scale=1.,max_lateness=.2)
    ref=cs.Reference(p,robot.pos,robot.quat)
    metadata={'commands_sent':0,'motion_command_attempted':False}
    cs.stream(robot,ref,SETTINGS,csv.writer(io.StringIO()),metadata,clock=clock,sleep=clock.sleep)
    motion=metadata['observed_motion']
    assert motion['tracking_error_peak_m']>.03      # old guard would have aborted
    assert motion['tracking_error_peak_rad']>0
    assert 'guard_failure' not in metadata and not robot.terminated


def test_joint_speed_is_never_policed():
    clock=Clock(); state=StubRobot(clock).state
    fast=State(**{**state.__dict__,'dq':np.full(7,2.)})
    cs.healthy(fast,clock(),cs.Plan())
    assert 'max_joint_speed' not in cs.Plan.__dataclass_fields__


@pytest.mark.parametrize('limits',[dict(max_position_error=.03),dict(max_angle_error=.01)])
def test_tracking_guard_aborts_only_when_enabled(limits):
    clock=Clock(); robot=LaggingRobot(clock)
    p=cs.Plan(stage='chirp',scale=1.,max_lateness=.2,**limits)
    ref=cs.Reference(p,robot.pos,robot.quat)
    metadata={'commands_sent':0,'motion_command_attempted':False}
    with pytest.raises(RuntimeError,match='Cartesian tracking error'):
        cs.stream(robot,ref,SETTINGS,csv.writer(io.StringIO()),metadata,clock=clock,sleep=clock.sleep)
    assert metadata['guard_failure']['kind']=='pose_tracking'


def test_tracking_guard_rejects_negative_limit():
    with pytest.raises(ValueError,match='0 disables'):
        cs.Plan(max_position_error=-.01).validate()


@pytest.mark.parametrize('change, message', [
    (dict(controller='joint_impedance'),'cartesian_impedance'),
    (dict(last_error='reflex'),'healthy'),
    (dict(received_at=0.),'stale'),(dict(quat_xyzw=np.zeros(4)),'unit length')])
def test_preflight_fails_without_motion(change,message):
    clock=Clock(); robot=StubRobot(clock); robot.changes.update(change)
    with pytest.raises((ValueError,RuntimeError),match=message):
        cs.healthy(robot.state,clock(),cs.Plan())
    assert not robot.sent


@pytest.mark.parametrize('aborted',[False,True])
def test_recording_is_closed_and_fault_termination_is_unconfirmed(tmp_path,monkeypatch,aborted):
    clock=Clock(); robot=StubRobot(clock)
    p=cs.Plan(stage='hold',hold_seconds=.12,padding_hold=.02)
    metadata=dict(commands_sent=0,motion_command_attempted=False,recording_id='test',
                  nuc_log_reference='/recordings/test.csv')
    real_stream=cs.stream
    def stream(*args,**kwargs):
        if aborted:
            metadata['motion_command_attempted']=True
            robot.record_timeout=True
            raise RuntimeError('injected failure')
        return real_stream(*args,**kwargs,clock=clock,sleep=clock.sleep)
    monkeypatch.setattr(cs,'stream',stream)
    monkeypatch.setattr(cs,'time',SimpleNamespace(monotonic=clock))
    if aborted:
        with pytest.raises(RuntimeError,match='injected failure'):
            cs.run_live(robot,p,SETTINGS,tmp_path,metadata,SimpleNamespace(record=True))
        assert robot.terminated
        assert metadata['termination']=='requested_unconfirmed'
        assert 'recording_stop_unconfirmed' in metadata
    else:
        cs.run_live(robot,p,SETTINGS,tmp_path,metadata,SimpleNamespace(record=True))
        assert metadata['status']=='completed'
        assert metadata['recording_stop']['complete']
        assert not robot.terminated
    assert [call[0] for call in robot.recording_calls]==['start','stop']


def test_existing_cartesian_wire_settings_and_no_mode_switch(daemon_streaming,tmp_path,monkeypatch):
    monkeypatch.delenv('FR3_CONFIG_DIR',raising=False)
    monkeypatch.chdir(tmp_path)
    output=tmp_path/'hold'
    with daemon_streaming.publish_loop(controller='cartesian_impedance',pos=cs.PREVIEW_POS.tolist(),
                                       quat_xyzw=cs.PREVIEW_QUAT.tolist(),q=cs.common.PREVIEW_Q.tolist()):
        assert cs.main(['--execute','--host','127.0.0.1','--robot-id','test',
                        '--cmd-port',str(daemon_streaming.cmd_port),
                        '--state-port',str(daemon_streaming.state_port),
                        '--stage','hold','--hold-seconds','.12','--padding-hold','.02',
                        '--max-lateness','.2','--output',str(output)])==0
    messages=daemon_streaming.drain_commands();assert messages
    for payload in messages:
        with SCHEMA.Command.from_bytes(payload) as cmd:
            assert not cmd.termination and cmd.config.which()=='cartesianImpedance'
            cfg=cmd.config.cartesianImpedance
            assert cfg.filterAlpha==.05 and cfg.ema and cfg.linearInterp
            assert list(cfg.k)==SETTINGS['K'] and list(cfg.d)==SETTINGS['D']
            np.testing.assert_allclose(list(cfg.targetQuatXyzw),cs.PREVIEW_QUAT)
    data=json.loads((output/'metadata.json').read_text())
    assert data['commanded_controller_settings']==SETTINGS
    assert data['status']=='completed' and not data['sysid_ready']


def test_joint_daemon_rejected_before_recording_or_commands(daemon_streaming,tmp_path):
    with daemon_streaming.publish_loop(controller='joint_impedance',q=cs.common.PREVIEW_Q.tolist()):
        assert cs.main(['--execute','--record','--host','127.0.0.1','--robot-id','test',
                        '--cmd-port',str(daemon_streaming.cmd_port),
                        '--state-port',str(daemon_streaming.state_port),
                        '--stage','hold','--output',str(tmp_path/'wrong-mode')])==1
    assert not daemon_streaming.drain_commands()
    data=json.loads((tmp_path/'wrong-mode'/'metadata.json').read_text())
    assert data['commands_sent']==0 and 'recording_start' not in data
    assert 'cartesian_impedance' in data['error']


def test_bundled_cart_sysid_stiff_profile_keeps_nullspace_and_smoothing():
    settings, source = cs.resolve_settings('sysid_stiff')
    assert source.endswith('cartesian_impedance.sysid_stiff.yaml')
    # Stiff translation; rotation 20 with damping lowered to 5, because more
    # rotational damping drove J7 into 25-31 Hz oscillations on the real
    # FR3 + XHand (30/10.95 sustained, 20/9 transient at scale 0.4).
    assert settings['K'] == [500.0, 500.0, 500.0, 20.0, 20.0, 20.0]
    assert np.allclose(settings['D'][:3], 2 * np.sqrt(500.0))
    assert settings['D'][3:] == [5.0, 5.0, 5.0]
    assert settings['K_null'] > 0
    assert settings['ema'] is True and settings['linear_interp'] is True
    assert 0 < settings['filter_alpha'] < 1
