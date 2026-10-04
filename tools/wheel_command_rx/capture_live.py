#!/usr/bin/env python3
"""Record diagnostics and RPM around the existing receive-only publisher suite.
Does not manage Agent/firmware. Stops only its own publisher on invariant violation.
"""
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Int16MultiArray

ROOT = Path(__file__).resolve().parents[2]


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '0':
        raise SystemExit('Explicit ROS_DOMAIN_ID=0 required')
    run = ROOT / 'wheel_command_checks' / ('live_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    run.mkdir()
    print('ARTIFACT_DIR=' + str(run), flush=True)
    rclpy.init(args=[])
    node = rclpy.create_node('wheel_rx_capture')
    start = time.monotonic()
    stats = {'rpm_messages': 0, 'diagnostic_messages': 0, 'nonzero_rpm': 0}
    latest = {}
    last_rpm = None
    bad = []
    proc = None
    diag = (run / 'diagnostics.jsonl').open('w')
    rpm = (run / 'rpm.jsonl').open('w')
    output = (run / 'publisher_stdout.log').open('w')
    protected = [ROOT/'wheel_command_checks/laptop_pico_b_node_app.cpp', ROOT/'wheel_command_checks/laptop_firmware_README.md', ROOT/'src/lessien_interfaces/msg/WheelCommand.msg', ROOT/'tools/wheel_command_rx/publisher.py']
    before = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in protected}
    meta = {'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(), 'domain':0, 'hashes_before':before, 'scope':'receive-only hardware test; not driving validation', 'conditional_continuation':'User explicitly approved continuing with nonzero measured RPM logged as separate FAIL; commandRpm and freshness guards retained'}
    def fail(reason):
        if reason not in bad:
            bad.append(reason)
            print('INVARIANT: ' + reason, flush=True)
    def rpm_cb(msg):
        nonlocal last_rpm
        now=time.monotonic_ns(); last_rpm=time.monotonic()
        values=list(msg.data)
        rpm.write(json.dumps({'monotonic_ns':now,'values':values})+'\n');rpm.flush()
        stats['rpm_messages']+=1
        if len(values)!=4:
            fail('RPM vector length is not four')
        if any(v!=0 for v in values):
            stats['nonzero_rpm']+=1
            # User explicitly authorized conditional continuation after baseline -1.
            # Preserve raw values and report strict all-zero RPM as FAIL.
            if stats['nonzero_rpm'] == 1:
                print('OBSERVATION: nonzero measured RPM '+repr(values), flush=True)
    def diag_cb(msg):
        now=time.monotonic_ns()
        statuses=[]
        for status in msg.status:
            v={kv.key:kv.value for kv in status.values}
            statuses.append({'name':status.name,'level':int.from_bytes(status.level,'little') if isinstance(status.level,bytes) else int(status.level),'message':status.message,'values':v})
            latest[status.name]={'monotonic_ns':now,'message':status.message,'values':v}
            if status.name=='pico_b/wheel_freshness' and v.get('sourceTrusted')!='false':
                fail('sourceTrusted is not false')
            if status.name=='pico_b/wheel_command' and v.get('commandFresh')!='false':
                fail('commandFresh is not false')
            if status.name=='pico_b/motors' and v.get('commandRpm')!='0':
                fail('commandRpm is not zero')
        diag.write(json.dumps({'monotonic_ns':now,'stamp_sec':msg.header.stamp.sec,'stamp_nanosec':msg.header.stamp.nanosec,'statuses':statuses})+'\n');diag.flush()
        stats['diagnostic_messages']+=1
    subs=[node.create_subscription(Int16MultiArray,'/pico_b/motors/rpm',rpm_cb,qos_profile_sensor_data),node.create_subscription(DiagnosticArray,'/diagnostics',diag_cb,qos_profile_sensor_data)]
    def spin_for(seconds, monitor=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.05)
            if monitor and (last_rpm is None or time.monotonic()-last_rpm>2):
                fail('RPM receive gap greater than 2 s')
            if monitor and bad:
                break
    try:
        print('Capturing 30 s baseline; no wheel publication yet.',flush=True)
        spin_for(30)
        meta['baseline_latest']=dict(latest)
        for name in ['pico_b/wheel_command','pico_b/wheel_freshness','pico_b/motors']:
            if name not in latest: fail('Missing baseline diagnostics: '+name)
        if last_rpm is None or time.monotonic()-last_rpm>2:fail('No fresh baseline RPM')
        if bad: raise RuntimeError('Baseline invariants failed')
        # Freshly uploaded image should have no accepted wheel command; do not reset Agent.
        wc=latest['pico_b/wheel_command']['values']
        wf=latest['pico_b/wheel_freshness']['values']
        if int(wf['acceptedCount']) != 0:
            raise RuntimeError('Existing accepted commands: inspect session/sequence before starting, do not guess')
        cmd=[sys.executable,'tools/wheel_command_rx/publisher.py','--publish','--case','suite','--protocol-version','1','--session-id','2026100101','--start-sequence','100','--values','0.10','0.20','0.30','0.40','--seconds','20','--stale-seconds','20','--firmware-contract','wheel_command_checks/laptop_firmware_README.md','--log',str(run/'publisher.jsonl')]
        meta['command']=cmd
        meta['publisher_start_monotonic_ns']=time.monotonic_ns()
        proc=subprocess.Popen(cmd,cwd=ROOT,stdout=output,stderr=subprocess.STDOUT)
        last_status=0
        while proc.poll() is None:
            spin_for(0.2,monitor=True)
            if bad:
                proc.send_signal(signal.SIGINT)
                break
            if time.monotonic()-last_status>=30:
                last_status=time.monotonic()
                print(json.dumps({'elapsed_s':round(time.monotonic()-start,1),**stats,'freshness':latest.get('pico_b/wheel_freshness')}),flush=True)
        if proc.poll() is None:
            proc.wait(timeout=10)
        meta['publisher_exit_code']=proc.returncode
        print('Publisher finished; capturing 20 s post-test diagnostics/RPM.',flush=True)
        spin_for(20)
        if proc.returncode!=0:raise RuntimeError('Publisher exited nonzero; inspect publisher_stdout.log')
        if bad:raise RuntimeError('Invariant violation; suite interrupted')
        meta['execution']='COMPLETE; analysis pending'
    except Exception as exc:
        meta['execution']='INCOMPLETE'
        meta['error']=repr(exc)
        print(repr(exc),flush=True)
    finally:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=10)
        meta.update(stats)
        meta['violations']=bad
        meta['finished_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        meta['duration_s']=time.monotonic()-start
        meta['final_latest']=latest
        meta['hashes_after']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in protected}
        (run/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
        diag.close();rpm.close();output.close()
        node.destroy_node();rclpy.shutdown()
        print(json.dumps({'directory':str(run),'execution':meta['execution'],**stats,'violations':bad}),flush=True)


if __name__=='__main__':
    main()
