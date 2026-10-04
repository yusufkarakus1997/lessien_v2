#!/usr/bin/env python3
"""Correlate receive-only publisher phases with timestamped diagnostics/RPM."""
import collections
import json
from pathlib import Path
import sys


def load(path):
    return [json.loads(x) for x in path.read_text().splitlines() if x.strip()]


def main():
    run=Path(sys.argv[1]); pub=load(run/'publisher.jsonl'); diag=load(run/'diagnostics.jsonl'); rpm=load(run/'rpm.jsonl')
    ds=[dict(s,t=x['monotonic_ns']) for x in diag for s in x['statuses']]
    starts=[x for x in pub if x.get('event')=='phase_start']
    sent=[x for x in pub if x.get('sent')]
    final_t=rpm[-1]['monotonic_ns']
    report={'scope':'receive-only; sparse diagnostics; not hardware driving PASS','phases':[]}
    def samples(name,a,b):return [s for s in ds if s['name']==name and a<=s['t']<b]
    for i,start in enumerate(starts):
        a=start['monotonic_ns'];b=starts[i+1]['monotonic_ns'] if i+1<len(starts) else final_t
        kind=start['phase'];f=samples('pico_b/wheel_freshness',a+500_000_000,b-100_000_000);w=samples('pico_b/wheel_command',a+500_000_000,b-100_000_000)
        earlier=[s for s in ds if s['name']=='pico_b/wheel_freshness' and s['t']<a]
        previous=earlier[-1] if earlier else None
        phase_sent=[s for s in sent if a<=s['monotonic_ns']<b]
        phase_rpm=[s for s in rpm if a<=s['monotonic_ns']<b]
        expected_reason={'normal':'TIME_UNSYNCED','stale':'LOCAL_TIMEOUT','duplicate':'SEQUENCE','old':'SEQUENCE','nan':'NONFINITE','inf':'NONFINITE','neg-inf':'NONFINITE'}[kind]
        expected_values=[0.1,0.2,0.3,0.4] if kind=='normal' else [0.0]*4
        checks={
            'freshness_observed':bool(f),'wheel_values_observed':bool(w),
            'reason':bool(f) and all(s['message']==expected_reason for s in f),
            'local_fresh':bool(f) and all(s['values']['localFresh']==str(kind=='normal').lower() for s in f),
            'source_untrusted':bool(f) and all(s['values']['sourceTrusted']=='false' for s in f),
            'combined_not_fresh':bool(w) and all(s['values']['commandFresh']=='false' for s in w),
            'wheel_values':bool(w) and all(all(abs(float(s['values'][k])-v)<1e-8 for k,v in zip(['flRadS','rlRadS','frRadS','rrRadS'],expected_values)) for s in w),
            'rpm_recorded':bool(phase_rpm),
        }
        if kind=='normal':
            checks['local_age_below_250']=bool(f) and all(int(s['values']['localAgeMs'])<250 for s in f)
            checks['accepted_increased']=bool(f) and previous is not None and int(f[-1]['values']['acceptedCount'])>int(previous['values']['acceptedCount'])
        elif kind=='stale':
            checks['no_messages_sent']=not phase_sent
            checks['local_age_at_least_250']=bool(f) and all(int(s['values']['localAgeMs'])>=250 for s in f)
        else:
            checks['reject_increased']=bool(f) and previous is not None and int(f[-1]['values']['rejectCount'])>int(previous['values']['rejectCount'])
        rate=(len(phase_sent)-1)*1e9/(phase_sent[-1]['monotonic_ns']-phase_sent[0]['monotonic_ns']) if len(phase_sent)>1 else None
        if kind=='normal':checks['publish_rate_20hz']=rate is not None and 19.8<=rate<=20.2
        report['phases'].append({'index':i,'phase':kind,'wheel_index':start['wheel_index'],'result':'PASS' if all(checks.values()) else 'FAIL_OR_MISSING_EVIDENCE','checks':checks,'publish_samples':len(phase_sent),'publish_rate_hz':rate,'freshness_samples':len(f),'wheel_samples':len(w),'rpm_samples':len(phase_rpm),'rpm_nonzero_samples':sum(any(v!=0 for v in s['values']) for s in phase_rpm),'freshness_observations':f,'wheel_observations':w})
    motor=[s for s in ds if s['name']=='pico_b/motors'];wf=[s for s in ds if s['name']=='pico_b/wheel_freshness'];wc=[s for s in ds if s['name']=='pico_b/wheel_command'];micro=[s for s in ds if s['name']=='pico_b/micro_ros']
    report['totals']={'phase_count':len(starts),'sent_messages':len(sent),'diagnostic_messages':len(diag),'rpm_samples':len(rpm),'rpm_rate_hz':(len(rpm)-1)*1e9/(rpm[-1]['monotonic_ns']-rpm[0]['monotonic_ns']),'rpm_max_gap_ms':max((y['monotonic_ns']-x['monotonic_ns'])/1e6 for x,y in zip(rpm,rpm[1:])),'rpm_counts_by_motor':[dict(collections.Counter(s['values'][i] for s in rpm)) for i in range(4)],'rpm_all_zero':all(len(s['values'])==4 and not any(s['values']) for s in rpm),'motor_summary_samples':len(motor),'command_rpm_values':sorted({s['values']['commandRpm'] for s in motor}),'source_trusted_values':sorted({s['values']['sourceTrusted'] for s in wf}),'command_fresh_values':sorted({s['values']['commandFresh'] for s in wc}),'final_freshness':wf[-1] if wf else None,'online_masks':sorted({s['values']['onlineMask'] for s in motor}),'fault_masks':sorted({s['values']['faultMask'] for s in motor}),'micro_ros_first':micro[0] if micro else None,'micro_ros_last':micro[-1] if micro else None}
    report['receive_only_result']='PASS' if len(starts)==32 and all(p['result']=='PASS' for p in report['phases']) and report['totals']['command_rpm_values']==['0'] and report['totals']['source_trusted_values']==['false'] and report['totals']['command_fresh_values']==['false'] else 'FAIL_OR_INCOMPLETE'
    (run/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'receive_only_result':report['receive_only_result'],'totals':report['totals'],'phase_results':[{k:p[k] for k in ['index','phase','wheel_index','result','freshness_samples']} for p in report['phases']]},indent=2))


if __name__=='__main__':main()
