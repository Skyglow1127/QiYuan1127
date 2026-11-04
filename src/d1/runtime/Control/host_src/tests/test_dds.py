"""显式运行的 DDS 集成测试，只在 lo/domain 231/测试话题上通信。"""
import subprocess
import sys

binary, peer = sys.argv[1:]
base = [binary, '--joints','all', '--nic','lo','--domain','231','--topic','d1_test_command',
        '--feedback-topic','d1_test_feedback','--no-servo','--timeout-ms','1500']
cases = [
    ('j4_bias', ['--j4-bias-deg','0.4','j','4','10'],0),
    ('success', ['--j4-bias-deg','0.4','--motion-timeout-ms','1200','j','4','10'],1),
    ('invalid_baseline', ['--joints','4,5,6','test-joint','5','-1'],2),
    ('single_enable', ['--joints','4,5,6','--exec','damping 5 1; test-joint 5 1'],1),
    ('success', ['--exec','damping 5 0; j 5 1'],1),
    ('success', ['--exec','damping 5 1000; j 5 1'],1),
    ('success', ['--exec','damping all 1000; ja 0 0 0 0 0 0 0'],1),
    ('hold_check', ['hold-current-check','all'],0),
    ('hold_check', ['hold-current','all'],1),
    ('reconnect_out_of_range', ['hold-current','all'],2),
    ('hold_gripper_clamp', ['hold-current-check','5'],2),
    ('hold_moving', ['hold-current-check','all'],2),
    ('fault', ['hold-current-check','5'],2),
    ('fault', ['j','5','1'],1),
    ('success', ['--joints','4,5,6','test-joint','5','1'],0),
    ('success', ['--exec','status; j 5 1; power off'],0),
    ('reject', ['--exec','j 5 1; j 5 2'],1),
    ('received_only', ['--ack','exec','j','5','1'],1),
    ('received_only', ['--ack','recv','j','5','1'],0),
    ('received_only', ['j','5','1'],0),
    ('stalled', ['--motion-timeout-ms','1200','j','5','1'],1),
    ('protected_drift', ['--joints','4,5,6','--exec','j 5 1; j 5 2'],1),
    ('state_mismatch', ['power','off'],1),
    ('no_feedback', ['j','5','1'],1),
]
for scenario, args, expected in cases:
    proc = subprocess.Popen([peer,scenario],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    try:
        while True:
            line = proc.stdout.readline()
            if line.strip() == 'READY': break
            if not line: raise RuntimeError('DDS peer did not start')
        result = subprocess.run(base+args,capture_output=True,text=True,timeout=10)
        output = result.stdout+result.stderr
        assert result.returncode == expected, (scenario,result.returncode,output)
        if scenario in ('reject','protected_drift','single_enable') or 'damping' in ' '.join(args):
            assert output.count('[tx]') == 1, output
        if scenario in ('no_feedback','invalid_baseline','reconnect_out_of_range','hold_check','hold_gripper_clamp','hold_moving','fault'):
            assert '[tx]' not in output, output
        if scenario.startswith('hold_') or scenario == 'reconnect_out_of_range':
            assert '无控制 publisher' in output, output
        if scenario == 'received_only' and expected == 1: assert 'recv=1 exec=未知' in output, output
        print('PASS',scenario,args)
    finally:
        proc.terminate()
        try: proc.wait(timeout=3)
        except subprocess.TimeoutExpired: proc.kill(); proc.wait()
print('All DDS integration cases passed')
