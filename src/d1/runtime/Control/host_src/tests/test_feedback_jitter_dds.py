"""Only lo/domain231/d1_test_*: authorized feedback jitter, strict command bounds."""
import subprocess
import sys
binary, peer = sys.argv[1:]
base = [binary, '--joints', 'all', '--nic', 'lo', '--domain', '231', '--topic',
        'd1_test_command', '--feedback-topic', 'd1_test_feedback', '--no-servo',
        '--ack', 'recv', '--timeout-ms', '700']
flag = ['--bootstrap-position-once']
cases = [
    ('bootstrap_jitter', flag + ['j','6','1'], 0, 1),
    ('bootstrap_jitter_boundary', flag + ['j','6','1'], 0, 1),
    ('bootstrap_jitter_excess', flag + ['j','6','1'], 2, 0),
    ('normal_jitter', ['j','6','1'], 0, 1),
    ('normal_jitter_boundary', ['j','6','1'], 0, 1),
    ('normal_jitter_excess', ['j','6','1'], 2, 0),
    ('normal_postsend_jitter', ['j','6','1'], 0, 1),
    ('normal_postsend_excess', ['--exec','j 6 1; j 6 2'], 2, 1),
    ('normal_j6_excess', ['j','5','1'], 2, 0),
    ('bootstrap_jitter', flag + ['j','1','-90.1'], 2, 0),
]
for scenario, args, expected, tx_count in cases:
    proc=subprocess.Popen([peer,scenario],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    try:
        while True:
            line=proc.stdout.readline()
            if line.strip()=='READY':break
            if not line:raise RuntimeError('DDS peer failed to start')
        result=subprocess.run(base+args,capture_output=True,text=True,timeout=7)
        output=result.stdout+result.stderr
        assert result.returncode==expected,(scenario,result.returncode,output)
        assert output.count('[tx]')==tx_count,(scenario,output)
        if 'excess' in scenario:
            assert '反馈抖动容差=' in output,(scenario,output)
        print('PASS',scenario,args)
    finally:
        proc.terminate()
        try:proc.wait(timeout=3)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
print('10 isolated authorized-feedback-jitter DDS cases passed')
