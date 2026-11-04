import json
import subprocess
import sys

binary = sys.argv[1]
def run(*args, code=0, stdin=None):
    p = subprocess.run([binary, "--joints", "all", *args], input=stdin, capture_output=True, text=True, timeout=5)
    assert p.returncode == code, (args, p.returncode, p.stdout, p.stderr)
    return p.stdout + p.stderr

run('--help')
run('limits')
for args in [('--nic',), ('--domain','abc'), ('--timeout-ms','0'), ('--exec',' '),
             ('--ack','bad'), ('--exec','help','status'), ('--unknown',)]:
    run(*args, code=2)
for command in ['j 5 nan','j 5 inf','j 5 1x','j -1 0','j 5 136','j 5 0 -1',
                'j 5 0 extra','power typo','enable 2 extra','enable','enable 5','release','release 5',
                'damping all -1','damping 7 1','hold-current 7','grip -1','unknown',
                'ja 0 0 0 0 0 0 0 7','sleep -1']:
    out = run('--dry-run','--exec','power on; '+command,code=2)
    assert '[tx]' not in out, 'Entire batch must validate before any transmission'
out = run('--dry-run','--nic','nonexistent','--exec','power on; damping all 1; j 5 -3.5; ja 0 -60 60 0 30 0 0; zero')
packets = [json.loads(line[5:]) for line in out.splitlines() if line.startswith('[tx] ')]
assert [x['funcode'] for x in packets] == [6,5,1,2,7]
assert len(set(x['seq'] for x in packets)) == 5
assert packets[2]['data'] == {'id':5,'angle':-3.5,'delay_ms':0}
assert 'data' not in packets[4]
assert '[DDS]' not in out and '[ok]' not in out
for command in ['hold-current all', 'hold-current-check all']:
    out = run('--dry-run', '--exec', command, code=1)
    assert '[tx]' not in out and '[DDS' not in out
    out = run('--dry-run', '--j4-bias-deg', '0', '--exec', command, code=2)
    assert '[tx]' not in out and '[DDS' not in out
for command in ['enable', 'enable 5', 'release', 'release 5']:
    # Rejected before DDS initialization even when --dry-run is absent.
    out = run('--nic', 'nonexistent', '--exec', 'power on; ' + command, code=2)
    assert '[tx]' not in out and '[DDS' not in out
out = run('--dry-run', 'damping', '5', '1000')
packet = [json.loads(line[5:]) for line in out.splitlines() if line.startswith('[tx] ')][0]
assert packet['funcode'] == 4 and packet['data'] == {'id': 5, 'mode': 1000}
out = run('--dry-run','--exec','nudge 5 1; j 5 0',code=1)
assert '[tx]' not in out, 'Runtime failure must stop the batch'
run('--dry-run', 'j', '5', '-1')
run('--dry-run', stdin='j 5 0\nquit\n')
out = run('--dry-run',stdin='bad\nj 5 0\n',code=2)
assert '[tx]' not in out
run('--dry-run','status',code=1)
print('Terminal CLI tests passed')

# Ctrl+C 必须能打断等待和终端输入，不得因 joinable 线程而 terminate。
import os
import pty
import select
import signal
import time
master, slave = pty.openpty()
p = subprocess.Popen([binary,'--dry-run'],stdin=slave,stdout=slave,stderr=slave)
os.close(slave)
try:
    collected = b''
    end = time.monotonic() + 3
    while b'arm> ' not in collected and time.monotonic() < end:
        if select.select([master],[],[],0.1)[0]: collected += os.read(master,4096)
    assert b'arm> ' in collected, collected
    p.send_signal(signal.SIGINT)
    assert p.wait(timeout=2) == 130
finally:
    if p.poll() is None: p.kill(); p.wait()
    os.close(master)
print('Interactive Ctrl+C test passed')

for command in ['j 0 0','test-joint 0 1','nudge 1 1','damping 2 1','damping 3 0','zero','power on','damping all 1','hold-current-check all','hold-current 2','ja 0 0 0 0 0 0 0']:
    out = run('--joints','4,5,6','--dry-run','--exec','j 5 0; '+command,code=2)
    assert '[tx]' not in out, 'Protected joint scope must validate the entire batch'
run('--joints','4,5,6','--dry-run','j','4','1')
run('--dry-run','j','5','1','32768',code=2)
print('Protected joint scope tests passed')

run('--dry-run','--seq-start','9','--exec','j 5 0; j 5 0; j 5 0')
run('--dry-run','test-joint','5','0.1',code=2)

out = run('--dry-run','--j4-bias-deg','0.4','--exec',
          'j 4 10; j 5 10; ja 0 0 0 0 10 0 0; zero')
packets = [json.loads(line[5:]) for line in out.splitlines() if line.startswith('[tx] ')]
assert packets[0]['data']['angle'] == 9.6
assert packets[1]['data']['angle'] == 10
assert packets[2]['data']['angle4'] == 9.6
assert 'data' not in packets[3]
for target in ['-85','85.1','-90']:
    out = run('--dry-run','--j4-bias-deg','0.4','--exec','power on; j 4 '+target,code=2)
    assert '[tx]' not in out and '[DDS]' not in out
for bias in ['nan','1.1','-1.1']:
    run('--dry-run','--j4-bias-deg',bias,'j','4','0',code=2)
run('--dry-run','--j4-bias-deg','0.4','j','4','-84.6')
out = run('--dry-run','j','4','10')
assert '"angle":10,' in out, 'Compensation must be explicitly enabled'
