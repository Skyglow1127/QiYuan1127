"""Only loopback, domain 231 and dedicated d1_test_* topics. No hardware network."""
import subprocess
import sys

binary, peer = sys.argv[1:]
base = [binary, '--joints', 'all', '--nic', 'lo', '--domain', '231', '--topic',
        'd1_test_command', '--feedback-topic', 'd1_test_feedback', '--no-servo',
        '--timeout-ms', '700', '--motion-timeout-ms', '1000']
flag = ['--bootstrap-position-once']
cases = [
    ('bootstrap', ['--ack', 'recv', 'j', '6', '1'], 1, 0, False),
    ('bootstrap', flag + ['--ack', 'recv', 'j', '6', '1'], 0, 1, True),
    ('bootstrap', flag + ['j', '6', '1'], 0, 1, True),
    ('bootstrap', flag + ['ja', '0', '0', '0', '0', '0', '0', '1'], 2, 0, False),
    ('bootstrap', flag + ['--ack', 'recv', '--exec', 'j 6 1; j 6 2'], 0, 2, True),
    ('bootstrap', flag + ['j', '6', '0'], 2, 0, False),
    ('bootstrap', flag + ['j', '6', '0.0001'], 2, 0, False),
    ('bootstrap', flag + ['zero'], 1, 0, False),
    ('bootstrap', flag + ['hold-current', 'all'], 1, 0, False),
    ('bootstrap', flag + ['nudge', '6', '1'], 1, 0, False),
    ('bootstrap', flag + ['test-joint', '5', '1'], 1, 0, False),
    ('bootstrap', flag + ['--ack', 'recv', '--exec', 'damping all 0; j 6 1'], 1, 1, False),
    ('bootstrap', flag + ['j', '6', '50.1'], 2, 0, False),
    ('bootstrap_quantized_half', flag + ['j', '6', '6.5005'], 2, 0, False),
    ('bootstrap_initial_enabled_drop', flag + ['--ack', 'recv', '--exec', 'j 6 1; j 6 2'], 1, 1, False),
    ('bootstrap_partial_status', flag + ['j', '6', '1'], 2, 0, False),
    ('bootstrap_no_ack', flag + ['--ack', 'recv', '--exec', 'j 6 1; j 6 2'], 1, 1, False),
    ('bootstrap_no_enable', flag + ['--ack', 'recv', '--exec', 'j 6 1; j 6 2'], 1, 1, False),
    ('bootstrap_power_off', flag + ['j', '6', '1'], 2, 0, False),
    ('bootstrap_fault', flag + ['j', '6', '1'], 2, 0, False),
    ('bootstrap_invalid_baseline', flag + ['j', '6', '1'], 2, 0, False),
]
for scenario, args, expected, tx_count, confirmed in cases:
    proc = subprocess.Popen([peer, scenario], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        while True:
            line = proc.stdout.readline()
            if line.strip() == 'READY': break
            if not line: raise RuntimeError('DDS test peer failed to start')
        result = subprocess.run(base + args, capture_output=True, text=True, timeout=7)
        output = result.stdout + result.stderr
        assert result.returncode == expected, (scenario, args, result.returncode, output)
        assert output.count('[tx]') == tx_count, (scenario, args, output)
        assert ('[position-bootstrap-ok]' in output) == confirmed, (scenario, args, output)
        print('PASS', scenario, args)
    finally:
        proc.terminate()
        try: proc.wait(timeout=3)
        except subprocess.TimeoutExpired: proc.kill(); proc.wait()
print('21 isolated bootstrap DDS cases passed')
