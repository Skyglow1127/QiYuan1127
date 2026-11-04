#!/usr/bin/env bash
# This script never writes MMIO, CMD, ARM, CAN or robot interfaces.
set -euo pipefail
script_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
build_dir="$script_dir/../build/kr260_desc_only"
mode=check
read_status=0
manager=/sys/class/fpga_manager/fpga0
firmware_dir=/lib/firmware
usage() {
    cat <<'EOF'
Usage: load.sh [--check | --load] [--build DIR] [--read-status]
               [--manager DIR] [--firmware-dir DIR] [--help]

Default --check: verify build hashes/address contract and read manager state only.
--load: explicitly install the validated .bit.bin and program fpga_manager.
--read-status: additionally open /dev/mem READ ONLY for ID/status/counters.
Loading does not initialize the PS: first use a matching KR260 Linux/PS platform
with HPM0 enabled, matching addresses.json nominal PL0 clock, and reset released.
Use clock_hz and requested_clock_hz from this build's addresses.json.
See docs/BOARD_SMOKE.md.
No ARM/configuration/chunk-memory writes occur. No CAN hardware is connected.

Examples (run manually on KR260 after copying an actual successful build):
  bash robot_rt/sw/load.sh --check --build /path/to/kr260_desc_only
  sudo bash robot_rt/sw/load.sh --load --build /path/to/kr260_desc_only --read-status
EOF
}
while (($#)); do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --check) mode=check; shift ;;
        --load) mode=load; shift ;;
        --read-status) read_status=1; shift ;;
        --build|--manager|--firmware-dir)
            key=$1
            (($# >= 2)) || { echo "$1 needs a value" >&2; exit 2; }
            case "$key" in --build) build_dir=$2;; --manager) manager=$2;; --firmware-dir) firmware_dir=$2;; esac
            shift 2 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done
python3 - "$build_dir" "$manager" "$mode" "$firmware_dir" "$read_status" <<'PY'
import hashlib, json, mmap, os, pathlib, struct, sys, time
build, manager, mode, firmware, read_status = sys.argv[1:]
build, manager, firmware = map(pathlib.Path, (build, manager, firmware))

def fail(message):
    raise SystemExit('ERROR: ' + message)

try:
    manifest = json.loads((build/'artifacts.sha256.json').read_text())
    addresses = json.loads((build/'addresses.json').read_text())
except (OSError, ValueError) as exc:
    fail(f'No verified build manifest/address file: {exc}')
if manifest.get('schema') != 'robot_rt.build.v1' or manifest.get('timing_passed') is not True:
    fail('Build manifest does not record successful timing signoff')
required = {'robot_rt.bit', 'robot_rt.bit.bin', 'robot_rt.xsa', 'addresses.json'}
if not required <= manifest.get('files', {}).keys():
    fail('Build manifest is missing required artifacts')
for name in sorted(required):
    path = build/name
    try:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as exc:
        fail(str(exc))
    if digest != manifest['files'][name]:
        fail(f'Artifact hash mismatch: {path}')
expected = dict(schema='robot_rt.addresses.v1', board_part='xilinx.com:kr260_som:part0:1.1',
                part='xck26-sfvc784-2LV-c', backend='DESC_ONLY',
                cfg_base=0xA0000000, cfg_bytes=4096,
                chunk_base=0xA0020000, chunk_bytes=131072, expected_id=0x5254504c)
for key, value in expected.items():
    if addresses.get(key) != value:
        fail(f'Address/design contract mismatch for {key}')
clock_hz = addresses.get('clock_hz')
if type(clock_hz) is not int or not 1 <= clock_hz <= 2_147_483_647:
    fail('Invalid compiled nominal CLOCK_HZ in address contract')
print(json.dumps({'artifacts_verified': True, 'addresses': addresses, 'mode': mode}, indent=2))
state_file = manager/'state'
state = state_file.read_text().strip() if state_file.is_file() else 'unavailable'
print('fpga_manager state:', state)
model = pathlib.Path('/proc/device-tree/model')
if model.is_file():
    print('device-tree model:', model.read_bytes().rstrip(b'\0').decode(errors='replace'))
if mode == 'load':
    if os.geteuid() != 0:
        fail('--load needs root permission on the target board; no automatic sudo')
    if not (manager/'firmware').is_file() or not (manager/'flags').is_file():
        fail('Target fpga_manager does not expose flags/firmware; no fallback programming method')
    # Filename is fixed by a hash, not arbitrary sysfs input or a shell command.
    raw = (build/'robot_rt.bit.bin').read_bytes()
    target_name = 'robot_rt-' + hashlib.sha256(raw).hexdigest()[:16] + '.bit.bin'
    firmware.mkdir(parents=True, exist_ok=True)
    target = firmware/target_name
    if target.exists() and target.read_bytes() != raw:
        fail(f'Existing firmware filename collision: {target}')
    with target.open('wb') as handle:
        handle.write(raw)
        handle.flush()
        os.fsync(handle.fileno())
    # Full bitstream only. No device tree, PS initialization, bridge or CAN writes.
    (manager/'flags').write_text('0\n')
    (manager/'firmware').write_text(target_name+'\n')
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        state = state_file.read_text().strip()
        if state == 'operating':
            break
        if 'error' in state:
            fail('FPGA manager failed: ' + state)
        time.sleep(0.05)
    if state != 'operating':
        fail('FPGA manager did not reach operating within 10s: ' + state)
    print('Loaded PL bitstream; this does not initialize the PS or verify CAN timing.')
if read_status == '1':
    if state != 'operating':
        fail('--read-status requires an operating manager and a matching PS platform')
    # An operating manager alone cannot prove address mapping. The operator must
    # use the matching generated platform described in BOARD_SMOKE before this read.
    fd = os.open('/dev/mem', os.O_RDONLY | os.O_SYNC)
    try:
        with mmap.mmap(fd, 4096, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ,
                       offset=addresses['cfg_base']) as regs:
            def u32(off): return struct.unpack_from('<I', regs, off)[0]
            def u64_live(off):
                for _ in range(16):
                    hi = u32(off+4); lo = u32(off); hi2 = u32(off+4)
                    if hi == hi2: return (hi << 32) | lo
                fail('Unstable 64-bit live counter read')
            ident = u32(0)
            if ident != addresses['expected_id']:
                fail(f'ID mismatch at generated cfg base: 0x{ident:08x}')
            caps = u32(8)
            if not caps & 1:
                fail('DESC_ONLY capability missing')
            compiled_clock = u64_live(0xa8)
            if compiled_clock != clock_hz:
                fail(f'Nominal compiled clock mismatch: registers={compiled_clock}, manifest={clock_hz}')
            live0 = u64_live(0x20)
            time.sleep(0.05)
            live1 = u64_live(0x20)
            result = dict(ID=hex(ident), VERSION=hex(u32(4)), CAPS=hex(caps),
                          STATE=u32(0x0c), STATUS=hex(u32(0x14)),
                          FAULT_CODE=u32(0x18), REJECT_CODE=u32(0x1c),
                          CYCLE=live1, CYCLE_ADVANCED=live1 > live0, NOMINAL_CLOCK_HZ=compiled_clock,
                          TICK_COUNT=u64_live(0x28),
                          SNAP_EMIT_COUNT=u64_live(0x510),
                          SNAP_REJECT_COUNT=u64_live(0x518),
                          SNAP_STARVE_COUNT=u64_live(0x528),
                          SNAP_BACKPRESSURE_COUNT=u64_live(0x530))
            result['counter_note'] = 'SNAP_* are prior snapshots; this script never writes SNAPSHOT/CMD'
            result['last_descriptor_words'] = [hex(u32(0x600+4*i)) for i in range(24)]
            result['descriptor_note'] = 'RO prior SNAPSHOT copy, possibly zero/stale; this script never triggers SNAPSHOT; not an execution FIFO'
            print(json.dumps(result, indent=2))
            if not result['CYCLE_ADVANCED']:
                fail('Live cycle counter did not advance; check PS clock/reset/platform')
    finally:
        os.close(fd)
PY
