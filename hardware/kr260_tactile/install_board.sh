#!/usr/bin/env bash
# Shipped for deliberate administrator execution on KR260. This script never loads FPGA firmware.
set -euo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="$SCRIPT_DIR/firmware/output"
DEST_ROOT='/lib/firmware/xilinx'
DEST_DIR="$DEST_ROOT/kr260-fsr"
MODE="${1:---install}"
STAGING_DIR=''

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
usage() {
  cat <<'HELP'
KR260 压力数字输入固件管理（不会自动卸载或加载 FPGA）
  ./install_board.sh --verify-package       只读校验交付固件，无需 root
  sudo ./install_board.sh --install        校验后安装依赖和固件文件（默认）
  sudo ./install_board.sh --grant-access ubuntu
                                          核实已加载设计后，仅授权对应 GPIO
激活与回退步骤见 firmware/INSTALL.txt。哈希匹配不等于实板验证通过。
HELP
}
case "$MODE" in
  --help|-h) usage; exit 0 ;;
  --verify-package|--install) [[ $# -le 1 ]] || die '此模式不接受额外参数。' ;;
  --grant-access) [[ $# -eq 2 ]] || die '--grant-access 必须指定一个已有普通用户，例如 ubuntu。' ;;
  *) usage >&2; die '未知模式；不支持自动 --activate。' ;;
esac
command -v python3 >/dev/null || die '缺少 Python 3；未进行任何安装。'

# Verify the exact build product set before apt, directory creation or device changes.
python3 - "$OUTPUT_DIR" <<'PY'
import hashlib, json, re, sys
from pathlib import Path

root = Path(sys.argv[1])
expected_files = {'kr260-fsr.bit.bin', 'kr260-fsr.dtbo', 'shell.json',
                  'fsr_top.bit', 'kr260-fsr.xsa', 'address_map.json'}
manifest_path = root / 'manifest.json'
try:
    if not root.is_dir() or not manifest_path.is_file() or manifest_path.is_symlink():
        raise ValueError('完整离线构建尚未生成：缺少 firmware/output/manifest.json')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('schema_version') != 1 or manifest.get('status') != 'OFFLINE_BUILD_PASS':
        raise ValueError('manifest 不符合本安装器要求，或离线构建未通过')
    if manifest.get('hardware_status') != 'NOT_TESTED':
        raise ValueError('本版本只接受交付时标为 NOT_TESTED 的固定构建清单，请核对版本')
    if manifest.get('revision') != 'r2_aux_reset_and_afi_20261025':
        raise ValueError('只允许已修正辅助复位和 AFI 的 r2 固件；拒绝旧版或未知版本')
    design = manifest.get('design', {})
    expected_design = {'part':'xck26-sfvc784-2LV-c', 'base_address':0xa0000000,
                       'address_range':0x10000, 'gpio_width':2, 'all_inputs':True,
                       'clock_hz':25000000, 'offsets':[0,1], 'pins':['AB14','Y14']}
    for key, expected in expected_design.items():
        if design.get(key) != expected:
            raise ValueError(f'设计元数据不匹配：{key}，需要 {expected!r}')
    if design.get('all_inputs') is not True:
        raise ValueError('all_inputs 必须明确为 true')
    artifacts = manifest.get('artifacts', {})
    if not isinstance(artifacts, dict) or set(artifacts) != expected_files:
        raise ValueError('artifact 清单必须恰好包含此版本规定的 6 个文件')
    for name in sorted(expected_files):
        digest = artifacts[name]
        p = root / name
        if not isinstance(digest, str) or not re.fullmatch(r'[0-9a-f]{64}', digest):
            raise ValueError(f'无效 SHA256：{name}')
        if not p.is_file() or p.is_symlink() or p.stat().st_size == 0:
            raise ValueError(f'文件缺失、为空或为符号链接：{name}')
        h = hashlib.sha256()
        with p.open('rb') as f:
            for block in iter(lambda: f.read(1024*1024), b''):
                h.update(block)
        if h.hexdigest() != digest:
            raise ValueError(f'SHA256 不匹配：{name}')
    shell = json.loads((root / 'shell.json').read_text(encoding='utf-8'))
    if shell != {'shell_type':'XRT_FLAT', 'num_slots':'1'}:
        raise ValueError('shell.json 与预期 XRT_FLAT 单 slot 格式不匹配')
except (OSError, ValueError, TypeError) as e:
    print(f'固件校验失败：{e}。未进行安装或硬件操作。', file=sys.stderr)
    sys.exit(1)
print('6 个固定构建文件 SHA256 与设计元数据均匹配；这是离线包校验，实机结果另见部署记录。')
PY
[[ "$MODE" != '--verify-package' ]] || exit 0

[[ $EUID -eq 0 ]] || die '此模式需要板端管理员使用 sudo 手动执行；脚本不收集密码。'
[[ "$(uname -m)" == 'aarch64' ]] || die '仅支持已核实的 KR260 aarch64 板端；请勿在电脑运行安装模式。'
python3 - <<'PY'
from pathlib import Path
import shlex, sys
try:
    model = Path('/sys/firmware/devicetree/base/model').read_bytes().rstrip(b'\0').decode()
    if 'KR260' not in model:
        raise ValueError(f'板卡型号不匹配：{model}')
    release = {}
    for line in Path('/etc/os-release').read_text().splitlines():
        if '=' in line and not line.startswith('#'):
            k,v = line.split('=', 1)
            fields = shlex.split(v)
            release[k] = fields[0] if fields else ''
    if release.get('ID') != 'ubuntu' or release.get('VERSION_ID') != '24.04':
        raise ValueError('本脚本仅核对过 Ubuntu 24.04 arm64；其他系统请先复核依赖和 overlay')
    print('目标型号：'+model+'；系统：Ubuntu 24.04 arm64。')
except (OSError, ValueError) as e:
    print(str(e), file=sys.stderr)
    sys.exit(1)
PY

RUNTIME_FILES=(kr260-fsr.bit.bin kr260-fsr.dtbo shell.json address_map.json manifest.json)
if [[ -L "$DEST_DIR" ]]; then die '固件目标目录为符号链接，拒绝安装或授权。'; fi
if [[ -e "$DEST_DIR" ]]; then
  [[ -d "$DEST_DIR" ]] || die '固件目标路径存在但不是目录。'
  for artifact in "${RUNTIME_FILES[@]}"; do
    [[ -f "$DEST_DIR/$artifact" && ! -L "$DEST_DIR/$artifact" ]] || die "已有安装不完整：$artifact；请管理员先检查，不自动覆盖。"
    cmp -s -- "$OUTPUT_DIR/$artifact" "$DEST_DIR/$artifact" || die "已有安装版本不同：$artifact；不自动覆盖，请先人工备份核对。"
  done
fi

if [[ "$MODE" == '--install' ]]; then
  command -v apt-get >/dev/null || die '缺少 apt-get。'
  command -v xmutil >/dev/null || die '缺少板端 xmutil；未执行安装。'
  command -v timeout >/dev/null || die '缺少 timeout；未执行安装。'
  LOG_DIR="/var/log/kr260-fsr-install/$(date -u +%Y%m%dT%H%M%SZ)-$$"
  install -d -m 0755 -- "$LOG_DIR"
  if ! timeout 20 xmutil listapps >"$LOG_DIR/listapps-before.txt" 2>&1; then
    die "无法只读记录当前加速器。请检查 $LOG_DIR/listapps-before.txt；未安装依赖或固件。"
  fi
  if ! python3 - "$LOG_DIR/listapps-before.txt" <<'PY'
import pathlib, sys
s = pathlib.Path(sys.argv[1]).read_text(errors='replace')
sys.exit(0 if 'Accelerator' in s and 'Accel_type' in s else 1)
PY
  then die "xmutil 未返回可识别的应用列表，请检查 $LOG_DIR/listapps-before.txt；未安装依赖或固件。"; fi
  cp -- "$OUTPUT_DIR/manifest.json" "$LOG_DIR/source-manifest.json"
  apt-get update
  apt-get install -y --no-install-recommends python3-libgpiod gpiod
  python3 -c 'import gpiod; print("Python gpiod:", gpiod.__file__)'
  if [[ ! -d "$DEST_DIR" ]]; then
    install -d -m 0755 -- "$DEST_ROOT"
    STAGING_DIR="$(mktemp -d "$DEST_ROOT/.kr260-fsr.install.XXXXXX")"
    cleanup() {
      if [[ -n "$STAGING_DIR" && "$STAGING_DIR" == "$DEST_ROOT"/.kr260-fsr.install.* ]]; then
        rm -rf -- "$STAGING_DIR"
      fi
    }
    trap cleanup EXIT
    for artifact in "${RUNTIME_FILES[@]}"; do install -m 0644 -- "$OUTPUT_DIR/$artifact" "$STAGING_DIR/$artifact"; done
    chmod 0755 -- "$STAGING_DIR"
    mv -T -- "$STAGING_DIR" "$DEST_DIR"
    STAGING_DIR=''
  fi
  printf '\n已安装依赖与固件文件：%s\n' "$DEST_DIR"
  printf '此前应用记录：%s/listapps-before.txt\n' "$LOG_DIR"
  printf '没有加载 FPGA，没有修改 GPIO 权限。下一步请按 firmware/INSTALL.txt 人工激活并验证。\n'
  exit 0
fi

[[ -d "$DEST_DIR" ]] || die '尚未安装本包；请先运行 --install。'
TARGET_USER="$2"
[[ "$TARGET_USER" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || die '用户名格式不支持。'
TARGET_UID="$(id -u -- "$TARGET_USER")" || die '指定用户不存在。'
[[ "$TARGET_UID" -ge 1000 && "$TARGET_UID" -lt 65534 ]] || die '请指定已有普通登录用户，不要使用 root 或系统账户。'

# No GPIO line request is made: inspect live DT and sysfs metadata only.
DEVICE_INFO="$(python3 - <<'PY'
from pathlib import Path
import os, re, stat, struct, sys
root=Path('/sys/firmware/devicetree/base')
expected=root/'axi/gpio@a0000000'
def text(p): return p.read_bytes().rstrip(b'\0').decode()
def cells(p):
    b=p.read_bytes()
    if len(b)%4: raise ValueError('无效 DT 整数属性：'+str(p))
    return list(struct.unpack('>'+str(len(b)//4)+'I',b))
try:
    if text(root/'fpga-region/firmware-name') != 'kr260-fsr.bit.bin':
        raise ValueError('当前 FPGA region 未标记为本固件；不授予 GPIO 权限')
    if not expected.is_dir(): raise ValueError('缺少已约定的 OF 节点 /axi/gpio@a0000000')
    if (expected/'status').exists() and text(expected/'status') not in ('okay','ok'):
        raise ValueError('目标 GPIO 节点未启用')
    compatible=(expected/'compatible').read_bytes().split(b'\0')
    if b'xlnx,xps-gpio-1.00.a' not in compatible:
        raise ValueError('目标不是预期的 Xilinx AXI GPIO')
    for key,value in [('reg',[0,0xa0000000,0,0x10000]),('xlnx,gpio-width',[2]),
                      ('xlnx,all-inputs',[1]),('xlnx,is-dual',[0]),('#gpio-cells',[2])]:
        if cells(expected/key)!=value: raise ValueError('GPIO 属性不匹配：'+key)
    if not (expected/'gpio-controller').is_file(): raise ValueError('目标不是 GPIO controller')
    if (expected/'gpio-line-names').read_bytes().rstrip(b'\0').split(b'\0') != [b'FSR_DO1',b'FSR_DO2']:
        raise ValueError('目标两路 line names 不匹配')
    matches=[]
    for chip in Path('/sys/bus/gpio/devices').glob('gpiochip*'):
        if (chip/'of_node').resolve()!=expected.resolve(): continue
        if not re.fullmatch(r'gpiochip[0-9]+',chip.name): continue
        dev=Path('/dev')/chip.name
        if not dev.exists() or not stat.S_ISCHR(dev.stat().st_mode): raise ValueError('目标字符设备不存在')
        if dev.is_symlink(): raise ValueError('目标字符设备不应为符号链接')
        major,minor=map(int,(chip/'dev').read_text().strip().split(':'))
        if dev.stat().st_rdev != os.makedev(major,minor): raise ValueError('sysfs 与字符设备不对应')
        legacy=[p for p in Path('/sys/class/gpio').glob('gpiochip*') if (p/'device').resolve()==chip.resolve().parent]
        if len(legacy)!=1 or int((legacy[0]/'ngpio').read_text())!=2:
            raise ValueError('无法从已验证 sysfs 结构确认控制器恰有 2 路')
        label=(legacy[0]/'label').read_text().strip().replace('\t',' ').replace('\n',' ')
        matches.append((str(dev),label))
    if len(matches)!=1: raise ValueError('找不到唯一匹配的已加载压力 GPIO 字符设备')
    print(matches[0][0]+'\t'+matches[0][1])
except (OSError, ValueError, UnicodeError) as e:
    print('GPIO 授权前核验失败：'+str(e),file=sys.stderr)
    sys.exit(1)
PY
)"
IFS=$'\t' read -r GPIO_DEVICE CHIP_LABEL <<<"$DEVICE_INFO"
[[ "$GPIO_DEVICE" =~ ^/dev/gpiochip[0-9]+$ ]] || die 'GPIO 核验没有返回有效设备。'
getent group pressure-gpio >/dev/null || groupadd --system pressure-gpio
usermod -aG pressure-gpio -- "$TARGET_USER"
chgrp pressure-gpio -- "$GPIO_DEVICE"
chmod 0660 -- "$GPIO_DEVICE"
printf '已仅授权 %s（内核 label=%s），权限 root:pressure-gpio 0660。\n' "$GPIO_DEVICE" "$CHIP_LABEL"
printf '用户 %s 需重新登录 SSH 才能取得新附加组。控制器重新创建/重启后需再次运行 --grant-access。\n' "$TARGET_USER"
printf '未申请 GPIO line、未采集传感器、未修改其他 GPIO、未安装 udev 或 sudoers 规则。\n'
