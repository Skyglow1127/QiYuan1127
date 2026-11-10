#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
board="${D1_HANDEYE_BOARD:-$root/../D1/Control/calibration/kr260/board.a4_chessboard.json}"
mapping="$root/calibration/d1_joint_mapping.json"

usage() {
  echo "用法："
  echo "  ./run_handeye.sh doctor"
  echo "  ./run_handeye.sh capture <样本名>"
  echo "  ./run_handeye.sh capture-kr <样本名>  # KR260当前预览，彩色图+关节停稳配对"
  echo "  ./run_handeye.sh export-kr <samples.yml> <样本目录...>"
  echo "  ./run_handeye.sh export <samples.yml> <样本目录...>"
  echo "  ./run_handeye.sh solve <samples.yml> <handeye.yml>"
  echo "  ./run_handeye.sh check-board <彩色图片> <新输出目录>"
  echo ""
  echo "首次使用前："
  echo "  当前棋盘配置：$board（默认6x8 内角点；格距与验证状态见配置）"
  echo "  关节映射配置：$mapping（已有文件不要用模板覆盖）"
  echo "  D1_HANDEYE_BOARD 可指定另一份配置；原 ChArUco 文件保留。"
  echo "实测尺寸、关节映射及证据未验证时，doctor/export 会拒绝继续。"
}

if [[ $# -lt 1 ]]; then usage; exit 2; fi
mode="$1"; shift
case "$mode" in
  capture-kr)
    [[ $# -eq 1 && "$1" =~ ^[A-Za-z0-9_-]+$ ]] || { usage; exit 2; }
    exec python3 "$root/../D1/Control/tools/kr260_handeye_live.py" capture \
      --board "$board" --nic "${D1_NIC:-enp99s0}" \
      --url "${KR260_PREVIEW_URL:-http://127.0.0.1:18760}" \
      --output "$root/output/handeye_live_sessions/$1"
    ;;
  export-kr)
    [[ $# -ge 9 ]] || { echo "至少传入输出文件和 8 个样本目录" >&2; exit 2; }
    destination="$1"; shift
    exec python3 "$root/../D1/Control/tools/kr260_handeye_live.py" export \
      --board "$board" --mapping "$mapping" --output "$destination" "$@"
    ;;
  check-board)
    [[ $# -eq 2 ]] || { usage; exit 2; }
    exec python3 "$root/../D1/Control/tools/check_handeye_board.py" \
      --board "$board" --image "$1" --output "$2"
    ;;
  doctor)
    exec python3 "$root/scripts/d1_handeye_aipc.py" --nic enp99s0 doctor --board "$board" --mapping "$mapping" \
      --output "$root/output/handeye_doctor.json"
    ;;
  capture)
    [[ $# -eq 1 && "$1" =~ ^[A-Za-z0-9_-]+$ ]] || { usage; exit 2; }
    exec python3 "$root/scripts/d1_handeye_aipc.py" --nic enp99s0 capture \
      --board "$board" --config "$root/configs/aipc_usb_motion.yml" \
      --output "$root/output/handeye_sessions/$1"
    ;;
  export)
    [[ $# -ge 9 ]] || { echo "至少传入输出文件和 8 个样本目录" >&2; exit 2; }
    destination="$1"; shift
    exec python3 "$root/scripts/d1_handeye_aipc.py" export --board "$board" \
      --mapping "$mapping" --output "$destination" "$@"
    ;;
  solve)
    [[ $# -eq 2 ]] || { usage; exit 2; }
    exec "$root/build/handeye_calibrate" "$1" "$2"
    ;;
  *) usage; exit 2 ;;
esac
