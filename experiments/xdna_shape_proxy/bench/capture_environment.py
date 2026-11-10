#!/usr/bin/env python3
"""Read-only P0 capture; never creates an ORT session or runs an NPU load.

Run through run_sdk.sh. --fingerprint emits {"environment": {...}} for bench use.
Unknown values remain JSON null and carry an explanation in unknown_fields.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time
from zoneinfo import ZoneInfo


ROOT = Path(__file__).resolve().parents[1]


def timestamp():
    return datetime.now(ZoneInfo("Asia/Singapore")).isoformat(timespec="milliseconds")


def capture():
    records = []

    def run(name, argv, timeout=30):
        start, tick = timestamp(), time.monotonic()
        try:
            proc = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
            out, err, rc = proc.stdout, proc.stderr, proc.returncode
        except (OSError, subprocess.TimeoutExpired) as exc:
            out, err, rc = "", f"{type(exc).__name__}: {exc}", None
        record = dict(id=name, argv=argv, command=shlex.join(argv), timestamp=start,
                      finished_at=timestamp(), elapsed_seconds=time.monotonic()-tick,
                      stdout=out, stderr=err, rc=rc, returncode=rc)
        records.append(record)
        return out

    def py(name, source):
        return run(name, [sys.executable, "-c", source])

    def parsed(out, fallback=None):
        try:
            return json.loads(out)
        except (ValueError, TypeError):
            return fallback

    os_text = run("os_release", ["/usr/bin/cat", "/etc/os-release"])
    os_info = dict(re.findall(r'^([A-Z_]+)="?([^"\n]*)"?$', os_text, re.M))
    kernel = run("kernel", ["/usr/bin/uname", "-srmo"]).strip()
    cpu = parsed(run("cpu", ["/usr/bin/lscpu", "--json"]), {})
    cpu_fields = {x["field"].rstrip(":"): x["data"] for x in cpu.get("lscpu", [])}
    mem = run("memory", ["/usr/bin/cat", "/proc/meminfo"])
    mem_match = re.search(r"^MemTotal:\s+(\d+) kB", mem, re.M)
    run("npu_pci", ["/usr/bin/lspci", "-nnk", "-s", "66:00.1"])
    run("gpu_pci", ["/usr/bin/lspci", "-nnk", "-s", "65:00.0"])
    run("accel_devices", ["/usr/bin/ls", "-la", "/dev/accel", "/sys/class/accel"])
    xrt = run("xrt_examine", ["/opt/xilinx/xrt/bin/xrt-smi", "examine", "--device",
                              "0000:66:00.1", "--report", "all", "--batch"])
    xrt_version = parsed(run("xrt_version", ["/usr/bin/cat", "/opt/xilinx/xrt/version.json"]), {})
    driver = run("amdxdna_version", ["/usr/bin/cat", "/sys/module/amdxdna/version"]).strip() or None
    run("amdxdna_modinfo", ["/usr/sbin/modinfo", "amdxdna"])
    firmware = run("npu_firmware", ["/usr/bin/cat", "/sys/bus/pci/devices/0000:66:00.1/fw_version"]).strip() or None
    run("driver_packages", ["/usr/bin/dpkg-query", "-W", "-f=${Package}\t${Version}\n",
                            "*xrt*", "*xdna*", "*rocm*"])
    power = run("system_power_profile", ["/usr/bin/powerprofilesctl", "get"]).strip() or None
    run("power_profile_details", ["/usr/bin/powerprofilesctl", "list"])
    cpu_power = py("cpu_power", '''import json,pathlib
p=pathlib.Path('/sys/devices/system/cpu/cpu0/cpufreq')
print(json.dumps({n:(p/n).read_text().strip() if (p/n).exists() else None for n in ['scaling_governor','energy_performance_preference']}))''')
    ac = parsed(py("ac_status", '''import pathlib,json
p=pathlib.Path('/sys/class/power_supply')
print(json.dumps({'devices':[{ 'name':d.name, **{n:(d/n).read_text().strip() if (d/n).exists() else None for n in ['type','online','status']}} for d in p.iterdir()] if p.exists() else None}))'''), {})
    npu_sys = parsed(py("npu_sysfs", '''import pathlib,json
p=pathlib.Path('/sys/bus/pci/devices/0000:66:00.1')
values={n:(p/n).read_text().strip() if (p/n).exists() else None for n in ['vbnv','fw_version','power/runtime_status','power_state']}
clocks={}
for pattern in ['*freq*','*clock*','*/*freq*','*/*clock*']:
 for f in p.glob(pattern):
  if f.is_file():
   try: clocks[str(f)]=f.read_text().strip()
   except OSError: pass
values['clock_attributes']=clocks
print(json.dumps(values))'''), {})
    run("platform_profile", ["/usr/bin/cat", "/sys/firmware/acpi/platform_profile",
                              "/sys/firmware/acpi/platform_profile_choices"])
    run("memory_dmi", ["/usr/bin/sudo", "-n", "/usr/sbin/dmidecode", "--type", "17"])
    packages = parsed(py("sdk_packages", '''import importlib.metadata as m,json
names=['onnxruntime-vitisai','onnxruntime','voe','flexml','flexmlrt','ryzenai-dynamic-dispatch','vitis-aie-essentials','onnx','numpy']
result={}
for n in names:
 try: result[n]=m.version(n)
 except m.PackageNotFoundError: result[n]=None
print(json.dumps(result))'''), {})
    runtime = parsed(py("ort_providers", '''import onnxruntime as ort,json,sys
print(json.dumps({'version':ort.__version__,'providers':ort.get_available_providers(),'python':sys.version,'executable':sys.executable,'module_path':ort.__file__}))'''), {})
    pip_check = run("pip_check", [sys.executable, "-m", "pip", "check"])
    rocm = parsed(py("local_rocm_metadata", '''import importlib.metadata as m,pathlib,json,shutil
p=pathlib.Path('/home/aup/Lerobot/envs/pi05-rocm-7.14.1/lib/python3.12/site-packages')
selected={d.metadata.get('Name'):d.version for d in m.distributions(path=[str(p)]) if any(s in d.metadata.get('Name','').lower() for s in ['torch','triton'])} if p.exists() else {}
print(json.dumps({'metadata_path':str(p),'packages':selected,'rocminfo_path':shutil.which('rocminfo'),'hipcc_path':shutil.which('hipcc'),'opt_rocm_paths':[str(x) for x in pathlib.Path('/opt').glob('rocm*')]}))'''), {})
    discovery = parsed(py("flm_openpi_discovery", '''import pathlib,json,shutil,os
roots=[pathlib.Path(p) for p in ['/usr/local/bin','/usr/bin','/home/aup/.local/bin','/opt','/home/aup/NPU','/home/aup/Lerobot','/home/aup/projects','/home/aup/Projects','/home/aup/Documents/Codex']]
skip={'.git','node_modules','site-packages','envs','models','cache','.cache','logs','outputs','work'}
flm=[]; openpi=[]
for root in roots:
 if not root.exists(): continue
 for here,dirs,files in os.walk(root):
  rel=pathlib.Path(here).relative_to(root)
  dirs[:]=[d for d in dirs if d not in skip and not d.startswith('.') and len(rel.parts)<3]
  if pathlib.Path(here).name.lower()=='openpi': openpi.append(here);dirs[:]=[]
  for name in files:
   if name.lower() in {'flm','flm.exe','fastflowlm','fastflowlm.exe'}:
    p=pathlib.Path(here)/name
    if os.access(p,os.X_OK): flm.append(str(p))
print(json.dumps({'roots':[str(p) for p in roots], 'max_depth':3,'excluded_directories':sorted(skip),'path_executables':{n:shutil.which(n) for n in ['flm','FastFlowLM','fastflowlm']},'flm_executables':sorted(set(flm)),'openpi_directories':sorted(set(openpi)),'lerobot_pi05_rocm_project_exists':pathlib.Path('/home/aup/Lerobot/projects/pi05_rocm').is_dir()}))'''), {})
    rocm_versions = sorted(set(re.findall(r"rocm([0-9.]+)", " ".join(rocm.get("packages", {}).values()))))
    npu_power_match = re.search(r"Power Mode\s*:\s*(\S+)", xrt)
    env = {
        "os": os_info.get("PRETTY_NAME"), "kernel": kernel, "cpu": cpu_fields.get("Model name"),
        "logical_cpus": cpu_fields.get("CPU(s)"), "driver": driver,
        "xrt_version": xrt_version.get("BUILD_VERSION"), "npu_firmware_version": firmware,
        "rocm_version": rocm_versions[0] if len(rocm_versions) == 1 else None,
        "rocm_version_scope": "local LeRobot environment package metadata only; no GPU import or benchmark",
        "ryzen_ai_version": packages.get("ryzenai-dynamic-dispatch"),
        "ort_version": runtime.get("version"), "ep_version": packages.get("voe"),
        "ep_version_scope": "Vitis AI EP from onnxruntime-vitisai; VOE distribution version recorded as EP implementation version",
        "ort_vitisai_package_version": packages.get("onnxruntime-vitisai"),
        "available_providers": runtime.get("providers", []),
        "power_mode": power, "npu_power_mode": npu_power_match.group(1) if npu_power_match else None,
        "npu_frequency": None, "npu_frequency_unit": "MHz", "ac_power": None,
        "memory_total_kib": int(mem_match.group(1)) if mem_match else None,
        "physical_memory_capacity": None, "memory_speed": None, "memory_bandwidth": None,
        "timestamp": timestamp(), "power_settings_changed": False,
        "unknown_fields": {
            "npu_frequency": "No clock/frequency attributes found in NPU sysfs; xrt-smi report does not expose frequency.",
            "ac_power": "No devices reported under /sys/class/power_supply.",
            "physical_memory_capacity": "DMI type 17 unavailable without root/password; OS-visible memory is recorded separately.",
            "memory_speed": "DMI type 17 unavailable without root/password; no measured LPDDR speed.",
            "memory_bandwidth": "No bandwidth benchmark run; supplied ~120 GB/s is not a local measurement."
        },
    }
    result = {
        "schema_version": 1, "stage": "P0", "status": "environment_captured",
        "environment": env, "sources": {"commands": "evidence/p0_commands.json"},
        "sdk_packages": packages, "runtime": runtime, "local_rocm_metadata": rocm,
        "cpu_power": parsed(cpu_power, {}), "ac_status": ac, "npu_sysfs": npu_sys,
        "discovery": discovery, "pip_check_stdout": pip_check.strip(),
        "npu_load_run": False, "flm_remeasured": False,
        "notes": ["Provider availability does not prove NPU execution or zero fallback.",
                  "User-supplied FLM facts are accepted without remeasurement.",
                  "Logical CPU count from lscpu is a string to preserve the probe output."],
    }
    return result, records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fingerprint", action="store_true", help="print only {environment: ...}, without writing files")
    parser.add_argument("--output", type=Path, default=ROOT / "results/P0_environment.json")
    parser.add_argument("--evidence", type=Path, default=ROOT / "evidence/p0_commands.json")
    args = parser.parse_args()
    result, records = capture()
    if args.fingerprint:
        print(json.dumps({"environment": result["environment"]}, ensure_ascii=False, indent=2))
        return
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.evidence.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    args.evidence.write_text(json.dumps({"schema_version":1, "timestamp":timestamp(), "commands":records}, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"result":str(args.output),"evidence":str(args.evidence),"environment":result["environment"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
