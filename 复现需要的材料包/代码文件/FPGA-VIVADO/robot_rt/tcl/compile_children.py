#!/usr/bin/env python3
"""Prepare or sequentially compile two real, parameter-bound OOC children.

Default dry-run never launches Vivado and never creates the output directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
import time
from pathlib import Path

PROFILE = {"clock_hz": 99999001, "part": "xck26-sfvc784-2LV-c",
           "vivado_short": "2025.2", "period_ns": 10.0, "synth_directive": "default"}
MODULES = ("player_math", "limit_gate_pipe")
GATE_STABLE_JOB_INPUTS = 0


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def word(value: object) -> str:
    text = str(value)
    if any(c in text for c in "{}\\\n\r"):
        raise ValueError(f"unsupported literal characters: {text!r}")
    return "{" + text + "}"


def dict_literal(value: dict) -> str:
    return " ".join(word(key) + " " + ("{" + dict_literal(item) + "}"
        if isinstance(item, dict) else word(item)) for key, item in value.items())


def desired_generics(module: str) -> dict:
    if module not in MODULES or type(GATE_STABLE_JOB_INPUTS) is not int or GATE_STABLE_JOB_INPUTS not in (0, 1):
        raise ValueError("unsupported module/gate mode")
    value = {"CLOCK_HZ": PROFILE["clock_hz"]}
    if module == "limit_gate_pipe":
        value["STABLE_JOB_INPUTS"] = GATE_STABLE_JOB_INPUTS
    return value


def same_generics(value: object, expected: dict) -> bool:
    return isinstance(value,dict) and set(value)==set(expected) and all(type(value[k]) is int and value[k]==v for k,v in expected.items())


def input_contract(module: str) -> str:
    return "math.exact.v1" if module == "player_math" else (
        "gate.shared_owner_stable_job.v1" if GATE_STABLE_JOB_INPUTS else "gate.snapshot.v1")


def literal_words(text: str) -> list[str]:
    """Controlled Tcl literal words only; never substitute/evaluate a command."""
    if any(c in text for c in '$[];\\"'):
        raise ValueError("dynamic/nonliteral synth command")
    words = []; i = 0
    while i < len(text):
        if text[i].isspace(): i += 1; continue
        if text[i] == '{':
            depth = 1; end = i + 1
            while end < len(text) and depth:
                if text[end] == '{': depth += 1
                elif text[end] == '}': depth -= 1
                end += 1
            if depth or end < len(text) and not text[end].isspace():
                raise ValueError("malformed literal word")
            words.append(text[i+1:end-1]); i = end
        else:
            end = i
            while end < len(text) and not text[end].isspace():
                if text[end] in '{}': raise ValueError("braces in unquoted word")
                end += 1
            words.append(text[i:end]); i = end
    return words


def parse_synth_command(line: str, module: str, generics: dict) -> dict:
    if not line.startswith("Command: "):
        raise ValueError("not an actual Command line")
    words = literal_words(line[len("Command: "):])
    if len(words) != 11 or words[0] != "synth_design":
        raise ValueError("unsupported actual synth command")
    options = {}
    for key, value in zip(words[1::2], words[2::2]):
        if key in options: raise ValueError("duplicate synth option")
        options[key] = value
    expected = {"-top": module, "-part": PROFILE["part"], "-mode": "out_of_context", "-directive": PROFILE["synth_directive"]}
    if set(options) != set(expected) | {"-generic"} or any(options[k] != v for k,v in expected.items()):
        raise ValueError("actual synth command/profile mismatch")
    actual = {}
    for word in literal_words(options["-generic"]):
        match = re.fullmatch(r"([A-Z_][A-Z_0-9]*)=(0|[1-9][0-9]*)", word)
        if not match or match[1] in actual: raise ValueError("invalid/duplicate HDL generic")
        actual[match[1]] = int(match[2])
    if actual != generics: raise ValueError("actual HDL generic map differs")
    return actual


def declaration(text: str, module: str) -> str:
    """Only the two reviewed ANSI headers; generic defaults remain general0."""
    match = re.search(r"(?m)^module\s+" + re.escape(module) + r"\b", text)
    if match is None: raise ValueError(f"missing module {module}")
    end = text.find(");", match.start())
    if end < 0: raise ValueError("unterminated declaration")
    header = text[match.start():end + 2]
    parameters = r"parameter\s+integer\s+CLOCK_HZ\s*=\s*100000000"
    if module == "limit_gate_pipe":
        parameters += r"\s*,\s*parameter\s+integer\s+STABLE_JOB_INPUTS\s*=\s*0"
    elif module != "player_math": raise ValueError("unsupported module")
    if not re.search(r"#\s*\(\s*" + parameters + r"\s*\)\s*\(", header):
        raise ValueError("unexpected parameter declaration; review extraction")
    if re.search(r"\b(?:endmodule|always|initial|function)\b", header):
        raise ValueError("declaration extraction crossed into module body")
    return header


def child_tcl(out: Path, module: str) -> str:
    # Insert controlled integer generics explicitly; no eval or sourced manifest.
    generics = desired_generics(module)
    generic_words = " ".join(f"{key}={value}" for key,value in generics.items())
    verified_json = json.dumps({"schema": "robot_rt.child_ooc.v2", "module": module,
        **PROFILE, "synthesis_clock_verified": True, "dcp_reopen_clock_count": 0,
        "ports_reopen_equal": True, "hdl_generics": generics}, separators=(",", ":"))
    preamble = (f"set out {word(out)}\nset child {word(module)}\n"
        f"set expected_part {word(PROFILE['part'])}\n"
        f"set expected_version {word(PROFILE['vivado_short'])}\n"
        f"set directive {word(PROFILE['synth_directive'])}\n"
        f"set generic_words {word(generic_words)}\n")
    body = r'''
proc require {condition message} {
    if {![uplevel 1 [list expr $condition]]} {error $message}
}
proc port_signature {} {
    set signature {}
    foreach port [lsort [get_ports -quiet *]] {
        lappend signature [list [get_property NAME $port] [get_property DIRECTION $port]]
    }
    require {[llength $signature] > 0} "empty synthesized port signature"
    return $signature
}
proc check_part {object expected} {
    set actual [get_property PART $object]
    require {[string equal -nocase $actual $expected]} "PART mismatch: $actual versus $expected"
}
if {[catch {
    # Explicit in-memory project prevents implicit-project default PART leakage.
    create_project -in_memory -part $expected_part
    check_part [current_project] $expected_part
    set actual_version [version -short]
    require {$actual_version eq $expected_version} "tool version mismatch: $actual_version"
    set_param general.maxThreads 1
    require {[get_param general.maxThreads] == 1} "maxThreads did not persist"
    set_property GENERIC $generic_words [current_fileset]
    require {[lsort [get_property GENERIC [current_fileset]]] eq [lsort $generic_words]} "generic map not configured"
    read_verilog -sv [file join $out ${child}_source.sv]
    read_xdc [file join $out child_ooc.xdc]
    synth_design -top $child -part $expected_part -mode out_of_context -generic $generic_words -directive $directive
    check_part [current_design] $expected_part
    set clocks [get_clocks -quiet *]
    set input_clocks [get_clocks -quiet -of_objects [get_ports clk]]
    require {[llength $clocks] == 1 && [llength $input_clocks] == 1} "expected exactly one child clock"
    require {[get_property NAME [lindex $clocks 0]] eq "child_ooc_clk"} "unexpected clock name"
    require {abs([get_property PERIOD [lindex $clocks 0]] - 10.0) < 0.000001} "child clock period is not 10ns"
    report_clocks -file [file join $out synthesis_clocks.rpt]
    report_utilization -file [file join $out synthesis_utilization.rpt]
    report_timing_summary -delay_type min_max -file [file join $out synthesis_timing.rpt]
    set ports_before [port_signature]
    set handle [open [file join $out port_signature.tcl] w]
    puts $handle [list set child_ports $ports_before]
    close $handle
    write_verilog -force -mode synth_stub [file join $out ${child}_tool_stub.v]
    # Official reset_timing removes this child-only in-memory timing view.
    # Parent/final design constraints are never opened by this process.
    reset_timing -invalid
    require {[llength [get_clocks -quiet *]] == 0} "clock remains before neutral checkpoint"
    write_checkpoint -force [file join $out ${child}.dcp]
    close_project
    open_checkpoint [file join $out ${child}.dcp]
    check_part [current_design] $expected_part
    require {[llength [get_clocks -quiet *]] == 0} "clock survived neutral checkpoint reopen"
    require {[port_signature] eq $ports_before} "reopened port signature changed"
    write_xdc -force [file join $out neutral_constraints.xdc]
    set handle [open [file join $out tool_verification.json] w]
    puts $handle {@VERIFIED_JSON@}
    close $handle
    close_project
    puts "PASS CHILD_OOC $child CLOCK_HZ=99999001 PART=$expected_part CLOCK_NEUTRAL=1"
} why options]} {
    puts stderr "CHILD_ERROR: $why"
    if {[dict exists $options -errorinfo]} {puts stderr [dict get $options -errorinfo]}
    exit 1
}
exit 0
'''
    return preamble + body.replace('@VERIFIED_JSON@', verified_json)


def prepare_source(repo: Path, module: str) -> tuple[Path, bytes, str]:
    source = (repo / "rtl" / f"{module}.sv").resolve(strict=True)
    content = source.read_bytes()
    return source, content, declaration(content.decode("utf-8"), module)


def bind_evidence(directory: Path, module: str, entry: dict, script: Path, stdout: Path,
                  tool_json: Path, command: str, kind: str) -> dict:
    entry = dict(entry, hdl_generics=desired_generics(module), input_contract=input_contract(module), evidence_kind=kind)
    for key,path in (("compile_script",script),("synth_log",stdout),("tool_verification",tool_json)):
        entry[key] = str(path.resolve()); entry[key+"_sha256"] = sha256(path)
    evidence = {"schema":"robot_rt.child_parameters.v2", "module":module,
        "hdl_generics":entry["hdl_generics"], "input_contract":entry["input_contract"],
        "evidence_kind":kind, "part":PROFILE["part"], "vivado_short":PROFILE["vivado_short"],
        "synth_directive":PROFILE["synth_directive"], "source_sha256":entry["source_sha256"],
        "dcp_sha256":entry["dcp_sha256"], "compile_script_sha256":entry["compile_script_sha256"],
        "synth_log_sha256":entry["synth_log_sha256"], "tool_verification_sha256":entry["tool_verification_sha256"]}
    if "reuse_origin_manifest" in entry:
        evidence["reuse_origin_manifest_sha256"] = entry["reuse_origin_manifest_sha256"]
    target = directory/"parameter_evidence.tcl"
    target.write_text("set child_parameters {"+dict_literal(evidence)+"}\n")
    entry["parameter_evidence"] = str(target.resolve()); entry["parameter_evidence_sha256"] = sha256(target)
    return entry


def reusable_child(repo: Path, output: Path, module: str, previous: Path) -> dict | None:
    """Exact v2 parameters; migrate v1 only for clock-only unchanged math."""
    manifest = previous/module/"manifest.json"
    if not manifest.is_file(): return None
    report = json.loads(manifest.read_text())
    if report.get("schema") not in ("robot_rt.child_ooc.v1", "robot_rt.child_ooc.v2") or report.get("module") != module or report.get("status") != "verified":
        raise ValueError(f"{module}: invalid previous child report")
    legacy = report["schema"] == "robot_rt.child_ooc.v1"
    if legacy and module != "player_math": return None  # Never normalize old a5 to a new gate generic.
    if report.get("profile") != PROFILE: return None
    entry = report["entry"]
    if not legacy and (not same_generics(entry.get("hdl_generics"),desired_generics(module)) or entry.get("input_contract") != input_contract(module)):
        return None  # Different real semantic profile requires fresh synthesis.
    source = (repo/"rtl"/f"{module}.sv").resolve(strict=True)
    if Path(entry["source"]).resolve(strict=True) != source: raise ValueError("previous source belongs to another repository")
    if sha256(source) != entry["source_sha256"]: return None
    declaration(source.read_text(),module)
    for key in ("source","source_snapshot","stub","dcp","tool_stub","port_signature"):
        path = Path(entry[key])
        if not path.is_file() or not path.stat().st_size or sha256(path) != entry[key+"_sha256"]:
            raise ValueError(f"{module}: previous artifact changed: {key}")
    if declaration(Path(entry["stub"]).read_text(),module) != declaration(source.read_text(),module):
        raise ValueError("previous stub is not exact general declaration")
    script = manifest.parent/"compile.tcl" if legacy else Path(entry["compile_script"])
    stdout = manifest.parent/"stdout.log" if legacy else Path(entry["synth_log"])
    tool_json = manifest.parent/"tool_verification.json" if legacy else Path(entry["tool_verification"])
    if sha256(script) != report["script_sha256"] or sha256(stdout) != report["stdout_sha256"]:
        raise ValueError("previous tool script/log changed")
    log = stdout.read_text(); commands = re.findall(r"(?m)^Command: synth_design\b[^\n]*",log)
    if len(commands)!=1 or commands[0] != report["actual_synth_command"]:
        raise ValueError("previous actual Command evidence incomplete")
    parse_synth_command(commands[0],module,desired_generics(module))
    if re.search(r"(?m)^(?:ERROR:|CHILD_ERROR:)",log): raise ValueError("previous tool errors")
    sentinel=f"PASS CHILD_OOC {module} CLOCK_HZ=99999001 PART={PROFILE['part']} CLOCK_NEUTRAL=1"
    if log.splitlines().count(sentinel)!=1: raise ValueError("previous completion sentinel incomplete")
    expected = {"schema":"robot_rt.child_ooc.v1" if legacy else "robot_rt.child_ooc.v2", "module":module,
        "clock_hz":PROFILE["clock_hz"],"part":PROFILE["part"],"vivado_short":PROFILE["vivado_short"],
        "synth_directive":PROFILE["synth_directive"],"synthesis_clock_verified":True,
        "dcp_reopen_clock_count":0,"ports_reopen_equal":True}
    if not legacy: expected.update(period_ns=PROFILE["period_ns"],hdl_generics=desired_generics(module))
    legacy_tool = legacy or entry.get("evidence_kind") == "legacy_math_v1"
    if legacy_tool and module != "player_math": raise ValueError("legacy parameter evidence is math-only")
    if legacy_tool:
        expected.pop("period_ns",None);expected.pop("hdl_generics",None);expected["schema"]="robot_rt.child_ooc.v1"
    verified = json.loads(tool_json.read_text())
    if (not legacy_tool and not same_generics(verified.get("hdl_generics"),desired_generics(module))) or verified != expected or report.get("tool_verification") != verified: raise ValueError("previous real verification incomplete")
    if not legacy:
        for key in ("compile_script","synth_log","tool_verification","parameter_evidence") + (("reuse_origin_manifest",) if "reuse_origin_manifest" in entry else ()):
            if sha256(Path(entry[key])) != entry[key+"_sha256"]: raise ValueError("previous parameter evidence changed")
        if entry.get("evidence_kind") not in ("fresh_v2","legacy_math_v1"): raise ValueError("unsupported evidence kind")
    directory=output/module;directory.mkdir()
    kind="legacy_math_v1" if legacy else entry["evidence_kind"]
    entry = dict(entry,reuse_origin_manifest=str(manifest.resolve()),reuse_origin_manifest_sha256=sha256(manifest))
    bound=bind_evidence(directory,module,entry,script,stdout,tool_json,commands[0],kind)
    normalized=dict(report,schema="robot_rt.child_ooc.v2",entry=bound,new_synthesis_run=False,
        reused_from_manifest=str(manifest.resolve()),reused_from_manifest_sha256=sha256(manifest),
        legacy_origin_schema="robot_rt.child_ooc.v1" if legacy else report.get("legacy_origin_schema"))
    # Original v1 report/artifacts remain untouched; this is a new reuse wrapper.
    (directory/"manifest.json").write_text(json.dumps(normalized,indent=2)+"\n")
    (directory/"reuse.json").write_text(json.dumps({"status":"reused_verified_bytes","previous_manifest":str(manifest),
        "previous_manifest_sha256":sha256(manifest),"entry":bound,"new_synthesis_run":False,
        "legacy_math_migration":legacy,"sta_accepted":False},indent=2)+"\n")
    print(f"PASS REUSED_CHILD_OOC {module}; original checkpoint/readback unchanged",flush=True)
    return bound


def compile_child(repo: Path, output: Path, module: str, vivado: Path) -> dict:
    source, content, header = prepare_source(repo, module)
    original_hash = hashlib.sha256(content).hexdigest()
    directory = output / module
    directory.mkdir()
    snapshot = directory / f"{module}_source.sv"
    snapshot.write_bytes(content)
    stub = directory / f"{module}_stub.sv"
    stub.write_text("// Exact original declaration; parent must override CLOCK_HZ=99999001.\n"
                    '(* black_box = "true" *)\n' + header + "\nendmodule\n", encoding="utf-8")
    (directory / "child_ooc.xdc").write_text(
        "create_clock -name child_ooc_clk -period 10.000 [get_ports clk]\n", encoding="utf-8")
    script = directory / "compile.tcl"
    script.write_text(child_tcl(directory, module), encoding="utf-8")
    stdout = directory / "stdout.log"
    started = time.monotonic()
    command = [str(vivado), "-mode", "batch", "-notrace", "-source", str(script),
               "-log", str(directory / "vivado.log"), "-journal", str(directory / "vivado.jou")]
    with stdout.open("wb") as stream:
        process = subprocess.run(command, cwd=directory, stdout=stream, stderr=subprocess.STDOUT, check=False)
    elapsed = time.monotonic() - started
    log = stdout.read_text(encoding="utf-8", errors="replace")
    sentinel = f"PASS CHILD_OOC {module} CLOCK_HZ=99999001 PART={PROFILE['part']} CLOCK_NEUTRAL=1"
    if process.returncode or re.search(r"(?m)^(?:ERROR:|CHILD_ERROR:)", log) or sentinel not in log.splitlines():
        raise RuntimeError(f"{module} tool failure/incomplete verification; see {stdout}")
    actual_commands = re.findall(r"(?m)^Command:\s*synth_design\b[^\n]*", log)
    if len(actual_commands) != 1:
        raise RuntimeError(f"{module}: missing/duplicate actual synth Command")
    parse_synth_command(actual_commands[0], module, desired_generics(module))
    if not re.search(r"-directive\s+" + re.escape(PROFILE["synth_directive"]) + r"(?:\s|$)", actual_commands[0]):
        raise RuntimeError(f"{module}: actual synth Command does not prove requested directive")
    verified = json.loads((directory / "tool_verification.json").read_text())
    for key, expected in {"schema":"robot_rt.child_ooc.v2", "hdl_generics":desired_generics(module),
                          "module": module, "clock_hz": PROFILE["clock_hz"], "part": PROFILE["part"],
                          "vivado_short": PROFILE["vivado_short"], "synth_directive": PROFILE["synth_directive"], "synthesis_clock_verified": True,
                          "dcp_reopen_clock_count": 0, "ports_reopen_equal": True}.items():
        if (key == "hdl_generics" and not same_generics(verified.get(key),expected)) or verified.get(key) != expected:
            raise RuntimeError(f"{module}: verification mismatch at {key}")
    artifacts = {"stub": stub, "dcp": directory / f"{module}.dcp",
                 "source_snapshot": snapshot, "tool_stub": directory / f"{module}_tool_stub.v",
                 "port_signature": directory / "port_signature.tcl"}
    for key, path in artifacts.items():
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError(f"{module}: missing/empty {key}")
    neutral = (directory / "neutral_constraints.xdc").read_text()
    if re.search(r"(?m)^\s*create_(?:generated_)?clock\b", neutral):
        raise RuntimeError(f"{module}: clock command remains in reopened neutral constraints")
    if sha256(source) != original_hash or sha256(snapshot) != original_hash:
        raise RuntimeError(f"{module}: source changed during compilation")
    # Preserve exact header, instead of reverse engineering the tool's flattened stub.
    if declaration(stub.read_text(), module) != header:
        raise RuntimeError("parameter-compatible stub differs from original declaration")
    entry = {"source": str(source), "source_sha256": original_hash,
             "clock_hz": PROFILE["clock_hz"], "synth_directive": PROFILE["synth_directive"], "clock_neutral": 1, "dcp_reopen_clock_count": 0}
    for key, path in artifacts.items():
        entry[key] = str(path)
        entry[key + "_sha256"] = sha256(path)
    entry = bind_evidence(directory,module,entry,script,stdout,directory/"tool_verification.json",actual_commands[0],"fresh_v2")
    report = {"schema": "robot_rt.child_ooc.v2", "status": "verified", "profile": PROFILE,
              "module": module, "entry": entry, "elapsed_s": elapsed,
              "actual_synth_command": actual_commands[0], "tool_verification": verified,
              "script_sha256": sha256(script), "stdout_sha256": sha256(stdout),
              "declaration_sha256": hashlib.sha256(header.encode()).hexdigest(),
              "sta_accepted": False}
    (directory / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    print(sentinel, flush=True)
    return entry


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, epilog=(
        "Dry-run: python3 compile_children.py --repo /path/robot_rt --out /tmp/fresh_children\n"
        "Local execution: add --execute. The output directory must not exist."),
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--profile", choices=["ps_99999001"], default="ps_99999001")
    parser.add_argument("--vivado-root", type=Path, default=Path("/home/aup/Xilinx/2025.2/Vivado"))
    parser.add_argument("--synth-directive", choices=["default", "runtimeoptimized"], default="default",
                        help="Official Vivado synthesis effort; final STA acceptance remains separate")
    parser.add_argument("--reuse-from", type=Path,
                        help="Reuse matching, fully verified children from a prior output; changed children compile again")
    parser.add_argument("--gate-stable-job-inputs",type=int,choices=(0,1),default=0,
                        help="Gate only: explicit stable shared-owner inputs; default0 snapshots")
    parser.add_argument("--execute", action="store_true", help="actually run the two jobs sequentially")
    args = parser.parse_args()
    global GATE_STABLE_JOB_INPUTS
    GATE_STABLE_JOB_INPUTS = args.gate_stable_job_inputs
    PROFILE["synth_directive"] = args.synth_directive
    repo, output = args.repo.resolve(strict=True), args.out.resolve()
    if output.exists():
        parser.error("--out must be a fresh, nonexistent directory")
    sources = {name: prepare_source(repo, name) for name in MODULES}
    plan = {"status": "requested_execution" if args.execute else "dry_run", "execute": args.execute,
            "profile": PROFILE, "output": str(output),
            "reuse_from": str(args.reuse_from.resolve()) if args.reuse_from else None,
            "sequence": [{"module": name, "source": str(source),
                          "source_sha256": hashlib.sha256(content).hexdigest(),
                          "actual_read_snapshot": str(output / name / f"{name}_source.sv"),
                          "hdl_generics":desired_generics(name), "input_contract":input_contract(name),
                          "synth_options": f"-top {name} -part {PROFILE['part']} -mode out_of_context -generic " + " ".join(f"{k}={v}" for k,v in desired_generics(name).items()) + f" -directive {PROFILE['synth_directive']}"}
                         for name, (source, content, _) in sources.items()],
            "partition_plan_published_only_after_both_verified": True, "sta_accepted": False}
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    vivado = (args.vivado_root / "bin" / "vivado").resolve(strict=True)
    output.mkdir(parents=True, exist_ok=False)
    (output / "request.json").write_text(json.dumps(plan, indent=2) + "\n")
    compiler_sha = sha256(Path(__file__))
    try:
        modules = {}
        for name in MODULES:
            entry = reusable_child(repo, output, name, args.reuse_from.resolve()) if args.reuse_from else None
            modules[name] = entry if entry is not None else compile_child(repo, output, name, vivado)
            if sha256(Path(__file__)) != compiler_sha:
                raise RuntimeError("child compiler changed during execution")
        # Recheck all original and exported bytes immediately before publication.
        for name, entry in modules.items():
            for key in ("source", "source_snapshot", "stub", "dcp", "tool_stub", "port_signature", "compile_script", "synth_log", "tool_verification", "parameter_evidence"):
                if sha256(Path(entry[key])) != entry[key + "_sha256"]:
                    raise RuntimeError(f"{name}: artifact changed before publication: {key}")
            if "reuse_origin_manifest" in entry and sha256(Path(entry["reuse_origin_manifest"])) != entry["reuse_origin_manifest_sha256"]:
                raise RuntimeError(f"{name}: reuse origin changed before publication")
        partition = {"schema": "robot_rt.partition.v2", "clock_hz": PROFILE["clock_hz"],
                     "part": PROFILE["part"], "vivado_short": PROFILE["vivado_short"], "modules": modules}
        path = output / "partition_plan.tcl"
        path.write_text("set partition_plan {" + dict_literal(partition) + "}\n", encoding="utf-8")
        manifest = {"status": "both_children_verified", "partition": partition,
                    "partition_plan_sha256": sha256(path), "entry_sha256": compiler_sha,
                    "sta_accepted": False}
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print("PASS CHILD_PARTITIONS BOTH_VERIFIED CLOCK_NEUTRAL=1", flush=True)
        return 0
    except Exception as error:
        (output / "failure.json").write_text(json.dumps({"status": "failed", "error": str(error)}, indent=2) + "\n")
        print(f"CHILD_ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
