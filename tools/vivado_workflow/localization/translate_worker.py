#!/usr/bin/env python3
"""Offline, bounded display-text translation; no network/API calls at runtime.

Queue: UTF-8 JSONL, one JSON string per line. Cache: original TAB translation.
Known dictionary entries take precedence in the Java caller. Machine output is
a draft, never an assertion that every Vivado surface is translated correctly.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import sys
import tempfile
import time
from functools import lru_cache

BASE = Path(__file__).resolve().parent
DEFAULT_MODEL = BASE / "translation-runtime" / "translate-en_zh-1_9"
MAX_LENGTH = 4000
MAX_QUEUE_BYTES = 16 * 1024 * 1024
HAN = re.compile(r"[\u3400-\u9fff]")
WORD = re.compile(r"[A-Za-z]{2,}")
IDENTIFIER = re.compile(r"[A-Za-z_$][A-Za-z0-9_$]*")
PROTECTED = re.compile(
    r"https?://[^\s<>]+|(?:/[A-Za-z0-9_.-]+){2,}|"
    r"\b(?:Vivado|Vitis|Xilinx|AMD|KR260|Zynq|Tcl|Verilog|SystemVerilog|VHDL|"
    r"Windows|Linux|Ubuntu|PetaLinux|JTAG|FPGA)\b|"
    r"\b[A-Z][A-Z0-9]{1,}(?:-[A-Z0-9]+)*\b|"
    r"\b[A-Za-z][A-Za-z0-9]*_[A-Za-z0-9_]+\b|"
    r"\b[A-Za-z][A-Za-z0-9-]*\d[A-Za-z0-9-]*\b|"
    r"\b[A-Za-z0-9_-]+\.(?:sv|v|vhd|vhdl|xdc|tcl|xpr|xci|bit|ltx|dcp|log|rpt|json)\b|"
    r"\b\d+(?:\.\d+)*(?:\s*(?:MHz|GHz|kHz|ns|ps|GB|GiB|MB|MiB))?\b"
)
_engine = None
_model_path = DEFAULT_MODEL
SHORT_WORDS = {"and": "和", "or": "或", "the": "", "a": "一个", "an": "一个",
               "use": "使用", "using": "使用", "to": "", "from": "从", "with": "使用",
               "for": "用于", "on": "在", "in": "在", "of": "的", "by": "由",
               "is": "为", "are": "为", "as": "作为", "at": "在", "not": "非", "no": "无"}


def skip_reason(text: str) -> str | None:
    if not isinstance(text, str):
        return "not_string"
    if not text.strip():
        return "empty"
    if len(text) > MAX_LENGTH:
        return "over_4000_characters"
    if any(c in text for c in "\t\r\n\x00"):
        return "not_single_line_tsv_text"
    if HAN.search(text):
        return "already_contains_chinese"
    if not WORD.search(text):
        return "no_english_prose"
    s = text.strip()
    if (s.startswith(("/", "./", "../", "file:", "http:", "https:", "\\"))
            or re.match(r"^[A-Za-z]:[\\/]", s)):
        return "path_or_url"
    if re.search(r"(?:\$\{|\$\(|=>|<=|==|!=|&&|\|\||[{};]|</?\w+>|::)", s):
        return "code_or_html"
    if re.match(r"^(?:set|puts|proc|create_clock|set_property|launch_runs|wait_on_run|"
                r"assign|module|endmodule|always|wire|reg|logic|import|def|class)\s", s):
        return "code_or_command"
    # A lone word cannot reliably be distinguished from a user object name.
    # Single-word UI labels belong in the reviewed display dictionary instead.
    if IDENTIFIER.fullmatch(s):
        return "identifier"
    if PROTECTED.fullmatch(s):
        return "protected_technical_token"
    if re.fullmatch(r"[^\s]+\.(?:sv|v|vhd|xdc|tcl|xpr|xci|bit|ltx|dcp|log|rpt|json)", s):
        return "filename"
    return None


def _load_engine():
    global _engine
    if _engine is None:
        # These limits precede native imports; keep inference off the Swing EDT.
        os.environ["OMP_NUM_THREADS"] = "2"
        os.environ["OPENBLAS_NUM_THREADS"] = "2"
        os.environ["MKL_NUM_THREADS"] = "2"
        import ctranslate2
        import sentencepiece
        model = Path(_model_path)
        if not (model / "model" / "model.bin").is_file():
            raise RuntimeError(f"Offline translation model missing: {model}")
        sp = sentencepiece.SentencePieceProcessor(model_file=str(model / "sentencepiece.model"))
        translator = ctranslate2.Translator(str(model / "model"), device="cpu", compute_type="int8",
                                            inter_threads=1, intra_threads=2, max_queued_batches=2)
        _engine = (sp, translator)
    return _engine


@lru_cache(maxsize=2048)
def _translate_piece(piece: str) -> str:
    if not WORD.search(piece):
        return piece
    leading = piece[:len(piece) - len(piece.lstrip())]
    trailing = piece[len(piece.rstrip()):]
    source = piece.strip()
    # Tiny connectors on either side of protected tokens have no context and
    # can produce wildly unrelated translations with a general-purpose model.
    if source.lower() in SHORT_WORDS:
        return leading + SHORT_WORDS[source.lower()] + trailing
    sp, translator = _load_engine()
    tokens = sp.encode(source, out_type=str)
    if len(tokens) > 1024:
        raise ValueError("Token limit exceeded; input was not silently truncated")
    result = translator.translate_batch([tokens], beam_size=1, max_batch_size=1,
                                         max_input_length=1024, max_decoding_length=1024,
                                         repetition_penalty=1.1, disable_unk=False)[0]
    translated = sp.decode(result.hypotheses[0]).replace("▁", " ").strip()
    if not translated:
        raise RuntimeError("Offline model returned an empty translation")
    if len(result.hypotheses[0]) >= 1024:
        raise RuntimeError("Offline model reached the output limit")
    return leading + translated + trailing


def translate_text(text: str) -> str:
    """Translate one display string; excluded data/code returns unchanged.

    Acronyms, product names, paths, filenames, numbers and HDL identifiers are
    copied exactly. Translation of adjacent prose runs locally and may be
    imperfect. No whole-table/model/editor values should be sent here.
    """
    if skip_reason(text):
        return text
    result = []
    cursor = 0
    for match in PROTECTED.finditer(text):
        result.append(_translate_piece(text[cursor:match.start()]))
        result.append(match.group(0))
        cursor = match.end()
    result.append(_translate_piece(text[cursor:]))
    translated = "".join(result)
    if any(c in translated for c in "\t\r\n\x00"):
        raise RuntimeError("Offline model returned invalid TSV characters")
    if not HAN.search(translated):
        raise RuntimeError("Offline model did not produce Chinese prose")
    return translated


def atomic_write(path: Path, contents: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
            f.write(contents)
            f.flush()
            os.fsync(f.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def read_cache(path: Path) -> dict[str, str]:
    cache = {}
    if path.exists():
        for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if not line:
                continue
            fields = line.split("\t")
            if len(fields) != 2 or not all(fields):
                raise ValueError(f"Malformed existing TSV cache at line {n}; left unchanged")
            cache[fields[0]] = fields[1]
    return cache


def write_status(path: Path, status: dict) -> None:
    status["updated_unix"] = time.time()
    atomic_write(path, json.dumps(status, ensure_ascii=False, indent=2) + "\n")


def run_once(queue: Path, output: Path, status_path: Path) -> dict:
    start = time.monotonic()
    status = {"state": "running", "offline": True, "machine_translation": True,
              "quality_reviewed": False, "cpu_threads": 2, "max_characters": MAX_LENGTH,
              "model": str(_model_path), "queued": 0, "translated": 0, "cached": 0,
              "skipped": 0, "failed": 0, "errors": [], "skip_reasons": {}}
    write_status(status_path, status)
    try:
        cache = read_cache(output)
        lines = []
        if queue.exists():
            if queue.stat().st_size > MAX_QUEUE_BYTES:
                raise ValueError("Queue exceeds 16 MiB bound")
            lines = queue.read_text(encoding="utf-8").splitlines()
        seen = set()
        changed = False
        for n, line in enumerate(lines, 1):
            if not line.strip():
                continue
            try:
                original = json.loads(line)
                if not isinstance(original, str):
                    raise ValueError("Queue item must be a JSON string")
                if original in seen:
                    continue
                seen.add(original)
                status["queued"] += 1
                if original in cache:
                    status["cached"] += 1
                    continue
                reason = skip_reason(original)
                if reason:
                    status["skipped"] += 1
                    status["skip_reasons"][reason] = status["skip_reasons"].get(reason, 0) + 1
                    continue
                cache[original] = translate_text(original)
                changed = True
                status["translated"] += 1
                # Publish promptly so newly visible labels can update without
                # waiting for the whole queue. Existing translations survive.
                if status["translated"] % 8 == 0:
                    atomic_write(output, "".join(f"{k}\t{v}\n" for k, v in cache.items()))
                    write_status(status_path, status)
            except Exception as exc:
                status["failed"] += 1
                if len(status["errors"]) < 100:
                    status["errors"].append({"line": n, "error": f"{type(exc).__name__}: {exc}"})
        if changed or not output.exists():
            atomic_write(output, "".join(f"{k}\t{v}\n" for k, v in cache.items()))
        status["cache_entries"] = len(cache)
        status["state"] = "completed_with_errors" if status["failed"] else "completed"
    except Exception as exc:
        status["state"] = "failed"
        status["failed"] += 1
        status["errors"].append({"error": f"{type(exc).__name__}: {exc}"})
    status["elapsed_seconds"] = round(time.monotonic() - start, 3)
    write_status(status_path, status)
    return status


def main() -> int:
    global _model_path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--queue", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--status", required=True, type=Path)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--poll-seconds", type=float, default=1.0)
    parser.add_argument("--owner-pid", type=int, help="Exit when the owning Vivado process exits; never signal it")
    args = parser.parse_args()
    if len({p.resolve() for p in [args.queue, args.output, args.status]}) != 3:
        parser.error("queue, output, and status must be different files")
    _model_path = args.model
    stopping = False
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    previous = None
    last = None
    while not stopping:
        if args.owner_pid and not Path(f"/proc/{args.owner_pid}").exists():
            break
        signature = (args.queue.stat().st_mtime_ns, args.queue.stat().st_size) if args.queue.exists() else None
        if args.once or last is None or signature != previous:
            last = run_once(args.queue, args.output, args.status)
            previous = signature
            if args.once:
                print(json.dumps(last, ensure_ascii=False))
                return 1 if last["failed"] else 0
        time.sleep(max(0.1, min(args.poll_seconds, 10.0)))
    if last is not None:
        last["state"] = "stopped"
        write_status(args.status, last)
    return 0


if __name__ == "__main__":
    sys.exit(main())
