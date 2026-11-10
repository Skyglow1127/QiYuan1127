#!/usr/bin/env python3
"""Small MCP client for the shared AMD Ross daemon; never launches Vivado directly."""
import argparse
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


class Ross:
    def __init__(self):
        configured = os.environ.get("ROSS_MCP_BINARY")
        candidates = sorted(Path.home().glob(
            ".vscode/extensions/amd.vivado-ai-extension-*/bin/vivado-mcp-server-linux-amd64-*"
        ), key=lambda p: p.stat().st_mtime)
        self.binary = Path(configured) if configured else (candidates[-1] if candidates else None)
        if self.binary is None or not self.binary.is_file():
            raise RuntimeError("Set ROSS_MCP_BINARY to the installed AMD Ross MCP executable")
        self.audit_dir = ROOT / "reports"
        self.audit_dir.mkdir(exist_ok=True)
        self.stderr = (self.audit_dir / "mcp_bridge.log").open("a", encoding="utf-8")
        env = os.environ.copy()
        settings = Path.home() / ".config/Code/User/settings.json"
        if not env.get("VIVADO_PATH") and settings.exists():
            try:
                configured_path = json.loads(settings.read_text()).get("vivadoAI.vivadoPath")
                if configured_path:
                    env["VIVADO_PATH"] = configured_path
            except json.JSONDecodeError:
                pass
        self.proc = subprocess.Popen([str(self.binary), "--stdio-bridge"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr,
            text=True, encoding="utf-8", bufsize=1, env=env)
        self.messages = queue.Queue()
        self.next_id = 0
        threading.Thread(target=self._reader, daemon=True).start()
        try:
            self.server = self.request("initialize", {
                "protocolVersion": "2024-11-05", "capabilities": {},
                "clientInfo": {"name": "ross-led-mvp", "version": "1.0"}})
            self._send({"jsonrpc": "2.0", "method": "notifications/initialized"})
        except BaseException:
            self.close()
            raise

    def _reader(self):
        try:
            for line in self.proc.stdout:
                try:
                    self.messages.put(json.loads(line))
                except json.JSONDecodeError:
                    pass
        finally:
            self.messages.put(None)

    def _send(self, msg):
        self.proc.stdin.write(json.dumps(msg, ensure_ascii=False) + "\n")
        self.proc.stdin.flush()

    def request(self, method, params, timeout=600):
        self.next_id += 1
        msg = {"jsonrpc": "2.0", "id": self.next_id, "method": method, "params": params}
        self._send(msg)
        deadline = time.monotonic() + timeout
        while True:
            response = self.messages.get(timeout=max(0.01, deadline - time.monotonic()))
            if response is None:
                raise RuntimeError("MCP bridge closed; see reports/mcp_bridge.log")
            if response.get("id") == self.next_id:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(method)
        with (self.audit_dir / "mcp_audit.jsonl").open("a", encoding="utf-8") as log:
            log.write(json.dumps({"timestamp": time.time(), "request": msg,
                "response": response}, ensure_ascii=False) + "\n")
        if "error" in response:
            raise RuntimeError(json.dumps(response["error"], ensure_ascii=False))
        return response["result"]

    def tools(self):
        return self.request("tools/list", {})["tools"]

    def call(self, name, arguments=None):
        result = self.request("tools/call", {"name": name, "arguments": arguments or {}})
        if result.get("isError"):
            raise RuntimeError(json.dumps(result, ensure_ascii=False))
        return result

    def close(self):
        if self.proc.poll() is None:
            self.proc.stdin.close()
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                self.proc.wait(timeout=3)
        self.stderr.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--schema", nargs="+")
    group.add_argument("--call")
    parser.add_argument("--arguments", default="{}")
    args = parser.parse_args()
    with Ross() as ross:
        result = ([t for t in ross.tools() if t["name"] in args.schema]
                  if args.schema else ross.call(args.call, json.loads(args.arguments)))
        print(json.dumps(result, indent=2, ensure_ascii=False))
