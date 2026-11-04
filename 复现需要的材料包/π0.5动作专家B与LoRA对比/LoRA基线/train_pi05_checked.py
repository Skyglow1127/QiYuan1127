#!/usr/bin/env python3
"""Run LeRobot 0.6.1 training, failing if PI05 base weights did not load.

Usage: python train_pi05_checked.py [the usual lerobot-train arguments]
This checks the existing load once; it does not load a second model or alter
the installed LeRobot files. All loader output is forwarded to stdout.
"""

from contextlib import redirect_stdout
from functools import wraps
from importlib.metadata import version
import os
from pathlib import Path
import sys


class _LoaderOutput:
    """Forward output immediately and remember the loader's success line."""

    SUCCESS_LINE = "All keys loaded successfully!"

    def __init__(self, stream):
        self.stream = stream
        self.loaded = False
        self._pending = ""

    def write(self, text):
        result = self.stream.write(text)
        self._pending += text
        while "\n" in self._pending:
            line, self._pending = self._pending.split("\n", 1)
            if line.rstrip("\r") == self.SUCCESS_LINE:
                self.loaded = True
        return result

    def flush(self):
        return self.stream.flush()

    def __getattr__(self, name):
        return getattr(self.stream, name)


def _install_load_guard():
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy

    original_loader = PI05Policy.from_pretrained.__func__

    @wraps(original_loader)
    def checked_loader(cls, *args, **kwargs):
        output = _LoaderOutput(sys.stdout)
        with redirect_stdout(output):
            model = original_loader(cls, *args, **kwargs)
        output.flush()
        if not output.loaded:
            raise RuntimeError(
                "PI05 base-weight loading was not confirmed. LeRobot 0.6.1 can "
                "suppress checkpoint-loading errors and return a model anyway. "
                "Training has been stopped before PEFT wrapping or optimizer "
                "updates. Check the loader messages above and resolve the "
                "download, access, or weight-compatibility error before retrying."
            )
        return model

    PI05Policy.from_pretrained = classmethod(checked_loader)


def _install_local_tokenizer():
    path = os.environ.get("PI05_TOKENIZER_PATH")
    if not path:
        return
    path = str(Path(path).resolve(strict=True))
    from lerobot.processor import TokenizerProcessorStep

    original_init = TokenizerProcessorStep.__post_init__

    @wraps(original_init)
    def local_tokenizer_init(self):
        if self.tokenizer is None and self.tokenizer_name == "google/paligemma-3b-pt-224":
            self.tokenizer_name = path
        original_init(self)

    TokenizerProcessorStep.__post_init__ = local_tokenizer_init


def main():
    installed = version("lerobot")
    if installed != "0.6.1":
        raise RuntimeError(
            f"This loader guard targets lerobot==0.6.1; installed: {installed}"
        )
    _install_load_guard()
    _install_local_tokenizer()
    from lerobot.scripts.lerobot_train import main as train_main

    train_main()


if __name__ == "__main__":
    main()
