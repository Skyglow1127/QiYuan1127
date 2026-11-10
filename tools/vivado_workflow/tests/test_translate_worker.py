import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location("translate_worker", Path(__file__).resolve().parents[1] / "localization" / "translate_worker.py")
worker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(worker)


class TranslationWorkerTest(unittest.TestCase):
    def test_data_and_code_are_excluded(self):
        for text in ["clk_count", "clk", "counter", "xck26-sfvc784-2LV-c", "/home/aup/demo.xpr", "123.45", "RTL",
                     "set_property PART xck26 [current_project]", "assign led = counter[4];", "x\ty", "x\ny"]:
            with self.subTest(text=text):
                self.assertIsNotNone(worker.skip_reason(text))
                self.assertEqual(worker.translate_text(text), text)

    def test_protected_technical_tokens_survive(self):
        with patch.object(worker, "_translate_piece", side_effect=lambda s: "译文" if worker.WORD.search(s) else s):
            result = worker.translate_text("Use Vivado to create an RTL project with ILA at 100 MHz.")
        for token in ["Vivado", "RTL", "ILA", "100 MHz"]:
            self.assertIn(token, result)

    def test_persists_cache_and_reports_failures(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); queue = root / "queue.jsonl"; cache = root / "cache.tsv"; status = root / "status.json"
            queue.write_text("\n".join(json.dumps(x) for x in ["Existing label", "New label", "Bad label", "RTL", "New label"]) + '\n{"invalid":true}\n')
            cache.write_text("Existing label\t已有标签\n", encoding="utf-8")
            def translate(text):
                if text == "Bad label":
                    raise RuntimeError("expected model failure")
                return "新标签"
            with patch.object(worker, "translate_text", side_effect=translate):
                result = worker.run_once(queue, cache, status)
            self.assertEqual(result["state"], "completed_with_errors")
            self.assertEqual(result["failed"], 2)
            self.assertEqual(result["cached"], 1)
            self.assertEqual(result["translated"], 1)
            self.assertEqual(worker.read_cache(cache), {"Existing label": "已有标签", "New label": "新标签"})
            self.assertEqual(json.loads(status.read_text())["failed"], 2)

    def test_bad_existing_cache_is_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); cache = root / "cache.tsv"; cache.write_text("bad cache line\n")
            result = worker.run_once(root / "queue", cache, root / "status")
            self.assertEqual(result["state"], "failed")
            self.assertEqual(cache.read_text(), "bad cache line\n")


if __name__ == "__main__":
    unittest.main()
