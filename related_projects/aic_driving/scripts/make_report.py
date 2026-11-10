#!/usr/bin/env python3
"""Create offline development-evaluation charts and a report from episodes.csv."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Running the script directly is supported before editable installation.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from aic_drive.visualization import make_report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Evaluation episodes.csv")
    parser.add_argument("--output", required=True, type=Path, help="Directory for generated charts and report")
    args = parser.parse_args()
    report = make_report(args.input, args.output)
    print(report.resolve())


if __name__ == "__main__":
    main()
