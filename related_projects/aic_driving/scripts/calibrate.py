#!/usr/bin/env python3
"""Estimate independent empirical forecast-error margins."""
import argparse
import json
from aic_drive.learning import calibrate_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = calibrate_model(args.dataset, args.model, args.output)
    print(json.dumps({"output": args.output, "buckets": result["buckets"],
                      "analytic": result["analytic"], "learned": result["learned"]}, indent=2))


if __name__ == "__main__":
    main()
