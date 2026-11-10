#!/usr/bin/env python3
"""Train the small CPU residual predictor with validation early stopping."""
import argparse
import json
from aic_drive.learning import train_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    metadata = train_model(args.dataset, args.output, args.epochs, args.seed)
    print(json.dumps({"best_epoch": metadata["best_epoch"],
                      "completed_epochs": metadata["completed_epochs"],
                      "heldout_errors": metadata["heldout_errors"],
                      "output": args.output}, indent=2))


if __name__ == "__main__":
    main()
