"""Standalone command-line entry point for the current question 4 solver."""
from __future__ import annotations

import argparse

from robot_solver import run


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--base", default="http://127.0.0.1:2026")
    parser.add_argument("--log", default="robot_log_q4.jsonl")
    args = parser.parse_args()
    run(args.robot_id, args.base, "q4", args.log)


if __name__ == "__main__":
    main()
