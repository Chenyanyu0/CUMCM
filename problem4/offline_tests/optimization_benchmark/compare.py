"""Ablate current p4 changes on previously generated paired benchmark scenarios."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parent.parent.parent
FLAGS = ("routeClear", "estimateCandidates", "unifiedChoice", "trialClear")
VARIANTS = {
    "baseline": (),
    "route_clear": ("routeClear",),
    "route_unified": ("routeClear", "unifiedChoice"),
    "estimate": ("estimateCandidates",),
    "unified": ("unifiedChoice",),
    "safe": ("routeClear", "estimateCandidates", "unifiedChoice"),
    "trial": ("trialClear",),
    "all": FLAGS,
}
BASELINE_FIELDS = ("virtual_time_s", "distance_m", "measurements", "switches", "failed_clears",
                   "cleared", "complete", "search_points_visited")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def hashes(executable):
    paths = list(ROOT.parent.parent.glob("*.hpp"))
    paths += list(ROOT.parent.glob("*.hpp"))
    paths += [ROOT.parent.parent / "vendor/json.hpp", ROOT / "bridge.cpp", ROOT / "compare.py", executable]
    return {path.relative_to(WORKSPACE).as_posix(): digest(path)
            for path in sorted(set(paths))}


def run_chunk(executable, jobs, timeout):
    payload = "".join(json.dumps(job, ensure_ascii=True) + "\n" for job in jobs)
    process = subprocess.run([str(executable)], input=payload, capture_output=True,
                             text=True, encoding="utf-8", timeout=timeout, check=False)
    if process.returncode:
        raise RuntimeError(f"Bridge exited {process.returncode}: {process.stderr[-4000:]}")
    lines = [line for line in process.stdout.splitlines() if line.strip()]
    if len(lines) != len(jobs):
        raise RuntimeError(f"Expected {len(jobs)} replies, received {len(lines)}: {process.stdout[-1000:]}")
    replies = []
    for job, line in zip(jobs, lines):
        reply = json.loads(line)
        if "bridge_error" in reply:
            raise RuntimeError(f"{job['id']}: {reply['bridge_error']}")
        if reply.get("id") != job["id"]:
            raise RuntimeError("Bridge changed reply order or identity")
        result = reply["result"]
        if abs(result["time_accounting_error_s"]) >= .001:
            raise RuntimeError(f"{job['id']}: invalid time accounting")
        replies.append(reply)
    return replies


def execute(executable, jobs, workers, chunk_size, timeout):
    chunks = [jobs[index:index + chunk_size] for index in range(0, len(jobs), chunk_size)]
    completed = 0
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(run_chunk, executable, chunk, timeout) for chunk in chunks]
        for future in as_completed(futures):
            replies = future.result()
            completed += len(replies)
            print(json.dumps({"finished": completed, "requested": len(jobs)}), flush=True)
            yield from replies


def quantile(values, fraction):
    return sorted(values)[max(0, math.ceil(fraction * len(values)) - 1)]


def summary(rows, variant):
    results = [row["variants"][variant] for row in rows]
    baseline = [row["baseline"] for row in rows]
    completed = [index for index, result in enumerate(results) if result["complete"]]
    saved = [baseline[index]["virtual_time_s"] - results[index]["virtual_time_s"]
             for index in completed]
    times = [results[index]["virtual_time_s"] for index in completed]
    result = {
        "cases": len(rows), "complete": len(completed),
        "failed_clears": sum(value["failed_clears"] for value in results),
        "incomplete_ids": [row["id"] for row, value in zip(rows, results) if not value["complete"]],
        "errors": [{"id": row["id"], "error": value["error"]}
                   for row, value in zip(rows, results) if value["error"]],
        "baseline_mean_time_s": statistics.fmean(value["virtual_time_s"] for value in baseline),
        "completed_mean_time_s": statistics.fmean(times) if times else None,
        "completed_weighted_time_per_source_s": sum(times) / sum(results[index]["sources"] for index in completed)
            if completed else None,
        "completed_p95_time_s": quantile(times, .95) if times else None,
        "completed_paired_mean_saved_s": statistics.fmean(saved) if saved else None,
        "completed_paired_median_saved_s": statistics.median(saved) if saved else None,
        "faster": sum(value > 1e-5 for value in saved),
        "slower": sum(value < -1e-5 for value in saved),
        "equal": sum(abs(value) <= 1e-5 for value in saved),
        "worst_regression_s": max([0.0] + [-value for value in saved]),
        "completed_mean_distance_m": statistics.fmean(results[index]["distance_m"] for index in completed)
            if completed else None,
        "completed_mean_measurements": statistics.fmean(results[index]["measurements"] for index in completed)
            if completed else None,
    }
    scalar_keys = {key for value in results for key, metric in value.get("planner", {}).items()
                   if any(word in key for word in ("trial", "route_clear", "transit_clear", "estimate", "unified"))
                   and isinstance(metric, (int, float)) and not isinstance(metric, bool)}
    result["diagnostic_totals"] = {
        key: sum(value.get("planner", {}).get(key, 0) for value in results)
        for key in sorted(scalar_keys)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=WORKSPACE / "algorithm_comparison_q4/paired_report.json")
    parser.add_argument("--executable", type=Path, default=ROOT / "bridge.exe")
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--per-group", type=int, default=10)
    parser.add_argument("--groups", nargs="+")
    parser.add_argument("--variants", nargs="+", choices=VARIANTS, default=["route_clear", "estimate", "unified", "safe", "all"])
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--chunk-size", type=int, default=8)
    parser.add_argument("--chunk-timeout", type=float, default=600)
    parser.add_argument("--verify-baseline-per-group", type=int, default=1)
    args = parser.parse_args()
    if min(args.per_group, args.workers, args.chunk_size, args.verify_baseline_per_group) < 1:
        parser.error("Counts must be positive; at least one baseline rerun per group is required")
    if args.workers > 4:
        parser.error("Use at most four subprocess workers")
    if args.report.exists():
        parser.error("Report already exists; choose a new output path")
    args.executable = args.executable.resolve()
    args.source = args.source.resolve()
    source = json.loads(args.source.read_text(encoding="utf-8"))
    available = {row["group"] for row in source["rows"]}
    groups = set(args.groups or available)
    if groups - available:
        parser.error(f"Unknown groups: {sorted(groups - available)}")
    selected = []
    verification = []
    for group in sorted(groups):
        group_rows = sorted((row for row in source["rows"] if row["group"] == group), key=lambda row: row["id"])
        if len(group_rows) < args.per_group:
            parser.error(f"Group {group} contains only {len(group_rows)} scenarios")
        selected.extend(group_rows[:args.per_group])
        verification.extend(group_rows[:min(args.per_group, args.verify_baseline_per_group)])
    by_id = {row["id"]: row for row in selected}
    if len(by_id) != len(selected):
        raise RuntimeError("Duplicate scenario IDs")
    if any(not row["p4"]["complete"] for row in selected):
        raise RuntimeError("Paired analysis requires completed baseline cases")
    started = time.monotonic()
    source_hashes = hashes(args.executable)
    baseline_jobs = [{"id": row["id"], "scenario": row["scenario"], "options": {}} for row in verification]
    baseline_checks = []
    print(json.dumps({"phase": "verify_baseline", "cases": len(baseline_jobs)}), flush=True)
    for reply in execute(args.executable, baseline_jobs, args.workers, args.chunk_size, args.chunk_timeout):
        old, current = by_id[reply["id"]]["p4"], reply["result"]
        differences = {}
        for field in BASELINE_FIELDS:
            left, right = old[field], current[field]
            equal = abs(left - right) < 1e-5 if isinstance(left, (float, int)) else left == right
            if not equal:
                differences[field] = {"previous": left, "current": right}
        if differences or not current["complete"]:
            raise RuntimeError(f"Baseline changed for {reply['id']}: {differences}; {current['error']}")
        baseline_checks.append({"id": reply["id"], "matches": True})
    variants = list(dict.fromkeys(args.variants))
    rows = {row["id"]: {"id": row["id"], "group": row["group"], "scenario": row["scenario"],
                         "baseline": row["p4"], "variants": {}} for row in selected}
    jobs = [{"id": f"{variant}:{row['id']}", "scenario": row["scenario"],
             "options": {flag: flag in VARIANTS[variant] for flag in FLAGS}}
            for row in selected for variant in variants]
    print(json.dumps({"phase": "variants", "cases": len(rows), "variants": variants, "jobs": len(jobs)}), flush=True)
    for reply in execute(args.executable, jobs, args.workers, args.chunk_size, args.chunk_timeout):
        variant, case_id = reply["id"].split(":", 1)
        rows[case_id]["variants"][variant] = reply["result"]
    if source_hashes != hashes(args.executable):
        raise RuntimeError("Benchmark source files changed during this run")
    ordered = [rows[key] for key in sorted(rows)]
    summaries = {variant: {"all": summary(ordered, variant), "groups": {
        group: summary([row for row in ordered if row["group"] == group], variant)
        for group in sorted(groups)}} for variant in variants}
    report = {
        "description": "Paired offline p4 ablations on unchanged p4/q4 comparison scenarios; not official scores.",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_report": str(args.source), "source_report_sha256": digest(args.source),
        "source_sha256": source_hashes, "baseline_checks": sorted(baseline_checks, key=lambda row: row["id"]),
        "options": {variant: {flag: flag in VARIANTS[variant] for flag in FLAGS} for variant in variants},
        "per_group": args.per_group, "summaries": summaries, "rows": ordered,
        "host_time_s": time.monotonic() - started,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"summary": {variant: value["all"] for variant, value in summaries.items()},
                      "report": str(args.report), "host_time_s": report["host_time_s"]}), flush=True)
    return 0 if all(item["all"]["complete"] == len(rows) for item in summaries.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
