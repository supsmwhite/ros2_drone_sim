#!/usr/bin/env python3
"""Create one immutable assessment run entry and update its output-root manifest."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path


SCHEMA_VERSION = 4


def report_eligibility(status, stop_reason, overall_pass, commit_before,
                       commit_after, clean_before, clean_after,
                       parameters_complete, manual_acceptance_complete):
    conditions = {
        "status_is_final": status == "final",
        "non_timeout_stop": bool(stop_reason) and not stop_reason.startswith("timeout"),
        "overall_pass": overall_pass is True,
        "commit_unchanged": bool(commit_before) and commit_before == commit_after,
        "source_clean_before": clean_before is True,
        "source_clean_after": clean_after is True,
        "parameter_snapshot_complete": parameters_complete is True,
        "manual_acceptance_complete": manual_acceptance_complete is True,
    }
    failures = [name for name, passed in conditions.items() if not passed]
    return not failures, conditions, failures


def load_manifest(path):
    if not path.exists():
        return {"schema_version": SCHEMA_VERSION,
                "generated_at": None,
                "runs": []}
    data = json.loads(path.read_text())
    if data.get("schema_version") != SCHEMA_VERSION or not isinstance(data.get("runs"), list):
        raise ValueError(f"manifest must use schema {SCHEMA_VERSION}: {path}")
    return data


def write_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def build_entry(args):
    run_dir = args.run_dir.resolve()
    metadata = json.loads((run_dir / "metadata.json").read_text())
    summary = json.loads((run_dir / "summary.json").read_text())
    eligible, conditions, failures = report_eligibility(
        args.status, metadata.get("stop_reason"), summary.get("overall_pass"),
        args.commit_before, args.commit_after, args.clean_before,
        args.clean_after, args.parameters_complete,
        args.manual_acceptance_complete)
    return {
        "scenario_id": args.scenario_id,
        "recorder_experiment": args.recorder_experiment,
        "status": args.status,
        "run_id": args.run_id,
        "path": args.relative_path,
        "protocol_version": metadata.get("protocol_version"),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "metadata": f"{args.relative_path}/metadata.json",
        "summary": f"{args.relative_path}/summary.json",
        "manifest_entry": f"{args.relative_path}/manifest_entry.json",
        "git": {
            "commit_before": args.commit_before,
            "commit_after": args.commit_after,
            "source_clean_before": args.clean_before,
            "source_clean_after": args.clean_after,
        },
        "parameter_snapshot": {
            "complete": args.parameters_complete,
            "directory": f"{args.relative_path}/parameters",
            "checksums": f"{args.relative_path}/parameter_sha256.txt",
        },
        "outcome": {
            "stop_reason": metadata.get("stop_reason"),
            "overall_pass": summary.get("overall_pass"),
            "failure_reasons": summary.get("failure_reasons", []),
        },
        "manual_acceptance": {
            "completed": args.manual_acceptance_complete,
            "file": f"{args.relative_path}/manual_acceptance.md",
        },
        "eligibility_conditions": conditions,
        "report_eligible": eligible,
        "eligibility_failures": failures,
    }


def boolean(value):
    if value == "true":
        return True
    if value == "false":
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--relative-path", required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--recorder-experiment", required=True)
    parser.add_argument("--status", choices=("smoke", "trial", "final"), required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--commit-before", required=True)
    parser.add_argument("--commit-after", required=True)
    parser.add_argument("--clean-before", required=True, type=boolean)
    parser.add_argument("--clean-after", required=True, type=boolean)
    parser.add_argument("--parameters-complete", required=True, type=boolean)
    parser.add_argument("--manual-acceptance-complete", default=False, type=boolean)
    return parser.parse_args()


def main():
    args = arguments()
    entry = build_entry(args)
    entry_path = args.run_dir / "manifest_entry.json"
    if entry_path.exists():
        raise SystemExit(f"refusing to overwrite manifest entry: {entry_path}")
    manifest = load_manifest(args.manifest)
    if any(run.get("path") == args.relative_path for run in manifest["runs"]):
        raise SystemExit(f"manifest already contains path: {args.relative_path}")
    write_json(entry_path, entry)
    manifest["generated_at"] = datetime.now(timezone.utc).isoformat()
    manifest["runs"].append(entry)
    write_json(args.manifest, manifest)
    print(json.dumps(entry, indent=2))


if __name__ == "__main__":
    main()
