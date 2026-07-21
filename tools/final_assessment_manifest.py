#!/usr/bin/env python3
"""Create assessment manifest entries and finalize reviewed final evidence."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re


SCHEMA_VERSION = 4
PROTECTED_EVIDENCE = (
    "metadata.json", "samples.csv", "diagnostics.csv", "events.csv",
    "paths.json", "summary.json")
REQUIRED_PARAMETERS = (
    "dynamics.yaml", "controller.yaml", "environment.yaml", "astar.yaml",
    "planned_trajectory.yaml", "interactive_goal_editor.yaml",
    "interactive_goal_executor.yaml", "mission.yaml")
SCREENSHOT_OPTIONAL_SCENARIOS = {
    "hover", "single_goal", "multi_goal",
    "disturbance_short_gust", "disturbance_persistent_release"}
SCREENSHOT_REQUIRED_SCENARIOS = {"static_avoidance", "multi_goal_navigation"}


def screenshots_required(scenario_id):
    if scenario_id in SCREENSHOT_OPTIONAL_SCENARIOS:
        return False
    if scenario_id in SCREENSHOT_REQUIRED_SCENARIOS:
        return True
    raise ValueError(f"unknown scenario_id for screenshot policy: {scenario_id}")


def report_eligibility(status, stop_reason, overall_pass, commit_before,
                       commit_after, clean_before, clean_after,
                       parameters_complete, manual_acceptance_complete,
                       required_screenshots_complete=True):
    conditions = {
        "status_is_final": status == "final",
        "non_timeout_stop": bool(stop_reason) and not stop_reason.startswith("timeout"),
        "overall_pass": overall_pass is True,
        "commit_unchanged": bool(commit_before) and commit_before == commit_after,
        "source_clean_before": clean_before is True,
        "source_clean_after": clean_after is True,
        "parameter_snapshot_complete": parameters_complete is True,
        "manual_acceptance_complete": manual_acceptance_complete is True,
        "required_screenshots_complete": required_screenshots_complete is True,
    }
    failures = [name for name, passed in conditions.items() if not passed]
    return not failures, conditions, failures


def load_manifest(path):
    if not path.exists():
        return {"schema_version": SCHEMA_VERSION, "generated_at": None, "runs": []}
    data = json.loads(path.read_text())
    if data.get("schema_version") != SCHEMA_VERSION or not isinstance(data.get("runs"), list):
        raise ValueError(f"manifest must use schema {SCHEMA_VERSION}: {path}")
    return data


def write_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_checksum_file(path, base_directory):
    checksums = {}
    for line in path.read_text().splitlines():
        parts = line.split(maxsplit=1)
        if len(parts) != 2:
            raise ValueError(f"invalid checksum line in {path}: {line!r}")
        digest, raw_name = parts
        raw_name = raw_name.lstrip("*")
        target = Path(raw_name)
        target = target.resolve() if target.is_absolute() else (base_directory / target).resolve()
        checksums[target] = digest
    return checksums


def verify_protected_evidence(run_dir):
    checksum_path = run_dir / "evidence_sha256.txt"
    if not checksum_path.is_file():
        raise ValueError("evidence_sha256.txt is missing")
    checksums = parse_checksum_file(checksum_path, run_dir)
    for name in PROTECTED_EVIDENCE:
        path = (run_dir / name).resolve()
        if not path.is_file() or path not in checksums:
            raise ValueError(f"protected evidence is missing from checksums: {name}")
        if file_sha256(path) != checksums[path]:
            raise ValueError(f"protected evidence changed after recording: {name}")


def verify_parameter_snapshot(run_dir):
    checksum_path = run_dir / "parameter_sha256.txt"
    if not checksum_path.is_file():
        return False
    parameter_dir = run_dir / "parameters"
    checksums = parse_checksum_file(checksum_path, parameter_dir)
    for name in REQUIRED_PARAMETERS:
        path = (parameter_dir / name).resolve()
        if not path.is_file() or checksums.get(path) != file_sha256(path):
            return False
    return True


def validate_manual_acceptance(run_dir, screenshot_arguments, screenshot_required):
    manual_path = run_dir / "manual_acceptance.md"
    text = manual_path.read_text()
    if not re.search(r"^Status:\s*complete\s*$", text, re.IGNORECASE | re.MULTILINE):
        raise ValueError("manual acceptance Status must be complete")
    if re.search(r"^- \[ \]", text, re.MULTILINE):
        raise ValueError("manual acceptance still contains unchecked items")
    for field in ("Reviewer", "Date"):
        if not re.search(rf"^{field}:\s*\S+", text, re.MULTILINE):
            raise ValueError(f"manual acceptance {field} is missing")
    screenshot_arguments = screenshot_arguments or []
    if screenshot_required and not screenshot_arguments:
        raise ValueError("at least one required RViz screenshot must be provided")
    screenshots = []
    for argument in screenshot_arguments:
        path = (run_dir / argument).resolve()
        try:
            relative = path.relative_to(run_dir.resolve())
        except ValueError as error:
            raise ValueError(f"screenshot must be inside run directory: {argument}") from error
        if not path.is_file() or path.suffix.lower() not in (".png", ".jpg", ".jpeg"):
            raise ValueError(f"required screenshot is missing or unsupported: {argument}")
        if str(relative) not in text and relative.name not in text:
            raise ValueError(f"manual acceptance does not reference screenshot: {relative}")
        screenshots.append(str(relative))
    return screenshots


def recalculate_evidence_checksums(run_dir):
    checksum_path = run_dir / "evidence_sha256.txt"
    files = sorted(path for path in run_dir.rglob("*")
                   if path.is_file() and path != checksum_path)
    checksum_path.write_text("".join(
        f"{file_sha256(path)}  {path.relative_to(run_dir)}\n" for path in files))


def build_entry(args):
    run_dir = args.run_dir.resolve()
    metadata = json.loads((run_dir / "metadata.json").read_text())
    summary = json.loads((run_dir / "summary.json").read_text())
    if metadata.get("scenario_id") != args.scenario_id:
        raise ValueError("metadata.json scenario_id does not match manifest scenario_id")
    eligible, conditions, failures = report_eligibility(
        args.status, metadata.get("stop_reason"), summary.get("overall_pass"),
        args.commit_before, args.commit_after, args.clean_before,
        args.clean_after, args.parameters_complete,
        args.manual_acceptance_complete, not screenshots_required(args.scenario_id))
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
        "evidence_checksums": f"{args.relative_path}/evidence_sha256.txt",
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
            "screenshots": [],
        },
        "eligibility_conditions": conditions,
        "report_eligible": eligible,
        "eligibility_failures": failures,
    }


def create_entry(args):
    if args.manual_acceptance_complete:
        raise ValueError("initial manifest entry cannot complete manual acceptance")
    entry = build_entry(args)
    entry_path = args.run_dir / "manifest_entry.json"
    if entry_path.exists():
        raise ValueError(f"refusing to overwrite manifest entry: {entry_path}")
    manifest = load_manifest(args.manifest)
    if any(run.get("path") == args.relative_path for run in manifest["runs"]):
        raise ValueError(f"manifest already contains path: {args.relative_path}")
    write_json(entry_path, entry)
    manifest["generated_at"] = datetime.now(timezone.utc).isoformat()
    manifest["runs"].append(entry)
    write_json(args.manifest, manifest)
    return entry


def finalize_entry(args):
    run_dir = args.run_dir.resolve()
    entry_path = run_dir / "manifest_entry.json"
    entry = json.loads(entry_path.read_text())
    manifest = load_manifest(args.manifest)
    matches = [index for index, item in enumerate(manifest["runs"])
               if item.get("path") == entry.get("path")]
    if len(matches) != 1:
        raise ValueError("root manifest must contain exactly one matching run")
    if entry.get("report_eligible") is True:
        raise ValueError("run is already finalized and report eligible")
    metadata = json.loads((run_dir / "metadata.json").read_text())
    summary = json.loads((run_dir / "summary.json").read_text())
    if metadata.get("scenario_id") != entry.get("scenario_id"):
        raise ValueError("metadata.json scenario_id does not match manifest scenario_id")
    if entry.get("status") != "final" or metadata.get("status") != "final" or summary.get("status") != "final":
        raise ValueError("only an existing final run may be finalized")
    if summary.get("overall_pass") is not True:
        raise ValueError("summary.json overall_pass must be true")
    verify_protected_evidence(run_dir)
    parameters_complete = verify_parameter_snapshot(run_dir)
    if not parameters_complete:
        raise ValueError("parameter snapshot or checksums are incomplete")
    screenshot_required = screenshots_required(metadata.get("scenario_id"))
    screenshots = validate_manual_acceptance(run_dir, args.screenshot, screenshot_required)
    git = entry.get("git", {})
    eligible, conditions, failures = report_eligibility(
        entry["status"], metadata.get("stop_reason"), summary.get("overall_pass"),
        git.get("commit_before"), git.get("commit_after"),
        git.get("source_clean_before"), git.get("source_clean_after"),
        parameters_complete, True, not screenshot_required or bool(screenshots))
    if not eligible:
        raise ValueError("run does not satisfy final eligibility: " + ", ".join(failures))
    reviewed_at = datetime.now(timezone.utc).isoformat()
    entry["manual_acceptance"] = {
        "completed": True,
        "file": entry.get("manual_acceptance", {}).get("file"),
        "screenshots": screenshots,
        "reviewed_at": reviewed_at,
    }
    entry["parameter_snapshot"]["complete"] = True
    entry["eligibility_conditions"] = conditions
    entry["report_eligible"] = True
    entry["eligibility_failures"] = []
    entry["finalized_at"] = reviewed_at
    manifest["runs"][matches[0]] = entry
    manifest["generated_at"] = reviewed_at
    write_json(entry_path, entry)
    write_json(args.manifest, manifest)
    recalculate_evidence_checksums(run_dir)
    return entry


def boolean(value):
    if value == "true":
        return True
    if value == "false":
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def create_arguments(parser):
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


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    create_arguments(subparsers.add_parser("create", help="create an initial ineligible run entry"))
    finalize = subparsers.add_parser("finalize", help="finalize an existing manually reviewed final run")
    finalize.add_argument("--manifest", required=True, type=Path)
    finalize.add_argument("--run-dir", required=True, type=Path)
    finalize.add_argument("--screenshot", action="append", default=[])
    return parser.parse_args()


def main():
    args = arguments()
    try:
        entry = create_entry(args) if args.command == "create" else finalize_entry(args)
    except (FileNotFoundError, KeyError, ValueError, json.JSONDecodeError) as error:
        raise SystemExit(str(error)) from error
    print(json.dumps(entry, indent=2))


if __name__ == "__main__":
    main()
