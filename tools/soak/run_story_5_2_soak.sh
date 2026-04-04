#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MINUTES="${1:-10}"
OUT_DIR="$ROOT_DIR/_bmad-output/implementation-artifacts/evidence/5-2"
TS="$(date +%Y%m%d-%H%M%S)"
CSV_FILE="$OUT_DIR/soak-samples-$TS.csv"
REPORT_FILE="$OUT_DIR/soak-report-$TS.md"

mkdir -p "$OUT_DIR"

PERCEPTION_BIN="$ROOT_DIR/build/dog_perception/test_perception_node"
LIFECYCLE_BIN="$ROOT_DIR/build/dog_lifecycle/test_lifecycle_node"
BEHAVIOR_BIN="$ROOT_DIR/build/dog_behavior/test_behavior_node"

for bin in "$PERCEPTION_BIN" "$LIFECYCLE_BIN" "$BEHAVIOR_BIN"; do
  if [[ ! -x "$bin" ]]; then
    echo "Missing executable: $bin" >&2
    echo "Please build tests first (colcon test / colcon build with tests enabled)." >&2
    exit 1
  fi
done

END_EPOCH="$(( $(date +%s) + MINUTES * 60 ))"

echo "timestamp,binary,test_name,elapsed_sec,max_rss_kb,exit_code" > "$CSV_FILE"

run_case() {
  local binary="$1"
  local test_name="$2"
  local start_epoch end_epoch elapsed rss exit_code
  local rss_file
  rss_file="$(mktemp)"

  start_epoch="$(date +%s)"
  set +e
  /usr/bin/time -f "%M" -o "$rss_file" "$binary" --gtest_filter="$test_name" --gtest_color=no >/dev/null 2>&1
  exit_code=$?
  set -e
  end_epoch="$(date +%s)"
  elapsed="$(( end_epoch - start_epoch ))"
  rss="$(cat "$rss_file" 2>/dev/null || echo 0)"
  rm -f "$rss_file"

  echo "$(date +%Y-%m-%dT%H:%M:%S),$binary,$test_name,$elapsed,$rss,$exit_code" >> "$CSV_FILE"
  return 0
}

while [[ "$(date +%s)" -lt "$END_EPOCH" ]]; do
  run_case "$PERCEPTION_BIN" "PerceptionNodeTest.SingleSideDropoutTriggersExtrapolationAndThenRecovers"
  run_case "$LIFECYCLE_BIN" "LifecycleNodeTest.EstopSwitchesIdleSpinningModeAndRecoversToNormal"
  run_case "$BEHAVIOR_BIN" "BehaviorNodeTest.IdleSpinningModeBlocksNewGoalAndKeepsRecoveredContext"
done

python3 - "$CSV_FILE" "$REPORT_FILE" "$MINUTES" << 'PY'
import csv
import statistics
import sys
from collections import defaultdict

csv_file, report_file, minutes = sys.argv[1], sys.argv[2], sys.argv[3]
rows = []
with open(csv_file, newline="", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    rows = list(reader)

by_case = defaultdict(list)
for row in rows:
    key = (row["binary"], row["test_name"])
    by_case[key].append(row)

lines = []
lines.append("# Story 5.2 Soak Evidence")
lines.append("")
lines.append(f"- Duration target: {minutes} minutes")
lines.append(f"- Sample count: {len(rows)}")
lines.append(f"- Source CSV: {csv_file}")
lines.append("")
lines.append("## Summary")
lines.append("")
lines.append("| Binary | Test | Runs | Failures | RSS min/max/avg (KB) |")
lines.append("|---|---|---:|---:|---:|")

for (binary, test_name), samples in by_case.items():
  rss_values = []
  for x in samples:
    rss_raw = x.get("max_rss_kb") or ""
    rss_raw = rss_raw.strip()
    if rss_raw.isdigit():
      rss_values.append(int(rss_raw))
    failures = sum(1 for x in samples if x["exit_code"] != "0")
    if rss_values:
        rss_min = min(rss_values)
        rss_max = max(rss_values)
        rss_avg = int(statistics.mean(rss_values))
        rss_text = f"{rss_min}/{rss_max}/{rss_avg}"
    else:
        rss_text = "N/A"
    lines.append(f"| {binary} | {test_name} | {len(samples)} | {failures} | {rss_text} |")

lines.append("")
lines.append("## Notes")
lines.append("")
lines.append("- This report is an approximated pressure/soak evidence for AC3 using repeated targeted tests.")
lines.append("- For strict 2-hour acceptance, rerun this script with `120` minutes and archive the generated report.")

with open(report_file, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")
PY

echo "Soak CSV: $CSV_FILE"
echo "Soak report: $REPORT_FILE"
