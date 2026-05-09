#!/usr/bin/env bash
# run_benchmark.sh — automated DerpBot benchmark runner (#33)
#
# Runs the formal benchmark protocol: 5 difficulties × 5 seeds × 3 runs = 75 total.
# Sequential only — hardware cannot sustain parallel sims.
# Resumable: skips any run whose output file already exists.
#
# Usage:
#   ./scripts/run_benchmark.sh --agent-name <name> [options]
#
#   --agent-name NAME        Required. Sub-directory under robot-sandbox/results/submissions/
#   --difficulties "D ..."   Space-separated list (default: all five tiers)
#   --seeds "S ..."          Space-separated list (default: 1 2 3 4 5)
#   --runs-per-seed N        Default: 3
#   --speed N                Sim RTF multiplier passed to start_stack.sh (default: 2).
#                            Scores are in sim-seconds so they're comparable across speeds.
#   --dry-run                Print the full plan without executing any runs.
#
# Results land in: ~/Projects/robot-sandbox/results/submissions/<agent-name>/
# Progress log:    ~/Projects/robot-sandbox/results/submissions/<agent-name>/benchmark_run.log
#
# Exit codes:
#   0  All runs completed or already existed, submission passes validate_submission.py.
#   1  One or more runs failed or timed out.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPLORER_ROOT="$(dirname "$SCRIPT_DIR")"
SANDBOX_ROOT="$HOME/Projects/robot-sandbox"

# --- Defaults ----------------------------------------------------------------
AGENT_NAME=""
DIFFICULTIES="easy medium hard brutal perception_stress"
SEEDS="1 2 3 4 5"
RUNS_PER_SEED=3
SPEED=2
DRY_RUN=0

# Difficulty → scenario file stem (easy uses 900s variant per updated guide).
declare -A SCENARIO_STEM=(
    [easy]="easy"
    [medium]="medium"
    [hard]="hard"
    [brutal]="brutal"
    [perception_stress]="perception_stress"
)

# Sim-second timeout per difficulty (from scenario YAMLs).
declare -A SIM_TIMEOUT=(
    [easy]=900
    [medium]=600
    [hard]=300
    [brutal]=180
    [perception_stress]=600
)

# --- Argument parsing --------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --agent-name)     AGENT_NAME="$2";      shift 2 ;;
        --difficulties)   DIFFICULTIES="$2";    shift 2 ;;
        --seeds)          SEEDS="$2";           shift 2 ;;
        --runs-per-seed)  RUNS_PER_SEED="$2";   shift 2 ;;
        --speed)          SPEED="$2";           shift 2 ;;
        --dry-run)        DRY_RUN=1;            shift ;;
        *) echo "ERROR: Unknown argument: $1"; exit 1 ;;
    esac
done

if [[ -z "$AGENT_NAME" ]]; then
    echo "ERROR: --agent-name is required"
    echo "Usage: $0 --agent-name <name> [--difficulties '...'] [--seeds '...'] [--runs-per-seed N] [--speed N] [--dry-run]"
    exit 1
fi

SUBMISSIONS_DIR="$SANDBOX_ROOT/results/submissions/$AGENT_NAME"
RESULTS_DIR="$SANDBOX_ROOT/results"
LOG_FILE="$SUBMISSIONS_DIR/benchmark_run.log"

# Count total runs for progress display
TOTAL=0
for _d in $DIFFICULTIES; do
    for _s in $SEEDS; do
        for _r in $(seq 1 "$RUNS_PER_SEED"); do
            TOTAL=$((TOTAL + 1))
        done
    done
done

echo "=== DerpBot Benchmark Runner ==="
echo "  agent-name:    $AGENT_NAME"
echo "  difficulties:  $DIFFICULTIES"
echo "  seeds:         $SEEDS"
echo "  runs-per-seed: $RUNS_PER_SEED"
echo "  speed:         ${SPEED}×"
echo "  total runs:    $TOTAL"
echo "  output dir:    $SUBMISSIONS_DIR"
echo ""

# --- Dry-run mode ------------------------------------------------------------
if [[ $DRY_RUN -eq 1 ]]; then
    echo "=== DRY RUN — planned runs ==="
    RUN_NUM=0
    SKIP_COUNT=0
    for DIFF in $DIFFICULTIES; do
        for SEED in $SEEDS; do
            for RUN in $(seq 1 "$RUNS_PER_SEED"); do
                RUN_NUM=$((RUN_NUM + 1))
                OUTFILE="$SUBMISSIONS_DIR/${DIFF}_seed${SEED}_run${RUN}.json"
                STEM="${SCENARIO_STEM[$DIFF]:-$DIFF}"
                WALL=$(( ${SIM_TIMEOUT[$DIFF]:-600} / SPEED + 120 ))
                if [[ -f "$OUTFILE" ]]; then
                    STATUS="[skip — exists]"
                    SKIP_COUNT=$((SKIP_COUNT + 1))
                else
                    STATUS="[run — ~${WALL}s wall]"
                fi
                printf "  %3d/%-3d  %-22s seed=%-2s  run=%d  scenario=%-20s  %s\n" \
                    "$RUN_NUM" "$TOTAL" "$DIFF" "$SEED" "$RUN" "${STEM}.yaml" "$STATUS"
            done
        done
    done
    echo ""
    echo "  skip: $SKIP_COUNT / $TOTAL already exist"
    echo ""
    echo "Dry run complete. Pass without --dry-run to execute."
    exit 0
fi

# --- Setup -------------------------------------------------------------------
mkdir -p "$SUBMISSIONS_DIR"

# Generate benchmark_submission.yaml on first run (or if missing)
YAML_PATH="$SUBMISSIONS_DIR/benchmark_submission.yaml"
if [[ ! -f "$YAML_PATH" ]]; then
    SANDBOX_SHA=$(git -C "$SANDBOX_ROOT" rev-parse --short HEAD 2>/dev/null || echo "unknown")
    # Build YAML difficulty list
    DIFF_YAML=""
    for D in $DIFFICULTIES; do
        DIFF_YAML="${DIFF_YAML}  - ${D}"$'\n'
    done
    DIFF_YAML="${DIFF_YAML%$'\n'}"
    # Build seed list
    SEED_CSV=$(echo "$SEEDS" | tr ' ' ', ')
    cat > "$YAML_PATH" <<EOF
stack_name: "DerpBot Explorer — ${AGENT_NAME}"
repo_url: "https://github.com/thaije/derpbot-explorer"
sandbox_version: "main-${SANDBOX_SHA}"
scenario: "office_explore_detect"
difficulties:
${DIFF_YAML}
seeds: [${SEED_CSV}]
runs_per_seed: ${RUNS_PER_SEED}
results_dir: "results/submissions/${AGENT_NAME}/"
description: "Classical pipeline: slam_toolbox + Nav2 MPPI + frontier explorer + OWLv2 detector."
EOF
    echo "Created $YAML_PATH"
fi

# Logging helper — timestamp prefix, writes to file and stdout
log() {
    local msg="[$(date -u +%Y-%m-%dT%H:%M:%SZ)] $*"
    echo "$msg" | tee -a "$LOG_FILE"
}

log "=== Benchmark run started: agent=$AGENT_NAME  speed=${SPEED}x  total=$TOTAL ==="

# --- Main loop ---------------------------------------------------------------
RUN_NUM=0
COMPLETED=0
SKIPPED=0
FAILED=0

for DIFF in $DIFFICULTIES; do
    SCENARIO="${SCENARIO_STEM[$DIFF]:-$DIFF}"

    for SEED in $SEEDS; do
        for RUN in $(seq 1 "$RUNS_PER_SEED"); do
            RUN_NUM=$((RUN_NUM + 1))
            OUTFILE="$SUBMISSIONS_DIR/${DIFF}_seed${SEED}_run${RUN}.json"

            if [[ -f "$OUTFILE" ]]; then
                log "SKIP  ${DIFF}  seed=${SEED}  run=${RUN}  (${RUN_NUM}/${TOTAL})  output exists"
                SKIPPED=$((SKIPPED + 1))
                continue
            fi

            log "START ${DIFF}  seed=${SEED}  run=${RUN}  (${RUN_NUM}/${TOTAL})  scenario=${SCENARIO}"

            # Wall-clock timeout: sim budget / speed + 300s overhead for stack
            # startup, agent-ready wait, and SLAM/Nav2 initialization.
            WALL_TIMEOUT=$(( ${SIM_TIMEOUT[$DIFF]:-600} / SPEED + 300 ))

            # Touch a marker file so we can find results written after this point.
            MARKER="/tmp/bench_marker_$$_${RUN_NUM}"
            touch "$MARKER"

            # Launch the full stack. start_stack.sh handles: cleanup, FastDDS,
            # sim, SLAM, Nav2, agent, and the Gazebo pause/unpause timing fix.
            # It returns once the agent signals ready (~30-60 wall-s).
            if ! "$SCRIPT_DIR/start_stack.sh" \
                    --scenario "$SCENARIO" \
                    --seed "$SEED" \
                    --speed "$SPEED" \
                    >> "$LOG_FILE" 2>&1; then
                log "ERROR  ${DIFF}  seed=${SEED}  run=${RUN}  start_stack.sh failed"
                rm -f "$MARKER"
                FAILED=$((FAILED + 1))
                # cleanup.sh runs at the top of the next start_stack.sh call,
                # but run it explicitly here so a crash leaves a clean slate.
                "$SCRIPT_DIR/cleanup.sh" >> "$LOG_FILE" 2>&1 || true
                sleep 5
                continue
            fi

            # Poll for the result JSON written by run_scenario.sh.
            # The file appears when the scenario ends (SUCCESS or TIME_LIMIT).
            RESULT_FILE=""
            DEADLINE=$(( $(date +%s) + WALL_TIMEOUT ))

            while [[ $(date +%s) -lt $DEADLINE ]]; do
                # Find any new result file for this difficulty newer than the marker.
                CANDIDATE=$(find "$RESULTS_DIR" -maxdepth 1 \
                    -name "office_${DIFF}_001_*.json" \
                    -newer "$MARKER" 2>/dev/null | sort | tail -1)

                if [[ -n "$CANDIDATE" ]]; then
                    # Verify the seed matches what we requested.
                    ACTUAL_SEED=$(python3 -c \
                        "import json; print(json.load(open('$CANDIDATE'))['random_seed'])" \
                        2>/dev/null || echo "")
                    if [[ "$ACTUAL_SEED" == "$SEED" ]]; then
                        RESULT_FILE="$CANDIDATE"
                        break
                    fi
                fi
                sleep 5
            done

            rm -f "$MARKER"

            if [[ -z "$RESULT_FILE" ]]; then
                log "TIMEOUT  ${DIFF}  seed=${SEED}  run=${RUN}  no result after ${WALL_TIMEOUT}s wall"
                FAILED=$((FAILED + 1))
                "$SCRIPT_DIR/cleanup.sh" >> "$LOG_FILE" 2>&1 || true
                sleep 5
                continue
            fi

            cp "$RESULT_FILE" "$OUTFILE"

            # Extract key metrics for the log line.
            METRICS=$(python3 -c "
import json
d = json.load(open('$OUTFILE'))
m = d.get('raw_metrics', {})
print(f\"{d['overall_score']:.1f} {d['overall_grade']}  found={m.get('found_ratio','?'):.2f}  cov={m.get('exploration_coverage','?'):.1f}%  col={m.get('collision_count','?')}\")
" 2>/dev/null || echo "? ?")
            log "DONE  ${DIFF}  seed=${SEED}  run=${RUN}  (${RUN_NUM}/${TOTAL})  score=${METRICS}"
            COMPLETED=$((COMPLETED + 1))
        done
    done
done

# --- Summary -----------------------------------------------------------------
echo ""
echo "=== Benchmark run complete ==="
echo "  completed: $COMPLETED"
echo "  skipped:   $SKIPPED"
echo "  failed:    $FAILED"
echo "  log:       $LOG_FILE"
echo ""
log "=== Run complete: completed=${COMPLETED}  skipped=${SKIPPED}  failed=${FAILED} ==="

# --- Validate submission -----------------------------------------------------
echo "=== Validating submission ==="
python3.12 "$SANDBOX_ROOT/scripts/validate_submission.py" "$YAML_PATH"

[[ $FAILED -eq 0 ]]
