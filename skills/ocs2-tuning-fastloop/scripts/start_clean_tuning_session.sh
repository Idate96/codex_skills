#!/usr/bin/env bash
set -euo pipefail

SESSION="mpc_orch"
WORKSPACE="${HOME}/ros2_ws"
OUT_ROOT="${HOME}/mpc_tuning/current"
CSV_PREFIX="mole_m4_matrix_live"
BENCH_SCRIPT="${WORKSPACE}/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_cyl_accuracy_benchmark.py"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session)
      SESSION="$2"
      shift 2
      ;;
    --workspace)
      WORKSPACE="$2"
      shift 2
      ;;
    --out-root)
      OUT_ROOT="$2"
      shift 2
      ;;
    --csv-prefix)
      CSV_PREFIX="$2"
      shift 2
      ;;
    *)
      echo "Unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Missing workspace setup file: ${WORKSPACE}/install/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${BENCH_SCRIPT}" ]]; then
  echo "Missing benchmark script: ${BENCH_SCRIPT}" >&2
  exit 1
fi

mkdir -p "${OUT_ROOT}/csv" "${OUT_ROOT}/analysis" "${OUT_ROOT}/artifacts" "${OUT_ROOT}/summaries" "${OUT_ROOT}/logs"

TS="$(date +%Y%m%d_%H%M%S)"
ACTIVE_CSV="${OUT_ROOT}/csv/${CSV_PREFIX}_${TS}.csv"
echo "${TS}" > "${OUT_ROOT}/.session_ts"
echo "${ACTIVE_CSV}" > "${OUT_ROOT}/.active_csv"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  tmux kill-session -t "${SESSION}"
fi

tmux new-session -d -s "${SESSION}" -n node_check
tmux new-window -t "${SESSION}" -n orchestrator
tmux new-window -t "${SESSION}" -n benchmark_live

tmux send-keys -t "${SESSION}:benchmark_live" \
  "bash -lc 'source \"${WORKSPACE}/install/setup.bash\" && python3 \"${BENCH_SCRIPT}\" --csv \"${ACTIVE_CSV}\" --append'" C-m

started=0
for _ in $(seq 1 24); do
  if [[ -s "${ACTIVE_CSV}" ]]; then
    started=1
    break
  fi
  sleep 0.5
done

if [[ "${started}" -ne 1 ]]; then
  echo "Benchmark process did not start for CSV: ${ACTIVE_CSV}" >&2
  tmux capture-pane -pt "${SESSION}:benchmark_live" -S -80 >&2 || true
  exit 1
fi

echo "Session: ${SESSION}"
echo "Workspace: ${WORKSPACE}"
echo "Active CSV: ${ACTIVE_CSV}"
echo "Windows:"
tmux list-windows -t "${SESSION}"
