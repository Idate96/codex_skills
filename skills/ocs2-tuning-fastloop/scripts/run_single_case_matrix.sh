#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${HOME}/ros2_ws"
OUT_ROOT="${HOME}/mpc_tuning/current"
MATRIX=""
CSV=""
ENABLE_HARD_GATES=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      WORKSPACE="$2"
      shift 2
      ;;
    --out-root)
      OUT_ROOT="$2"
      shift 2
      ;;
    --matrix)
      MATRIX="$2"
      shift 2
      ;;
    --csv)
      CSV="$2"
      shift 2
      ;;
    --enable-hard-gates)
      ENABLE_HARD_GATES=1
      shift
      ;;
    *)
      echo "Unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -z "${MATRIX}" ]]; then
  echo "--matrix is required" >&2
  exit 2
fi

if [[ ! -f "${MATRIX}" ]]; then
  echo "Matrix file not found: ${MATRIX}" >&2
  exit 1
fi

if [[ -z "${CSV}" ]]; then
  if [[ ! -f "${OUT_ROOT}/.active_csv" ]]; then
    echo "Missing ${OUT_ROOT}/.active_csv. Run start_clean_tuning_session.sh first or pass --csv." >&2
    exit 1
  fi
  CSV="$(cat "${OUT_ROOT}/.active_csv")"
fi

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Missing workspace setup file: ${WORKSPACE}/install/setup.bash" >&2
  exit 1
fi

SCRIPT_PATH="${WORKSPACE}/src/moleworks_ros/high_level_controllers/ocs2/mole_ocs2_arm_controller/scripts/mole_m4_matrix_autotune.py"
if [[ ! -f "${SCRIPT_PATH}" ]]; then
  echo "Missing matrix script: ${SCRIPT_PATH}" >&2
  exit 1
fi

mkdir -p "${OUT_ROOT}/summaries" "${OUT_ROOT}/artifacts"

CMD=(
  python3 "${SCRIPT_PATH}"
  --matrix "${MATRIX}"
  --csv "${CSV}"
  --output-dir "${OUT_ROOT}/summaries"
  --write-analyze-artifacts
  --analyze-artifacts-dir "${OUT_ROOT}/artifacts"
  --no-auto-apply
  --json
)

if [[ "${ENABLE_HARD_GATES}" -eq 0 ]]; then
  CMD+=(--disable-hard-gates)
fi

(
  source "${WORKSPACE}/install/setup.bash"
  "${CMD[@]}"
)
