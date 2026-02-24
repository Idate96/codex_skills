#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  upload_verify.sh --project <project-id-or-name> --mission <mission-name> --payload <payload-dir> [options]

Options:
  --project <value>      Kleinkram project id or name (required)
  --mission <value>      Kleinkram mission name (required)
  --payload <path>       Payload directory created by prepare_payload.py (required)
  --no-create            Do not create mission if missing
  --no-list              Skip final "klein list files"
  -h, --help             Show this help
EOF
}

PROJECT=""
MISSION=""
PAYLOAD=""
CREATE_MISSION=1
SHOW_LIST=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --project)
      PROJECT="$2"
      shift 2
      ;;
    --mission)
      MISSION="$2"
      shift 2
      ;;
    --payload)
      PAYLOAD="$2"
      shift 2
      ;;
    --no-create)
      CREATE_MISSION=0
      shift
      ;;
    --no-list)
      SHOW_LIST=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if ! command -v klein >/dev/null 2>&1; then
  echo "Missing required command: klein" >&2
  exit 1
fi

if [[ -z "$PROJECT" || -z "$MISSION" || -z "$PAYLOAD" ]]; then
  echo "Missing required arguments." >&2
  usage >&2
  exit 1
fi

PAYLOAD_DIR="$(realpath "$PAYLOAD")"
FILES_DIR="$PAYLOAD_DIR/files"

if [[ ! -d "$FILES_DIR" ]]; then
  echo "Payload files directory not found: $FILES_DIR" >&2
  exit 1
fi

mapfile -t FILES < <(find "$FILES_DIR" -maxdepth 1 -type f | sort)
if [[ "${#FILES[@]}" -eq 0 ]]; then
  echo "No files to upload in $FILES_DIR" >&2
  exit 1
fi

echo "project=$PROJECT"
echo "mission=$MISSION"
echo "files_to_upload=${#FILES[@]}"

if [[ "$CREATE_MISSION" -eq 1 ]]; then
  klein mission create -p "$PROJECT" -m "$MISSION" --ignore-missing-tags || true
fi

klein upload -p "$PROJECT" -m "$MISSION" "${FILES[@]}"
klein verify -p "$PROJECT" -m "$MISSION" --check-file-hash --check-file-size "${FILES[@]}"
klein mission info -p "$PROJECT" -m "$MISSION"

if [[ "$SHOW_LIST" -eq 1 ]]; then
  klein list files -p "$PROJECT" -m "$MISSION"
fi
