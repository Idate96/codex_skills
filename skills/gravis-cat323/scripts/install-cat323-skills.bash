#!/usr/bin/env bash
# Run inside CAT323 Docker; restore user discovery links from the mounted catalog.
set -euo pipefail

workspace="${CAT323_WORKSPACE:-/workspaces/gravis_ws}"
agent_home="${CAT323_AGENT_HOME:-$HOME}"
legacy_home="${CAT323_LEGACY_HOME:-/home/lorenzo}"
source_dir="$workspace/codex_skills/skills"
[[ -d "$source_dir" ]] || { echo "Missing persistent skill catalog: $source_dir" >&2; exit 1; }
shopt -s nullglob
skill_files=("$source_dir"/*/SKILL.md)
(( ${#skill_files[@]} > 0 )) || { echo "No skills found in $source_dir" >&2; exit 1; }

for destination_dir in "$agent_home/.codex/skills" "$agent_home/.agents/skills"; do
  mkdir -p "$destination_dir"
  linked=0
  kept=0
  for skill_file in "${skill_files[@]}"; do
    skill_dir="${skill_file%/SKILL.md}"
    name="${skill_dir##*/}"
    destination="$destination_dir/$name"
    if [[ -L "$destination" ]]; then
      target="$(readlink "$destination")"
      if [[ "$target" == "$skill_dir" ]]; then
        linked=$((linked + 1))
        continue
      fi
      # Only replace the isolated handoff links that this CAT323 setup installed.
      if [[ "$target" == "$workspace/handoff/codex-skills/$name" ]]; then
        ln -sfn "$skill_dir" "$destination"
        linked=$((linked + 1))
        continue
      fi
      echo "KEEP unrelated skill link: $destination -> $target"
      kept=$((kept + 1))
    elif [[ -e "$destination" ]]; then
      echo "KEEP existing skill path: $destination"
      kept=$((kept + 1))
    else
      ln -s "$skill_dir" "$destination"
      linked=$((linked + 1))
    fi
  done
  printf 'CAT323 skills: %s linked, %s existing paths retained in %s\n' "$linked" "$kept" "$destination_dir"
done

# Older shared skills embed Lorenzo's paths. Add aliases only where absent.
for legacy_path in "$legacy_home/codex_skills" "$legacy_home/.codex/skills"; do
  if [[ -e "$legacy_path" || -L "$legacy_path" ]]; then
    continue
  fi
  mkdir -p "${legacy_path%/*}"
  if [[ "$legacy_path" == "$legacy_home/codex_skills" ]]; then
    ln -s "$workspace/codex_skills" "$legacy_path"
  else
    ln -s "$source_dir" "$legacy_path"
  fi
done
