# codex_skills

Shared agent skills for **Codex** and **Claude Code**. One tree, both agents.

## Why there is no per-agent split

Codex and Claude Code use the *same* skill format: a directory containing a
`SKILL.md` whose frontmatter carries `name` and `description`. Nothing in this
repo needs reformatting for either agent.

So `skills/` is shared verbatim and symlinked into both agent directories.
There is deliberately no `codex/` or `claude/` copy — a second copy is how
skills drift apart, and it is what this layout exists to prevent.

```
codex_skills/
  install.sh
  skills/
    robot-ros/
      SKILL.md            <- read by BOTH agents
      agents/openai.yaml  <- Codex-only metadata; Claude ignores it
      references/         <- optional bundled docs
      scripts/            <- optional bundled scripts

~/.codex/skills/robot-ros   -> <repo>/skills/robot-ros
~/.claude/skills/robot-ros  -> <repo>/skills/robot-ros
```

`agents/openai.yaml` is Codex interface metadata. Claude Code ignores unknown
subdirectories, so it costs nothing to keep it alongside the shared `SKILL.md`.

## Setup on a new machine

```bash
git clone git@github.com:Idate96/codex_skills.git ~/git/codex_skills
cd ~/git/codex_skills
./install.sh
```

## Pulling updates

```bash
cd ~/git/codex_skills && git pull && ./install.sh
```

`install.sh` is idempotent. Because the agent directories hold symlinks rather
than copies, a plain `git pull` already updates every existing skill — re-running
`install.sh` is only needed to pick up **newly added** skills and to prune ones
that were deleted upstream. Running it always is harmless.

Restart Codex / Claude Code afterwards so they re-scan the skill directories.

Useful flags:

| Flag | Effect |
|---|---|
| `--dry-run` | print planned changes, modify nothing |
| `--codex` | only link into `~/.codex/skills` |
| `--claude` | only link into `~/.claude/skills` |

## Adding a skill

```bash
mkdir -p skills/my-skill
cat > skills/my-skill/SKILL.md <<'MD'
---
name: my-skill
description: What it does, and when an agent should reach for it.
---

# My Skill
...
MD
./install.sh          # links it into both agents
git add -A && git commit -m "Add my-skill" && git push
```

`description` is what each agent matches against to decide whether to load the
skill, so write it as *what it does + when to use it*, not just a title.

`install.sh` skips any directory without a `SKILL.md`, and never overwrites a
real directory sitting in an agent's skills folder — if it reports `KEEP`, move
that directory into this repo first, then re-run.

## Safety notes

- Only directories containing `SKILL.md` are linked.
- Broken links left behind by deleted skills are pruned automatically.
- Real (non-symlink) directories in the agent folders are never touched.
