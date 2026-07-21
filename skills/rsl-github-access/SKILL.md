---
name: rsl-github-access
description: Manage GitHub organization, team, and repository access for the ETH RSL `leggedrobotics` organization through the `rsl_github_access` CSV source of truth. Use when Lorenzo asks to add, restore, renew, change, or remove a GitHub team membership or repository role, or asks how current leggedrobotics access management works.
---

# RSL GitHub Access

Manage `leggedrobotics` access by editing the desired-state CSVs and opening a focused pull request. Treat local and generated memories as hints; refresh the repository and read its current instructions before changing access.

## Source Of Truth

- Local checkout: `/home/lorenzo/git/rsl_github_access`
- GitHub repository: `leggedrobotics/rsl_github_access`
- Team membership and organization retention: `access_team_membership.csv`
- Direct repository roles: `access_repo.csv`
- Current workflow and safety rules: `README.md` and `ADMIN.md`

Do not mutate GitHub teams or repository collaborators directly when the request belongs in this system. The merged CSV state is reconciled to GitHub by automation.

## Workflow

1. Read `/home/lorenzo/AGENTS.md`, then any repository-local `AGENTS.md`.
2. Verify `gh auth status`, the `origin` remote, the default branch, and the working-tree state.
3. Fetch `origin` and start from the refreshed remote default branch. Do not reuse a merged or stale access branch.
4. Resolve the exact GitHub login, target team slug or `owner/repo`, requested role, human ownership comment, and expiry date from current sources. Do not invent an expiry if it materially changes the request.
5. Search both CSVs and recent history for the user and target. This catches existing rows, expired access, renamed targets, and prior ownership context.
6. Make the smallest possible CSV change, normally only rows for one user.
7. Validate the file and inspect the complete diff before staging.
8. Commit, push, and open a draft PR against the verified default branch.
9. Inspect the PR dry-run workflow/comment. Confirm it plans only the intended grant, role change, renewal, or removal. Report unexpected removals instead of merging.

The dry-run comment may also list pre-existing expired entries and organization removals from the base CSV. Distinguish the requested `[CHANGE]` from those baseline expiry effects by comparing the PR diff and base branch. Call out collateral removal context even when the PR did not introduce it.

Follow the repository's branch convention when documented. At the time this skill was written, user access branches use `<yyyymmdd>/<target_user>` and PRs target `main`; always re-verify because this is operational state.

## CSV Contracts

Team membership rows:

```csv
github_user,target,comment,expire_date
```

Repository access rows:

```csv
github_user,target,role,comment,expire_date
```

Use the exact team slug and a fully qualified repository target such as `leggedrobotics/example`. Use only roles supported by the current parser. Dates use `YYYY-MM-DD`; an expiry remains valid on that date and expires afterward.

Team membership rows also protect users from organization removal. Treat removal or expiry of a user's last qualifying team row as potentially removing them from the organization, and inspect the dry-run carefully.

## Editing And Validation

- Preserve CSV headers and avoid blank lines.
- Quote comments containing commas or quotes according to CSV rules.
- Avoid `--make-csv-pretty` for a one-row request if it would rewrite unrelated rows.
- Stage only the intended CSV file when the worktree contains other changes.
- Run `git diff --check` and the repository's currently documented tests when available.
- Parse the edited CSV and assert the intended `(github_user, target)` occurs exactly once.
- If local tests are unavailable because of the checked-out branch or interpreter, state that plainly and rely on the PR workflow for the authoritative plan.

## Lorenzo Shortcut

Lorenzo Terenzi's GitHub login is `Idate96`. Use `Lorenzo Terenzi` as the ownership comment unless the current request or repository history gives more specific approval context. Never reuse a historical expiry without confirming it is still appropriate.
