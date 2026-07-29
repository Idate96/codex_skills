---
name: address-pr-review
description: Investigate and address pull-request reviewer feedback end to end. Use when asked to inspect a PR, assess whether reviewer suggestions are valid, compile evidence before replying, revise code after review, update a PR title or description, answer reviewer questions, or publish and verify a PR update.
---

# Address PR Review

Handle review feedback as an engineering task first and a writing task second.
Verify the premise, improve the solution, validate it, then communicate only
what the reviewer needs.

## Preserve the authorization boundary

- Treat inspection, implementation, and publishing as separate permissions.
- Keep all work read-only when asked only to investigate or explain.
- If told not to reply yet, do not post, edit the PR, push, or otherwise signal
  a response. Compile evidence and prepare a draft locally.
- Publish only after the user explicitly authorizes the PR update or reply.
- Never broaden authorization from one repository or worktree to another.

## Establish current state

1. Resolve the exact repository, worktree, branch, PR, base branch, and remote.
2. Read the applicable `AGENTS.md` and repository contribution instructions.
3. Inspect the worktree before editing; preserve unrelated and user-owned
   changes.
4. Refresh the live PR head, body, comments, reviews, and checks. Treat cached
   context and memories as historical hints only.
5. Restate the reviewer feedback internally as concrete questions and claims.

## Verify before deciding

- Inspect the relevant source, tests, and call paths rather than accepting or
  rejecting the review from intuition.
- Distinguish “the suggested alternative is generally possible” from “it is a
  drop-in replacement for this use case.”
- Identify the smallest behavioral contract that actually matters. Exclude
  internal training, product, or implementation details that do not affect the
  reviewer’s question.
- Express downstream needs through public fields and behavior. Remove
  task-specific object names, policy details, and internal terminology unless
  they are required to reproduce the problem or justify a scope boundary.
- Prefer a model-driven or repository-native design when it satisfies the
  requirement. Avoid private-field copying, carrier objects, compatibility
  layers, and speculative runtime APIs unless the use case truly requires
  them.
- If a construction-time decision removes a runtime requirement, state that
  directly and keep the runtime API out of scope.
- Preserve uncertainty when evidence is incomplete.

## Implement and validate

1. Change only the authorized repository and scope.
2. Add regressions for the failure modes raised in review, including subtle
   transforms, cached metadata, updates, or persistence when relevant.
3. Run focused tests first, then the repository’s required checks.
4. Record the exact commands and outcomes. Do not claim a command that was not
   run.
5. Use an independent reviewer only when requested or required by repository
   instructions; apply only findings verified against current code.

## Write external PR prose

- Write as Lorenzo in first-person singular unless he explicitly requests
  another voice.
- Never refer to Lorenzo as “the task author,” “the contributor,” or another
  third-person role in text posted from his account.
- Do not mention Codex or describe agent activity.
- Answer the reviewer’s actual question before summarizing the implementation.
- For a focused issue, prefer a short problem statement, minimal reproducer,
  expected behavior or precedence, consumer-facing field dependencies, and a
  clear scope boundary. Keep implementation design in the PR unless it is an
  acceptance criterion.
- Keep the reply concise and natural. Prefer three short paragraphs:

  1. Acknowledge the useful point and answer the scope or workflow question.
  2. Explain any remaining constraint with a few verified technical facts.
  3. State what changed, what was tested, and ask one focused follow-up if
     useful.

- Use bullets only when they materially improve a multi-item technical mapping.
- Remove internal arguments that do not change the reviewer’s decision.
- Avoid robotic inventory prose such as “There are no X, Y, or Z; the new
  regressions cover A, B, and C.” Prefer a direct sentence such as “I removed
  the manual copying and ran the focused conversion tests.”
- Avoid vague claims such as “fully tested,” “better,” or “should work.”
- Keep memory citations and private workspace details out of PR text.

## Update the PR

1. Recheck the diff and worktree.
2. Commit with the repository’s conventions and push only the intended branch.
3. Refresh the PR title and body so they describe the current implementation,
   not the abandoned approach.
4. Follow the repository PR template. Include behavior, motivation, scope,
   exact validation, and a minimal regression reference.
5. Post the concise reviewer reply.
6. Verify remotely:
   - the PR head matches the pushed commit;
   - the title and body contain the intended text;
   - the reply appears under Lorenzo’s account;
   - checks and new review comments have been inspected.
7. Treat required manual approval for external CI as pending, not as a test
   failure.

When editing a live PR or issue body, fetch the current text, require the
expected source text and a non-empty result before publishing, then read the
body back from GitHub. Never pipe an unchecked transformation into a remote
update.

If `gh pr edit` fails because of an unrelated GraphQL project-card query, use
the pull-request REST endpoint through `gh api --method PATCH`, then verify the
result with `gh pr view`.

## Completion gate

Finish only when the code and communication agree:

- the implementation addresses the verified concern;
- focused tests and required checks pass locally;
- the pushed SHA is the PR head;
- the current PR body and reply are live and accurate;
- remaining CI or reviewer action is reported precisely;
- unrelated local changes and unpublished worktrees remain untouched.
