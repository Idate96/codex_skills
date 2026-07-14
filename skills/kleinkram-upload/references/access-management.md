# Kleinkram Access Management

## Fallback Model

`manage_access.py grant-user` first attempts the legacy direct-user route. When that route is unavailable:

- with a trusted primary-group UUID, it can use `/projects/:uuid/access`;
- otherwise it reuses or creates a small project-specific custom group, adds the user, and grants that group access.

Primary access groups may be hidden from normal group search. Do not guess a primary-group UUID. Use `--fallback-group-name` only when a specific custom group name is required.

Rights map:

- `read` = `0`
- `create` = `10`
- `write` = `20`
- `delete` = `30`

Use `create` only when that collaborator level matches the user's request.

## Diego And `moleworks_arm_control`

- email: `digarcia@ethz.ch`
- user UUID: `5e4e0af7-0fca-4934-9470-070131c207e5`
- project UUID: `6b62504e-f6fd-4aa6-bacd-fc2a231ed761`
- verified fallback group: `moleworks_arm_control__diego`
- group UUID: `9596a4f5-02eb-4250-b2c4-4f20e4f4c891`
- expected right: `create` (`10`)

If Diego reports missing access, inspect the exact project ACL first. Reuse the existing verified group when present; otherwise dry-run and recreate the same narrow pattern. Do not grant all Moleworks projects from this shortcut.
