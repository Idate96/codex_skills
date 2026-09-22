# Working from another computer

The website runs on Cloudflare independently of the original workstation. Editing requires the current source; deployment also requires Cloudflare access. The skill is instructions, not a source backup or an authentication credential.

## Transfer what is needed

Use a current portable source package or an up-to-date repository that actually includes the latest edits and media. On 22 September 2026, the latest edits were uncommitted/untracked and Git origin was a local bundle. A clone of that bundle alone would omit the latest site.

A useful portable package contains:
- Full current source and `public/` media, including untracked additions.
- `package.json`, `package-lock.json`, and build configuration.
- Non-secret `.openai/hosting.json` and `build/sites-vite-plugin.ts`, which the current build imports.
- Deployment `wrangler.jsonc` and `HOSTING.md`.
- This complete `moleworks/` skill folder.

Do not copy `node_modules`, `dist`, `.next`, `.wrangler`, `.env*`, browser profiles, or credentials. The current site has no configured D1/R2 application bindings (`d1` and `r2` are null in hosting metadata); no production database export is needed for the present static content and media. Recheck if the application changes.

A package produced on 22 September 2026 is a dated snapshot. Later edits must be transferred again. Do not overwrite a newer copy with it.

## Set up the destination

1. Extract the source into a project folder of your choice. Paths under `/home/lorenzo` are not required.
2. Install Node.js compatible with the lockfile (22.13+; 22.22.2 was verified).
3. Copy the `moleworks` skill directory into the destination Codex skills directory, normally `~/.codex/skills/moleworks`. Discover it in Codex and invoke `$moleworks`.
4. In the website folder, run `npm ci --no-audit --no-fund`, `npm run build`, and `npm run lint`. Use `npm run dev` for editing or `npm run start -- --port 4188` to preview a built version.
5. To publish, sign into the existing Cloudflare account with `npx wrangler login`, verify it with `npx wrangler whoami`, and deploy the generated `dist/server/wrangler.json` as described in `SKILL.md`.

No Hostpoint login or mailbox password is needed for ordinary website updates. Those are only relevant to explicit domain or mail changes.

## Keeping two computers in sync

A private hosted Git repository is a suitable next step if requested: include source, media (or an explicit large-file strategy), and deployment config, then commit and push changes before switching computers. The existing local bundle is not such a repository. Do not create a remote repository, change remotes, or publish source merely because cross-computer work was discussed.

Coordinate edits and deployments: whichever computer deploys last replaces the live site with its built source. Pull or transfer the latest changes before editing and deploying. A local development server affects only local preview; the Cloudflare deploy command updates production.
