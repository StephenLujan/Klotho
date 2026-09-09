# TWP Fork of Klotho

This is TWP's fork of [`xpTURN/Klotho`](https://github.com/xpTURN/Klotho), consumed as the
`lib/Klotho` submodule of the TWP repo. See TWP decision **D31** and
`.kiro/specs/klotho-fork-transition/` + `.kiro/specs/klotho-v0.14.0-upgrade/` in the TWP repo for the
full rationale.

## Baseline

- Tracks upstream **v0.14.0** (commit `2242a1c`). Upgraded from the original v0.11.0 transition
  baseline once all TWP-origin fixes had merged upstream (see below).
- `origin` = `https://github.com/StephenLujan/Klotho` (this fork, push/pull our work).
- `upstream` = `https://github.com/xpTURN/Klotho` (fetch upstream, merge).

## Branches

| Branch | Base | Purpose |
|---|---|---|
| `main` | tracks upstream | v0.14.0; not committed to — merge source + PR target base. |
| `twp/v0.14.0-base` | upstream v0.14.0 | **Integration branch the TWP submodule tracks.** v0.14.0 + this doc. All prior TWP fixes are now native in the v0.14.0 base, so the fork delta is effectively just this file. |
| `twp/v0.11.0-base` | upstream v0.11.0 | *Retained* prior integration branch (v0.11.0 + the transition-era fixes/repacks). History of the fork transition; no longer consumed. |

## Divergence from `upstream/main`

**As of the v0.14.0 baseline, the fork carries no source divergence** — every TWP-origin below-adapter
fix has been merged upstream and is present natively in v0.14.0. The only fork-only content is this
`TWP-FORK.md`.

### TWP-origin fixes (all merged upstream, now native in v0.14.0)

| PR | Title | State |
|---|---|---|
| [#10](https://github.com/xpTURN/Klotho/pull/10) | guard Stop against concurrent Disconnect | MERGED |
| [#11](https://github.com/xpTURN/Klotho/pull/11) | ServerLoop: don't Environment.Exit / hook process signals when embedded | MERGED |
| [#12](https://github.com/xpTURN/Klotho/pull/12) | stop the previous manager in Listen() | MERGED |
| [#13](https://github.com/xpTURN/Klotho/pull/13) | close the remaining manager-lifetime windows | MERGED |
| [#14](https://github.com/xpTURN/Klotho/pull/14) | release the shutdown watchdog on every exit path | MERGED |

Verified present in v0.14.0's shipped `dist/` core DLL (`_stopLock` + `_ownsProcess` both in
`dist/addons/klotho/lib/xpTURN.Klotho.Runtime.dll`).

## Build model (hybrid repack — Option A, D31) — currently DORMANT

TWP consumes the committed `dist/addons/klotho/lib/xpTURN.Klotho.Runtime.dll` via
`<Import Project="addons/klotho/Klotho.props" />`. On the v0.14.0 baseline this `dist/` is the
**upstream-shipped artifact** (not a TWP repack), because there is no TWP source divergence to bake in.

The repack tooling in the TWP repo remains for the next time TWP *does* patch core source below the
adapter layer:

```
scripts/klotho-repack.sh          # gen.sh + pack-godot-addon.sh, then writes dist/.pack-stamp
scripts/klotho-drift-check.sh     # verifies dist/ matches source (must run on Linux/LF + dotnet-on-PATH)
```

There is **no `dist/.pack-stamp` on this branch** — the drift guard only applies to a TWP-repacked
core. `klotho-drift-check.sh` treats a missing stamp as "repack needed," which is the correct signal
the day core source is changed. Do not stamp the unmodified upstream `dist/` — it would add a
maintenance point guarding nothing.

> Caveat (unchanged from the transition): `klotho-repack.sh`/`klotho-drift-check.sh` call bare `dotnet`
> and hash source content, so they must run in an LF, `dotnet`-on-PATH environment (Linux/CI, or WSL
> with a Linux .NET SDK). A Windows checkout is CRLF and would bake false hashes.

## Merging upstream (future)

1. `git fetch upstream`
2. `git checkout main && git merge --ff-only upstream/main` (fork main tracks upstream).
3. `git checkout twp/v0.14.0-base && git merge main` (or rebase this doc onto the new tag). While the
   fork delta is just this file, the merge is trivial.
4. If TWP has since added a custom core patch: `scripts/klotho-repack.sh` (Linux/LF), then
   `scripts/klotho-drift-check.sh`.
5. `dotnet test` from the TWP repo root — determinism/lockstep tests must pass.
6. Bump the TWP submodule pointer.

## History

- **Fork transition (v0.11.0)** — `.kiro/specs/klotho-fork-transition/` (D31). Established the fork,
  the source-built hybrid-repack model, and landed the Disconnect + ownsProcess fixes that are now
  upstream. Integration branch `twp/v0.11.0-base` (retained).
- **v0.14.0 upgrade** — `.kiro/specs/klotho-v0.14.0-upgrade/`. Moved the baseline to v0.14.0 once the
  fixes merged upstream; fork delta collapsed to this doc.
