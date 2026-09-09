# TWP Fork of Klotho

This is TWP's fork of [`xpTURN/Klotho`](https://github.com/xpTURN/Klotho), consumed as the
`lib/Klotho` submodule of the TWP repo. See TWP decision **D31** and
`.kiro/specs/klotho-fork-transition/` in the TWP repo for the full rationale.

## Baseline

- Forked from upstream **v0.11.0** (commit `6d4f652`). TWP stays pinned to the v0.11.0 line for now;
  the v0.13.0 upgrade is a separate, deliberate decision (nav-focused, its own risk surface).
- `origin` = `https://github.com/StephenLujan/Klotho` (this fork, push/pull our work).
- `upstream` = `https://github.com/xpTURN/Klotho` (fetch upstream, merge).

## Branches

| Branch | Base | Purpose |
|---|---|---|
| `main` | tracks upstream | v0.13.0; not committed to — PR target base + merge source. |
| `twp/v0.11.0-base` | upstream v0.11.0 | **Integration branch the TWP submodule tracks.** v0.11.0 + minimal TWP commits (cherry-picked fixes + TWP-only repack commits). |
| `fix/litenet-disconnect-race` | upstream `main` (v0.13.0) | Clean single-fix PR head against upstream. Cherry-picked onto `twp/v0.11.0-base`. |

## Local commits (divergence from `upstream/main`)

| Commit (on `twp/v0.11.0-base`) | Kind | Purpose | Upstream PR |
|---|---|---|---|
| `2caf6f6` `fix(litenetlib-transport): guard Stop against concurrent Disconnect` | Upstreamable fix (cherry-picked from `fix/litenet-disconnect-race` @ `a8cdb8e`) | `_stopLock` capture-then-null in `LiteNetLibTransport.Disconnect()`/`Connect()` so concurrent/repeat stops can't orphan the manager's native receive thread. | PR against `xpTURN/Klotho` `main` — _link TBD_ |
| `6b74133` `chore(twp): repack dist addon with the Disconnect guard baked in` | **TWP-only** (never upstream) | Rebuilds the committed `dist/` core DLL + generator from source (`scripts/klotho-repack.sh`) so the guard is in the consumed binary; adds `dist/.pack-stamp` for the drift guard. | n/a |

## Build model (hybrid repack — Option A, D31)

The core is a committed prebuilt `dist/addons/klotho/lib/xpTURN.Klotho.Runtime.dll`, rebuilt from fork
source. After any change under `com.xpturn.klotho/Runtime/**` or the generator, re-run (from the TWP
repo root):

```
scripts/klotho-repack.sh          # gen.sh + pack-godot-addon.sh, then writes dist/.pack-stamp
scripts/klotho-drift-check.sh     # verifies dist/ matches source (fatal in CI: --ci / CI=true)
```

`dist/.pack-stamp` records a hash of the build inputs (`Runtime/**` minus Unity + vendored LiteNetLib,
the generator source, and the Packaging csprojs) so drift between the committed DLL and its source is
detectable.

## Merging upstream

1. `git fetch upstream`
2. `git checkout twp/v0.11.0-base && git merge upstream/<target>` (resolve any conflicts — kept cheap by
   minimal, branch-isolated divergence).
3. From the TWP repo root: `scripts/klotho-repack.sh` to rebuild `dist/`.
4. `dotnet test` from the TWP repo root — determinism/lockstep tests must pass.
5. Commit the updated submodule + `dist/`; update the TWP submodule pointer.

## Known follow-ups (not in this fork's scope)

- A separate CoreCLR debugger-lock-during-shutdown deadlock on the in-process server's `ServerLoop`
  thread (distinct from the LiteNetLib leak the guard fixed). Tracked in the TWP repo under
  `.kiro/specs/hang-on-session-teardown/` and its follow-up spec.
