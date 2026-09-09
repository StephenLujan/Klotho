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
| `2caf6f6` `fix(litenetlib-transport): guard Stop against concurrent Disconnect` | Upstreamable fix (cherry-picked from `fix/litenet-disconnect-race` @ `a8cdb8e`) | `_stopLock` capture-then-null in `LiteNetLibTransport.Disconnect()`/`Connect()` so concurrent/repeat stops can't orphan the manager's native receive thread. | [xpTURN/Klotho#10](https://github.com/xpTURN/Klotho/pull/10) — **MERGED** (head: `fix/litenet-disconnect-race`) |
| `6b74133` `chore(twp): repack dist addon with the Disconnect guard baked in` | **TWP-only** (never upstream) | Rebuilds the committed `dist/` core DLL + generator from source (`scripts/klotho-repack.sh`) so the guard is in the consumed binary; adds `dist/.pack-stamp` for the drift guard. | n/a |
| `566146b` `fix(server-loop): add ownsProcess flag; don't Environment.Exit / hook process signals when embedded` | Upstreamable fix | `ServerLoop` gains `bool ownsProcess = true`. When `false` (an embedded in-process loop), `GracefulShutdown()` does not arm the `Environment.Exit(1)` hard-timeout watchdog and `Run()` does not hook `Console.CancelKeyPress` / `AppDomain.ProcessExit`; the watchdog was also made cancellable so a clean dedicated-server shutdown no longer force-exits. Fixes an embedded host (TWP's in-process Single Player / Host server) being force-killed ~3s after session back-out. Default `true` keeps dedicated-server behaviour unchanged. | [xpTURN/Klotho#11](https://github.com/xpTURN/Klotho/pull/11) (head: `fix/server-loop-embedded-no-exit`) |
| `adf64d6` `chore(twp): repack dist addon with the ownsProcess ServerLoop fix` | **TWP-only** (never upstream) | Repack so the `ServerLoop` change is in the consumed `xpTURN.Klotho.Runtime.dll`; refreshes `dist/.pack-stamp`. | n/a |

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

> **Known caveats (verify on Linux):**
> - `klotho-repack.sh`/`klotho-drift-check.sh` call bare `dotnet` and hash source content, so they
>   must run in a **LF, `dotnet`-on-PATH** environment (Linux/CI, or WSL with a Linux .NET SDK). On a
>   Windows checkout the submodule source is CRLF, which makes `hash_inputs` diverge and
>   `klotho-drift-check.sh` report a **false** drift.
> - The committed `dist/.pack-stamp` from the `adf64d6` repack records `klotho_head=58b68d1`, which is
>   *older* than the `566146b` source fix that repack baked in. The DLL itself is correct (the
>   `ownsProcess` change is present and consumed), but the stamp's `input_hash` looks inconsistent with
>   HEAD source. **Re-run `scripts/klotho-repack.sh` on a Linux/LF box and commit the refreshed stamp**
>   to make `klotho-drift-check.sh` authoritative again. Not fixable from a Windows checkout (a Windows
>   repack would bake CRLF into the hash).

## Upstream status of our fixes

All TWP-origin below-adapter fixes have been merged into `xpTURN/Klotho`:

| PR | Title | State |
|---|---|---|
| [#10](https://github.com/xpTURN/Klotho/pull/10) | guard Stop against concurrent Disconnect | MERGED |
| [#11](https://github.com/xpTURN/Klotho/pull/11) | ServerLoop: don't Environment.Exit / hook process signals when embedded | MERGED |
| [#12](https://github.com/xpTURN/Klotho/pull/12) | stop the previous manager in Listen() | MERGED |
| [#13](https://github.com/xpTURN/Klotho/pull/13) | close the remaining manager-lifetime windows | MERGED |
| [#14](https://github.com/xpTURN/Klotho/pull/14) | release the shutdown watchdog on every exit path | MERGED |

PRs #12–#14 are **follow-on hardening merged upstream but not yet on `twp/v0.11.0-base`** — they land in
TWP when the fork next merges `upstream/main` (or on the v0.13.0 upgrade). The two fixes TWP currently
consumes (#10, #11) are already baked into the committed `dist/` core DLL.

## Merging upstream

1. `git fetch upstream`
2. `git checkout twp/v0.11.0-base && git merge upstream/<target>` (resolve any conflicts — kept cheap by
   minimal, branch-isolated divergence).
3. From the TWP repo root: `scripts/klotho-repack.sh` to rebuild `dist/`.
4. `dotnet test` from the TWP repo root — determinism/lockstep tests must pass.
5. Commit the updated submodule + `dist/`; update the TWP submodule pointer.

## Resolved follow-ups

- **The in-process `ServerLoop` freeze/exit on session back-out — RESOLVED** by the `ownsProcess`
  fix above. It was originally filed here as "a separate CoreCLR debugger-lock-during-shutdown
  deadlock," but that was a mis-attribution: the debugger only *amplified* it. The real cause was
  `ServerLoop.GracefulShutdown()`'s unconditional `Environment.Exit(1)` hard-timeout watchdog, correct
  for a dedicated server but fatal for TWP's embedded in-process loop. Full write-up in the TWP repo
  under `.kiro/specs/hang-on-session-teardown/investigation.md` + `.kiro/specs/hang-teardown-debugger-deadlock/`.
