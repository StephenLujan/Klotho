#!/usr/bin/env bash
#
# check-asset-sidecars.sh — every source file that a host editor imports must carry its sidecar.
#
# Unity refuses to see a .cs without a .cs.meta: the generated csproj simply omits the file, so
# the type it declares stops existing and the whole runtime assembly fails to compile. Nothing in
# the .NET build notices, because dotnet does not read .meta at all. That combination has already
# happened once: a new file under Runtime/Deterministic/Navigation was committed without its
# .cs.meta, 2,687 dotnet tests stayed green, and the Unity runtime assembly did not compile at all.
# This script is the check that would have caught it.
#
# Godot has the same shape with .cs.uid, which the addon pack and the sample copies carry.
#
# Usage: .github/scripts/check-asset-sidecars.sh
# Exit code: non-zero if any source file is missing its sidecar.
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

status=0

# --- Unity: com.xpturn.klotho/** needs .cs.meta, and so does every asmdef ---------------------
# Pruned: any directory whose name ends in '~' (Unity's own "do not import" marker — Godot~,
# Server~, Plugins~ are not Unity assets and carry .cs.uid or nothing), plus build output.
missing_meta="$(
  find com.xpturn.klotho \
    \( -type d \( -name '*~' -o -name 'obj' -o -name 'bin' \) \) -prune -o \
    \( -name '*.cs' -o -name '*.asmdef' \) -print 2>/dev/null |
  while read -r f; do
    [[ -f "${f}.meta" ]] || echo "${f}"
  done
)"

if [[ -n "${missing_meta}" ]]; then
  echo "Unity .meta missing — these files will be INVISIBLE to Unity:" >&2
  echo "${missing_meta}" | sed 's/^/  /' >&2
  echo >&2
  echo "A .cs without its .cs.meta is left out of the generated csproj, so the types it declares" >&2
  echo "vanish and the assembly stops compiling. dotnet builds and tests will not catch this." >&2
  echo >&2
  status=1
fi

# --- Godot: the adapter source and every shipped copy need .cs.uid ---------------------------
missing_uid="$(
  find com.xpturn.klotho/Godot~ dist/addons/klotho \
    \( -type d \( -name 'obj' -o -name 'bin' \) \) -prune -o \
    -name '*.cs' -print 2>/dev/null |
  while read -r f; do
    [[ -f "${f}.uid" ]] || echo "${f}"
  done
)"

if [[ -n "${missing_uid}" ]]; then
  echo "Godot .cs.uid missing:" >&2
  echo "${missing_uid}" | sed 's/^/  /' >&2
  echo >&2
  status=1
fi

if [[ ${status} -eq 0 ]]; then
  echo "asset sidecars are complete (.meta for Unity, .cs.uid for Godot)."
fi

exit "${status}"
