# Godot NavMesh Visualizer — User Guide

An editor tool that visualizes a serialized `FPNavMesh` (`.bytes`) in the Godot 3D viewport and lets you validate pathfinding and agent simulation.

> Target: `com.xpturn.klotho` Godot adapter · **Godot 4.x mono (.NET)** · editor-only (`#if TOOLS`)
> Related: [Navigation.md](Navigation.md) · NavMesh exporter (`Klotho: Export FPNavMesh`)

---

## 1. Prerequisites

1. **Godot mono (.NET) build** + an installed `dotnet` SDK.
2. The **Klotho addon must be enabled** (`Project > Project Settings > Plugins` → enable the Klotho plugin). The addon entry point is [`plugin.gd`](../com.xpturn.klotho/Godot~/plugin.gd).
3. The **C# solution must have been built once** (`Project > Tools > C#: Build`, or it builds on first run). The visualizer is a C# `[Tool]` class, so the assembly must be built for the menu to work.
4. A **3D scene must be open in the editor** — overlay geometry is attached as temporary nodes under the edited scene root. With no scene open the dock and info still appear, but 3D geometry is not shown (a warning is printed).
5. An input `.bytes` file — produced by the NavMesh exporter (`Klotho: Export FPNavMesh`, run with a `NavigationRegion3D` selected) at `<scene_dir>/<RegionName>.NavMeshData.bytes`. Sample: [`Samples/GodotPolySample/NavigationRegion3D.NavMeshData.bytes`](../Samples/GodotPolySample/NavigationRegion3D.NavMeshData.bytes).

---

## 2. Open / Close

- Click the top menu **`Project > Tools > Klotho: NavMesh Visualizer`** to toggle it.
- When on, an **`FPNavMesh`** dock appears on the right and the 3D viewport overlay and input become active.
- Clicking again removes the dock and overlay and stops intercepting input (the editor returns to default behavior).

> While the tool is off it does not intervene in 3D viewport input/drawing at all.

---

## 3. Loading a NavMesh (`NavMesh Data` section)

1. Enter the `.bytes` `res://` path in the text field (e.g. `res://NavigationRegion3D.NavMeshData.bytes`).
2. Click **`Load`** → after parsing, geometry is shown in the viewport and the vertex / triangle / grid / blocked / boundary & internal edge counts are shown as labels.
3. Click **`Unload`** to clear it.

> You can sanity-check load integrity by comparing the counts against the exporter's sidecar `.json` (e.g. `NavigationRegion3D.NavMeshData.json`).

> **A notice may appear beside the triangle count.** A mesh large enough that a flat A\* search can
> run out of budget gets a warning; one large enough only to have a path clipped gets an information
> line. Both are worded as *can*, not *will* — the condition is necessary, not sufficient. Nothing is
> installed on your behalf; the notice points at §7.1, where you can turn planning in legs on and
> compare. Being able to look at legs **off** is why the tool never enables them itself.

---

## 4. Visualization Layers (`Visualization Layers`)

Toggle on/off with checkboxes. Geometry layers are re-drawn immediately.

| Toggle | Shows |
|---|---|
| Triangles | Triangle fill (blue; blocked = red) |
| Edges / Boundary | Internal edges / boundary edges |
| Vertices | Vertex markers (line cross) |
| Tri Indices | Triangle-index labels (2D overlay) |
| Centers | Triangle center points |
| Blocked | Whether blocked triangles are highlighted |
| Cost Heatmap | `costMultiplier` gradient (green→red) |
| *(automatic)* Retained footprints | Triangles a **retained** building occupies are shaded apart from ordinary ground — walkable geometry stamped `FPNavMeshAreas.BUILDING_MASK`. Read off the mesh, so it needs nothing from the game; without it retain leaves no visual trace at all |
| Obstacle Rings | ORCA static-obstacle rings extracted from the boundary — flat XZ footprint with per-vertex convex/reflex markers and CW/CCW winding (the same data the runtime obstacle layer feeds to `FPNavAvoidance`) |
| Show node partition | Only while a graph is installed (§7.1). Colours the triangle fill by **node** instead of by area and outlines where one node ends and the next begins. Ground the graph does not cover — a retained footprint, a blocked triangle — is greyed out rather than given a node colour, and the rim around it is drawn in the same grey. Node ids are handed out afresh on every rebake, so placing a building recolours everything |

> Labels (Tri Indices · Cell Labels · agent `#i`) are drawn in 2D over the 3D view and appear once the camera is captured — i.e. **after the mouse has entered the 3D viewport once**. Labels are drawn only within a fixed distance (~40m) of the camera.

---

## 5. Pathfinding (`Pathfinding`)

1. Press one of **`Set Start`** / **`Set End`** / **`Inspect`** to enter that mode (press again to exit).
2. **`Shift` + left-click in the 3D viewport** → sets the point on the NavMesh.
3. Once both start and end are set, the path is **found automatically**; use **`Find Path`** to re-run manually and **`Clear Path`** to reset.
4. **`Area mask: all areas (through buildings)`** decides which mask `Find Path` uses. Off (the default) is `FPNavAgentSystem.DEFAULT_AREA_MASK`, which excludes the building area, so a retained footprint is a wall and the corridor routes around it; on is `FPNavMeshAreas.ALL_AREAS`, which plans straight through. Same mesh, opposite answers — toggling it re-runs the search immediately.
5. The result (corridor triangle count · waypoint count) is shown, with **Corridor / Waypoints / Portals** toggles to control the display.
6. In `Inspect` mode, Shift+click a triangle to show its details (vertex indices · neighbors · areaMask · cost · blocked · area) in the `Info` section.

---

## 6. Buildings (`Buildings`)

Places buildings on the loaded mesh and rebakes it — the same runtime rebaker a game drives, so what
the tool refuses is what the game would refuse. The shapes belong to the **tool**, not to any game: a
2×1 box and a hexagon.

1. Press **`Place Building`** to enter the mode, then **`Shift` + left-click** in the viewport to
   place one. The mesh is rebaked immediately and everything re-draws.
2. **`Retain (keep ground)`** picks the mode. Carved leaves a hole; retained keeps the footprint as
   walkable ground stamped as a building — the shaded triangles in the viewport, and the reason the
   agents' default mask treats it as a wall (see [Navigation.md § The area filter](Navigation.md#the-area-filter)).
3. **`Snap to tiling lattice (flush)`** puts the centre where footprints pack flush against each
   other. Those spacings are not round numbers, so a free-hand click otherwise leaves millimetres of
   walkable ground between two buildings.
4. **`Shape`** (Box / Hexagon) and, for the box only, **`Turn`** — the hexagon has no orientation in
   the catalog, because no integer hexagon is symmetric under 60°.
5. **`Boundary`** (`Reject` / `Touch` / `ClipOverlap`) and **`Allow contact`** are the placement
   rules. `Touch` is the tool's default rather than a game's `ClipOverlap`, because under
   `ClipOverlap` a **retained** footprint that crosses the walkable boundary is refused while a carve
   of the same footprint is clipped — worth seeing on purpose rather than by accident.
6. The list shows each placement as `#i  carve|retain  shape  (x, z)`; **`Remove last`** undoes one
   and **`Revert to loaded mesh`** drops them all.

> A refusal is normal use, not an error: pointing at ground you cannot build on prints
> `Refused: <reason>` under the list and the placement is not added. The reasons are the rebaker's
> own (`BuildingsOverlap`, `TouchesWalkableBoundary`, …) — see
> [Navigation.Rebake.md § 4.5](Navigation.Rebake.md#45-what-gets-rejected).

> The agent simulator follows the rebaked mesh: a placement pauses the simulation, swaps the mesh at
> a tick boundary and resumes it, so agents already walking re-plan against the new geometry. That is
> the same swap-then-reseed protocol a game performs — placing a building under a running crowd is
> exactly the scenario worth watching here.

## 7. Agent Simulation (`Agent Simulation`)

Drives the deterministic `FPNavAgentSystem` directly in the editor.

- **Playback**: `▶ Play` (toggles pause) · `Step` (1 tick) · `Reset` · current `Tick` readout.
- **`Sim Speed`** slider (0.25–4×). Advances on a fixed dt 1/60 accumulator.
- **Agent defaults**: `Speed` · `Radius` · `Accel` spinboxes, `Avoidance` (ORCA) checkbox.
- **Placement**: `Place Agent` mode + Shift+click to add; `Set Dest` mode + Shift+click to set the selected agent's destination.
- **Spawn by coordinates**: enter `x, y, z` on the two lines (start / destination), then `Spawn` (clears existing agents and spawns one). `Remove All` clears everything.
- **Selected agent's area masks**: two checkboxes — `plan: all areas` (what it may route through) and `walk: all areas` (what it may enter) — then `Apply masks to selected agent`. Pick the agent in the viewport first; the list row shows each agent's pair as `[plan/walk]`. Applying also drops the corridor the old masks planned, so the agent replans on the next tick.

  The pair worth setting is **plan on, walk off**: the path is drawn straight through a retained building and the agent walks into it and stops, showing `Blocked` in the list. Give it a destination **inside** the footprint — with both endpoints outside, A\* routes around a small footprint whichever mask it has, and the run proves nothing. Ticking `walk` afterwards and re-applying releases it. Note that `Spawn` clears every agent, so build a mixed pair with `Place Agent` instead. See [Navigation.md § The area filter](Navigation.md#the-area-filter) for all four combinations.
- **Display toggles**: `Agents` (disc) · `Paths` (corridor + corner lines) · `Velocity` (actual/desired velocity arrows) · `ORCA` (avoidance half-plane lines).

> While the simulation runs, dynamic meshes and labels are updated every tick. When the tool is inactive, ticks stop.

### 7.1 Planning in legs

A long move order can fail: one A\* search from a unit to a far destination is expensive and can give
up before it finds anything. Planning in legs cuts the walkable surface into **nodes**, picks a route
through those nodes first, then asks the real A\* for only the next hop. This section derives such a
graph over the loaded mesh and installs it into the simulated agent system, so you can watch the same
crowd with it on and off. See [Navigation.md § Planning in legs](Navigation.md#planning-in-legs) for
what a game does.

- **`Cell size`** — how wide a node is, in world units. Smaller means more, smaller nodes: shorter
  legs and a route that hugs the terrain more closely, at the cost of a slower derivation. 16 is the
  recommended starting point; on a coarse mesh 32 gives a shorter route for fewer nodes.
- **`Cost fold: Mean (else Min)`** — how the triangle cost inside a node folds into one edge cost.
  `Min` is optimistic and never overstates what the real path charges; `Mean` is closer to the real
  cost but gives that guarantee up.
- **`Derive for: all areas (else agent default)`** — the area mask the graph is built under. Leave it
  off to match ordinary agents, which route around retained buildings. **An agent whose own plan mask
  differs from this one silently keeps the flat path** — matching the two is how you watch that
  fallback disappear (`mask fallback` in the counters below).
- **`Apply legs`** derives and installs; **`Clear legs`** removes it. Derivation is not free — about
  10 ms on a 22k-triangle mesh — so it runs on the button and never on a field edit.
- A refusal is shown as `⚠ …` rather than thrown away. The usual one is a cell size so large that a
  leg through the widest node could not fit the corridor buffer; the message carries both numbers, so
  halve the cell and press again.

**What the readout says.** With a graph installed you get `Planning in legs — ON` and one line of
shape: node and edge counts, `widest node N hops -> up to M corridor tris (cap C)`, and the graph's
checksum. Two of those numbers count different things: **hops** across the widest node, and the
**triangles** a leg through it may ask for — the latter is what the cap is compared against.
`one node: every route ends inside it` means the cell size swallowed the mesh and legs change nothing
here.

**`Nav counters (since load)`** is how you tell it is working rather than merely installed:

| Counter | Read it as |
|---|---|
| `legs advanced` | legs completed and handed on. Rising steadily is the feature working |
| `legs that ended on the tick they were planned` | a **rate against the line above**. Near zero is healthy. Climbing together means the agents' turning radius (`Speed² / Accel`) is wider than a node, so every leg is "reached" the moment it is planned and each agent re-plans every tick — lower `Speed`, raise `Accel`, or apply a larger cell size |
| `abstract search failed` | no route between two nodes. If the mesh really is split, this is the map and not a defect |
| `leg unsolvable` | the node route named a crossing the real search could not reach; the agent falls back to the flat path |
| `mask fallback` | agents skipping the graph because their plan mask is not the one it was derived under (see `Derive for` above) |
| `corridor copy truncated` | **0 is the only correct value.** Anything else means the search and the agent's corridor storage were built from different caps |
| `corridor clamped` / `budget exhausted` | shared with the `Find Path` button, so pressing that moves them with no agent involved |
| `exhausted with nothing shortening the search` | the number legs exist to drive to zero — a search that ran out of budget with no graph shortening it |

> The counters accumulate from load, never reset, and are per-editor-session only — nothing here
> reaches the simulation state.

> Installing a graph changes where units walk, which is why a game must decide it deliberately: it
> moves that game's navigation fingerprint and its older recordings stop loading. Nothing about that
> applies to this tool — it installs into its own simulated system and touches no project file.

---

## 8. Spatial Grid / Info (`Spatial Grid` · `Info`)

- **Grid Lines / Cell Labels** toggles. The hovered cell is highlighted and `(col, row) - triangle count` is shown.
- **Info**: shows details of the selected (Inspect) or hovered triangle.

---

## 9. Limitations / Notes

- **PlayMode runtime agent visualization is not supported** — only editor simulation (based on the loaded `.bytes`) is provided.
- **Coordinate space**: raycasts and queries are in the simulation coordinate space. Picking is accurate only when the NavMesh was baked in the simulation coordinate space.
- **Steep walkable slopes (ramps)**: multi-floor traversal compares the **representative-height difference** of edge-adjacent triangles (`MultiFloorYThreshold`, default 2.0) — a difference above the threshold is treated as a separate floor and blocked. So a **single triangle with a large Y-span** (a ramp baked as one polygon) can exceed the threshold against the adjacent flat triangle and **the agent may fail to cross it**. In particular, if one ramp connects a lower and an upper flat area and its Y-span exceeds **about 2× the threshold**, no single representative height can be within the threshold of both neighbors, so one side is necessarily blocked. → Fix: bake with finer tessellation — lower the NavigationMesh **`edge_max_length` (recommended ≤ 3)** so the ramp splits into several triangles whose representative heights step gradually (`agent_max_slope` bounds the per-triangle rise, so this is safe in practice). *Diagnostic: if raising the dock's `Floor Y Thr` (an editor-simulation-only knob) makes it pass, this is the case.*

---

## 10. Troubleshooting

| Symptom | Check |
|---|---|
| Menu `Klotho: NavMesh Visualizer` is missing | Addon enabled + `C#: Build` run once? |
| Nothing shows in 3D after Load | Is a **3D scene open** (no scene → geometry not attached)? Is the path a valid `res://`? Is the `.bytes` non-empty? |
| `No triangles` / empty data | Did you **bake** the NavMesh before exporting? |
| Shift+click does nothing | Is the tool on? Is a mode button active? Is the click point on the NavMesh? |
| Labels not visible | Has the mouse entered the 3D viewport once (camera cache)? Within ~40m? Tri Indices / Cell Labels toggled on? |
| Agent can't cross a steep ramp/slope | Was the ramp baked as one large triangle? Lower the NavMesh `edge_max_length` (≤3) to subdivide and re-export (§9). *Diagnostic*: raise the dock `Floor Y Thr` (e.g. 5) — if it then passes, this is the case. |
| `Place Building` prints `Refused: …` | Normal — the rebaker refused that spot. `BuildingsOverlap` = it hits one already down; `TouchesWalkableBoundary` = it reaches the edge of the walkable region (and under `ClipOverlap` a **retained** footprint is refused there where a carve would be clipped); see [Navigation.Rebake.md § 4.5](Navigation.Rebake.md#45-what-gets-rejected). |
| A retained building looks like ordinary ground | It is ordinary ground — that is the point. Check the shading (§4) and inspect a triangle: a retained footprint's `areaMask` is `BUILDING_MASK`. Whether agents may enter it is the mask question, not the geometry (§5, §7). |
| `Apply legs` prints `⚠ Cell … gives a widest node of …` | Normal — that cell size makes a node too wide for the corridor buffer, so legs through it would come back clipped. Halve `Cell size` and press again (§7.1). |
| Legs are on but nothing looks different | Check `Nav counters`: `legs advanced` stuck at 0 with `mask fallback` climbing means the agents' plan mask is not the one the graph was derived under — match them with `Derive for` (§7.1). A single-node graph (`one node: every route ends inside it`) means the cell size swallowed the mesh; make it smaller. |
| Units jitter or re-plan every tick with legs on | `legs that ended on the tick they were planned` climbing in step with `legs advanced` is exactly that: the turning radius `Speed² / Accel` is wider than a node. Lower `Speed`, raise `Accel`, or apply a larger `Cell size` (§7.1). |
| Triangles/edges show but **Obstacle Rings** don't after Load | The boundary may be non-manifold — obstacle extraction is isolated in a try/catch, so the mesh still renders; check the Godot console for an `Obstacle ring extraction failed` warning. |

---

*Tool entry point: `Project > Tools > Klotho: NavMesh Visualizer` · implementation: [`com.xpturn.klotho/Godot~/Adapters/Editor/GodotFPNavMeshVisualizer*.cs`](../com.xpturn.klotho/Godot~/Adapters/Editor/)*
