# Deterministic Navigation

A deterministic NavMesh navigation system based on FP64. All computation runs on fixed-point arithmetic, guaranteeing synchronization across clients.

> **Engine scope**: the navigation **runtime** (`FPNavMesh`, `FPNavMeshQuery`, `FPNavMeshPathfinder`, `FPNavAgentSystem`, …) is engine-agnostic core — it runs unchanged on Unity, Godot, and the .NET server. The **baking + visualization editor tools** exist for **both Unity and Godot**: the geometry pipeline core (`FPNavMeshBuildPipeline`) is shared in Runtime, with an engine-specific editor exporter and visualizer on each side. Each engine bakes its **own** scene's NavMesh to a `.bytes` asset, which it then loads at runtime (Unity via `TextAsset.bytes`, Godot via `Godot.FileAccess.GetFileAsBytes`). Cross-engine `.bytes` sharing is **not** supported (coordinate-handedness differs — each engine bakes independently).

## Components

| Class | Role |
| ------ | ---- |
| `FPNavMesh` | NavMesh data: vertices, triangles, and a spatial grid |
| `FPNavMeshTriangle` | Triangle vertices / adjacency / portals / area / cost data |
| `FPNavMeshQuery` | Triangle lookup, height sampling, nearest-point queries |
| `FPNavMeshPathfinder` | A* search over the triangle graph (GC 0) |
| `FPNavMeshBinaryHeap` | A* open-set priority queue |
| `FPNavMeshFunnel` | SSFA (Simple Stupid Funnel Algorithm) — corridor → waypoints |
| `NavAgentComponent` | ECS agent component (`[KlothoComponent(11)]`; position / velocity / corridor / destination) |
| `FPNavAgentStatus` | Agent-status enum (Idle / PathPending / Moving / Arrived / PathFailed / **Blocked**) |
| `FPNavAgentSystem` | Path request → steering → ORCA avoidance → movement → NavMesh constraint → position-correction pass (operates on Frame + EntityRef[]); `LoadNavMeshObstacles()` registers the NavMesh boundary as ORCA obstacles and builds the graph-local obstacle-query CSR |
| `FPNavAvoidance` | ORCA (Optimal Reciprocal Collision Avoidance): agent-agent **and** static-obstacle half-planes (wall/cliff), general non-convex + winding-aware |
| `FPObstacleVertex` | Static-obstacle vertex (RVO2 `Obstacle` equivalent), flat-array ring representation (point / unitDir / isConvex / prev/next / polygonIndex) |
| `FPNavMeshObstacleExtractor` | Extracts ORCA obstacle rings from the NavMesh boundary (`neighbor == -1` edges); load/build-time only |
| `NavCorridorHelper` | Corridor-search and corridor-maintenance helper utilities (`SetCorridor` returns what it had to drop) |
| `FPNavTuning` | The navigation caps as an instance value — buffer sizes and loop budgets, handed to the five types above as an optional constructor argument (see [Tuning the caps](#tuning-the-caps-fpnavtuning)) |
| `FPNavPathFailure` | Names *why* an agent sits at `PathFailed` (`Diagnose`) and turns that into a label (`Describe`) — one runtime member both editor visualizers call |
| `FPNavMeshSerializer` | Binary serialization / deserialization |
| `FPNavMeshRebaker` | Runtime rebake: base mesh + building footprints → new `FPNavMesh` (see [Runtime Rebake](#runtime-rebake)) |
| `FPNavMeshAreas` | The area indices and masks the runtime reserves: `BUILDING_AREA` / `BUILDING_MASK`, `DEFAULT_AGENT_MASK`, `ALL_AREAS` |
| `FPNavMeshPlacementProbe` | Base mesh + an editable, ordered placement list, rebaked on demand — the data layer editor placement tools drive |
| `FPConstrainedDelaunay` | Deterministic constrained Delaunay triangulation on the exact integer grid |
| `FPBuildingShapeCatalog` | The set of footprints a game can place (integer offsets about the shape centre) |
| `FPConvexOffset` | Expands a convex footprint by the agent radius (integer miter, conservative) |

## File Layout

```text
com.xpturn.klotho/Runtime/Deterministic/Navigation/
├── FPNavMesh.cs              # NavMesh data (vertices, triangles, spatial grid)
├── FPNavMeshTriangle.cs      # Triangle struct (adjacency, portals, area, cost)
├── FPNavMeshQuery.cs         # Spatial queries (triangle lookup, height sampling)
├── FPNavMeshPathfinder.cs    # A* pathfinding
├── FPNavMeshBinaryHeap.cs    # A* priority queue
├── FPNavMeshFunnel.cs        # SSFA path smoothing
├── FPNavMeshSerializer.cs    # Binary serialization
├── FPNavMeshBuildPipeline.cs # Engine-agnostic bake pipeline (degenerate/T-junction/adjacency/grid)
├── FPNavMeshAreas.cs         # Reserved area indices and the masks built from them
├── NavAgentComponent.cs      # ECS agent component + FPNavAgentStatus enum
├── FPNavAgentSystem.cs       # Agent update system (Frame + EntityRef[]) + LoadNavMeshObstacles()
├── FPNavAvoidance.cs         # ORCA collision avoidance (agent-agent + static obstacles)
├── FPObstacleVertex.cs       # Static-obstacle vertex struct (flat ring)
├── FPNavMeshObstacleExtractor.cs # NavMesh boundary → ORCA obstacle rings
├── NavCorridorHelper.cs      # Corridor helper utilities
├── FPNavPathFailure.cs       # Why a path failed: reason enum + Diagnose/Describe
├── FPNavTuning.cs            # Per-instance caps (buffer sizes + loop budgets) and their validation
├── FPNavMeshRebaker.cs       # Runtime rebake orchestrator + snapshot/context/placement types
├── FPNavMeshRebakeBufferPool.cs  # Reusable rebake work buffers (per room)
├── FPNavMeshRebakeDriver.cs  # Derives the installed mesh from the frame; slicing + mesh cache
├── FPNavMeshRebakeSeam.cs    # What a game supplies the driver (placement table, install/reseed)
├── FPNavMeshPlacementValidator.cs # Command-path verdict from the driver's own derivation
├── FPNavMeshPlacementTableOps.cs  # The derivation both share (active set, canonical order, audits)
├── FPNavMeshPlacementProbe.cs # Editable placement list + rebake on demand (editor placement tools)
├── FPNavAgentInstaller.cs    # Swap/reseed call protocol, as two separate halves
├── FPConstrainedDelaunay.cs  # Deterministic constrained Delaunay triangulation
├── FPConstraintCrossingException.cs # Crossing constraint edges, named (see Navigation.Rebake.md)
├── FPBuildingShapeCatalog.cs # Shape table + radius-expanded derivative
├── FPConvexOffset.cs         # Convex footprint ⊕ agent radius (integer miter)
└── INavFingerprintSource.cs  # Cross-peer nav fingerprint hook

com.xpturn.klotho/Unity/Editor/NavMesh/   # Unity Editor (baking + visualization)
├── FPNavMeshExporter.cs          # Unity NavMesh → FPNavMesh conversion tool
├── FPNavMeshVisualizerWindow.cs  # NavMesh visualization editor window
├── FPNavMeshVisualizerData.cs    # Visualization state
├── FPNavMeshVisualizerStyles.cs  # Visualization styles
├── FPNavMeshSceneOverlay.cs      # Scene-view overlay rendering
├── FPNavMeshAgentSimulator.cs    # Agent-movement simulator
└── FPNavMeshInteraction.cs       # Click-to-navigate interaction

com.xpturn.klotho/Godot~/Adapters/Editor/ # Godot Editor (baking + visualization; #if TOOLS)
├── GodotFPNavMeshExporter.cs           # NavigationRegion3D → FPNavMesh conversion tool
├── GodotFPNavMeshVisualizer.cs         # Visualizer controller (plugin.gd forwards 3D virtuals)
├── GodotFPNavMeshVisualizerData.cs     # Visualization state
├── GodotFPNavMeshVisualizerDock.cs     # Dock UI (Control tree)
├── GodotFPNavMeshOverlay.cs            # Viewport overlay (ImmediateMesh)
├── GodotFPNavMeshAgentSimulator.cs     # Agent-movement simulator
├── GodotFPNavMeshInteraction.cs        # Shift+Click interaction
└── GodotFPNavMeshVisualizerStyles.cs   # Visualization styles
```

## NavMesh Pipeline

```text
Unity NavMesh / Godot NavigationRegion3D ──[exporter]──▸ .bytes file   ← bake: Unity or Godot Editor (each its own scene)
                                         │              ← load: any engine (Unity TextAsset / Godot FileAccess)
                            FPNavMeshSerializer.Deserialize()
                                         │
                                         ▼
                                     FPNavMesh
                                         │
    ┌────────────────────────────────────┼────────────────────────────────┐
    ▼                                    ▼                                ▼
FPNavMeshQuery                  FPNavMeshPathfinder                FPNavMeshFunnel
(triangle lookup, height)        (A* corridor build)             (corridor → waypoints)
    │                                    │                                │
    └──────────────┬─────────────────────┴────────────────────────────────┘
                   ▼
           FPNavAgentSystem.Update(ref Frame, EntityRef[] ...)
           ┌──────────────────────┐
           │ 1. ProcessPathRequest│ ─▸ A* + Funnel
           │ 2. ProcessSteering   │ ─▸ Seek + Arrive
           │ 3. ORCA Avoidance    │ ─▸ agent-agent + static-obstacle (wall) avoidance (graph-local select)
           │ 4. ProcessMovement   │ ─▸ acceleration, speed clamp, position update
           │ 5. ConstrainToNavMesh│ ─▸ boundary-edge sliding
           │ 6. ResolveCollisions │ ─▸ position-space overlap push (gated on avoidance)
           └──────────────────────┘
```

## Core Data Structures

### FPNavMesh

| Field | Type | Description |
| ---- | ---- | ---- |
| `Vertices` | `FPVector3[]` | 3D vertices (Y = height, XZ = plane) |
| `Triangles` | `FPNavMeshTriangle[]` | Triangle array with adjacency info |
| `BoundsXZ` | `FPBounds2` | Total XZ bounds |
| `GridCells` | `int[]` | Spatial-grid cells (start, count pairs) |
| `GridTriangles` | `int[]` | Triangle indices referenced by cells |
| `GridWidth/Height` | `int` | Grid dimensions |
| `GridCellSize` | `FP64` | Cell size |
| `GridOrigin` | `FPVector2` | Grid origin |
| `BakeAgentRadius` | `FP64` | Agent radius the source was baked with (bake settings block) — consumed as `ObstacleRadiusInset` |
| `BakeMaxSlopeDeg` | `FP64` | Max walkable slope baked with (bake settings block) — graph-local query auto-derives its climb cap |
| `BakeAgentHeight` / `BakeAgentClimb` | `FP64` | Recorded bake settings block (no runtime consumer) |

### FPNavMeshTriangle

| Field | Type | Description |
| ---- | ---- | ---- |
| `v0, v1, v2` | `int` | Vertex indices |
| `neighbor0/1/2` | `int` | Neighbor triangles (-1 = boundary) |
| `portal*Left/Right` | `int` | Funnel-portal vertex indices |
| `centerXZ` | `FPVector2` | Precomputed centroid (A* heuristic) |
| `area` | `FP64` | Triangle area |
| `areaMask` | `int` | Area membership, one bit (`1 << areaIndex` from the bake). A query passes an allowed-area *set* and a triangle is walkable to it where the two intersect — see below |
| `costMultiplier` | `FP64` | Cost multiplier (1.0 = default) |
| `isBlocked` | `bool` | Dynamic-block flag |

### NavAgentComponent

A `[KlothoComponent(11)]` ECS component. Holds the corridor with `unsafe` + `fixed` buffers, GC-free.

| Field | Type | Description |
| ---- | ---- | ---- |
| **Configuration** | | |
| `Speed` | `FP64` | Max speed (default: 5) |
| `Acceleration` | `FP64` | Acceleration (default: 10) |
| `AngularSpeed` | `FP64` | Max angular speed (default: 360) |
| `Radius` | `FP64` | Agent radius |
| `StoppingDistance` | `FP64` | Stopping distance |
| `PathRepathCooldown` | `FP64` | Re-pathing cooldown |
| **Runtime State** | | |
| `Position` | `FPVector3` | Current position |
| `Velocity` | `FPVector2` | Current velocity (XZ) |
| `DesiredVelocity` | `FPVector2` | Desired velocity (steering / ORCA result) |
| `CurrentSpeed` | `FP64` | Current linear speed |
| **Path (corridor)** | | |
| `Corridor[128]` | `int (fixed)` | Triangle corridor (`MAX_CORRIDOR`) |
| `CorridorLength` | `int` | Effective corridor length |
| `PathTarget` | `FPVector3` | Final path target |
| `PathId` | `int` | Path identifier |
| `PathIsValid` | `bool` | Path validity |
| **Destination / Triangle** | | |
| `Destination` | `FPVector3` | Destination |
| `HasNavDestination` | `bool` | Whether a destination is set |
| `HasPath` | `bool` | Whether a path exists |
| `CurrentTriangleIndex` | `int` | Currently occupied triangle |
| **Internal Counters** | | |
| `LastRepathTick` | `int` | Last re-path tick |
| `PathRequestId` | `int` | Path-request ID |
| `OffCorridorTicks` | `int` | Off-corridor tick counter |
| `Status` | `byte` (`FPNavAgentStatus`) | Idle / PathPending / Moving / Arrived / PathFailed / Blocked |
| **Area masks** | | |
| `PlanAreaMaskOverride` | `int` | What this agent may ROUTE through. **0 = no override** → `FPNavAgentSystem.DEFAULT_AREA_MASK` |
| `WalkAreaMaskOverride` | `int` | What this agent may ENTER. Same 0 rule. Assign both through `SetAreaMask`; see [the area filter](#the-area-filter) |

Initialization is performed via the static method `NavAgentComponent.Init(ref nav, startPosition)`, which leaves both area-mask overrides at 0.

## Usage

Inside an ECS system, batch-process entities holding `NavAgentComponent` via `FPNavAgentSystem.Update`.

```csharp
// Bootstrap — load NavMesh and wire up the system (e.g., in a RegisterSystems hook)
byte[] data = navMeshAsset.bytes;                                   // Unity (TextAsset)
// Godot: byte[] data = Godot.FileAccess.GetFileAsBytes("res://Data/NavMesh.bytes");
FPNavMesh navMesh = FPNavMeshSerializer.Deserialize(data);          // same binary format on both engines

var query      = new FPNavMeshQuery(navMesh, logger);                      // logger may be null
var pathfinder = new FPNavMeshPathfinder(navMesh, query, logger);
var funnel     = new FPNavMeshFunnel(navMesh, query, logger);
var navSystem  = new FPNavAgentSystem(navMesh, query, pathfinder, funnel, logger);
// Each of the five also takes an optional FPNavTuning last — see Tuning the caps.

// Enable ORCA avoidance (optional)
navSystem.SetAvoidance(new FPNavAvoidance());
// Register the NavMesh boundary as static obstacles (wall avoidance). Call AFTER SetAvoidance.
// On all peers/server for a deterministic match (see Static Obstacles below).
navSystem.LoadNavMeshObstacles();

// Spawn an agent — attach NavAgentComponent to an ECS entity
var entity = frame.CreateEntity();
frame.Add(entity, new NavAgentComponent());
ref var nav = ref frame.Get<NavAgentComponent>(entity);
NavAgentComponent.Init(ref nav, new FPVector3(FP64.FromInt(1), FP64.Zero, FP64.FromInt(1)));
nav.Destination       = new FPVector3(FP64.FromInt(9), FP64.Zero, FP64.FromInt(1));
nav.HasNavDestination = true;

// Per-tick update (inside ISystem.Update) — collect entities with NavAgentComponent and batch-process
navSystem.Update(ref frame, entities, entityCount, currentTick, dt);
```

### The area filter

`areaMask` is a per-triangle **area membership** — the bake writes `1 << areaIndex`, so exactly one
bit — and a query passes the **set of areas it accepts**. A triangle is walkable to that query where
the two intersect (`(queryMask & tri.areaMask) != 0`); `~0` (`FPNavMeshAreas.ALL_AREAS`) accepts
everything. The agent system passes `FPNavAgentSystem.DEFAULT_AREA_MASK` =
`FPNavMeshAreas.DEFAULT_AGENT_MASK`: every area except the runtime's **building area** (index 1,
`FPNavMeshAreas.BUILDING_AREA`), which the rebaker stamps onto retained building footprints — so
agents treat those as walls while a caller passing `ALL_AREAS` plans through them.

**An agent can name its own masks, and it names two.** `NavAgentComponent.PlanAreaMaskOverride`
decides what it may ROUTE through and `WalkAreaMaskOverride` what it may ENTER; assign them through
`NavAgentComponent.SetAreaMask`, which also drops the corridor the old masks planned. **Zero means
"no override"** and resolves to `DEFAULT_AREA_MASK`, so an agent that names nothing behaves exactly
as it did before these fields existed — zero as a literal mask would be total paralysis, and zero is
what a `default(NavAgentComponent)` carries. Setting the plan mask permissively and the walk mask
restrictively is the interesting combination: the path is drawn straight through a retained building
and the unit walks into it and stops, i.e. it plans as if it did not know the building was there.
That stop is reported as `FPNavAgentStatus.Blocked` — see the movement contract below.

**A destination ON a boundary between allowed and forbidden ground belongs to the allowed side.**
Such a point is genuinely in both triangles — `PointInTriangle2D` is tolerant, and the surface is
continuous across a shared edge, so the interpolated heights are bit-identical there — and it is the
answer `ProjectToPassable` gives, since the projection returns the closest point on a triangle
*edge*. `FindPath` therefore resolves its **endpoint** with a lookup that breaks height ties toward
ground the mask allows; without it, whichever triangle carried the lower index won, and a snapped
destination was refused about half the time with nothing in the log to say so.

The tie-break is deliberately narrow: it swaps only between candidates on the **same surface**, i.e.
whose interpolated height at that point is equal. Equal height *distance* is not enough — two floors
are equidistant whenever the agent's y is the midpoint between them, and swapping there would move
the destination a storey away rather than reinterpret it. So a destination on a forbidden upper floor
is still refused: the walkable floor below is a different place. And where every same-surface
candidate is allowed — all of ordinary ground, every interior edge on it included — "first allowed"
is "first", so the answer is identical to the plain lookup. That bounds the change to meshes carrying
forbidden ground, and to destinations on its boundary.

The **start** keeps the plain lookup: its mask check is an exemption whose value is being reported
(`DebugMaskedStartCount`), and resolving an ambiguous start toward allowed ground would silence it
at exactly the positions it exists to report.

There are four pairs and three of them mean something:

| `PlanAreaMaskOverride` | `WalkAreaMaskOverride` | What the agent does at a retained footprint |
|---|---|---|
| `0` / `DEFAULT_AGENT_MASK` | `0` / `DEFAULT_AGENT_MASK` | **Routes around it.** The behaviour before these fields existed, and what every agent does until something assigns them. A destination *inside* the footprint is refused outright: `PathFailed`, counted in `DebugAreaMaskRejectedCount` |
| **`ALL_AREAS`** | **`DEFAULT_AGENT_MASK`** | **Plans through it and stops on contact.** The corridor is the shortest route as if the building were not there; the walk refuses to enter, so the agent arrives at the edge and reports `Blocked`. This is the pair the two masks exist for |
| `ALL_AREAS` | `ALL_AREAS` | **Walks through it.** Plans and enters — a unit the building does not obstruct |
| `DEFAULT_AGENT_MASK` | `ALL_AREAS` | Nothing observable. The plan already routes around, so the walk permission is never exercised |

Only the middle two need a permissive plan mask, and that is the load-bearing half: **an agent whose
plan mask excludes buildings never touches one**, so giving it a permissive walk mask changes nothing
and it can never report `Blocked`. If you are trying to observe the stop, the plan mask is what has
to be widened.

Area 1 is
therefore reserved, and `FPNavMeshBuildPipeline.Build` refuses a bake that uses it. `int` is the
ceiling, so 32 areas. The Unity exporter forwards `NavMeshTriangulation.areas` verbatim, so Unity's area types
are the indices; the Godot exporter emits area 0 for every triangle.

Both halves of the engine's movement apply it: `FindPath` filters the start triangle, the end
triangle and every expansion, and `FPNavMeshQuery.MoveAlongSurface`/`MoveAlongSurfaceWithVisited`
filter every expansion of the surface walk. Two contract points are worth knowing before you narrow
a mask:

- **Neither half gates the ground the agent is already on, and both let it leave.** The walk never
  tests the starting triangle, `FindPath` exempts a masked-out start, and — the part that makes the
  promise real — a neighbour the mask refuses is still expandable **when the triangle being expanded
  from is refused too**, in the walk and in the A\* alike. So the guarantee is "does not walk **into**
  what the mask forbids", not "cannot walk out of it": narrow a mask under an agent's feet and it
  crosses the forbidden region and leaves. Entry from accepted ground is unchanged, and the rule
  cannot carry an agent between two separate forbidden regions — crossing accepted ground resets it,
  so the reachable set is exactly *the forbidden component you stand in, plus the accepted ground it
  touches*.

  Exempting only the start was not enough, and the reason is worth knowing before you narrow a mask:
  a single small building footprint is a handful of triangles that all touch ordinary ground, but two
  snapped flush leave interior triangles with **no accepted neighbour at all**. An agent there had a
  start exemption and nowhere to use it. `FindPath` reports a masked-out start in
  `DebugMaskedStartCount` rather than refusing the call, because otherwise nothing in the result says
  the unit was ever inside a building.

- **The destination is still gated.** Only the start is exempt: a path *into* ground the mask forbids
  is refused as before and counted in `DebugAreaMaskRejectedCount`. That asymmetry is the whole
  contract — you may leave, you may not enter.
- **A rejected neighbour is a wall of the same class as `isBlocked`**, so the agent slides along its
  edge. Starting from ground the mask **accepts**, a mask that forbids every neighbour pins the agent
  where it stands; starting from ground it **refuses** it does not, because the escape rule opens those
  neighbours. The matching `FindPath` runs the same start rather than refusing it and reports it in
  `DebugMaskedStartCount` — `DebugAreaMaskRejectedCount` is the end-point counter and is silent about
  the start.
  The walk does **not** tell its caller which term refused a neighbour — `isBlocked`, the mask and
  the multi-floor threshold all take the same wall path, and the mask is re-tested there only to log
  it, so the production path pays nothing for the distinction.
- **`FPNavAgentStatus.Blocked` names the stop that follows.** When an agent's corridor leads into
  ground its own walk mask refuses and it has run into it, the agent system stops it there and says
  so. Three conditions are required — the agent **asked to move**, the walk moved it nowhere, AND the
  corridor's next triangle is refused by this mask — because each alone misfires: a real wall also
  stops the walk, "not advancing" is the normal state of an agent part-way across a triangle, and an
  agent held still by a crowd asked for nothing and has touched nothing. So `Blocked` arrives on the
  tick an agent pushes against the edge, not on the tick its velocity reaches zero. Without the status the
  stall is invisible: such an agent keeps `Moving` and a valid path, and the off-corridor repath
  never fires, because standing still inside the corridor's current triangle counts as being *on*
  the corridor.
  *"The corridor's next triangle"* means the one after **this agent's own index** in the corridor. An
  agent that is **off** its corridor gets no verdict at all: the corridor's head is not its next
  step, and the off-corridor repath — not a terminal status — is the answer for it.
- **Leaving `Blocked` is mostly the game's decision, with one engine-side exception.** Widen the mask
  (`SetAreaMask` releases the agent), retarget it, or destroy what is in the way; the engine does not
  decide who believes what. The exception is the event that can make the block untrue on its own: a
  **navmesh swap**. `ReseedAgents` hands a `Blocked` agent that still has a destination back to the
  planner (`PathPending`, cooldown bypassed), so demolishing the building that stopped a unit starts
  it moving again on the next tick. `Arrived` and `PathFailed` are deliberately left alone.
  While an agent is `Blocked` nothing recomputes its status, but the position-correction pass still
  moves it: crowd pressure can press it along the footprint edge — never into the footprint, because
  that pass carries the agent's own walk mask.

Non-uniform areas do **not** survive a runtime rebake: the rebake feeds the pipeline an all-zero
`areas` array and then copies the base mesh's uniform `areaMask` over every triangle, logging an
error and falling back to the pipeline default if the base is non-uniform. Per-region reassignment
needs the area polygons preserved as assets, which is a follow-up. A mask stamped through
`FPNavMesh.TrianglesMutable` is lost the same way. The one runtime producer that survives is the
rebaker's own: after the inherit it stamps retained building footprints
`FPNavMeshAreas.BUILDING_MASK` (exclusively), on every rebake and on the patch path too, and the
stamp is part of `ComputeFingerprint`.

`areaMask == 0` on a triangle makes it unreachable to every query including `~0`, since no bit can
intersect. The bake never produces one; `TrianglesMutable` can.

#### Which lookups filter, and which deliberately do not

`FindPath` and `MoveAlongSurface` filter. The "nearest walkable point" family comes in **two
flavours**, and picking the wrong one is the mistake this split exists to prevent:

| Unfiltered | Filtered | Ask the filtered one when |
| ---- | ---- | ---- |
| `FindTriangle(xz)` | `FindPassableTriangle(xz, mask)` | *may this point be used* |
| `ClosestPointOnNavMesh` | `ClosestPassablePoint` | *where may I stand instead* |
| `ProjectToNavMesh` | `ProjectToPassable` | *snap me somewhere usable* |
| `FindTriangle(xz, agentY)` | `FindPassableTriangleForEndpoint(xz, y, mask)` | *may this point be used, on this floor* |

The last row is the multi-floor pair, and the difference matters exactly where floors stack:
`FindPassableTriangle` is y-blind, so above a walkable ground floor it answers "usable" for a click
on forbidden ground upstairs. Ask the endpoint form whenever a height is part of the question.

Separate from both is **`FindTriangleForEndpoint(xz, y, tieBreakMask)`** — the rule `FindPath` itself
uses to resolve a destination, described under [the area filter](#the-area-filter). It is not a
filter: it always answers with a triangle if the point is on the mesh, and only *breaks a tie*
toward allowed ground. A tool that wants to report what the engine decided has to call this one, or
it names a refusal the engine never made.

**The unfiltered members are not deprecated.** They are the right answer wherever the question is
*where is this point* rather than *may I go there* — `FindPath` finds its endpoints unfiltered so it
can attribute an `isBlocked` or `areaMask` refusal to its own counters instead of reporting
"off-mesh", the post-swap agent reseed keeps an agent inside forbidden ground on a valid triangle so
the walk's escape rule has something to fire from, and an editor picking a triangle under the cursor
wants whatever is there.

That split is also how a caller expresses *"can't enter, can leave"* for a position, which is what
the walk's escape rule does for movement: ask the **unfiltered** lookup where you are, and only when
it answers nothing ask the **filtered** projection where to go. Filtering both steps projects the
position out of a footprint while the caller's triangle index still names it — a pair the walk is
not defined for. The Brawler sample's position re-snap is written that way.

One consequence worth planning for: `ProjectToPassable` reports failure when the nearest passable
point is beyond `maxDist` and impassable ground is nearer, where the unfiltered member returned an
unusable point. Callers that treat failure as "stop" stop more often near retained buildings.

## Static Obstacles (ORCA)

`FPNavAvoidance` adds RVO2 static-obstacle half-planes on top of agent-agent avoidance, so agents steer around walls/cliffs (not just each other). Obstacle lines are **hard constraints** — `LinearProgram3` never relaxes them (only agent lines are relaxed), so velocity never leaks through a wall.

**Two obstacle sources (same flat format):** `FPVector2[] vertices + int[] polygonOffsets` fed to `FPNavAvoidance.LoadObstacles(...)`:

1. **NavMesh boundary** (Unity/Godot) — `FPNavMeshObstacleExtractor.Extract(navMesh, out verts, out offsets)` walks the boundary edges (`neighbor == -1`), orienting each by the interior (opposite) vertex — no triangle-winding assumption — and chains them by rotating around each shared vertex through the walkable fan (handles pinch/bowtie vertices). Outer boundaries come out clockwise, holes counter-clockwise, automatically. `FPNavAgentSystem.LoadNavMeshObstacles()` wraps extract + load using the system's own NavMesh.
2. **Convex polygon rings** — e.g. procedurally baked terrain blocks — passed directly to `LoadObstacles`.

Convex and non-convex (reflex) rings are both handled (the RVO2 non-convex leg arms are present). Convex CCW input exercises only the convex paths.

**Winding convention:** free (walkable) space is on the **right** of each edge's `unitDir`. Solid blocks (agents outside) are wound CCW; walkable-boundary loops (agents inside) CW. Callers guarantee this — the NavMesh extractor by construction, procedural sources by their gate.

**Determinism:** obstacle data is **per-peer local static** — it is *not* in the wire/frame/state hash. All peers load the same baked geometry, so every peer extracts an identical obstacle set and computes identical avoidance velocities. In server-driven (SD) mode the **client and server must both** call `LoadNavMeshObstacles()` — a one-sided wiring makes `ComputeNewVelocity` diverge and desyncs. `FPNavAgentSystem.DebugObstacleCount` is a setup-time diagnostic (0 while avoidance is set ⇒ missing wiring or a boundary-free mesh). Load is once per match (tick-independent), idempotent-replace across stage changes; the hot path stays GC-0.

**An agent standing exactly on an obstacle corner is a normal position, not an edge case.** Agent
coordinates and obstacle ring vertices live on the same snap lattice, and a runtime rebake can drop a
new hole ring where an agent already stands, so the segment test treats the closed end of a segment
as a hit — both ends of a segment agree, which matters because every shared ring vertex is one
segment's end and the next one's start.

**Graph-local obstacle selection (multi-floor / ramp):** `FPNavAgentSystem` picks obstacle candidates by walking the NavMesh adjacency (BFS) outward from the agent's current triangle rather than scanning every loaded segment, so only walls on the agent's own floor/ramp are considered — a stacked upper floor whose walls overlap in XZ no longer yields phantom obstacles. Expansion is gated by a per-edge step-delta floor test, a padded XZ range, and a climb cap: on a mesh that records its bake slope (`FPNavMesh.BakeMaxSlopeDeg`) the cap auto-derives as `obstRange·sin(slope)`, otherwise `MaxClimbWithinHorizon` (∞ by default). Only *which* segments are selected changes; the half-plane math stays XZ-2D and GC-0. An unlocalized agent (seed triangle `-1`) falls back to the brute-force scan.

> **Radius inset / clearance (`ObstacleRadiusInset`):** a baked NavMesh boundary is inset from the real wall by the bake **Agent Radius** (Unity min 0.05, **0 not allowed**; Godot per-resource). Since ORCA also holds the agent its own radius off the obstacle line, a naive setup double-counts clearance — agents stop ~`bakeRadius + simRadius` from walls, blocking tight corridors. `FPNavAvoidance.ObstacleRadiusInset` cancels the baked inset: the effective obstacle radius is `max(0, agent.Radius − ObstacleRadiusInset)`. `LoadNavMeshObstacles()` auto-sets it to the asset's recorded `BakeAgentRadius` (bake settings block), so under the `R_sim = R_bake` convention the effective radius is 0 and the boundary itself is the constraint (matching the point-agent funnel path, which hugs corners). Per-peer local config — auto-riding the asset keeps lockstep peers symmetric; consumers may override after the load. Keep the baked mesh clean (a fragmented mesh yields spurious obstacle rings).

## Runtime Rebake

Everything above assumes the NavMesh is fixed. It does not have to be: a building placed during a
match can be carved out of the walkable region and the rest re-triangulated, deterministically, on
every peer. A footprint then blocks only the space it covers rather than the whole corridor triangle
it happens to sit in.

The pieces live in this folder — `FPNavMeshRebaker` orchestrates, `FPConstrainedDelaunay` does the
triangulation on the exact integer grid, `FPBuildingShapeCatalog` holds the footprints a game may
place, `FPConvexOffset` expands them by the agent radius. The result is an ordinary `FPNavMesh`,
indistinguishable from a baked one, which you install with `FPNavAgentSystem.SwapNavMesh(...)` —
that rebinds the query, pathfinder and funnel.

**Installing it is a second decision, not a detail.** The swap does *not* reseed the agents; that
call is yours and it is not optional, because every agent still holds a triangle index and corridor
into the mesh you just replaced and both are hashed frame state. And in a rollback netcode you must
not install where the command was handled at all: the rewind returns the frame and leaves the
NavMesh ahead of it. `FPNavMeshRebakeDriver` owns that problem — it re-derives the installed mesh
from frame state every tick, keeps the reseed on the tick that owns it, spreads a large rebake across
frames, and keeps the two meshes a predicting peer bounces between. **Registering it as a system is
the whole wiring**: the engine paces its slices and re-derives the mesh at world init and after a full
state applies, so a game writes a placement table and an installer and nothing else.

**[Navigation.Rebake.md](Navigation.Rebake.md)** is the guide: footprint types, the placement grid,
what gets rejected, the determinism envelope, installing from a command stream, and measured costs.

## Cross-peer identity: the navigation fingerprint

`FPNavAgentSystem.GetNavFingerprint()` is the value peers compare to find out whether they will
navigate alike **before** the match starts, and it folds three things:

| Term | Answers |
| ---- | ---- |
| the mesh's content hash | *same stage* |
| `NAV_BEHAVIOUR_REVISION` | *same pathfinding* — a hand-bumped constant standing for what `FindPath` returns for unchanged inputs |
| `FPNavTuning.Digest` | *same caps* — see [Tuning the caps](#tuning-the-caps-fpnavtuning) |
| `FPNavTuning.PartialPathDigest` | *same partial-path switch* — its own term, zero when off, so a tuning that never names it keeps the fingerprint it had (see [Partial paths](#partial-paths-when-the-budget-runs-out)) |

Nothing new goes over the wire: the Ready exchange already compares an **environment fingerprint**
that this value is folded into (in both P2P and server-driven modes), and the FullState resync's
static-geometry check reads the same term. Two peers on builds that plan different corridors, or
tuned differently, now differ **there** — before the match — rather than desyncing later with nothing
pointing at navigation. `0` still means "no navigation registered", so a peer
without a mesh is never reported as a mismatch, and the digest is normalised so that
`FPNavTuning.Default` contributes nothing — the common path's fingerprint is what it always was.

**A replay is checked against it.** `StartReplay` throws `InvalidDataException` for a recording whose
`NavFingerprint` disagrees with this process — a different stage, or a build that plans different
corridors — before playback starts and before `OnGameStart` fires, instead of loading the file and
drifting with nothing to say where. Two situations are *not* refusals. A `0` on either side keeps its
"not provided" meaning: a recording without the anchor is played, and a process that reports none
logs a warning saying the check did not run (usually a navigation system that was never registered).
And a recording that started mid-match (`InitialStateTick != 0`) is warned rather than refused,
because its anchor describes the rebaked mesh at that instant rather than the base asset playback
loads. This is attribution, not integrity: a forged file rewrites the anchor along with the payload.

**Bumping the revision is a manual step, and the rule is narrow.** The constant is `internal` to the
package, so this is an engine-side edit rather than a game-side one. Bump it for any edit that moves
what `FindPath` returns for unchanged inputs — the endpoint or start lookup, the A\* order or budget,
the funnel, the surface walk — and not for refactors or diagnostics. Nothing enforces the bump; the
sites most likely to move it carry a comment pointing back at the constant.

## NavMesh Export & Visualization *(Editor)*

Both engines ship an editor exporter and visualizer; they share the geometry pipeline (`FPNavMeshBuildPipeline`) and produce the same `.bytes` binary format.

**Unity** — `Tools > Klotho > Export NavMesh` exports the current scene's Unity NavMesh; visualization is at `Tools > Klotho > Visualizer > NavMesh`.

**Godot** — `Project > Tools > Klotho: Export FPNavMesh` exports the selected `NavigationRegion3D` (output: `<scene_dir>/<RegionName>.NavMeshData.bytes` + `.json` sidecar); the visualizer toggles at `Project > Tools > Klotho: NavMesh Visualizer` — see [NavMeshVisualizer.Godot.md](NavMeshVisualizer.Godot.md).

**Both visualizers can also place buildings on the loaded mesh and rebake it**, carved or retained,
and then run the agent simulator over the result — including per-agent plan/walk masks, so two agents
that differ only in their walk mask can be watched splitting at a footprint edge. The shapes belong
to the tool, not to a game. The data layer behind it is the runtime's `FPNavMeshPlacementProbe`, so
both editors drive one code path; the Godot side is documented in
[NavMeshVisualizer.Godot.md](NavMeshVisualizer.Godot.md).

**A destination click is snapped to ground the agent may plan through.** Clicking a *retained*
building used to hand `SetDestination` the raw point: the footprint is on-mesh but carries
`BUILDING_MASK`, so `FindPath` refused the endpoint by mask and the agent sat at `PathFailed` with a
destination and no corridor — on screen, "it has somewhere to go and will not move". Both
visualizers now ask two questions with the agent's resolved **plan** mask
(`FPNavAgentSystem.ResolvePlanMask`, public for exactly this). First *is the click usable as
clicked* — `FindPassableTriangleForEndpoint`, the height-aware form, so a click on forbidden ground
above walkable ground is not waved through by the floor beneath it. If it is not, the click goes
through `ProjectToPassable` and Y is resampled at the snapped XZ (keeping the click's own y would
pair a moved XZ with an unrelated height, which on a multi-floor mesh lands on the wrong floor). When
no passable ground is in range the destination is *not* set: the agent is stopped (so the status
reads `Idle`, not `PathFailed`) and the reason is shown.

The reach is set by **`Dest Snap Max`**, defaulting to the mesh's `GridCellSize`. That default is the
distance the projection *always* covers — its fallback searches the click's cell plus the 8 around
it, so anything within one cell is certain to be considered. Larger values still help, but whether
they do depends on where in its cell the click landed, and past the 3×3 block's far corner
(~2.83 cells) there is nothing left to find. Raising the knob therefore cannot rescue a click deep
inside a footprint wider than the cell ring; the refusal message says so.

**The agent row names why a path failed.** `PathFailed` covers five different situations, so
`FPNavPathFailure.Diagnose` re-walks `FindPath`'s guard chain against the mesh as it stands and
reports which one: the agent is off the mesh, it stands on blocked ground, the destination is off
the mesh or blocked or outside the agent's plan mask, the failure is *stale* (the mesh changed and
nothing blocks it any more — set the destination again), or the search itself failed.
`FPNavPathFailure.Describe` turns that into the suffix the row prints. Handed the tool's **own**
pathfinder, `Diagnose` splits that last case by searching again from where the agent stands (a
`PathFailed` agent does not move): a drained open set is `NoRoute`, a spent budget is
`BudgetExhausted` — which, with [partial paths](#partial-paths-when-the-budget-runs-out) on, means
the closest point the budget reached was no closer than the agent already stood. Without a
pathfinder the verdict stays the undivided `NoRouteOrBudget`. The pathfinder's *cumulative* counters
are still not read for this: the tool shares one pathfinder with its own Start/End path preview so
a lifetime delta cannot be attributed (the re-search takes its own before/after inside one call,
which can), the counters never reset, and the case worth explaining most — an agent a rebake left
off-mesh — never calls `FindPath`, so no counter ever moves for it.

Both editor tools call the same member. It is public runtime API rather than editor-local for the
reason `FindTriangleForEndpoint` is: Unity's tool is one fixed assembly, but the Godot adapter ships
as source inside `addons/klotho/Adapters/` and compiles into whatever assembly the consuming project
happens to be, so there is no `InternalsVisibleTo` list to write. The `StaleFailure` vs
`NoRouteOrBudget` split is the one thing the caller must supply — whether the failure was already
standing when the current mesh was installed is bookkeeping only the tool has.

Shared bake steps:
- Vertex welding (WELD_EPSILON = 0.001)
- Degenerate-triangle removal + T-junction split
- Automatic adjacency + portal build
- Spatial-grid build (default cell size = 4.0)

> Each engine bakes its **own** scene; the resulting `.bytes` is not interchangeable across engines (coordinate-handedness). Load it on the engine that produced it.

> **Steep walkable slopes**: multi-floor traversal compares the **representative-height difference** of edge-adjacent triangles (`MultiFloorYThreshold`, default 2.0); a difference above the threshold is treated as a separate floor and blocked. A ramp baked as **one large triangle** whose Y-span exceeds ~2× the threshold therefore cannot stay within the threshold of *both* its lower and upper neighbour, so one side becomes impassable. Bake with finer tessellation (Godot NavigationMesh `edge_max_length` ≤ ~3; `agent_max_slope` bounds the per-triangle rise) so a ramp splits into triangles whose representative heights step gradually.

## Crowd Scaling

`FPNavAgentSystem.Update(ref frame, entities, count, tick, dt)` is **not engine-scheduled**. The game
collects the array and calls it, so who is in it, in what order, and how many calls a tick takes are
all decisions you already own. That is the whole lever this section is about — no engine change is
involved in any of it.

### What a move order costs

Measured with `FPNavAgentCrowdPerfTests` (`[Explicit]`, Release, warmup 32) on a 96×96-cell open
field — 192×192 world units, 18,432 triangles — with agents on a 1.5-unit lattice. Median ms per
tick, .NET 8 on one desktop machine: read the **shape**, not the absolute numbers, and re-measure on
your target before budgeting against them. **These are flat numbers**: the harness asks for no
abstract graph (`NavAgentTestHelper.NoAutoGraph`), because since 0.13 the agent system would
install one on this mesh by itself and the storm below would not exist — that is the point of the
[section on legs](#planning-in-legs). Re-measured under the 0.13 defaults (partial paths on).

| Agents | A* storm tick | ORCA + correction | path follow + movement | `Frame.CopyFrom` |
|---:|---:|---:|---:|---:|
| 64 | 42.5 | 0.49 | 0.33 | 0.009 |
| 256 | 257.0 | 2.40 | 1.34 | 0.009 |
| 800 | **983.5** | 12.65 | 4.22 | 0.027 |
| 3200 | **4408.0** | 163.89 | 17.71 | 0.111 |

Two things in that table are worth more than the rest:

**The order tick dominates everything else by orders of magnitude.** Steady-state following at 800
agents costs ~16 ms; the tick where all 800 receive the order costs ~970 ms. The repath cooldown
(`PathRepathCooldown`, default 10 ticks) does not help — it only gates agents that have *already*
repathed, so an idle army all fires on the same tick.

**Most of that second is spent running out.** The diagnostic counters say so directly: across the
measured runs at 800 agents, 11,200 searches produced 11,200 corridor clamps and 4,658 budget
exhaustions — every single search hit the corridor cap, and the ~42% that exhausted
`MAX_ITERATIONS` got a **partial path** (before 0.13 they got no path at all and sat in
`PathFailed` with nothing logged; the exhaustion counts are identical, the outcome is not). A
cross-map order on a field this size is past the built-in ceiling, and before these counters
existed that fact was invisible from outside the engine.

| Agents | searches | corridor-clamped | budget-exhausted (partial path) |
|---:|---:|---:|---:|
| 64 | 896 | 896 | 0 |
| 256 | 3,584 | 3,584 | 351 |
| 800 | 11,200 | 11,200 | 4,658 |
| 3200 | 44,800 | 42,078 | 26,992 |

The two counters overlap since 0.13: an exhausted search returns the corridor it has, and that
corridor is clamped to the buffer like any other, so a search can count in both columns. Before
partial paths the columns were disjoint (an exhausted search returned nothing to clamp).

So the first thing to build for hundreds of units is not a faster A* — it is **not calling A* for
everyone at once**: a bounded admission queue (promote K destinations per tick), and for
same-destination groups a flow field, which needs nothing from the engine that is not already public
(see below).

### What spreading the load does not do

Both levers in this section — the admission queue and the cluster split below — **spread** work.
Neither makes a search cheaper, so neither touches the failures. Be clear about that before
budgeting around them:

- **The ~42% that exhaust the budget still exhaust it.** They run out for the same reason whenever
  they run, and walk a partial path to re-plan from. Admitting them over ten ticks produces ten
  ticks of exhaustions instead of one.
- **The admission queue does not fit a 10 Hz tick either.** Spread over ten ticks, the A* share is
  ~98 ms — but by the last of those ticks every already-admitted unit is moving, so the steady-state
  ~17 ms runs alongside it: **~115 ms against a 100 ms budget**, before any game logic or physics.
- **The cluster split does not help here at all.** It reduces the steady-state tick by ~10 ms, which
  is **1% of the order tick**. It is worth doing for the reasons in the next section; relieving a
  mass order is not one of them.

**`MaxIterations` is two things since 0.13**: the search budget, and the threshold above which the
agent system installs an abstract graph on its own (`AutoInstallAbstractGraph`). Lowering it turns
legs on for more meshes; raising it — to 65536, say — turns automatic legs off for a 22,000-triangle
stage without saying so. There is deliberately no separate threshold: the budget is the exact
condition under which a flat search can fail, and a second number would drift from it.

**Raising `MaxIterations` is the wrong direction**, and it is worth saying because `FPNavTuning` now
makes it reachable. The searches that exhaust the budget are the *expensive* ones; giving them a
larger budget converts fast failures into slow successes and raises the total. The measured average
of ~1.2 ms per agent is already a blend of successes and exhaustions, so it understates what an
exhausting search costs. If cross-map orders on a large map are a requirement, the answer is a
cheaper *kind* of search — planning in legs over an abstract graph, or a flow field for grouped
destinations — not a bigger budget for the same one.

### Splitting the array into clusters

Calling `Update` once per spatial cluster turns the ORCA neighbour scan from O(N²) into O(Σnᵢ²).
Measured at 800 agents, steady state:

| Cluster size | Calls | Median ms | Agents actually position-corrected |
|---:|---:|---:|---:|
| 16 | 50 | **6.35** | 800 |
| 32 | 25 | 7.30 | 800 |
| 64 | 13 | 9.94 | 800 |
| 128 | 7 | 10.54 | 448 |
| 256 | 4 | 11.57 | 256 |
| 800 (single call) | 1 | 16.28 | **64** |

Smaller clusters win on both axes at once: the tick gets cheaper *and* more agents come out of the
correction pass separated, because the agent cap is a per-call cap — 13 calls of 62 correct all 800,
while one call of 800 corrects 64 and drops the other 736 silently. (That cap is also settable per
system now — see [Tuning the caps](#tuning-the-caps-fpnavtuning) — but raising it trades against the
pass being O(iterations · n²), which is the cost the split avoids in the first place.) At 3200 agents the same split is
5.1× cheaper (152.0 → 30.0 ms).

**It is not lossless, and the cost is not in the table.** In `ComputeNewVelocity` the update set *is*
the neighbour-candidate set, so agents stop seeing each other across a cluster boundary and avoidance
is cut there. Nothing in the engine can express "update this agent, but consider that one as a
neighbour" today. Put boundaries where density is low, prefer larger clusters where crowding matters,
and treat the sweep above as a cost curve to trade against quality — not as a recommendation to
cluster as small as possible.

### A worked partitioner

`FPNavAgentClusterSplitSampleTests` is the rule above as code you can copy — sort agents by
`(cell z, cell x, entity index)`, break a run at every cell change and again at `MAX_AGENTS`, and
call `Update` once per run. Three details in it are load-bearing:

- **The cell index is a shift, not a division.** Pick the cell size as a power of two in world
  units and the index is `position.RawValue >> (FP64.FRACTIONAL_BITS + log2)` — exact integer
  arithmetic, and a shift floors correctly on negative coordinates, so an agent at `x = -0.1` lands
  in cell −1 instead of sharing cell 0 with `x = +0.1`.
- **The sort key ends in the entity index**, which makes it a total order. Leave ties open and the
  sort implementation decides the array order — and the array order is simulation input.
- **Break runs at cell boundaries first, and at the cap second.** Cutting the sorted sequence every
  `MAX_AGENTS` agents instead merges whole cells into one call whenever cells are smaller than the
  cap, and that produces clusters *wider* than plain index chunking (measured on the fixture's
  layout: worst cluster bounding box 89 vs 45 world units², where breaking at cells gives 9.7).
  The cap is a ceiling, not a target.

The cell size is the only dial: cells hold whatever local density puts in them, so a smaller cell
means more, smaller calls. It is also what the enumeration-order property rests on — the partition
is a function of positions and entity ids, so a peer that walks its entities in another order still
produces the same runs in the same order and converges. Index chunking does not: the fixture pins
both, the spatial rule converging and index chunking diverging under a reversed input array.

### The partition is a determinism input

Not a local optimisation. ORCA breaks coincident ties on **array index**, and the correction pass
takes the **first `MAX_AGENTS`** of whatever it is handed, so changing the partition changes the
simulation. Two peers that cluster differently diverge, and both results look internally consistent.

- **Required**: the rule is a pure function of frame state (e.g. sort by grid cell, then by entity
  index). `FPSpatialGrid.GetPairs` sorting after cell traversal is the idiom this repo already uses.
- **Forbidden**: wall-clock, local input, hash-map iteration order, camera or visibility.
- **Partition, not overlap**: an agent in two arrays moves twice in one tick.
- **A path-admission cursor lives in frame state or nowhere.** `FPNavAgentSystem` is not an
  `ISnapshotParticipant`, so a round-robin cursor kept in a system field is not restored on rollback
  and resimulation diverges. Derive it from the tick (`index % K == tick % K`) or keep it in a game
  component.

Multiple `Update` calls in one tick are safe: no scratch state carries between calls (the only
instance state that survives is visit-stamp generations and diagnostic counters, and neither affects
results).

### Diagnostic counters

`FindPath` returns a bare `false` for five different reasons and logs only one of them (an endpoint
off the mesh). The counters below name the other four, and the correction-pass cap besides — all as
diagnostic fields outside the state hash, the wire and replay:

| Counter | Owner | Reports |
|---|---|---|
| `DebugCollisionResolveTruncatedCount` | `FPNavAgentSystem` | Agents dropped from the correction pass past `MAX_AGENTS` |
| `DebugCorridorTruncatedCount` | `FPNavMeshPathfinder` | Paths clamped to the corridor cap (the far end is dropped, so the agent runs off the corridor and repaths) |
| `DebugCorridorCopyTruncatedCount` | `FPNavAgentSystem` | Triangles dropped while copying a planned corridor into an agent. **0 is the only correct value** — the search and the storage share one cap, so a nonzero count means they were built from different ones |
| `DebugIterationExhaustedCount` | `FPNavMeshPathfinder` | Searches that failed with work still queued — the budget ran out, as opposed to there being no route |
| `DebugBlockedEndpointCount` | `FPNavMeshPathfinder` | Start or end triangle flagged `isBlocked` — the search never started |
| `DebugAreaMaskRejectedCount` | `FPNavMeshPathfinder` | The requested `areaMask` shares no bit with the **end** triangle. `FindPath` only — the surface walk applies the same filter but treats a rejection as a wall rather than a failure, so it has nothing to count |
| `DebugMaskedStartCount` | `FPNavMeshPathfinder` | The search **started** on ground the mask forbids. Not a failure: the start is exempt and the escape rule carries the path out, so this is the only trace that the agent was inside a building at all |

They accumulate over the owning instance's lifetime and are **not** rollback-aware: a resimulated
tick is counted again. They are never reset — take two readings if you want a delta. Across a
navmesh swap the two overloads differ: `SwapNavMesh(mesh)` rebinds the existing pathfinder, so the
pathfinder totals carry across a runtime rebake (this is the path `FPNavAgentInstaller` uses), while
the four-argument overload installs a caller-built one and starts its totals over.
`DebugCollisionResolveTruncatedCount` lives on the agent system and survives either. `DebugIterationExhaustedCount` deliberately keys off the open set rather than
the iteration count, so a search that drains its frontier exactly at the budget is reported as "no
route", not as a truncation.

`DebugBlockedEndpointCount` is worth watching in particular, because it counts the one silent failure
a game creates **deliberately**. Nothing in the engine sets `isBlocked` — the runtime rebaker carves
geometry away rather than blocking it, and a destination on carved ground is *off-mesh*, which logs.
The flag has exactly one producer: `FPNavMesh.TrianglesMutable`, the door this API opens for closing
a gate at runtime. So a rising count means units are being ordered through something the game shut,
and until now that produced the same wordless `false` as "there is no route".

`DebugAreaMaskRejectedCount` is a **plan-side** instrument: it counts **end**-triangle refusals
inside `FindPath` and nothing else, so an agent whose PLAN mask admits buildings never trips it
however often its walk is refused — that half is what `FPNavAgentStatus.Blocked` is for. It moves
through `FPNavAgentSystem` in exactly one case: a **destination** inside a **retained building
footprint**, which `DEFAULT_AREA_MASK` excludes (see the area filter above). A masked-out *start* no
longer refuses the call and no longer lands here — it is reported in `DebugMaskedStartCount`
instead. On a mesh without retained buildings both count direct callers of the pathfinder only.

Note that a non-zero `DebugCollisionResolveTruncatedCount` is only *observable* for
position-authoritative consumers: the correction pass writes `NavAgentComponent.Position` and nothing
else, so an integration driving the character from `Velocity` (the Brawler sample does) never sees
its effect either way.

### Flow fields stay on the game side

For same-destination groups the answer to the A* storm is one Dijkstra pass over the triangle graph
plus a per-triangle next-hop, and everything that needs is already public: `FPNavMesh.Triangles` with
`neighbor0..2`, `centerXZ`, `isBlocked`, `costMultiplier` and `areaMask`. It stays out of the core
because the cost function and grouping policy differ per game, and anything the core owns becomes a
**state-hash input** the game can no longer change.

Flyers have no off-mesh link concept — keep them out of the `entities` array entirely and steer them
directly.

### Planning in legs

Everything above spreads the order tick around; none of it makes a search cheaper, which is why the
failures survive it. The one thing that removes them is planning a **cheaper kind of search**: cut
the walkable surface into nodes, hop nodes to pick a direction, and hand the real A\* only the
current leg. A leg never leaves its node, so it never runs out of budget and never overruns the
corridor buffer — the two caps stop binding instead of being raised.

Measured on the same 96×96-cell field as the table above, 800 agents receiving one order — one
run, one day, cell 16 (the flat row is the same harness with no graph; the counters are run
totals):

| | order tick | got no path | budget exhausted | corridor clamped |
|---|---:|---:|---:|---:|
| flat | 982.7 ms | 0 (partial paths since 0.13; 358/800 before) | 3,226 | 8,000 |
| **in legs** | **45.8 ms** | **0** | **0** | **0** |

**A hierarchical route walks the straight line to within measurement.** A single unit crossing the
field at 27° walks 0.995–1.000× the straight-line distance at cell 16 and 0.996–1.000× at cell 32,
against 0.995–1.004× for the flat search on the same routes (the arrival threshold at the
destination is why both sit a hair under 1). Travel time tracks distance rather than exceeding it, and a unit
changing legs a dozen times neither stalls nor circles at the boundaries. Until 0.14 this
paragraph said 1.10× at worst; what changed is below.

**Measure the ratio off the diagonal.** The first measurements of this were all taken at exactly
45°, which on a square lattice is the one heading where the route steps cleanly through node
centres — the detour there is close to constant, so it dilutes over distance and the ratio looks
better the further you go. Off the diagonal a per-crossing component does not dilute, and that is
where the detour lived: priced centre to centre, 27° routes walked 1.06–1.10× however long they
were. Three things take it to 1.00:

- The abstract search prices its first and last hop from **where the agent actually stands and
  where it is actually going**, not from the centres of the nodes those points sit in. An agent
  that has just changed legs is on its node's boundary, so a portal chosen from the centre would
  sit off the line it has to walk.
- A leg aims **along the portal** rather than at its midpoint, at the point that makes the crossing
  straightest given the portal after it. How much this is worth depends on how coarse the mesh is:
  a portal is one triangle edge, so a fine triangulation leaves little room to slide along it.
- The hops **between** the two ends are priced the same way (0.14): from the portal a node is
  entered by to the portal it is left by, not centre to centre. That distance is looked up from a
  table derived with the graph — one row per portal of a node, straight-line for a pair the walk
  between is clear, and the length of the walk around whatever is in the way for a pair it is not
  (the same edge-midpoint cost the real A\* charges a corridor). One value is kept per *pair* of
  portals and read for hops going either way through it: walking a pair the other way round is not
  provably the same walk, and on that stage the two differ for about two portal pairs in five — which
  changes the first hop of about one route in a thousand, because the abstract search is deciding
  between much larger differences. A node's entry is chosen by the
  best estimate *through* it rather than by the cheapest way *to* it, so a route no longer hugs
  the nearest boundary point and then turns. On the 27° route the middle of the route walks
  **1.000×** its straight line at cell 16 and 1.002× at cell 32, where it walked 1.06–1.10×
  before; the 27° route across the 48-cell sentinel field fell from 1.062× to 0.997×. The table
  costs derivation time — on a cluttered mesh it is most of a graph's derive cost (the Field at
  cell 16: 45 ms, of which the walled-pair search is about half) — and nothing per tick.

Two more things keep a unit from *behaving* badly at a node, both of which cost route length nothing
and were only ever visible by watching:

- **A leg aims three crossings ahead, not at the next one.** A portal is a shared triangle edge, so
  reaching it leaves the agent on the near side with its node unchanged — the planner would hand
  back the crossing just completed, and the hand-off keeps velocity, so the unit circled instead of
  stalling. The plan therefore skips crossings it is already within reach of. Measured, it skips
  two on essentially every plan and aims at the third: the reach test compares against the previous
  plan's target as well as the agent, and after a hand-off the crossing being asked for is inside
  that ball by construction. So the practical rule is a **fixed three-hop lookahead**, and the
  cost of being wrong about a skip is only that the leg looks one hop further. The alternative —
  aiming at the next crossing — is what produced the circling, and removing the second reference
  point doubles the number of legs on the same journey for the same travel time.
- **A leg hands off at the agent's turning radius**, `v² / a`, rather than at the arrival threshold
  used for the destination. An agent cannot hold an arc tighter than that, so asking it to pass
  within a few centimetres of a portal it must turn at is asking for something no steering can do;
  it orbits the point. Handing off earlier is not a loss of precision — the leg planner exists to
  keep the *search* local, not to march the unit through gates.

Cluster size is the dial, and it is bounded on both sides. Too small and a leg is two triangles long,
so the unit commits to a portal every few metres for no gain; too large and a leg no longer fits the
corridor buffer. On a 22k-triangle stage the usable window ran up to 32 world units per cell, with
**16 the best measured**: it plans fastest (800-agent order tick 45.8 ms against 60.9 at cell 32
and 52.4 at cell 8), and since hops are priced portal to portal the larger cell no longer buys a
longer walk either (the two are within a hundredth of each other on that stage).

Three things are worth knowing, because since 0.13 this is something you turn *off* or narrow rather
than something you reach for:

- **It is on by default since 0.13, and the off switch is exact.** The agent system's constructor
  installs a graph itself when the mesh is one a flat search can run out of budget on
  (`triangles > MaxIterations`; `FPNavTuning.AutoInstallAbstractGraph`), choosing the cell size the
  way `TryInstallAbstractGraphIfBeneficial` does. A mesh within the budget gets nothing and does not
  change by a bit. With `autoInstallAbstractGraph: false` the system plans the flat path it always
  did, bit for bit — that is how the two are compared on one fixture. Turning the default on moved
  the fingerprint of every game whose mesh is past the budget (the graph's checksum is part of it),
  which is why it shipped in the same minor version as the partial-path default: one break, not two.
  A game that named `partialPathOnExhaustion: false` to keep its 0.12 replays must name this off too.
- **It rides the navigation fingerprint.** The derived graph's checksum folds into
  `GetNavFingerprint`, contributing zero when there is no graph. Two peers disagreeing about the
  graph — one planning in legs and one not, or two with different cell sizes — differ there and are
  caught by the Ready exchange, exactly like a mesh mismatch.
- **Agents planning under a different mask take the flat path.** The graph is derived for one mask,
  so planning an agent whose resolved plan mask is a different one against it would promise
  crossings that mask forbids. They fall back rather than being told a route they cannot walk, and
  `DebugMaskFallbackCount` reports how many — the number that decides whether per-mask graphs would
  earn their memory. **Carrying an override is not itself the trigger**: an override that resolves
  to the mask the graph was built for keeps its legs, so writing the default mask explicitly does
  not quietly opt an agent out. The test is equality, which is narrower than the exact safety
  condition (a graph mask that is a bit-subset of the agent's is also safe) — the narrower rule is
  chosen because the wider one lets an `ALL_AREAS` agent be routed around footprints it could have
  crossed, a detour only that agent pays and nothing reports.

```csharp
// The same decision, made by hand — and it asks one more question than the constructor does.
navSystem.TryInstallAbstractGraphIfBeneficial(out FP64 cellSize);
```

**The constructor and this call do not ask the same thing.** The constructor asks only whether a flat
search can run out of budget; the call asks that *and* whether a corridor can be clamped, because it
defaults to `exhaustionOnly: false`. So there is a band of mesh sizes where the automatic install
declines and this call does not, and in that band the line above is the only way to get legs:

| triangles | the constructor | `TryInstallAbstractGraphIfBeneficial` |
|---|---|---|
| ≤ `CorridorCap` (128) | nothing | `NotNeeded` — the fingerprint does not move |
| **129 – 4096** | **nothing** — it does not ask about the clamp | **`Installed`** — the only way in |
| > `MaxIterations` (4096) | installs | `AlreadyInstalled` — a no-op, the graph is already there |

**Where the call still installs, it moves this game's navigation fingerprint and its existing replays
stop loading.** Past the search budget that cost was already paid, once, by 0.13 turning the automatic
install on; in the band between the two lines it has not been paid and this call is what pays it. On a
mesh under both thresholds the call answers `NotNeeded`, installs nothing, and the fingerprint does not
move — so adding the line to a small stage costs nothing at all.

**Every outcome is logged, including the ones where nothing happens** — a game reads this decision from its boot log, and a branch that stays silent cannot be told apart from the call not having run. A small stage prints `planning in legs: off — not needed. 116 triangles is within both the corridor cap (128) and the search budget (4096)…`; a large one prints the cell size the ladder settled on, the node counts, which threshold was crossed, and the new fingerprint.

The four outcomes are separate because they call for different responses: `Installed`, `NotNeeded`
(the mesh cannot reach either failure), `AlreadyInstalled` (a graph is already there and was left
alone — since 0.13 that is usually the constructor's own automatic install rather than a cell size
you picked), `NoCellSizeFits` (no rung of the ladder produced nodes inside the corridor cap —
returned as a value, never thrown, because this runs on the initialization path). The chosen cell
size comes back out so a tool can draw the same partition and another peer can rebuild the identical
graph.

**Choosing the cell size yourself is still supported**, and is what the helper does underneath:

```csharp
var graph = new FPNavAbstractGraph(
    mesh, FP64.FromInt(16), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK);
navSystem.SetAbstractGraph(graph);       // null turns it back off
```

`FPNavAbstractGraph` is an opaque handle: build it, hand it over, and read `NodeCount`, `EdgeCount`,
`MaxNodeDiameter`, `MaxLegCorridorTriangles`, `NodeComponentCount` and `Checksum` to tune it. The node and edge accessors stay internal — they are the
representation rather than a format. A swap rebinds the graph along with the query, pathfinder and
funnel, so hand it over once and leave it alone.

**Installing a graph drops whatever was prepared for the one it replaces.** A game that rebakes at
runtime has the engine building the next graph a frame ahead (below). That spare is built to the
*outgoing* graph's cell size, cost fold and mask, and its cell size cannot be repointed — so a later
swap adopting it would quietly put the old cell size back, on the machines that happened to prepare
and not on the ones that did not. Handing a graph over therefore discards the prepared one, and the
next frame prepares again at the new build identity.

**`SetAbstractGraph` refuses two things rather than letting them run.** A graph derived from a
different mesh (node ids index that mesh's triangles, so the route would run through geometry that is
not there — and every peer would agree, so it would never surface as a desync), and a graph whose
widest node cannot fit the corridor. The second is what those two node measurements exist for: a leg
stays inside its node, so a node too wide for the cap plans corridors that come back clamped, which
is the silent replanning loop this whole section exists to remove.

**Two numbers, because they count different things.** `MaxNodeDiameter` is a count of **hops** across
the widest node — a node of one triangle is 0. `MaxLegCorridorTriangles` is a count of **triangles**,
which is what `CorridorCap` counts, so it is the one to compare against the cap. It is two more than
the diameter: one because a path of *d* hops visits *d + 1* triangles, and one because a leg aims at
a point on the boundary itself, which can resolve to the triangle on the far side. Comparing the
diameter against the cap directly is off by exactly that two.

The measure is a double sweep — exact on a tree, a lower bound otherwise — so it catches a cell size
that is clearly too large rather than proving the cap can never be reached;
`DebugCorridorTruncatedCount` stays the runtime net for whatever slips through.

**The derivation also reports what it found in the mesh.** Two things it can neither fix nor hide:

- **Ground that costs less than the distance across it.** `FPNavAbstractCostFold.Min` prices a node
  by its cheapest triangle, which keeps the route estimate conservative — but only while no triangle
  is *cheaper* than plain distance to cross. Nothing bounds `costMultiplier`, and a road authored at
  0.5 is ordinary, so the derivation counts such triangles and warns once when it finds any: from
  there the estimate is no longer conservative and a first hop may not be the best one.
- **Triangles that disagree about who their neighbours are.** The bake pairs both sides of a shared
  edge in one step, so a mesh Klotho built cannot be one-directional about it; a hand-made or
  third-party one can, and nothing checks it on load. The in-node walk skips such a pair and the
  derivation logs an error with the count, rather than throwing from inside a mesh swap.

**Decide from the mesh, not from taste — and there are two lines, not one.** A corridor cannot hold
more triangles than the mesh has, and A* cannot expand more than it has either, so both failures have
an exact necessary condition:

| failure | necessary condition | default |
|---|---|---|
| corridor clamped | `triangles > FPNavTuning.CorridorCap` | 128 |
| **search runs out of budget** | `triangles > FPNavTuning.MaxIterations` | 4096 |

They are different lines and they mean different things. Between them a mesh can be clamped but
cannot exhaust; past the second, units report `PathFailed` while standing on walkable ground. The
failure that strands units is the **second** one — on a real 22k-triangle asset a flat search runs
out after 22 world units — and a condition written against the corridor cap answers the wrong
question. Read both from `system.Tuning`, not from the constants: those are the defaults, and an
instance handed a different tuning still compiles against them.

Necessary is not sufficient. Past either line the failure becomes *reachable*; whether an actual
route reaches it depends on the shape of the mesh.

`TryInstallAbstractGraphIfBeneficial` answers both and logs which one was true; the constructor asks
only the second and logs which one it asked. Brawler's own stages are 116 and 60 triangles, so both
answer `NotNeeded` — the sample has no call left, only a comment where one used to be, because the
constructor makes the decision now and says so in the boot log. A stage that grows past a line gets a
graph on the next boot, and that stage's older replays stop loading. Read the base mesh, not the
rebaked one: the decision has to be identical on every peer and stay put for the match.

**When it does install, the cell size is searched rather than guessed.** Node width scales with cell
size and local triangle density, and density varies by an order of magnitude between assets, so no
fixed value is safe everywhere and any formula carries a constant fitted to whatever meshes it was
measured on. The helper derives at `mesh.GridCellSize * 16` and halves until the widest node fits
`CorridorCap`, taking the first that does — the largest node that fits, so the graph holds the fewest
nodes it can. Deriving is *cheaper* at larger cells, so the ladder spends its cheap probes first; on
the 22k-triangle asset it settles in two derivations. The search runs once, at install: a rebake
re-derives at the size it chose.

**And when the graph is on but not shortening a particular search, that is reported.**
`DebugExhaustedWithoutLegsCount` counts searches that ran out of budget while nothing was making them
local — which includes plans that skipped an installed graph (a differently masked agent, an endpoint
off the mesh, no abstract route). `DebugIterationExhaustedCount` alone cannot tell those from an
overrun inside a healthy leg. The first occurrence is also logged once.

**The other thing worth watching is a leg that ends the moment it is planned.** A leg hands off at
the agent's turning radius `v² / a`, and nothing bounds that against the width of a node. Once the
radius grows wider than a node, every leg target is already "reached" on the tick it is chosen: the
hand-off fires immediately, clears the repath cooldown, and the agent runs a full A\* every tick —
the opposite of what this feature is for.

`DebugLegEndedOnPlanTickCount` counts exactly that, and it is a **rate**, read against
`DebugLegAdvanceCount` beside it. Near zero is healthy — a leg that takes even one tick to walk never
lands there. Climbing in step with the advances means the radius has swallowed a node, and a warning
says so once with both numbers. Do **not** read `DebugLegAdvanceRepeatCount` for this: it rises about
once per leg either way, so its ratio is the same whether or not anything is wrong.

The fix is on the game's side of the line — lower the speed, raise the acceleration, or derive with a
larger cell — because the radius is a property of how your units move.

A game that rebakes at runtime rebuilds this graph on every swap, because it is derived from the
mesh. You do not have to arrange anything for that: the engine builds the new graph a frame before
the swap and the swap adopts it, so the cost stays off the tick — and from the second rebake on,
the new graph copies the rows of every node the rebake left alone from the graph it replaces, so a
one-building rebake on the Field derives in ~11 ms at cell 32 instead of ~44.
[Navigation.Rebake.md § 8](./Navigation.Rebake.md#8-performance) has the measured numbers and the
two cases where a swap still rebuilds on the spot.

#### Seeing it work

`Tools > Klotho > Visualizer > NavMesh` (Godot: the FPNavMesh dock) can turn legs on over any
navmesh asset you load, inside **Agent Simulation** — not at the top of the window, because the
Pathfinding section's *Find Path* calls the pathfinder directly and is unaffected by the graph.

Set a cell size, pick the mask to derive under, press **Apply**. What the panel then shows is what
decides whether legs are doing anything:

- **`NodeCount`** — one node means every route already ends inside it, so the path is the flat one.
  That is correct, not broken; a mesh has to be big enough to have somewhere to hop to.
- **widest node vs the corridor cap** — the refusal `SetAbstractGraph` would throw is pre-checked
  and shown as text with both numbers, because raising the cell size until you meet it is normal
  use of the dial.
- **the counters**, which no editor tool used to show. `legs advanced` climbing is the feature
  working. `abstract search failed` is the failure legs introduce and **the pathfinder's own budget
  counter cannot see it** — that search never runs when the abstract one gives up first. `mask
  fallback` counts agents whose plan mask is not the one the graph was derived under; matching them
  in the per-agent mask controls is how you watch it drop to zero. The last two are the
  pathfinder's and are **shared with the Find Path button**, so pressing that moves them with no
  agent involved.

Applying pauses the simulation, installs, hands every agent back to the planner and resumes — an
agent already holding a corridor would otherwise keep walking the flat one and the button would
look inert. Loading a different mesh drops the graph (a new agent system carries none), and the
panel clears with it.

### Partial paths when the budget runs out

Legs make a search small. When they cannot — a mesh whose nodes never fit the corridor cap at any
cell size, or a game that turned them off — a search that runs out of `MaxIterations` still
has to answer. Before 0.13 it answered **nothing**: `FindPath` returned `false`, the agent sat at
`PathFailed`, and it stayed there until the game gave it a new destination (a rebake does not
re-plan it). Detour and Unity's NavMesh do not fail on cost; they return the path to the closest
point the search reached, and since 0.13 so does Klotho by default —
`FPNavTuning.PartialPathOnExhaustion` is on. The old answer is one named argument away:

```csharp
var tuning = new FPNavTuning(partialPathOnExhaustion: false);  // pre-0.13 behaviour, bit for bit
```

Naming it moves the navigation fingerprint (the switch is its own term of it), so peers and replays
that disagree about the switch refuse each other — which is also why turning the default on was a
minor version: every game that never named a tuning had its fingerprint move, and replays recorded
before 0.13 are refused by a 0.13 build.

**What the agent gets.** The corridor to the node that got closest to the destination by the
search's own heuristic (ties to the lower triangle index, so the choice is a property of the mesh
and not of the heap), clamped like any corridor with the agent's side kept. The agent aims at the
**end of that corridor** rather than at the destination, so reaching it is the leg hand-off — a
re-plan at full speed, not an arrival — and the next search starts from there. A partial is handed
back only when the closest node is closer to the goal than the agent stands by **more than the
agent's own hand-off radius** (`Speed² / Acceleration`, never below the arrival threshold); a
corridor that ends inside that radius would be "reached" on the tick it was planned. Below that
the search fails exactly as it does today, and it still fails for a drained open set: *there is no
route* keeps its answer.

**What it costs, and the failure it adds.** Every hop is a full-budget search, and hops follow each
other without the repath cooldown. The new way to fail is **after moving**: the heuristic is a
straight line, so a pocket whose closed side faces the goal — or, on a multi-floor mesh, the floor
right under the goal — looks like progress, the unit walks into it, and the next search finds
nothing closer and fails there. A `PathFailed` agent may therefore no longer stand where it was
ordered from. Measured over the shipped `Field` asset (256 deterministic pairs, 105 of which exhaust
the default budget), judging progress at the best node and walking the clamped chain reached the
goal in 93 of 105 with a median of one hop and no cycle in 541 chains; at a quarter of the budget
about half the chains end in a pocket, which is what the mode above looks like at scale. Judging
the *clipped* end instead — the point the unit actually walks to — refused every winding route
(0 of 37 on a serpentine, where each clipped end lies further from the goal than the hop started),
which is why the rule is what it is.

**Where a clipped chain ends.** The best node is always outside the unit's reach radius — its
progress is at least that radius, and progress cannot exceed distance — but the corridor cap cuts
at a count, and on a switchback that count landed the end straight across the wall from the unit,
inside the radius at full speed. The hand-off then fired on the tick the plan was made, cleared the
repath cooldown, and the next tick planned again: a full-budget search per tick until the unit's own
motion carried the end away (measured at 62 such plans in bursts of 17 ticks on a 28-cell
serpentine at speed 7, and 191 hand-offs against 71 at speed 9 on a 96-cell one). So a clipped
partial now ends at the triangle of the kept prefix **farthest from the start** — a valid prefix of
the chain, minus exactly the part that came back, which on a switchback is the turn. Unclipped
partials are unchanged. `DebugPartialEndedOnPlanTickCount` on the agent system counts the event
(exact without a graph, like the hand-off counter) and stays at zero.

**When this and when legs.** Legs first, always: they make the failure not happen. This is the net
under them — a mesh that cannot be cut, a game that has not cut it yet — and the two compose: with a
graph installed, a leg search that exhausts gets a partial toward its portal instead of falling back
to the flat search.

**What moves.** The switch is part of the navigation fingerprint as its own term, zero when off, so
peers that disagree about it are refused at Ready, on FullState, and on replay load, while a tuning
that never names it keeps the fingerprint it had — it is deliberately *not* folded into
`FPNavTuning.Digest`, whose chain would move every custom tuning's digest. `DebugPartialPathCount`
and `DebugPartialRejectedCount` on the pathfinder count partials given and partials refused for no
progress (both also count in `DebugIterationExhaustedCount` — the budget did run out);
`DebugPartialHandoffCount` on the agent system counts partial ends reached, exactly without a graph
and folded into `DebugLegAdvanceCount` with one (nothing in the frame says which kind of target
`PathTarget` is, and a field for it would change the wire); it can trail `DebugPartialPathCount`,
because a fast unit that leaves the corridor at a turn is re-planned by the off-corridor repath
before it reaches the partial's end. The visualizers' Find Path marks a
partial result as such, and the agent row's `BudgetExhausted` names the pocket case.

---

## Tuning the caps (`FPNavTuning`)

The sizes below are **defaults, not fixed limits**. Each navigation type takes an optional
`FPNavTuning` and sizes its buffers and loop budgets from it; omit the argument and you get exactly
what shipped.

```csharp
var tuning     = new FPNavTuning(maxAgents: 128, corridorCap: 32);   // name only what you change
var query      = new FPNavMeshQuery(navMesh, logger, tuning);
var pathfinder = new FPNavMeshPathfinder(navMesh, query, logger, tuning);
var funnel     = new FPNavMeshFunnel(navMesh, query, logger, tuning);
var navSystem  = new FPNavAgentSystem(navMesh, query, pathfinder, funnel, logger, tuning);
navSystem.SetAvoidance(new FPNavAvoidance(tuning));
```

`partialPathOnExhaustion` is the one *switch* among the caps and the one knob outside the digest —
see [Partial paths when the budget runs out](#partial-paths-when-the-budget-runs-out).

**Hand the same value to all five**, and `FPNavAgentSystem` checks that you did. The five types size
buffers and bound loops from their own copy, so a stack whose parts disagree plans a corridor the
rest of it will not walk — a search that returns more triangles than the storage keeps is the
disagreement `DebugCorridorCopyTruncatedCount` exists to report. The system compares what it is
handed at all three doors — the constructor, `SetAvoidance`, and the four-argument `SwapNavMesh` —
and throws naming the collaborator and the first cap that differs. A null collaborator passes; that
is a different question.

That check is **local**: it catches a stack that disagrees with itself, not two peers that are each
internally consistent but tuned differently. The tuning digest in
[the navigation fingerprint](#cross-peer-identity-the-navigation-fingerprint) is what catches those.

**These are build identity, not preferences.** Every one of them can move an agent, so lockstep peers
must construct navigation with the same tuning, and a replay must be played back against the tuning
it recorded. They are also **immutable after construction** — the buffers are sized once — and
validated in each constructor, which throws rather than clamping: every cap must be positive,
`MaxNeighbors` must leave room for obstacle lines, and `CorridorCap` must fit the compile-time
storage ceiling (`FPNavTuning.CorridorCeiling`).

Two things the validation deliberately does **not** enforce, because the shipped defaults sit on the
far side of both:

- **Portals against the corridor.** A corridor of length `L` wants `L + 1` portals; the defaults are
  128 against 128, so the funnel already drops the last corridor edge on a full-length corridor. Raise
  `MaxPortals` with `CorridorCap` if you want the whole chain.
- **Waypoints against portals.** Same relationship, same shipped asymmetry (64 against 128). Both
  truncate rather than break: `FindCorners` clamps the caller's requested corner count to the
  waypoint buffer, so a small `MaxWaypoints` yields fewer corners instead of overrunning.

**What a smaller `CorridorCap` buys, and what it does not.** The corridor is a clamp applied *after* a
completed A\*, so the search cost is unchanged; what you get is a smaller copy per tick and — the
other side of the trade — a replan roughly every `cap` triangles of the journey. It does **not** shrink
the frame reservation: the corridor lives in `NavAgentComponent`'s `fixed` buffer, whose size is the
compile-time `MAX_CORRIDOR`. To move *that* you edit the constant and rebuild every binary that talks
to another; the component's wire size moves with it, and `ComponentStorageRegistry.LayoutFingerprint`
is what reports a mismatched pair. For per-slot memory the shipped lever is
`ISimulationConfig.ComponentMaxCountOverrides` instead — with the caveat that exceeding a component's
slot count throws, so that number is the worst case a match has to survive, not a tuning dial.

`new FPNavTuning()` and `default` are **all zeros** — a struct's implicit parameterless constructor
wins overload resolution even when every parameter is optional — and `Validate` rejects that rather
than letting a zero-sized buffer through. Start from `FPNavTuning.Default`, or name the values you
want as above.

---

## Constants

The first block is what the caps default to; the `FPNavTuning` column is the field that overrides
each one per instance (blank = not settable).

| Constant | Location | Value | `FPNavTuning` | Description |
| ---- | ---- | -- | ---- | ---- |
| `MAX_AGENTS` | `FPNavAgentSystem` | 64 | `MaxAgents` | Max agents the position-correction pass separates per `Update` call. The pass is O(iterations · n²) |
| *(unnamed)* | `FPNavAgentSystem` | 4 | `CollisionResolveIterations` | Relaxation passes the position-correction loop makes per tick |
| `MAX_CORRIDOR` | `NavAgentComponent`, `FPNavMeshPathfinder` | 128 | `CorridorCap` | Max length of A* / agent corridor. The constants are also the **compile-time storage ceiling** the cap is validated against — `NavAgentComponent`'s is the `fixed` buffer, so only it decides the reservation |
| `MAX_PORTALS` | `FPNavMeshFunnel` | 128 | `MaxPortals` | Max funnel portals (a corridor of length `L` wants `L + 1`) |
| `MAX_WAYPOINTS` | `FPNavMeshFunnel` | 64 | `MaxWaypoints` | Max number of Funnel waypoints |
| `MAX_ITERATIONS` | `FPNavMeshPathfinder` | 4096 | `MaxIterations` | Max A* iterations |
| `MAX_ORCA_LINES` | `FPNavAvoidance` | 64 | `MaxOrcaLines` | Max ORCA half-planes (obstacle + agent) |
| `MAX_OBST_LINES` | `FPNavAvoidance` | 48 | *(derived)* | Max obstacle half-planes (= lines − neighbours; reserves agent-line slots) |
| `MAX_NEIGHBORS` | `FPNavAvoidance` | 16 | `MaxNeighbors` | Max ORCA neighbor agents |
| *(unnamed)* | `FPNavAgentSystem` | 256 | `BfsFrontierCap` | Frontier/visited bound of the graph-local obstacle query (overflow reported by `DebugBfsFrontierOverflowCount`). The candidate buffer is 3× this, and moves with it |
| *(unnamed)* | `FPNavMeshQuery` | 48 | `MoveMaxQueue` | BFS bound of one `MoveAlongSurface` step |
| `DEFAULT_AREA_MASK` | `FPNavAgentSystem` | `FPNavMeshAreas.DEFAULT_AGENT_MASK` | — | The mask a path or walk gets when the agent names none — i.e. what a **zero** `PlanAreaMaskOverride`/`WalkAreaMaskOverride` resolves to |
| `BUILDING_AREA` | `FPNavMeshAreas` | 1 | — | Area index the rebaker stamps onto retained building footprints; reserved — the build pipeline refuses a bake that uses it |
| `BUILDING_MASK` | `FPNavMeshAreas` | `1 << 1` | — | A retained triangle's `areaMask`, exclusively |
| `DEFAULT_AGENT_MASK` | `FPNavMeshAreas` | `~BUILDING_MASK` | — | Every area except the building's |
| `ALL_AREAS` | `FPNavMeshAreas` | `~0` | — | Every area, retained footprints included — for a planner allowed to route through buildings |

> The cap constants remain public because tests and existing code read them, but they are now the
> **defaults rather than the truth**: an instance handed a different tuning still compiles against
> them. Read `Tuning` on the type you hold when you need the value it is actually running with.

---

*Last updated: 2026-09-09 (0.14.0) — hops between a route's two ends are priced portal to portal from a table derived with the graph (one value per portal pair, read both ways), a node's entry is chosen by the best estimate through it, and a leg aims three crossings ahead; installing a graph now discards whatever the engine had prepared for the one it replaces; and the derivation reports two things it finds in a mesh — ground cheaper than the distance across it, and triangles that disagree about their neighbours. (2026-09-08 (0.13.0) — planning in legs and partial paths, both on by default. (2026-09-06 (0.12.1) — the endpoint lookup `FindPath` uses is public and documented (`FindTriangleForEndpoint`, tie broken toward allowed ground on the same surface only), the filtered lookups gained the height-aware `FindPassableTriangleForEndpoint`, `FPNavPathFailure` names why an agent will not move, and the navigation fingerprint now folds a behaviour revision and the tuning digest — with a replay checked against it at `StartReplay`. (2026-09-05 — the navigation caps became per-instance values: `FPNavTuning` is an optional constructor argument on the query, pathfinder, funnel, avoidance and agent system (defaults unchanged, validated at construction, immutable afterwards), the constants stay as those defaults, and `NavCorridorHelper.SetCorridor` now reports what it dropped through `DebugCorridorCopyTruncatedCount` — where 0 is the only correct value. (2026-09-04 (0.12.0) — `FPNavMeshPlacementProbe` and `FPNavMeshAreas` added to the component list and file layout, the NavMesh visualizers gained a building-placement tool on both editors, and ORCA now treats an agent standing exactly on an obstacle corner as an ordinary position. (2026-09-03 — an agent standing on ground its mask forbids can now leave it (the start is exempt in `FindPath`, and both the walk and the A\ expand a refused neighbour when the triangle they expand from is refused too; `DebugMaskedStartCount` reports it, and `Blocked` now requires that the agent actually asked to move), per-agent area masks (`PlanAreaMaskOverride`/`WalkAreaMaskOverride`, zero = no override, `SetAreaMask`, `FPNavAgentStatus.Blocked`) and, earlier the same day, the building area: `FPNavMeshAreas` (index 1 reserved and stamped onto retained footprints), `DEFAULT_AREA_MASK` now excludes it, and `DebugAreaMaskRejectedCount` can move through the agent path. (2026-09-02 — crowd scaling: a worked spatial partitioner (`FPNavAgentClusterSplitSampleTests`) with the three details that make it safe to copy; the four measured costs of a move order at 64/256/800/3200 agents, the cluster-split call pattern and its sweep, the determinism rules the partition has to meet, and the three diagnostic counters that make the caps visible (`MAX_AGENTS` and `BFS_FRONTIER_CAP` added to the constants table). (2026-08-18: the rebake driver is self-wiring: registering the system is the whole wiring, and `KlothoEngine` owns slice pacing plus the corrections at world init and after a full-state apply; `FPNavMeshPlacementValidator` gives the command path the driver’s own derivation, order and audits. (2026-08-17: delayed install: `FPNavMeshRebakeDriver` derives the installed mesh from frame state each tick (rollback-safe), time-sliced rebake across frames, two-mesh boundary cache, `FPNavAgentInstaller` swap/reseed protocol.) (2026-08-12: runtime NavMesh rebake (deterministic re-triangulation from building footprints, shape catalog, placement grid) — see [Navigation.Rebake.md](Navigation.Rebake.md).) (2026-07-23: graph-local obstacle query (BFS multi-floor/ramp, bake-slope climb cap), clearance tuning (`ObstacleRadiusInset` auto-applied from the recorded bake-settings block), position-correction pass, non-convex/dual-source extraction.) (2026-07-22: ORCA static obstacles — hard-constraint LP3, `FPNavMeshObstacleExtractor`, `LoadNavMeshObstacles()`, `MAX_OBST_LINES`.)))))*
