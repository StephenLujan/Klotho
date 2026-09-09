using System;
using xpTURN.Klotho.Logging;

using xpTURN.Klotho.Deterministic.Math;
using xpTURN.Klotho.ECS;

namespace xpTURN.Klotho.Deterministic.Navigation
{
    /// <summary>
    /// What <see cref="FPNavAgentSystem.TryInstallAbstractGraphIfBeneficial"/> did. Four outcomes
    /// rather than a bool: three of them mean "no graph", and they call for different responses —
    /// <see cref="NotNeeded"/> is the expected answer on a small stage, <see cref="AlreadyInstalled"/>
    /// says the game already made this decision itself, and <see cref="NoCellSizeFits"/> is a mesh
    /// the helper could not serve at all.
    /// </summary>
    public enum FPNavAbstractGraphInstall : byte
    {
        /// <summary>A graph was derived and installed. The chosen cell size is the out parameter.</summary>
        Installed = 0,

        /// <summary>
        /// The mesh is small enough that the failure asked about is not reachable, so a graph would
        /// move the navigation fingerprint for nothing. See the two exact conditions on the helper —
        /// the automatic install from the constructor asks only about budget exhaustion, an explicit
        /// call asks about the corridor clamp as well.
        /// </summary>
        NotNeeded = 1,

        /// <summary>
        /// A graph was already installed and the helper left it alone. A game that picked its own
        /// cell size keeps it.
        /// </summary>
        AlreadyInstalled = 2,

        /// <summary>
        /// Every cell size on the ladder left a node wider than <see cref="FPNavTuning.CorridorCap"/>,
        /// down to the mesh's own broadphase cell. Nothing was installed and nothing was thrown.
        /// </summary>
        NoCellSizeFits = 3,
    }

    /// <summary>
    /// Per-tick agent update system.
    /// Handles path requests, steering, movement, and NavMesh constraints.
    /// </summary>
    public class FPNavAgentSystem : INavFingerprintSource, INavGraphPreparer
    {
        // Non-readonly: SwapNavMesh rebinds these to a rebaked mesh at runtime.
        private FPNavMesh _navMesh;
        private FPNavMeshQuery _query;
        private FPNavMeshPathfinder _pathfinder;
        private FPNavMeshFunnel _funnel;
        private readonly IKLogger _logger;

        private FPNavAvoidance _avoidance;

        /// <summary>
        /// The navmesh this system is currently running on — the base mesh until the first
        /// <see cref="SwapNavMesh"/>, a rebaked mesh afterwards. This is the single source of
        /// truth for "current": the swap rebinds the fields these read, so an
        /// <c>INavMeshProvider</c> that delegates here cannot go stale, including when
        /// <see cref="SwapNavMesh"/> is called directly rather than through a game system.
        /// Diagnostics only — the simulation reads the fields.
        /// </summary>
        public FPNavMesh CurrentMesh => _navMesh;

        /// <summary>Query bound to <see cref="CurrentMesh"/>. See that property for the contract.</summary>
        public FPNavMeshQuery CurrentQuery => _query;

        // Sized from the SAME knob the query walks with: MoveAlongSurfaceWithVisited clamps its
        // handover to this buffer, so a smaller one silently truncates the visited path with no
        // counter to say so. The two used to be a hardcoded 48 against a default 48 — equal by
        // coincidence, and wrong the moment anyone raised moveMaxQueue.
        private readonly int[] _visitedBuffer;

        /// <summary>Width of the visited handover, for the gate that pins it to the query's walk.</summary>
        internal int DebugVisitedBufferLength => _visitedBuffer.Length;
        private readonly int[] _corridorBuffer;

        // --- Graph-local obstacle query (BFS here, reusing the navmesh topology directly) ---
        // Forward triangle->segment CSR built alongside LoadObstacles (segment index aligns with
        // FPNavAvoidance._obstacles). null until LoadNavMeshObstacles runs on a navmesh.
        private int[] _triSegStart;   // CSR offsets, valid over [0, triCount] — may be oversized
        private int[] _triSegList;    // segment indices grouped by owner triangle — may be oversized
        private FPNavMeshObstacleExtractor.ExtractScratch _extractScratch;
        // Generation of the avoidance obstacle load captured when the CSR was built, used as a
        // desync guard: if the avoidance is re-loaded out from under us, the CSR is stale -> fall back.
        private int _obstacleLoadGenerationCache = -1;

        // BFS buffers (GC-0, generation-stamped — same pattern as FPNavMeshQuery.MoveAlongSurface).
        // Frontier is a monotone-tail queue, so the cap equals the visited bound. Sized for horizon
        // coverage (NOT copied from MOVE_MAX_QUEUE, which is a single-tick move size).
        private int[] _bfsStamp;      // length triCount, generation stamp
        private int _bfsGeneration;
        private readonly int[] _bfsFrontier;
        // ×3 = (max 3 boundary edges per triangle) × the frontier/visited cap: candCount is bounded by
        // 3·FPNavTuning.BfsFrontierCap == this length, so candidate collection can never truncate and
        // needs no overflow counter (unlike the frontier). Keep this coupling — shrinking it turns the
        // `candCount < _candidateSegs.Length` guard below into a silent, undiagnosed coverage cliff.
        private readonly int[] _candidateSegs;
        private int _bfsFrontierOverflowCount;

        /// <summary>Diagnostic: times the BFS frontier cap truncated a query (coverage cliff).</summary>
        public int DebugBfsFrontierOverflowCount => _bfsFrontierOverflowCount;

        private int _collisionResolveTruncatedCount;

        /// <summary>
        /// Diagnostic: agents dropped from the position-correction pass because the array was
        /// longer than <see cref="MAX_AGENTS"/>, accumulated over the lifetime of this instance
        /// (never reset — same convention as <see cref="DebugBfsFrontierOverflowCount"/>, so a
        /// tick split across several <see cref="Update"/> calls sums instead of overwriting).
        /// <para>
        /// Reading it: a rollback resimulation counts the same tick again, so the total runs ahead
        /// of the number of distinct ticks that truncated. And a non-zero count only means agents
        /// were skipped — whether that is observable depends on the consumer model documented on
        /// <see cref="Update"/>.
        /// </para>
        /// </summary>
        public int DebugCollisionResolveTruncatedCount => _collisionResolveTruncatedCount;

        /// <summary>
        /// Upper bound for the position-correction pass buffers. entityCount beyond this is still
        /// moved by ProcessMovement but not collision-resolved (guarded, no overflow). Callers should
        /// keep nav agent count within this bound.
        /// </summary>
        /// <remarks><b>This is the default, not this instance's value.</b> The pass is sized from
        /// <see cref="FPNavTuning.MaxAgents"/>; read <see cref="Tuning"/> to know what a given
        /// system actually runs with.</remarks>
        public const int MAX_AGENTS = 64;

        // Position-correction pass (DetourCrowd-style). Zero-GC pre-allocated buffers.
        private static readonly FP64 COLLISION_RESOLVE_FACTOR = FP64.FromDouble(0.7);
        private static readonly FP64 COINCIDENT_PEN = FP64.FromDouble(0.01);
        private static readonly FP64 POS_EPSILON = FP64.FromRaw(100);
        private readonly FPVector2[] _disp;
        private readonly int[] _dispWeight;

        private readonly FPNavTuning _tuning;

        /// <summary>
        /// The sizes this instance was built with. <b>Read this, not the constants</b> — they are
        /// the defaults, and an instance handed a different tuning still compiles against them.
        /// </summary>
        public FPNavTuning Tuning => _tuning;

        private int _corridorCopyTruncatedCount;

        /// <summary>
        /// Diagnostic: triangles dropped while copying a planned corridor into an agent
        /// (<see cref="NavCorridorHelper.SetCorridor"/>). <b>0 is the only correct value</b> — under
        /// one effective cap the copy cannot truncate, so a nonzero count means the search and the
        /// component storage were built from different caps.
        /// </summary>
        public int DebugCorridorCopyTruncatedCount => _corridorCopyTruncatedCount;

        /// <summary>
        /// Distance threshold for waypoint arrival detection.
        /// </summary>
        /// <remarks>
        /// Group-convergence guidance: when N agents share one destination and the
        /// position-correction pass is active, they settle into a non-overlapping cluster whose
        /// outermost members sit up to ~radius/(2·sin(π/N)) from the shared point (single-ring
        /// bound; 2D packing is tighter). The pass converges regardless of this value, but to let
        /// every converging agent register arrival inside the ball, set WaypointThreshold at least
        /// that large for big same-destination groups (or give agents slightly offset destinations).
        /// </remarks>
        public FP64 WaypointThreshold;

        /// <summary>
        /// Y difference threshold between floors. Triangles differing more than this are considered different floors.
        /// </summary>
        public FP64 MultiFloorYThreshold = FP64.FromDouble(2.0);

        /// <summary>
        /// Times the abstract graph's re-derivation on a navmesh swap and puts the number in the
        /// swap log. Off by default: reading the clock is cheap but not free, and a diagnostic that
        /// nobody asked for should not ride the deterministic command path.
        ///
        /// <para><b>Why this is a field here and not the engine's
        /// <c>SystemPerfMonitoring</c>.</b> That switch reaches update SYSTEMS, through
        /// <c>EcsSimulation.EnableSystemPerfMonitor</c> and the system runner. A navmesh swap is not
        /// a system — it happens in a command handler — so that instrumentation never covers this
        /// call, and this type holds no reference to <c>ISimulationConfig</c> to read the flag from
        /// anyway. The smallest honest answer is a field the game sets, in the shape
        /// <see cref="MultiFloorYThreshold"/> already established.</para>
        ///
        /// <para><b>Peers need not agree on it.</b> It changes nothing but a log line — no state, no
        /// hash, no timing that any decision reads.</para>
        /// </summary>
        public bool DebugTimeGraphDerivation;

        private int _graphRederiveCount;
        private bool _prepareWarned;

        /// <summary>
        /// Diagnostic: how many times a swap re-derived the abstract graph. Counted always, unlike
        /// the timing above, because the FREQUENCY is what decides whether the cost is worth moving
        /// and it costs an increment to know.
        ///
        /// <para>Zero on a game that installs no graph — which is most of them, and the reason this
        /// cost went unnoticed for as long as it did.</para>
        /// </summary>
        public int DebugGraphRederiveCount => _graphRederiveCount;

        // The idle half of a double buffer. Prepared derivations go in here; adopting swaps it with
        // the live graph and the outgoing one becomes the next spare. Two instances, forever — see
        // PrepareAbstractGraphFor for why that number and not a pool.
        private FPNavAbstractGraph _spareGraph;
        private FPNavMesh _preparedMesh;          // what _spareGraph was derived for, or null
        private long _preparedFingerprint;
        private int _graphPreparedAdoptedCount;
        private int _graphPreparedMissedCount;
        private int _graphInstancesCreated;

        /// <summary>
        /// The graph agents are planning against right now, or null. Internal because it is the
        /// representation rather than an API — a game hands one in through
        /// <see cref="SetAbstractGraph"/> and reads the diagnostics, it does not reach back for the
        /// object. What needs it is the assertion that adopting a prepared graph and deriving one
        /// synchronously produce the SAME graph, which has to compare the two checksums directly.
        /// </summary>
        internal FPNavAbstractGraph CurrentAbstractGraph => _abstractGraph;

        /// <summary>
        /// The graph agents are planning against right now, or null. Public since legs turned on by
        /// default (0.13): a tool that shows the partition can no longer assume a fresh system has
        /// none, and a game that wants to know what the constructor decided reads it here — the
        /// object itself stays the runtime's (see <see cref="SetAbstractGraph"/> for the one door in).
        /// </summary>
        public FPNavAbstractGraph AbstractGraph => _abstractGraph;

        /// <summary>Diagnostic: swaps that took a graph prepared off-tick instead of deriving one.</summary>
        public int DebugGraphPreparedAdoptedCount => _graphPreparedAdoptedCount;

        /// <summary>
        /// Diagnostic: swaps where a preparation existed but was NOT for the mesh being installed, so
        /// the derivation happened on the tick anyway. <b>This is the number that says whether
        /// preparing is worth calling</b> — the driver's cache-hit and rebuild paths never hand a
        /// mesh to the heartbeat, and a boundary that finishes its own task does not either, so a
        /// game whose swaps mostly come from those will see this climb and the adopted count stay
        /// flat. Read the two together, against the driver's own <c>CacheHits</c> and
        /// <c>RebuildInstalls</c>.
        /// </summary>
        public int DebugGraphPreparedMissedCount => _graphPreparedMissedCount;

        /// <summary>
        /// Diagnostic: <see cref="FPNavAbstractGraph"/> instances THIS system has allocated —
        /// <b>at most one, ever</b>, the spare half of the double buffer. The live graph is the
        /// game's, made once and never replaced by this type.
        ///
        /// <para>It exists because the obvious way to prepare — derive a fresh graph each time —
        /// throws away exactly what <c>Rebind</c> was written for (buffer reuse) and leaves a
        /// graph's worth of garbage per rebake. Byte-level allocation gates in this repository are
        /// skipped by default, so the invariant needs an assertion that always runs.</para>
        /// </summary>
        public int DebugGraphInstancesCreated => _graphInstancesCreated;

        private int _ladderProbes;
        private int _ladderPairTablesBuilt;

        /// <summary>
        /// Diagnostic: cell sizes the install ladder has derived a graph for, over this system's
        /// lifetime (the ladder runs in the constructor when the tuning asks for it, and on every
        /// <see cref="TryInstallAbstractGraphIfBeneficial"/>). Cumulative — a test that calls the
        /// ladder more than once reads the difference.
        /// </summary>
        public int DebugLadderProbes => _ladderProbes;

        /// <summary>
        /// Diagnostic: how many graphs the ladder actually BUILT — the ones that fit the corridor
        /// cap, which is at most one per ladder run. Every other candidate is only measured
        /// (<c>FPNavAbstractGraph.MeasureLegCorridorTriangles</c>), so on a mesh where the ladder
        /// has to step down the difference from <see cref="DebugLadderProbes"/> is what the boot
        /// no longer pays for.
        /// </summary>
        public int DebugLadderPairTablesBuilt => _ladderPairTablesBuilt;

        /// <summary>
        /// Derives, ahead of time and OFF the deterministic path, the graph a coming navmesh swap
        /// will need. <b>Call it from the frame heartbeat</b> — the same place the host drives
        /// <c>FPNavMeshRebakeDriver.AdvanceSlice</c> — passing that driver's
        /// <c>PeekPreparedMesh</c>. Nulls and repeats are free.
        ///
        /// <para><b>What it buys.</b> A swap otherwise re-derives the whole graph inside the tick;
        /// measured at 41 ms on the Field asset at the cell size the install ladder picks (cell 32,
        /// Release, tiered compilation off — the pair table is most of it; it was 10 ms before hops
        /// were priced portal to portal and 83 ms before that table learned to look distances up),
        /// against the 2.13 ms budget a sliced rebake exists to hold. The derivation is a pure function of
        /// (mesh, cell size, cost fold, area mask), so doing it early changes nothing about the
        /// result — only when the clock is spent.</para>
        ///
        /// <para><b>Skipping it is not a different behaviour.</b> A peer that never calls this, or
        /// calls it and misses, derives synchronously and gets the SAME graph: same checksum, same
        /// fingerprint, same routes. Peers may therefore disagree about whether they prepared, run
        /// at different frame rates, or mix a game that wires this with one that does not.
        /// <b>Nothing here is allowed to break that</b>, which is why adoption is conservative below
        /// rather than clever.</para>
        ///
        /// <para><b>Same thread as the simulation.</b> Nothing here is synchronised, and the live
        /// graph is read by every planning agent. The frame boundary the host drives slicing from is
        /// that thread; a job is not.</para>
        ///
        /// <para>Does nothing when no graph is installed — there is no cell size to derive at, and a
        /// game with legs off has no cost to move.</para>
        /// </summary>
        public void PrepareAbstractGraphFor(FPNavMesh mesh)
        {
            if (mesh == null || _abstractGraph == null || ReferenceEquals(mesh, _navMesh))
                return;

            // Already prepared for exactly this one. Re-deriving would be correct and wasteful: the
            // heartbeat runs every frame and the task's mesh sits there until a tick takes it.
            if (ReferenceEquals(_preparedMesh, mesh))
                return;

            if (_spareGraph == null)
            {
                // The one allocation, matched to the live graph's build identity so the two are
                // interchangeable. CellSize/CostFold/AreaMask are exposed for exactly this.
                _spareGraph = new FPNavAbstractGraph(
                    mesh, _abstractGraph.CellSize, _abstractGraph.CostFold,
                    _abstractGraph.AreaMask, _logger);
                _graphInstancesCreated++;
            }
            else
            {
                // The live graph donates: every node whose surroundings the rebake left alone takes
                // its pair-table rows from it instead of walking them again (FPNavAbstractGraph.Rebind).
                // The result is the same graph to the bit — the donor moves the clock, never a
                // value — and the live graph is only read. Measured on the Field's rebake meshes at
                // cell 32: 43.6 ms plain, 10.8 with the donor, one building per rebake.
                _spareGraph.Rebind(mesh, _abstractGraph);
            }

            _preparedMesh = mesh;
            _preparedFingerprint = unchecked((long)FPNavMeshRebaker.ComputeFingerprint(mesh));
        }

        /// <summary>
        /// Graph-local obstacle BFS climb cap: an agent's query never expands to a triangle whose
        /// centerY differs from the seed triangle's by more than this. On meshes that record a
        /// bake slope (FPNavMesh.BakeMaxSlopeDeg &gt; 0) the query auto-derives the sound bound
        /// obstRange*sin(bakeMaxSlope) per agent and combines it with this value via min(), so the
        /// default ∞ already gets the tightest safe cap; set this only to tighten FURTHER on a
        /// specific stage. Never set it below the auto bound's formula — a smaller cap drops
        /// genuinely reachable walls (clip hazard). Meshes without a recorded slope (0 = unknown,
        /// e.g. synthetic fixtures) get no auto bound. Only used by the graph path.
        /// </summary>
        public FP64 MaxClimbWithinHorizon = FP64.MaxValue;

        /// <summary>
        /// Consecutive off-corridor tick threshold. Triggers repath when exceeded continuously.
        /// </summary>
        public int OffCorridorRepathThreshold = 10;

        /// <summary>
        /// The areaMask this system passes to a path or a walk when the agent names none: every
        /// baked area, but not <see cref="FPNavMeshAreas.BUILDING_AREA"/> — such an agent neither
        /// plans through nor walks into a retained building footprint. See
        /// <see cref="FPNavMeshAreas"/>.
        /// </summary>
        /// <remarks>
        /// This is the value a ZERO override resolves to, not a value forced on everyone: an agent
        /// can name its own plan and walk masks through
        /// <see cref="NavAgentComponent.SetAreaMask"/>, and asking for different ones on the two
        /// sides is the point (see <see cref="ResolvePlanMask"/>). Leaving both at zero — which is
        /// what every agent carries until something assigns them — keeps this constant in force,
        /// so per-agent masks changed no existing behaviour.
        /// </remarks>
        public const int DEFAULT_AREA_MASK = FPNavMeshAreas.DEFAULT_AGENT_MASK;

        /// <summary>
        /// The mask for PLANNING this agent's path: what it may route through.
        ///
        /// <para>Separate from <see cref="ResolveWalkMask"/> because a game may want an agent to
        /// plan as if a building were not there and then discover it by walking into it. That is
        /// the asymmetry per-agent masks exist for: plan permissively, walk restrictively, and the
        /// unit learns by contact instead of by knowing.</para>
        ///
        /// <para><b>Public because a caller that snaps a destination has to agree with this</b>: it
        /// must ask <see cref="FPNavMeshQuery.ProjectToPassable"/> with the mask this system will
        /// plan with, or the snap lands somewhere <c>FindPath</c> then refuses. Duplicating the
        /// <c>!= 0 ?</c> fold at such a caller is the failure worth preventing — an override of
        /// zero folded wrong means EVERY triangle is impassable, whose symptom (the agent does not
        /// move) is identical to the defect the snap exists to fix.</para>
        /// </summary>
        public static int ResolvePlanMask(in NavAgentComponent nav)
            => nav.PlanAreaMaskOverride != 0 ? nav.PlanAreaMaskOverride : DEFAULT_AREA_MASK;

        /// <summary>
        /// The mask for WALKING: what this agent may actually enter. A triangle this refuses is a
        /// wall to the walk, exactly like an unwalkable one. Public for the same reason as
        /// <see cref="ResolvePlanMask"/>.
        /// </summary>
        public static int ResolveWalkMask(in NavAgentComponent nav)
            => nav.WalkAreaMaskOverride != 0 ? nav.WalkAreaMaskOverride : DEFAULT_AREA_MASK;

        /// <summary>What <see cref="WaypointThreshold"/> starts at.</summary>
        public const double DEFAULT_WAYPOINT_THRESHOLD = 0.3;

        /// <summary>
        /// The hand-off radius: the tightest arc an agent moving at <paramref name="speed"/> with
        /// <paramref name="acceleration"/> can hold, <c>speed² / acceleration</c>, never below
        /// <paramref name="threshold"/>; the threshold alone when there is no acceleration to
        /// divide by.
        ///
        /// <para><b>One function, two callers, and the two must agree.</b>
        /// <see cref="ReachRadius"/> asks it at the agent's CURRENT speed to decide when a leg or
        /// partial end counts as reached; <see cref="PartialMinProgress"/> asks it at the agent's
        /// TOP speed to decide how much progress a partial corridor must make before it is worth
        /// walking. The guarantee that a partial's best node is never already inside the reach
        /// radius on the tick it is planned rests on <c>PartialMinProgress ≥ ReachRadius</c> for
        /// every speed the agent can have — progress cannot exceed distance, and this function is
        /// monotone in speed with speed clamped to <c>Speed</c> by the movement pass. Two
        /// hand-written copies of the formula held that only while nobody edited one of them.</para>
        /// </summary>
        internal static FP64 HandoffRadius(FP64 speed, FP64 acceleration, FP64 threshold)
        {
            if (acceleration <= FP64.Zero)
                return threshold;
            FP64 r = speed * speed / acceleration;
            return r > threshold ? r : threshold;
        }

        /// <summary>
        /// The least a partial corridor must bring this agent closer to its goal to be worth
        /// walking: the widest hand-off radius the agent can have (<see cref="HandoffRadius"/> at
        /// <c>Speed</c>, the arc it holds at full speed — see <see cref="ReachRadius"/>), never below
        /// the arrival threshold. A partial end closer than this would be "reached" on the tick it
        /// was planned, and the re-plan from there would start where the last one did.
        /// </summary>
        /// <param name="waypointThreshold">The system's <see cref="WaypointThreshold"/>; a tool without one passes <see cref="DEFAULT_WAYPOINT_THRESHOLD"/>.</param>
        public static FP64 PartialMinProgress(in NavAgentComponent nav, FP64 waypointThreshold)
            => HandoffRadius(nav.Speed, nav.Acceleration, waypointThreshold);

        public FPNavAgentSystem(FPNavMesh navMesh, FPNavMeshQuery query,
            FPNavMeshPathfinder pathfinder, FPNavMeshFunnel funnel, IKLogger logger,
            FPNavTuning? tuning = null)
        {
            _navMesh = navMesh;
            _query = query;
            _pathfinder = pathfinder;
            _funnel = funnel;
            _logger = logger;

            _tuning = tuning ?? FPNavTuning.Default;
            _tuning.Validate();

            RequireSameTuning(query?.Tuning, nameof(query));
            RequireSameTuning(pathfinder?.Tuning, nameof(pathfinder));
            RequireSameTuning(funnel?.Tuning, nameof(funnel));

            _visitedBuffer = new int[_tuning.MoveMaxQueue];
            _disp = new FPVector2[_tuning.MaxAgents];
            _dispWeight = new int[_tuning.MaxAgents];
            _bfsFrontier = new int[_tuning.BfsFrontierCap];
            _candidateSegs = new int[_tuning.BfsFrontierCap * 3];
            _corridorBuffer = new int[_tuning.CorridorCap];

            WaypointThreshold = FP64.FromDouble(DEFAULT_WAYPOINT_THRESHOLD);
            _avoidance = null;

            // Legs on by default (0.13): a mesh a flat search can run out of budget on gets an
            // abstract graph here, before the first tick and before Ready compares fingerprints.
            // HERE and not in the first Update: the graph's checksum is part of the navigation
            // fingerprint, and a fingerprint that moved after the Ready exchange would let two peers
            // agree at the handshake and diverge a tick later. Only the exhaustion condition — the
            // corridor clamp is a walk-and-replan the flat planner already handles, and asking about
            // it too would put a graph on every mesh past 128 triangles. Never throws; every outcome
            // is logged; a game that wires its own graph turns this off in the tuning or simply
            // installs over it (SetAbstractGraph replaces). This must run on the BASE mesh: the
            // ladder's choice is part of the fingerprint, and a peer that built its system on a
            // rebaked mesh would pick a different cell than the peers that built on the base and
            // rebound — which is why a late joiner is constructed on the base and swapped forward.
            // A null mesh is the "no navigation" sentinel some hosts build (GetNavFingerprint
            // answers zero for it); there is nothing to partition and nothing to log.
            if (_tuning.AutoInstallAbstractGraph && _navMesh != null)
                InstallAbstractGraphCore(out _, FPNavAbstractCostFold.Min, DEFAULT_AREA_MASK,
                    exhaustionOnly: true, automatic: true);
        }

        /// <summary>
        /// Every part of a navigation stack must run the SAME tuning: the caps size buffers and
        /// bound loops in five separate objects, and a stack that disagrees plans a corridor one
        /// half of it will not walk. The type doc says so — <i>lockstep peers must construct their
        /// navigation with the same tuning</i> — and this is what makes it true rather than
        /// advisory.
        ///
        /// <para>Null passes: an absent collaborator is a different question, answered where it is
        /// used. The tunings are compared by value, so <c>FPNavTuning.Default</c> on both sides
        /// agrees — which is the case for every caller that names no tuning at all.</para>
        ///
        /// <para><b>This is a local check only.</b> It cannot see the other peer; two peers each
        /// internally consistent but tuned differently still shake hands here. That is what the
        /// tuning digest in <see cref="GetNavFingerprint"/> is for.</para>
        /// </summary>
        private void RequireSameTuning(FPNavTuning? theirs, string which)
        {
            if (theirs == null || theirs.Value == _tuning)
                return;

            FPNavTuning t = theirs.Value;
            throw new ArgumentException(
                $"FPNavAgentSystem: {which} was built with a different FPNavTuning. The whole stack " +
                $"must share one — differing caps mean the planner and the walker disagree about " +
                $"how far they may look. First difference: {FirstDifference(_tuning, t)}. " +
                $"Hand the same tuning to the query, pathfinder, funnel, avoidance and this system, " +
                $"or omit it everywhere to take FPNavTuning.Default.");
        }

        private static string FirstDifference(FPNavTuning mine, FPNavTuning theirs)
        {
            if (mine.MaxAgents != theirs.MaxAgents)
                return $"MaxAgents {mine.MaxAgents} vs {theirs.MaxAgents}";
            if (mine.CollisionResolveIterations != theirs.CollisionResolveIterations)
                return $"CollisionResolveIterations {mine.CollisionResolveIterations} vs {theirs.CollisionResolveIterations}";
            if (mine.BfsFrontierCap != theirs.BfsFrontierCap)
                return $"BfsFrontierCap {mine.BfsFrontierCap} vs {theirs.BfsFrontierCap}";
            if (mine.MaxIterations != theirs.MaxIterations)
                return $"MaxIterations {mine.MaxIterations} vs {theirs.MaxIterations}";
            if (mine.MaxPortals != theirs.MaxPortals)
                return $"MaxPortals {mine.MaxPortals} vs {theirs.MaxPortals}";
            if (mine.MaxWaypoints != theirs.MaxWaypoints)
                return $"MaxWaypoints {mine.MaxWaypoints} vs {theirs.MaxWaypoints}";
            if (mine.MaxNeighbors != theirs.MaxNeighbors)
                return $"MaxNeighbors {mine.MaxNeighbors} vs {theirs.MaxNeighbors}";
            if (mine.MaxOrcaLines != theirs.MaxOrcaLines)
                return $"MaxOrcaLines {mine.MaxOrcaLines} vs {theirs.MaxOrcaLines}";
            if (mine.MoveMaxQueue != theirs.MoveMaxQueue)
                return $"MoveMaxQueue {mine.MoveMaxQueue} vs {theirs.MoveMaxQueue}";
            if (mine.CorridorCap != theirs.CorridorCap)
                return $"CorridorCap {mine.CorridorCap} vs {theirs.CorridorCap}";
            if (mine.PartialPathOnExhaustion != theirs.PartialPathOnExhaustion)
                return $"PartialPathOnExhaustion {mine.PartialPathOnExhaustion} vs {theirs.PartialPathOnExhaustion}";
            return "none (equal)";
        }

        /// <summary>
        /// Sets the ORCA avoidance system. Pass null to disable avoidance.
        /// </summary>
        public void SetAvoidance(FPNavAvoidance avoidance)
        {
            RequireSameTuning(avoidance?.Tuning, nameof(avoidance));
            _avoidance = avoidance;
        }

        private FPNavAbstractGraph _abstractGraph;

        /// <summary>
        /// Installs the abstract graph, which turns planning into legs: the agent aims at the next
        /// portal instead of at its destination, and asks again when it gets there. <b>Null is the
        /// off switch</b> and restores the flat path exactly — bit for bit, which is what lets a
        /// game measure the two against each other and what keeps replays recorded without legs
        /// valid.
        ///
        /// <para><b>Derive against the same mesh this system runs on.</b> A swap rebinds the graph
        /// along with the query, pathfinder and funnel, so hand it over once and leave it alone.</para>
        ///
        /// <para><b>Every peer must install the same graph.</b> The cell size and cost fold change
        /// where agents walk, so they are build identity rather than preference — the graph's
        /// checksum folds into <see cref="GetNavFingerprint"/> and contributes zero when there is
        /// none, so legs-on against legs-off and two different cell sizes are both caught by the
        /// Ready exchange.</para>
        ///
        /// <para><b>Refused if a leg could not fit the corridor.</b> A leg never leaves its node, so
        /// the widest node bounds the longest leg; a graph whose
        /// <see cref="FPNavAbstractGraph.MaxLegCorridorTriangles"/> exceeds
        /// <see cref="FPNavTuning.CorridorCap"/> would hand the planner corridors it has to clamp,
        /// and a clamped corridor is the silent replanning loop this feature exists to remove.
        /// <b>That property, not <see cref="FPNavAbstractGraph.MaxNodeDiameter"/>, is what the cap
        /// compares against</b> — the diameter counts HOPS and the cap counts TRIANGLES, and reading
        /// one as the other put this boundary two on the wrong side. The measure is a double sweep,
        /// which is exact on a tree and a lower bound otherwise — so this catches a cell size that
        /// is clearly too large rather than proving the cap can never be reached.
        /// <see cref="FPNavMeshPathfinder.DebugCorridorTruncatedCount"/> stays the runtime net for
        /// what slips through.</para>
        /// </summary>
        /// <exception cref="System.ArgumentException">
        /// The graph was derived from a different mesh, or a leg through its widest node would ask
        /// for more triangles than the corridor cap.
        /// </exception>
        public void SetAbstractGraph(FPNavAbstractGraph graph)
        {
            if (graph != null)
            {
                if (!ReferenceEquals(graph.CurrentMesh, _navMesh))
                    throw new System.ArgumentException(
                        "FPNavAgentSystem.SetAbstractGraph: the graph was derived from a different " +
                        "mesh than this system runs on. Node ids index that mesh's triangles, so " +
                        "planning against it would follow a route through geometry that is not there.",
                        nameof(graph));

                if (graph.MaxLegCorridorTriangles > _tuning.CorridorCap)
                    throw new System.ArgumentException(
                        $"FPNavAgentSystem.SetAbstractGraph: a leg through the widest node can ask " +
                        $"for {graph.MaxLegCorridorTriangles} triangles ({graph.MaxNodeDiameter} " +
                        $"hops across it, plus the portal's far side) against a corridor cap of " +
                        $"{_tuning.CorridorCap}. A leg stays inside its node, so a node wider than " +
                        $"the cap plans corridors that come back clamped — the silent replanning " +
                        $"loop legs exist to remove. Derive with a smaller cell size.",
                        nameof(graph));

                // Replacing the constructor's automatic graph is allowed and costs only the
                // derivation that is now thrown away. Said once, so a game that meant to wire its own
                // graph learns it can turn the automatic one off in the tuning — and so nothing here
                // can turn a migration into a failed boot.
                if (_abstractGraph != null && !ReferenceEquals(_abstractGraph, graph)
                    && _tuning.AutoInstallAbstractGraph && !_autoGraphReplacedLogged)
                {
                    _autoGraphReplacedLogged = true;
                    _logger?.KInformation(
                        $"[FPNavAgentSystem] SetAbstractGraph replaced the graph this system already " +
                        $"had ({_abstractGraph.NodeCount} nodes at cell {_abstractGraph.CellSize.ToDouble():F2}) " +
                        $"with the game's own ({graph.NodeCount} nodes at cell {graph.CellSize.ToDouble():F2}). " +
                        $"The one it had was the automatic install (FPNavTuning.AutoInstallAbstractGraph); " +
                        $"a game that wires its own graph can turn that off and skip the derivation " +
                        $"it just discarded.");
                }
            }

            // The prepared spare was built to the OUTGOING graph's build identity, and _cellSize is
            // readonly there — Rebind cannot repair it. Left in place it is adopted at the next swap,
            // so this system would run the old cell size while a peer that never prepared derives at
            // the new one: a fingerprint split after Ready has already passed. _preparedMesh goes with
            // it, because PrepareAbstractGraphFor returns early on that reference before it reaches
            // anything that could notice the mismatch.
            if (!ReferenceEquals(_abstractGraph, graph))
            {
                _spareGraph = null;
                _preparedMesh = null;
            }

            _abstractGraph = graph;
        }

        /// <summary>
        /// The ladder <see cref="TryInstallAbstractGraphIfBeneficial"/> walks, as a multiple of the
        /// mesh's own broadphase cell: start there and halve down to the broadphase cell itself.
        /// Measured across four assets, all of which bake at 4.0 — so this is the widest rung any
        /// of them was seen to need, not a law.
        /// </summary>
        private const int LEG_CELL_LADDER_START_MULTIPLE = 16;

        /// <summary>
        /// Installs an abstract graph when this mesh is one where flat planning can fail, choosing
        /// the cell size itself. This is the one-line form of what a game would otherwise copy out
        /// of the sample — it answers <i>should legs be on here</i> and <i>how big should a node be</i>
        /// so the game does not have to.
        ///
        /// <para><b>Since 0.13 the constructor calls this itself</b> when
        /// <see cref="FPNavTuning.AutoInstallAbstractGraph"/> is on (the default), asking only
        /// about budget exhaustion. Calling it explicitly still has a use: with
        /// <paramref name="exhaustionOnly"/> false it also installs where only the corridor clamp
        /// is reachable, and a game that turned the automatic install off can pick the moment.
        /// Installing moves the navigation fingerprint — the graph's checksum folds into
        /// <see cref="GetNavFingerprint"/> and contributes zero when there is none — so a build with
        /// a graph and one without are different builds, which is the point: the Ready exchange
        /// catches the mismatch instead of letting peers walk different routes.</para>
        ///
        /// <para><b>Two exact conditions, and either is enough.</b> A* cannot pop more triangles
        /// than exist and a corridor cannot be longer than one, so
        /// <c>triangles &gt; <see cref="FPNavTuning.MaxIterations"/></c> is the exact necessary
        /// condition for a search to run out of budget, and
        /// <c>triangles &gt; <see cref="FPNavTuning.CorridorCap"/></c> the exact one for a path to
        /// come back clamped. Both are read from the tuning this system runs on, not from the
        /// constants — an instance handed a different tuning still compiles against those. Necessary
        /// is not sufficient: past either line the failure becomes <i>reachable</i>, which is when a
        /// graph starts earning its cost.</para>
        ///
        /// <para><b>The cell size is searched, not guessed.</b> Node width scales with cell size and
        /// local triangle density, and density varies by an order of magnitude between assets, so no
        /// fixed value is safe everywhere and any formula would carry a constant fitted to whatever
        /// meshes it was measured on. Instead this derives at
        /// <c><see cref="FPNavMesh.GridCellSize"/> * 16</c> and halves until the widest node fits
        /// <see cref="FPNavTuning.CorridorCap"/>, taking the first that does — the largest node that
        /// fits, so the graph has the fewest nodes it can. Deriving gets <i>cheaper</i> as cells grow,
        /// so the ladder spends its cheap probes first. The rungs are powers of two on purpose:
        /// <see cref="FPNavAbstractGraph.MaxNodeDiameter"/> is a lower bound off a tree, so landing
        /// with room to spare is worth more than landing exactly on the cap.</para>
        ///
        /// <para><b>The search runs once, here.</b> A rebake re-derives the graph at the size this
        /// chose (see <see cref="SwapNavMesh(FPNavMesh)"/>), so a swap pays one derivation rather
        /// than another ladder.</para>
        ///
        /// <para><b>Never throws</b>, unlike <see cref="SetAbstractGraph"/> — a game wires this on
        /// its initialization path, where an exception is a failed boot. Every refusal is a value.
        /// Call it before the first tick, from the deterministic setup path, on every peer.</para>
        ///
        /// <para><b>What that rests on, since it is not obvious from here.</b> The method reads the
        /// mesh this system was constructed with and never checks it for null — and does not need
        /// to: <see cref="FPNavMeshQuery"/> and <see cref="FPNavMeshPathfinder"/> size their buffers
        /// from <c>navMesh.Triangles.Length</c> in their constructors, so a system cannot be built
        /// around a null mesh at all. <b>A peer with no navigation is a null SYSTEM, not a system
        /// with a null mesh</b> — the engine's fingerprint path handles that with <c>?.</c> at the
        /// call site, which is what the null branch in <see cref="GetNavFingerprint"/> is defending
        /// rather than describing. The only other throw in reach is
        /// <see cref="FPNavAbstractGraph"/>'s refusal of a non-positive cell size, and the ladder
        /// returns <see cref="FPNavAbstractGraphInstall.NoCellSizeFits"/> as a value before it can
        /// happen.</para>
        ///
        /// <para><b>Every outcome is logged, including the ones where nothing happens.</b> A game
        /// reads this decision from its boot log, and a path that stays silent is one nobody can
        /// tell apart from the call not having run — which is the failure mode this whole feature
        /// exists to remove, one level up. The lines carry the numbers the decision was made from,
        /// so <i>legs are off</i> can be checked rather than taken on faith.</para>
        /// </summary>
        /// <param name="cellSize">
        /// The cell size the ladder settled on, or zero when nothing was installed. Hand this to
        /// <see cref="FPNavAbstractGraph"/> to rebuild the identical graph — a tool that draws the
        /// partition, or a peer that builds its own, needs the value and cannot re-derive it.
        /// </param>
        /// <param name="costFold">How per-node cost is folded; build identity, same on every peer.</param>
        /// <param name="areaMask">The plan mask the graph is derived for; build identity.</param>
        /// <param name="exhaustionOnly">
        /// Ask only whether a flat search can run out of budget, not whether a corridor can come back
        /// clamped. This is what the constructor's automatic install asks: the clamp is a
        /// walk-and-replan the flat planner already handles, and counting it would put a graph on
        /// every mesh past the corridor cap (128 triangles). The default, false, is the original
        /// contract — either condition is enough.
        /// </param>
        public FPNavAbstractGraphInstall TryInstallAbstractGraphIfBeneficial(
            out FP64 cellSize,
            FPNavAbstractCostFold costFold = FPNavAbstractCostFold.Min,
            int areaMask = DEFAULT_AREA_MASK,
            bool exhaustionOnly = false)
            => InstallAbstractGraphCore(out cellSize, costFold, areaMask, exhaustionOnly, automatic: false);

        private bool _autoGraphReplacedLogged;

        // The body of TryInstallAbstractGraphIfBeneficial. `automatic` is the constructor: it changes
        // the words (so a boot log says which of the two did this) and one level — a mesh no cell
        // size fits is an error to a game that asked for legs and a warning to one that got the
        // default, because the default path must not fail a boot log for a mesh it merely cannot
        // partition.
        private FPNavAbstractGraphInstall InstallAbstractGraphCore(
            out FP64 cellSize, FPNavAbstractCostFold costFold, int areaMask,
            bool exhaustionOnly, bool automatic)
        {
            cellSize = FP64.Zero;
            string who = automatic ? "planning in legs (automatic)" : "planning in legs";

            // A game that picked its own cell size has already made this decision. Overwriting it
            // would silently move that game's agents onto a different partition.
            if (_abstractGraph != null)
            {
                _logger?.KInformation(
                    $"[FPNavAgentSystem] {who}: already on and left alone — " +
                    $"{_abstractGraph.NodeCount} nodes, widest node {_abstractGraph.MaxNodeDiameter} " +
                    $"hops across ({_abstractGraph.MaxLegCorridorTriangles} corridor triangles). " +
                    $"This helper does not overwrite a graph the game installed itself; " +
                    $"its cell size is that game's choice and replacing it would move every agent " +
                    $"onto a partition nobody picked.");
                return FPNavAbstractGraphInstall.AlreadyInstalled;
            }

            int triangles = _navMesh.Triangles.Length;
            bool canExhaust = triangles > _tuning.MaxIterations;
            bool canClamp = !exhaustionOnly && triangles > _tuning.CorridorCap;
            if (!canExhaust && !canClamp)
            {
                // The quiet answer, said out loud. This is the branch a small stage takes every
                // boot, and leaving it silent makes "legs are off because the mesh does not need
                // them" indistinguishable from "the call was never wired" — two states with very
                // different fixes. The numbers are here so the margin can be read: a stage growing
                // toward a cap shows it in this line before it crosses one. The automatic path says
                // which condition it asked about, because it asks about one fewer than an explicit
                // call does.
                // KLogger takes an interpolated-string handler: a ternary between two literals is
                // a plain string and binds to the ref overload (CS1620), so pick the text first.
                string notNeeded = exhaustionOnly
                    ? $"[FPNavAgentSystem] {who}: off — not needed. {triangles} triangles is within " +
                      $"the search budget ({_tuning.MaxIterations}), so a flat search cannot run out on " +
                      $"this mesh and a graph would move the navigation fingerprint for nothing " +
                      $"(the corridor cap {_tuning.CorridorCap} is not asked about here — a clamped " +
                      $"corridor is walked and re-planned, not failed)."
                    : $"[FPNavAgentSystem] {who}: off — not needed. {triangles} triangles " +
                      $"is within both the corridor cap ({_tuning.CorridorCap}) and the search budget " +
                      $"({_tuning.MaxIterations}), so neither failure is reachable on this mesh and a " +
                      $"graph would move the navigation fingerprint for nothing.";
                _logger?.KInformation($"{notNeeded}");
                return FPNavAbstractGraphInstall.NotNeeded;
            }

            FP64 floor = _navMesh.GridCellSize;
            if (floor <= FP64.Zero)
            {
                _logger?.KError(
                    $"[FPNavAgentSystem] TryInstallAbstractGraphIfBeneficial: the mesh has no " +
                    $"broadphase cell size, so there is no ladder to walk. Install a graph by hand " +
                    $"with a cell size you measured, or leave legs off.");
                return FPNavAbstractGraphInstall.NoCellSizeFits;
            }

            FP64 candidate = floor * FP64.FromInt(LEG_CELL_LADDER_START_MULTIPLE);
            FP64 two = FP64.FromInt(2);
            FPNavAbstractGraph chosen = null;
            int probes = 0;

            while (true)
            {
                // Measure first: a candidate is rejected on its diameter alone, and measuring is
                // the partition and that diameter — a few percent of a derivation. Only the one
                // that fits is built, and it is built by the ordinary constructor, so nothing here
                // ever holds a graph that is missing its edges or its pair table.
                int corridor = FPNavAbstractGraph.MeasureLegCorridorTriangles(
                    _navMesh, candidate, costFold, areaMask);
                probes++;
                _ladderProbes++;

                if (corridor <= _tuning.CorridorCap)
                {
                    chosen = new FPNavAbstractGraph(_navMesh, candidate, costFold, areaMask, _logger);
                    _ladderPairTablesBuilt++;
                    break;
                }

                if (candidate <= floor)
                    break;

                candidate = candidate / two;
                if (candidate < floor)
                    candidate = floor;
            }

            if (chosen == null)
            {
                string noFit =
                    $"[FPNavAgentSystem] {who}: no cell size fits. " +
                    $"Down to the mesh's own broadphase cell ({floor.ToDouble():F2}) the widest node " +
                    $"still spans more than the corridor cap {_tuning.CorridorCap}, so every leg " +
                    $"would come back clamped. This mesh plans flat; " +
                    $"DebugIterationExhaustedCount says whether that costs anything.";
                // An error to a game that asked for legs, a warning to one that got the default:
                // the automatic path must not put an error in the boot log of a mesh it merely
                // cannot partition.
                if (automatic) _logger?.KWarning($"{noFit}"); else _logger?.KError($"{noFit}");
                return FPNavAbstractGraphInstall.NoCellSizeFits;
            }

            // Through the same door a hand-wired install uses, so the mesh-identity and cap
            // invariants have one implementation. Neither can throw from here: the graph was
            // derived against _navMesh, and the ladder only stops on a diameter within the cap.
            SetAbstractGraph(chosen);
            cellSize = candidate;

            // Which threshold was true, not just that one was — the prescription is the same but
            // the diagnosis is not, and the next person to read this log needs the difference.
            string why = canExhaust
                ? $"{triangles} triangles is past the iteration budget {_tuning.MaxIterations}, so a " +
                  $"flat search can run out before it decides" +
                  (canClamp ? $" (and past the corridor cap {_tuning.CorridorCap})" : "")
                : $"{triangles} triangles is past the corridor cap {_tuning.CorridorCap}, so a flat " +
                  $"path can come back clamped (the iteration budget {_tuning.MaxIterations} is not " +
                  $"reachable on this mesh)";

            _logger?.KInformation(
                $"[FPNavAgentSystem] {who} at cell {candidate.ToDouble():F2} " +
                $"({probes} derivation(s) on the ladder from {floor.ToDouble():F2} x " +
                $"{LEG_CELL_LADDER_START_MULTIPLE}, the pair table only for this one): {chosen.NodeCount} nodes, {chosen.EdgeCount} " +
                $"edges, widest node {chosen.MaxNodeDiameter} hops across " +
                $"({chosen.MaxLegCorridorTriangles} corridor triangles) against cap " +
                $"{_tuning.CorridorCap}. {why}. Navigation fingerprint is now " +
                $"0x{GetNavFingerprint():X16} — replays recorded without a graph will refuse.");

            return FPNavAbstractGraphInstall.Installed;
        }

        private int _abstractSearchFailedCount;
        private int _legResolveFailedCount;
        private int _legAdvanceCount;

        /// <summary>
        /// Diagnostic: node routes the abstract search could not find. <b>This is a failure the leg
        /// planner introduces</b>, and it does not reach
        /// <see cref="FPNavMeshPathfinder.DebugIterationExhaustedCount"/> — that counter belongs to
        /// the triangle search, which never runs when the abstract one gives up first. Reading only
        /// that one would show a clean budget while every unit stands still.
        /// </summary>
        public int DebugAbstractSearchFailedCount => _abstractSearchFailedCount;

        /// <summary>
        /// Diagnostic: legs the abstract search chose that the real A* then could not solve. Two
        /// unrelated defects surface here — an abstract cost on a different scale than the real one,
        /// and a node that claims a crossing the mesh does not have — and neither shows as a desync,
        /// because every peer computes the same wrong route. The agent falls back to planning
        /// straight at its destination, so a rising count is a correctness signal, not a stall.
        /// </summary>
        public int DebugLegResolveFailedCount => _legResolveFailedCount;

        /// <summary>Diagnostic: legs completed and handed on to the next one.</summary>
        public int DebugLegAdvanceCount => _legAdvanceCount;

        private int _legAdvanceRepeatCount;

        /// <summary>
        /// Diagnostic: crossings a plan asked for that the agent had already finished, skipped in
        /// favour of the next one. Reaching a portal does not put the agent past it — the portal
        /// IS the shared edge — so the node it stands in has not changed yet and the abstract search
        /// would otherwise hand back the same hop.
        ///
        /// <para><b>Expect exactly two per leg, everywhere</b> — not "roughly one, two at a lattice
        /// corner", which is what this said before it was measured. Both skips fire on every plan
        /// after the first, because the second reach test compares against
        /// <see cref="NavAgentComponent.PathTarget"/>, which after any successful plan holds the
        /// crossing this leg is walking toward; the plan that follows the hand-off asks for that
        /// same crossing or its neighbour and is inside that ball by construction. Measured on open
        /// ground at cell 4, 8 and 16 and reach radius 2.5 to 100, the ratio to
        /// <see cref="DebugLegAdvanceCount"/> was 2.0 in every configuration.</para>
        ///
        /// <para>What it must NOT do is stay at zero while <see cref="DebugLegAdvanceCount"/>
        /// runs several times the number of nodes on the route — that was the shape of the defect
        /// this replaced, where the agent circled at each boundary until it drifted across.</para>
        /// </summary>
        public int DebugLegAdvanceRepeatCount => _legAdvanceRepeatCount;

        private int _legEndedOnPlanTickCount;
        private bool _legReachRadiusWarned;

        /// <summary>
        /// Diagnostic: legs that ended on the very tick they were planned — the agent was already
        /// inside <c>ReachRadius</c> of its leg target when the plan was made, so it travelled
        /// nothing before the hand-off sent it back to the planner.
        ///
        /// <para><b>This is the counter that says the reach radius is too wide for the node.</b>
        /// The radius is <c>v² / a</c>, the tightest arc the agent can hold, and nothing bounds it
        /// against the graph's <see cref="FPNavAbstractGraph.CellSize"/>. Once it exceeds a node's
        /// width every leg target is "reached" the moment it is chosen, and because the hand-off
        /// clears <c>LastRepathTick</c> to bypass the repath cooldown, the agent runs a full A*
        /// every tick — the opposite of what planning in legs is for.</para>
        ///
        /// <para><b>Read it as a rate, and do not read
        /// <see cref="DebugLegAdvanceRepeatCount"/> for this.</b> That one rises about once per leg
        /// in healthy operation AND about once per leg here, so its ratio to
        /// <see cref="DebugLegAdvanceCount"/> is 1:1 either way and separates nothing. This one is
        /// near zero when legs are working — a leg that takes even one tick to walk does not land
        /// here — and approaches one per agent per tick when the radius has swallowed the node.</para>
        ///
        /// <para>An occasional count is not a defect: a portal can genuinely be a step away.</para>
        /// </summary>
        public int DebugLegEndedOnPlanTickCount => _legEndedOnPlanTickCount;

        private int _partialHandoffCount;

        /// <summary>
        /// Diagnostic: times an agent reached the end of a PARTIAL corridor and was handed back to
        /// the planner (<see cref="FPNavTuning.PartialPathOnExhaustion"/>). The hand-off is the leg
        /// hand-off — the same code, the same reach radius — and with no abstract graph installed
        /// every hand-off is one of these, so this count is exact there. <b>With a graph installed
        /// the two cannot be told apart at the hand-off</b> (nothing in the frame says which kind
        /// of target <c>PathTarget</c> is, and adding a field would change the wire), so partial
        /// ends are then counted in <see cref="DebugLegAdvanceCount"/> instead; the pathfinder's
        /// <c>DebugPartialPathCount</c> stays exact either way, counting them where they are made.
        /// </summary>
        public int DebugPartialHandoffCount => _partialHandoffCount;

        private int _partialEndedOnPlanTickCount;

        /// <summary>
        /// Diagnostic: partial corridors whose end was already inside <c>ReachRadius</c> on the tick
        /// they were planned — the twin of <see cref="DebugLegEndedOnPlanTickCount"/> for the
        /// no-graph hand-off, and exact only there (with a graph the hand-off cannot tell a partial
        /// end from a portal). The best node can never sit inside the radius (its progress is at
        /// least the reach radius, and progress cannot exceed distance), so this counts only ends
        /// that the corridor cap moved: a chain that doubled back toward the agent and was clipped
        /// where it passed close by. Each count is one full-budget search spent on a hop that moved
        /// nothing, and because the hand-off clears the repath cooldown they arrive in bursts, one per
        /// tick, until the agent's own motion carries the clipped end out of the radius.
        /// </summary>
        public int DebugPartialEndedOnPlanTickCount => _partialEndedOnPlanTickCount;

        private int _maskFallbackCount;

        /// <summary>
        /// Diagnostic: plans that skipped the abstract graph because the agent's resolved plan mask
        /// is not the one the graph was derived under. Those agents keep the flat path — correct,
        /// but they also keep the cost the leg planner exists to remove, so this is the ratio that
        /// says whether a per-mask graph would earn its memory. An agent whose override happens to
        /// resolve to the graph's own mask does NOT land here; only a genuinely different mask does,
        /// so a game whose agents all plan under the mask its graph was built for never moves it.
        /// </summary>
        public int DebugMaskFallbackCount => _maskFallbackCount;

        /// <summary>
        /// What the last leg resolution actually produced. Two of these mean the search that
        /// follows is short or shortened; the rest mean it is flat and runs the whole way to the
        /// destination — which is the search that can run out of budget.
        /// </summary>
        private enum LegPlanKind : byte
        {
            /// <summary>The hierarchy handed back a portal: the search is one leg long.</summary>
            Legs = 0,

            /// <summary>Start and goal share a node. Nothing to shorten, and nothing far.</summary>
            SameNode = 1,

            /// <summary>No graph installed at all — the case this whole feature is opt-in about.</summary>
            NoGraph = 2,

            /// <summary>A graph is installed, but this agent plans under a mask it was not derived for.</summary>
            MaskMismatch = 3,

            /// <summary>The agent's own triangle could not be found, so no node could be either.</summary>
            StartOffMesh = 4,

            /// <summary>The destination is not on the mesh under this agent's plan mask.</summary>
            GoalOffMesh = 5,

            /// <summary>Both triangles resolved, but the graph does not name a node for one of them.</summary>
            NodeUnknown = 6,

            /// <summary>The abstract search found no route between the two nodes.</summary>
            NoAbstractRoute = 7,

            /// <summary>Legs planned a portal the triangle search could not reach, so this fell back.</summary>
            AbstractRouteUnreachable = 8,
        }

        private LegPlanKind _lastLegPlanKind;

        /// <summary>
        /// True when the search about to run is flat and goes the whole way to the destination —
        /// <b>not</b> simply "no graph installed". A graph can be present and still not shorten this
        /// particular search, and those are precisely the plans that keep the failure the graph was
        /// installed to remove while looking, from the outside, like a healthy leg-planning build.
        /// </summary>
        private static bool PlansFlatToDestination(LegPlanKind kind)
            => kind != LegPlanKind.Legs && kind != LegPlanKind.SameNode;

        private int _exhaustedWithoutLegsCount;
        private bool _exhaustedWithoutLegsWarned;

        /// <summary>
        /// Diagnostic: searches that ran out of <see cref="FPNavTuning.MaxIterations"/> while the
        /// hierarchy was not shortening them. <b>The conjunction is the point.</b>
        /// <see cref="FPNavMeshPathfinder.DebugIterationExhaustedCount"/> on its own counts every
        /// budget overrun including the ones inside a leg, and neither counter alone says the thing
        /// a reader needs: <i>this agent stopped, and nothing was making its search local</i>.
        ///
        /// <para>This exists because that fact was already knowable and nobody read it. The budget
        /// counter was rising, correctly, through an entire session of units standing still — it
        /// took opening the visualizer to notice. A value is not a signal until something says what
        /// it means.</para>
        ///
        /// <para><b>Not proof that a route exists.</b> Exhaustion means the search stopped before it
        /// decided, not that it would have succeeded with more budget. See the one-time warning for
        /// the wording that keeps this honest.</para>
        /// </summary>
        public int DebugExhaustedWithoutLegsCount => _exhaustedWithoutLegsCount;

        /// <summary>
        /// Called after the plan's searches, with the budget counter as it stood before them and
        /// what the plan ended with. Warns once per system, then counts silently — a per-tick line
        /// from eight hundred agents buries the thing it is trying to say.
        ///
        /// <para>The outcome is passed in rather than read off the pathfinder: the delta can span
        /// two searches (a leg that exhausted, then the flat retry), and the pathfinder's
        /// last-call flags describe only the second. The retry can find a whole path; saying "no
        /// path" there was the one thing the line got wrong.</para>
        /// </summary>
        private void NoteExhaustionWithoutLegs(int exhaustedBefore, bool found, bool partial)
        {
            if (_pathfinder.DebugIterationExhaustedCount == exhaustedBefore)
                return;
            if (!PlansFlatToDestination(_lastLegPlanKind))
                return;

            _exhaustedWithoutLegsCount++;
            if (_exhaustedWithoutLegsWarned)
                return;
            _exhaustedWithoutLegsWarned = true;

            // Deliberately not "turn legs on and this is fixed". Exhaustion says the search stopped
            // before it decided; it does not say a route was there to find. What is true is the
            // mechanism: the budget is per search, and a leg is a shorter search.
            // No graph has two very different causes since legs turned on by default (0.13): the
            // automatic install is off in this tuning, or it is on and found no cell size that fits
            // (the boot log says which — NotNeeded cannot be it, because a mesh within the budget
            // cannot exhaust). "Wire the one-line helper" is only the prescription for the first.
            string what = _lastLegPlanKind == LegPlanKind.NoGraph
                ? (_tuning.AutoInstallAbstractGraph
                    ? $"no abstract graph is installed on this system — the automatic install is on, " +
                      $"so either no cell size fit this mesh (see the boot log) or the game removed it"
                    : $"no abstract graph is installed on this system — the automatic install is off " +
                      $"in this tuning (FPNavTuning.AutoInstallAbstractGraph)")
                : $"an abstract graph is installed but this plan did not use it ({_lastLegPlanKind})";

            // What the unit ended with changes what it does about it, not the diagnosis: the
            // search still stopped before it decided, and legs are still the way to make it not.
            // Three outcomes, not two: a whole path can follow an exhaustion when the flat retry
            // after an exhausted leg search finds one, or when the goal was already in the open set
            // as the budget ran out (PartialPathOnExhaustion hands that back whole).
            string outcome = !found
                ? $"The unit got no path. "
                : partial
                    ? $"The unit got a PARTIAL corridor (PartialPathOnExhaustion) and will walk to the " +
                      $"closest point the search reached, then plan again from there. "
                    : $"The unit got a whole path anyway (the goal was already in hand when the budget " +
                      $"ran out, or a flat retry after an exhausted leg search found it). ";

            _logger?.KWarning(
                $"[FPNavAgentSystem] a path search ran out of its {_tuning.MaxIterations}-triangle " +
                $"budget on a {_navMesh.Triangles.Length}-triangle mesh, and {what}. The search " +
                $"stopped before it decided — that is not the same as there being no route. {outcome}" +
                $"Planning in legs makes each search local rather than raising the budget; " +
                $"it is on by default, TryInstallAbstractGraphIfBeneficial is the explicit form, " +
                $"and either moves this game's navigation fingerprint. Further occurrences are " +
                $"counted in DebugExhaustedWithoutLegsCount, not logged.");
        }

        /// <summary>
        /// Extracts this system's NavMesh boundary as ORCA static obstacles into the current
        /// avoidance. MUST be called AFTER SetAvoidance. No-op if avoidance or NavMesh is null.
        /// Load-time only (not the hot path) — allocation here is fine. Idempotent: re-extracts
        /// and replaces on each call (safe across stage changes).
        /// </summary>
        public void LoadNavMeshObstacles()
        {
            if (_avoidance == null)
                return;

            // This runs on every SwapNavMesh, not just at load — see the note on this method's
            // summary. The scratch is what keeps the re-extract from re-allocating its working
            // set: the visited flags, the segment map, the counting-sort cursor, and this pair.
            // It is a field rather than a shared pool because a snapshot extract can happen on a
            // worker thread; ownership by the object that re-extracts is what makes serial use
            // structural (FPNavMeshObstacleExtractor.ExtractScratch).
            //
            // EVERYTHING here comes back OVERSIZED — the ring vertices and offsets as well as the
            // CSR pair. Read them through the counts, never through Length: the CSR through
            // (_triSegList[_triSegStart[t] .. _triSegStart[t+1])), the other two through the
            // counts handed to LoadObstacles below. Do not add a Length-based loop over any of
            // them; that is the whole reason this path allocates nothing per swap.
            _extractScratch = _extractScratch ?? new FPNavMeshObstacleExtractor.ExtractScratch();
            FPNavMeshObstacleExtractor.Extract(_navMesh, _extractScratch,
                out var vertices, out int vertexCount,
                out var polygonOffsets, out int polygonCount,
                out _triSegStart, out _triSegList);
            _avoidance.LoadObstacles(vertices, vertexCount, polygonOffsets, polygonCount);
            _obstacleLoadGenerationCache = _avoidance.ObstacleLoadGeneration;
            // The baked asset records its own bake Agent Radius (VERSION 3): apply it as the
            // obstacle inset so clearance is not double-charged (boundary inset + full radius).
            // Riding the asset keeps lockstep peers symmetric by construction — no hand-synced
            // constant. Consumers may still override the field after this call.
            _avoidance.ObstacleRadiusInset = _navMesh?.BakeAgentRadius ?? FP64.Zero;

            // Size the BFS visited-stamp to the navmesh (load-time; hot path stays GC-0). The
            // frontier/candidate buffers are fixed-cap and allocated once with the system.
            //
            // The generation reset belongs INSIDE the grow branch and nowhere else. SwapNavMesh
            // re-enters this method mid-match, and the array is only reallocated when it is too
            // SMALL — so a rebake that does not grow the triangle count (removing a building)
            // keeps stamps from before the swap. Restarting the counter at 0 there would make
            // generation 1 alias every slot still holding 1, and the BFS would treat those
            // triangles as visited and drop their wall segments. Scoped like this the counter
            // only restarts alongside a freshly zeroed array, and on reuse it keeps climbing
            // past every stale value, so a collision is impossible rather than merely unlikely.
            int triCount = _navMesh?.TriangleCount ?? 0;
            if (_bfsStamp == null || _bfsStamp.Length < triCount)
            {
                _bfsStamp = new int[triCount];
                _bfsGeneration = 0;
            }
        }

        /// <summary>
        /// Swaps in a rebaked navmesh, rebinding the query, pathfinder and funnel this system
        /// already holds. This is the cheap form: their working arrays are kept and only grown
        /// when the new mesh has more triangles, which on a large stage is the difference between
        /// a megabyte-and-a-half per building and nothing.
        ///
        /// <para><b>Follow with <see cref="ReseedAgents"/> before the next Update.</b> The swap is
        /// one line now, which makes it easy to read as the whole job — it is not. Every agent is
        /// still holding a triangle index and corridor that point into the mesh being replaced,
        /// and both are hashed frame state, so skipping the reseed desyncs the peer rather than
        /// merely misplacing an agent.</para>
        ///
        /// <para>MUST be invoked only from the deterministic command stream (same tick, same order
        /// on all peers), at a tick boundary. Re-extracts ORCA obstacles from the new mesh (R5)
        /// and invalidates the graph-local CSR.</para>
        ///
        /// <para>Requires the trio to be installed already — by the constructor or by the
        /// four-argument overload. That is the one thing this form cannot do that the other can:
        /// it rebinds what is there rather than replacing it.</para>
        /// </summary>
        public void SwapNavMesh(FPNavMesh newMesh)
        {
            if (newMesh == null)
                throw new System.ArgumentException("FPNavAgentSystem.SwapNavMesh: newMesh must be non-null");
            if (_query == null || _pathfinder == null || _funnel == null)
                throw new System.InvalidOperationException(
                    "FPNavAgentSystem.SwapNavMesh(mesh): no query/pathfinder/funnel to rebind. " +
                    "Install them through the constructor or the four-argument overload first.");

            _query.Rebind(newMesh);
            _pathfinder.Rebind(newMesh);
            _funnel.Rebind(newMesh);

            InstallSwappedMesh(newMesh);
        }

        /// <summary>
        /// Swaps in a rebaked navmesh, taking a query, pathfinder and funnel the caller built
        /// against <paramref name="newMesh"/>. Prefer <see cref="SwapNavMesh(FPNavMesh)"/>, which
        /// reuses the existing trio instead of allocating a new one; this form stays for games
        /// that own their own instances, and for tests that need to install a specific trio.
        ///
        /// <para><b>Follow with <see cref="ReseedAgents"/> before the next Update</b> — see the
        /// other overload for why. MUST be invoked only from the deterministic command stream
        /// (same tick, same order on all peers), at a tick boundary.</para>
        /// </summary>
        public void SwapNavMesh(FPNavMesh newMesh, FPNavMeshQuery query,
            FPNavMeshPathfinder pathfinder, FPNavMeshFunnel funnel)
        {
            if (newMesh == null || query == null || pathfinder == null || funnel == null)
                throw new System.ArgumentException("FPNavAgentSystem.SwapNavMesh: all arguments must be non-null");

            // The other door into the trio. Without this the constructor check is bypassable: build
            // a consistent stack, then swap in one that is not.
            RequireSameTuning(query.Tuning, nameof(query));
            RequireSameTuning(pathfinder.Tuning, nameof(pathfinder));
            RequireSameTuning(funnel.Tuning, nameof(funnel));

            _query = query;
            _pathfinder = pathfinder;
            _funnel = funnel;

            InstallSwappedMesh(newMesh);
        }

        /// <summary>
        /// Takes the graph <see cref="PrepareAbstractGraphFor"/> built, if it is genuinely the one
        /// this swap needs. Returns false — and costs nothing but two comparisons — otherwise.
        ///
        /// <para><b>Two checks, and neither is redundant.</b></para>
        /// <list type="number">
        /// <item><b>The same INSTANCE.</b> <see cref="SetAbstractGraph"/> enforces
        /// <c>ReferenceEquals(graph.CurrentMesh, _navMesh)</c> as an invariant — node ids index one
        /// mesh's triangles — so a graph derived from an identical-but-separate mesh is not
        /// installable no matter how equal its contents are. Identity is required, not preferred.</item>
        /// <item><b>The same CONTENT.</b> Identity alone is not enough in the other direction: the
        /// rebake driver pools meshes and <c>CommitSwap</c> retires the one it replaces, so a
        /// reference can be recycled and rewritten. Adopting on identity alone would then install a
        /// graph indexing geometry that is no longer there — and every peer would do it identically,
        /// so the state hash would agree and nothing would report it. The fingerprint is a fold over
        /// the mesh; against a 40 ms derivation it is free.</item>
        /// </list>
        ///
        /// <para>On adoption the outgoing graph becomes the spare, so the pair alternates and no
        /// allocation happens after the first prepare.</para>
        /// </summary>
        private bool TryAdoptPreparedGraph(FPNavMesh newMesh)
        {
            if (_spareGraph == null || _preparedMesh == null)
                return false;

            if (!ReferenceEquals(_preparedMesh, newMesh)
                || unchecked((long)FPNavMeshRebaker.ComputeFingerprint(newMesh)) != _preparedFingerprint)
            {
                // Held, not dropped: the mesh it was built for may still be the one that arrives.
                _graphPreparedMissedCount++;
                return false;
            }

            FPNavAbstractGraph outgoing = _abstractGraph;
            _abstractGraph = _spareGraph;
            _spareGraph = outgoing;
            _preparedMesh = null;
            _graphPreparedAdoptedCount++;
            return true;
        }

        /// <summary>
        /// The part of a swap that is the same whichever way the trio got bound: adopt the mesh,
        /// drop the graph-local CSR, re-extract obstacles, and say so. Shared so the two
        /// overloads cannot drift — a swap that skipped any of this would be observably different.
        /// </summary>
        private void InstallSwappedMesh(FPNavMesh newMesh)
        {
            _navMesh = newMesh;

            // The abstract graph is derived from the mesh, so a swap invalidates it wholesale.
            // Agents mid-leg are handed back to the planner by ReseedAgents, which clears HasPath.
            if (_abstractGraph != null)
            {
                // Adopted or derived, the graph that comes out of here is the same one — that is
                // the whole safety argument for preparing ahead, and the reason the cap check below
                // sits outside this branch rather than inside either arm of it.
                if (!TryAdoptPreparedGraph(newMesh))
                {
                    // The whole derivation, on the deterministic command path. Measured at 41 ms
                    // on the Field asset at the cell size the install ladder picks (cell 32,
                    // Release, tiered compilation off; the pair table is most of it — 10 ms before
                    // hops were priced portal to portal), against the ~2 ms that time-slicing the
                    // rebake itself works to stay inside — roughly twenty times the budget it lands
                    // in. Timing it is opt-in; counting is not.
                    long startTicks = DebugTimeGraphDerivation
                        ? System.Diagnostics.Stopwatch.GetTimestamp() : 0L;

                    _abstractGraph.Rebind(newMesh);
                    _graphRederiveCount++;

                    // Once per system: the swap just paid a whole derivation on the deterministic
                    // command path because nothing prepared one ahead. With legs on by default this
                    // is the path a large-mesh game with runtime rebakes lands on unless it wires
                    // PrepareAbstractGraphFor — a cost of wall clock, not of determinism (the graph
                    // is the same either way), which is why this is a warning and not an error.
                    if (!_prepareWarned)
                    {
                        _prepareWarned = true;
                        _logger?.KWarning(
                            $"[FPNavAgentSystem] a navmesh swap re-derived the abstract graph on the " +
                            $"deterministic command path ({newMesh.Triangles.Length} triangles at cell " +
                            $"{_abstractGraph.CellSize.ToDouble():F2}) because no graph was prepared for " +
                            $"this mesh ahead of the swap. The result is identical either way; the cost " +
                            $"is the tick that commits the rebake, which a sliced rebake exists to keep " +
                            $"small. Call PrepareAbstractGraphFor(mesh) from the rebake's off-tick path " +
                            $"before committing, and the swap adopts it for free. Said once; " +
                            $"DebugGraphRederiveCount counts the rest.");
                    }

                    if (DebugTimeGraphDerivation)
                    {
                        double ms = (System.Diagnostics.Stopwatch.GetTimestamp() - startTicks)
                            * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
                        _logger?.KInformation(
                            $"[FPNavAgentSystem] abstract graph re-derived on the swap in " +
                            $"{ms:F2} ms at cell {_abstractGraph.CellSize.ToDouble():F2} " +
                            $"({_abstractGraph.NodeCount} nodes, {_abstractGraph.EdgeCount} edges, " +
                            $"{newMesh.Triangles.Length} triangles) — swap #{_graphRederiveCount}. " +
                            $"This runs on the deterministic command path, inside the tick budget a " +
                            $"sliced rebake exists to protect. PrepareAbstractGraphFor moves it off.");
                    }
                }

                // SetAbstractGraph refuses a graph whose widest node exceeds the corridor cap, but a
                // rebake can widen one AFTER that check passed: carving merges walkable regions that
                // the base mesh kept apart. Throwing here is not on the table — this runs inside a
                // swap, on the deterministic command path — so it is reported and the run continues
                // with the same clamping the flat planner had. DebugCorridorTruncatedCount is what
                // shows the cost while the log says why.
                if (_abstractGraph.MaxLegCorridorTriangles > _tuning.CorridorCap)
                {
                    _logger?.KError(
                        $"[FPNavAgentSystem] after the swap a leg through the widest node can ask " +
                        $"for {_abstractGraph.MaxLegCorridorTriangles} triangles against a corridor cap of " +
                        $"{_tuning.CorridorCap}. The rebake widened a node past what SetAbstractGraph " +
                        $"accepted, so legs through it come back clamped. Derive with a smaller cell " +
                        $"size, or stop installing a graph for this stage.");
                }
            }

            // Invalidate the graph-local obstacle CSR; LoadNavMeshObstacles rebuilds it (and the
            // BFS stamp sizing + radius inset) from the new mesh when avoidance is wired.
            _triSegStart = null;
            _triSegList = null;
            _obstacleLoadGenerationCache = -1;
            LoadNavMeshObstacles();

            _logger?.KInformation($"[FPNavAgentSystem] navmesh swapped: {newMesh.Triangles.Length} triangles, " +
                $"fingerprint 0x{GetNavFingerprint():X16}{DonorClause(_abstractGraph)}");
        }

        /// <summary>
        /// What the prepared graph did with the donor it was offered, as a clause for the swap line.
        /// A refusal is otherwise invisible: the reason is an internal string with no reader, and a
        /// silently refused donor costs the full derivation with nothing but the missing clause to
        /// say so. No line of its own — an RTS re-bakes once per placement, so one per re-bake is
        /// spam (Plan-IncrementalRederive, change 4). Built inside the interpolation hole, so a
        /// disabled log level allocates nothing.
        /// </summary>
        private static string DonorClause(FPNavAbstractGraph graph)
        {
            if (graph == null)
                return "";
            if (graph.DebugRebindDonorUsed)
                return $"; the prepared graph took {graph.DebugRebindNodesReused} of " +
                    $"{graph.NodeCount} nodes' rows from the previous one";
            if (graph.DebugRebindDonorIgnored != null)
                return $"; the prepared graph reused nothing — {graph.DebugRebindDonorIgnored}";
            return "";
        }

        /// <summary>
        /// Reseeds every agent after a navmesh swap: re-queries
        /// CurrentTriangleIndex from the agent position on the new mesh and invalidates the
        /// cached corridor (both live in the hashed frame state — stale values over rebuilt
        /// triangle indices would corrupt determinism and gameplay alike). Agents that still
        /// have a destination and were moving, planning or
        /// <see cref="FPNavAgentStatus.Blocked"/> are set to PathPending with the repath cooldown
        /// bypassed, so the next Update repaths them deterministically. Arrived and PathFailed are
        /// left alone on purpose: the first would re-plan every rebake, and re-trying "no route
        /// exists" on every mesh swap is a policy the game decides, not this pass. Agents whose
        /// position no longer lies on the mesh (e.g. standing inside a carved hole — placement
        /// rules should prevent this) fail their path and are reported.
        /// </summary>
        public unsafe void ReseedAgents(ref Frame frame, EntityRef[] entities, int entityCount)
        {
            // The caller hands in its own array and count, and nothing about that pair tells the
            // engine whether it is the WHOLE agent set or a truncated view of it. An agent that
            // is missed here keeps a CurrentTriangleIndex and corridor that index the mesh that
            // was just replaced — and because both fields are hashed frame state, a peer that
            // reseeds fewer agents than another diverges rather than merely misbehaving.
            //
            // That is not hypothetical: the Brawler seam once collected into a fixed-size array
            // and stopped at its length, so the post-fullstate swap (which can run before that
            // peer's first Update has grown the array) reseeded fewer agents than the authority.
            // The engine cannot fix the caller's bookkeeping, but it can refuse to let it pass
            // silently — which is the only reason this class of bug is expensive.
            int actual = 0;
            var audit = frame.Filter<NavAgentComponent>();
            while (audit.Next(out _))
                actual++;
            if (actual != entityCount)
            {
                _logger?.KError(
                    $"[FPNavAgentSystem] ReseedAgents: caller passed {entityCount} agent(s) but the frame " +
                    $"has {actual} — the difference is NOT reseeded and keeps corridor/triangle indices " +
                    $"into the replaced mesh. Both fields are hashed state, so peers that disagree here " +
                    $"desync. Fix the caller's collection (grow, do not truncate).");
            }

            int lost = 0;
            for (int i = 0; i < entityCount; i++)
            {
                ref var nav = ref frame.Get<NavAgentComponent>(entities[i]);

                nav.CurrentTriangleIndex = _query.FindTriangle(nav.Position.ToXZ(), nav.Position.y);
                nav.CorridorLength = 0;
                nav.PathIsValid = false;
                nav.HasPath = false;
                nav.OffCorridorTicks = 0;

                if (nav.CurrentTriangleIndex < 0)
                {
                    // Off the new mesh — typically a building was placed on top of this agent.
                    //
                    // NOTE: leaving it at -1 means it is frozen for good, not just this tick.
                    // Nothing else in the engine writes CurrentTriangleIndex back to a valid
                    // value: MoveAlongSurface returns early on startTri < 0 and hands the -1
                    // straight back, so the next rebake's reseed is the only thing that can
                    // recover it. Brawler hides this in its own layer (BotFSMSystem re-snaps
                    // nav.Position from the transform every tick and steers straight at the
                    // destination when velocity is zero), which is why the gap has never shown
                    // up in the sample.
                    //
                    // That is a deliberate deferral, not an oversight. The current behaviour is
                    // pinned by FPNavAgentOffMeshFreezeTests, whose tests are written to FAIL once
                    // a recovery path exists, and which record the conditions that should reopen
                    // the question.
                    lost++;
                    if (nav.HasNavDestination)
                        nav.Status = (byte)FPNavAgentStatus.PathFailed;
                    continue;
                }

                // Blocked belongs here and it is not obvious: that state is TERMINAL — once set,
                // ProcessSteering and ProcessMovement both return on `Status != Moving`, so no
                // engine path writes the status again. The mesh swap is the one event that can
                // make the block untrue (the building that stopped this agent is gone), and
                // leaving it out meant a demolished building left its units frozen for good.
                if (nav.HasNavDestination
                    && (nav.Status == (byte)FPNavAgentStatus.Moving
                        || nav.Status == (byte)FPNavAgentStatus.PathPending
                        || nav.Status == (byte)FPNavAgentStatus.Blocked))
                {
                    nav.Status = (byte)FPNavAgentStatus.PathPending;
                    nav.LastRepathTick = 0; // bypass the repath cooldown: repath on the next Update
                }
            }

            if (lost > 0)
                _logger?.KWarning($"[FPNavAgentSystem] ReseedAgents: {lost}/{entityCount} agent(s) off the new mesh " +
                    $"(inside a carved region?) — paths failed");
            else
                _logger?.KInformation($"[FPNavAgentSystem] ReseedAgents: {entityCount} agent(s) reseeded");
        }

        /// <summary>
        /// Bumped BY HAND whenever a change to navigation makes the SAME mesh produce a DIFFERENT
        /// corridor. Not a mesh format version and not a version of this file — it answers one
        /// question: <i>would a peer on the previous build plan the same path?</i>
        ///
        /// <para><b>When to bump.</b> Any edit that moves what <c>FindPath</c> returns for
        /// unchanged inputs: the endpoint or start lookup, the A* order or budget, the funnel, the
        /// surface walk. NOT for refactors, comments, diagnostics, or anything the frame hash
        /// cannot see. Bumping too eagerly costs a false refusal; not bumping costs a silent
        /// divergence, so when in doubt, bump.</para>
        ///
        /// <para><b>Nothing enforces this.</b> The two places most likely to move it carry a
        /// pointer comment back here (<c>FPNavMeshPathfinder</c>'s endpoint lookup and
        /// <c>FPNavMeshQuery.FindTriangleForEndpoint</c>'s tie-break), and that is the whole
        /// defence.</para>
        ///
        /// <list type="bullet">
        /// <item><description>1 — everything up to and including 0.12.0.</description></item>
        /// <item><description>2 — <c>FindPath</c>'s endpoint moved to
        /// <c>FindTriangleForEndpoint</c>, and that tie-break was narrowed to one surface.</description></item>
        /// </list>
        /// </summary>
        internal const long NAV_BEHAVIOUR_REVISION = 2;

        /// <summary>
        /// Mixer for <see cref="NAV_BEHAVIOUR_REVISION"/>: a small ordinal XORed straight in would
        /// only stir the bottom bits of a mesh fingerprint. Odd, so multiplication is invertible
        /// and distinct revisions stay distinct.
        /// </summary>
        private const long NAV_REVISION_MIXER = unchecked((long)0x9E3779B97F4A7C15UL);

        /// <summary>
        /// Cross-peer navigation fingerprint: folded into the FullState resync
        /// static-geometry check. Never 0 while a mesh is present.
        ///
        /// <para>Folds <see cref="NAV_BEHAVIOUR_REVISION"/> as well as the mesh, so the value
        /// answers "same content AND same pathfinding" rather than "same content". Two peers on
        /// builds that plan differently now differ here, and the Ready exchange compares it before
        /// the match starts.</para>
        ///
        /// <para><b>And the tuning.</b> <see cref="FPNavTuning.Digest"/> joins the fold, so the value
        /// answers "same content AND same pathfinding AND same caps". The digest is 0 for
        /// <c>FPNavTuning.Default</c> by construction, so the common path is bit-identical to what
        /// this returned before tuning entered — an existing replay is not refused by a feature that
        /// changed no behaviour.</para>
        ///
        /// <para><b>And the partial-path switch</b>, as its own term
        /// (<see cref="FPNavTuning.PartialPathDigest"/>, zero when off) rather than inside the tuning
        /// digest — the same shape as the abstract graph below, and for the same reason: a switch
        /// nobody turned on must not move anybody's fingerprint, and a knob appended to the digest's
        /// fold would move every custom tuning's.</para>
        ///
        /// <para><b>0 stays "no mesh".</b> The revision is folded INSIDE the null check on purpose:
        /// <c>KlothoEngine.FingerprintsDiffer</c> reads 0 as "not provided" and must keep doing so,
        /// or a peer with no navigation at all would be reported as a mismatch against one that has
        /// it. The <c>0 -&gt; 1</c> normalisation stays for the same reason — the fold could
        /// otherwise land on 0 by coincidence and be misread as "not provided".</para>
        /// </summary>
        public long GetNavFingerprint()
        {
            if (_navMesh == null)
                return 0;
            long fp = unchecked((long)FPNavMeshRebaker.ComputeFingerprint(_navMesh));
            fp = unchecked(fp ^ (NAV_BEHAVIOUR_REVISION * NAV_REVISION_MIXER));
            fp = unchecked(fp ^ _tuning.Digest);
            fp = unchecked(fp ^ _tuning.PartialPathDigest);
            fp = unchecked(fp ^ AbstractGraphDigest());
            return fp == 0 ? 1L : fp;
        }

        /// <summary>
        /// The abstract graph's contribution to the fingerprint — <b>0 when there is no graph</b>,
        /// the same normalisation <see cref="FPNavTuning.Digest"/> uses for its default.
        ///
        /// <para>This is what makes the leg planner a fingerprint concern rather than a revision
        /// one. Planning in legs is opt-in and its off switch is exact — with no graph installed
        /// the agent system plans the flat path it always did — so bumping
        /// <see cref="NAV_BEHAVIOUR_REVISION"/> would refuse every recorded replay for a feature
        /// nobody turned on. That is precisely what the digest normalisation above exists to
        /// prevent. What DOES have to be caught is two peers disagreeing about the graph: one with
        /// legs on and one without, or two with different cell sizes. The graph's own checksum
        /// answers all of those in one value, because it already folds its parameters along with
        /// the shape they produced.</para>
        /// </summary>
        private long AbstractGraphDigest()
            => _abstractGraph == null ? 0L : unchecked((long)_abstractGraph.Checksum);

        /// <summary>
        /// Number of obstacle vertices currently loaded into the avoidance (0 if no avoidance).
        /// Setup-time diagnostic: after wiring, a value of 0 while avoidance is set signals a
        /// missing LoadNavMeshObstacles wiring (SD desync hazard) or a boundary-free NavMesh.
        /// </summary>
        public int DebugObstacleCount => _avoidance?.DebugObstacleCount ?? 0;

        /// <summary>
        /// Constrains a position to the NavMesh (uses MoveAlongSurface internally).
        /// </summary>
        /// <remarks>
        /// <para>The mask is a required argument, following <c>MoveAlongSurface</c>: this method
        /// takes no agent, so it cannot resolve one, and defaulting it would silently constrain a
        /// narrow-masked agent against ground it may not stand on. Pass
        /// <see cref="NavAgentComponent.WalkAreaMaskOverride"/> (or
        /// <see cref="DEFAULT_AREA_MASK"/> when it is zero) to agree with what that agent's walk
        /// will accept.</para>
        ///
        /// <para><b>It pairs with the WALK, not with the path</b>, and that is a correction: this
        /// remark used to promise agreement with <c>FindPath</c>, which held only while one mask
        /// served both. It no longer does — an agent may plan through a building on purpose and be
        /// refused entry to it, so agreeing with the path would mean constraining a position INTO
        /// a footprint the agent cannot occupy. Constrain answers "where may this agent stand",
        /// which is the walk's question.</para>
        /// </remarks>
        public FPVector3 ConstrainToNavMesh(FPVector3 newPos, FPVector3 oldPos, int currentTri,
            int areaMask)
        {
            var (resultPos, _) = _query.MoveAlongSurface(oldPos, newPos, currentTri,
                areaMask, MultiFloorYThreshold);
            return resultPos;
        }

        /// <summary>
        /// Updates all agents by one tick based on NavAgentComponent data.
        /// </summary>
        /// <remarks>
        /// The caller owns the array: who is in it, in what order, and how many calls a tick takes
        /// are all game-side decisions. Three consequences worth knowing before splitting it up:
        /// <list type="bullet">
        /// <item><description>
        /// <b>Past <see cref="MAX_AGENTS"/> the position-correction pass silently drops the tail.</b>
        /// Every agent is still steered and moved; only the pass that pushes residual overlaps apart
        /// takes the first <see cref="MAX_AGENTS"/>. Nothing throws and nothing logs —
        /// <see cref="DebugCollisionResolveTruncatedCount"/> is the only signal.
        /// </description></item>
        /// <item><description>
        /// <b>That pass is effective only for position-authoritative consumers.</b> It writes
        /// <c>NavAgentComponent.Position</c> and nothing else, so an integration that drives the
        /// character from <c>Velocity</c> and re-syncs Position from an external transform each tick
        /// never observes it — there the counter can be non-zero with no behavioural change at all.
        /// </description></item>
        /// <item><description>
        /// <b>Diagnostic counters accumulate over the instance lifetime and are not rollback-aware.</b>
        /// A resimulated tick is counted again. They are diagnostic fields, outside the state hash,
        /// wire and replay.
        /// </description></item>
        /// </list>
        /// </remarks>
        public unsafe void Update(ref Frame frame, EntityRef[] entities, int entityCount, int currentTick, FP64 dt)
        {
            for (int i = 0; i < entityCount; i++)
            {
                ref var nav = ref frame.Get<NavAgentComponent>(entities[i]);
                ProcessPathRequest(ref nav, currentTick);
                ProcessSteering(ref nav);
            }

            if (_avoidance != null)
            {
                // Graph-local obstacle query is usable only when the CSR was built from THIS navmesh
                // and no external LoadObstacles re-load has desynced it, verified by the generation guard.
                bool graphAvailable = _triSegStart != null
                    && _obstacleLoadGenerationCache == _avoidance.ObstacleLoadGeneration
                    && _avoidance.DebugObstacleCount > 0;
                FP64 timeHorizonObst = _avoidance.TimeHorizonObst; // read the same field the adopt gate uses (F2)

                for (int i = 0; i < entityCount; i++)
                {
                    ref var nav = ref frame.Get<NavAgentComponent>(entities[i]);
                    if (nav.Status != (byte)FPNavAgentStatus.Moving)
                        continue;

                    int seedTri = nav.CurrentTriangleIndex;
                    if (graphAvailable && seedTri >= 0)
                    {
                        FP64 obstRange = timeHorizonObst * nav.Speed + nav.Radius;
                        int candCount = CollectGraphLocalObstacles(seedTri, nav.Position.ToXZ(), obstRange);
                        nav.DesiredVelocity = _avoidance.ComputeNewVelocity(
                            i, ref frame, entities, entityCount, dt, _candidateSegs, candCount, true);
                    }
                    else
                    {
                        // Fallback: no navmesh CSR / unlocalized agent (seed -1) -> brute-force scan.
                        nav.DesiredVelocity = _avoidance.ComputeNewVelocity(
                            i, ref frame, entities, entityCount, dt);
                    }
                }
            }

            for (int i = 0; i < entityCount; i++)
            {
                ref var nav = ref frame.Get<NavAgentComponent>(entities[i]);
                ProcessMovement(ref nav, dt, currentTick);
            }

            // Pass 4: position-based collision resolution.
            // Complements velocity-space ORCA/LP3 by pushing residual overlaps apart in position
            // space (incl. Arrived/stopped agents that ORCA does not steer). Gated on avoidance so
            // non-avoidance configs stay bit-identical to before.
            if (_avoidance != null)
                ResolveCollisions(ref frame, entities, entityCount);
        }

        /// <summary>
        /// Graph-local obstacle candidate collection. BFS the navmesh adjacency from the
        /// agent's seed triangle to within obstRange, gathering the boundary segments of every
        /// visited triangle into <see cref="_candidateSegs"/> (unsorted). Only the topological
        /// selection lives here; the exact point-segment gate, nearest-first sort, and line
        /// generation stay in FPNavAvoidance. Returns the candidate count. GC-0.
        ///
        /// Expansion gates (all node-local, so the visited set is pop-order independent):
        ///  - not blocked (a blocked neighbor is a wall; don't reach walls behind it),
        ///  - step-delta floor: |centerY(nb) - centerY(cur)| ≤ MultiFloorYThreshold (per-edge, lets
        ///    ramps through; NOT the seed-band that pathfinding movement uses),
        ///  - climb cap: |centerY(nb) - centerY(seed)| ≤ MaxClimbWithinHorizon (∞ default),
        ///  - XZ padding: the triangle's nearest point to the agent ≤ obstRange (padded, so a
        ///    triangle whose center is out of range but whose wall edge is in range is still
        ///    visited — the adopt gate then exactly re-tests each segment).
        /// </summary>
        private int CollectGraphLocalObstacles(int seedTri, FPVector2 agentXZ, FP64 obstRange)
        {
            var tris = _navMesh.Triangles;
            if (seedTri < 0 || seedTri >= tris.Length || _triSegStart == null || _bfsStamp == null)
                return 0;

            FP64 obstRangeSqr = obstRange * obstRange;

            // Effective climb cap: combine the manual cap with the sound bound derived from the
            // mesh's recorded bake slope — within the horizon the agent walks at most obstRange,
            // gaining at most obstRange*sin(maxSlope) height on a mesh baked with that slope limit,
            // so higher walls are unreachable and cutting them only removes phantoms. Recorded
            // slope 0 = unknown → no auto bound (manual cap / ∞ as before). min() keeps a manual
            // cap meaningful only when it tightens further. FP64.Sin is LUT-based (deterministic).
            FP64 climbCap = MaxClimbWithinHorizon;
            if (_navMesh.BakeMaxSlopeDeg > FP64.Zero)
            {
                FP64 capAuto = obstRange * FP64.Sin(_navMesh.BakeMaxSlopeDeg * FP64.Deg2Rad);
                if (capAuto < climbCap)
                    climbCap = capAuto;
            }

            // Generation stamp: bump per query, wrap resets (no per-call clear).
            _bfsGeneration++;
            if (_bfsGeneration == int.MaxValue)
            {
                System.Array.Clear(_bfsStamp, 0, _bfsStamp.Length);
                _bfsGeneration = 1;
            }

            int candCount = 0;
            int head = 0, tail = 0;
            _bfsFrontier[tail++] = seedTri;
            _bfsStamp[seedTri] = _bfsGeneration;
            FP64 seedCenterY = tris[seedTri].centerY;

            while (head < tail)
            {
                int t = _bfsFrontier[head++];

                // Collect this triangle's boundary segments (CSR[t]) — 1:1 partition, no dedup.
                int segEnd = _triSegStart[t + 1];
                for (int k = _triSegStart[t]; k < segEnd; k++)
                {
                    if (candCount < _candidateSegs.Length)
                        _candidateSegs[candCount++] = _triSegList[k];
                }

                FP64 curCenterY = tris[t].centerY;
                for (int e = 0; e < 3; e++)
                {
                    int nb = tris[t].GetNeighbor(e);
                    if (nb < 0)
                        continue; // boundary edge: adopted above via CSR, not an expansion target
                    if (_bfsStamp[nb] == _bfsGeneration)
                        continue;
                    if (tris[nb].isBlocked)
                        continue;

                    FP64 nbCenterY = tris[nb].centerY;
                    FP64 stepDelta = nbCenterY - curCenterY;
                    if (stepDelta < FP64.Zero) stepDelta = -stepDelta;
                    if (stepDelta > MultiFloorYThreshold)
                        continue;

                    FP64 climb = nbCenterY - seedCenterY;
                    if (climb < FP64.Zero) climb = -climb;
                    if (climb > climbCap)
                        continue;

                    if (TriangleNearestDistSqr(agentXZ, nb, tris) > obstRangeSqr)
                        continue;

                    _bfsStamp[nb] = _bfsGeneration;
                    if (tail < _bfsFrontier.Length)
                        _bfsFrontier[tail++] = nb;
                    else
                        _bfsFrontierOverflowCount++;
                }
            }
            return candCount;
        }

        /// <summary>
        /// Squared XZ distance from a point to a triangle (0 if inside; else nearest edge). Used as
        /// the padded BFS expansion gate — never under-approximates for a point outside the
        /// triangle, so an in-range wall edge is never skipped.
        /// </summary>
        private FP64 TriangleNearestDistSqr(FPVector2 p, int triIdx, ReadOnlySpan<FPNavMeshTriangle> tris)
        {
            ref readonly FPNavMeshTriangle tri = ref tris[triIdx];
            FPVector2 a = _navMesh.Vertices[tri.v0].ToXZ();
            FPVector2 b = _navMesh.Vertices[tri.v1].ToXZ();
            FPVector2 c = _navMesh.Vertices[tri.v2].ToXZ();
            if (FPNavMeshQuery.PointInTriangle2D(p, a, b, c))
                return FP64.Zero;
            FP64 d = FPVector2.SqrDistance(p, FPNavMeshQuery.ClosestPointOnSegment2D(p, a, b));
            FP64 d1 = FPVector2.SqrDistance(p, FPNavMeshQuery.ClosestPointOnSegment2D(p, b, c));
            if (d1 < d) d = d1;
            FP64 d2 = FPVector2.SqrDistance(p, FPNavMeshQuery.ClosestPointOnSegment2D(p, c, a));
            if (d2 < d) d = d2;
            return d;
        }

        /// <summary>
        /// How close counts as "there" for the point the agent is currently steering at.
        ///
        /// <para>For the destination this is <see cref="WaypointThreshold"/> — arriving means
        /// arriving. For a LEG it is the agent's turning radius, <c>v² / a</c>, because that is the
        /// tightest arc it can hold at its present speed: a portal it must turn at cannot be reached
        /// within a smaller radius than that no matter how the steering is written, and an agent
        /// asked to do it circles the point instead of passing through.</para>
        ///
        /// <para>Handing off early is not a loss of precision. The leg planner's job is to keep the
        /// SEARCH local, not to march the unit through a series of gates — and
        /// <see cref="ResolveLegTarget"/> uses the same radius to notice that the crossing it was
        /// handed is the one just made, so an early hand-off turns into a longer lookahead rather
        /// than a repeat.</para>
        /// </summary>
        private FP64 ReachRadius(in NavAgentComponent nav)
        {
            if (nav.PathTarget == nav.Destination)
                return WaypointThreshold;
            return HandoffRadius(nav.CurrentSpeed, nav.Acceleration, WaypointThreshold);
        }

        /// <summary>
        /// Where THIS leg aims. Without an abstract graph that is always the destination, which is
        /// what makes the off switch exact. With one, it is the portal onto the next node — the
        /// agent walks there, the corridor runs out, and the planner is asked again.
        ///
        /// <para>Falling back to the destination is always safe: it is the flat plan, and every
        /// reason to fall back (no graph, an unlocalised agent, endpoints in one node) is a case
        /// where legs would buy nothing anyway.</para>
        /// </summary>
        private FPVector3 ResolveLegTarget(ref NavAgentComponent nav)
        {
            if (_abstractGraph == null)
            {
                _lastLegPlanKind = LegPlanKind.NoGraph;
                return nav.Destination;
            }

            // D-5 (c) — the safety net. The graph was derived for ONE mask, so an agent planning
            // under a different one could be promised crossings that mask forbids. Such agents take
            // the flat path: always right, and no worse than they had before legs existed. How many
            // land here is the number that decides whether per-mask graphs are worth building (the
            // plan's D-5 (a)) — hence the counter rather than a silent branch.
            //
            // The test is the RESOLVED mask against the graph's, not "does an override exist". The
            // older form asked the latter, which is a proxy: an override that resolves to the same
            // mask the graph was derived under is safe, and refusing it cost the agent legs for no
            // reason. It also made an explicit SetAreaMask(nav, DEFAULT_AREA_MASK, ...) — the most
            // natural way to say "use the default" — silently opt that agent out of the graph while
            // changing nothing else about it.
            //
            // Equality is deliberately narrower than the exact safety condition, which is the
            // SUBSET one: the graph's walkable set is contained in the agent's iff every bit of the
            // graph's mask is in the agent's, i.e. (graphMask & ~planMask) == 0. Under that rule a
            // DEFAULT_AREA_MASK graph would also serve an ALL_AREAS agent — legally, but the graph
            // does not know the retained footprints that agent may cross, so it would route around
            // them. That is a detour only that agent pays and nothing reports, so the narrower
            // rule is chosen on optimality, not on correctness. Widening it to the subset test is
            // a behaviour change, not a cleanup.
            if (FPNavAgentSystem.ResolvePlanMask(nav) != _abstractGraph.AreaMask)
            {
                _maskFallbackCount++;
                _lastLegPlanKind = LegPlanKind.MaskMismatch;
                return nav.Destination;
            }

            int startTri = nav.CurrentTriangleIndex >= 0
                ? nav.CurrentTriangleIndex
                : _query.FindTriangle(nav.Position.ToXZ(), nav.Position.y);
            if (startTri < 0)
            {
                _lastLegPlanKind = LegPlanKind.StartOffMesh;
                return nav.Destination;
            }

            int goalTri = _query.FindTriangleForEndpoint(
                nav.Destination.ToXZ(), nav.Destination.y, ResolvePlanMask(nav));
            if (goalTri < 0)
            {
                _lastLegPlanKind = LegPlanKind.GoalOffMesh;
                return nav.Destination;
            }

            int startNode = _abstractGraph.NodeOf(startTri);
            int goalNode = _abstractGraph.NodeOf(goalTri);
            // Split, because the two halves mean opposite things. A node the graph cannot name is
            // the same trouble as an endpoint off the mesh — the search that follows is flat and
            // full-distance. Start and goal sharing a node is the opposite: the hierarchy has
            // nothing to shorten because there is nothing far to shorten. Folded together, the
            // second would have masked the first in every diagnostic that reads this.
            if (startNode < 0 || goalNode < 0)
            {
                _lastLegPlanKind = LegPlanKind.NodeUnknown;
                return nav.Destination;
            }
            if (startNode == goalNode)
            {
                _lastLegPlanKind = LegPlanKind.SameNode;   // last leg — nothing the graph can say
                return nav.Destination;
            }

            // The agent's position and the real destination go INTO the search, standing in for the
            // two node centres at the ends of the route. Without them the first portal is chosen
            // from a point the agent is not at — and an agent that just changed legs is on its
            // node's boundary, about as far from that centre as it gets.
            if (!_abstractGraph.TryFindFirstHop(
                    startNode, goalNode, nav.Position.ToXZ(), nav.Destination.ToXZ(),
                    out int edge, out int nextEdge, out int thirdEdge))
            {
                // No node route at all. The triangle search never runs, so its budget counter stays
                // clean — this is the only place the failure is visible.
                _abstractSearchFailedCount++;
                _lastLegPlanKind = LegPlanKind.NoAbstractRoute;
                return nav.Destination;
            }

            // Aim ACROSS the portal rather than at its middle. What to aim through it at is the
            // portal after this one when there is one, and the destination when this hop is the
            // last: the destination can point clean out of the next node, and steering at it would
            // hug a corner the route then has to come back around.
            //
            // But first, the crossing we have ALREADY made. A leg ends when the agent reaches its
            // portal, but the triangle it stands on at that moment is still on the near side of the
            // boundary — the portal is the shared edge, and arriving at it does not put the agent
            // past it. The abstract search then starts from the same node and hands back the same
            // crossing, whose aim point is where the agent is standing, so the leg ends again on the
            // next tick. The hand-off keeps the agent's velocity, so this does not stall: it circles.
            //
            // Measured before this guard: 66 leg advances over a route that changes node 17 times —
            // 49 of them repeats in a node the agent had not left. Aiming at what comes AFTER the
            // crossing is what carries it through.
            //
            // Two more shapes of the same repeat, both from pricing hops portal to portal (rule
            // revision 6). A plan made from a slightly different position can come back with the
            // NEIGHBOUR of the crossing just made — a portal a unit along the same boundary, whose
            // aim sits just outside the reach ball but within it of the SECOND reference point
            // below. And where the route passes a lattice corner, the first two crossings meet
            // there and the agent can be within reach of both: skipping only the first handed it a
            // leg that ended on the tick it was planned, every tick until it had physically crossed
            // (measured: 45 such legs on a 3×3-node field walked corner to corner).
            //
            // WHAT THIS ACTUALLY DOES, measured. Read as written the loop below is a ladder that
            // skips leading crossings until one falls outside the ball. It does not behave like
            // one: counting which rung returns, over six open-field runs (cell 4/8/16, reach 2.5 to
            // 100), the FIRST rung returned exactly once per journey — the opening plan — the
            // second returned NEVER, and every other plan skipped twice and returned the third aim
            // below without a reach test. So this is not "skip what was already crossed"; it is a
            // fixed three-hop lookahead, and the two reach tests are true by construction after the
            // first plan.
            //
            // The reason is the second reference point. lastLegEndXZ reads nav.PathTarget, which is
            // written only on a successful plan (below) and therefore holds the crossing this leg
            // is walking TOWARD, not where the last one ended. The plan that follows a hand-off asks
            // for that same crossing or its neighbour, which is inside that ball by construction —
            // so rungs one and two always skip. That is load-bearing rather than a defect: removing
            // the term doubles the number of legs on the same journey (the travel time is
            // unchanged, so it is pure planning cost) and introduces leg targets reached on the
            // plan tick. It is also harmless where it is theoretically wrong — the value is
            // FPVector3.Zero before the first plan, and over seventeen runs on a field centred on
            // the world origin, walking both toward it and away, that zero decided a skip zero
            // times: the term is only consulted on a plan, and the first aim is always a node ahead
            // of the agent, so "aim within reach of the origin while the agent is not" did not
            // occur. Renaming the field would say what it is; changing it would cost the above.
            //
            // The third aim is returned without a test, which is where the ladder ends today.
            // Measured, that is harmless at a sane reach radius (it was never inside the ball) and
            // makes no difference at a wide one (everything is inside the ball, so testing it would
            // return the same point anyway). What DOES go wrong at a wide radius is the hand-off
            // itself — see WarnLegReachRadiusOnce, which is the honest place for it.
            _lastLegPlanKind = LegPlanKind.Legs;
            FP64 reach = ReachRadius(nav);
            FPVector2 posXZ = nav.Position.ToXZ();
            FPVector2 lastLegEndXZ = nav.PathTarget.ToXZ();
            FPVector2 destXZ = nav.Destination.ToXZ();

            FPVector2 beyond = nextEdge >= 0 ? _abstractGraph.EdgePortal(nextEdge).ToXZ() : destXZ;
            FPVector3 aim = _abstractGraph.AimPointOn(edge, posXZ, beyond);
            if (!WithinReach(aim.ToXZ(), posXZ, lastLegEndXZ, reach))
                return aim;
            _legAdvanceRepeatCount++;
            if (nextEdge < 0)
                return nav.Destination;

            beyond = thirdEdge >= 0 ? _abstractGraph.EdgePortal(thirdEdge).ToXZ() : destXZ;
            aim = _abstractGraph.AimPointOn(nextEdge, posXZ, beyond);
            if (!WithinReach(aim.ToXZ(), posXZ, lastLegEndXZ, reach))
                return aim;
            _legAdvanceRepeatCount++;
            if (thirdEdge < 0)
                return nav.Destination;

            return _abstractGraph.AimPointOn(thirdEdge, posXZ, destXZ);
        }

        /// <summary>
        /// Whether a crossing's aim point counts as already made: within the hand-off radius of the
        /// agent, or of where its last leg ended (see the guard in <see cref="ResolveLegTarget"/>).
        /// </summary>
        private static bool WithinReach(FPVector2 aim, FPVector2 pos, FPVector2 lastLegEnd, FP64 reach)
            => FPVector2.Distance(pos, aim) < reach || FPVector2.Distance(lastLegEnd, aim) < reach;

        private unsafe void ProcessPathRequest(ref NavAgentComponent nav, int currentTick)
        {
            if (!nav.HasNavDestination || nav.HasPath)
                return;

            if (nav.Status != (byte)FPNavAgentStatus.PathPending)
                return;

            {
                FP64 distToTarget = FPVector2.Distance(nav.Position.ToXZ(), nav.Destination.ToXZ());
                FP64 yDistToTarget = FP64.Abs(nav.Position.y - nav.Destination.y);
                if (distToTarget < WaypointThreshold && yDistToTarget < MultiFloorYThreshold)
                {
                    nav.Status = (byte)FPNavAgentStatus.Arrived;
                    nav.Velocity = FPVector2.Zero;
                    nav.DesiredVelocity = FPVector2.Zero;
                    return;
                }
            }

            FP64 ticksSinceLast = FP64.FromInt(currentTick - nav.LastRepathTick);
            if (ticksSinceLast < nav.PathRepathCooldown && nav.LastRepathTick > 0)
                return;

            nav.LastRepathTick = currentTick;

            FPVector3 legTarget = ResolveLegTarget(ref nav);

            int exhaustedBefore = _pathfinder.DebugIterationExhaustedCount;
            FP64 partialMinProgress = PartialMinProgress(in nav, WaypointThreshold);
            bool found = _pathfinder.FindPath(nav.Position, legTarget, ResolvePlanMask(nav),
                partialMinProgress, out int[] corridor, out int corridorLength,
                out bool partial, out FPVector3 partialEnd);

            if (!found && legTarget != nav.Destination)
            {
                // The abstract route picked a portal the real search cannot reach. Two defects look
                // identical here (a cost scale the triangle search disagrees with, and a node that
                // claims a crossing the mesh does not have) and neither is a desync, so it is
                // counted rather than swallowed. Falling straight back to the destination keeps the
                // agent moving on the flat path it would have had without any of this.
                _legResolveFailedCount++;
                // The retry is flat and goes the whole way, so from here on this plan is one of the
                // ones the signal is about — a graph is installed and is not shortening this search.
                _lastLegPlanKind = LegPlanKind.AbstractRouteUnreachable;
                legTarget = nav.Destination;
                found = _pathfinder.FindPath(nav.Position, legTarget, ResolvePlanMask(nav),
                    partialMinProgress, out corridor, out corridorLength, out partial, out partialEnd);
            }

            NoteExhaustionWithoutLegs(exhaustedBefore, found, partial);

            if (found)
            {
                fixed (int* dst = nav.Corridor)
                {
                    _corridorCopyTruncatedCount += NavCorridorHelper.SetCorridor(
                        dst, ref nav.CorridorLength, _tuning.CorridorCap, corridor, corridorLength);
                }
                // A partial corridor ends short of what it was planned toward, so the agent aims
                // at ITS end: that is what makes the leg hand-off below treat arriving there as
                // "re-plan", not "arrived". Nothing else marks the corridor as partial — the frame
                // does not need to know, and a field for it would change the wire.
                nav.PathTarget = partial ? partialEnd : legTarget;
                nav.PathId = nav.PathRequestId;
                nav.PathIsValid = true;
                nav.HasPath = true;
                nav.Status = (byte)FPNavAgentStatus.Moving;
            }
            else
            {
                nav.Status = (byte)FPNavAgentStatus.PathFailed;
            }
        }

        private unsafe void ProcessSteering(ref NavAgentComponent nav)
        {
            if (nav.Status != (byte)FPNavAgentStatus.Moving)
                return;

            if (!nav.PathIsValid || nav.CorridorLength <= 0)
            {
                nav.Status = (byte)FPNavAgentStatus.Arrived;
                nav.Velocity = FPVector2.Zero;
                nav.DesiredVelocity = FPVector2.Zero;
                return;
            }

            // Clamped to the buffer, not to CorridorLength: a FullState apply can deliver a
            // component written by a peer built with a larger cap, and the length travels with it.
            // That disagreement is reported by the fingerprint, not fixed here — this only keeps it
            // from becoming an out-of-range crash on the receiving side.
            int corridorLen = nav.CorridorLength < _corridorBuffer.Length
                ? nav.CorridorLength : _corridorBuffer.Length;
            fixed (int* src = nav.Corridor)
            {
                for (int k = 0; k < corridorLen; k++)
                    _corridorBuffer[k] = src[k];
            }

            int cornerCount = _funnel.FindCorners(_corridorBuffer, corridorLen,
                nav.Position, nav.PathTarget, 4);
            FPVector3[] corners = _funnel.Corners;

            if (cornerCount == 0)
            {
                if (nav.CorridorLength > 1)
                {
                    nav.HasPath = false;
                    nav.Status = (byte)FPNavAgentStatus.PathPending;
                    return;
                }
                nav.Status = (byte)FPNavAgentStatus.Arrived;
                nav.Velocity = FPVector2.Zero;
                nav.DesiredVelocity = FPVector2.Zero;
                return;
            }

            FPVector3 nextCorner = corners[0];
            FPVector2 posXZ = nav.Position.ToXZ();
            FPVector2 cornerXZ = nextCorner.ToXZ();

            FPVector2 direction = (cornerXZ - posXZ).normalized;
            nav.DesiredVelocity = direction * nav.Speed;

            // Slowing down measures to the DESTINATION, not to this leg's end. With legs on, the
            // leg end is a portal the agent passes through at speed; braking for it would put a
            // stutter at every node boundary — silent, because nothing fails, the units just crawl.
            FPVector2 targetXZ = nav.Destination.ToXZ();
            FP64 distToTarget = FPVector2.Distance(posXZ, targetXZ);

            if (nav.Acceleration > FP64.Zero)
            {
                FP64 brakingRadius = nav.Speed * nav.Speed / (nav.Acceleration * FP64.FromInt(2));
                if (distToTarget < brakingRadius)
                {
                    nav.DesiredVelocity = nav.DesiredVelocity * distToTarget / brakingRadius;
                }
            }

            if (nav.StoppingDistance > FP64.Zero)
            {
                FP64 yDist = FP64.Abs(nav.Position.y - nav.Destination.y);
                if (yDist < MultiFloorYThreshold)
                {
                    if (distToTarget < nav.StoppingDistance)
                    {
                        nav.DesiredVelocity = nav.DesiredVelocity * distToTarget / nav.StoppingDistance;
                    }
                }
            }
        }

        /// <summary>
        /// Says once, per system, that an agent's reach radius is wider than a node — the condition
        /// behind <see cref="DebugLegEndedOnPlanTickCount"/>. Latched because the state it reports
        /// is a TUNING mismatch that holds for the whole run: repeating it every tick would bury
        /// the log of the match it is trying to explain.
        ///
        /// <para>It cannot be checked at install time. The radius is <c>v² / a</c> off the agent's
        /// CURRENT speed, and the ceiling <c>Speed² / Acceleration</c> lives per entity in the
        /// frame — <see cref="SetAbstractGraph"/> knows the cell size but not who will walk on it.
        /// So the check rides the one place both numbers are in hand.</para>
        ///
        /// <para>Diagnostic only: the latch is instance state, never frame state, and nothing here
        /// changes what any agent does.</para>
        /// </summary>
        private void WarnLegReachRadiusOnce(in NavAgentComponent nav)
        {
            if (_legReachRadiusWarned || _abstractGraph == null || _logger == null)
                return;

            FP64 radius = ReachRadius(in nav);
            FP64 cell = _abstractGraph.CellSize;
            if (radius <= cell)
                return; // a portal that was genuinely a step away; the radius is not the problem

            _legReachRadiusWarned = true;
            _logger.KWarning(
                $"[FPNavAgentSystem] planning in legs: an agent's reach radius " +
                $"{radius.ToDouble():F2} is wider than a node ({_abstractGraph.CellSize.ToDouble():F2}), " +
                $"so its leg targets are reached on the tick they are planned and it re-plans every " +
                $"tick instead of once per leg. The radius is speed² / acceleration " +
                $"({nav.CurrentSpeed.ToDouble():F2}² / {nav.Acceleration.ToDouble():F2}) — lower the " +
                $"speed, raise the acceleration, or derive the graph with a larger cell. " +
                $"DebugLegEndedOnPlanTickCount counts how often this happens; said once per system.");
        }

        private unsafe void ProcessMovement(ref NavAgentComponent nav, FP64 dt, int currentTick)
        {
            if (nav.Status != (byte)FPNavAgentStatus.Moving)
            {
                nav.Velocity = FPVector2.Zero;
                nav.CurrentSpeed = FP64.Zero;
                return;
            }

            FPVector2 diff = nav.DesiredVelocity - nav.Velocity;
            FP64 maxAccelStep = nav.Acceleration * dt;
            FP64 diffSqrMag = diff.sqrMagnitude;

            if (diffSqrMag > maxAccelStep * maxAccelStep)
            {
                diff = diff.normalized * maxAccelStep;
            }

            nav.Velocity = nav.Velocity + diff;

            FP64 velSqrMag = nav.Velocity.sqrMagnitude;
            if (velSqrMag > nav.Speed * nav.Speed)
            {
                nav.Velocity = nav.Velocity.normalized * nav.Speed;
            }

            nav.CurrentSpeed = nav.Velocity.magnitude;

            FPVector3 displacement = new FPVector3(
                nav.Velocity.x * dt,
                FP64.Zero,
                nav.Velocity.y * dt);
            FPVector3 newPos = nav.Position + displacement;

            int walkMask = ResolveWalkMask(nav);
            var (resultPos, resultTri) = _query.MoveAlongSurfaceWithVisited(
                nav.Position, newPos, nav.CurrentTriangleIndex, walkMask,
                MultiFloorYThreshold, _visitedBuffer, out int visitedCount);

            // Stop-on-contact: this agent's corridor leads into ground its walk mask refuses, and
            // it has now run into it. Two conditions, and BOTH are needed.
            //
            //   (1) the walk did not move. A refused neighbour is a wall to MoveAlongSurface, so a
            //       head-on approach is clipped to the edge and comes back where it started. The
            //       walk cannot tell us WHY it refused — every reason falls through the same wall
            //       path, and that is deliberate (the query re-tests the mask only to log it, so
            //       the production path pays nothing) — which is why the reason is derived here.
            //   (2) the next corridor triangle is refused by this mask. This is what separates a
            //       building from a real wall: against a real wall the corridor's next triangle is
            //       still enterable, so the agent keeps sliding, which is the behaviour that must
            //       survive.
            //
            // Condition (1) alone would fire on any wall. Condition (2) alone would fire on the
            // FIRST tick of every such path — an agent crossing its current triangle is "not
            // advancing" for as many ticks as the crossing takes, so it would stop before it ever
            // touched anything.
            //
            // And (1) needs the ATTEMPT, not just the outcome: `resultPos == nav.Position` is also
            // true of an agent that requested no displacement at all, so a unit held still by a
            // crowd one triangle short of a footprint used to report Blocked without having
            // touched anything. Comparing against newPos costs nothing — it is already computed —
            // and it is what makes this state mean "stopped where it touched" rather than "is not
            // moving". The cost is that Blocked now arrives on the tick the agent pushes against
            // the edge instead of the tick its velocity reaches zero, which is the correct
            // definition of contact.
            //
            // Left to itself this stall is silent: the agent keeps Status = Moving and a valid
            // path, and the off-corridor repath never fires because standing still inside the
            // corridor's current triangle counts as being ON the corridor and resets that counter.
            // Naming it is the whole point — the game decides what to do about a Blocked agent.
            //
            // Which triangle is "the next one" is decided by THIS agent's index in its corridor,
            // found once here and used twice — by the verdict below and by the advance further
            // down, which used to search for it a second time. Off the corridor the search comes
            // back empty (idx < 0) and both readers take their off-corridor branch: the verdict is
            // SKIPPED, because the corridor head is not an off-corridor agent's next step and
            // latching the terminal Blocked on it would pre-empt the off-corridor repath that is
            // the actual answer for that agent.
            int corridorIdx = -1;
            if (nav.PathIsValid && nav.CorridorLength > 0)
            {
                fixed (int* p = nav.Corridor)
                {
                    for (int i = 0; i < nav.CorridorLength; i++)
                    {
                        if (p[i] == resultTri)
                        {
                            corridorIdx = i;
                            break;
                        }
                    }
                }
            }

            if (corridorIdx >= 0 && corridorIdx + 1 < nav.CorridorLength
                && newPos != nav.Position && resultPos == nav.Position)
            {
                int nextTri;
                fixed (int* p = nav.Corridor)
                    nextTri = p[corridorIdx + 1];

                if (nextTri >= 0 && nextTri < _navMesh.Triangles.Length
                    && (walkMask & _navMesh.Triangles[nextTri].areaMask) == 0)
                {
                    nav.Velocity = FPVector2.Zero;
                    nav.DesiredVelocity = FPVector2.Zero;
                    nav.CurrentSpeed = FP64.Zero;
                    nav.Status = (byte)FPNavAgentStatus.Blocked;
                    return;
                }
            }

            nav.CurrentTriangleIndex = resultTri;
            nav.Position = resultPos;

            if (nav.PathIsValid && nav.CorridorLength > 0)
            {
                int advanceIdx = corridorIdx; // found once, above the contact verdict
                fixed (int* p = nav.Corridor)
                {
                    if (advanceIdx > 0)
                    {
                        int newLen = nav.CorridorLength - advanceIdx;
                        for (int i = 0; i < newLen; i++)
                            p[i] = p[i + advanceIdx];
                        nav.CorridorLength = newLen;
                        nav.OffCorridorTicks = 0;

                        if (newLen == 1)
                        {
                            FP64 currentSpeed = nav.Velocity.magnitude;
                            if (currentSpeed > FP64.Zero)
                            {
                                FPVector2 desiredDir = nav.DesiredVelocity.normalized;
                                if (desiredDir.sqrMagnitude > FP64.Zero)
                                {
                                    nav.Velocity = desiredDir * currentSpeed;
                                }
                            }
                        }
                    }
                    else if (advanceIdx == 0)
                    {
                        nav.OffCorridorTicks = 0;
                    }
                    else
                    {
                        nav.OffCorridorTicks++;
                        if (nav.OffCorridorTicks >= OffCorridorRepathThreshold)
                        {
                            nav.HasPath = false;
                            nav.Status = (byte)FPNavAgentStatus.PathPending;
                            nav.OffCorridorTicks = 0;
                            return;
                        }
                    }
                }
            }

            {
                FP64 distToTarget = FPVector2.Distance(
                    nav.Position.ToXZ(), nav.PathTarget.ToXZ());
                FP64 yDistToTarget = FP64.Abs(nav.Position.y - nav.PathTarget.y);
                // A leg hands off from FURTHER OUT than the destination does, and the distance is
                // physical rather than chosen: an agent moving at v with acceleration a cannot hold
                // an arc tighter than v²/a, so asking it to pass within WaypointThreshold of a
                // portal it has to turn at is asking for something it cannot do. It orbits instead —
                // measured on a switchback fixture at 346 ticks spent inside 3 units of a leg target
                // against 36 for a straight pass, and that circling is what a user sees at a node.
                //
                // The destination keeps the tight threshold: arriving there means arriving.
                if (distToTarget < ReachRadius(nav) && yDistToTarget < MultiFloorYThreshold)
                {
                    if (nav.PathTarget == nav.Destination)
                    {
                        nav.Status = (byte)FPNavAgentStatus.Arrived;
                        nav.Velocity = FPVector2.Zero;
                        nav.DesiredVelocity = FPVector2.Zero;
                    }
                    else
                    {
                        // A leg ended, not the journey. Hand the agent back to the planner without
                        // touching its velocity — stopping here is exactly the stutter the braking
                        // change above avoids.
                        //
                        // LastRepathTick goes back to the "never planned" sentinel on purpose: the
                        // repath cooldown exists to stop an agent replanning its DESTINATION every
                        // tick, and applying it to a leg hand-off would park the unit at every
                        // portal for the length of the cooldown.
                        //
                        // Without a graph, a PathTarget that is not the destination can only be the
                        // end of a partial corridor, so that case is counted as what it is. With a
                        // graph the hand-off cannot tell the two apart (see
                        // DebugPartialHandoffCount) and counts as a leg advance.
                        if (_abstractGraph == null)
                        {
                            _partialHandoffCount++;
                            // Same test as the leg branch below, same meaning: planned in pass 1
                            // and handed off in pass 3 of this Update, so nothing was walked.
                            if (nav.LastRepathTick == currentTick)
                                _partialEndedOnPlanTickCount++;
                        }
                        else
                        {
                            _legAdvanceCount++;

                            // Did this leg travel at all? Planning runs in pass 1 and this hand-off
                            // in pass 3 of the same Update, so LastRepathTick still holds the tick
                            // the leg was planned on. Equal to the current tick means the agent was
                            // already within ReachRadius when the target was chosen — nothing moved,
                            // and the cleared cooldown below sends it straight back to a full A*.
                            if (nav.LastRepathTick == currentTick)
                            {
                                _legEndedOnPlanTickCount++;
                                WarnLegReachRadiusOnce(in nav);
                            }
                        }

                        nav.HasPath = false;
                        nav.PathIsValid = false;
                        nav.CorridorLength = 0;
                        nav.OffCorridorTicks = 0;
                        nav.LastRepathTick = 0;
                        nav.Status = (byte)FPNavAgentStatus.PathPending;
                    }
                }
            }
        }

        /// <summary>
        /// Position-based collision resolution (RVO2/DetourCrowd style). Iteratively pushes
        /// overlapping agent pairs apart in position space, compute-then-apply per iteration for
        /// order-independence. Pushes all states (incl. Arrived) but never writes Velocity, and
        /// does not re-trigger Arrived agents to Moving. Mesh-clamped per agent.
        /// </summary>
        /// <remarks>
        /// Consumer model: this pass writes only <c>NavAgentComponent.Position</c>. It is effective
        /// only for POSITION-AUTHORITATIVE consumers that treat Position as the entity's canonical
        /// location. VELOCITY-AUTHORITATIVE integrations (which drive the character from
        /// <c>Velocity</c> and re-sync Position from an external transform each tick — e.g. the
        /// Brawler sample) do not observe this correction; there the velocity-space fixes
        /// (coincident guard, LinearProgram3) apply but overlapping STOPPED agents are separated at
        /// the transform/physics layer instead (out of nav scope).
        /// </remarks>
        private void ResolveCollisions(ref Frame frame, EntityRef[] entities, int entityCount)
        {
            int maxAgents = _tuning.MaxAgents;
            int n = entityCount < maxAgents ? entityCount : maxAgents;
            if (entityCount > maxAgents)
                _collisionResolveTruncatedCount += entityCount - maxAgents;

            for (int iter = 0; iter < _tuning.CollisionResolveIterations; iter++)
            {
                // Sub-pass 1: accumulate displacement from current positions (barrier → order-independent)
                for (int a = 0; a < n; a++)
                {
                    _disp[a] = FPVector2.Zero;
                    _dispWeight[a] = 0;
                }

                for (int a = 0; a < n; a++)
                {
                    ref readonly var na = ref frame.GetReadOnly<NavAgentComponent>(entities[a]);
                    FPVector2 pa = na.Position.ToXZ();

                    for (int b = a + 1; b < n; b++)
                    {
                        ref readonly var nb = ref frame.GetReadOnly<NavAgentComponent>(entities[b]);

                        FP64 combinedRadius = na.Radius + nb.Radius;
                        FPVector2 diff = pa - nb.Position.ToXZ();
                        FP64 distSqr = diff.sqrMagnitude;

                        if (distSqr >= combinedRadius * combinedRadius)
                            continue; // no overlap

                        if (distSqr <= POS_EPSILON)
                        {
                            // Coincident: idx fixed axis (matches the coincident guard — lower idx → -x).
                            // a < b → a pushed -x, b pushed +x. Small fixed nudge; radial resolve follows.
                            _disp[a] = _disp[a] + new FPVector2(-COINCIDENT_PEN, FP64.Zero);
                            _disp[b] = _disp[b] + new FPVector2(COINCIDENT_PEN, FP64.Zero);
                        }
                        else
                        {
                            FP64 dist = FP64.Sqrt(distSqr);
                            // scaled = (1/dist) * ((r₁+r₂ − dist) * 0.5) * factor ; diff (len=dist) → unit push
                            FP64 scaled = (FP64.One / dist)
                                * ((combinedRadius - dist) * FP64.Half)
                                * COLLISION_RESOLVE_FACTOR;
                            FPVector2 push = diff * scaled;
                            _disp[a] = _disp[a] + push; // a away from b
                            _disp[b] = _disp[b] - push; // b away from a
                        }

                        _dispWeight[a]++;
                        _dispWeight[b]++;
                    }
                }

                // Sub-pass 2: apply averaged displacement (mesh-clamped). Position only — Velocity untouched.
                for (int a = 0; a < n; a++)
                {
                    if (_dispWeight[a] == 0)
                        continue; // no overlap → no-op (bit-identical when no collisions)

                    ref var na = ref frame.Get<NavAgentComponent>(entities[a]);
                    FP64 iw = FP64.One / FP64.FromInt(_dispWeight[a]);
                    FPVector2 d = _disp[a] * iw;

                    FPVector3 newPos = na.Position + new FPVector3(d.x, FP64.Zero, d.y);
                    // This agent's own walk mask, NOT FPNavMeshAreas.ALL_AREAS: a crowd may press a
                    // unit against ground it is not allowed to enter, never into it. Unfiltered,
                    // this pass is the one way an agent ends up standing inside a retained
                    // building's footprint — and if it is Blocked when that happens it parks there,
                    // because Blocked is terminal and nothing recomputes it.
                    //
                    // Passing the mask does NOT trap an agent that is already inside forbidden
                    // ground: the walk's expansion rule exempts refused neighbours while the
                    // triangle it expands FROM is refused too, so such an agent can be pushed
                    // across its forbidden component and out. That asymmetry — the mask blocks
                    // entering, not leaving — lives in MoveAlongSurface and must not be
                    // re-implemented here; ALL_AREAS would additionally allow what that rule
                    // deliberately forbids, carrying an agent between two SEPARATE forbidden
                    // regions.
                    var (resultPos, resultTri) = _query.MoveAlongSurface(
                        na.Position, newPos, na.CurrentTriangleIndex, ResolveWalkMask(na),
                        MultiFloorYThreshold);

                    na.CurrentTriangleIndex = resultTri;
                    na.Position = resultPos;
                }
            }
        }
    }
}
