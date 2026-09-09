using System;
using System.Collections.Generic;

using xpTURN.Klotho.Deterministic.Math;
using xpTURN.Klotho.Logging;

namespace xpTURN.Klotho.Deterministic.Navigation
{
    /// <summary>
    /// How a node's <c>costMultiplier</c> folds into one abstract edge cost. The abstract search
    /// plans over nodes while the real A* charges per triangle, so the fold decides which way the
    /// two disagree when a node's triangles do not share a cost.
    /// </summary>
    public enum FPNavAbstractCostFold : byte
    {
        /// <summary>Cheapest triangle in the node. Optimistic, and therefore admissible — the
        /// abstract estimate never exceeds what the real path charges.
        /// <para>That holds only while every <c>costMultiplier</c> is at least 1. Nothing bounds
        /// them: the bake carries whatever was authored, and ground cheaper than plain distance
        /// (a road at 0.5, say) makes this fold an OVER-estimate of a path that runs over it, so
        /// the first hop the abstract search picks need not be the best one. The derivation counts
        /// such triangles and warns once — see <c>DebugCheapTriangles</c>.</para></summary>
        Min = 0,

        /// <summary>Mean over the node's triangles. Closer to what the leg actually costs, at the
        /// price of admissibility.</summary>
        Mean = 1,
    }

    /// <summary>
    /// A coarse graph over the navmesh: the walkable surface cut into nodes, with an edge wherever
    /// two nodes touch. Planning hops nodes first and solves only the current leg with the real A*,
    /// which is what keeps a search local.
    ///
    /// <para><b>Derived, never baked.</b> Everything here is a function of data the navmesh
    /// fingerprint already folds, so two peers with the same fingerprint build the same graph. The
    /// lattice is anchored at a FIXED world origin rather than at the mesh bounds precisely so that
    /// nothing outside the fingerprint can move a node id.</para>
    ///
    /// <para><b>Determinism rules this type lives under.</b> No hash containers, and no iteration
    /// whose order depends on anything but an index: node ids are assigned in triangle order,
    /// portals are emitted in (triangle, edge) order, and every fold accumulates in that same order.
    /// The analysis prototype this grew from used dictionaries for its statistics; none of that
    /// shape survives here, and <see cref="Checksum"/> is the net if it ever creeps back.</para>
    ///
    /// <para><b>An opaque handle.</b> Build one, hand it to
    /// <see cref="FPNavAgentSystem.SetAbstractGraph"/>, and read the four diagnostics below to tune
    /// it. The node and edge accessors stay internal: they are the representation rather than a
    /// format, and a public surface outlives every reason to change it.</para>
    /// </summary>
    public sealed class FPNavAbstractGraph
    {
        /// <summary>
        /// Bumped whenever leg planning changes where an agent walks WITHOUT changing the derived
        /// graph — a different rule for picking or aiming at a portal, say. It folds into
        /// <see cref="Checksum"/>, which folds into the navigation fingerprint, so two peers running
        /// different rules are caught at the Ready exchange instead of diverging quietly.
        ///
        /// <para><b>Why this is not <c>NAV_BEHAVIOUR_REVISION</c>.</b> That constant is folded for
        /// every navigating peer, so bumping it refuses every recorded replay — including the
        /// overwhelming majority that never installed a graph and whose behaviour did not change.
        /// This one rides the graph's own checksum, which contributes zero when there is no graph,
        /// so a bump refuses exactly the replays that used leg planning and no others.</para>
        ///
        /// <para>1 = the shipped rule (portal chosen and aimed at from node centres). 2 = endpoint
        /// insertion: the agent's position and the destination replace the two node centres they
        /// stand in for. 3 = the portal is aimed at as a SEGMENT rather than at its midpoint.
        /// 4 = a plan that asks for the crossing just finished is sent on to the next one.
        /// 5 = a leg hands off at the agent's turning radius rather than at the waypoint
        /// threshold, so it is not asked to reach a point it cannot arc into.
        /// 6 = every hop is priced from the portal a node was entered by to the portal it is left
        /// by (a per-node pair table — see <see cref="PairCost"/>), instead of centre to portal to
        /// centre; node centres and the per-edge costs that were built on them are gone, and the
        /// heuristic measures from the entry portal. With it, rule 4 skips EVERY leading crossing
        /// the agent is already within reach of (the neighbour of the one just made, and both sides
        /// of a lattice corner), aiming at the first one it is not.</para>
        /// </summary>
        private const long PLAN_RULE_REVISION = 6;

        private readonly IKLogger _logger;
        private readonly FP64 _cellSize;
        private readonly FPNavAbstractCostFold _costFold;
        private readonly int _areaMask;

        private FPNavMesh _navMesh;

        private int[] _nodeOfTriangle;        // triangle -> node, -1 when no node claims it
        private int[] _nodeTriangleCount;
        private FP64[] _nodeFold;             // costMultiplier folded per node, see FPNavAbstractCostFold
        private int _cheapTriangles;          // of them, how many priced below 1 — see DebugCheapTriangles
        private bool[] _nodeHasWall;          // some triangle edge in the node has nothing walkable across it

        private int[] _edgeStart;             // CSR over node ids
        private int[] _edgeTarget;
        private int[] _edgeTri;               // the source-side triangle the portal is an edge of
        private int[] _edgeTriEdge;           // and which of its three edges
        private int[] _edgeOfTriEdge;         // the inverse: (triangle, edge) -> out-edge slot, -1 where none
        private int[] _edgeReverse;           // the same portal seen from the target node, -1 if unmatched
        private FPVector3[] _edgePortal;      // midpoint of the shared edge — the cost model's point
        private FPVector3[] _edgePortalA;     // and the segment itself, which is what a leg aims along
        private FPVector3[] _edgePortalB;

        // Per node, an outdeg x outdeg table: what walking the node from one of its portals to
        // another costs (see PairCost). Rows and columns are the node's own out-edges in CSR order;
        // the portal a node was ENTERED by is mapped to a row through _edgeReverse.
        private int[] _pairStart;             // node -> first entry, nodeCount + 1
        private FP64[] _pairCost;
        private int _pairEntryCount;
        private int _reverseUnmatched;
        private int _adjacencyAsymmetric;     // one-directional triangle adjacency met by the in-node search

        // Pair-table build scratch: triangle-sized, reused across portals by generation stamp.
        private FP64[] _pairDist;
        private int[] _pairEntryEdge;         // which of a triangle's three edges the search entered it by
        private int[] _pairStamp;
        private int _pairGeneration;
        private FPNavMeshBinaryHeap _pairHeap;
        // The edge-midpoint model prices a triangle by the distance between two of its edge
        // midpoints — three constants per triangle, so the search looks them up instead of taking
        // a square root per relaxation. Filled the first time a triangle is relaxed, per
        // derivation (stamped): a mesh whose nodes are convex never runs the search and fills
        // nothing. Grown with the other scratch in EnsurePairCapacity, never on its own.
        private FP64[] _triMidDist;           // 3 per triangle, pair {a, b} at 3 - a - b
        private int[] _triMidStamp;
        private int _triMidGeneration;
        // Diagnostics of the LAST derivation (see the Debug* accessors).
        private long _debugMidDistFills, _debugPairRepushes, _debugWalkSteps, _debugWalkDivisions;
        private long _debugWalkExhausted;     // walks that ran out of steps — see DebugWalkExhausted

        // Triangles of the mesh the last derivation ran over — what a donor is checked against
        // (see Rebind): a mesh instance that was recycled and rewritten since then is not the one
        // the rows were computed on.
        private int _triCount;
        // Incremental re-derivation (Rebind with a donor) diagnostics of the LAST derivation; all
        // zero and DonorUsed false on a fresh one.
        private bool _debugRebindDonorUsed;
        private string _debugRebindDonorIgnored;   // why not, when a donor was given and not used
        private int _debugRebindNodesReused, _debugRebindNodesRebuilt, _debugRebindWallNearFlips,
                    _debugRebindMatchFailures, _debugRebindPairsRecomputed;

        private int[] _scratch;               // flood-fill stack, doubles as the BFS queue
        private int[] _depth;
        private int[] _visitStamp;
        private int _visitGeneration;
        private int[] _cellCol;
        private int[] _cellRow;
        private int[] _cursor;
        private bool[] _nodeSeen;             // component walk, see CountNodeComponents

        /// <summary>Nodes the last derivation produced.</summary>
        public int NodeCount { get; private set; }

        /// <summary>Directed edges the last derivation produced (a portal contributes both ways).</summary>
        public int EdgeCount { get; private set; }

        /// <summary>
        /// The longest shortest-path inside any single node, <b>in HOPS</b> — measured, not assumed.
        /// A single-triangle node is 0. The path it names therefore spans
        /// <c>MaxNodeDiameter + 1</c> TRIANGLES; do not compare this against a triangle count
        /// directly, and see <see cref="MaxLegCorridorTriangles"/> for the number that may be.
        /// Triangle COUNT is the safe bound but far too pessimistic (a compact node's internal path
        /// is nearer its square root), which is why the derivation measures the diameter instead of
        /// clamping the count.
        /// <para>Double sweep — BFS to the farthest triangle, then BFS from there. Exact on a tree
        /// and a lower bound in general, so read it as "at least this long".</para>
        /// </summary>
        public int MaxNodeDiameter { get; private set; }

        /// <summary>
        /// The longest corridor a leg through this graph can ask for, <b>in TRIANGLES</b> — the
        /// number to compare against <see cref="FPNavTuning.CorridorCap"/>, which counts triangles
        /// too. Two more than <see cref="MaxNodeDiameter"/>, and both are load-bearing:
        ///
        /// <list type="bullet">
        /// <item><b>+1 for the units.</b> A path of <c>d</c> hops visits <c>d + 1</c> triangles.</item>
        /// <item><b>+1 for the far side of the portal.</b> A leg does not aim inside its node — it
        /// aims at a point ON the shared edge (<c>FPNavAgentSystem.AimPointOn</c>), and
        /// <see cref="FPNavMeshQuery.FindTriangleForEndpoint"/> may resolve a point on an edge to
        /// the triangle across it. That one belongs to the NEXT node and still lands in the
        /// corridor.</item>
        /// </list>
        ///
        /// <para><b>Still not a proof.</b> <see cref="MaxNodeDiameter"/> is a lower bound off a
        /// tree, so this is a lower bound too — it catches a cell size that is clearly too large,
        /// and <c>FPNavMeshPathfinder.DebugCorridorTruncatedCount</c> stays the runtime net for what
        /// gets through. What it removes is the off-by-two that came from comparing hops against
        /// triangles, which put the guard's own boundary case on the wrong side of the cap.</para>
        ///
        /// <para><b>And it is one node's worth, while a leg spans three.</b> A plan skips the
        /// crossings it is already within reach of, and measured on open ground it skips two on
        /// essentially every plan (<c>FPNavAgentSystem.ResolveLegTarget</c> says why), so the
        /// corridor a leg asks for runs current node → next → the one after. This number therefore
        /// under-counts by roughly a factor of three in the common case. It is left as it is because
        /// the overrun does not reach the cap in practice: measured over nine open-field
        /// configurations — cell 4, 8 and 16, reach radius 2.5 to 100 — the corridor was clamped
        /// <b>zero</b> times. <c>FPNavAgentSystem.DebugCorridorCopyTruncatedCount</c> is the counter
        /// that would say otherwise, and raising this bound to match would make the install ladder
        /// pick a much smaller cell for every game to cover a case none of them reached.</para>
        /// </summary>
        public int MaxLegCorridorTriangles => MaxNodeDiameter + 2;

        /// <summary>
        /// Fold of the derived graph's shape. Peers that derive differently — a nondeterministic
        /// container, a platform-dependent order — differ here even when the mesh fingerprint
        /// agrees, which is the one failure a same-process "derive twice" check cannot see.
        /// </summary>
        public ulong Checksum { get; private set; }

        /// <summary>
        /// Connected components of the NODE graph. <b>1 means every node can reach every other</b>;
        /// more means the walkable surface is split, and a route between two of those pieces does
        /// not exist for the abstract search to find.
        ///
        /// <para>This is the answer <see cref="FPNavAgentSystem.DebugAbstractSearchFailedCount"/>
        /// cannot give. That counter says a node route was not found, which is a symptom shared by
        /// "the map really is split" and "the derivation is wrong" — and the triangle search never
        /// runs to leave any other trace. Reading this alongside it separates them without exposing
        /// the graph's representation: it is a value, not an accessor.</para>
        ///
        /// <para><b>Deliberately NOT folded into <see cref="Checksum"/>.</b> It is derived from the
        /// node count and edge targets, both of which the checksum already folds, so two peers that
        /// agree there cannot disagree here — folding it would add no discrimination while changing
        /// every checksum that exists, and the checksum rides the nav fingerprint. That is a replay
        /// refusal bought for nothing.</para>
        /// </summary>
        public int NodeComponentCount { get; private set; }

        /// <summary>The mesh this graph was last derived from.</summary>
        public FPNavMesh CurrentMesh => _navMesh;

        /// <summary>
        /// The lattice pitch this graph was derived at, in world units — <b>the width of a node</b>,
        /// and therefore how far a leg can be. Build identity: it decides where agents walk, and it
        /// rides <see cref="Checksum"/>.
        ///
        /// <para>Exposed because a node's width is the scale a leg has to be read against.
        /// <see cref="FPNavAgentSystem"/> compares an agent's reach radius to it — a radius wider
        /// than a node means the agent is "there" the moment a leg is planned — and a hand-wired
        /// graph carries no other record of the size its caller picked.</para>
        /// </summary>
        public FP64 CellSize => _cellSize;

        /// <summary>
        /// The area mask this graph was derived under — the one that decided which triangles are
        /// walkable to it. <see cref="FPNavAgentSystem.ResolveLegTarget"/> compares an agent's
        /// resolved plan mask against this to decide whether the graph may speak for that agent.
        /// </summary>
        internal int AreaMask => _areaMask;

        /// <summary>
        /// How per-node cost was folded. Build identity alongside <see cref="CellSize"/> and
        /// <see cref="AreaMask"/>, and exposed for the same reason: those three plus the mesh are
        /// the whole input, so anything rebuilding this graph — a tool, or
        /// <c>FPNavAgentSystem.PrepareAbstractGraphFor</c> deriving the next one ahead of a swap —
        /// needs all three and a hand-wired graph records them nowhere else.
        /// </summary>
        internal FPNavAbstractCostFold CostFold => _costFold;

        /// <summary>
        /// Derives the graph. <paramref name="cellSize"/> is the lattice pitch in world units and is
        /// the dial worth tuning: too small and a leg is a couple of triangles, so an agent commits
        /// to a portal every few metres for nothing; too large and a leg no longer fits the corridor
        /// buffer, which <see cref="FPNavAgentSystem.SetAbstractGraph"/> refuses. Both the size and
        /// <paramref name="costFold"/> change where agents walk, so <b>every peer must derive with
        /// the same ones</b> — the fingerprint carries <see cref="Checksum"/> and says so if they
        /// do not.
        /// </summary>
        public FPNavAbstractGraph(
            FPNavMesh navMesh, FP64 cellSize, FPNavAbstractCostFold costFold,
            int areaMask, IKLogger logger = null)
            : this(navMesh, cellSize, costFold, areaMask, logger, measureOnly: false)
        {
        }

        private FPNavAbstractGraph(
            FPNavMesh navMesh, FP64 cellSize, FPNavAbstractCostFold costFold,
            int areaMask, IKLogger logger, bool measureOnly)
        {
            if (navMesh == null)
                throw new ArgumentNullException(nameof(navMesh));
            if (cellSize <= FP64.Zero)
                throw new ArgumentException("FPNavAbstractGraph: cellSize must be positive", nameof(cellSize));

            _logger = logger;
            _cellSize = cellSize;
            _costFold = costFold;
            _areaMask = areaMask;
            Derive(navMesh, measureOnly, null);
        }

        /// <summary>
        /// <see cref="MaxLegCorridorTriangles"/> for a graph over this mesh at this cell size,
        /// without building one. The install ladder tries cell sizes from the top down and needs
        /// only that number to reject a candidate — building the rest is most of a derivation's
        /// cost (on the Field the first candidate's pair table alone was ~95 ms that nothing read).
        ///
        /// <para>The partition and the diameter are all this runs, measured at 3.6-6.3% of a full
        /// derivation on the Field (2.1-2.7 ms). The candidate the ladder keeps therefore pays that
        /// prefix twice — once here, once building — which is the price of never handing anyone a
        /// half-built graph. Nothing constructed by the public constructor is missing its edges,
        /// its pair table or its checksum.</para>
        /// </summary>
        internal static int MeasureLegCorridorTriangles(
            FPNavMesh navMesh, FP64 cellSize, FPNavAbstractCostFold costFold, int areaMask)
            => new FPNavAbstractGraph(navMesh, cellSize, costFold, areaMask, null, measureOnly: true)
                .MaxLegCorridorTriangles;

        /// <summary>
        /// Points this graph at a different mesh and rebuilds — part of a navmesh swap, mirroring
        /// the query/pathfinder/funnel trio.
        ///
        /// <para><paramref name="donor"/>, when given, is a graph of the same build
        /// identity (cell size, cost fold, area mask) whose pair-table rows may be COPIED for every
        /// node whose surroundings the rebake left exactly as they were, instead of walked and
        /// searched again — the rows are most of a derivation's cost and a rebake moves a handful
        /// of cells. The result is bit for bit the graph a derivation without a donor gives; the
        /// donor decides only how much of the clock is spent, never a value, and is read, never
        /// written. A donor that does not qualify is ignored, not refused (see
        /// <see cref="DebugRebindDonorIgnored"/>) — the derivation is then the plain one. A graph
        /// cannot donate to itself: the rows it would copy are the ones this derivation overwrites,
        /// which is exactly the in-place swap fallback's situation, and that path derives plain.</para>
        /// </summary>
        internal void Rebind(FPNavMesh newMesh, FPNavAbstractGraph donor = null)
        {
            if (newMesh == null)
                throw new ArgumentNullException(nameof(newMesh));
            if (ReferenceEquals(donor, this))
                throw new ArgumentException(
                    "FPNavAbstractGraph.Rebind: a graph cannot be its own donor — the rows it would " +
                    "copy are the ones this derivation overwrites. The in-place swap fallback " +
                    "re-derives without one.", nameof(donor));
            Derive(newMesh, false, donor);      // a rebind is always the whole derivation: the cell size was chosen already
        }

        /// <summary>
        /// Which node claims a triangle, or <b>-1 when none does</b> — the triangle is blocked, its
        /// area is outside <see cref="AreaMask"/>, or the index names no triangle of
        /// <see cref="CurrentMesh"/>.
        ///
        /// <para><b>Public because a tool has to draw the partition, and there is no list of tools
        /// to write.</b> Unity's visualizer is one fixed assembly and could have been named in
        /// <c>InternalsVisibleTo</c>; the Godot one ships as source and is compiled into whatever
        /// assembly the consuming project happens to be, including projects outside this repository
        /// — the same reason <see cref="FPNavMeshQuery.FindTriangleForEndpoint"/> and
        /// <see cref="FPNavPathFailure"/> are public.</para>
        ///
        /// <para><b>This one accessor is the whole partition.</b> An abstract edge is exactly
        /// "two triangles that are mesh neighbours and belong to different nodes", and its portal is
        /// the midpoint of the edge they share — no folding, no chosen representative — so node
        /// membership, node adjacency and every portal position follow from this plus the mesh. The
        /// edge COSTS do not follow from it in any way worth reproducing, and that is where the line
        /// is drawn: reproducing them would be a second copy of a formula that can drift, with
        /// nothing but a picture to notice.</para>
        ///
        /// <para><b>Node ids are not stable.</b> They are handed out in triangle order as the
        /// derivation seeds each component, so a rebake renumbers all of them. They identify a group
        /// within one derivation and mean nothing across two.</para>
        /// </summary>
        public int NodeOf(int triangleIndex)
        {
            // Normalised at BOTH ends, and neither is theoretical. The backing array is grown by
            // capacity and reused across Rebind, so an index past the current mesh reads a node id
            // left over from a larger one; a negative index would throw. Internal callers only ever
            // pass an index they got from a query, which is why this never had to be said before —
            // and exactly why it has to be said now.
            if (_nodeOfTriangle == null || _navMesh == null) return -1;
            if (triangleIndex < 0 || triangleIndex >= _navMesh.Triangles.Length) return -1;
            return _nodeOfTriangle[triangleIndex];
        }

        internal int NodeTriangleCount(int node) => _nodeTriangleCount[node];
        internal int EdgeTarget(int edge) => _edgeTarget[edge];

        /// <summary>The edge that crosses the same portal the other way, or -1 (see <see cref="DebugReverseUnmatched"/>).</summary>
        internal int EdgeReverse(int edge) => _edgeReverse[edge];

        /// <summary>
        /// The cost of walking <paramref name="node"/> from the portal at its local out-edge index
        /// <paramref name="inLocal"/> to the one at <paramref name="outLocal"/> — the entry the
        /// abstract search charges for every hop through a node it is passing THROUGH.
        ///
        /// <para>The value is the straight distance between the two portal midpoints when that
        /// segment is walkable, and the in-node midpoint-graph distance (the same model the
        /// triangle A* prices a corridor with) when a wall lies between them; both times the node's
        /// cost fold. Priced from the portal it was entered by, a node that is walked straight
        /// through costs what it is — the centre-to-portal-to-centre stand-in this replaced charged
        /// a detour to the centre for every pass, and picked crossings by it.</para>
        ///
        /// <para><b>One value per UNORDERED pair, read as if it were directional.</b> The table
        /// stores what the walk from the lower-indexed portal to the higher one found and mirrors
        /// it (see <see cref="BuildPairRows"/>), but the walk is not symmetric — it pivots through
        /// vertex fans in the order it meets them — so the entry a hop reads is the value of ONE
        /// direction, whichever way the hop is going. Measured on the Field asset at cell 32, the
        /// cell the auto-install ladder picks: 3,898 of 9,974 pairs (39%) differ between the two
        /// directions, worst ratio 1.17, and it grows as cells shrink (5.08 at cell 8, 19.29 at
        /// cell 4). It is nearly invisible at the level that matters — recomputing the table
        /// directionally changes the FIRST hop of 6 of 5,903 queries (0.10%) — which is why this
        /// stands: fixing it means computing every ordered pair, so twice the calls to
        /// <c>PairValue</c> and twice the table, and it would move every fingerprint.</para>
        /// </summary>
        internal FP64 PairCost(int node, int inLocal, int outLocal)
        {
            int outdeg = _edgeStart[node + 1] - _edgeStart[node];
            return _pairCost[PairIndex(_pairStart[node], outdeg, inLocal, outLocal)];
        }

        /// <summary>
        /// Where a node's (in, out) entry lives — the ONE place the table's layout is written.
        /// It is folded into <see cref="Checksum"/>, so two encodings that drift give two peers
        /// two tables while every other symptom stays silent.
        /// </summary>
        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private static int PairIndex(int baseIdx, int outdeg, int inLocal, int outLocal)
            => baseIdx + inLocal * outdeg + outLocal;

        /// <summary>Entries in the pair table — the sum over nodes of outdeg squared.</summary>
        internal int PairEntryCount => _pairEntryCount;

        /// <summary>
        /// Edges whose reverse could not be found by matching portal ends. Zero by construction
        /// on a mesh with symmetric adjacency; a hop through such an edge prices itself live in
        /// <see cref="HopCost"/> — which is NOT the table's value but a LOWER BOUND on it, because
        /// the live price is the straight portal-to-portal line and the table would have charged
        /// the in-node search where a wall lies between them. So a non-zero count is a mesh defect
        /// that makes some hops look cheaper than they are, not merely a slowdown.
        /// </summary>
        internal int DebugReverseUnmatched => _reverseUnmatched;

        /// <summary>
        /// Triangle adjacencies the in-node search found to be one-directional, last derivation —
        /// the same mesh defect <see cref="DebugReverseUnmatched"/> counts at a node boundary, met
        /// inside a node instead. Zero on anything the build pipeline made, which pairs both sides
        /// in one statement; a deserialized mesh is not checked, so this is where one shows up.
        ///
        /// <para>Reaching it is not a function of the mesh alone: the search runs only for pairs a
        /// wall stands between, and only over triangles of the SAME node — so the same mesh can
        /// derive cleanly at one cell size and report here at another, or first report after a
        /// rebake moves the geometry. Deterministic either way, so peers agree.</para>
        /// </summary>
        internal int DebugAdjacencyAsymmetric => _adjacencyAsymmetric;

        /// <summary>
        /// Edge-midpoint distance triples the last derivation filled — one per triangle the
        /// walled-pair search relaxed. Zero on a mesh whose nodes are all convex: no straight line
        /// between two portals of such a node touches a wall, so the search never runs.
        /// </summary>
        internal long DebugMidDistFills => _debugMidDistFills;

        /// <summary>
        /// Triangles the walled-pair search pushed again after popping them, last derivation.
        /// Provably zero — a popped key is the minimum of a heap fed by non-negative weights, so
        /// nothing arriving later can beat it — which makes a non-zero value the signature of the
        /// heap no longer being a min-heap. The branch that would do it is kept for that reason.
        /// </summary>
        internal long DebugPairRepushes => _debugPairRepushes;

        /// <summary>Triangles stepped through by the segment walks of the last derivation.</summary>
        internal long DebugWalkSteps => _debugWalkSteps;

        /// <summary>FP64 divisions taken by those walks (see <see cref="SegmentWalkable"/> for what is divided when).</summary>
        internal long DebugWalkDivisions => _debugWalkDivisions;

        /// <summary>
        /// Of those walks, how many ran out of steps rather than arriving — the cap in
        /// <see cref="SegmentWalkable"/> being reached. Zero over the shipped assets (worst 99 of
        /// 1024), and a pair that hits it is priced by the in-node search instead, which only ever
        /// raises the value: the failure mode is an over-estimate and a second-best hop, not a
        /// wrong answer. Non-zero is the signal that the cap has become load-bearing.
        ///
        /// <para>Like every counter here it is of the LAST call to <c>BuildPairTable</c>, and a
        /// <see cref="Rebind"/> goes through that same reset — so a rebind that copied most of its
        /// rows from a donor did not walk them, and this under-reports by construction. Measure it
        /// on a derivation with no donor.</para>
        /// </summary>
        internal long DebugWalkExhausted => _debugWalkExhausted;

        /// <summary>
        /// Triangles of the last derivation whose <c>costMultiplier</c> is below 1 — the condition
        /// that costs <see cref="FPNavAbstractCostFold.Min"/> its admissibility. Zero over the
        /// shipped assets (all 22,563 are exactly 1). The derivation warns once when it is not.
        /// </summary>
        internal int DebugCheapTriangles => _cheapTriangles;

        /// <summary>Whether some edge of the node's triangles has nothing walkable across it —
        /// the flag that, with its neighbours', decides whether a node's pairs are walked at all.
        /// Not folded into <see cref="Checksum"/>, which is why an equality test reads it.</summary>
        internal bool NodeHasWall(int node) => _nodeHasWall[node];

        /// <summary>Whether the last derivation copied rows from a donor (<see cref="Rebind"/>).</summary>
        internal bool DebugRebindDonorUsed => _debugRebindDonorUsed;
        /// <summary>Why a donor given to the last derivation was not used; null when it was, or none was given.</summary>
        internal string DebugRebindDonorIgnored => _debugRebindDonorIgnored;
        /// <summary>Nodes whose rows the last derivation copied from the donor.</summary>
        internal int DebugRebindNodesReused => _debugRebindNodesReused;
        /// <summary>Nodes the last derivation walked and searched despite a donor.</summary>
        internal int DebugRebindNodesRebuilt => _debugRebindNodesRebuilt;
        /// <summary>Of the rebuilt nodes, those whose surroundings were unchanged but whose
        /// <c>wallNear</c> flipped — a wall two cells away appeared or went.</summary>
        internal int DebugRebindWallNearFlips => _debugRebindWallNearFlips;
        /// <summary>Nodes whose surroundings were unchanged and yet no donor node or portal
        /// permutation matched — zero by construction; anything else is a rule bug.</summary>
        internal int DebugRebindMatchFailures => _debugRebindMatchFailures;
        /// <summary>Pairs of reused nodes computed afresh because the donor had walked them the
        /// other way (the portal order flipped with the triangle order).</summary>
        internal int DebugRebindPairsRecomputed => _debugRebindPairsRecomputed;

        /// <summary>
        /// Where a leg through this edge aims — the midpoint of the shared triangle edge. On the
        /// mesh by construction (both its triangles are walkable), which is what makes it a legal
        /// destination for the real A* to solve the leg against.
        /// </summary>
        internal FPVector3 EdgePortal(int edge) => _edgePortal[edge];

        /// <summary>
        /// How far in from each end of a portal an aim point is allowed to sit, as a fraction of the
        /// segment. A portal's ends are mesh VERTICES, shared by every triangle around them, so a
        /// point landing exactly on one leaves <c>FindTriangleForEndpoint</c> to pick between them —
        /// the leg would be solved against a triangle chosen by tie-break rather than by the route.
        /// A sixteenth is exact in fixed point and small enough not to bend a corner.
        /// </summary>
        private static readonly FP64 PortalInset = FP64.One / FP64.FromInt(16);

        /// <summary>
        /// Where a leg through this edge should aim, given where the agent stands and what it is
        /// heading for next. <b>This is the segment being used as a segment</b> — the midpoint is
        /// only the cost model's stand-in for it.
        ///
        /// <para>The point that minimises <c>|from - X| + |X - to|</c> over a segment is where the
        /// straight line between them crosses it; if it does not cross, the best point is whichever
        /// end is nearer to that line. So: cross once, else take the better end. Both branches are
        /// pulled <see cref="PortalInset"/> in from the vertices.</para>
        ///
        /// <para>Aiming at the midpoint instead is what bent the route — a unit crossing a wide
        /// portal diagonally was dragged to its centre, and the next leg started from there. The
        /// funnel could never undo that: it only runs inside the current corridor, which ends at the
        /// portal.</para>
        /// </summary>
        internal FPVector3 AimPointOn(int edge, FPVector2 from, FPVector2 to)
        {
            FPVector3 a3 = _edgePortalA[edge];
            FPVector3 b3 = _edgePortalB[edge];
            FPVector2 a = a3.ToXZ();
            FPVector2 b = b3.ToXZ();

            FPVector2 ab = b - a;
            FPVector2 pq = to - from;
            FP64 denom = ab.x * pq.y - ab.y * pq.x;

            FP64 t;
            if (denom == FP64.Zero)
            {
                // Parallel: no crossing to find, so fall through to the endpoint comparison below.
                t = FP64.MinValue;
            }
            else
            {
                FPVector2 ap = from - a;
                t = (ap.x * pq.y - ap.y * pq.x) / denom;
            }

            FP64 lo = PortalInset;
            FP64 hi = FP64.One - PortalInset;
            if (t >= FP64.Zero && t <= FP64.One)
            {
                t = t < lo ? lo : (t > hi ? hi : t);
            }
            else
            {
                // No crossing inside the segment — hug the end that makes the detour smallest.
                FPVector2 loP = a + ab * lo;
                FPVector2 hiP = a + ab * hi;
                FP64 loCost = FPVector2.Distance(from, loP) + FPVector2.Distance(loP, to);
                FP64 hiCost = FPVector2.Distance(from, hiP) + FPVector2.Distance(hiP, to);
                t = loCost <= hiCost ? lo : hi;
            }

            return a3 + (b3 - a3) * t;
        }

        /// <summary>Half-open edge range of a node in the CSR arrays.</summary>
        internal void EdgeRange(int node, out int start, out int end)
        {
            start = _edgeStart[node];
            end = _edgeStart[node + 1];
        }

        #region Derivation

        /// <summary>
        /// The lattice cell a point falls in, anchored at the world origin. Fixed rather than
        /// bounds-relative on purpose: a bounds-anchored lattice moves whenever the extremes move
        /// and every node id moves with it. Measured today the rebaker never moves them — it
        /// refuses or clips placements at the walkable boundary — but that stability is a side
        /// effect of the placement policies rather than a promise, and relying on it would let a
        /// policy change renumber nodes silently.
        /// </summary>
        private void CellOf(FPVector2 p, out int col, out int row)
        {
            col = FloorDiv(p.x, _cellSize);
            row = FloorDiv(p.y, _cellSize);
        }

        /// <summary>Floor division — negatives round the same way as positives, so the lattice has
        /// no seam at the origin.</summary>
        private static int FloorDiv(FP64 value, FP64 size)
            // FP64.ToInt is an arithmetic shift of the raw value, so it already floors — for
            // negatives too. Correcting it as if it truncated toward zero floors a second time,
            // which is what put a seam at the origin: the band just left of zero landed two cell
            // indices away from the band just right of it, while the plane itself kept the index
            // in between and became a cell of its own. Everything that reads these as ADJACENT
            // (the re-derivation's 3x3 dirty window) then looked past its own neighbour.
            => (value / size).ToInt();

        private void Derive(FPNavMesh mesh, bool measureOnly, FPNavAbstractGraph donor)
        {
            _navMesh = mesh;
            ReadOnlySpan<FPNavMeshTriangle> tris = mesh.Triangles;
            int triCount = tris.Length;
            _triCount = triCount;
            _reach2Valid = false;
            _cellOrderValid = false;
            _debugRebindDonorUsed = false;
            _debugRebindDonorIgnored = null;
            _debugRebindNodesReused = 0;
            _debugRebindNodesRebuilt = 0;
            _debugRebindWallNearFlips = 0;
            _debugRebindMatchFailures = 0;
            _debugRebindPairsRecomputed = 0;

            EnsureTriangleCapacity(triCount);

            // Pass 1 — the cell each triangle sits in, keyed by its centre so a triangle lands in
            // exactly one cell. Membership has to be a partition; the mesh's own cell lists record
            // overlap instead, which would put one triangle in several nodes at once.
            for (int t = 0; t < triCount; t++)
            {
                CellOf(tris[t].centerXZ, out _cellCol[t], out _cellRow[t]);
                _nodeOfTriangle[t] = -1;
            }

            // Pass 2 — nodes are connected components WITHIN a cell. A cell is not a node on its
            // own: one cell routinely holds both sides of a wall (measured: 18.5% of cells on the
            // Field asset), and merging those invents a crossing that does not exist.
            int nodeCount = 0;
            for (int seed = 0; seed < triCount; seed++)
            {
                if (_nodeOfTriangle[seed] >= 0 || !Walkable(tris[seed]))
                    continue;
                FloodFill(tris, seed, nodeCount);
                nodeCount++;
            }
            NodeCount = nodeCount;
            EnsureNodeCapacity(nodeCount);

            // Pass 3 — node payload in one sweep. Accumulating in triangle-index order is what
            // makes the fixed-point sum bit-identical everywhere: a different visit order rounds
            // differently and, through the edge costs, plans a different abstract path.
            AccumulateNodes(tris, triCount, nodeCount);

            // Pass 4 — the diameter that decides whether a leg fits the corridor buffer.
            MaxNodeDiameter = MeasureMaxDiameter(tris, triCount);

            // MeasureLegCorridorTriangles wanted only the number above, and the instance it
            // measured on never leaves that method — so the rest, which is most of a derivation's
            // cost, is skipped and no half-built graph reaches a caller.
            if (measureOnly) return;

            BuildEdges(tris, triCount, nodeCount);
            BuildReverseEdges(nodeCount);
            BuildPairTable(tris, triCount, nodeCount, QualifiedDonor(donor));
            // Said here rather than inside the table build so a rebind that copied most of its
            // rows reports what it actually walked, and says it once either way.
            if (_adjacencyAsymmetric > 0)
                _logger?.KError($"[FPNavAbstractGraph] navmesh adjacency is not symmetric: the in-node search met {_adjacencyAsymmetric} triangle(s) listed as a neighbour by one they do not list back, and skipped them. Those pairs fall back to their straight line, which is a LOWER bound on what walking them costs. The mesh did not come from this build pipeline.");
            NodeComponentCount = CountNodeComponents(nodeCount);
            Checksum = ComputeChecksum(triCount, nodeCount);
        }

        private const string DONOR_IDENTITY = "the donor's cell size, cost fold or area mask differs";
        private const string DONOR_MESH_RETIRED = "the donor's mesh was retired — its storage belongs to a newer one";

        /// <summary>
        /// The donor if its rows can mean anything here, else null with the reason recorded. Each
        /// check is one the double buffer in <c>FPNavAgentSystem</c> satisfies by construction; they
        /// are made anyway, because copying a row from a graph that fails one is a wrong table on
        /// every peer alike.
        /// </summary>
        private FPNavAbstractGraph QualifiedDonor(FPNavAbstractGraph donor)
        {
            if (donor == null) return null;
            if (donor._cellSize != _cellSize || donor._costFold != _costFold || donor._areaMask != _areaMask)
            { _debugRebindDonorIgnored = DONOR_IDENTITY; return null; }
            // The rows were computed over the donor's mesh AS IT WAS. The rebake driver pools
            // meshes and hands a retired one's arrays to the next mesh, so a caller that donates
            // something other than the live graph can offer a table describing geometry that is
            // gone. The retirement flag is the tripwire: it stays true after the arrays have been
            // overwritten, which neither a triangle count (the pool hands back the same sizes) nor
            // a fingerprint (it would read the new contents) can manage. It also touches no array,
            // so asking does not trip the mesh's own live guard.
            if (donor._navMesh == null || donor._navMesh.IsRetired)
            { _debugRebindDonorIgnored = DONOR_MESH_RETIRED; return null; }
            return donor;
        }

        private bool Walkable(in FPNavMeshTriangle t)
            => !t.isBlocked && (_areaMask & t.areaMask) != 0;

        /// <summary>Claims every triangle reachable from the seed without leaving its cell.</summary>
        private void FloodFill(ReadOnlySpan<FPNavMeshTriangle> tris, int seed, int node)
        {
            int col = _cellCol[seed], row = _cellRow[seed];
            int top = 0;
            _scratch[top++] = seed;
            _nodeOfTriangle[seed] = node;

            while (top > 0)
            {
                int cur = _scratch[--top];
                for (int e = 0; e < 3; e++)
                {
                    int nb = tris[cur].GetNeighbor(e);
                    if (nb < 0 || _nodeOfTriangle[nb] >= 0) continue;
                    if (!Walkable(tris[nb])) continue;
                    if (_cellCol[nb] != col || _cellRow[nb] != row) continue;
                    _nodeOfTriangle[nb] = node;
                    _scratch[top++] = nb;
                }
            }
        }

        /// <summary>
        /// Node centre, size and cost fold, in ONE pass over the triangles.
        /// <para>The fold belongs here rather than at the edge that needs it. Computing it per edge
        /// reads every triangle again, which is O(edges x triangles) — measured at 222 ms on the
        /// Field asset against 21 ms for the same derivation with this pass (before the pair table,
        /// which is where a derivation's time goes now — see BuildPairTable), so "thin first draft"
        /// would have shipped an accidentally quadratic one and skewed the D-7 budget call.</para>
        /// </summary>
        private void AccumulateNodes(ReadOnlySpan<FPNavMeshTriangle> tris, int triCount, int nodeCount)
        {
            bool min = _costFold == FPNavAbstractCostFold.Min;
            _cheapTriangles = 0;
            for (int n = 0; n < nodeCount; n++)
            {
                _nodeTriangleCount[n] = 0;
                _nodeFold[n] = min ? FP64.MaxValue : FP64.Zero;
                _nodeHasWall[n] = false;
            }
            for (int t = 0; t < triCount; t++)
            {
                int n = _nodeOfTriangle[t];
                if (n < 0) continue;
                _nodeTriangleCount[n]++;

                FP64 c = tris[t].costMultiplier;
                if (c < FP64.One) _cheapTriangles++;
                if (min) { if (c < _nodeFold[n]) _nodeFold[n] = c; }
                else _nodeFold[n] += c;
            }
            // A fold below 1 makes the abstract estimate PESSIMISTIC where the real path is
            // cheaper than plain distance, which is the one direction Min was chosen to avoid
            // (see FPNavAbstractCostFold.Min). Nothing here refuses it — the bake does not bound
            // costMultiplier and a game is free to author fast ground — so the derivation says so
            // once instead, and the counter is what a test reads.
            if (_cheapTriangles > 0)
                _logger?.KWarning($"[FPNavAbstractGraph] {_cheapTriangles} of {triCount} triangles price below 1 — the abstract estimate is no longer optimistic, so a first hop may not be the best one");
            for (int n = 0; n < nodeCount; n++)
            {
                int size = _nodeTriangleCount[n];
                if (size <= 0) { _nodeFold[n] = FP64.One; continue; }
                if (!min) _nodeFold[n] = _nodeFold[n] / FP64.FromInt(size);
            }
        }

        /// <summary>Double sweep per node — see <see cref="MaxNodeDiameter"/>.</summary>
        private int MeasureMaxDiameter(ReadOnlySpan<FPNavMeshTriangle> tris, int triCount)
        {
            int worst = 0;
            int seen = -1;
            for (int t = 0; t < triCount; t++)
            {
                int n = _nodeOfTriangle[t];
                if (n < 0 || n <= seen) continue;    // node ids rise with seed index, so this
                seen = n;                            // visits each node once, at its seed
                if (_nodeTriangleCount[n] <= 1) continue;

                int far = BfsFarthest(tris, t, n, out _);
                BfsFarthest(tris, far, n, out int depth);
                if (depth > worst) worst = depth;
            }
            return worst;
        }

        private int BfsFarthest(ReadOnlySpan<FPNavMeshTriangle> tris, int from, int node, out int depth)
        {
            int head = 0, tail = 0;
            _visitGeneration++;
            _scratch[tail++] = from;
            _depth[from] = 0;
            _visitStamp[from] = _visitGeneration;

            int farthest = from;
            depth = 0;
            while (head < tail)
            {
                int cur = _scratch[head++];
                if (_depth[cur] > depth) { depth = _depth[cur]; farthest = cur; }
                for (int e = 0; e < 3; e++)
                {
                    int nb = tris[cur].GetNeighbor(e);
                    if (nb < 0 || _nodeOfTriangle[nb] != node) continue;
                    if (_visitStamp[nb] == _visitGeneration) continue;
                    _visitStamp[nb] = _visitGeneration;
                    _depth[nb] = _depth[cur] + 1;
                    _scratch[tail++] = nb;
                }
            }
            return farthest;
        }

        /// <summary>
        /// One directed edge per portal — an adjacency whose two triangles sit in different nodes.
        /// Cost follows the shape the real A* charges (a leg into the portal, a leg out of it, times
        /// the destination's cost multiplier) rather than plain node-to-node distance: if the two
        /// scales disagree, the abstract search picks legs the real search then refuses.
        /// </summary>
        private void BuildEdges(ReadOnlySpan<FPNavMeshTriangle> tris, int triCount, int nodeCount)
        {
            for (int n = 0; n <= nodeCount; n++) _edgeStart[n] = 0;

            int total = 0;
            for (int t = 0; t < triCount; t++)
            {
                int a = _nodeOfTriangle[t];
                if (a < 0) continue;
                for (int e = 0; e < 3; e++)
                {
                    int nb = tris[t].GetNeighbor(e);
                    if (nb < 0) continue;
                    int b = _nodeOfTriangle[nb];
                    if (b < 0 || b == a) continue;
                    _edgeStart[a + 1]++;
                    total++;
                }
            }
            for (int n = 0; n < nodeCount; n++) _edgeStart[n + 1] += _edgeStart[n];

            EnsureEdgeCapacity(total);
            for (int n = 0; n < nodeCount; n++) _cursor[n] = 0;
            // The inverse of (_edgeTri, _edgeTriEdge). Filling it here costs one store per portal
            // and turns two searches that were O(outdeg) into a lookup — see BuildReverseEdges
            // and MapPortals, both of which were finding an edge they could already name.
            if (_edgeOfTriEdge == null || _edgeOfTriEdge.Length < triCount * 3)
                _edgeOfTriEdge = new int[triCount * 3];
            for (int i = 0, end = triCount * 3; i < end; i++) _edgeOfTriEdge[i] = -1;

            for (int t = 0; t < triCount; t++)
            {
                int a = _nodeOfTriangle[t];
                if (a < 0) continue;
                for (int e = 0; e < 3; e++)
                {
                    int nb = tris[t].GetNeighbor(e);
                    if (nb < 0 || _nodeOfTriangle[nb] < 0)
                    {
                        _nodeHasWall[a] = true;      // nothing walkable across this edge
                        continue;
                    }
                    int b = _nodeOfTriangle[nb];
                    if (b == a) continue;

                    tris[t].GetEdgeVertices(e, out int va, out int vb);

                    int slot = _edgeStart[a] + _cursor[a]++;
                    _edgeTarget[slot] = b;
                    _edgeTri[slot] = t;
                    _edgeTriEdge[slot] = e;
                    _edgeOfTriEdge[t * 3 + e] = slot;
                    _edgePortalA[slot] = _navMesh.Vertices[va];
                    _edgePortalB[slot] = _navMesh.Vertices[vb];
                    _edgePortal[slot] =
                        (_navMesh.Vertices[va] + _navMesh.Vertices[vb]) * FP64.Half;
                }
            }
            EdgeCount = total;
        }

        private ulong ComputeChecksum(int triCount, int nodeCount)
        {
            // Same mixer shape as the navmesh fingerprint: fixed multipliers, both halves of every
            // value, no container in sight. This value crosses processes.
            ulong h = FPHash.FNV_OFFSET;
            void Mix(long v)
            {
                unchecked
                {
                    h = (h ^ (ulong)v) * FPHash.FNV_PRIME;
                    h = (h ^ ((ulong)v >> 32)) * FPHash.FNV_PRIME;
                }
            }

            Mix(PLAN_RULE_REVISION);
            Mix(_cellSize.RawValue);
            Mix((long)_costFold);
            Mix(_areaMask);
            Mix(nodeCount);
            Mix(EdgeCount);
            Mix(MaxNodeDiameter);
            for (int t = 0; t < triCount; t++) Mix(_nodeOfTriangle[t]);
            for (int n = 0; n < nodeCount; n++)
                Mix(_nodeTriangleCount[n]);
            for (int i = 0; i < EdgeCount; i++)
            {
                Mix(_edgeTarget[i]);
                // The portal SEGMENT, not just the cost. Once a leg aims along it rather than at
                // its midpoint, the two ends decide where an agent walks — and "the mesh and the
                // edge order already pin it" stops being true the moment the representation is a
                // first-class input rather than a derived convenience.
                Mix(_edgePortalA[i].x.RawValue);
                Mix(_edgePortalA[i].z.RawValue);
                Mix(_edgePortalB[i].x.RawValue);
                Mix(_edgePortalB[i].z.RawValue);
            }
            // The pair table decides where agents walk, so it is folded like the portals are:
            // not for discrimination (it is a pure function of what is already folded) but as the
            // net that catches a derivation that has stopped being deterministic.
            for (int i = 0; i < _pairEntryCount; i++) Mix(_pairCost[i].RawValue);
            return h;
        }

        #endregion

        #region Pair table

        /// <summary>
        /// For every edge, the edge that crosses the same portal the other way — found by matching
        /// portal ends within the target node's range. Both directions are always emitted
        /// (adjacency is symmetric), so this is an index, not a search for something that may be
        /// missing; an unmatched edge is counted and reported, and prices itself live.
        /// </summary>
        private void BuildReverseEdges(int nodeCount)
        {
            _reverseUnmatched = 0;
            for (int a = 0; a < nodeCount; a++)
            {
                for (int e = _edgeStart[a]; e < _edgeStart[a + 1]; e++)
                {
                    // The same portal from the other side is the out-edge of the triangle across
                    // it, on the edge that faces back — which _edgeOfTriEdge names outright. The
                    // scan this replaced compared portal endpoints over the target node's whole
                    // out-edge list; on a well-formed mesh the two pick the same edge, and where
                    // they would not (one-directional adjacency) there is no reverse to pick.
                    int t = _edgeTri[e], te = _edgeTriEdge[e];
                    int nb = _navMesh.Triangles[t].GetNeighbor(te);
                    int back = nb < 0 ? -1 : EdgeIndexOf(in _navMesh.Triangles[nb], t);
                    int found = back < 0 ? -1 : _edgeOfTriEdge[nb * 3 + back];
                    _edgeReverse[e] = found;
                    if (found < 0) _reverseUnmatched++;
                }
            }
            if (_reverseUnmatched > 0)
                _logger?.KWarning($"[FPNavAbstractGraph] {_reverseUnmatched} of {EdgeCount} portal edges have no reverse — those hops price themselves live");
        }

        /// <summary>
        /// Fills the per-node pair table (see <see cref="PairCost"/>). Straight distance between the
        /// two portal midpoints when the segment between them is walkable; otherwise a Dijkstra
        /// over the node's triangles from the entry portal, in the same edge-midpoint model the
        /// triangle A* uses, never below the straight line. Both times the node's cost fold.
        ///
        /// <para>The walkability test is skipped for a node with no wall on it or on any node it
        /// shares a portal with — an open field prices every pair straight without walking one
        /// segment. Where walls are near, the Dijkstra runs once per portal that has at least one
        /// blocked pair, and every pair in that row reads from it.</para>
        ///
        /// <para>Everything here is index-ordered fixed point: the table is folded into
        /// <see cref="Checksum"/> and must come out identical on every peer.</para>
        /// </summary>
        private void BuildPairTable(ReadOnlySpan<FPNavMeshTriangle> tris, int triCount, int nodeCount,
            FPNavAbstractGraph donor)
        {
            _debugMidDistFills = 0;
            _debugPairRepushes = 0;
            _debugWalkSteps = 0;
            _debugWalkDivisions = 0;
            _debugWalkExhausted = 0;
            _adjacencyAsymmetric = 0;

            int total = 0;
            for (int n = 0; n < nodeCount; n++)
            {
                _pairStart[n] = total;
                int d = _edgeStart[n + 1] - _edgeStart[n];
                total += d * d;
            }
            _pairStart[nodeCount] = total;
            _pairEntryCount = total;
            EnsurePairCapacity(total, triCount);

            // A new derivation invalidates every midpoint-distance triple: the same triangle index
            // can be a different triangle after a rebind.
            _triMidGeneration++;
            if (_triMidGeneration == int.MaxValue)
            {
                Array.Clear(_triMidStamp, 0, _triMidStamp.Length);
                _triMidGeneration = 1;
            }

            // With a donor, every node whose surroundings the rebake left exactly as they were
            // takes its rows from it (see PrepareReuse); the rest are built as always.
            bool reuse = donor != null && PrepareReuse(tris, triCount, nodeCount, donor);
            _debugRebindDonorUsed = reuse;
            for (int n = 0; n < nodeCount; n++)
            {
                if (reuse && _reuseNodeDonor[n] >= 0)
                {
                    BuildPairRows(tris, n, donor);
                    _debugRebindNodesReused++;
                }
                else
                {
                    BuildPairRows(tris, n, null);
                    if (reuse) _debugRebindNodesRebuilt++;
                }
            }
        }

        /// <summary>
        /// Whether the walkability test is needed for a node's rows at all: a wall on the node or on
        /// any node it shares a portal with. Read off the topology, so a donor answers it for its
        /// own nodes exactly as it did when it built them.
        ///
        /// <para><b>This 1-ring is narrower than what it guards.</b> The walk it skips is not bounded
        /// to the node or to its neighbours — <see cref="SegmentWalkable"/> follows the segment
        /// wherever it leads, stopping only at a wall or at the target triangle. So a node whose own
        /// ring is clear can still have a wall on the straight line between two of its portals, and
        /// this returns false for it: the pair keeps the straight price and the wall is not charged.
        /// Widening the ring or always walking would both move values, hence fingerprints, so it
        /// stays as it is until it is shown to matter. Measured over the shipped assets at cells
        /// 4..256: exactly ONE node in twenty configurations has a clear ring, and it is on a stage
        /// whose graph is not installed.</para>
        /// </summary>
        private bool WallNear(int n)
        {
            int start = _edgeStart[n], end = _edgeStart[n + 1];
            bool wallNear = _nodeHasWall[n];
            for (int e = start; e < end && !wallNear; e++)
                wallNear = _nodeHasWall[_edgeTarget[e]];
            return wallNear;
        }

        /// <summary>
        /// One node's rows, every pair walked and searched — the whole cost of the table is here.
        /// The upper triangle is computed from the lower-indexed portal to the higher and mirrored,
        /// and the direction is part of the value: the walk pivots through vertex fans in the
        /// order it meets them, so a pair is not provably the same walked the other way.
        ///
        /// <para>Which means the mirroring is a choice, not an identity — it stores one direction
        /// and hands it to hops going both ways. <see cref="PairCost"/> carries what that costs,
        /// measured. Note that canonicalising the walk direction geometrically would NOT fix it:
        /// storage is one value per unordered pair either way, so it only changes WHICH direction
        /// both hops read. What it would fix is the reverse-pair recomputation <paramref name="donor"/>
        /// forces below, which is a different concern.</para>
        /// </summary>
        /// <param name="donor">When given, the node's counterpart in a graph whose rows may be
        /// reused: a pair is COPIED when the donor computed it in the same direction — from the
        /// lower-indexed portal to the higher, and a rebake reorders portals with the triangles —
        /// and walked afresh otherwise, because a row is exact or it is not a row. Null derives
        /// every pair, which is what a fresh derivation and a rebind without a donor both do.</param>
        private void BuildPairRows(ReadOnlySpan<FPNavMeshTriangle> tris, int n, FPNavAbstractGraph donor)
        {
            int start = _edgeStart[n];
            int d = _edgeStart[n + 1] - start;
            if (d == 0) return;
            int baseIdx = _pairStart[n];
            FP64 fold = _nodeFold[n];
            bool wallNear = WallNear(n);
            bool reuse = donor != null;
            int donorBase = reuse ? donor._pairStart[_reuseNodeDonor[n]] : 0;

            for (int i = 0; i < d; i++)
            {
                _pairCost[PairIndex(baseIdx, d, i, i)] = FP64.Zero;
                bool searching = false;      // this row's in-node search has begun
                int si = reuse ? _reuseSigma[start + i] : 0;
                for (int j = i + 1; j < d; j++)
                {
                    FP64 cost;
                    int sj = reuse ? _reuseSigma[start + j] : 0;
                    if (reuse && si < sj)
                    {
                        cost = donor._pairCost[PairIndex(donorBase, d, si, sj)];
                    }
                    else
                    {
                        cost = PairValue(tris, n, start + i, start + j, wallNear, fold, ref searching);
                        if (reuse) _debugRebindPairsRecomputed++;
                    }
                    _pairCost[PairIndex(baseIdx, d, i, j)] = cost;
                    _pairCost[PairIndex(baseIdx, d, j, i)] = cost;
                }
            }
        }

        /// <summary>
        /// The cost of one pair, walked from portal <paramref name="ei"/> to <paramref name="ej"/>
        /// (both out-edges of node <paramref name="n"/>): the straight distance between the two
        /// midpoints when that segment is walkable, else the in-node search from
        /// <paramref name="ei"/>, never below the straight line; times the fold either way.
        /// <paramref name="searching"/> is the row's flag that its search has begun — pairs of one
        /// row share one search, in whatever order they ask.
        /// </summary>
        private FP64 PairValue(ReadOnlySpan<FPNavMeshTriangle> tris, int n, int ei, int ej,
            bool wallNear, FP64 fold, ref bool searching)
        {
            FPVector2 mi = _edgePortal[ei].ToXZ();
            FPVector2 mj = _edgePortal[ej].ToXZ();
            FP64 geo = FPVector2.Distance(mi, mj);
            if (wallNear && !SegmentWalkable(tris, ei, mi, mj, _edgeTri[ej]))
            {
                // The search is resumed per blocked target and runs only until that target has
                // been popped — a popped distance is final (see RunNodeSearchUntilPopped), so the
                // row stops at the last portal it actually needs instead of settling the whole
                // node. Begin comes before the first read of the row: the generation only moves
                // here, so a row with no blocked pair leaves the previous row's stamps in place
                // and a read ahead of Begin would take them for its own.
                int tj = _edgeTri[ej];
                if (!searching) { BeginNodeSearch(ei); searching = true; }
                RunNodeSearchUntilPopped(tris, n, tj);
                if (Popped(tj))
                {
                    FP64 viaNode = _pairDist[tj] + MidDist(tris, tj, _pairEntryEdge[tj], _edgeTriEdge[ej]);
                    if (viaNode > geo) geo = viaNode;
                }
            }
            return geo * fold;
        }

        private const int MAX_SEGMENT_WALK = 1024;

        // Parametric slack along the segment: an exit has to lie measurably PAST the current
        // position, otherwise the edges meeting at a vertex the segment passes through would all
        // read as exits at once. Exact in fixed point; about 2.4e-4 of the segment.
        private static readonly FP64 WalkStep = FP64.FromRaw(1L << 20);

        /// <summary>
        /// Walks the triangles a straight segment crosses, starting on the portal of
        /// <paramref name="fromEdge"/> and stopping when it reaches <paramref name="stopTri"/> or a
        /// triangle containing <paramref name="q"/>. False the moment it would cross an edge with
        /// nothing walkable across it, or when it cannot find its way — treated as blocked so the
        /// pair gets the searched value rather than the straight one.
        ///
        /// <para>Lattice meshes make this harder than it looks: a segment between two portal
        /// midpoints routinely runs along a triangle edge and through vertices. So the exit of a
        /// triangle is found parametrically — the edge the segment crosses first, strictly past
        /// where it is now, with edges parallel to it ignored — and when the segment sits exactly
        /// on a vertex with no such exit it pivots around that vertex's fan, one triangle at a
        /// time, until a triangle offers one. Every choice is by index order, so peers agree.</para>
        /// </summary>
        private bool SegmentWalkable(ReadOnlySpan<FPNavMeshTriangle> tris, int fromEdge,
            FPVector2 p, FPVector2 q, int stopTri)
        {
            int cur = _edgeTri[fromEdge];
            int prev = tris[cur].GetNeighbor(_edgeTriEdge[fromEdge]);   // the far side of the start portal
            FPVector2 d = q - p;
            FP64 t = FP64.Zero;
            for (int step = 0; step < MAX_SEGMENT_WALK; step++)
            {
                _debugWalkSteps++;
                if (cur == stopTri) return true;
                ref readonly FPNavMeshTriangle tri = ref tris[cur];
                FPVector2 a = _navMesh.Vertices[tri.v0].ToXZ();
                FPVector2 b = _navMesh.Vertices[tri.v1].ToXZ();
                FPVector2 c = _navMesh.Vertices[tri.v2].ToXZ();
                if (FPNavMeshQuery.PointInTriangle2D(q, a, b, c)) return true;

                int exit = -1, pivot = -1;
                FP64 exitT = FP64.MaxValue;
                for (int k = 0; k < 3; k++)
                {
                    int nb = tri.GetNeighbor(k);
                    if (nb >= 0 && nb == prev) continue;
                    tri.GetEdgeVertices(k, out int va, out int vb);
                    FPVector2 u = _navMesh.Vertices[va].ToXZ();
                    FPVector2 e = _navMesh.Vertices[vb].ToXZ() - u;
                    FP64 denom = d.x * e.y - d.y * e.x;
                    if (denom == FP64.Zero) continue;                 // parallel or along it: not an exit
                    FPVector2 w = u - p;
                    // The division is the expensive part of a step (FP64 takes ~30 ns on a
                    // numerator past half a unit), and every test here is one quotient against
                    // one threshold — so the tests are made on the quotient's numerator and
                    // denominator directly (CompareQuotient), bit for bit what the division would
                    // have decided, and the quotient itself is only computed for an edge that is
                    // a candidate exit: those are compared against each other, and that ordering
                    // needs the rounded values. The parameter along the SEGMENT goes first, so an
                    // edge line hit past the end or behind the current position is out before the
                    // edge parameter is looked at.
                    long dr = denom.RawValue;
                    long nt = (w.x * e.y - w.y * e.x).RawValue;     // numerator of where along p->q the edge line is hit
                    if (QuotientAbove(nt, dr, FP64.One.RawValue)) continue;
                    bool ahead = QuotientAbove(nt, dr, (t + WalkStep).RawValue);
                    if (!ahead && !(!QuotientBelow(nt, dr, (t - WalkStep).RawValue) && pivot < 0)) continue;
                    long ns = (w.x * d.y - w.y * d.x).RawValue;     // and of where along u->v
                    if (QuotientBelow(ns, dr, 0L) || QuotientAbove(ns, dr, FP64.One.RawValue)) continue;
                    if (ahead)
                    {
                        FP64 tk = FP64.FromRaw(nt) / denom;
                        _debugWalkDivisions++;
                        if (tk < exitT) { exitT = tk; exit = k; }
                    }
                    else
                        pivot = k;                                   // the segment is on this edge's vertex
                }

                int chosen = exit >= 0 ? exit : pivot;
                if (chosen < 0) return false;
                int next = tri.GetNeighbor(chosen);
                if (next < 0 || _nodeOfTriangle[next] < 0) return false;   // a wall
                if (exit >= 0) t = exitT;
                prev = cur;
                cur = next;
            }
            // Out of steps. The pair falls back to the searched value exactly as a blocked one
            // does, so this is conservative — the value only ever goes UP from the straight line,
            // never below it — and it costs one increment per walk, not per step. Measured 99 of
            // 1024 at worst over the shipped assets (cell 4..256, peak at 64), and reproducible:
            // a strip one lattice quad wide cut into three cells hits the cap at 520 quads a cell.
            _debugWalkExhausted++;
            return false;
        }

        /// <summary>
        /// Starts the in-node search for one portal: a Dijkstra over the node's triangles in the
        /// edge-midpoint model the triangle A* prices a corridor with — a triangle costs the
        /// distance from the midpoint of the edge it was entered by to the midpoint of the edge it
        /// is left by. State is the triangle, its label the entry edge; the distances live in
        /// <c>_pairDist</c> under the current generation.
        /// </summary>
        private void BeginNodeSearch(int edge)
        {
            _pairGeneration++;
            if (_pairGeneration == int.MaxValue)
            {
                Array.Clear(_pairStamp, 0, _pairStamp.Length);
                _pairGeneration = 1;
            }
            _pairHeap.Clear();

            int src = _edgeTri[edge];
            _pairStamp[src] = _pairGeneration;
            _pairDist[src] = FP64.Zero;
            _pairEntryEdge[src] = _edgeTriEdge[edge];
            _pairHeap.Push(src, FP64.Zero);
        }

        /// <summary>
        /// Whether the current search has popped <paramref name="tri"/>, i.e. whether its distance
        /// is final. Read off the heap's own bookkeeping — stamped means pushed, and the heap's
        /// position array answers "still in it" in O(1) — rather than off a third scratch array.
        /// </summary>
        private bool Popped(int tri)
            => _pairStamp[tri] == _pairGeneration && !_pairHeap.Contains(tri);

        /// <summary>
        /// Runs the search begun by <see cref="BeginNodeSearch"/> until <paramref name="target"/>
        /// has been popped, or the heap is empty (a target no walkable triangle of the node reaches
        /// — the caller then prices the pair by the straight line). Calling it again for a later
        /// target picks up where it stopped.
        ///
        /// <para>Stopping at the pop is exact, not an approximation: every weight is a distance
        /// between two midpoints, so it is non-negative, and the heap hands out its minimum — a
        /// key popped later is never smaller, so nothing that arrives after a pop can improve on
        /// it. The re-push below is therefore unreachable; it stays as the tripwire that would
        /// fire if the heap ever stopped being a min-heap (<see cref="DebugPairRepushes"/>).</para>
        /// </summary>
        private void RunNodeSearchUntilPopped(ReadOnlySpan<FPNavMeshTriangle> tris, int node, int target)
        {
            while (!Popped(target) && _pairHeap.Count > 0)
            {
                int cur = _pairHeap.Pop();
                FP64 dc = _pairDist[cur];
                int a = _pairEntryEdge[cur];
                ref readonly FPNavMeshTriangle tri = ref tris[cur];
                for (int k = 0; k < 3; k++)
                {
                    // Not back out through the entry edge: across it lies the predecessor, whose
                    // distance is no worse, or — for the source — the far side of the portal.
                    if (k == a) continue;
                    int nb = tri.GetNeighbor(k);
                    if (nb < 0 || _nodeOfTriangle[nb] != node) continue;
                    FP64 tentative = dc + MidDist(tris, cur, a, k);
                    if (_pairStamp[nb] == _pairGeneration)
                    {
                        if (tentative >= _pairDist[nb]) continue;
                        int back = EdgeIndexOf(in tris[nb], cur);
                        if (back < 0) { _adjacencyAsymmetric++; continue; }
                        _pairDist[nb] = tentative;
                        _pairEntryEdge[nb] = back;
                        if (_pairHeap.Contains(nb)) _pairHeap.DecreaseKey(nb, tentative);
                        else { _debugPairRepushes++; _pairHeap.Push(nb, tentative); }
                    }
                    else
                    {
                        int back = EdgeIndexOf(in tris[nb], cur);
                        if (back < 0) { _adjacencyAsymmetric++; continue; }
                        _pairStamp[nb] = _pairGeneration;
                        _pairDist[nb] = tentative;
                        _pairEntryEdge[nb] = back;
                        _pairHeap.Push(nb, tentative);
                    }
                }
            }
        }

        /// <summary>
        /// The distance between the midpoints of edges <paramref name="a"/> and <paramref name="b"/>
        /// of triangle <paramref name="t"/> — the edge-midpoint model's cost of crossing it. The
        /// triple for a triangle is computed on its first use in a derivation and looked up after;
        /// the midpoints are the same sums the portal midpoints are built from, so the values are
        /// bit for bit what a distance taken on the spot would be.
        /// </summary>
        private FP64 MidDist(ReadOnlySpan<FPNavMeshTriangle> tris, int t, int a, int b)
        {
            if (a == b) return FP64.Zero;
            if (_triMidStamp[t] != _triMidGeneration)
            {
                _debugMidDistFills++;
                ref readonly FPNavMeshTriangle tri = ref tris[t];
                tri.GetEdgeVertices(0, out int a0, out int b0);
                tri.GetEdgeVertices(1, out int a1, out int b1);
                tri.GetEdgeVertices(2, out int a2, out int b2);
                FPVector2 m0 = (_navMesh.Vertices[a0].ToXZ() + _navMesh.Vertices[b0].ToXZ()) * FP64.Half;
                FPVector2 m1 = (_navMesh.Vertices[a1].ToXZ() + _navMesh.Vertices[b1].ToXZ()) * FP64.Half;
                FPVector2 m2 = (_navMesh.Vertices[a2].ToXZ() + _navMesh.Vertices[b2].ToXZ()) * FP64.Half;
                int at = t * 3;
                _triMidDist[at + 2] = FPVector2.Distance(m0, m1);   // {0, 1}
                _triMidDist[at + 0] = FPVector2.Distance(m1, m2);   // {1, 2}
                _triMidDist[at + 1] = FPVector2.Distance(m0, m2);   // {0, 2}
                _triMidStamp[t] = _triMidGeneration;
            }
            return _triMidDist[t * 3 + (3 - a - b)];
        }

        /// <summary>
        /// The edge of <paramref name="tri"/> that faces <paramref name="neighbour"/>, or -1 when
        /// none does. Adjacency is symmetric in a well-formed mesh, so one of the three always
        /// does: the build pipeline pairs both sides in one statement and only a mesh that did not
        /// come through it can be one-directional (nothing validates adjacency on deserialize).
        ///
        /// <para>Guessing is not an option — a wrong entry edge would price the triangle by the
        /// wrong pair of midpoints and fold into the checksum, wrong on every peer alike. Neither
        /// is throwing: this runs inside a swap, on the deterministic command path, and the same
        /// derivation TOLERATES the node-boundary form of exactly this defect (see
        /// <see cref="DebugReverseUnmatched"/>). So the caller skips the neighbour and the
        /// derivation reports how often — one policy for one defect.</para>
        /// </summary>
        private static int EdgeIndexOf(in FPNavMeshTriangle tri, int neighbour)
        {
            for (int k = 0; k < 3; k++)
                if (tri.GetNeighbor(k) == neighbour) return k;
            return -1;
        }

        /// <summary>
        /// <c>(a / b).RawValue &gt; t</c> for raw FP64 <paramref name="a"/>, <paramref name="b"/>
        /// (b non-zero) and raw threshold <paramref name="t"/> — without dividing. It reproduces
        /// <c>FP64</c>'s division exactly, which has TWO regimes: a numerator whose magnitude is
        /// under 2^31 raw (half a unit) goes through native 64-bit division, truncated toward zero;
        /// anything larger goes through a bit loop on the magnitudes that rounds half up, then
        /// takes the sign. Both reduce to one comparison of <c>|a| · 2^32</c> (or <c>2^33</c>)
        /// against a multiple of <c>|b|</c>, exact in 128 bits — one product per question, which
        /// is why the walk asks one-sided questions. A division saturates far outside any
        /// threshold the walk uses, so the comparison and the saturated value never disagree on
        /// a decision.
        /// </summary>
        internal static bool QuotientAbove(long a, long b, long t)
        {
            if (a == 0) return t < 0;
            ulong ua = (ulong)(a < 0 ? -a : a);
            ulong ub = (ulong)(b < 0 ? -b : b);
            bool truncating = ua < 0x80000000UL;
            if ((a < 0) == (b < 0))
            {
                // Q = +m: m > t. floor(x) > t ⟺ x ≥ t + 1; round(x) > t ⟺ 2x ≥ 2t + 1.
                if (t < 0) return true;
                ulong m = (ulong)t;
                return truncating ? ShiftedAtLeast(ua, 32, m + 1, ub) : ShiftedAtLeast(ua, 33, 2 * m + 1, ub);
            }
            // Q = -m: -m > t ⟺ m < -t, impossible unless t < 0. floor(x) < M ⟺ x < M;
            // round(x) < M ⟺ 2x < 2M - 1.
            if (t >= 0) return false;
            ulong big = (ulong)(-t);
            return truncating ? !ShiftedAtLeast(ua, 32, big, ub) : !ShiftedAtLeast(ua, 33, 2 * big - 1, ub);
        }

        /// <summary><c>(a / b).RawValue &lt; t</c> — the mirror of <see cref="QuotientAbove"/>: negating the numerator negates the quotient exactly in both regimes.</summary>
        internal static bool QuotientBelow(long a, long b, long t) => QuotientAbove(-a, b, -t);

        /// <summary>Sign of <c>(a / b).RawValue - t</c>; the two one-sided questions, for tests.</summary>
        internal static int CompareQuotient(long a, long b, long t)
            => QuotientAbove(a, b, t) ? 1 : (QuotientBelow(a, b, t) ? -1 : 0);

        /// <summary><c>(a &lt;&lt; shift) ≥ m · b</c> in 128 bits, unsigned; shift is 32 or 33.</summary>
        private static bool ShiftedAtLeast(ulong a, int shift, ulong m, ulong b)
        {
            ulong leftHi = a >> (64 - shift), leftLo = a << shift;
            ulong rightHi = FPInt128.MulUnsigned(m, b, out ulong rightLo);
            return leftHi != rightHi ? leftHi > rightHi : leftLo >= rightLo;
        }

        /// <summary>
        private void EnsurePairCapacity(int entries, int triCount)
        {
            if (_pairCost == null || _pairCost.Length < entries)
                _pairCost = new FP64[entries];
            // One condition for the whole triangle-sized bundle: a rebind to a larger mesh must
            // grow every one of these, or the first one left behind is an index out of range that
            // only a small-to-large rebake would ever hit.
            if (_pairDist == null || _pairDist.Length < triCount)
            {
                _pairDist = new FP64[triCount];
                _pairEntryEdge = new int[triCount];
                _pairStamp = new int[triCount];
                _pairHeap = new FPNavMeshBinaryHeap(triCount);
                _pairGeneration = 0;
                _triMidDist = new FP64[triCount * 3];
                _triMidStamp = new int[triCount];
                _triMidGeneration = 0;
            }
        }

        #endregion

        #region Incremental re-derivation — copying pair rows from a donor

        // Scratch for the donor comparison (see PrepareReuse). Grow-only like the rest; indexed by
        // this derivation's triangles, nodes and edges, or by the donor's triangles.
        private int[] _reuseOrderNew;         // this mesh's triangle indices in cell order
        private int[] _reuseOrderDonor;       // the donor mesh's, likewise
        private long[] _reuseKeysNew;         // one cell's canonical triangle keys, KEY_LONGS each
        private long[] _reuseKeysDonor;
        private int[] _reuseSortNew;          // the permutation that sorts those keys
        private int[] _reuseSortDonor;
        private int[] _reuseTriMatch;         // this mesh's triangle -> the donor's, -1 where its cell differs
        private int[] _reuseDirtyCol;         // the cells that differ, in (col, row) order
        private int[] _reuseDirtyRow;
        private int _reuseDirtyCount;
        private int[] _reuseNodeTri;          // one triangle of each node, for its cell
        private int[] _reuseNodeDonor;        // node -> donor node whose rows it takes, -1 to rebuild
        private int[] _reuseSigma;            // this graph's out-edge -> the donor node's local out-edge index
        private int[] _reuseCellStart;        // counting-sort scratch over the cell bounds
        private CellOrder _cellOrder;
        private KeyOrder _keyOrder;
        // The reach of this mesh (see ReachSquared), computed by PrepareReuse and kept for the day
        // this graph is the donor; false after every derivation until then.
        private FP64 _reach2;
        private bool _reach2Valid;
        private bool _cellOrderValid;         // _reuseOrderNew describes THIS graph's current mesh

        // The canonical key of a triangle: its three vertices IN STORED ORDER, then its flags. The
        // order is part of the key on purpose — edge k is (v_k, v_k+1), and the walk and the in-node
        // search take their decisions in edge order, so a rotated copy of the same triangle is a
        // different triangle to them. Sorting the vertices would call it the same and copy a row
        // that a fresh derivation would not have produced.
        //
        // The last long carries which of the three edges are BOUNDARIES, and that is deliberately
        // not the neighbour indices. A re-bake preserves geometry and renumbers triangles: measured
        // over the Field's first placement, 22,311 triangles are geometrically identical across the
        // re-bake and only 20.4% of them keep their neighbour indices, while 100% keep which edges
        // are boundaries. Keying on the indices would call four fifths of unchanged geometry
        // different and leave nothing to reuse; keying on boundaries costs nothing and closes the
        // gap that mattered — a walk reads a neighbour only through `< 0` and through whether the
        // triangle it names is in a node, so a severed link is what changes an answer.
        private const int KEY_LONGS = 13;

        private const string DONOR_REACH =
            "a triangle reaches more than half a cell from the point that placed it (2 r_max > cell), so a walk could leave the neighbouring cells";

        /// <summary>
        /// Decides, node by node, whose rows the donor can supply. A node's rows depend on its own
        /// triangles, on the triangles across its boundary (the portal set, the wall flag), on
        /// whatever its walks step through — measured never more than one cell away, and provably
        /// not while no triangle reaches more than half a cell from its centre — and on
        /// <c>wallNear</c>, which reads the neighbouring NODES' wall flags and so can reach a cell
        /// further. Hence the rule: a node takes the donor's rows when every cell of its 3x3
        /// neighbourhood holds, triangle for triangle in canonical order, exactly the triangles it
        /// held in the donor's mesh, and its <c>wallNear</c> is what the donor's was. Everything
        /// else is rebuilt. The cell comparison is exact — no hash — and the match is by geometry
        /// throughout, because a rebake renumbers every triangle, node and portal.
        ///
        /// <para>Returns false, with the reason recorded, when no row may be copied at all.</para>
        /// </summary>
        private bool PrepareReuse(ReadOnlySpan<FPNavMeshTriangle> tris, int triCount, int nodeCount,
            FPNavAbstractGraph donor)
        {
            int donorTris = donor._triCount;
            EnsureReuseCapacity(triCount, donorTris, nodeCount, EdgeCount);

            // 1. The reach guard, on both meshes: the 3x3 rule rests on it (plan §6 13). This
            //    mesh's reach is kept for when this graph donates; a donor derived fresh (the
            //    ladder's, at boot) has none recorded and is measured on the spot, once per match.
            FP64 cell2 = _cellSize * _cellSize;
            FP64 four = FP64.FromInt(4);
            _reach2 = ReachSquared(_navMesh, triCount);
            _reach2Valid = true;
            FP64 donorReach2 = donor._reach2Valid ? donor._reach2 : ReachSquared(donor._navMesh, donorTris);
            if (four * _reach2 > cell2 || four * donorReach2 > cell2)
            {
                _debugRebindDonorIgnored = DONOR_REACH;
                return false;
            }

            // 2. Both triangle lists in cell order, index order within a cell. The donor's mesh
            //    has not moved since it derived, so if it kept its own ordering the answer is
            //    already there — copy it instead of sorting again. It is missing exactly once per
            //    match: the first rebake's donor is the asset graph, which never prepared.
            for (int t = 0; t < triCount; t++) _reuseTriMatch[t] = -1;
            SortByCell(_cellCol, _cellRow, triCount, _reuseOrderNew);
            _cellOrderValid = true;
            if (donor._cellOrderValid && donor._reuseOrderNew != null
                && donor._reuseOrderNew.Length >= donorTris)
                Array.Copy(donor._reuseOrderNew, _reuseOrderDonor, donorTris);
            else
                SortByCell(donor._cellCol, donor._cellRow, donorTris, _reuseOrderDonor);

            // 3. Merge join by cell. A cell on both sides with the same triangles matches them one to
            //    one; a cell that differs, or exists on one side only, is dirty. Dirty cells come out
            //    in (col, row) order, which is what the lookup below binary-searches.
            _reuseDirtyCount = 0;
            int a = 0, b = 0;
            while (a < triCount || b < donorTris)
            {
                int cmp;
                if (a >= triCount) cmp = 1;
                else if (b >= donorTris) cmp = -1;
                else cmp = CompareCells(
                    _cellCol[_reuseOrderNew[a]], _cellRow[_reuseOrderNew[a]],
                    donor._cellCol[_reuseOrderDonor[b]], donor._cellRow[_reuseOrderDonor[b]]);

                if (cmp <= 0)
                {
                    int col = _cellCol[_reuseOrderNew[a]], row = _cellRow[_reuseOrderNew[a]];
                    int aEnd = a;
                    while (aEnd < triCount && _cellCol[_reuseOrderNew[aEnd]] == col && _cellRow[_reuseOrderNew[aEnd]] == row) aEnd++;
                    if (cmp < 0)
                    {
                        MarkDirty(col, row);            // this mesh only
                        a = aEnd;
                        continue;
                    }
                    int bEnd = b;
                    while (bEnd < donorTris && donor._cellCol[_reuseOrderDonor[bEnd]] == col && donor._cellRow[_reuseOrderDonor[bEnd]] == row) bEnd++;
                    if (aEnd - a != bEnd - b || !SameCell(a, aEnd - a, b, bEnd - b, donor))
                        MarkDirty(col, row);
                    a = aEnd;
                    b = bEnd;
                }
                else
                {
                    int col = donor._cellCol[_reuseOrderDonor[b]], row = donor._cellRow[_reuseOrderDonor[b]];
                    int bEnd = b;
                    while (bEnd < donorTris && donor._cellCol[_reuseOrderDonor[bEnd]] == col && donor._cellRow[_reuseOrderDonor[bEnd]] == row) bEnd++;
                    MarkDirty(col, row);                // the donor's mesh only
                    b = bEnd;
                }
            }

            // 4. Node by node: clean neighbourhood, then the donor node the matched triangle names,
            //    then the portal permutation, then wallNear.
            for (int n = 0; n < nodeCount; n++) { _reuseNodeTri[n] = -1; _reuseNodeDonor[n] = -1; }
            for (int t = 0; t < triCount; t++)
            {
                int n = _nodeOfTriangle[t];
                if (n >= 0 && _reuseNodeTri[n] < 0) _reuseNodeTri[n] = t;
            }
            for (int n = 0; n < nodeCount; n++)
            {
                int rep = _reuseNodeTri[n];
                if (rep < 0) continue;
                if (AnyDirtyAround(_cellCol[rep], _cellRow[rep])) continue;
                int dt = _reuseTriMatch[rep];
                int dn = dt >= 0 ? donor._nodeOfTriangle[dt] : -1;
                if (dn < 0 || donor._nodeTriangleCount[dn] != _nodeTriangleCount[n] || !MapPortals(n, dn, donor))
                {
                    // Cannot happen when the cell rule holds: the same triangles with the same
                    // neighbours form the same components with the same portals. Counted, never
                    // patched around — a partial copy would be a wrong row.
                    _debugRebindMatchFailures++;
                    continue;
                }
                if (WallNear(n) != donor.WallNear(dn))
                {
                    _debugRebindWallNearFlips++;
                    continue;
                }
                _reuseNodeDonor[n] = dn;
            }
            return true;
        }

        /// <summary>
        /// Whether the cell's triangles on the two sides are the same set, comparing canonical keys
        /// in sorted order; when they are, records the triangle-for-triangle match.
        /// </summary>
        private bool SameCell(int aStart, int count, int bStart, int donorCount, FPNavAbstractGraph donor)
        {
            EnsureCellCapacity(count);

            // Fast path: the rebaker keeps the triangles it did not touch in their relative order,
            // so a cell it left alone reads the same in index order on both sides. Sorting is for
            // the cells it did not leave alone, and for a donor from another triangulation.
            //
            // Streaming this — filling and comparing one triangle at a time, to stop a dirty cell
            // at its first difference — was tried and measured no better than filling both key
            // sets out first (rebind 8.10 -> 8.27 ms on the Field at cell 16, i.e. slightly worse
            // for the per-call overhead). A cell holds a handful of triangles; there is no run to
            // amortise anything over.
            FillKeys(_navMesh, _reuseOrderNew, aStart, count, _reuseKeysNew);
            FillKeys(donor._navMesh, _reuseOrderDonor, bStart, count, _reuseKeysDonor);
            bool aligned = true;
            for (int i = 0, end = count * KEY_LONGS; i < end; i++)
                if (_reuseKeysNew[i] != _reuseKeysDonor[i]) { aligned = false; break; }
            if (aligned)
            {
                for (int i = 0; i < count; i++)
                    _reuseTriMatch[_reuseOrderNew[aStart + i]] = _reuseOrderDonor[bStart + i];
                return true;
            }

            for (int i = 0; i < count; i++) { _reuseSortNew[i] = i; _reuseSortDonor[i] = i; }
            _keyOrder.Keys = _reuseKeysNew;
            HeapSort(_reuseSortNew, count, _keyOrder);
            _keyOrder.Keys = _reuseKeysDonor;
            HeapSort(_reuseSortDonor, count, _keyOrder);

            for (int i = 0; i < count; i++)
            {
                int ka = _reuseSortNew[i] * KEY_LONGS, kb = _reuseSortDonor[i] * KEY_LONGS;
                for (int k = 0; k < KEY_LONGS; k++)
                    if (_reuseKeysNew[ka + k] != _reuseKeysDonor[kb + k]) return false;
            }
            for (int i = 0; i < count; i++)
                _reuseTriMatch[_reuseOrderNew[aStart + _reuseSortNew[i]]] = _reuseOrderDonor[bStart + _reuseSortDonor[i]];
            return true;
        }

        private static void FillKeys(FPNavMesh mesh, int[] order, int start, int count, long[] keys)
        {
            ReadOnlySpan<FPNavMeshTriangle> tris = mesh.Triangles;
            ReadOnlySpan<FPVector3> verts = mesh.Vertices;
            for (int i = 0; i < count; i++)
            {
                ref readonly FPNavMeshTriangle tri = ref tris[order[start + i]];
                int at = i * KEY_LONGS;
                FPVector3 v0 = verts[tri.v0], v1 = verts[tri.v1], v2 = verts[tri.v2];
                keys[at + 0] = v0.x.RawValue; keys[at + 1] = v0.y.RawValue; keys[at + 2] = v0.z.RawValue;
                keys[at + 3] = v1.x.RawValue; keys[at + 4] = v1.y.RawValue; keys[at + 5] = v1.z.RawValue;
                keys[at + 6] = v2.x.RawValue; keys[at + 7] = v2.y.RawValue; keys[at + 8] = v2.z.RawValue;
                keys[at + 9] = tri.isBlocked ? 1 : 0;
                keys[at + 10] = tri.areaMask;
                keys[at + 11] = tri.costMultiplier.RawValue;
                keys[at + 12] = (tri.neighbor0 < 0 ? 1L : 0L)
                              | (tri.neighbor1 < 0 ? 2L : 0L)
                              | (tri.neighbor2 < 0 ? 4L : 0L);
            }
        }

        /// <summary>
        /// The portal permutation between a node and its donor node: each out-edge is one edge of
        /// one triangle, and the triangle match plus the edge number name the donor's slot. False
        /// when a slot is missing or the degrees differ — then nothing is copied.
        /// </summary>
        private bool MapPortals(int n, int dn, FPNavAbstractGraph donor)
        {
            int start = _edgeStart[n], end = _edgeStart[n + 1];
            int dStart = donor._edgeStart[dn], dEnd = donor._edgeStart[dn + 1];
            if (end - start != dEnd - dStart) return false;
            for (int e = start; e < end; e++)
            {
                int dt = _reuseTriMatch[_edgeTri[e]];
                int k = _edgeTriEdge[e];
                int found = donor._edgeOfTriEdge[dt * 3 + k];
                if (found < dStart || found >= dEnd) return false;   // -1, or the donor put it in another node
                _reuseSigma[e] = found - dStart;
            }
            return true;
        }

        /// <summary>
        /// Triangle indices in cell order — (col, row) ascending, index order within a cell. A
        /// counting sort over the mesh's cell bounds, which the lattice keeps dense; a mesh whose
        /// bounds are mostly empty falls back to a heapsort, whose only difference is the order
        /// within a cell, which nothing depends on.
        /// </summary>
        private void SortByCell(int[] col, int[] row, int count, int[] order)
        {
            if (count == 0) return;
            int minC = int.MaxValue, maxC = int.MinValue, minR = int.MaxValue, maxR = int.MinValue;
            for (int t = 0; t < count; t++)
            {
                if (col[t] < minC) minC = col[t];
                if (col[t] > maxC) maxC = col[t];
                if (row[t] < minR) minR = row[t];
                if (row[t] > maxR) maxR = row[t];
            }
            long h = (long)(maxR - minR) + 1;
            long cells = ((long)(maxC - minC) + 1) * h;
            if (cells > 4L * count + 1024)
            {
                for (int t = 0; t < count; t++) order[t] = t;
                _cellOrder.Col = col; _cellOrder.Row = row;
                HeapSort(order, count, _cellOrder);
                return;
            }
            int n = (int)cells;
            if (_reuseCellStart == null || _reuseCellStart.Length < n + 1)
                _reuseCellStart = new int[n + 1];
            Array.Clear(_reuseCellStart, 0, n + 1);
            for (int t = 0; t < count; t++)
                _reuseCellStart[(col[t] - minC) * h + (row[t] - minR) + 1]++;
            for (int i = 0; i < n; i++) _reuseCellStart[i + 1] += _reuseCellStart[i];
            for (int t = 0; t < count; t++)
                order[_reuseCellStart[(col[t] - minC) * h + (row[t] - minR)]++] = t;
        }

        /// <summary>The largest squared distance from a triangle's cell-placing point to one of
        /// its vertices, over the whole mesh — the reach the 3x3 rule is checked against.</summary>
        private static FP64 ReachSquared(FPNavMesh mesh, int triCount)
        {
            ReadOnlySpan<FPNavMeshTriangle> tris = mesh.Triangles;
            ReadOnlySpan<FPVector3> verts = mesh.Vertices;
            FP64 worst = FP64.Zero;
            for (int t = 0; t < triCount; t++)
            {
                ref readonly FPNavMeshTriangle tri = ref tris[t];
                FPVector2 c = tri.centerXZ;
                Consider(verts[tri.v0], c, ref worst);
                Consider(verts[tri.v1], c, ref worst);
                Consider(verts[tri.v2], c, ref worst);
            }
            return worst;

            static void Consider(FPVector3 v, FPVector2 c, ref FP64 worst)
            {
                FP64 dx = v.x - c.x, dz = v.z - c.y;
                FP64 d = dx * dx + dz * dz;
                if (d > worst) worst = d;
            }
        }

        private void MarkDirty(int col, int row)
        {
            if (_reuseDirtyCount == _reuseDirtyCol.Length)
            {
                Array.Resize(ref _reuseDirtyCol, _reuseDirtyCount * 2);
                Array.Resize(ref _reuseDirtyRow, _reuseDirtyCount * 2);
            }
            _reuseDirtyCol[_reuseDirtyCount] = col;
            _reuseDirtyRow[_reuseDirtyCount] = row;
            _reuseDirtyCount++;
        }

        private bool AnyDirtyAround(int col, int row)
        {
            for (int dc = -1; dc <= 1; dc++)
                for (int dr = -1; dr <= 1; dr++)
                    if (IsDirty(col + dc, row + dr)) return true;
            return false;
        }

        private bool IsDirty(int col, int row)
        {
            int lo = 0, hi = _reuseDirtyCount - 1;
            while (lo <= hi)
            {
                int mid = (lo + hi) >> 1;
                int c = CompareCells(_reuseDirtyCol[mid], _reuseDirtyRow[mid], col, row);
                if (c == 0) return true;
                if (c < 0) lo = mid + 1; else hi = mid - 1;
            }
            return false;
        }

        private static int CompareCells(int c1, int r1, int c2, int r2)
        {
            if (c1 != c2) return c1 < c2 ? -1 : 1;
            if (r1 != r2) return r1 < r2 ? -1 : 1;
            return 0;
        }

        private struct CellOrder : IComparer<int>
        {
            public int[] Col, Row;
            public int Compare(int a, int b) => CompareCells(Col[a], Row[a], Col[b], Row[b]);
        }

        private struct KeyOrder : IComparer<int>
        {
            public long[] Keys;
            public int Compare(int a, int b)
            {
                int ia = a * KEY_LONGS, ib = b * KEY_LONGS;
                for (int k = 0; k < KEY_LONGS; k++)
                {
                    long x = Keys[ia + k], y = Keys[ib + k];
                    if (x != y) return x < y ? -1 : 1;
                }
                return 0;
            }
        }

        /// <summary>
        /// In place and allocation-free. Any comparison sort would do: items that compare equal
        /// are interchangeable everywhere the order is read, so the result is the same on every
        /// host whatever the sort does with them.
        ///
        /// <para>The comparer is a STRUCT type argument, not an <c>IComparer&lt;int&gt;</c>
        /// reference: the JIT gives each one its own copy of this and the comparison inlines. An
        /// interface parameter would put a virtual call on the inner loop of a sort that runs per
        /// cell of a rebind.</para>
        /// </summary>
        private static void HeapSort<TOrder>(int[] items, int count, TOrder order)
            where TOrder : struct, IComparer<int>
        {
            for (int i = count / 2 - 1; i >= 0; i--) SiftDown(items, i, count, order);
            for (int end = count - 1; end > 0; end--)
            {
                (items[0], items[end]) = (items[end], items[0]);
                SiftDown(items, 0, end, order);
            }
        }

        private static void SiftDown<TOrder>(int[] items, int root, int count, TOrder order)
            where TOrder : struct, IComparer<int>
        {
            while (true)
            {
                int child = 2 * root + 1;
                if (child >= count) return;
                if (child + 1 < count && order.Compare(items[child + 1], items[child]) > 0) child++;
                if (order.Compare(items[child], items[root]) <= 0) return;
                (items[root], items[child]) = (items[child], items[root]);
                root = child;
            }
        }

        private void EnsureReuseCapacity(int triCount, int donorTris, int nodeCount, int edgeCount)
        {
            if (_reuseOrderNew == null || _reuseOrderNew.Length < triCount)
            {
                _reuseOrderNew = new int[triCount];
                _reuseTriMatch = new int[triCount];
            }
            if (_reuseOrderDonor == null || _reuseOrderDonor.Length < donorTris)
                _reuseOrderDonor = new int[donorTris];
            if (_reuseNodeTri == null || _reuseNodeTri.Length < nodeCount)
            {
                _reuseNodeTri = new int[nodeCount];
                _reuseNodeDonor = new int[nodeCount];
            }
            if (_reuseSigma == null || _reuseSigma.Length < edgeCount)
                _reuseSigma = new int[edgeCount];
            if (_reuseDirtyCol == null)
            {
                _reuseDirtyCol = new int[64];
                _reuseDirtyRow = new int[64];
            }
        }

        private void EnsureCellCapacity(int count)
        {
            if (_reuseSortNew != null && _reuseSortNew.Length >= count) return;
            _reuseSortNew = new int[count];
            _reuseSortDonor = new int[count];
            _reuseKeysNew = new long[count * KEY_LONGS];
            _reuseKeysDonor = new long[count * KEY_LONGS];
        }

        #endregion

        #region Abstract search

        private FPNavMeshBinaryHeap _open;
        /// <summary>The widest f margin the search squares (see the relaxation); 2^15 units.</summary>
        private static readonly FP64 SquareableMargin = FP64.FromInt(1 << 15);
        private FP64[] _gScore;
        private FP64[] _fScore;               // g + h(entry) — what a node's entry is chosen BY, see the search
        private int[] _cameFrom;
        private int[] _searchStamp;
        private bool[] _closed;
        private int _searchGeneration;
        private int _debugMissedImprovements;
        private bool _countMissedImprovements;

        /// <summary>
        /// Search-scratch readers for the measurement harness (a friend assembly): after a search
        /// it counts relaxations, closed and touched nodes from these, at no cost to the search.
        /// Meaningful only until the next search on this instance.
        /// </summary>
        internal int DebugSearchGeneration => _searchGeneration;
        internal bool DebugSearchTouched(int node)
            => _searchStamp != null && node < _searchStamp.Length && _searchStamp[node] == _searchGeneration;
        internal bool DebugSearchClosed(int node) => DebugSearchTouched(node) && _closed[node];

        /// <summary>
        /// How many relaxations found a CLOSED node whose entry they would have replaced (a lower
        /// f), cumulative. The node-state search never re-opens a closed node (Detour does); each
        /// such event is a place where that approximation cost something.
        /// </summary>
        internal int DebugMissedImprovements => _debugMissedImprovements;

        /// <summary>
        /// Whether the search counts <see cref="DebugMissedImprovements"/>. Off by default: the
        /// count needs a square root for every relaxation that lands on a closed node, which is
        /// most of them, and nothing but a measurement reads it. Turning it on changes no value —
        /// the counter is written, never read, by the search.
        /// </summary>
        internal bool DebugCountMissedImprovements
        {
            get => _countMissedImprovements;
            set => _countMissedImprovements = value;
        }

        /// <summary>The two ends of an edge's portal segment — what <see cref="EdgePortal"/> is the midpoint of.</summary>
        internal void EdgePortalSegment(int edge, out FPVector3 a, out FPVector3 b)
        {
            a = _edgePortalA[edge];
            b = _edgePortalB[edge];
        }

        /// <summary>
        /// The first hop of the cheapest node route from <paramref name="startNode"/> to
        /// <paramref name="goalNode"/>, and the edge that takes it — plus the two hops after it,
        /// which the unwinding hands out for free. Only those are returned: the agent walks one leg
        /// and asks again, so holding the rest would be state that has to survive a rollback for no
        /// gain.
        /// <para>No iteration budget is needed — A* over a graph closes each node at most once, so
        /// the search is bounded by the node count by construction. That is the whole point of
        /// planning up here.</para>
        /// </summary>
        /// <param name="fromXZ">Where the agent actually stands — the start node has no entry
        /// portal, so this is what its hops are priced from (see <see cref="HopCost"/>).</param>
        /// <param name="toXZ">The actual destination — the goal node has no exit portal, so the
        /// hop into it is charged the walk on to this point; it also anchors the heuristic.</param>
        internal bool TryFindFirstHop(
            int startNode, int goalNode, FPVector2 fromXZ, FPVector2 toXZ,
            out int firstEdge, out int secondEdge, out int thirdEdge)
        {
            firstEdge = -1;
            secondEdge = -1;
            thirdEdge = -1;
            if (startNode < 0 || goalNode < 0 || startNode >= NodeCount || goalNode >= NodeCount)
                return false;
            if (startNode == goalNode)
                return false;

            EnsureSearchCapacity(NodeCount);
            _searchGeneration++;
            _open.Clear();

            Touch(startNode);
            _gScore[startNode] = FP64.Zero;
            _fScore[startNode] = FPVector2.Distance(fromXZ, toXZ);
            _open.Push(startNode, _fScore[startNode]);

            while (_open.Count > 0)
            {
                int cur = _open.Pop();
                if (cur == goalNode)
                    return Unwind(startNode, goalNode, out firstEdge, out secondEdge, out thirdEdge);

                _closed[cur] = true;
                EdgeRange(cur, out int start, out int end);
                int rowBase = PairRowBase(cur, startNode, start, end - start);
                for (int e = start; e < end; e++)
                {
                    int nb = _edgeTarget[e];
                    Touch(nb);
                    FP64 tentative = _gScore[cur] + HopCost(cur, e, start, rowBase, startNode, goalNode, fromXZ, toXZ);

                    // A node is one state but has many entries, and what it costs to go ON from
                    // it depends on which one it was entered by. So an entry is kept or replaced
                    // by f, not by g: the nearest portal is the cheapest way IN and, when the
                    // destination lies elsewhere, the dearest way ON — priced by g alone the search
                    // hugged corners (measured: a 27° route grew 1.06x -> 1.14x). Detour compares
                    // its "total" here for the same reason.
                    //
                    // But the heuristic is a square root, and comparing f on every relaxation
                    // would take one each time — a Field search tripled that way (29 -> 90 us).
                    // The comparison is one distance against a margin, f_recorded - tentative, so
                    // it is made on squares: only a candidate that wins takes the root, which is
                    // the count the g-only loop had. The margin is squared in 32.32, exact below
                    // 2^15 units; a wider one falls through to the root.
                    if (_closed[nb])
                    {
                        // A closed node is not entered again, so everything below would be spent
                        // to reach the same `continue`. Measured on the Field: leaving here rather
                        // than three steps down is 44.65 -> 40.35 us per search at cell 16 (-9.6%)
                        // and -7.6% at cell 32, with the touched, closed and relaxation counts
                        // identical — the same search, the same routes, the same values.
                        //
                        // What moves up past is the count of improvements a closed node would have
                        // taken, which says whether the heuristic is consistent (measured: 0 on
                        // the Field, 12 per 800 searches on a synthetic open field). It costs a
                        // square root per closed relaxation now that the squared pre-filter is no
                        // longer in front of it, so it is taken only when asked for.
                        if (_countMissedImprovements
                            && (nb == goalNode
                                || tentative + FPVector2.Distance(_edgePortal[e].ToXZ(), toXZ) < _fScore[nb]))
                            _debugMissedImprovements++;
                        continue;
                    }
                    FP64 margin = _fScore[nb] - tentative;
                    if (margin <= FP64.Zero) continue;
                    FPVector2 mid = _edgePortal[e].ToXZ();
                    if (nb != goalNode && _fScore[nb] != FP64.MaxValue && margin < SquareableMargin
                        && FPVector2.SqrDistance(mid, toXZ) >= margin * margin)
                        continue;
                    FP64 f = tentative + (nb == goalNode ? FP64.Zero : FPVector2.Distance(mid, toXZ));
                    if (f >= _fScore[nb]) continue;

                    _gScore[nb] = tentative;
                    _fScore[nb] = f;
                    _cameFrom[nb] = e;               // the EDGE we arrived by, not the node
                    if (_open.Contains(nb)) _open.DecreaseKey(nb, f);
                    else _open.Push(nb, f);
                }
            }
            return false;
        }

        /// <summary>
        /// The row of <paramref name="cur"/>'s pair table for the portal it was entered by, computed
        /// once per expansion: <c>_cameFrom[cur]</c> is the edge that arrived, its reverse is the
        /// same portal as one of cur's own out-edges, and that local index is the row. -1 for the
        /// start node (it was not entered) and for an edge whose reverse is unmatched.
        /// </summary>
        private int PairRowBase(int cur, int startNode, int start, int outdeg)
        {
            if (cur == startNode) return -1;
            int rev = _edgeReverse[_cameFrom[cur]];
            if (rev < 0) return -1;
            return _pairStart[cur] + (rev - start) * outdeg;
        }

        /// <summary>
        /// The cost of one hop: walking <paramref name="cur"/> from where it was entered to the
        /// portal of <paramref name="edge"/>, times cur's fold — a table lookup for a node passed
        /// through, the distance from the agent's own position for the start node — plus, when the
        /// edge enters the goal node, the walk from that portal to the destination times the goal
        /// node's fold. Each stretch is charged to the node it is walked in.
        ///
        /// <para>This is where the two endpoints enter the search (HPA*'s temporary-node insertion,
        /// without allocating a node): the start's position and the destination stand in for the
        /// portals those two nodes have no entry or exit through. The graph itself does not change
        /// for either, which is why the rule carries a version of its own
        /// (<c>PLAN_RULE_REVISION</c>) — the table is folded into <see cref="Checksum"/>, the
        /// substitution is not.</para>
        /// </summary>
        private FP64 HopCost(int cur, int edge, int start, int rowBase,
            int startNode, int goalNode, FPVector2 fromXZ, FPVector2 toXZ)
        {
            FPVector2 mid = _edgePortal[edge].ToXZ();
            FP64 cost;
            if (cur == startNode)
                cost = FPVector2.Distance(fromXZ, mid) * _nodeFold[cur];
            else if (rowBase >= 0)
                cost = _pairCost[rowBase + (edge - start)];
            else
                // No reverse edge, so no row to read: priced straight, live. This is a LOWER BOUND
                // on what the table holds, not the same value — the table would have searched the
                // node where a wall lies between the two portals (see PairValue). Reached only on
                // a mesh with asymmetric adjacency; DebugReverseUnmatched counts how often.
                cost = FPVector2.Distance(_edgePortal[_cameFrom[cur]].ToXZ(), mid) * _nodeFold[cur];
            if (_edgeTarget[edge] == goalNode)
                cost += FPVector2.Distance(mid, toXZ) * _nodeFold[goalNode];
            return cost;
        }

        /// <summary>
        /// Walks the cameFrom chain back and hands out the first THREE edges of the route.
        ///
        /// <para>The second and third cost nothing: the walk already visits every edge from the goal
        /// back to the start, so remembering the previous ones as it goes is free. The second is what
        /// lets a leg aim at the portal AFTER the one it is crossing, which is the only direction
        /// that does not leave the next node — the destination can point clean out of it. The third
        /// is for the route that passes a lattice corner, where the first two crossings meet and an
        /// agent can be within its hand-off radius of both: the leg then has to aim past the
        /// second one too, or it ends on the tick it was planned.</para>
        ///
        /// <para>Each is -1 when the route has no such hop, i.e. the node before it is the goal
        /// node; the caller aims at the destination then.</para>
        /// </summary>
        private bool Unwind(int startNode, int goalNode,
            out int firstEdge, out int secondEdge, out int thirdEdge)
        {
            firstEdge = -1;
            secondEdge = -1;
            thirdEdge = -1;
            int node = goalNode;
            while (node != startNode)
            {
                int edge = _cameFrom[node];
                if (edge < 0) return false;          // chain broken — refuse rather than guess
                thirdEdge = secondEdge;
                secondEdge = firstEdge;
                firstEdge = edge;
                node = NodeOfEdgeSource(edge);
                if (node < 0) return false;
            }
            return firstEdge >= 0;
        }

        /// <summary>Which node an edge leaves, found from the CSR offsets by binary search.</summary>
        private int NodeOfEdgeSource(int edge)
        {
            int lo = 0, hi = NodeCount - 1;
            while (lo <= hi)
            {
                int mid = (lo + hi) >> 1;
                if (edge < _edgeStart[mid]) hi = mid - 1;
                else if (edge >= _edgeStart[mid + 1]) lo = mid + 1;
                else return mid;
            }
            return -1;
        }

        private void Touch(int node)
        {
            if (_searchStamp[node] == _searchGeneration) return;
            _searchStamp[node] = _searchGeneration;
            _gScore[node] = FP64.MaxValue;
            _fScore[node] = FP64.MaxValue;
            _cameFrom[node] = -1;
            _closed[node] = false;
        }

        private void EnsureSearchCapacity(int nodeCount)
        {
            if (_gScore != null && _gScore.Length >= nodeCount) return;
            _gScore = new FP64[nodeCount];
            _fScore = new FP64[nodeCount];
            _cameFrom = new int[nodeCount];
            _searchStamp = new int[nodeCount];
            _closed = new bool[nodeCount];
            _open = new FPNavMeshBinaryHeap(nodeCount);
            _searchGeneration = 0;
        }

        #endregion

        #region Buffers

        private void EnsureTriangleCapacity(int triCount)
        {
            if (_nodeOfTriangle != null && _nodeOfTriangle.Length >= triCount)
                return;
            _nodeOfTriangle = new int[triCount];
            _scratch = new int[triCount];
            _depth = new int[triCount];
            _visitStamp = new int[triCount];
            _cellCol = new int[triCount];
            _cellRow = new int[triCount];
            _visitGeneration = 0;
        }

        private void EnsureNodeCapacity(int nodeCount)
        {
            if (_edgeStart == null || _edgeStart.Length < nodeCount + 1)
                _edgeStart = new int[nodeCount + 1];
            if (_pairStart == null || _pairStart.Length < nodeCount + 1)
                _pairStart = new int[nodeCount + 1];
            if (_nodeTriangleCount == null || _nodeTriangleCount.Length < nodeCount)
            {
                _nodeTriangleCount = new int[nodeCount];
                _nodeFold = new FP64[nodeCount];
                _nodeHasWall = new bool[nodeCount];
                _cursor = new int[nodeCount];
                _nodeSeen = new bool[nodeCount];
            }
        }

        /// <summary>
        /// Connected components of the node graph, walked over the CSR the edge pass just built.
        /// Runs AFTER <see cref="BuildEdges"/> for that reason, and reuses <c>_scratch</c> as the
        /// queue — it is sized to the triangle count, which is never smaller than the node count,
        /// and the two passes that own it (the flood fill and the diameter sweep) are both done.
        ///
        /// <para>Node ids are visited in order and the queue is index-ordered, so this is one more
        /// deterministic pass; it never reaches <see cref="Checksum"/> anyway (see
        /// <see cref="NodeComponentCount"/> for why not).</para>
        /// </summary>
        private int CountNodeComponents(int nodeCount)
        {
            if (nodeCount <= 0) return 0;
            for (int n = 0; n < nodeCount; n++) _nodeSeen[n] = false;

            int components = 0;
            for (int seed = 0; seed < nodeCount; seed++)
            {
                if (_nodeSeen[seed]) continue;
                components++;

                int head = 0, tail = 0;
                _scratch[tail++] = seed;
                _nodeSeen[seed] = true;
                while (head < tail)
                {
                    int node = _scratch[head++];
                    for (int e = _edgeStart[node]; e < _edgeStart[node + 1]; e++)
                    {
                        int next = _edgeTarget[e];
                        if (_nodeSeen[next]) continue;
                        _nodeSeen[next] = true;
                        _scratch[tail++] = next;
                    }
                }
            }
            return components;
        }

        private void EnsureEdgeCapacity(int edgeCount)
        {
            if (_edgeTarget == null || _edgeTarget.Length < edgeCount)
            {
                _edgeTarget = new int[edgeCount];
                _edgeTri = new int[edgeCount];
                _edgeTriEdge = new int[edgeCount];
                _edgeReverse = new int[edgeCount];
                _edgePortal = new FPVector3[edgeCount];
                _edgePortalA = new FPVector3[edgeCount];
                _edgePortalB = new FPVector3[edgeCount];
            }
        }

        #endregion
    }
}
