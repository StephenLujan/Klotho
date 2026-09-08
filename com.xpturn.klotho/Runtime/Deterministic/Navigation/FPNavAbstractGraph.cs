using System;

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
        /// abstract estimate never exceeds what the real path charges.</summary>
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
        /// threshold, so it is not asked to reach a point it cannot arc into.</para>
        /// </summary>
        private const long PLAN_RULE_REVISION = 5;

        private readonly IKLogger _logger;
        private readonly FP64 _cellSize;
        private readonly FPNavAbstractCostFold _costFold;
        private readonly int _areaMask;

        private FPNavMesh _navMesh;

        private int[] _nodeOfTriangle;        // triangle -> node, -1 when no node claims it
        private FPVector2[] _nodeCenter;
        private int[] _nodeTriangleCount;
        private FP64[] _nodeFold;             // costMultiplier folded per node, see FPNavAbstractCostFold

        private int[] _edgeStart;             // CSR over node ids
        private int[] _edgeTarget;
        private FP64[] _edgeCost;
        private FPVector3[] _edgePortal;      // midpoint of the shared edge — the cost model's stand-in
        private FPVector3[] _edgePortalA;     // and the segment itself, which is what a leg aims along
        private FPVector3[] _edgePortalB;

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
        {
            if (navMesh == null)
                throw new ArgumentNullException(nameof(navMesh));
            if (cellSize <= FP64.Zero)
                throw new ArgumentException("FPNavAbstractGraph: cellSize must be positive", nameof(cellSize));

            _logger = logger;
            _cellSize = cellSize;
            _costFold = costFold;
            _areaMask = areaMask;
            Derive(navMesh);
        }

        /// <summary>
        /// Points this graph at a different mesh and rebuilds — part of a navmesh swap, mirroring
        /// the query/pathfinder/funnel trio.
        /// </summary>
        internal void Rebind(FPNavMesh newMesh)
        {
            if (newMesh == null)
                throw new ArgumentNullException(nameof(newMesh));
            Derive(newMesh);
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

        internal FPVector2 NodeCenter(int node) => _nodeCenter[node];
        internal int NodeTriangleCount(int node) => _nodeTriangleCount[node];
        internal int EdgeTarget(int edge) => _edgeTarget[edge];
        internal FP64 EdgeCost(int edge) => _edgeCost[edge];

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
        {
            FP64 q = value / size;
            int i = q.ToInt();                       // truncates toward zero
            if (q < FP64.Zero && FP64.FromInt(i) != q)
                i--;
            return i;
        }

        private void Derive(FPNavMesh mesh)
        {
            _navMesh = mesh;
            ReadOnlySpan<FPNavMeshTriangle> tris = mesh.Triangles;
            int triCount = tris.Length;

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

            BuildEdges(tris, triCount, nodeCount);
            NodeComponentCount = CountNodeComponents(nodeCount);
            Checksum = ComputeChecksum(triCount, nodeCount);
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
        /// Field asset against 21 ms for the same derivation with this pass, so "thin first draft"
        /// would have shipped an accidentally quadratic one and skewed the D-7 budget call.</para>
        /// </summary>
        private void AccumulateNodes(ReadOnlySpan<FPNavMeshTriangle> tris, int triCount, int nodeCount)
        {
            bool min = _costFold == FPNavAbstractCostFold.Min;
            for (int n = 0; n < nodeCount; n++)
            {
                _nodeCenter[n] = FPVector2.Zero;
                _nodeTriangleCount[n] = 0;
                _nodeFold[n] = min ? FP64.MaxValue : FP64.Zero;
            }
            for (int t = 0; t < triCount; t++)
            {
                int n = _nodeOfTriangle[t];
                if (n < 0) continue;
                _nodeCenter[n] += tris[t].centerXZ;
                _nodeTriangleCount[n]++;

                FP64 c = tris[t].costMultiplier;
                if (min) { if (c < _nodeFold[n]) _nodeFold[n] = c; }
                else _nodeFold[n] += c;
            }
            for (int n = 0; n < nodeCount; n++)
            {
                int size = _nodeTriangleCount[n];
                if (size <= 0) { _nodeFold[n] = FP64.One; continue; }
                _nodeCenter[n] = _nodeCenter[n] / FP64.FromInt(size);
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

                    tris[t].GetEdgeVertices(e, out int va, out int vb);
                    FPVector2 portalMid =
                        (_navMesh.Vertices[va].ToXZ() + _navMesh.Vertices[vb].ToXZ()) * FP64.Half;

                    FP64 cost = (FPVector2.Distance(_nodeCenter[a], portalMid)
                               + FPVector2.Distance(portalMid, _nodeCenter[b])) * _nodeFold[b];

                    int slot = _edgeStart[a] + _cursor[a]++;
                    _edgeTarget[slot] = b;
                    _edgeCost[slot] = cost;
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
            {
                Mix(_nodeCenter[n].x.RawValue);
                Mix(_nodeCenter[n].y.RawValue);
                Mix(_nodeTriangleCount[n]);
            }
            for (int i = 0; i < EdgeCount; i++)
            {
                Mix(_edgeTarget[i]);
                Mix(_edgeCost[i].RawValue);
                // The portal SEGMENT, not just the cost. Once a leg aims along it rather than at
                // its midpoint, the two ends decide where an agent walks — and "the mesh and the
                // edge order already pin it" stops being true the moment the representation is a
                // first-class input rather than a derived convenience.
                Mix(_edgePortalA[i].x.RawValue);
                Mix(_edgePortalA[i].z.RawValue);
                Mix(_edgePortalB[i].x.RawValue);
                Mix(_edgePortalB[i].z.RawValue);
            }
            return h;
        }

        #endregion

        #region Abstract search

        private FPNavMeshBinaryHeap _open;
        private FP64[] _gScore;
        private int[] _cameFrom;
        private int[] _searchStamp;
        private bool[] _closed;
        private int _searchGeneration;

        /// <summary>
        /// The first hop of the cheapest node route from <paramref name="startNode"/> to
        /// <paramref name="goalNode"/>, and the edge that takes it. Only the first hop is returned:
        /// the agent walks one leg and asks again, so holding the rest would be state that has to
        /// survive a rollback for no gain.
        /// <para>No iteration budget is needed — A* over a graph closes each node at most once, so
        /// the search is bounded by the node count by construction. That is the whole point of
        /// planning up here.</para>
        /// </summary>
        /// <param name="fromXZ">Where the agent actually stands. Replaces the start node's centre
        /// for every edge leaving it — see the remarks on endpoint insertion below.</param>
        /// <param name="toXZ">The actual destination. Replaces the goal node's centre for every
        /// edge arriving at it, and anchors the heuristic.</param>
        internal bool TryFindFirstHop(
            int startNode, int goalNode, FPVector2 fromXZ, FPVector2 toXZ,
            out int firstEdge, out int secondEdge)
        {
            firstEdge = -1;
            secondEdge = -1;
            if (startNode < 0 || goalNode < 0 || startNode >= NodeCount || goalNode >= NodeCount)
                return false;
            if (startNode == goalNode)
                return false;

            EnsureSearchCapacity(NodeCount);
            _searchGeneration++;
            _open.Clear();

            Touch(startNode);
            _gScore[startNode] = FP64.Zero;
            _open.Push(startNode, FPVector2.Distance(fromXZ, toXZ));

            while (_open.Count > 0)
            {
                int cur = _open.Pop();
                if (cur == goalNode)
                    return Unwind(startNode, goalNode, out firstEdge, out secondEdge);

                _closed[cur] = true;
                EdgeRange(cur, out int start, out int end);
                for (int e = start; e < end; e++)
                {
                    int nb = _edgeTarget[e];
                    Touch(nb);
                    if (_closed[nb]) continue;

                    FP64 tentative = _gScore[cur] + EdgeCostFrom(cur, e, startNode, goalNode, fromXZ, toXZ);
                    if (tentative >= _gScore[nb]) continue;

                    _gScore[nb] = tentative;
                    _cameFrom[nb] = e;               // the EDGE we arrived by, not the node
                    FP64 f = tentative + (nb == goalNode
                        ? FP64.Zero
                        : FPVector2.Distance(_nodeCenter[nb], toXZ));
                    if (_open.Contains(nb)) _open.DecreaseKey(nb, f);
                    else _open.Push(nb, f);
                }
            }
            return false;
        }

        /// <summary>
        /// The cost of crossing one edge, with the two ENDPOINTS substituted for the node centres
        /// they stand in for. Everywhere else this is the precomputed <c>_edgeCost</c>.
        ///
        /// <para><b>Why the endpoints have to enter the search.</b> A precomputed edge cost is
        /// centre-to-portal-to-centre, which is the right stand-in for a node being passed THROUGH
        /// and the wrong one for the two nodes the journey starts and ends in. An agent that just
        /// changed legs stands on its node's boundary, not at its centre, so the portal that is
        /// cheapest from the centre can sit well off the line it actually has to walk — and the
        /// same is true at the far end, where the destination is rarely the goal node's centre.
        /// Both show up as a leg that leans away from the straight line and then leans back.</para>
        ///
        /// <para>This is HPA*'s temporary-node insertion, done without allocating a node: rather
        /// than splicing start and goal into the graph and connecting them to every transition,
        /// the two costs are substituted while the search runs. Same answer, no mutation, and the
        /// graph stays shareable across agents planning at the same time.</para>
        ///
        /// <para><b>The graph itself does not change</b>, so nothing here touches
        /// <see cref="Checksum"/> — which is exactly why the rule needs a version of its own
        /// (see <c>PLAN_RULE_REVISION</c>).</para>
        /// </summary>
        private FP64 EdgeCostFrom(
            int fromNode, int edge, int startNode, int goalNode, FPVector2 fromXZ, FPVector2 toXZ)
        {
            int toNode = _edgeTarget[edge];
            bool leavingStart = fromNode == startNode;
            bool enteringGoal = toNode == goalNode;
            if (!leavingStart && !enteringGoal)
                return _edgeCost[edge];

            FPVector2 portalMid = _edgePortal[edge].ToXZ();
            FPVector2 a = leavingStart ? fromXZ : _nodeCenter[fromNode];
            FPVector2 b = enteringGoal ? toXZ : _nodeCenter[toNode];
            return (FPVector2.Distance(a, portalMid) + FPVector2.Distance(portalMid, b))
                 * _nodeFold[toNode];
        }

        /// <summary>
        /// Walks the cameFrom chain back and hands out the first TWO edges of the route.
        ///
        /// <para>The second one costs nothing: the walk already visits every edge from the goal back
        /// to the start, so remembering the previous one as it goes is free. It is what lets a leg
        /// aim at the portal AFTER the one it is crossing, which is the only direction that does not
        /// leave the next node — the destination can point clean out of it.</para>
        ///
        /// <para><paramref name="secondEdge"/> is -1 when the route is a single hop, i.e. the next
        /// node is the goal node; the caller aims at the destination then.</para>
        /// </summary>
        private bool Unwind(int startNode, int goalNode, out int firstEdge, out int secondEdge)
        {
            firstEdge = -1;
            secondEdge = -1;
            int node = goalNode;
            while (node != startNode)
            {
                int edge = _cameFrom[node];
                if (edge < 0) return false;          // chain broken — refuse rather than guess
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
            _cameFrom[node] = -1;
            _closed[node] = false;
        }

        private FP64 Heuristic(int node, int goalNode)
            => FPVector2.Distance(_nodeCenter[node], _nodeCenter[goalNode]);

        private void EnsureSearchCapacity(int nodeCount)
        {
            if (_gScore != null && _gScore.Length >= nodeCount) return;
            _gScore = new FP64[nodeCount];
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
            if (_nodeCenter == null || _nodeCenter.Length < nodeCount)
            {
                _nodeCenter = new FPVector2[nodeCount];
                _nodeTriangleCount = new int[nodeCount];
                _nodeFold = new FP64[nodeCount];
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
                _edgeCost = new FP64[edgeCount];
                _edgePortal = new FPVector3[edgeCount];
                _edgePortalA = new FPVector3[edgeCount];
                _edgePortalB = new FPVector3[edgeCount];
            }
        }

        #endregion
    }
}
