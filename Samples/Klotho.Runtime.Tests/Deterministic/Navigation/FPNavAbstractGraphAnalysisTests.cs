using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using NUnit.Framework;

using xpTURN.Klotho.Deterministic.Math;

namespace xpTURN.Klotho.Deterministic.Navigation.Tests
{
    /// <summary>
    /// The measurements the abstract-graph design was settled on, kept so they can be taken again.
    /// Nothing here touches production code: it derives a candidate cluster/portal graph test-side,
    /// over the real baked assets, and reports the numbers rather than asserting them.
    ///
    /// Two questions, in order of what they settle:
    ///
    /// 1. <b>Can a leg fit the corridor buffer?</b> The plan's headline is that planning in legs
    ///    removes the 128-triangle ceiling. That only holds if a within-cluster path is shorter
    ///    than the cap, and nothing in the mesh guarantees it — the broadphase cell size is a free
    ///    bake parameter. If the mesh's own grid gives clusters that are too big, reusing that grid
    ///    (D-2 option a) cannot work and the derivation has to build its own partition (option b).
    /// 2. <b>What does deriving cost?</b> The graph is rebuilt on every navmesh swap, and the
    ///    runtime rebake already runs on a sliced budget, so a derivation that dwarfs it is a
    ///    non-starter (D-7).
    ///
    /// Run explicitly, in Release:
    ///   dotnet test -c Release --filter FullyQualifiedName~FPNavAbstractGraphAnalysisTests
    /// </summary>
    [TestFixture]
    [Explicit("P0a measurement — run in Release with an explicit filter")]
    public class FPNavAbstractGraphAnalysisTests
    {
        private static string RepoRoot()
        {
            var dir = new DirectoryInfo(AppContext.BaseDirectory);
            while (dir != null && !Directory.Exists(Path.Combine(dir.FullName, "com.xpturn.klotho")))
                dir = dir.Parent;
            Assert.IsNotNull(dir, "repo root not found from test base directory");
            return dir.FullName;
        }

        private static readonly string[] Assets =
        {
            "Samples/Brawler/Assets/NavMesh/Data/Field.NavMeshData.bytes",
            "Samples/Brawler/Assets/Brawler/Data/Stage01.NavMeshData.bytes",
            "Samples/Brawler/Assets/Brawler/Data/Stage02.NavMeshData.bytes",
            "Samples/Brawler/Assets/NavMesh/Data/8_heightmesh.NavMeshData.bytes",
        };

        #region Candidate derivation (test-side, D-2 option a: reuse the mesh grid)

        /// <summary>
        /// A node is a connected component *within* one broadphase cell — the standard HPA*
        /// construction. A cell is not a node on its own: one cell can hold both sides of a wall or
        /// two floors, and merging those would invent a crossing that does not exist.
        /// </summary>
        private sealed class Derived
        {
            public int NodeCount;
            public int PortalCount;
            public int MaxNodeTriangles;      // upper bound on a leg's corridor length
            public int P99NodeTriangles;
            public int MedianNodeTriangles;
            public int CellsWithMultipleComponents;
            public int[] NodeOfTriangle;      // triangle -> node id (-1 = none)
        }

        private static Derived Derive(FPNavMesh mesh)
        {
            int triCount = mesh.Triangles.Length;
            var nodeOf = new int[triCount];
            for (int i = 0; i < triCount; i++) nodeOf[i] = -1;

            // Cell of a triangle: the cell its centre falls in. Using one cell per triangle (rather
            // than the mesh's cell->triangle lists, which record overlap) keeps nodes a partition,
            // which is what "an agent is in exactly one cluster" requires.
            var cellOf = new int[triCount];
            for (int t = 0; t < triCount; t++)
            {
                mesh.GetCellCoords(mesh.Triangles[t].centerXZ, out int col, out int row);
                if (col < 0) col = 0; if (col >= mesh.GridWidth) col = mesh.GridWidth - 1;
                if (row < 0) row = 0; if (row >= mesh.GridHeight) row = mesh.GridHeight - 1;
                cellOf[t] = row * mesh.GridWidth + col;
            }

            var sizes = new List<int>();
            var componentsPerCell = new Dictionary<int, int>();
            var stack = new Stack<int>();
            int nodeId = 0;

            for (int seed = 0; seed < triCount; seed++)
            {
                if (nodeOf[seed] >= 0 || mesh.Triangles[seed].isBlocked)
                    continue;

                int cell = cellOf[seed];
                int size = 0;
                stack.Push(seed);
                nodeOf[seed] = nodeId;
                while (stack.Count > 0)
                {
                    int cur = stack.Pop();
                    size++;
                    for (int e = 0; e < 3; e++)
                    {
                        int nb = mesh.Triangles[cur].GetNeighbor(e);
                        if (nb < 0 || nodeOf[nb] >= 0) continue;
                        if (mesh.Triangles[nb].isBlocked) continue;
                        if (cellOf[nb] != cell) continue;      // flood fill stays inside the cell
                        nodeOf[nb] = nodeId;
                        stack.Push(nb);
                    }
                }
                sizes.Add(size);
                componentsPerCell.TryGetValue(cell, out int c);
                componentsPerCell[cell] = c + 1;
                nodeId++;
            }

            // Portals: adjacencies that cross a node boundary, counted once per pair.
            int portals = 0;
            for (int t = 0; t < triCount; t++)
            {
                if (nodeOf[t] < 0) continue;
                for (int e = 0; e < 3; e++)
                {
                    int nb = mesh.Triangles[t].GetNeighbor(e);
                    if (nb < 0 || nb < t) continue;
                    if (nodeOf[nb] < 0) continue;
                    if (nodeOf[nb] != nodeOf[t]) portals++;
                }
            }

            sizes.Sort();
            int multi = 0;
            foreach (var kv in componentsPerCell) if (kv.Value > 1) multi++;

            return new Derived
            {
                NodeCount = nodeId,
                PortalCount = portals,
                MaxNodeTriangles = sizes.Count > 0 ? sizes[sizes.Count - 1] : 0,
                P99NodeTriangles = sizes.Count > 0 ? sizes[(int)(sizes.Count * 0.99)] : 0,
                MedianNodeTriangles = sizes.Count > 0 ? sizes[sizes.Count / 2] : 0,
                CellsWithMultipleComponents = multi,
                NodeOfTriangle = nodeOf,
            };
        }

        #endregion

        /// <summary>
        /// How many legs does a real path become? This is the number D-4 and D-6 are multiplied by:
        /// every node boundary is a portal the agent commits to, and every commit is both a place
        /// the path can bend away from the optimum and a place the agent would brake if the three
        /// call sites still read <c>PathTarget</c>. Measured by walking the flat A* corridor between
        /// far-apart triangles and counting node changes along it.
        /// </summary>
        private static void ReportLegCount(FPNavMesh mesh, Derived d, string name)
        {
            var query = new FPNavMeshQuery(mesh, null);
            var pathfinder = new FPNavMeshPathfinder(mesh, query, null);

            // Far-apart walkable pairs: first and last non-blocked triangles by index, plus the two
            // extremes of the bounds diagonal. Index order is deterministic and asset-defined.
            int first = -1, last = -1;
            for (int t = 0; t < mesh.Triangles.Length; t++)
            {
                if (mesh.Triangles[t].isBlocked) continue;
                if (first < 0) first = t;
                last = t;
            }
            if (first < 0 || first == last) { TestContext.Out.WriteLine("  legs          n/a"); return; }

            FPVector3 Center(int tri)
            {
                var c = mesh.Triangles[tri].centerXZ;
                return new FPVector3(c.x, mesh.Vertices[mesh.Triangles[tri].v0].y, c.y);
            }

            bool found = pathfinder.FindPath(Center(first), Center(last),
                FPNavAgentSystem.DEFAULT_AREA_MASK, out int[] corridor, out int len);
            if (!found)
            {
                TestContext.Out.WriteLine(
                    $"  legs          flat A* FAILED between the extremes " +
                    $"(exhausted {pathfinder.DebugIterationExhaustedCount}, " +
                    $"clamped {pathfinder.DebugCorridorTruncatedCount}) — the very case this plan targets");
                return;
            }

            int legs = 0, prev = -1, longestRun = 0, run = 0;
            for (int i = 0; i < len; i++)
            {
                int node = d.NodeOfTriangle[corridor[i]];
                if (node != prev) { legs++; prev = node; longestRun = System.Math.Max(longestRun, run); run = 1; }
                else run++;
            }
            longestRun = System.Math.Max(longestRun, run);

            TestContext.Out.WriteLine(
                $"  legs          {legs} legs over a {len}-triangle corridor" +
                (pathfinder.DebugCorridorTruncatedCount > 0 ? " (corridor was CLAMPED — true path is longer)" : "") +
                $"; longest single leg {longestRun} tris");
        }

        /// <summary>
        /// P0a2 — where does the cluster lattice get anchored? The plan wants a fixed (world)
        /// origin rather than one derived from the mesh bounds, because a moving origin renumbers
        /// every node and invalidates every leg in flight. The mesh's own broadphase grid is built
        /// the way we are told NOT to build ours: ComputeBoundsXZ feeds BuildSpatialGrid, which
        /// produces gridOrigin, and a rebake inherits the cell SIZE while recomputing the origin.
        /// This measures whether that actually bites — rebake with a building, then compare bounds,
        /// origin and grid dimensions against the base.
        /// </summary>
        /// <summary>
        /// P1 — the real derivation (FPNavAbstractGraph), on the shipped assets, at a range of
        /// lattice sizes. Two questions the plan needs answered before P2:
        /// what cell size makes a leg worth planning, and does the measured node diameter actually
        /// stay under the corridor cap (V-2's guarantee, which P0a showed cannot be had from the
        /// mesh's own grid).
        /// </summary>
        [Test]
        public void P1_WhatLatticeSizeMakesALegWorthPlanning()
        {
            string root = RepoRoot();
            int cap = FPNavMeshPathfinder.MAX_CORRIDOR;
            TestContext.Out.WriteLine($"=== own lattice — nodes, edges and derivation cost by cell size (corridor cap {cap}) ===");
            TestContext.Out.WriteLine("");

            foreach (string rel in Assets)
            {
                string path = Path.Combine(root, rel);
                if (!File.Exists(path)) continue;
                FPNavMesh mesh = FPNavMeshSerializer.Deserialize(path);
                TestContext.Out.WriteLine($"--- {Path.GetFileNameWithoutExtension(rel)} ({mesh.Triangles.Length} tris) ---");

                foreach (double cell in new[] { 4.0, 8.0, 16.0, 32.0, 64.0 })
                {
                    var sw = Stopwatch.StartNew();
                    var g = new FPNavAbstractGraph(
                        mesh, FP64.FromDouble(cell), FPNavAbstractCostFold.Min,
                        FPNavAgentSystem.DEFAULT_AREA_MASK);
                    sw.Stop();

                    int largest = 0;
                    for (int n = 0; n < g.NodeCount; n++)
                        largest = System.Math.Max(largest, g.NodeTriangleCount(n));

                    TestContext.Out.WriteLine(
                        $"  cell {cell,5:F1}   {g.NodeCount,6} nodes, {g.EdgeCount,6} edges   " +
                        $"largest node {largest,5} tris, diameter {g.MaxNodeDiameter,4}" +
                        (g.MaxNodeDiameter >= cap ? "   <-- OVER the cap" : "") +
                        $"   {sw.Elapsed.TotalMilliseconds,7:F2} ms" +
                        $"   pair table {g.PairEntryCount * 8 / 1024,5} KB, reverse unmatched {g.DebugReverseUnmatched}");
                }
                TestContext.Out.WriteLine("");
            }

            // Two rows the real assets cannot give. A synthetic open field, whose convex nodes never
            // run the walled-pair search, is the floor a derivation can have at this size; and the
            // Field through the install ladder is what a boot actually pays — every candidate the
            // ladder tries is a derivation, and until the ladder stopped building the pair table
            // for candidates it was going to reject, that was two full ones on the Field.
            {
                var synth = NavAgentTestHelper.CreateOpenFieldNavMesh(96);
                TestContext.Out.WriteLine($"--- synthetic open field 96x96 ({synth.Triangles.Length} tris) ---");
                foreach (double cell in new[] { 16.0, 32.0 })
                {
                    var sw = Stopwatch.StartNew();
                    var g = new FPNavAbstractGraph(synth, FP64.FromDouble(cell), FPNavAbstractCostFold.Min,
                        FPNavAgentSystem.DEFAULT_AREA_MASK);
                    sw.Stop();
                    TestContext.Out.WriteLine(
                        $"  cell {cell,5:F1}   {g.NodeCount,6} nodes, {g.EdgeCount,6} edges   {sw.Elapsed.TotalMilliseconds,7:F2} ms" +
                        $"   pair table {g.PairEntryCount * 8 / 1024,5} KB, mid-distance fills {g.DebugMidDistFills}, walk steps {g.DebugWalkSteps}");
                }
                TestContext.Out.WriteLine("");

                string fieldPath = Path.Combine(root, Assets[0]);
                if (File.Exists(fieldPath))
                {
                    FPNavMesh field = FPNavMeshSerializer.Deserialize(fieldPath);
                    var sw = Stopwatch.StartNew();
                    var system = NavAgentTestHelper.CreateSystem(field, null);   // default tuning: the ladder runs in the constructor
                    sw.Stop();
                    TestContext.Out.WriteLine(
                        $"--- Field through the install ladder: {sw.Elapsed.TotalMilliseconds:F2} ms wall clock, " +
                        $"installed cell {system.AbstractGraph?.CellSize.ToDouble():F1} ---");
                }
            }
        }

        [Test]
        public void P0a2_DoesARebakeMoveTheGridOrigin()
        {
            string root = RepoRoot();
            TestContext.Out.WriteLine("=== is the mesh's grid anchor stable across a rebake? ===");
            TestContext.Out.WriteLine("");

            foreach (string rel in Assets)
            {
                string path = Path.Combine(root, rel);
                if (!File.Exists(path)) continue;
                FPNavMesh baseMesh = FPNavMeshSerializer.Deserialize(path);
                string name = Path.GetFileNameWithoutExtension(rel);

                var b = baseMesh.BoundsXZ;
                TestContext.Out.WriteLine($"--- {name} ---");
                TestContext.Out.WriteLine(
                    $"  base    bounds ({b.min.x.ToDouble():F2},{b.min.y.ToDouble():F2})-" +
                    $"({b.max.x.ToDouble():F2},{b.max.y.ToDouble():F2})  " +
                    $"origin ({baseMesh.GridOrigin.x.ToDouble():F2},{baseMesh.GridOrigin.y.ToDouble():F2})  " +
                    $"grid {baseMesh.GridWidth}x{baseMesh.GridHeight}");

                FP64 W(double v) => FP64.FromDouble(v);
                double midX = (b.min.x.ToDouble() + b.max.x.ToDouble()) / 2;
                double midZ = (b.min.y.ToDouble() + b.max.y.ToDouble()) / 2;
                var cases = new (string label, FPBuildingRect rect)[]
                {
                    ("interior", new FPBuildingRect(W(midX - 2), W(midZ - 2), W(midX + 2), W(midZ + 2), FP64.Zero)),
                    ("at the SW corner", new FPBuildingRect(
                        W(b.min.x.ToDouble()), W(b.min.y.ToDouble()),
                        W(b.min.x.ToDouble() + 4), W(b.min.y.ToDouble() + 4), FP64.Zero)),
                };

                var policies = new (string name, FPBuildingPlacementRules rules)[]
                {
                    ("Reject", new FPBuildingPlacementRules(true, FPBoundaryPlacementPolicy.Reject)),
                    ("Touch", new FPBuildingPlacementRules(true, FPBoundaryPlacementPolicy.Touch)),
                    ("ClipOverlap", new FPBuildingPlacementRules(true, FPBoundaryPlacementPolicy.ClipOverlap)),
                };

                foreach (var (label, rect) in cases)
                foreach (var (policyName, rules) in policies)
                {
                    FPNavMesh re;
                    try { re = FPNavMeshRebaker.Rebake(baseMesh, new[] { rect }, null, rules); }
                    catch (Exception ex)
                    {
                        TestContext.Out.WriteLine($"  {label,-16} {policyName,-11} refused: {ex.Message}");
                        continue;
                    }

                    var rb = re.BoundsXZ;
                    bool boundsSame = rb.min.x == b.min.x && rb.min.y == b.min.y
                                   && rb.max.x == b.max.x && rb.max.y == b.max.y;
                    bool originSame = re.GridOrigin.x == baseMesh.GridOrigin.x
                                   && re.GridOrigin.y == baseMesh.GridOrigin.y;
                    bool dimsSame = re.GridWidth == baseMesh.GridWidth && re.GridHeight == baseMesh.GridHeight;

                    TestContext.Out.WriteLine(
                        $"  {label,-16} {policyName,-11} bounds {(boundsSame ? "same" : "MOVED")}, " +
                        $"origin {(originSame ? "same" : "MOVED")}, " +
                        $"grid {(dimsSame ? "same" : $"CHANGED -> {re.GridWidth}x{re.GridHeight}")}" +
                        (originSame && dimsSame ? "" : "   <-- a bounds-anchored lattice would renumber here"));
                }
                TestContext.Out.WriteLine("");
            }
        }

        [Test]
        public void P0a_CanALegFitTheCorridorBuffer_AndWhatDoesDerivingCost()
        {
            string root = RepoRoot();
            int cap = FPNavMeshPathfinder.MAX_CORRIDOR;

            TestContext.Out.WriteLine(
                $"=== clusters taken from the mesh's own broadphase grid ===");
            TestContext.Out.WriteLine(
                $"corridor cap = {cap}; a node larger than that CANNOT guarantee V-2");
            TestContext.Out.WriteLine("");

            foreach (string rel in Assets)
            {
                string path = Path.Combine(root, rel);
                if (!File.Exists(path)) { TestContext.Out.WriteLine($"{rel}: MISSING"); continue; }

                FPNavMesh mesh = FPNavMeshSerializer.Deserialize(path);
                string name = Path.GetFileNameWithoutExtension(rel);

                // Warmup, then steady-state derivation cost (same discipline as the perf fixtures).
                for (int i = 0; i < 8; i++) Derive(mesh);
                var sw = Stopwatch.StartNew();
                Derived d = null;
                const int iters = 5;
                for (int i = 0; i < iters; i++) d = Derive(mesh);
                sw.Stop();
                double ms = sw.Elapsed.TotalMilliseconds / iters;

                TestContext.Out.WriteLine($"--- {name} ---");
                TestContext.Out.WriteLine(
                    $"  mesh          {mesh.Triangles.Length,7} tris, grid {mesh.GridWidth}x{mesh.GridHeight}" +
                    $" @ cell {mesh.GridCellSize.ToDouble():F2}");
                TestContext.Out.WriteLine(
                    $"  derived       {d.NodeCount,7} nodes, {d.PortalCount} portals," +
                    $" {d.CellsWithMultipleComponents} cells split by connectivity");
                TestContext.Out.WriteLine(
                    $"  node size     median {d.MedianNodeTriangles}, p99 {d.P99NodeTriangles}, " +
                    $"max {d.MaxNodeTriangles} tris" +
                    (d.MaxNodeTriangles > cap ? $"   <-- OVER the {cap} cap" : "   (within cap)"));
                TestContext.Out.WriteLine($"  derivation    {ms,7:F3} ms");
                ReportLegCount(mesh, d, name);
                TestContext.Out.WriteLine("");
            }
        }

        #region P0 — the real asset, walked and analysed (Plan-MidHopEntryPoint)

        private static FPVector3 Centroid3(FPNavMesh mesh, int tri)
        {
            var t = mesh.Triangles[tri];
            return (mesh.Vertices[t.v0] + mesh.Vertices[t.v1] + mesh.Vertices[t.v2]) * (FP64.One / FP64.FromInt(3));
        }

        private struct FieldWalk
        {
            public string Status; public int Ticks; public double Distance; public int Legs;
            public double MidDistance, MidStraight;
            public int Exhausted, LegResolveFailed, CorridorTruncated, Partial;
            public string Graph;
        }

        /// <summary>
        /// One agent on the shipped asset. <paramref name="tuning"/> decides flat or auto-legs;
        /// <paramref name="cell"/> &gt; 0 installs an explicit graph over whatever the tuning did.
        /// Cut at every leg hand-off so the middle of the route can be read on its own (V-M0's
        /// measure, on the real mesh — V-M1).
        /// </summary>
        private static FieldWalk WalkField(FPNavMesh mesh, FPVector3 start, FPVector3 goal,
            FPNavTuning tuning, double cell)
        {
            var system = NavAgentTestHelper.CreateSystem(mesh, null, tuning, out var pathfinder);
            if (cell > 0)
                system.SetAbstractGraph(new FPNavAbstractGraph(
                    mesh, FP64.FromDouble(cell), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK));
            var g = system.AbstractGraph;
            var w = new FieldWalk { Graph = g == null ? "flat" : $"cell {g.CellSize.ToDouble():F0} ({g.NodeCount} nodes)" };

            var query = new FPNavMeshQuery(mesh, null);
            int tri = query.FindTriangle(start.ToXZ(), start.y);
            Assert.GreaterOrEqual(tri, 0, "start is on the mesh");
            var frame = NavAgentTestHelper.CreateFrameWithAgent(start, tri, out var entity, out var entities);
            ref var nav0 = ref frame.Get<NavAgentComponent>(entity);
            NavAgentComponent.SetDestination(ref nav0, goal);

            FPVector3 prev = start; double travelled = 0; int legsSeen = 0;
            FPVector3 firstHandoff = start, lastHandoff = start; double atFirst = 0, atLast = 0;
            const int budget = 30000;
            for (int tick = 1; tick <= budget; tick++)
            {
                system.Update(ref frame, entities, 1, tick, NavAgentTestHelper.DT);
                ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
                travelled += FPVector2.Distance(prev.ToXZ(), nav.Position.ToXZ()).ToDouble();
                prev = nav.Position;
                if (system.DebugLegAdvanceCount > legsSeen)
                {
                    legsSeen = system.DebugLegAdvanceCount;
                    if (legsSeen == 1) { firstHandoff = nav.Position; atFirst = travelled; }
                    lastHandoff = nav.Position; atLast = travelled;
                }
                bool done = nav.Status == (byte)FPNavAgentStatus.Arrived || nav.Status == (byte)FPNavAgentStatus.PathFailed;
                if (done || tick == budget)
                {
                    w.Status = nav.Status == (byte)FPNavAgentStatus.Arrived ? "Arrived"
                        : nav.Status == (byte)FPNavAgentStatus.PathFailed ? "PathFailed" : "timeout";
                    w.Ticks = tick; w.Distance = travelled; w.Legs = system.DebugLegAdvanceCount;
                    if (w.Legs >= 2)
                    {
                        w.MidDistance = atLast - atFirst;
                        w.MidStraight = FPVector2.Distance(firstHandoff.ToXZ(), lastHandoff.ToXZ()).ToDouble();
                    }
                    break;
                }
            }
            w.Exhausted = pathfinder.DebugIterationExhaustedCount;
            w.LegResolveFailed = system.DebugLegResolveFailedCount;
            w.CorridorTruncated = pathfinder.DebugCorridorTruncatedCount + system.DebugCorridorCopyTruncatedCount;
            w.Partial = pathfinder.DebugPartialPathCount;
            return w;
        }

        /// <summary>
        /// V-M1 — the real asset. Two routes on Field: the one the visualizer reported as failing
        /// flat (the parent plan's §1e), and the long diagonal. Flat is attempted with a large budget
        /// and NO graph, and its number only counts when nothing was clamped or partial — beyond the
        /// corridor buffer a flat walk is a chain of clamped partials, not an optimum. The honest
        /// reading is legs / straight, before and after.
        /// </summary>
        [Test]
        public void P0_FieldWalk_LegsAgainstStraight()
        {
            string path = Path.Combine(RepoRoot(), Assets[0]);
            if (!File.Exists(path)) { TestContext.Out.WriteLine($"{Assets[0]}: MISSING"); return; }
            FPNavMesh mesh = FPNavMeshSerializer.Deserialize(path);

            // Route 1: the visualizer's failing pair (38.7 units). Route 2: the long diagonal, from
            // the walkable triangle with the smallest x+z centroid to the one with the largest.
            var r1s = new FPVector3(FP64.FromDouble(-24.14), FP64.Zero, FP64.FromDouble(1.86));
            var r1g = new FPVector3(FP64.FromDouble(14.46), FP64.Zero, FP64.FromDouble(-0.48));
            var probe = new FPNavAbstractGraph(mesh, FP64.FromInt(32), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK);
            int lo = -1, hi = -1; FP64 loV = FP64.MaxValue, hiV = FP64.MinValue;
            for (int t = 0; t < mesh.Triangles.Length; t++)
            {
                if (probe.NodeOf(t) < 0) continue;
                var c = Centroid3(mesh, t).ToXZ(); FP64 v = c.x + c.y;
                if (v < loV) { loV = v; lo = t; }
                if (v > hiV) { hiV = v; hi = t; }
            }
            var r2s = Centroid3(mesh, lo); var r2g = Centroid3(mesh, hi);

            var flatBig = new FPNavTuning(maxIterations: 1 << 20, autoInstallAbstractGraph: false);
            TestContext.Out.WriteLine("=== V-M1 — Field: legs / straight, middle ratio, and whether flat can even be measured ===");
            foreach (var (label, s, gl) in new[] { ("route 1 (visualizer pair)", r1s, r1g), ("route 2 (diagonal)", r2s, r2g) })
            {
                double straight = FPVector2.Distance(s.ToXZ(), gl.ToXZ()).ToDouble();
                TestContext.Out.WriteLine($"--- {label}: ({s.x.ToDouble():F2}, {s.z.ToDouble():F2}) -> ({gl.x.ToDouble():F2}, {gl.z.ToDouble():F2})  straight {straight:F1} ---");
                foreach (var (name, tuning, cell) in new[]
                {
                    ("flat, big budget", flatBig, 0.0),
                    ("auto (default)  ", FPNavTuning.Default, 0.0),
                    ("cell 16         ", NavAgentTestHelper.NoAutoGraph, 16.0),
                    ("cell 32         ", NavAgentTestHelper.NoAutoGraph, 32.0),
                })
                {
                    var w = WalkField(mesh, s, gl, tuning, cell);
                    string mid = w.Legs >= 2 && w.MidStraight > 0 ? $"middle {w.MidDistance / w.MidStraight,6:F3}x over {w.MidStraight,5:F1}" : "middle    n/a";
                    string valid = w.CorridorTruncated == 0 && w.Partial == 0 && w.Exhausted == 0 ? "" : "  <-- clamped/partial/exhausted: not a baseline";
                    TestContext.Out.WriteLine(
                        $"  {name} [{w.Graph,-22}] {w.Status,-10} {w.Ticks,6} ticks  total {w.Distance / straight,6:F3}x  {mid}  " +
                        $"legs {w.Legs,3}  exhausted {w.Exhausted} truncated {w.CorridorTruncated} partial {w.Partial} legFail {w.LegResolveFailed}{valid}");
                }
                TestContext.Out.WriteLine("");
            }
        }

        /// <summary>
        /// V-M11 — how often a straight line between two portals of one node is not walkable. For
        /// every node and every pair of its portals, the real A* + funnel path between the portal
        /// midpoints is compared with the Euclidean distance. A ratio above ~1 means a wall between
        /// them: exactly the pairs a midpoint-to-midpoint cost model under-prices, and the only pairs
        /// an intra-node distance table (D-P4) would need. The unconstrained path is a lower bound
        /// on the in-node distance, so a path that leaves the node is counted separately.
        /// </summary>
        [Test]
        public void P0_PortalPairAnalysis_WallsBetweenPortals()
        {
            string path = Path.Combine(RepoRoot(), Assets[0]);
            if (!File.Exists(path)) { TestContext.Out.WriteLine($"{Assets[0]}: MISSING"); return; }
            FPNavMesh mesh = FPNavMeshSerializer.Deserialize(path);
            var tuning = new FPNavTuning(maxIterations: 1 << 16, autoInstallAbstractGraph: false);
            var query = new FPNavMeshQuery(mesh, null, tuning);
            var pathfinder = new FPNavMeshPathfinder(mesh, query, null, tuning);
            var funnel = new FPNavMeshFunnel(mesh, query, null, tuning);

            TestContext.Out.WriteLine("=== V-M11 — Field: portal pairs per node, A*+funnel length / Euclid between midpoints ===");
            foreach (double cell in new[] { 16.0, 32.0 })
            {
                var g = new FPNavAbstractGraph(mesh, FP64.FromDouble(cell), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK);
                long pairs = 0, over105 = 0, over120 = 0, leftNode = 0, failed = 0;
                int nodesWithWall = 0; double maxRatio = 0; int maxNode = -1;
                var worst = new List<(double ratio, int node)>();
                for (int n = 0; n < g.NodeCount; n++)
                {
                    g.EdgeRange(n, out int es, out int ee);
                    bool wall = false;
                    for (int i = es; i < ee; i++)
                    for (int j = i + 1; j < ee; j++)
                    {
                        FPVector3 a = g.EdgePortal(i), b = g.EdgePortal(j);
                        double euclid = FPVector2.Distance(a.ToXZ(), b.ToXZ()).ToDouble();
                        if (euclid <= 0.0) continue;
                        pairs++;
                        if (!pathfinder.FindPath(a, b, FPNavAgentSystem.DEFAULT_AREA_MASK, FP64.Zero,
                                out int[] corridor, out int len, out bool partial, out _) || partial)
                        { failed++; continue; }
                        bool inside = true;
                        for (int k = 0; k < len; k++) if (g.NodeOf(corridor[k]) != n) { inside = false; break; }
                        int corners = funnel.FindCorners(corridor, len, a, b, FPNavMeshFunnel.MAX_WAYPOINTS);
                        double length = 0; FPVector3 p = a;
                        for (int k = 0; k < corners; k++) { length += FPVector2.Distance(p.ToXZ(), funnel.Corners[k].ToXZ()).ToDouble(); p = funnel.Corners[k]; }
                        length += FPVector2.Distance(p.ToXZ(), b.ToXZ()).ToDouble();
                        double ratio = length / euclid;
                        if (!inside) leftNode++;
                        if (ratio > 1.05) { over105++; wall = true; }
                        if (ratio > 1.20) over120++;
                        if (ratio > maxRatio) { maxRatio = ratio; maxNode = n; }
                        if (ratio > 1.05) worst.Add((ratio, n));
                    }
                    if (wall) nodesWithWall++;
                }
                worst.Sort((x, y) => y.ratio.CompareTo(x.ratio));
                TestContext.Out.WriteLine(
                    $"  cell {cell,3:F0}: {g.NodeCount} nodes, {pairs} pairs | ratio > 1.05: {over105} ({100.0 * over105 / System.Math.Max(1, pairs):F2}%)  > 1.20: {over120}  " +
                    $"max {maxRatio:F3} (node {maxNode}) | nodes with any wall pair {nodesWithWall} ({100.0 * nodesWithWall / g.NodeCount:F1}%) | " +
                    $"paths that left the node {leftNode}  failed/partial {failed}");
                for (int k = 0; k < System.Math.Min(5, worst.Count); k++)
                    TestContext.Out.WriteLine($"      worst: node {worst[k].node} ratio {worst[k].ratio:F3}");
            }
        }

        #endregion

    }
}
