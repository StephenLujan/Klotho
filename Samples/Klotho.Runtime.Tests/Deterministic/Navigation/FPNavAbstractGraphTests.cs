using System;
using System.Collections.Generic;
using NUnit.Framework;

using xpTURN.Klotho.Deterministic.Math;
using xpTURN.Klotho.Helper.Tests;
using xpTURN.Klotho.Logging;

namespace xpTURN.Klotho.Deterministic.Navigation.Tests
{
    /// <summary>
    /// P1 contract for the abstract graph. Determinism comes first because everything else in the
    /// plan rests on it: two peers that derive different graphs plan different routes while their
    /// navmesh fingerprints agree, and nothing downstream would notice.
    /// </summary>
    [TestFixture]
    public class FPNavAbstractGraphTests
    {
        private static FPNavAbstractGraph Derive(
            FPNavMesh mesh, double cellSize = 8.0,
            FPNavAbstractCostFold fold = FPNavAbstractCostFold.Min)
            => new FPNavAbstractGraph(mesh, FP64.FromDouble(cellSize), fold,
                FPNavAgentSystem.DEFAULT_AREA_MASK);

        #region Determinism (V-5)

        [Test]
        public void DerivingTwice_ProducesTheSameChecksum()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            Assert.AreEqual(Derive(mesh).Checksum, Derive(mesh).Checksum);
        }

        [Test]
        public void ReboundInstance_MatchesAFreshOne()
        {
            // A swap reuses the instance, so a rebind must land exactly where a fresh derivation
            // would — otherwise a peer that rebaked mid-match diverges from one that joined after.
            var first = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var second = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out _);

            var reused = Derive(first);
            reused.Rebind(second);

            Assert.AreEqual(Derive(second).Checksum, reused.Checksum,
                "a rebound graph must equal a freshly derived one");
        }

        /// <summary>
        /// The Field asset, read from the repository the way the analysis harnesses do. A real,
        /// cluttered mesh is the only fixture on which the pair table's in-node walk-around
        /// search runs at scale (80% of its portal pairs have a wall between them), so its
        /// checksum is what pins that search's VALUES — the synthetic goldens barely enter it.
        /// </summary>
        private static FPNavMesh LoadFieldAsset()
        {
            var dir = new System.IO.DirectoryInfo(System.AppContext.BaseDirectory);
            while (dir != null && !System.IO.Directory.Exists(System.IO.Path.Combine(dir.FullName, "com.xpturn.klotho")))
                dir = dir.Parent;
            Assert.IsNotNull(dir, "repo root not found from the test base directory");
            return FPNavMeshSerializer.Deserialize(System.IO.Path.Combine(dir!.FullName,
                "Samples/Brawler/Assets/NavMesh/Data/Field.NavMeshData.bytes"));
        }

        /// <summary>
        /// Pins the derivation on the shipped Field asset at the two cell sizes the ladder and the
        /// harnesses use. These move for the same reasons the golden below does (a graph change, a
        /// PLAN_RULE_REVISION bump) and for one more — the asset being re-exported — and every move
        /// is re-pinned by the commit that caused it, with the reason. What they buy that the golden
        /// cannot: a change to the walled-pair search that alters a value by one raw unit somewhere
        /// in 37k entries shows up here and nowhere else in the automatic suite.
        /// </summary>
        [Test]
        public void TheFieldAsset_ChecksumsArePinned()
        {
            var mesh = LoadFieldAsset();
            var at16 = Derive(mesh, cellSize: 16.0);
            Assert.AreEqual(0xA5B1AD54CC8C46A9UL, at16.Checksum, "Field @ 16");
            Assert.AreEqual(0x0032FFE2F59ACC20UL, Derive(mesh, cellSize: 32.0).Checksum, "Field @ 32");

            // The walled-pair search stops at each target's pop because a popped key is minimal;
            // a re-push would mean the heap broke that, and the values above would be the next thing
            // to go. And the segment walk divides at most twice per edge it visits (~1.83 per step),
            // the second only for edges that survive the first — the ratio is the shape of that.
            Assert.AreEqual(0, at16.DebugPairRepushes, "the in-node search re-pushed a popped triangle");
            Assert.Greater(at16.DebugWalkSteps, 0, "fixture: the Field has walls between portals, so walks ran");
            Assert.LessOrEqual((double)at16.DebugWalkDivisions / at16.DebugWalkSteps, 1.3,
                "divisions per walk step (only candidate exits divide; the tests are made on numerator and denominator)");
        }

        /// <summary>
        /// The segment walk decides with <c>CompareQuotient</c> instead of dividing, and the only
        /// thing that makes that legitimate is that it agrees with FP64's division on every input
        /// — including the seam between its two regimes (a numerator of half a unit), the rounding
        /// ties of the bit loop, and thresholds one raw unit either side of the quotient. This
        /// checks the sign of <c>(a / b).RawValue - t</c> against the real division on random
        /// operands shaped like the walk's, plus those seams on purpose.
        /// </summary>
        [Test]
        public void CompareQuotient_AgreesWithFP64DivisionOnBothRegimes()
        {
            var rnd = new System.Random(20260908);
            long half = 1L << 31;
            int checkedCases = 0;
            for (int i = 0; i < 400_000; i++)
            {
                long a, b;
                switch (i % 5)
                {
                    case 0: a = rnd.NextInt64(-half + 1, half); break;                       // truncating regime
                    case 1: a = rnd.NextInt64(-(1L << 40), 1L << 40); break;                 // both regimes
                    case 2: a = (rnd.Next(2) == 0 ? -1 : 1) * (half + rnd.Next(-2, 3)); break;   // the seam
                    case 3: a = rnd.NextInt64(-(1L << 36), 1L << 36) & ~0xFFFFL; break;      // multiples: rounding ties
                    default: a = rnd.NextInt64(-(1L << 34), 1L << 34); break;
                }
                do b = rnd.NextInt64(-(1L << 36), 1L << 36); while (b == 0);
                if (i % 7 == 0) b = 1L << rnd.Next(1, 40);                                  // powers of two: exact and tied quotients
                FP64 q = FP64.FromRaw(a) / FP64.FromRaw(b);
                if (q == FP64.MaxValue || q == FP64.MinValue) continue;                      // saturated: outside the walk's window
                long r = q.RawValue;
                foreach (long t in new[] { r, r + 1, r - 1, 0L, FP64.One.RawValue, rnd.NextInt64(-(1L << 34), 1L << 34) })
                {
                    int expected = r > t ? 1 : (r < t ? -1 : 0);
                    Assert.AreEqual(expected, FPNavAbstractGraph.CompareQuotient(a, b, t),
                        $"a={a} b={b} t={t} (q.raw={r})");
                    checkedCases++;
                }
            }
            Assert.Greater(checkedCases, 1_000_000, "fixture: the sweep ran");
        }

        /// <summary>
        /// The 128-bit product behind <c>QuotientAbove</c> is the intrinsic on hosts that have it
        /// and 32-bit halves on the rest; the two must be the same integer or two hosts would
        /// derive two pair tables. The test runs where the intrinsic exists, so it is the halves
        /// that get checked.
        /// </summary>
        [Test]
        public void Mul64_MatchesTheIntrinsic()
        {
            var rnd = new System.Random(1128);
            for (int i = 0; i < 200_000; i++)
            {
                ulong a = (ulong)rnd.NextInt64(long.MinValue, long.MaxValue);
                ulong b = i % 3 == 0 ? (ulong)rnd.NextInt64(0, 1L << 36) : (ulong)rnd.NextInt64(long.MinValue, long.MaxValue);
                ulong expectedHi = System.Math.BigMul(a, b, out ulong expectedLo);
                ulong hi = FPInt128.MulUnsignedByHalves(a, b, out ulong lo);
                Assert.AreEqual(expectedHi, hi, $"hi of {a} x {b}");
                Assert.AreEqual(expectedLo, lo, $"lo of {a} x {b}");
            }
        }

        /// <summary>
        /// An open field's nodes are convex, so the straight line between any two of its portals
        /// stays inside the node and the walled-pair search never runs: the midpoint-distance
        /// table stays empty and the derivation pays nothing for having it. The walks DO run — the
        /// border nodes have walls — which is what makes "no fills" a statement about the search
        /// rather than about the fixture.
        /// </summary>
        [Test]
        public void AnOpenField_FillsNoMidpointDistances()
        {
            var graph = Derive(NavAgentTestHelper.CreateOpenFieldNavMesh(48), cellSize: 16.0);
            Assert.Greater(graph.DebugWalkSteps, 0, "fixture: the border nodes have walls, so walks ran");
            Assert.AreEqual(0, graph.DebugMidDistFills, "a convex node ran the walled-pair search");
        }

        /// <summary>
        /// The one synthetic fixture whose pair table takes the walled-pair path: an open field's
        /// nodes are convex, so no straight line between two of its portals ever touches a wall and
        /// the in-node search never runs there. The serpentine's does, on every corridor turn.
        /// </summary>
        [Test]
        public void TheSerpentine_ChecksumIsPinned()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out _);
            Assert.AreEqual(SERPENTINE_10x5_AT_16, Derive(mesh, cellSize: 16.0).Checksum, "serpentine 10x5 @ 16");
        }

        private const ulong SERPENTINE_10x5_AT_16 = 0x0034E0D20BB89446UL;   // pinned from the code before the derive-cost work (2026-09-08)

        [Test]
        public void ReboundInstance_MatchesAFreshOne_ThroughLargeSmallLarge()
        {
            // The pair-table scratch is triangle-sized and reused by generation stamp; rebinding a
            // big mesh to a small walled one and back is the sequence that would expose a stamp or
            // a lazily filled table leaking across derivations, because the small mesh's triangle
            // indices alias the large one's.
            var large = NavAgentTestHelper.CreateOpenFieldNavMesh(24);
            var small = NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out _);

            var reused = Derive(large, cellSize: 8.0);
            reused.Rebind(small);
            Assert.AreEqual(Derive(small, cellSize: 8.0).Checksum, reused.Checksum, "large -> small");
            reused.Rebind(large);
            Assert.AreEqual(Derive(large, cellSize: 8.0).Checksum, reused.Checksum, "small -> large again");
        }

        [Test]
        public void Rebind_AllocatesNothing_ForAMeshOfTheSameSize()
        {
            // The spare graph the swap path keeps exists so a re-derivation costs wall clock and
            // nothing else; capacity is grow-only, so a same-sized mesh must not allocate at all.
            var a = NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out _);
            var b = NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out _);
            var graph = Derive(a, cellSize: 8.0);
            graph.Rebind(b);      // warm: any lazily grown scratch grows here
            graph.Rebind(a);
            long before = System.GC.GetAllocatedBytesForCurrentThread();
            graph.Rebind(b);
            long allocated = System.GC.GetAllocatedBytesForCurrentThread() - before;
            TestContext.Out.WriteLine($"rebind allocated {allocated} B");
            Assert.AreEqual(0, allocated, "a same-sized rebind allocated");
        }

        [Test]
        public void TuningChanges_MoveTheChecksum()
        {
            // The cell size and the cost fold are simulation inputs, not preferences: if two peers
            // built with different ones, this is the value that says so.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            ulong baseline = Derive(mesh, cellSize: 8.0, fold: FPNavAbstractCostFold.Min).Checksum;

            Assert.AreNotEqual(baseline, Derive(mesh, cellSize: 16.0).Checksum, "cell size");
            Assert.AreNotEqual(baseline, Derive(mesh, fold: FPNavAbstractCostFold.Mean).Checksum, "cost fold");
        }

        #endregion

        #region The partition

        [Test]
        public void EveryWalkableTriangleLandsInExactlyOneNode()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var g = Derive(mesh);

            var counted = new int[g.NodeCount];
            for (int t = 0; t < mesh.Triangles.Length; t++)
            {
                int n = g.NodeOf(t);
                Assert.IsTrue(n >= 0 && n < g.NodeCount, $"triangle {t} claims node {n}");
                counted[n]++;
            }
            for (int n = 0; n < g.NodeCount; n++)
                Assert.AreEqual(g.NodeTriangleCount(n), counted[n], $"node {n} size");
        }

        [Test]
        public void ANodeNeverSpansTwoDisconnectedPieces()
        {
            // The reason a cell is not a node: one cell can hold both sides of a wall, and merging
            // them would invent a crossing. Walk each node's triangles by adjacency and check the
            // whole node is reachable from one of them.
            var mesh = NavAgentTestHelper.CreateSplitFieldNavMesh(6, out _);
            var g = Derive(mesh, cellSize: 64.0);   // one cell swallows both patches

            // Without this the test could pass on a mesh the cell never merged in the first place:
            // the split field spans ~28 world units, so a 64-unit lattice puts every triangle in
            // one cell, and more than one node can only come from connectivity.
            Assert.GreaterOrEqual(g.NodeCount, 2,
                "the whole mesh sits in one lattice cell, so 2+ nodes means connectivity split it — " +
                "1 node would mean a cell was mistaken for a node");

            var members = new List<int>[g.NodeCount];
            for (int t = 0; t < mesh.Triangles.Length; t++)
            {
                int n = g.NodeOf(t);
                if (n < 0) continue;
                (members[n] ??= new List<int>()).Add(t);
            }

            for (int n = 0; n < g.NodeCount; n++)
            {
                var list = members[n];
                var seen = new HashSet<int> { list[0] };
                var stack = new Stack<int>();
                stack.Push(list[0]);
                while (stack.Count > 0)
                {
                    int cur = stack.Pop();
                    for (int e = 0; e < 3; e++)
                    {
                        int nb = mesh.Triangles[cur].GetNeighbor(e);
                        if (nb < 0 || g.NodeOf(nb) != n || !seen.Add(nb)) continue;
                        stack.Push(nb);
                    }
                }
                Assert.AreEqual(list.Count, seen.Count,
                    $"node {n} is not connected — a cell was mistaken for a node");
            }
        }

        [Test]
        public void ALargerCellMakesFewerNodes()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            Assert.Less(Derive(mesh, cellSize: 32.0).NodeCount, Derive(mesh, cellSize: 8.0).NodeCount);
        }

        #endregion

        #region Edges

        [Test]
        public void EveryEdgeIsMatchedByOneGoingBack()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var g = Derive(mesh);

            for (int a = 0; a < g.NodeCount; a++)
            {
                g.EdgeRange(a, out int start, out int end);
                for (int i = start; i < end; i++)
                {
                    int b = g.EdgeTarget(i);
                    Assert.AreNotEqual(a, b, "an edge inside one node is not a portal");

                    g.EdgeRange(b, out int bs, out int be);
                    bool back = false;
                    for (int j = bs; j < be && !back; j++) back = g.EdgeTarget(j) == a;
                    Assert.IsTrue(back, $"node {b} has no edge back to {a}");

                    // And the reverse INDEX names that edge: same portal, seen from b (V-M15).
                    int rev = g.EdgeReverse(i);
                    Assert.IsTrue(rev >= bs && rev < be, $"edge {i}'s reverse {rev} is not in node {b}'s range");
                    Assert.AreEqual(a, g.EdgeTarget(rev), "the reverse edge comes back to the source node");
                    g.EdgePortalSegment(i, out var ia, out var ib);
                    g.EdgePortalSegment(rev, out var ra, out var rb);
                    Assert.IsTrue((ia == ra && ib == rb) || (ia == rb && ib == ra), "the reverse edge crosses the same portal");
                }
            }
            Assert.AreEqual(0, g.DebugReverseUnmatched, "every edge found its reverse");
        }

        [Test]
        public void ReverseEdges_AreMatchedOnEveryFixture()
        {
            // V-M15: adjacency is symmetric on every mesh the build pipeline makes, so the reverse
            // index is complete — on the open field, on the serpentine (walls and dead ends inside
            // nodes) and at a cell size that leaves single-triangle nodes.
            foreach (var (name, mesh, cell) in new[]
            {
                ("open 12 @ 8", NavAgentTestHelper.CreateOpenFieldNavMesh(12), 8.0),
                ("open 12 @ 4", NavAgentTestHelper.CreateOpenFieldNavMesh(12), 4.0),
                ("serpentine 10x5 @ 16", NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out _), 16.0),
                ("serpentine 10x5 @ 4", NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out _), 4.0),
            })
            {
                var g = Derive(mesh, cell);
                Assert.AreEqual(0, g.DebugReverseUnmatched, $"{name}: unmatched reverse edges");
                for (int e = 0; e < g.EdgeCount; e++)
                    Assert.AreEqual(e, g.EdgeReverse(g.EdgeReverse(e)), $"{name}: reverse is an involution at edge {e}");
            }
        }

        #endregion

        #region The pair table — what a hop through a node costs

        [Test]
        public void PairCost_IsTheStraightDistanceTimesFold_OnAnOpenField()
        {
            // Nothing but the field's rim is a wall, so every portal pair of every node prices
            // straight: midpoint to midpoint, times a fold of 1. Symmetric, zero on the diagonal,
            // positive everywhere else. This is the entry the search charges for a node it passes
            // THROUGH — what replaced centre-to-portal-to-centre.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var g = Derive(mesh);
            int checkedPairs = 0;
            for (int n = 0; n < g.NodeCount; n++)
            {
                g.EdgeRange(n, out int start, out int end);
                int d = end - start;
                for (int i = 0; i < d; i++)
                for (int j = 0; j < d; j++)
                {
                    FP64 actual = g.PairCost(n, i, j);
                    if (i == j) { Assert.AreEqual(FP64.Zero, actual, $"node {n}: diagonal entry {i}"); continue; }
                    FP64 expected = FPVector2.Distance(g.EdgePortal(start + i).ToXZ(), g.EdgePortal(start + j).ToXZ());
                    Assert.AreEqual(expected, actual, $"node {n}: entry ({i},{j}) is not the straight distance");
                    Assert.AreEqual(actual, g.PairCost(n, j, i), $"node {n}: entry ({i},{j}) is not symmetric");
                    Assert.IsTrue(actual > FP64.Zero, $"node {n}: entry ({i},{j}) is not positive");
                    checkedPairs++;
                }
            }
            Assert.Greater(checkedPairs, 0, "the fixture has portal pairs to check");

            // Sized independently of what the graph reports: the old form compared PairEntryCount
            // with itself and passed whatever the table did.
            long expectedEntries = 0;
            for (int n = 0; n < g.NodeCount; n++)
            {
                g.EdgeRange(n, out int s, out int e);
                long outdeg = e - s;
                expectedEntries += outdeg * outdeg;
            }
            Assert.AreEqual(expectedEntries, g.PairEntryCount, "the table is sized outdeg^2 per node");
        }

        [Test]
        public void PairCost_PricesAWalledPair_AlongTheWalk_NeverBelowTheStraightLine()
        {
            // At cell 16 a serpentine node holds two corridor rows joined only by a U-turn at one
            // end. Two portals on the open side of those rows are a few units apart in a straight
            // line and a whole row-length apart on foot; the straight line crosses the gap between
            // rows, which is not mesh. The table has to say the walk, and no entry anywhere may be
            // priced below its straight line.
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out _);
            var g = Derive(mesh, cellSize: 16.0);
            Assert.AreEqual(0, g.DebugReverseUnmatched);

            double worst = 0; int pairs = 0;
            for (int n = 0; n < g.NodeCount; n++)
            {
                g.EdgeRange(n, out int start, out int end);
                int d = end - start;
                for (int i = 0; i < d; i++)
                for (int j = i + 1; j < d; j++)
                {
                    FP64 straight = FPVector2.Distance(g.EdgePortal(start + i).ToXZ(), g.EdgePortal(start + j).ToXZ());
                    FP64 priced = g.PairCost(n, i, j);
                    Assert.IsTrue(priced >= straight, $"node {n} pair ({i},{j}) priced {priced} below its straight line {straight}");
                    // Positivity, which >= straight does not give: two portals can share a midpoint,
                    // and a searched pair that failed to reach its target would read zero.
                    Assert.IsTrue(priced > FP64.Zero, $"node {n} pair ({i},{j}) is not positive");
                    if (straight > FP64.Zero) worst = System.Math.Max(worst, (priced / straight).ToDouble());
                    pairs++;
                }
            }
            Assert.Greater(pairs, 0);
            Assert.Greater(worst, 2.0,
                $"no pair was priced along a U-turn (worst ratio {worst:F2}) — the wall between rows was not seen");
        }

        /// <summary>Open field with <c>costMultiplier</c> 2 on every triangle whose centre lies in the given x/z ranges.</summary>
        private static FPNavMesh SwampField(int cells, double x0, double x1, double z0, double z1)
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(cells);
            for (int t = 0; t < mesh.Triangles.Length; t++)
            {
                double x = mesh.Triangles[t].centerXZ.x.ToDouble(), z = mesh.Triangles[t].centerXZ.y.ToDouble();
                if (x >= x0 && x < x1 && z >= z0 && z < z1)
                    mesh.TrianglesMutable[t].costMultiplier = FP64.FromInt(2);   // what the rebaker stamps through
            }
            return mesh;
        }

        [Test]
        public void PairCost_ChargesTheSwampToTheSwampNode_NotItsNeighbour()
        {
            // V-M9's first half — the fold convention. A node column of swamp (cost 2) aligned to
            // the lattice: every entry of a swamp node is straight distance × 2, every entry of a
            // neighbouring plain node × 1. The old edge cost charged a whole centre-portal-centre
            // hop with the TARGET node's fold, so entering the swamp was priced at 2 for the half
            // walked on plain ground — and leaving it at 1 for the half walked in swamp.
            var mesh = SwampField(48, 32, 48, 0, 96);
            var g = Derive(mesh, cellSize: 16.0);

            int swampNodes = 0, plainNodes = 0;
            for (int n = 0; n < g.NodeCount; n++)
            {
                g.EdgeRange(n, out int start, out int end);
                int d = end - start;
                if (d < 2) continue;
                // Which column is this node? Its portals' midpoints share the node's x range.
                double x = 0; for (int i = start; i < end; i++) x += g.EdgePortal(i).x.ToDouble(); x /= d;
                bool swamp = x >= 32 && x < 48;
                FP64 fold = swamp ? FP64.FromInt(2) : FP64.One;
                for (int i = 0; i < d; i++)
                for (int j = i + 1; j < d; j++)
                {
                    FP64 straight = FPVector2.Distance(g.EdgePortal(start + i).ToXZ(), g.EdgePortal(start + j).ToXZ());
                    Assert.AreEqual(straight * fold, g.PairCost(n, i, j),
                        $"node {n} ({(swamp ? "swamp" : "plain")}, x≈{x:F1}) pair ({i},{j})");
                }
                if (swamp) swampNodes++; else plainNodes++;
            }
            Assert.Greater(swampNodes, 0, "the fixture has swamp nodes");
            Assert.Greater(plainNodes, 0, "and plain ones");
        }

        /// <summary>
        /// Min is admissible only while every cost is at least 1, and nothing bounds them — so the
        /// derivation counts ground cheaper than that and says so ONCE. Both halves are the gate:
        /// the count, and that a mesh of plain ground stays silent.
        /// </summary>
        [Test]
        public void GroundPricedBelowOne_IsCountedAndWarnedOnce()
        {
            int Warnings(LogCapture log)
            {
                int n = 0;
                foreach (var e in log.Entries)
                    if (e.Level == KLogLevel.Warning && e.Message.Contains("price below 1")) n++;
                return n;
            }

            var control = new LogCapture();
            var plain = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var g0 = new FPNavAbstractGraph(plain, FP64.FromDouble(8.0), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK, control);
            Assert.AreEqual(0, g0.DebugCheapTriangles, "control: an open field is all 1.0");
            Assert.AreEqual(0, Warnings(control), "control: nothing to say");

            // A road at 0.5 — natural authoring, and what the rebaker would stamp through.
            var road = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            int stamped = 0;
            for (int t = 0; t < road.Triangles.Length; t++)
                if (road.Triangles[t].centerXZ.x < FP64.FromInt(16))
                {
                    road.TrianglesMutable[t].costMultiplier = FP64.FromDouble(0.5);
                    stamped++;
                }
            Assert.Greater(stamped, 0, "fixture: the road covers triangles");

            var log = new LogCapture();
            var g1 = new FPNavAbstractGraph(road, FP64.FromDouble(8.0), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK, log);
            Assert.AreEqual(stamped, g1.DebugCheapTriangles);
            Assert.AreEqual(1, Warnings(log), "once per derivation, not once per triangle");
        }

        /// <summary>
        /// The segment walk gives up after a fixed number of steps, and the counter that reports it
        /// is not decoration — the cap is reachable. A strip one lattice cell wide cut into three
        /// abstract cells makes exactly ONE node with two portals, so the whole derivation is one
        /// pair and one walk, and the walk crosses two triangles per quad.
        /// </summary>
        [TestCase(500, 0, 1000, TestName = "ASegmentWalk_WithinItsStepBudget_Arrives")]
        [TestCase(520, 1, 1024, TestName = "ASegmentWalk_OutOfSteps_IsCounted")]
        public void SegmentWalkExhaustion(int quadsPerCell, int expectedExhausted, int expectedSteps)
        {
            var mesh = NavAgentTestHelper.CreateStripNavMesh(3 * quadsPerCell);
            var g = Derive(mesh, cellSize: quadsPerCell * NavAgentTestHelper.LatticeCell);

            int pairs = 0;
            for (int n = 0; n < g.NodeCount; n++)
            {
                g.EdgeRange(n, out int s, out int e);
                int d = e - s;
                pairs += d * (d - 1) / 2;
            }
            Assert.AreEqual(1, pairs, "fixture: three cells in a row, so one node has two portals");

            Assert.AreEqual(expectedSteps, g.DebugWalkSteps, "steps of that one walk");
            Assert.AreEqual(expectedExhausted, g.DebugWalkExhausted);
        }

        /// <summary>
        /// The ladder rejects a candidate on this number alone, and it now gets it from a probe
        /// rather than from a half-built graph. The probe is only useful if it is the SAME number:
        /// a probe that measured something else would install a cell size the cap forbids.
        /// </summary>
        [TestCase(12, 8.0)]
        [TestCase(12, 4.0)]
        [TestCase(16, 8.0)]
        [TestCase(24, 16.0)]
        public void TheCorridorProbe_AgreesWithTheGraphItAvoidsBuilding(int cells, double cellSize)
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(cells);
            int probed = FPNavAbstractGraph.MeasureLegCorridorTriangles(
                mesh, FP64.FromDouble(cellSize), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK);

            Assert.AreEqual(Derive(mesh, cellSize).MaxLegCorridorTriangles, probed);
        }

        /// <summary>
        /// One-directional triangle adjacency — a triangle listed as a neighbour by one that does
        /// not list it back. The build pipeline pairs both sides in one statement and the
        /// deserializer checks nothing, so this is what a hand-made or third-party mesh can carry.
        /// It used to throw from inside the in-node search, on the deterministic command path; the
        /// same derivation already TOLERATES the node-boundary form of it. Now both are tolerated.
        ///
        /// <para>Every link in the fixture is broken in turn because whether the search meets one
        /// is not a property of the mesh alone: it walks only the pairs a wall stands between, and
        /// only within a node. The sweep asserts both halves — none of them throws, and at least
        /// one is actually reached, so the counter is not decoration.</para>
        /// </summary>
        [Test]
        public void OneDirectionalAdjacency_IsSkippedAndCounted_NotThrown()
        {
            var pristine = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out _);
            Assert.Greater(Derive(pristine, 8.0).DebugMidDistFills, 0,
                "fixture: the serpentine has walls between portals, so the in-node search runs");
            Assert.AreEqual(0, Derive(pristine, 8.0).DebugAdjacencyAsymmetric,
                "control: a mesh from the build pipeline is symmetric");

            int broken = 0, met = 0;
            for (int t = 0; t < pristine.Triangles.Length; t++)
            for (int k = 0; k < 3; k++)
            {
                int nb = pristine.Triangles[t].GetNeighbor(k);
                if (nb < 0) continue;

                // A fresh mesh each time: cut only nb's pointer back to t, so t still lists nb.
                var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out _);
                for (int e = 0; e < 3; e++)
                    if (mesh.Triangles[nb].GetNeighbor(e) == t) mesh.TrianglesMutable[nb].SetNeighbor(e, -1);
                broken++;

                FPNavAbstractGraph g = null;
                Assert.DoesNotThrow(() => g = Derive(mesh, 8.0), $"triangle {nb} no longer lists {t}");
                if (g.DebugAdjacencyAsymmetric > 0) met++;
            }

            Assert.Greater(broken, 0, "fixture: the serpentine has adjacencies to break");
            Assert.Greater(met, 0, "the in-node search never met one, so the sweep proved nothing");
        }

        #endregion

        #region Determinism (V-5), continued

        #endregion

        #region The diameter that V-2 rests on

        [Test]
        public void TheNodeDiameterIsMeasuredAndBoundedByNodeSize()
        {
            // V-2's guarantee lives here: a leg never leaves its node, so no leg can be longer than
            // the widest node. The measure has to be the internal diameter rather than the triangle
            // count — a compact node's internal path is nearer its square root, and clamping the
            // count would force uselessly small clusters.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var g = Derive(mesh, cellSize: 16.0);

            int largest = 0;
            for (int n = 0; n < g.NodeCount; n++)
                largest = System.Math.Max(largest, g.NodeTriangleCount(n));

            Assert.Greater(g.MaxNodeDiameter, 0, "a multi-triangle node has a diameter");
            Assert.Less(g.MaxNodeDiameter, largest,
                "the diameter must beat the triangle count, or measuring it bought nothing");
        }

        [Test]
        public void ASingleTriangleNodeHasNoDiameter()
        {
            var mesh = NavAgentTestHelper.Create4TriNavMesh();
            var g = Derive(mesh, cellSize: 0.5);   // every triangle gets its own cell
            Assert.AreEqual(0, g.MaxNodeDiameter);
        }

        #endregion

        #region Masking (D-5 — the door stays open)

        [Test]
        public void TheDerivationTakesAMaskSoPerMaskGraphsStayPossible()
        {
            // D-5 (a) is rejected today, but the core must not close the door in code: a game whose
            // mask classes are fixed at authoring time can still want one graph per class.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var everything = new FPNavAbstractGraph(
                mesh, FP64.FromInt(8), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK);
            var nothing = new FPNavAbstractGraph(
                mesh, FP64.FromInt(8), FPNavAbstractCostFold.Min, 1 << 3);

            Assert.Greater(everything.NodeCount, 0);
            Assert.AreEqual(0, nothing.NodeCount, "a mask that matches nothing claims no triangle");
            Assert.AreNotEqual(everything.Checksum, nothing.Checksum);
        }

        #endregion

        #region The public surface a tool draws from (W3)

        [Test]
        public void NodeOf_NormalisesBothEndsOfTheRange()
        {
            // The accessor is public now, so out-of-range stops being "callers do not do that".
            // Both ends were broken in different ways: the backing array is grown by CAPACITY and
            // reused across Rebind, so an index past the current mesh read a node id left over from
            // a larger one, and a negative index threw.
            var large = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var small = NavAgentTestHelper.CreateOpenFieldNavMesh(4);
            Assert.Greater(large.Triangles.Length, small.Triangles.Length,
                "the fixture only means anything while the second mesh is the smaller one");

            var graph = Derive(large);
            graph.Rebind(small);

            Assert.AreEqual(-1, graph.NodeOf(small.Triangles.Length),
                "one past the end is not in the graph");
            Assert.AreEqual(-1, graph.NodeOf(large.Triangles.Length - 1),
                "an index the PREVIOUS mesh had must not read that mesh's leftover node id");
            Assert.AreEqual(-1, graph.NodeOf(-1), "a negative index answers, it does not throw");
            Assert.AreEqual(-1, graph.NodeOf(int.MinValue));

            Assert.GreaterOrEqual(graph.NodeOf(0), 0,
                "and a real triangle still reports the node that claims it");
        }

        [Test]
        public void NodeOf_ReconstructsEveryPortalThePlannerWouldUse()
        {
            // Why one accessor is the whole partition (D-V2). An abstract edge is exactly "two
            // triangles that are mesh neighbours in different nodes" and its portal is the midpoint
            // of the edge they share — no folding, no chosen representative. So a tool holding
            // NodeOf and the mesh can rebuild the adjacency and every portal position, which is
            // what the overlay draws. If this ever stops holding, the overlay starts lying and
            // nothing else would say so.
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out _);
            var graph = Derive(mesh, cellSize: 4.0);
            Assert.Greater(graph.EdgeCount, 0, "the fixture must actually produce crossings");

            int derived = 0;
            for (int t = 0; t < mesh.Triangles.Length; t++)
            {
                int a = graph.NodeOf(t);
                if (a < 0) continue;
                for (int e = 0; e < 3; e++)
                {
                    int nb = mesh.Triangles[t].GetNeighbor(e);
                    if (nb < 0) continue;
                    int b = graph.NodeOf(nb);
                    if (b < 0 || b == a) continue;
                    derived++;
                }
            }

            Assert.AreEqual(graph.EdgeCount, derived,
                "every directed edge the graph holds is derivable from NodeOf plus the mesh");
        }

        [Test]
        public void NodeComponentCount_SeparatesASplitMapFromAConnectedOne()
        {
            // The answer DebugAbstractSearchFailedCount cannot give. That counter reports "no node
            // route", which a split map and a broken derivation produce identically; this says
            // which. One value, no accessor.
            var connected = Derive(NavAgentTestHelper.CreateOpenFieldNavMesh(12), cellSize: 4.0);
            Assert.Greater(connected.NodeCount, 1, "an open field must produce several nodes");
            Assert.AreEqual(1, connected.NodeComponentCount,
                "an open field is one piece, however many nodes it is cut into");

            var split = Derive(
                NavAgentTestHelper.CreateSplitFieldNavMesh(12, out _), cellSize: 4.0);
            Assert.AreEqual(2, split.NodeComponentCount,
                "a field with an impassable divide is two pieces");
        }

        [Test]
        public void TheChecksumIsAGolden_AndMovesOnlyForAReason()
        {
            // The checksum rides the navigation fingerprint, so anything that moves it refuses every
            // replay recorded with a graph installed. This golden is where that becomes a decision
            // rather than an accident: a diff here means the change either altered the derived graph
            // or bumped PLAN_RULE_REVISION, and both are things to do on purpose.
            //
            // NodeComponentCount was added WITHOUT moving it — it is derived from the node count and
            // edge targets, both already folded, so it adds no discrimination and folding it would
            // have bought a replay refusal for nothing.
            //
            // Every PLAN_RULE_REVISION bump DID move it, and had to: those rules change where agents
            // walk without changing one byte of the derived graph, so the checksum is the only place
            // the difference can surface. Two peers on different rules would otherwise meet with
            // matching fingerprints and diverge quietly.
            //
            // Revision 6 moved it three ways at once and on purpose: the rule (hops priced from the
            // entry portal), the representation (the pair table is folded, node centres and the
            // per-edge costs are gone), and the constant. One commit, one move.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var graph = Derive(mesh, cellSize: 4.0);

            Assert.Greater(graph.NodeComponentCount, 0, "the diagnostic is populated");
            Assert.AreEqual(0xDB3D9E595B179530UL, graph.Checksum,
                "the checksum moved — was that a graph change or a deliberate PLAN_RULE_REVISION bump?");
        }

        /// <summary>
        /// The pair table is a behaviour input — the search reads it on every hop — so it has to be
        /// IN the checksum: a derivation contaminated by anything non-deterministic (a container
        /// with unstable order, a stray float) would otherwise put two peers on different tables
        /// behind matching fingerprints. Nudging one entry by the smallest step and re-folding must
        /// move the checksum. Reflection, because nothing legitimate can write the table.
        /// </summary>
        [Test]
        public void PairTable_IsFoldedIntoTheChecksum()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var graph = Derive(mesh, cellSize: 4.0);
            ulong before = graph.Checksum;

            var type = typeof(FPNavAbstractGraph);
            var table = (FP64[])type.GetField("_pairCost",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)!.GetValue(graph)!;
            var fold = type.GetMethod("ComputeChecksum",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)!;
            Assert.Greater(graph.PairEntryCount, 1, "fixture: there is a table to nudge");

            // The first off-diagonal entry of the first node with more than one portal.
            int nudged = 1;
            table[nudged] = FP64.FromRaw(table[nudged].RawValue + 1);
            ulong after = (ulong)fold.Invoke(graph, new object[] { mesh.TriangleCount, graph.NodeCount })!;

            Assert.AreNotEqual(before, after, "a pair-table entry one step off leaves the checksum alone — the table is not folded");
        }

        #endregion
    }
}
