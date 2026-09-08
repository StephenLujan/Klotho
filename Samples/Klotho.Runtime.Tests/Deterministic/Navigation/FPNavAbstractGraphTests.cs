using System;
using System.Collections.Generic;
using NUnit.Framework;

using xpTURN.Klotho.Deterministic.Math;

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
                }
            }
        }

        [Test]
        public void CostRisesWithDistanceAndStaysPositive()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var g = Derive(mesh);

            for (int a = 0; a < g.NodeCount; a++)
            {
                g.EdgeRange(a, out int start, out int end);
                for (int i = start; i < end; i++)
                    Assert.IsTrue(g.EdgeCost(i) > FP64.Zero, $"edge {i} out of node {a} has non-positive cost");
            }
        }

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
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var graph = Derive(mesh, cellSize: 4.0);

            Assert.Greater(graph.NodeComponentCount, 0, "the diagnostic is populated");
            Assert.AreEqual(0x4072EF33D9C68FUL, graph.Checksum,
                "the checksum moved — was that a graph change or a deliberate PLAN_RULE_REVISION bump?");
        }

        #endregion
    }
}
