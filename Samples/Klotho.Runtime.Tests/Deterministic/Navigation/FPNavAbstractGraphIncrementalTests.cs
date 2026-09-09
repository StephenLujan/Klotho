using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using NUnit.Framework;

using xpTURN.Klotho.Deterministic.Math;
using xpTURN.Klotho.Helper.Tests;
using xpTURN.Klotho.Logging;

namespace xpTURN.Klotho.Deterministic.Navigation.Tests
{
    /// <summary>
    /// IMP110 Plan-IncrementalRederive — a graph re-derived for a rebaked mesh with the previous
    /// live graph as DONOR must be, bit for bit, the graph a plain derivation gives. The donor only
    /// moves where the clock is spent. Every test here compares the whole graph — the checksum
    /// (which folds the pair table, edges and portals) AND what the checksum does not fold — against
    /// a fresh derivation of the same mesh, because a peer that joins late derives fresh.
    /// </summary>
    [TestFixture]
    public class FPNavAbstractGraphIncrementalTests
    {
        #region Fixtures — the Field rebaked, as the game rebakes it

        private static FPNavMesh _field;
        private static FPNavMeshRebakeSnapshot _snapshot;
        private static readonly Dictionary<string, FPNavMesh> _rebaked = new Dictionary<string, FPNavMesh>();

        private static FPNavMesh Field()
        {
            if (_field != null) return _field;
            var dir = new DirectoryInfo(AppContext.BaseDirectory);
            while (dir != null && !Directory.Exists(Path.Combine(dir.FullName, "com.xpturn.klotho")))
                dir = dir.Parent;
            Assert.IsNotNull(dir, "repo root not found from the test base directory");
            _field = FPNavMeshSerializer.Deserialize(Path.Combine(dir!.FullName,
                "Samples/Brawler/Assets/NavMesh/Data/Field.NavMeshData.bytes"));
            return _field;
        }

        private static FPNavMeshRebakeSnapshot Snapshot()
            => _snapshot ??= FPNavMeshRebaker.CreateSnapshot(Field(), null, prewarm: false);

        /// <summary>The Field with the first <paramref name="count"/> golden placements, in the
        /// golden's canonical order — the same meshes FPNavMeshRebakeGoldenTests pins.</summary>
        private static FPNavMesh Rebaked(int count)
        {
            string key = "sorted:" + count;
            if (!_rebaked.TryGetValue(key, out var mesh))
            {
                mesh = FPNavMeshRebaker.Rebake(Snapshot(),
                    FPNavMeshRebakeGoldenTests.Take(FPNavMeshRebakeGoldenTests.FieldCenters, count,
                        FPNavMeshRebakeGoldenTests.FieldHalf), null);
                _rebaked[key] = mesh;
            }
            return mesh;
        }

        /// <summary>
        /// The same mesh as <see cref="Rebaked"/> but an instance nobody else holds. <see cref="Rebaked"/>
        /// caches, so a test that retires what it gets back would retire it for the whole fixture.
        /// </summary>
        private static FPNavMesh RebakedUncached(int count)
            => FPNavMeshRebaker.Rebake(Snapshot(),
                FPNavMeshRebakeGoldenTests.Take(FPNavMeshRebakeGoldenTests.FieldCenters, count,
                    FPNavMeshRebakeGoldenTests.FieldHalf), null);

        /// <summary>
        /// The 32 golden placements in one fixed shuffled order (so a chain of rebakes adds
        /// buildings all over the map rather than sweeping one column), first
        /// <paramref name="count"/> of them. The golden set is mutually validated, so any prefix
        /// in any order passes the rebaker's placement rules.
        /// </summary>
        private static FPNavMesh RebakedShuffled(int count)
        {
            string key = "shuffled:" + count;
            if (!_rebaked.TryGetValue(key, out var mesh))
            {
                var all = FPNavMeshRebakeGoldenTests.FieldCenters;
                var order = new int[all.Length];
                for (int i = 0; i < order.Length; i++) order[i] = i;
                uint state = 0x5EED1234u;                    // fixed: the order is part of the fixture
                for (int i = order.Length - 1; i > 0; i--)
                {
                    state = state * 1664525u + 1013904223u;
                    int j = (int)(state % (uint)(i + 1));
                    (order[i], order[j]) = (order[j], order[i]);
                }
                var centers = new (double x, double z)[count];
                for (int i = 0; i < count; i++) centers[i] = all[order[i]];
                mesh = FPNavMeshRebaker.Rebake(Snapshot(),
                    FPNavMeshRebakeGoldenTests.Take(centers, count, FPNavMeshRebakeGoldenTests.FieldHalf), null);
                _rebaked[key] = mesh;
            }
            return mesh;
        }

        private static FPNavAbstractGraph Derive(FPNavMesh mesh, double cell)
            => new FPNavAbstractGraph(mesh, FP64.FromDouble(cell), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK);

        #endregion

        #region The oracle

        /// <summary>
        /// Everything a derivation produces, compared field by field; null when equal, else the
        /// first difference. The checksum folds the node-triangle assignment, the edges with their
        /// portal segments, the diameter and the pair table; the wall flags, the reverse edges and
        /// the per-node counts it does not, so they are read directly.
        /// </summary>
        private static string Difference(FPNavAbstractGraph fresh, FPNavAbstractGraph actual)
        {
            if (fresh.NodeCount != actual.NodeCount) return $"node count {fresh.NodeCount} vs {actual.NodeCount}";
            if (fresh.EdgeCount != actual.EdgeCount) return $"edge count {fresh.EdgeCount} vs {actual.EdgeCount}";
            if (fresh.MaxNodeDiameter != actual.MaxNodeDiameter) return "diameter";
            if (fresh.NodeComponentCount != actual.NodeComponentCount) return "component count";
            if (fresh.PairEntryCount != actual.PairEntryCount) return "pair entry count";
            int tris = fresh.CurrentMesh.TriangleCount;
            for (int t = 0; t < tris; t++)
                if (fresh.NodeOf(t) != actual.NodeOf(t)) return $"node of triangle {t}";
            for (int n = 0; n < fresh.NodeCount; n++)
            {
                if (fresh.NodeHasWall(n) != actual.NodeHasWall(n)) return $"hasWall of node {n}";
                if (fresh.NodeTriangleCount(n) != actual.NodeTriangleCount(n)) return $"triangle count of node {n}";
                fresh.EdgeRange(n, out int fs, out int fe);
                actual.EdgeRange(n, out int @as, out int ae);
                if (fs != @as || fe != ae) return $"edge range of node {n}";
                int d = fe - fs;
                for (int i = 0; i < d; i++)
                    for (int j = 0; j < d; j++)
                        if (fresh.PairCost(n, i, j).RawValue != actual.PairCost(n, i, j).RawValue)
                            return $"pair ({i},{j}) of node {n}: {fresh.PairCost(n, i, j).RawValue} vs {actual.PairCost(n, i, j).RawValue}";
            }
            for (int e = 0; e < fresh.EdgeCount; e++)
            {
                if (fresh.EdgeTarget(e) != actual.EdgeTarget(e)) return $"target of edge {e}";
                if (fresh.EdgeReverse(e) != actual.EdgeReverse(e)) return $"reverse of edge {e}";
                if (!(fresh.EdgePortal(e) == actual.EdgePortal(e))) return $"portal of edge {e}";
                fresh.EdgePortalSegment(e, out var fa, out var fb);
                actual.EdgePortalSegment(e, out var aa, out var ab);
                if (!(fa == aa) || !(fb == ab)) return $"portal segment of edge {e}";
            }
            if (fresh.Checksum != actual.Checksum) return $"checksum 0x{fresh.Checksum:X16} vs 0x{actual.Checksum:X16}";
            return null;
        }

        private static void AssertSameGraph(FPNavAbstractGraph fresh, FPNavAbstractGraph actual, string what)
        {
            string diff = Difference(fresh, actual);
            Assert.IsNull(diff, $"{what}: the incremental graph differs from a fresh one at {diff}");
        }

        /// <summary>Rebinds <paramref name="spare"/> to <paramref name="mesh"/> with the donor and
        /// checks it against a fresh derivation and the donor against itself.</summary>
        private static FPNavAbstractGraph RebindAndCheck(FPNavAbstractGraph spare, FPNavAbstractGraph donor,
            FPNavMesh mesh, double cell, string what)
        {
            ulong donorBefore = donor.Checksum;
            int donorEntries = donor.PairEntryCount;
            spare.Rebind(mesh, donor);
            Assert.AreEqual(donorBefore, donor.Checksum, $"{what}: the donor was written to");
            Assert.AreEqual(donorEntries, donor.PairEntryCount, $"{what}: the donor was written to");
            var fresh = Derive(mesh, cell);
            AssertSameGraph(fresh, spare, what);
            return fresh;
        }

        #endregion

        #region V-I1 — equality

        [TestCase(32.0)]
        [TestCase(16.0)]
        public void AcrossOneRebake_TheDonorGraphIsTheFreshGraph(double cell)
        {
            // Adding buildings and removing them: the rebaker takes the whole set each time, so
            // both directions are the same path; the reuse they allow differs.
            // V-I3 — the floor is the P0b measurement of the 3x3 rule (c32: 27 / 22 / 49 / 49 dirty
            // of 157; c16: 18 / 26 / 48 / 93 of 487) less a margin for wallNear flips. A match
            // failure is never tolerated: the same triangles with the same neighbours form the same
            // components with the same portals, so one would be a rule bug, not a near miss.
            var floor = cell >= 32
                ? new Dictionary<(int, int), int> { [(0, 1)] = 120, [(1, 2)] = 120, [(0, 8)] = 95, [(8, 32)] = 95, [(1, 0)] = 120, [(8, 7)] = 120 }
                : new Dictionary<(int, int), int> { [(0, 1)] = 440, [(1, 2)] = 440, [(0, 8)] = 400, [(8, 32)] = 360, [(1, 0)] = 440, [(8, 7)] = 440 };
            foreach (var (from, to) in new[] { (0, 1), (1, 2), (0, 8), (8, 32), (1, 0), (8, 7) })
            {
                var donor = Derive(Rebaked(from), cell);
                var spare = Derive(Rebaked(from), cell);          // any complete instance; it is overwritten
                RebindAndCheck(spare, donor, Rebaked(to), cell, $"{from} -> {to} @{cell}");
                TestContext.Out.WriteLine($"{from} -> {to} @{cell}: reused {spare.DebugRebindNodesReused} / {spare.NodeCount}, " +
                    $"rebuilt {spare.DebugRebindNodesRebuilt}, flips {spare.DebugRebindWallNearFlips}, " +
                    $"pairs recomputed {spare.DebugRebindPairsRecomputed}, donor used {spare.DebugRebindDonorUsed}" +
                    (spare.DebugRebindDonorIgnored != null ? $" ({spare.DebugRebindDonorIgnored})" : ""));
                Assert.IsTrue(spare.DebugRebindDonorUsed, $"{from} -> {to} @{cell}: the donor qualifies");
                Assert.AreEqual(0, spare.DebugRebindMatchFailures, $"{from} -> {to} @{cell}: a clean node found no donor node or portal");
                Assert.GreaterOrEqual(spare.DebugRebindNodesReused, floor[(from, to)], $"{from} -> {to} @{cell}: reuse fell under the P0b floor");
                Assert.AreEqual(spare.NodeCount, spare.DebugRebindNodesReused + spare.DebugRebindNodesRebuilt, "every node is one or the other");
            }
        }

        [TestCase(32.0)]
        [TestCase(16.0)]
        public void FromTheAssetGraph_TheRealFirstRebake_IsTheFreshGraph(double cell)
        {
            // In a match the first live graph is the ladder's, derived from the ASSET; the first
            // rebake's donor is that graph, and the rebake's triangulation is a different one
            // (§1.6). Reuse is expected to be near zero; equality is not optional.
            var donor = Derive(Field(), cell);
            var spare = Derive(Field(), cell);
            RebindAndCheck(spare, donor, Rebaked(1), cell, $"asset -> rebake(1) @{cell}");
            TestContext.Out.WriteLine($"asset -> rebake(1) @{cell}: reused {spare.DebugRebindNodesReused} / {spare.NodeCount}");
            Assert.IsTrue(spare.DebugRebindDonorUsed, "the asset graph qualifies as a donor; it just has little to give");
            Assert.AreEqual(0, spare.DebugRebindMatchFailures);
        }

        [TestCase(32.0)]
        [TestCase(16.0)]
        public void AlongAChainOfRebakes_EveryStepIsTheFreshGraph(double cell)
        {
            // The double buffer as the game runs it: the live graph donates, the spare is rebound,
            // they swap. Twenty placements in a fixed shuffled order, then three removals.
            var live = Derive(RebakedShuffled(0), cell);
            var spare = Derive(RebakedShuffled(0), cell);
            int steps = 0;
            var sequence = new List<int>();
            for (int n = 1; n <= 20; n++) sequence.Add(n);
            sequence.Add(19); sequence.Add(18); sequence.Add(17);
            foreach (int n in sequence)
            {
                RebindAndCheck(spare, live, RebakedShuffled(n), cell, $"chain step {steps} -> {n} buildings @{cell}");
                (live, spare) = (spare, live);
                steps++;
            }
            Assert.AreEqual(23, steps);
        }

        [Test]
        public void ADegenerateDonor_IsIgnoredInEffect_AndTheGraphIsStillTheFreshOne()
        {
            // Nothing of an open field survives in a serpentine: every cell differs, nothing is
            // reused, and the result is still exactly the fresh graph.
            var donor = Derive(NavAgentTestHelper.CreateOpenFieldNavMesh(24), 8.0);
            var spare = Derive(NavAgentTestHelper.CreateOpenFieldNavMesh(24), 8.0);
            var serpentine = NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out _);
            RebindAndCheck(spare, donor, serpentine, 8.0, "open field donor -> serpentine");
            Assert.AreEqual(0, spare.DebugRebindNodesReused, "nothing of an open field is a serpentine cell");
        }

        [Test]
        public void ATwinMesh_SameContentOtherInstance_IsTheFreshGraph()
        {
            var a = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var b = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var donor = Derive(a, 8.0);
            var spare = Derive(a, 8.0);
            RebindAndCheck(spare, donor, b, 8.0, "twin");
            Assert.AreEqual(0, spare.DebugRebindMatchFailures);
            Assert.AreEqual(spare.NodeCount, spare.DebugRebindNodesReused, "identical content: every node takes the donor's rows");
            Assert.AreEqual(0, spare.DebugRebindNodesRebuilt);
        }

        #endregion

        #region The donor's qualification

        [Test]
        public void ADonorOfAnotherCellSize_IsIgnored_AndTheGraphIsStillTheFreshOne()
        {
            var donor = Derive(Rebaked(0), 16.0);
            var spare = Derive(Rebaked(0), 32.0);
            RebindAndCheck(spare, donor, Rebaked(1), 32.0, "cell mismatch");
            Assert.IsFalse(spare.DebugRebindDonorUsed);
            StringAssert.Contains("cell size", spare.DebugRebindDonorIgnored);
            Assert.AreEqual(0, spare.DebugRebindNodesReused);
        }

        [Test]
        public void ADonorWhoseMeshWasRetired_IsIgnored()
        {
            // The rows were computed over the donor's mesh AS IT WAS. The pool hands a retired
            // mesh's arrays to the next one, so the donor's table can describe geometry that is
            // gone while its triangle COUNT is untouched — the count cannot see it, and neither
            // can a fingerprint, which would read the new contents. The retirement flag can.
            var donor = Derive(RebakedUncached(0), 32.0);
            donor.CurrentMesh.MarkRetired();
            var spare = Derive(Rebaked(0), 32.0);

            spare.Rebind(Rebaked(1), donor);

            Assert.IsFalse(spare.DebugRebindDonorUsed);
            StringAssert.Contains("retired", spare.DebugRebindDonorIgnored);
            AssertSameGraph(Derive(Rebaked(1), 32.0), spare, "retired donor mesh");
        }

        /// <summary>
        /// And it refuses without READING the retired mesh. This is the half a triangle count or a
        /// fingerprint could not have: both go through <c>Triangles</c>/<c>Vertices</c>, whose live
        /// guard throws on a retired mesh in a DEBUG build — turning the detection into an exception
        /// raised from inside a swap, on the deterministic command path.
        /// </summary>
        [Test]
        public void RefusingARetiredDonor_DoesNotReadIt()
        {
            var donor = Derive(RebakedUncached(0), 32.0);
            donor.CurrentMesh.MarkRetired();
            // The tripwire is AssertLive, which is [Conditional("DEBUG")] — in a Release build the
            // call is not emitted and reading a retired mesh is silent. There is nothing to detect
            // then, so say so rather than pass: a green with no tripwire reads as coverage.
            try
            {
                _ = donor.CurrentMesh.Triangles.Length;
                Assert.Ignore("no live guard in this build (AssertLive is DEBUG-only) — nothing to trip");
            }
            catch (System.InvalidOperationException) { }

            var spare = Derive(Rebaked(0), 32.0);
            Assert.DoesNotThrow(() => spare.Rebind(Rebaked(1), donor));
        }

        [Test]
        public void AGraphCannotDonateToItself()
        {
            var graph = Derive(Rebaked(0), 32.0);
            Assert.Throws<ArgumentException>(() => graph.Rebind(Rebaked(1), graph));
        }

        /// <summary>
        /// The lattice is anchored at the world origin and its own doc promises that "negatives round
        /// the same way as positives, so the lattice has no seam at the origin". Everything that reads
        /// two cell indices as ADJACENT depends on it — above all the 3x3 dirty window, which decides
        /// whose rows may be copied. Two assertions, because the seam had two shapes: a band one cell
        /// wide landed two indices from its neighbour, and the plane between them kept the index in
        /// between and became a cell of its own.
        /// </summary>
        [Test]
        public void TheCellLatticeHasNoSeamAtTheOrigin()
        {
            var floorDiv = typeof(FPNavAbstractGraph).GetMethod("FloorDiv",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Static)!;
            FP64 cell = FP64.FromInt(32);
            int Col(double x) => (int)floorDiv.Invoke(null, new object[] { FP64.FromDouble(x), cell })!;

            // Stepping one cell width moves the index by exactly one, everywhere — including across 0.
            for (double x = -96.0; x <= 96.0; x += 0.5)
                Assert.AreEqual(1, Col(x + 32.0) - Col(x),
                    $"one cell width apart at x={x} must be one cell apart");

            // A cell owns its lower boundary, on both sides of the origin.
            Assert.AreEqual(Col(-31.5), Col(-32.0), "the plane x=-32 belongs to the cell it opens");
            Assert.AreEqual(Col(31.5), Col(32.0) - 1, "and the positive side is unchanged");
        }

        /// <summary>
        /// A neighbour link that is severed on one mesh and present on the other changes what the
        /// in-node walk answers, so the canonical key has to see it. It sees WHICH EDGES ARE
        /// BOUNDARIES rather than the neighbour indices: a re-bake keeps geometry and renumbers
        /// triangles, so indices differ for most unchanged triangles and keying on them would leave
        /// nothing to reuse. This pins the half that must be caught.
        /// </summary>
        [Test]
        public void ADonorWhoseTriangleLostANeighbour_IsNotCopiedFrom()
        {
            var donorMesh = RebakedUncached(0);
            var donor = Derive(donorMesh, 32.0);

            // Sever one link on the DONOR's mesh after it derived: same geometry, one edge that is a
            // boundary here and an interior edge on the mesh the spare will be rebound to.
            var mutable = donorMesh.TrianglesMutable;
            int severed = -1;
            for (int t = 0; t < mutable.Length && severed < 0; t++)
                if (mutable[t].neighbor0 >= 0) severed = t;
            Assert.GreaterOrEqual(severed, 0, "fixture: some triangle has an interior edge 0");
            mutable[severed].neighbor0 = -1;

            var spare = Derive(Rebaked(0), 32.0);
            spare.Rebind(Rebaked(0), donor);

            Assert.IsTrue(spare.DebugRebindDonorUsed, "the donor still qualifies — only one cell differs");
            AssertSameGraph(Derive(Rebaked(0), 32.0), spare, "severed donor neighbour");
            Assert.Less(spare.DebugRebindNodesReused, spare.NodeCount,
                "the node whose triangle lost a neighbour must be re-derived, not copied");
        }

        [Test]
        public void WithoutADonor_NothingIsReportedAsIgnored()
        {
            var graph = Derive(Rebaked(0), 32.0);
            graph.Rebind(Rebaked(1));
            Assert.IsFalse(graph.DebugRebindDonorUsed);
            Assert.IsNull(graph.DebugRebindDonorIgnored);
            Assert.AreEqual(0, graph.DebugRebindNodesReused);
            Assert.AreEqual(0, graph.DebugRebindNodesRebuilt);
        }

        #endregion

        #region V-I2 — what the preparation costs now (explicit harness)

        /// <summary>
        /// Wall clock of a fresh derivation against an incremental one on the meshes the game
        /// rebakes, per cell size. Run in Release with <c>DOTNET_TieredCompilation=0</c> (the
        /// harness warms up once, but the tiering trap of the parent plan still applies). The
        /// first-rebake row (asset donor) is the cost of a donor that has nothing to give: the
        /// signature and the merge are paid, the rows are not.
        /// </summary>
        [Test, Explicit("V-I2 measurement — Release, DOTNET_TieredCompilation=0, explicit filter")]
        public void Harness_FreshVersusIncremental()
        {
            foreach (double cell in new[] { 32.0, 16.0 })
            foreach (var (label, donorMesh, mesh) in new (string, FPNavMesh, FPNavMesh)[]
            {
                ("asset -> rebake(1)", Field(), Rebaked(1)),
                ("0 -> 1", Rebaked(0), Rebaked(1)),
                ("1 -> 2", Rebaked(1), Rebaked(2)),
                ("0 -> 8", Rebaked(0), Rebaked(8)),
            })
            {
                var donor = Derive(donorMesh, cell);
                var spare = Derive(donorMesh, cell);
                spare.Rebind(mesh, donor);                     // warm
                double fresh = double.MaxValue, incremental = double.MaxValue;
                for (int rep = 0; rep < 3; rep++)
                {
                    var sw = System.Diagnostics.Stopwatch.StartNew();
                    spare.Rebind(mesh);
                    sw.Stop();
                    if (sw.Elapsed.TotalMilliseconds < fresh) fresh = sw.Elapsed.TotalMilliseconds;
                    sw = System.Diagnostics.Stopwatch.StartNew();
                    spare.Rebind(mesh, donor);
                    sw.Stop();
                    if (sw.Elapsed.TotalMilliseconds < incremental) incremental = sw.Elapsed.TotalMilliseconds;
                }
                TestContext.Out.WriteLine($"{label} @{cell}: fresh {fresh:F1} ms, incremental {incremental:F1} ms " +
                    $"(reused {spare.DebugRebindNodesReused} / {spare.NodeCount}, rebuilt {spare.DebugRebindNodesRebuilt}, " +
                    $"flips {spare.DebugRebindWallNearFlips}, pairs recomputed {spare.DebugRebindPairsRecomputed}, " +
                    $"walk steps {spare.DebugWalkSteps})");
            }
        }

        #endregion

        #region V-I6 — through the system: the prepared graph takes the live one as donor

        private static FPNavAgentSystem SystemOn(FPNavMesh mesh, double cell, LogCapture log = null)
        {
            var system = NavAgentTestHelper.CreateSystem(mesh, log, NavAgentTestHelper.NoAutoGraph, out _);
            system.SetAbstractGraph(Derive(mesh, cell));
            return system;
        }

        [Test]
        public void ASecondPreparation_TakesItsRowsFromTheLiveGraph_AndIsTheSynchronousGraph()
        {
            // The first preparation constructs the spare (no donor: at that moment the live graph is
            // the asset's, which has little to give). From the second on, the spare is rebound with
            // the live graph as donor; on twins that is every row.
            var a = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var b = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var c = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var prepared = SystemOn(a, 8.0);
            var control = SystemOn(NavAgentTestHelper.CreateOpenFieldNavMesh(16), 8.0);

            prepared.PrepareAbstractGraphFor(b);
            prepared.SwapNavMesh(b);
            Assert.IsFalse(prepared.CurrentAbstractGraph.DebugRebindDonorUsed, "fixture: the first preparation is the constructor's");

            prepared.PrepareAbstractGraphFor(c);
            prepared.SwapNavMesh(c);
            control.SwapNavMesh(NavAgentTestHelper.CreateOpenFieldNavMesh(16));
            control.SwapNavMesh(NavAgentTestHelper.CreateOpenFieldNavMesh(16));

            Assert.AreEqual(2, prepared.DebugGraphPreparedAdoptedCount, "fixture: both preparations were adopted");
            Assert.AreEqual(0, prepared.DebugGraphRederiveCount, "and the tick paid nothing");
            var graph = prepared.CurrentAbstractGraph;
            Assert.IsTrue(graph.DebugRebindDonorUsed, "the second preparation had the live graph to copy from");
            Assert.AreEqual(graph.NodeCount, graph.DebugRebindNodesReused, "twins: every node's rows came from the donor");
            Assert.AreEqual(0, graph.DebugRebindMatchFailures);
            Assert.AreEqual(control.GetNavFingerprint(), prepared.GetNavFingerprint(),
                "same inputs, same graph — the donor may only change WHEN the work is done");
        }

        [Test]
        public void TheFieldRebaked_PreparedWithADonor_IsTheSynchronousGraph_AndSaysWhatItReused()
        {
            // The game's sequence on the Field: a graph on the base mesh, then a rebake per building,
            // each prepared off-tick and adopted at the swap. The control swaps without preparing —
            // the in-place fallback, a plain derivation every time.
            var log = new LogCapture();
            var prepared = SystemOn(Rebaked(0), 32.0, log);
            var control = SystemOn(Rebaked(0), 32.0);

            prepared.PrepareAbstractGraphFor(Rebaked(1));
            prepared.SwapNavMesh(Rebaked(1));
            control.SwapNavMesh(Rebaked(1));
            Assert.AreEqual(control.GetNavFingerprint(), prepared.GetNavFingerprint(), "after the first rebake");

            prepared.PrepareAbstractGraphFor(Rebaked(2));
            prepared.SwapNavMesh(Rebaked(2));
            control.SwapNavMesh(Rebaked(2));
            Assert.AreEqual(control.GetNavFingerprint(), prepared.GetNavFingerprint(), "after the second rebake");

            Assert.AreEqual(2, prepared.DebugGraphPreparedAdoptedCount);
            Assert.AreEqual(0, prepared.DebugGraphRederiveCount);
            Assert.AreEqual(2, control.DebugGraphRederiveCount, "fixture: the control derived on the tick, twice");
            var graph = prepared.CurrentAbstractGraph;
            Assert.IsTrue(graph.DebugRebindDonorUsed);
            Assert.GreaterOrEqual(graph.DebugRebindNodesReused, 120, "1 -> 2 at cell 32: the P0b floor");
            Assert.AreEqual(0, graph.DebugRebindMatchFailures);
            Assert.IsTrue(log.Contains(KLogLevel.Information, "nodes' rows from the previous one"),
                "the swap line says what the prepared graph reused — no line of its own");
        }

        #endregion

        #region V-I4 — allocation

        [TestCase(32.0)]
        [TestCase(16.0)]
        public void AnIncrementalRebind_AllocatesNothing_AtSteadyState(double cell)
        {
            // Scratch is grow-only, like the plain derivation's: the first incremental rebind
            // grows it, and a rebind that fits what has been seen allocates nothing. (A mesh that
            // keeps growing by a few triangles per building regrows the triangle-sized arrays on
            // the plain path too — that is the existing policy, not this one's.) Steady state is
            // therefore the same rebind performed twice.
            var one = Rebaked(1);                              // looked up outside the window
            var live = Derive(Rebaked(0), cell);
            var spare = Derive(Rebaked(0), cell);
            spare.Rebind(one, live);                           // warm: every buffer this pair needs
            long before = GC.GetAllocatedBytesForCurrentThread();
            spare.Rebind(one, live);
            long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
            TestContext.Out.WriteLine($"incremental rebind @{cell} allocated {allocated} B");
            Assert.AreEqual(0, allocated, "an incremental rebind allocated at steady state");
        }

        #endregion
    }
}
