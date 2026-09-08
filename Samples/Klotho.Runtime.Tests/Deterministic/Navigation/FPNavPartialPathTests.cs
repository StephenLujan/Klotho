using System;
using System.IO;
using NUnit.Framework;
using xpTURN.Klotho.ECS;
using xpTURN.Klotho.Deterministic.Math;

namespace xpTURN.Klotho.Deterministic.Navigation.Tests
{
    /// <summary>
    /// Partial paths on budget exhaustion (IMP111): a search that runs out of its iteration budget
    /// may return the corridor to the node that got closest to the goal, instead of nothing, and the
    /// agent walks it and re-plans from its end. Opt-in through <see cref="FPNavTuning"/>.
    ///
    /// <para><b>The goldens come first.</b> "Off is bit-identical to today" can only be asserted
    /// against a value captured BEFORE the feature existed, so the corridor folds below were pinned
    /// on the build immediately preceding the pathfinder change. If one moves, either the search
    /// changed for unchanged inputs (a NAV_BEHAVIOUR_REVISION matter) or the feature leaks into the
    /// off state — both are exactly what this fixture exists to catch.</para>
    /// </summary>
    [TestFixture]
    public class FPNavPartialPathTests
    {
        #region V-0 — goldens captured before the change

        private static string RepoRoot()
        {
            var dir = new DirectoryInfo(AppContext.BaseDirectory);
            while (dir != null && !Directory.Exists(Path.Combine(dir.FullName, "com.xpturn.klotho")))
                dir = dir.Parent;
            Assert.IsNotNull(dir, "repo root not found from test base directory");
            return dir.FullName;
        }

        private const string FieldAsset = "Samples/Brawler/Assets/NavMesh/Data/Field.NavMeshData.bytes";

        /// <summary>
        /// Folds one search's observable output: the verdict, the corridor, and the counters that
        /// say why a false was false. Everything a caller can see, nothing it cannot.
        /// </summary>
        private static ulong FoldSearch(ulong h, FPNavMeshPathfinder pathfinder,
            FPVector3 start, FPVector3 end)
        {
            bool found = pathfinder.FindPath(start, end, FPNavAgentSystem.DEFAULT_AREA_MASK,
                out int[] corridor, out int len);
            h = FPHash.Hash(h, found);
            h = FPHash.Hash(h, len);
            for (int i = 0; i < len; i++)
                h = FPHash.Hash(h, corridor[i]);
            h = FPHash.Hash(h, pathfinder.DebugIterationExhaustedCount);
            h = FPHash.Hash(h, pathfinder.DebugCorridorTruncatedCount);
            h = FPHash.Hash(h, pathfinder.DebugLastSearchIterations);
            return h;
        }

        private static FPVector3 TriangleCenter(FPNavMesh mesh, int tri)
        {
            var c = mesh.Triangles[tri].centerXZ;
            return new FPVector3(c.x, mesh.Vertices[mesh.Triangles[tri].v0].y, c.y);
        }

        /// <summary>
        /// The corridor golden over synthetic meshes: a bending route, an open field, a route that
        /// runs out of budget, and one that has no route at all. Two pins: the OFF stack is the
        /// pre-IMP111 fold, bit for bit — "off is the old behaviour" has to have an old behaviour to
        /// compare with — and the default (on since 0.13) is pinned to its own value, which differs
        /// only in the one search that runs out of budget.
        /// </summary>
        [Test]
        public void Golden_SyntheticCorridors_DidNotMove()
        {
            ulong off = SyntheticFold(Off);
            ulong on = SyntheticFold(FPNavTuning.Default);
            TestContext.Out.WriteLine($"synthetic corridor fold: off = 0x{off:X16}, default = 0x{on:X16}");
            Assert.AreEqual(0xEE16AFD51364AC9BUL, off,
                "the OFF corridors moved for unchanged inputs — either the search changed "
                + "(bump NAV_BEHAVIOUR_REVISION) or the partial-path feature leaks into the off state");
            Assert.AreEqual(0xF7CFB3E630D5632BUL, on,
                "the default-tuning corridors moved for unchanged inputs");
        }

        private static ulong SyntheticFold(FPNavTuning tuning)
        {
            ulong h = FPHash.FNV_OFFSET;

            {
                var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(16, 6, out var endCell);
                var (_, pathfinder) = Stack(mesh, tuning);
                h = FoldSearch(h, pathfinder, NavAgentTestHelper.CellCenter(0, 0),
                    NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz));
            }
            {
                var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
                var (_, pathfinder) = Stack(mesh, tuning);
                h = FoldSearch(h, pathfinder, NavAgentTestHelper.CellCenter(0, 0), NavAgentTestHelper.CellCenter(11, 11));
                h = FoldSearch(h, pathfinder, NavAgentTestHelper.CellCenter(0, 11), NavAgentTestHelper.CellCenter(11, 0));
                h = FoldSearch(h, pathfinder, NavAgentTestHelper.CellCenter(3, 4), NavAgentTestHelper.CellCenter(9, 2));
            }
            {
                // Runs out of budget: the verdict AND the counters are part of the golden — this is
                // the one search where the two tunings fold differently.
                var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(64, 40, out var endCell);
                var (_, pathfinder) = Stack(mesh, tuning);
                h = FoldSearch(h, pathfinder, NavAgentTestHelper.CellCenter(0, 0),
                    NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz));
            }
            {
                var mesh = NavAgentTestHelper.CreateSplitFieldNavMesh(12, out var farCell);
                var (_, pathfinder) = Stack(mesh, tuning);
                h = FoldSearch(h, pathfinder, NavAgentTestHelper.CellCenter(0, 0),
                    NavAgentTestHelper.CellCenter(farCell.gx, farCell.gz));
            }
            return h;
        }

        /// <summary>
        /// The same golden over the shipped Field asset — 32 deterministic triangle-centre pairs,
        /// several of which exhaust the default budget on that mesh. This is the fold the real
        /// world is compared against; the synthetic one above is the fold that runs without assets.
        /// Two pins as above: OFF is the pre-IMP111 fold, the default (on since 0.13) its own.
        /// </summary>
        [Test]
        public void Golden_FieldCorridors_DidNotMove()
        {
            string path = Path.Combine(RepoRoot(), FieldAsset);
            Assert.IsTrue(File.Exists(path), $"asset missing: {FieldAsset}");
            FPNavMesh mesh = FPNavMeshSerializer.Deserialize(path);

            var walkable = new System.Collections.Generic.List<int>();
            for (int t = 0; t < mesh.Triangles.Length; t++)
                if (!mesh.Triangles[t].isBlocked) walkable.Add(t);
            Assert.Greater(walkable.Count, 64, "fixture: Field must be a real mesh");

            ulong Fold(FPNavTuning tuning, out FPNavMeshPathfinder pathfinder)
            {
                (_, pathfinder) = Stack(mesh, tuning);
                ulong h = FPHash.FNV_OFFSET;
                int n = walkable.Count;
                for (int k = 0; k < 32; k++)
                {
                    int a = walkable[(int)((k * 7919L) % n)];
                    int b = walkable[(int)((k * 104729L + 12345L) % n)];
                    h = FoldSearch(h, pathfinder, TriangleCenter(mesh, a), TriangleCenter(mesh, b));
                }
                return h;
            }

            ulong off = Fold(Off, out var pfOff);
            ulong on = Fold(FPNavTuning.Default, out var pfOn);
            TestContext.Out.WriteLine(
                $"Field corridor fold: off = 0x{off:X16} (exhausted {pfOff.DebugIterationExhaustedCount}, clamped {pfOff.DebugCorridorTruncatedCount} of 32), " +
                $"default = 0x{on:X16} (exhausted {pfOn.DebugIterationExhaustedCount}, partial {pfOn.DebugPartialPathCount})");
            Assert.AreEqual(0x0C5599303F9697DCUL, off,
                "the OFF corridors on Field moved for unchanged inputs");
            Assert.AreEqual(0xBBA3C019379A2475UL, on,
                "the default-tuning corridors on Field moved for unchanged inputs");
        }

        #endregion

        #region Fixtures

        private const double CELL = 2.0;   // NavAgentTestHelper's lattice cell — CellCenter assumes it

        /// <summary>
        /// A lattice field with holes: every cell for which <paramref name="walkable"/> is false is
        /// simply not emitted, so a wall is a gap in the surface and the walk treats it as one. Same
        /// build pipeline as the helper's meshes.
        /// </summary>
        private static FPNavMesh BuildField(int width, int height, Func<int, int, bool> walkable)
        {
            var index = new System.Collections.Generic.Dictionary<(int, int), int>();
            var vertices = new System.Collections.Generic.List<FPVector3>();
            var indices = new System.Collections.Generic.List<int>();
            var areas = new System.Collections.Generic.List<int>();
            int Vertex(int gx, int gz)
            {
                if (index.TryGetValue((gx, gz), out int existing)) return existing;
                int id = vertices.Count;
                vertices.Add(new FPVector3(FP64.FromDouble(gx * CELL), FP64.Zero, FP64.FromDouble(gz * CELL)));
                index[(gx, gz)] = id;
                return id;
            }
            for (int gz = 0; gz < height; gz++)
                for (int gx = 0; gx < width; gx++)
                {
                    if (!walkable(gx, gz)) continue;
                    areas.Add(0); areas.Add(0);
                    int v00 = Vertex(gx, gz), v10 = Vertex(gx + 1, gz), v11 = Vertex(gx + 1, gz + 1), v01 = Vertex(gx, gz + 1);
                    indices.Add(v00); indices.Add(v10); indices.Add(v11);
                    indices.Add(v00); indices.Add(v11); indices.Add(v01);
                }
            return FPNavMeshBuildPipeline.Build(vertices.ToArray(), indices.ToArray(), areas.ToArray(),
                CELL * 4.0, null, bakeAgentRadius: 0.5);
        }

        /// <summary>
        /// A 24x24 field with a U-shaped pocket whose closed side faces the goal: a wall at gx=12
        /// over gz 8..16 with arms back along gz=8 and gz=16 to gx=4. A unit west of it, ordered
        /// east, sees the pocket as the way forward — the heuristic points straight through the
        /// wall — and a small budget never gets around the arms. The shape D-3 cannot save.
        /// </summary>
        private static FPNavMesh BuildPocketField() => BuildField(24, 24, (gx, gz) =>
            !((gx == 12 && gz >= 8 && gz <= 16) || ((gz == 8 || gz == 16) && gx >= 4 && gx <= 12)));

        private static (FPNavMeshQuery query, FPNavMeshPathfinder pathfinder) Stack(FPNavMesh mesh, FPNavTuning tuning)
        {
            var query = new FPNavMeshQuery(mesh, null, tuning);
            return (query, new FPNavMeshPathfinder(mesh, query, null, tuning));
        }

        private static FPNavAgentSystem MakeSystem(FPNavMesh mesh, FPNavTuning tuning, out FPNavMeshPathfinder pathfinder,
            xpTURN.Klotho.Logging.IKLogger logger = null)
        {
            var query = new FPNavMeshQuery(mesh, logger, tuning);
            pathfinder = new FPNavMeshPathfinder(mesh, query, logger, tuning);
            var funnel = new FPNavMeshFunnel(mesh, query, logger, tuning);
            return new FPNavAgentSystem(mesh, query, pathfinder, funnel, logger, tuning);
        }

        private static (Frame frame, EntityRef entity, EntityRef[] entities) Agent(FPNavMesh mesh, FPVector3 start, FPVector3 destination)
        {
            var query = new FPNavMeshQuery(mesh, null);
            int tri = query.FindTriangle(start.ToXZ(), start.y);
            Assert.GreaterOrEqual(tri, 0, "fixture: the agent starts on the mesh");
            var frame = NavAgentTestHelper.CreateFrameWithAgent(start, tri, out var entity, out var entities);
            ref var nav = ref frame.Get<NavAgentComponent>(entity);
            NavAgentComponent.SetDestination(ref nav, destination);
            return (frame, entity, entities);
        }

        private static int TriangleAt(FPNavMeshQuery query, FPVector3 p) => query.FindTriangle(p.ToXZ(), p.y);

        private static readonly FPNavTuning On = new FPNavTuning(partialPathOnExhaustion: true);
        // The pre-0.13 stack. Default turned the switch on in 0.13, so "off" must be named.
        private static readonly FPNavTuning Off = new FPNavTuning(partialPathOnExhaustion: false);
        // Flat planning on a mesh past the budget: since 0.13 the default also installs a graph
        // there, and these tests are about what the FLAT search does when it runs out — so the
        // automatic install is named off. The partial switch is what each test is about.
        private static readonly FPNavTuning OnFlat = new FPNavTuning(partialPathOnExhaustion: true, autoInstallAbstractGraph: false);
        private static readonly FPNavTuning OffFlat = new FPNavTuning(partialPathOnExhaustion: false, autoInstallAbstractGraph: false);

        #endregion

        #region V-1 — the end is chosen by definition, not by pop order

        /// <summary>
        /// Two candidates with exactly the same heuristic, and the lower index wins. Budget 1 pops
        /// only the start, so the best node is one of the start's own neighbours; the goal sits on
        /// the perpendicular bisector of two of their entry-edge midpoints (x = 1.5 for the right
        /// neighbour's midpoint (2,1) and the diagonal neighbour's (1,1)), so their heuristics are
        /// the same FP64 to the bit. Which of the two the search happened to push first is not
        /// allowed to matter.
        /// </summary>
        [Test]
        public void TiedCandidates_ResolveToTheLowerTriangleIndex()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var (query, pathfinder) = Stack(mesh, new FPNavTuning(maxIterations: 1, partialPathOnExhaustion: true));

            var start = new FPVector3(FP64.FromDouble(1.5), FP64.Zero, FP64.FromDouble(0.5));   // lower-right triangle of cell (0,0)
            var goal = new FPVector3(FP64.FromDouble(1.5), FP64.Zero, FP64.FromDouble(30.0));
            var midRight = new FPVector2(FP64.FromInt(2), FP64.One);       // entry into the cell to the right
            var midDiag = new FPVector2(FP64.One, FP64.One);               // entry into the cell's other triangle
            Assert.AreEqual(FPVector2.Distance(midRight, goal.ToXZ()), FPVector2.Distance(midDiag, goal.ToXZ()),
                "fixture: the two entry midpoints must tie exactly");

            int rightTri = TriangleAt(query, new FPVector3(FP64.FromDouble(2.7), FP64.Zero, FP64.FromDouble(1.3)));
            int diagTri = TriangleAt(query, new FPVector3(FP64.FromDouble(0.7), FP64.Zero, FP64.FromDouble(1.3)));
            Assert.AreNotEqual(rightTri, diagTri);
            int expected = System.Math.Min(rightTri, diagTri);

            bool found = pathfinder.FindPath(start, goal, FPNavAgentSystem.DEFAULT_AREA_MASK,
                FP64.Zero, out int[] corridor, out int len, out bool partial, out FPVector3 end);

            Assert.IsTrue(found && partial, "budget 1 exhausts with the neighbours still queued");
            Assert.AreEqual(2, len);
            Assert.AreEqual(expected, corridor[1], "the lower index of the two tied candidates");
            Assert.AreEqual(expected, TriangleAt(query, end), "and the end point lies in it");
        }

        /// <summary>
        /// The end triangle on Field, pinned. A twin instance would agree with any bug; a constant
        /// captured once is what notices the tracked heuristic and the chosen end drifting apart.
        /// </summary>
        [Test]
        public void Field_PartialEnd_DidNotMove()
        {
            string path = Path.Combine(RepoRoot(), FieldAsset);
            Assert.IsTrue(File.Exists(path), $"asset missing: {FieldAsset}");
            FPNavMesh mesh = FPNavMeshSerializer.Deserialize(path);
            var (query, pathfinder) = Stack(mesh, new FPNavTuning(maxIterations: 1024, partialPathOnExhaustion: true));

            var walkable = new System.Collections.Generic.List<int>();
            for (int t = 0; t < mesh.Triangles.Length; t++)
                if (!mesh.Triangles[t].isBlocked) walkable.Add(t);
            int n = walkable.Count;
            const int k = 3;
            FPVector3 start = TriangleCenter(mesh, walkable[(int)((k * 7919L) % n)]);
            FPVector3 goal = TriangleCenter(mesh, walkable[(int)((k * 104729L + 12345L) % n)]);

            bool found = pathfinder.FindPath(start, goal, FPNavAgentSystem.DEFAULT_AREA_MASK,
                FP64.Zero, out int[] corridor, out int len, out bool partial, out FPVector3 end);
            Assert.IsTrue(found && partial, "fixture: this pair exhausts a 1024 budget on Field");

            TestContext.Out.WriteLine($"Field partial: len {len}, end tri {corridor[len - 1]}, end {end}");
            Assert.AreEqual(123, len);
            Assert.AreEqual(9266, corridor[len - 1],
                "the end triangle moved for unchanged inputs — the best-node rule or its tie-break changed");
        }

        #endregion

        #region V-2 / V-3 — the switch and the fingerprint

        [Test]
        public void Off_ExhaustionIsStillFalse_AndNothingPartialLeaks()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(64, 40, out var endCell);
            var (_, pathfinder) = Stack(mesh, Off);

            bool found = pathfinder.FindPath(NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz), FPNavAgentSystem.DEFAULT_AREA_MASK,
                FP64.Zero, out _, out int len, out bool partial, out _);

            Assert.IsFalse(found);
            Assert.IsFalse(partial);
            Assert.AreEqual(0, len);
            Assert.AreEqual(1, pathfinder.DebugIterationExhaustedCount, "the budget did run out");
            Assert.AreEqual(0, pathfinder.DebugPartialPathCount);
            Assert.AreEqual(0, pathfinder.DebugPartialRejectedCount, "off means the partial tail is never entered");
            Assert.IsFalse(pathfinder.DebugLastPathWasPartial);
        }

        [Test]
        public void TheSwitch_CountsInEquality_ButNotInTheDigest()
        {
            Assert.IsTrue(FPNavTuning.Default.PartialPathOnExhaustion, "on by default since 0.13");
            Assert.AreEqual(FPNavTuning.Default, On, "naming it on is the default");
            Assert.AreNotEqual(FPNavTuning.Default, Off, "Equals must see the switch, or the stack cross-check waves it through");
            Assert.AreNotEqual(FPNavTuning.Default.GetHashCode(), Off.GetHashCode());
            Assert.AreEqual(FPNavTuning.Default.Digest, Off.Digest, "the digest fold must not move — see PartialPathDigest");
            Assert.AreEqual(0L, FPNavTuning.Default.Digest, "the default digest is still the identity — the switch lives outside the fold");
            Assert.AreEqual(0L, Off.PartialPathDigest);
            Assert.AreNotEqual(0L, On.PartialPathDigest);
            Assert.AreEqual(new FPNavTuning(corridorCap: 32).Digest, new FPNavTuning(corridorCap: 32, partialPathOnExhaustion: false).Digest,
                "a custom tuning's digest is untouched by the switch too");
            Assert.DoesNotThrow(() => Off.Validate());
        }

        /// <summary>
        /// The switch is exactly one XOR term of the fingerprint: off is the pre-0.13 value, on (the
        /// default) is that value XOR the term, and two peers that agree on the switch agree on the
        /// fingerprint. The replay and FullState gates compare this same value
        /// (<c>ReplayAttributionTests.NavFingerprintMismatch_IsRefused</c> pins the refusal itself).
        /// </summary>
        [Test]
        public void TheSwitch_IsExactlyOneFingerprintTerm_ForDefaultAndCustomTunings()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            long Fp(FPNavTuning t) => MakeSystem(mesh, t, out _).GetNavFingerprint();

            long defOff = Fp(Off), defOn = Fp(FPNavTuning.Default);
            long cusOff = Fp(new FPNavTuning(corridorCap: 32, partialPathOnExhaustion: false));
            long cusOn = Fp(new FPNavTuning(corridorCap: 32));

            Assert.AreEqual(unchecked((long)0x303F02AD9AB50251UL), defOff, "off: the pre-0.13 default fingerprint");
            Assert.AreEqual(unchecked((long)0xDC7A49767F3709DDUL), cusOff, "off: the pre-0.13 custom fingerprint");
            Assert.AreNotEqual(defOff, defOn, "on must be refused by an off peer");
            Assert.AreNotEqual(cusOff, cusOn);
            Assert.AreEqual(defOn, Fp(On), "two peers with the switch on agree");
            Assert.AreEqual(unchecked(defOff ^ On.PartialPathDigest), defOn, "the term is exactly the XOR the plan promised");
            Assert.AreEqual(unchecked(cusOff ^ On.PartialPathDigest), cusOn, "for a custom tuning too");
        }

        #endregion

        #region V-4 — what a partial corridor is

        [Test]
        public void Exhausted_WithProgress_ReturnsTheChainToTheClosestNode()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(64, 40, out var endCell);
            var (query, pathfinder) = Stack(mesh, On);
            FPVector3 start = NavAgentTestHelper.CellCenter(0, 0);
            FPVector3 goal = NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz);

            bool found = pathfinder.FindPath(start, goal, FPNavAgentSystem.DEFAULT_AREA_MASK,
                FP64.Zero, out int[] corridor, out int len, out bool partial, out FPVector3 end);

            Assert.IsTrue(found);
            Assert.IsTrue(partial);
            Assert.IsTrue(pathfinder.DebugLastPathWasPartial);
            Assert.AreEqual(1, pathfinder.DebugIterationExhaustedCount, "a partial is still an exhaustion");
            Assert.AreEqual(1, pathfinder.DebugPartialPathCount);
            Assert.Greater(len, 1);
            Assert.LessOrEqual(len, On.CorridorCap);
            Assert.AreEqual(TriangleAt(query, start), corridor[0], "the corridor starts under the agent");
            Assert.AreEqual(corridor[len - 1], TriangleAt(query, end), "and ends where the end point is");
            Assert.AreNotEqual(TriangleAt(query, goal), corridor[len - 1], "partial: it does not reach the goal");

            ref readonly var last = ref mesh.Triangles[corridor[len - 1]];
            Assert.AreEqual(last.centerXZ, end.ToXZ(), "the end point is the last triangle's centre");
            Assert.AreEqual(mesh.Vertices[last.v0].y, end.y, "at the triangle's height (flat fixture)");
        }

        [Test]
        public void Exhausted_BelowTheCallersMinimum_IsRefused()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(64, 40, out var endCell);
            var (_, pathfinder) = Stack(mesh, On);

            bool found = pathfinder.FindPath(NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz), FPNavAgentSystem.DEFAULT_AREA_MASK,
                FP64.FromInt(10000), out _, out int len, out bool partial, out _);

            Assert.IsFalse(found);
            Assert.IsFalse(partial);
            Assert.AreEqual(0, len);
            Assert.AreEqual(1, pathfinder.DebugIterationExhaustedCount);
            Assert.AreEqual(1, pathfinder.DebugPartialRejectedCount);
            Assert.AreEqual(0, pathfinder.DebugPartialPathCount);
        }

        /// <summary>
        /// Standing against the pocket wall with the goal behind it, the closest node the budget
        /// reaches is where the agent already is. No progress, no partial — this is the failure
        /// that stays a failure.
        /// </summary>
        [Test]
        public void Exhausted_InAPocketFacingTheGoal_IsRefused()
        {
            var mesh = BuildPocketField();
            var (_, pathfinder) = Stack(mesh, new FPNavTuning(maxIterations: 120, partialPathOnExhaustion: true));

            bool found = pathfinder.FindPath(NavAgentTestHelper.CellCenter(11, 12), NavAgentTestHelper.CellCenter(20, 12),
                FPNavAgentSystem.DEFAULT_AREA_MASK, FP64.FromDouble(2.5), out _, out _, out bool partial, out _);

            Assert.IsFalse(found);
            Assert.IsFalse(partial);
            Assert.AreEqual(1, pathfinder.DebugIterationExhaustedCount, "fixture: the budget must run out, not the graph");
            Assert.AreEqual(1, pathfinder.DebugPartialRejectedCount);
        }

        [Test]
        public void ReturnsBeforeTheSearch_NeverHandBackAPartial()
        {
            var mesh = NavAgentTestHelper.CreateSplitFieldNavMesh(12, out var farCell);
            var (query, pathfinder) = Stack(mesh, On);
            FPVector3 start = NavAgentTestHelper.CellCenter(0, 0);

            // Off-mesh end, blocked end, masked end, same triangle, and a drained graph.
            var offMesh = new FPVector3(FP64.FromInt(-50), FP64.Zero, FP64.FromInt(-50));
            Assert.IsFalse(pathfinder.FindPath(start, offMesh, FPNavAgentSystem.DEFAULT_AREA_MASK, FP64.Zero, out _, out _, out bool p1, out _));
            Assert.IsFalse(p1);

            FPVector3 goal = NavAgentTestHelper.CellCenter(farCell.gx, farCell.gz);
            int goalTri = TriangleAt(query, goal);
            mesh.TrianglesMutable[goalTri].isBlocked = true;
            Assert.IsFalse(pathfinder.FindPath(start, goal, FPNavAgentSystem.DEFAULT_AREA_MASK, FP64.Zero, out _, out _, out bool p2, out _));
            Assert.IsFalse(p2);
            mesh.TrianglesMutable[goalTri].isBlocked = false;

            Assert.IsFalse(pathfinder.FindPath(start, goal, 1 << 5, FP64.Zero, out _, out _, out bool p3, out _), "masked end");
            Assert.IsFalse(p3);

            Assert.IsTrue(pathfinder.FindPath(start, start, FPNavAgentSystem.DEFAULT_AREA_MASK, FP64.Zero, out _, out int len, out bool p4, out _));
            Assert.AreEqual(1, len);
            Assert.IsFalse(p4, "same triangle is a whole path, not a partial one");

            Assert.IsFalse(pathfinder.FindPath(start, goal, FPNavAgentSystem.DEFAULT_AREA_MASK, FP64.Zero, out _, out _, out bool p5, out _), "no route");
            Assert.IsFalse(p5, "graph exhaustion is not budget exhaustion — 'no route' keeps its answer");
            Assert.AreEqual(0, pathfinder.DebugIterationExhaustedCount);
            Assert.AreEqual(0, pathfinder.DebugPartialPathCount);
            Assert.AreEqual(0, pathfinder.DebugPartialRejectedCount);
        }

        #endregion

        #region The goal in the open set when the budget runs out

        /// <summary>
        /// The budget can run out in the window between the push that discovers the goal and the
        /// pop that would finish — about ten pops on Field, always at least one. The chain behind
        /// the goal is a real path, so with the switch on it comes back whole, not as a partial to
        /// the triangle beside it. With the switch off the answer stays <c>false</c>: that search
        /// failed before this change too, and a path there would move the frame hash without a
        /// revision bump.
        /// </summary>
        [Test]
        public void AGoalAlreadyInTheOpenSet_ComesBackWhole_OnlyWithTheSwitchOn()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(48);
            FPVector3 start = NavAgentTestHelper.CellCenter(4, 1);
            FPVector3 goal = NavAgentTestHelper.CellCenter(42, 46);

            // How many pops the search needs, from a pathfinder that cannot run out.
            var (queryBig, big) = Stack(mesh, new FPNavTuning(maxIterations: 1_000_000));
            Assert.IsTrue(big.FindPath(start, goal, FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _));
            int pops = big.DebugLastSearchIterations;
            int goalTri = TriangleAt(queryBig, goal);

            // One pop short: the goal is queued, not popped. Off must be named since 0.13.
            var (_, off) = Stack(mesh, new FPNavTuning(maxIterations: pops - 1, partialPathOnExhaustion: false));
            Assert.IsFalse(off.FindPath(start, goal, FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _),
                "off: one pop short is still a failure — unchanged, so no revision bump");
            Assert.AreEqual(1, off.DebugIterationExhaustedCount);

            var (_, on) = Stack(mesh, new FPNavTuning(maxIterations: pops - 1, partialPathOnExhaustion: true));
            bool found = on.FindPath(start, goal, FPNavAgentSystem.DEFAULT_AREA_MASK,
                FP64.FromDouble(2.5), out int[] corridor, out int len, out bool partial, out FPVector3 end);
            Assert.IsTrue(found, "on: the goal was in hand");
            Assert.IsFalse(partial, "and it is a whole path, not a partial to the triangle beside the goal");
            Assert.IsFalse(on.DebugLastPathWasPartial);
            Assert.AreEqual(goalTri, corridor[len - 1], "the corridor ends at the goal triangle");
            Assert.AreEqual(1, on.DebugIterationExhaustedCount, "the budget did run out, and that stays counted");
            Assert.AreEqual(0, on.DebugPartialPathCount, "nothing partial was given");
            Assert.AreEqual(0, on.DebugPartialRejectedCount);
        }

        #endregion

        #region One hand-off radius, two callers

        /// <summary>
        /// The reach radius and the partial minimum progress must be the same formula, or the
        /// guarantee that a partial's best node is never already inside the reach radius fails
        /// silently. Both now call one helper; this pins the helper and the delegation.
        /// </summary>
        [Test]
        public void HandoffRadius_IsOneFormula_AndPartialMinProgressDelegatesToIt()
        {
            FP64 thr = FP64.FromDouble(0.3);

            Assert.AreEqual(thr, FPNavAgentSystem.HandoffRadius(FP64.FromInt(5), FP64.Zero, thr),
                "no acceleration to divide by: the threshold alone");
            Assert.AreEqual(thr, FPNavAgentSystem.HandoffRadius(FP64.FromInt(1), FP64.FromInt(10), thr),
                "1²/10 = 0.1 is under the threshold: the threshold is the floor");
            Assert.AreEqual(FP64.FromDouble(2.5), FPNavAgentSystem.HandoffRadius(FP64.FromInt(5), FP64.FromInt(10), thr),
                "5²/10 = 2.5 clears the floor");

            var nav = default(NavAgentComponent);
            NavAgentComponent.Init(ref nav, FPVector3.Zero);   // Speed 5, Acceleration 10
            Assert.AreEqual(FPNavAgentSystem.HandoffRadius(nav.Speed, nav.Acceleration, thr),
                FPNavAgentSystem.PartialMinProgress(in nav, thr),
                "PartialMinProgress is the helper at top speed — the same formula ReachRadius uses at current speed");
            nav.Speed = FP64.FromInt(1);
            Assert.AreEqual(thr, FPNavAgentSystem.PartialMinProgress(in nav, thr));
            nav.Acceleration = FP64.Zero;
            Assert.AreEqual(thr, FPNavAgentSystem.PartialMinProgress(in nav, thr));
        }

        #endregion

        #region V-8 — the clamp keeps the agent side, and the walked point is the clipped end

        [Test]
        public void PartialChainOverTheCap_IsClipped_AndTheEndIsTheClippedTriangle()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(64);
            var tuning = new FPNavTuning(corridorCap: 16, maxIterations: 128, partialPathOnExhaustion: true);
            var (query, pathfinder) = Stack(mesh, tuning);
            FPVector3 start = NavAgentTestHelper.CellCenter(1, 1);

            bool found = pathfinder.FindPath(start, NavAgentTestHelper.CellCenter(62, 62), FPNavAgentSystem.DEFAULT_AREA_MASK,
                FP64.Zero, out int[] corridor, out int len, out bool partial, out FPVector3 end);

            Assert.IsTrue(found && partial, "fixture: 128 pops on an open field exhaust far short of (62,62)");
            Assert.AreEqual(16, len, "clipped to the cap");
            Assert.AreEqual(1, pathfinder.DebugCorridorTruncatedCount, "and counted as a clamp, like any other");
            Assert.AreEqual(TriangleAt(query, start), corridor[0], "the agent side is what survives");
            Assert.AreEqual(corridor[15], TriangleAt(query, end), "the walked point is the clipped end, not the best node");
        }

        /// <summary>
        /// A clipped chain that doubles back is cut at its farthest point from the start, not at the
        /// cap. From 7 cells short of a switchback's turn, a 16-cell prefix runs 7 out, 1 up and 8
        /// back — its cap end sits straight across the wall, 4.5 units from where the agent stands.
        /// The corridor now ends at the turn instead: shorter than the cap, every triangle in it no
        /// farther from the start than the last, and the walked point well outside any reach radius.
        /// </summary>
        [Test]
        public void AClippedChainThatDoublesBack_EndsAtItsFarthestPoint_NotAtTheCap()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(24, 12, out var endCell);
            var tuning = new FPNavTuning(corridorCap: 32, maxIterations: 256, partialPathOnExhaustion: true);
            var (query, pathfinder) = Stack(mesh, tuning);
            FPVector3 start = NavAgentTestHelper.CellCenter(16, 0);   // 7 cells short of the turn at gx 23

            bool found = pathfinder.FindPath(start, NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz), FPNavAgentSystem.DEFAULT_AREA_MASK,
                FP64.Zero, out int[] corridor, out int len, out bool partial, out FPVector3 end);

            Assert.IsTrue(found && partial, "fixture: 256 pops exhaust far short of the far end");
            Assert.AreEqual(1, pathfinder.DebugCorridorTruncatedCount, "fixture: the chain to the best node is clipped");
            Assert.Less(len, 32, "cut short of the cap, at the farthest point");
            Assert.AreEqual(TriangleAt(query, start), corridor[0], "the agent side is what survives");

            FP64 endDist = FPVector2.Distance(start.ToXZ(), end.ToXZ());
            for (int i = 0; i < len; i++)
                Assert.LessOrEqual(FPVector2.Distance(start.ToXZ(), mesh.Triangles[corridor[i]].centerXZ).ToDouble(), endDist.ToDouble(),
                    $"corridor[{i}] lies farther from the start than the end does");
            Assert.Greater(endDist.ToDouble(), 12.0, "the end is out at the turn (7 cells = 14 units), not 4.5 units across the wall");
            Assert.AreEqual(corridor[len - 1], TriangleAt(query, end), "the walked point is the corridor's last triangle");
        }

        #endregion

        #region V-5 / V-6 / V-9 — the agent walks it, re-plans, and stops when it cannot

        /// <summary>Runs until the agent stops; returns the tick, and the ticks on which it was handed back to the planner.</summary>
        private static int Walk(FPNavAgentSystem system, ref Frame frame, EntityRef entity, EntityRef[] entities,
            System.Collections.Generic.List<int> handoffTicks, int maxTicks = 6000)
        {
            for (int tick = 1; tick <= maxTicks; tick++)
            {
                system.Update(ref frame, entities, entities.Length, tick, NavAgentTestHelper.DT);
                ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
                if (nav.Status == (byte)FPNavAgentStatus.PathPending) handoffTicks.Add(tick);
                if (nav.Status == (byte)FPNavAgentStatus.Arrived || nav.Status == (byte)FPNavAgentStatus.PathFailed)
                    return tick;
            }
            return maxTicks;
        }

        [Test]
        public void TheEndOfAPartialCorridor_IsAReplan_NotAnArrival()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(48);
            var tuning = new FPNavTuning(maxIterations: 96, partialPathOnExhaustion: true, autoInstallAbstractGraph: false);
            var system = MakeSystem(mesh, tuning, out var pathfinder);
            var (frame, entity, entities) = Agent(mesh, NavAgentTestHelper.CellCenter(1, 1), NavAgentTestHelper.CellCenter(46, 46));

            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);
            {
                ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
                Assert.AreEqual((byte)FPNavAgentStatus.Moving, nav.Status, "a partial corridor is walked, not failed");
                Assert.AreNotEqual(nav.Destination, nav.PathTarget, "the agent aims at the corridor's end");
                Assert.AreEqual(1, pathfinder.DebugPartialPathCount);
                Assert.AreEqual(FPNavPathFailureReason.None,
                    FPNavPathFailure.Diagnose(nav, new FPNavMeshQuery(mesh, null, tuning), mesh, false, pathfinder),
                    "a walking agent has nothing to diagnose");
            }

            var handoffs = new System.Collections.Generic.List<int>();
            Walk(system, ref frame, entity, entities, handoffs);

            ref readonly var done = ref frame.GetReadOnly<NavAgentComponent>(entity);
            Assert.AreEqual((byte)FPNavAgentStatus.Arrived, done.Status, "hop by hop, it gets there");
            Assert.GreaterOrEqual(system.DebugPartialHandoffCount, 1, "at least one partial end was handed back to the planner");
            Assert.AreEqual(system.DebugPartialHandoffCount, handoffs.Count, "every hand-off was observed as PathPending");
            Assert.AreEqual(0, system.DebugLegAdvanceCount, "with no graph, none of this is a leg");
            Assert.AreEqual(0, system.DebugLegEndedOnPlanTickCount);
            // Equal at this speed. Faster units (speed 9 and up on these rows) leave the corridor at
            // turns, and the off-corridor repath then replaces a partial before its end is reached —
            // a re-plan that is not a hand-off — so in general partials given >= partials walked out.
            Assert.GreaterOrEqual(pathfinder.DebugPartialPathCount, system.DebugPartialHandoffCount,
                "a partial corridor is handed off at most once");
            Assert.AreEqual(pathfinder.DebugPartialPathCount, system.DebugPartialHandoffCount,
                "at the default speed every partial is walked to its end — the last search found the goal and was not partial");
        }

        /// <summary>
        /// The new failure mode, and its bound. The unit is west of the pocket, ordered east; the
        /// first partial leads INTO the pocket (real progress by the heuristic), the second finds no
        /// closer point and fails. So: PathFailed, after moving, in exactly one hop.
        /// </summary>
        [Test]
        public void APocketFacingTheGoal_EndsInPathFailed_AfterMoving_InOneHop()
        {
            var mesh = BuildPocketField();
            var tuning = new FPNavTuning(maxIterations: 120, partialPathOnExhaustion: true, autoInstallAbstractGraph: false);
            var system = MakeSystem(mesh, tuning, out var pathfinder);
            FPVector3 start = NavAgentTestHelper.CellCenter(2, 12);
            var (frame, entity, entities) = Agent(mesh, start, NavAgentTestHelper.CellCenter(20, 12));

            var handoffs = new System.Collections.Generic.List<int>();
            int ticks = Walk(system, ref frame, entity, entities, handoffs);

            ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
            Assert.AreEqual((byte)FPNavAgentStatus.PathFailed, nav.Status);
            Assert.Less(ticks, 6000, "it stops on its own");
            Assert.AreEqual(1, system.DebugPartialHandoffCount, "one hop into the pocket");
            Assert.AreEqual(1, pathfinder.DebugPartialPathCount);
            Assert.AreEqual(1, pathfinder.DebugPartialRejectedCount, "the second search found nothing closer");
            Assert.Greater(FPVector2.Distance(nav.Position.ToXZ(), start.ToXZ()).ToDouble(), 10.0,
                "the contract: a PathFailed agent may no longer stand where it was ordered from");

            // V-7: the tool's re-search names the cause.
            var toolQuery = new FPNavMeshQuery(mesh, null, tuning);
            var toolPathfinder = new FPNavMeshPathfinder(mesh, toolQuery, null, tuning);
            Assert.AreEqual(FPNavPathFailureReason.BudgetExhausted, FPNavPathFailure.Diagnose(nav, toolQuery, mesh, false, toolPathfinder));
            Assert.AreEqual(FPNavPathFailureReason.NoRouteOrBudget, FPNavPathFailure.Diagnose(nav, toolQuery, mesh, false),
                "without a pathfinder the verdict is what it was");
            Assert.AreEqual(FPNavPathFailureReason.StaleFailure, FPNavPathFailure.Diagnose(nav, toolQuery, mesh, true, toolPathfinder),
                "a failure older than the mesh is stale before it is anything else");
        }

        [Test]
        public void AnIsland_IsDiagnosedAsNoRoute_NotBudget()
        {
            var mesh = NavAgentTestHelper.CreateSplitFieldNavMesh(12, out var farCell);
            var system = MakeSystem(mesh, On, out _);
            var (frame, entity, entities) = Agent(mesh, NavAgentTestHelper.CellCenter(0, 0), NavAgentTestHelper.CellCenter(farCell.gx, farCell.gz));
            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);

            ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
            Assert.AreEqual((byte)FPNavAgentStatus.PathFailed, nav.Status);
            var toolQuery = new FPNavMeshQuery(mesh, null, On);
            var toolPathfinder = new FPNavMeshPathfinder(mesh, toolQuery, null, On);
            Assert.AreEqual(FPNavPathFailureReason.NoRoute, FPNavPathFailure.Diagnose(nav, toolQuery, mesh, false, toolPathfinder));
            Assert.AreNotEqual(FPNavPathFailure.Describe(FPNavPathFailureReason.NoRoute), FPNavPathFailure.Describe(FPNavPathFailureReason.BudgetExhausted));
        }

        /// <summary>
        /// A winding route in 128-triangle bites: the serpentine's clipped ends lie FURTHER from the
        /// goal than where each hop started, and the unit still arrives — because the progress test
        /// is on the best node, not on the walked point. This is the case a walked-point rule
        /// refused outright in the P0 measurement (0 of 37 reached).
        /// </summary>
        [Test]
        public void AWindingRoute_ArrivesHopByHop_WithoutCycling()
        {
            // Rows of 8 cells, 4 units apart; a 60-pop budget reaches about three rows ahead, so each
            // hop's best node is ~12 units closer while its clipped end (24 triangles, one and a half
            // rows along the switchback) is about where the hop started, or further away.
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(8, 8, out var endCell);
            var tuning = new FPNavTuning(maxIterations: 60, corridorCap: 24, partialPathOnExhaustion: true, autoInstallAbstractGraph: false);
            var system = MakeSystem(mesh, tuning, out var pathfinder);
            var (frame, entity, entities) = Agent(mesh, NavAgentTestHelper.CellCenter(0, 0), NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz));

            var handoffs = new System.Collections.Generic.List<int>();
            Walk(system, ref frame, entity, entities, handoffs, maxTicks: 12000);

            ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
            Assert.AreEqual((byte)FPNavAgentStatus.Arrived, nav.Status);
            Assert.Greater(pathfinder.DebugPartialPathCount, 1, "fixture: the route takes several partials");
            Assert.LessOrEqual(system.DebugPartialHandoffCount, 40, "bounded: no cycling through the same rows");
        }

        /// <summary>
        /// The clipped end must never be handed off on the tick it was planned. The best node cannot
        /// be (its progress is at least the reach radius, and progress never exceeds distance), so
        /// only the corridor cap can put the walked point inside the radius: a chain that doubles
        /// back along a switchback and is clipped where it passes the agent again. Rows of 28 cells
        /// 4 units apart and a 32-triangle cap (16 cells): the hop landings drift along the rows, and
        /// where a hop is planned within ~7 cells of a turn its chain — out, up, and back — used to
        /// be cut straight across the wall from the agent, inside the 4.9-unit reach radius of a unit
        /// at speed 7. That hand-off fired in pass 3 of the tick that planned it, cleared the
        /// cooldown, and the next tick planned again: measured before the fix, 62 such plans in
        /// bursts of up to 17 consecutive ticks on this fixture, each a full-budget search that
        /// moved nothing. Width 24 needs speed 9 to show it, width 64 (half the default cap) never
        /// does — the landings sit on the connectors — which is why the shipped fixtures were quiet.
        /// </summary>
        [Test]
        public void AClippedEnd_IsNeverInsideTheReachRadius_OnThePlanTick()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(28, 12, out var endCell);
            var tuning = new FPNavTuning(maxIterations: 256, corridorCap: 32, partialPathOnExhaustion: true, autoInstallAbstractGraph: false);
            var system = MakeSystem(mesh, tuning, out var pathfinder);
            var (frame, entity, entities) = Agent(mesh, NavAgentTestHelper.CellCenter(0, 0), NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz));
            {
                ref var nav = ref frame.Get<NavAgentComponent>(entity);
                nav.Speed = FP64.FromInt(7);   // reach radius 4.9 > the 4-unit row gap
            }

            var handoffs = new System.Collections.Generic.List<int>();
            Walk(system, ref frame, entity, entities, handoffs, maxTicks: 20000);

            ref readonly var done = ref frame.GetReadOnly<NavAgentComponent>(entity);
            Assert.AreEqual((byte)FPNavAgentStatus.Arrived, done.Status, "fixture: the route is walkable");
            Assert.Greater(pathfinder.DebugCorridorTruncatedCount, 0, "fixture: the chains are clipped by the cap");
            for (int i = 1; i < handoffs.Count; i++)
                Assert.AreNotEqual(handoffs[i - 1] + 1, handoffs[i],
                    $"hand-offs on consecutive ticks {handoffs[i - 1]} and {handoffs[i]}: a plan was handed off on the tick it was made");
            Assert.AreEqual(0, system.DebugPartialEndedOnPlanTickCount, "the runtime's own count of the same event");
        }

        /// <summary>
        /// With a graph installed a LEG search can exhaust too (a node larger than the budget). The
        /// partial applies there as well: the leg does not fall back to the flat search, the agent
        /// walks toward the portal in bites, and still arrives.
        /// </summary>
        [Test]
        public void WithAGraph_AnExhaustedLegSearch_GetsAPartial_NotTheFlatFallback()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(32);
            var tuning = new FPNavTuning(maxIterations: 64, partialPathOnExhaustion: true);
            var system = MakeSystem(mesh, tuning, out var pathfinder);
            system.SetAbstractGraph(new FPNavAbstractGraph(mesh, FP64.FromInt(32), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK));
            var (frame, entity, entities) = Agent(mesh, NavAgentTestHelper.CellCenter(1, 1), NavAgentTestHelper.CellCenter(30, 30));

            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);
            {
                ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
                Assert.AreEqual((byte)FPNavAgentStatus.Moving, nav.Status);
                Assert.AreEqual(1, pathfinder.DebugPartialPathCount, "the leg search exhausted and gave a partial");
                Assert.AreEqual(0, system.DebugLegResolveFailedCount, "so the flat fallback was not taken");
            }
            var handoffs = new System.Collections.Generic.List<int>();
            Walk(system, ref frame, entity, entities, handoffs);
            ref readonly var done = ref frame.GetReadOnly<NavAgentComponent>(entity);
            Assert.AreEqual((byte)FPNavAgentStatus.Arrived, done.Status);
            Assert.AreEqual(0, system.DebugPartialHandoffCount, "with a graph the hand-off cannot tell partial from leg — counted as legs");
            Assert.GreaterOrEqual(system.DebugLegAdvanceCount, 1);
        }

        #endregion

        #region V-10 — exhaustion is still counted and still warned, differently

        [Test]
        public void TheExhaustionWarning_SaysWhatTheUnitGot()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(64, 40, out var endCell);
            FPVector3 goal = NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz);

            var logOn = new xpTURN.Klotho.Helper.Tests.LogCapture();
            var system = MakeSystem(mesh, OnFlat, out var pathfinder, logOn);
            var (frame, entity, entities) = Agent(mesh, NavAgentTestHelper.CellCenter(0, 0), goal);
            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);
            Assert.AreEqual(1, pathfinder.DebugIterationExhaustedCount);
            Assert.AreEqual(1, system.DebugExhaustedWithoutLegsCount, "a partial is still an exhaustion without legs");
            Assert.IsTrue(logOn.Contains(xpTURN.Klotho.Logging.KLogLevel.Warning, "PARTIAL corridor"), "the warning says what the unit got");

            var logOff = new xpTURN.Klotho.Helper.Tests.LogCapture();
            var systemOff = MakeSystem(mesh, OffFlat, out _, logOff);
            var (frameOff, _, entitiesOff) = Agent(mesh, NavAgentTestHelper.CellCenter(0, 0), goal);
            systemOff.Update(ref frameOff, entitiesOff, entitiesOff.Length, 1, NavAgentTestHelper.DT);
            Assert.IsTrue(logOff.Contains(xpTURN.Klotho.Logging.KLogLevel.Warning, "got no path"));
            Assert.IsFalse(logOff.Contains(xpTURN.Klotho.Logging.KLogLevel.Warning, "PARTIAL corridor"));

            // Third outcome: the budget ran out with the goal already in the open set, and the unit
            // got a whole path. The line used to say "no path" here, because it read the pathfinder's
            // last-call flag instead of what the plan ended with.
            var field = NavAgentTestHelper.CreateOpenFieldNavMesh(48);
            FPVector3 s = NavAgentTestHelper.CellCenter(4, 1), g = NavAgentTestHelper.CellCenter(42, 46);
            var (_, big) = Stack(field, new FPNavTuning(maxIterations: 1_000_000));
            Assert.IsTrue(big.FindPath(s, g, FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _));
            var logWhole = new xpTURN.Klotho.Helper.Tests.LogCapture();
            var systemWhole = MakeSystem(field, new FPNavTuning(maxIterations: big.DebugLastSearchIterations - 1, partialPathOnExhaustion: true, autoInstallAbstractGraph: false), out var pfWhole, logWhole);
            var (frameWhole, entityWhole, entitiesWhole) = Agent(field, s, g);
            systemWhole.Update(ref frameWhole, entitiesWhole, entitiesWhole.Length, 1, NavAgentTestHelper.DT);
            ref readonly var navWhole = ref frameWhole.GetReadOnly<NavAgentComponent>(entityWhole);
            Assert.AreEqual((byte)FPNavAgentStatus.Moving, navWhole.Status);
            Assert.AreEqual(navWhole.Destination, navWhole.PathTarget, "a whole path aims at the destination");
            Assert.AreEqual(1, pfWhole.DebugIterationExhaustedCount);
            Assert.IsTrue(logWhole.Contains(xpTURN.Klotho.Logging.KLogLevel.Warning, "whole path anyway"), "the warning says what the unit got");
            Assert.IsFalse(logWhole.Contains(xpTURN.Klotho.Logging.KLogLevel.Warning, "got no path"));
        }

        #endregion
    }
}
