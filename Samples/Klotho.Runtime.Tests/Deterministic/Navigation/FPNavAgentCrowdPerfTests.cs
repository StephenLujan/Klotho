using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using NUnit.Framework;

using xpTURN.Klotho.ECS;
using xpTURN.Klotho.Deterministic.Math;

namespace xpTURN.Klotho.Deterministic.Navigation.Tests
{
    /// <summary>
    /// Measurement harness for hundreds of units under one nav system: what a move order actually
    /// costs, decomposed into the four places the time goes — the A* storm on the order tick, the
    /// O(N²) ORCA neighbour scan every tick after it, the position-correction pass, and the frame
    /// copy that carries <c>NavAgentComponent</c> through the rollback ring.
    ///
    /// It also measures the one lever that needs no engine change: the game owns the entities
    /// array, so it can call <c>Update</c> once per spatial cluster instead of once for everyone.
    /// The split turns O(N²) into O(Σnᵢ²) — this fixture is where that claim stops being arithmetic
    /// and becomes a number.
    ///
    /// Excluded from the normal suite: run explicitly, in Release (DEBUG builds run Debug.Asserts
    /// and the CDT integrity scan — numbers are meaningless there):
    ///   dotnet test -c Release --filter FullyQualifiedName~FPNavAgentCrowdPerfTests
    /// </summary>
    [TestFixture]
    [Explicit("perf measurement — run in Release with an explicit filter")]
    public class FPNavAgentCrowdPerfTests
    {
        #region Harness

        // Same discipline as FPNavMeshRebakerPerfTests: tiered JIT promotes at ~30 calls, so fewer
        // warmups measure tier-0 cold code and inflate the record several-fold.
        private static (double minMs, double medianMs) Measure(Action action, int warmup = 32, int iterations = 9)
        {
            for (int i = 0; i < warmup; i++)
                action();

            var samples = new List<double>(iterations);
            var sw = new Stopwatch();
            for (int i = 0; i < iterations; i++)
            {
                sw.Restart();
                action();
                sw.Stop();
                samples.Add(sw.Elapsed.TotalMilliseconds);
            }
            samples.Sort();
            return (samples[0], samples[samples.Count / 2]);
        }

        private static long MeasureAlloc(Action action, int warmup = 32, int iterations = 5)
        {
            for (int i = 0; i < warmup; i++)
                action();

            var samples = new List<long>(iterations);
            for (int i = 0; i < iterations; i++)
            {
                long before = GC.GetAllocatedBytesForCurrentThread();
                action();
                samples.Add(GC.GetAllocatedBytesForCurrentThread() - before);
            }
            samples.Sort();
            return samples[samples.Count / 2];
        }

        private static void Report(string label, (double minMs, double medianMs) m, string extra = "")
        {
            TestContext.Out.WriteLine($"{label,-44} min {m.minMs,8:F3} ms   median {m.medianMs,8:F3} ms   {extra}");
        }

        private static readonly int[] Sizes = { 64, 256, 800, 3200 };

        // One field wide enough to hold 3200 agents without stacking them all in one spot: agents
        // are laid out on a lattice at AGENT_STRIDE spacing, which is what makes the neighbour scan
        // cost representative (a heap of coincident agents is a different, easier shape).
        private const int FIELD_CELLS = 96;
        private const double AGENT_STRIDE = 1.5;

        private sealed class Crowd
        {
            public FPNavAgentSystem System;
            public Frame Frame;
            public EntityRef[] Entities;
            public EntityRef[][] Clusters;   // the same agents, partitioned into <= MAX_AGENTS runs
            public FPNavMeshPathfinder Pathfinder;
            public FPNavMesh Mesh;
        }

        /// <summary>
        /// N agents on one mesh, already Moving with a live corridor — the steady state, not the
        /// order tick. <paramref name="avoidance"/> off isolates path following from ORCA.
        /// </summary>
        private static Crowd BuildCrowd(int count, bool avoidance, double abstractCell = 0)
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(FIELD_CELLS);
            // Since 0.13 the default tuning installs a graph on a mesh this size by itself, so
            // "flat" has to be asked for: the graph here is always the explicit one or none.
            var system = NavAgentTestHelper.CreateSystem(
                mesh, null, NavAgentTestHelper.NoAutoGraph, out var pathfinder);
            if (abstractCell > 0)
                system.SetAbstractGraph(new FPNavAbstractGraph(
                    mesh, FP64.FromDouble(abstractCell), FPNavAbstractCostFold.Min,
                    FPNavAgentSystem.DEFAULT_AREA_MASK));
            else
                Assert.IsNull(system.AbstractGraph, "flat means no graph installed");
            if (avoidance)
                system.SetAvoidance(new FPNavAvoidance(NavAgentTestHelper.NoAutoGraph));   // same tuning as the system

            int perRow = (int)System.Math.Ceiling(System.Math.Sqrt(count));
            var positions = new FPVector3[count];
            var velocities = new FPVector2[count];
            for (int i = 0; i < count; i++)
            {
                positions[i] = new FPVector3(
                    FP64.FromDouble(8.0 + (i % perRow) * AGENT_STRIDE), FP64.Zero,
                    FP64.FromDouble(8.0 + (i / perRow) * AGENT_STRIDE));
                velocities[i] = FPVector2.Zero;
            }

            var frame = NavAgentTestHelper.CreateFrameWithMovingAgents(
                positions, velocities, out var entities, maxEntities: count + 16);

            // Give everyone a destination across the field and let the first tick resolve paths,
            // so the measured ticks are steady-state following rather than the A* storm.
            FPVector3 target = NavAgentTestHelper.CellCenter(FIELD_CELLS - 2, FIELD_CELLS - 2);
            for (int i = 0; i < count; i++)
            {
                ref var nav = ref frame.Get<NavAgentComponent>(entities[i]);
                NavAgentComponent.SetDestination(ref nav, target);
            }
            system.Update(ref frame, entities, count, 1, NavAgentTestHelper.DT);

            return new Crowd
            {
                System = system,
                Frame = frame,
                Entities = entities,
                Clusters = Partition(entities, count, FPNavAgentSystem.MAX_AGENTS),
                Pathfinder = pathfinder,
                Mesh = mesh,
            };
        }

        /// <summary>
        /// Contiguous partition into runs of at most <paramref name="size"/>. Index order is a pure
        /// function of the array the caller already owns, so it satisfies the determinism rule a
        /// real clustering rule has to satisfy — a spatial rule would sort by cell, but the cost
        /// shape being measured here is the same.
        /// </summary>
        private static EntityRef[][] Partition(EntityRef[] entities, int count, int size)
        {
            int clusters = (count + size - 1) / size;
            var result = new EntityRef[clusters][];
            for (int c = 0; c < clusters; c++)
            {
                int start = c * size;
                int len = System.Math.Min(size, count - start);
                result[c] = new EntityRef[len];
                Array.Copy(entities, start, result[c], 0, len);
            }
            return result;
        }

        private static void RunSplit(Crowd crowd, int tick)
        {
            for (int c = 0; c < crowd.Clusters.Length; c++)
                crowd.System.Update(ref crowd.Frame, crowd.Clusters[c], crowd.Clusters[c].Length,
                    tick, NavAgentTestHelper.DT);
        }

        #endregion

        [Test]
        public void SteadyState_SingleCallVersusClusterSplit()
        {
            TestContext.Out.WriteLine(
                "=== steady-state Update, one call for everyone vs one call per <=64 cluster ===");
            TestContext.Out.WriteLine(
                $"field {FIELD_CELLS}x{FIELD_CELLS} cells, agents on a {AGENT_STRIDE} lattice, ORCA on");

            foreach (int n in Sizes)
            {
                var single = BuildCrowd(n, avoidance: true);
                var split = BuildCrowd(n, avoidance: true);
                int tick = 2;

                var whole = Measure(() => single.System.Update(
                    ref single.Frame, single.Entities, n, tick++, NavAgentTestHelper.DT));
                var clustered = Measure(() => RunSplit(split, tick++));

                double ratio = clustered.medianMs > 0 ? whole.medianMs / clustered.medianMs : 0;
                Report($"{n,5} agents  single call", whole);
                Report($"{n,5} agents  {split.Clusters.Length,3} clusters", clustered,
                    $"{ratio,6:F2}x cheaper than the single call");
            }
        }

        [Test]
        public void CostDecomposition_AStarStorm_Orca_Correction_FrameCopy()
        {
            TestContext.Out.WriteLine("=== where the time goes, per tick ===");

            foreach (int n in Sizes)
            {
                // (1) A* storm: every agent asks for a path on the same tick. This is the order
                //     tick, not the steady state — the cooldown does not gate a first request.
                //     The tick must jump past PathRepathCooldown between samples: a +1 tick leaves
                //     `ticksSinceLast < cooldown` true for every agent that already repathed once,
                //     so ProcessPathRequest returns before FindPath and the sample measures an
                //     empty storm. An early record here was exactly that: it read 0.27 ms at 800
                //     agents against the 966 ms below, understating the storm by three orders of
                //     magnitude and making the cheapest cost look like the second most expensive.
                var storm = BuildCrowd(n, avoidance: true);
                FPVector3 target = NavAgentTestHelper.CellCenter(2, FIELD_CELLS - 2);
                int cooldownTicks = 11;
                int stormTick = 100;
                var stormCost = Measure(() =>
                {
                    for (int i = 0; i < n; i++)
                    {
                        ref var nav = ref storm.Frame.Get<NavAgentComponent>(storm.Entities[i]);
                        NavAgentComponent.SetDestination(ref nav, target);
                    }
                    storm.System.Update(ref storm.Frame, storm.Entities, n, stormTick,
                        NavAgentTestHelper.DT);
                    stormTick += cooldownTicks;
                }, warmup: 8, iterations: 5);

                //     The storm is expensive for reasons the new counters can name, so let them:
                //     a corridor longer than the buffer and a search that runs out of budget are
                //     both silent `false`/clamps without this readout.
                var pathfinder = storm.Pathfinder;

                // (2)+(3) ORCA scan + position correction: the delta between avoidance on and off.
                var withOrca = BuildCrowd(n, avoidance: true);
                var withoutOrca = BuildCrowd(n, avoidance: false);
                int t1 = 2, t2 = 2;
                var on = Measure(() => withOrca.System.Update(
                    ref withOrca.Frame, withOrca.Entities, n, t1++, NavAgentTestHelper.DT));
                var off = Measure(() => withoutOrca.System.Update(
                    ref withoutOrca.Frame, withoutOrca.Entities, n, t2++, NavAgentTestHelper.DT));

                // (4) Frame copy: what the rollback ring pays per tick regardless of nav work.
                var source = withOrca.Frame;
                var dest = new Frame(n + 16, null);
                var copy = Measure(() => dest.CopyFrom(source));

                TestContext.Out.WriteLine($"--- {n} agents ---");
                Report("  (1) A* storm tick (all repath)", stormCost,
                    $"corridor-clamped {pathfinder.DebugCorridorTruncatedCount}, " +
                    $"budget-exhausted {pathfinder.DebugIterationExhaustedCount} (cumulative)");
                Report("  (2+3) ORCA + correction", (on.minMs - off.minMs, on.medianMs - off.medianMs),
                    $"(avoidance on {on.medianMs:F3} - off {off.medianMs:F3})");
                Report("  (rest) path follow + movement", off);
                Report("  (4) Frame.CopyFrom", copy, $"{n + 16} entity slots reserved");
            }
        }

        #region The measurement that decided whether planning in legs was worth building

        /// <summary>
        /// PG · V-3 — the order tick, legs off against legs on, on the same field the baseline was
        /// measured on. This is the number the whole plan exists for: 800 units receiving one move
        /// order cost ~965 ms flat, and ~42% of those searches came back with no path at all.
        /// </summary>
        [Test]
        public void PG_TheOrderTick_LegsOffVersusOn()
        {
            TestContext.Out.WriteLine("=== PG · V-3 — order tick, flat vs legs ===");
            TestContext.Out.WriteLine($"field {FIELD_CELLS}x{FIELD_CELLS} cells, {800} agents on a {AGENT_STRIDE} lattice");
            TestContext.Out.WriteLine("");

            foreach (double cell in new[] { 0.0, 8.0, 16.0, 32.0 })
            {
                const int n = 800;
                var crowd = BuildCrowd(n, avoidance: true, abstractCell: cell);
                FPVector3 target = NavAgentTestHelper.CellCenter(2, FIELD_CELLS - 2);
                int tick = 100;
                const int cooldown = 11;

                var cost = Measure(() =>
                {
                    for (int i = 0; i < n; i++)
                    {
                        ref var nav = ref crowd.Frame.Get<NavAgentComponent>(crowd.Entities[i]);
                        NavAgentComponent.SetDestination(ref nav, target);
                    }
                    crowd.System.Update(ref crowd.Frame, crowd.Entities, n, tick, NavAgentTestHelper.DT);
                    tick += cooldown;
                }, warmup: 4, iterations: 5);

                int failed = 0;
                for (int i = 0; i < n; i++)
                    if (crowd.Frame.GetReadOnly<NavAgentComponent>(crowd.Entities[i]).Status
                        == (byte)FPNavAgentStatus.PathFailed) failed++;

                Report(cell == 0 ? "  legs OFF (flat)" : $"  legs ON  cell {cell,4:F0}", cost,
                    $"{failed,4}/{n} PathFailed | exhausted {crowd.Pathfinder.DebugIterationExhaustedCount,6}, " +
                    $"clamped {crowd.Pathfinder.DebugCorridorTruncatedCount,6}, " +
                    $"abstract-fail {crowd.System.DebugAbstractSearchFailedCount,5}, " +
                    $"leg-fail {crowd.System.DebugLegResolveFailedCount,4}");
            }
        }

        /// <summary>
        /// PG · V-4 and V-11 — what the units actually do. A single agent walks the field with legs
        /// off and on; the distance it covers answers the optimality question (a hierarchical route
        /// is not optimal, and a route that visibly detours fails regardless of planning cost), and
        /// the tick count answers whether anything stalls at a portal.
        /// </summary>
        [Test]
        public void PG_WhatTheUnitDoes_DistanceAndTravelTime()
        {
            TestContext.Out.WriteLine("=== PG · V-4 / V-11 — one unit crossing the field ===");
            TestContext.Out.WriteLine("");

            foreach (int span in new[] { 12, 32, 64, 90 })
            {
                FPVector3 start = NavAgentTestHelper.CellCenter(2, 2);
                FPVector3 goal = NavAgentTestHelper.CellCenter(span, span);
                FP64 straight = FPVector2.Distance(start.ToXZ(), goal.ToXZ());

                TestContext.Out.WriteLine(
                    $"--- {span}x{span} cells apart (straight line {straight.ToDouble():F1}) ---");

                foreach (double cell in new[] { 0.0, 16.0, 32.0 })
                {
                    var r = WalkOne(start, goal, cell);
                    TestContext.Out.WriteLine(
                        $"  {(cell == 0 ? "flat   " : $"cell {cell,4:F0}")}  " +
                        $"{r.status,-11} {r.ticks,5} ticks, travelled {r.distance,8:F1}" +
                        (r.status == "Arrived"
                            ? $"  ratio {r.distance / straight.ToDouble(),5:F2}x straight, {r.legs,3} legs"
                            : ""));
                }
                TestContext.Out.WriteLine("");
            }
        }

        /// <summary>
        /// V-P0 — the ratio off the diagonal. PG measured four routes and all four were exactly
        /// 45 degrees, which is the one angle where committing to portal midpoints costs nothing:
        /// on a square lattice a 45-degree route steps through node centres and the midpoints of
        /// the edges it crosses all land on the same diagonal, so the "zigzag" is a straight line.
        ///
        /// <para>That is why 1.17x passed while a user watching the visualizer sees V-shaped
        /// detours. This walks the same field at roughly 27 degrees (2 east per 1 north) at three
        /// distances, because the question is not "is the ratio bad" but "does it stay bad as the
        /// route gets longer". At 45 degrees the detour was a constant 14.4 units — one bad first
        /// leg — so the ratio improved with distance (1.17x at 85, 1.03x at 249).</para>
        ///
        /// <para><b>Prediction, written before running.</b> If midpoint commitment is what bends the
        /// route, a 2-east-1-north staircase pays 16 + 8*sqrt(2) + 8*sqrt(2) = 38.6 per period
        /// against a straight 35.8, so the ratio should sit near <b>1.08x and stay there at every
        /// distance</b>. A ratio that instead improves with distance says the midpoints are not the
        /// problem and cause (1) is confined to irregular triangulations — which would move this
        /// plan's whole first half.</para>
        /// </summary>
        [Test]
        public void VP0_TheRatioOffTheDiagonal()
        {
            TestContext.Out.WriteLine("=== V-P0 — ratio off the 45-degree diagonal (~27 degrees) ===");
            TestContext.Out.WriteLine(
                "prediction: ~1.08x, CONSTANT across distance, if midpoint commitment is the cause");
            TestContext.Out.WriteLine("");

            // 2 east per 1 north. Same start as PG so the first-leg offset is the one already
            // measured there; only the heading changes.
            foreach (int k in new[] { 12, 32, 44 })
            {
                FPVector3 start = NavAgentTestHelper.CellCenter(2, 2);
                FPVector3 goal = NavAgentTestHelper.CellCenter(2 + 2 * k, 2 + k);
                FP64 straight = FPVector2.Distance(start.ToXZ(), goal.ToXZ());

                TestContext.Out.WriteLine(
                    $"--- k={k,3}  (straight line {straight.ToDouble():F1}) ---");

                foreach (double cell in new[] { 0.0, 16.0, 32.0 })
                {
                    var r = WalkOne(start, goal, cell);
                    double ratio = r.distance / straight.ToDouble();
                    TestContext.Out.WriteLine(
                        $"  {(cell == 0 ? "flat   " : $"cell {cell,4:F0}")}  " +
                        $"{r.status,-11} {r.ticks,5} ticks, travelled {r.distance,8:F1}" +
                        (r.status == "Arrived"
                            ? $"  ratio {ratio,5:F2}x straight, {r.legs,3} legs, "
                              + $"detour {r.distance - straight.ToDouble(),7:F1}"
                            : ""));
                }
                TestContext.Out.WriteLine("");
            }

            TestContext.Out.WriteLine(
                "read the DETOUR column: constant-with-distance means one bad leg (cause 2), " +
                "growing-with-distance means every crossing bends (cause 1)");
        }

        /// <summary>PG — Min against Mean for the cost fold (D-3's open choice).</summary>
        [Test]
        public void PG_WhichCostFold()
        {
            TestContext.Out.WriteLine("=== PG — cost fold: Min (admissible) vs Mean (realistic) ===");
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(FIELD_CELLS);
            foreach (var fold in new[] { FPNavAbstractCostFold.Min, FPNavAbstractCostFold.Mean })
            {
                var sw = Stopwatch.StartNew();
                var g = new FPNavAbstractGraph(mesh, FP64.FromInt(16), fold,
                    FPNavAgentSystem.DEFAULT_AREA_MASK);
                sw.Stop();
                TestContext.Out.WriteLine(
                    $"  {fold,-5}  {g.NodeCount} nodes, {g.EdgeCount} edges, diameter {g.MaxNodeDiameter}, " +
                    $"derive {sw.Elapsed.TotalMilliseconds:F2} ms, checksum {g.Checksum:X16}");
            }
            TestContext.Out.WriteLine(
                "  (an open field gives every triangle the same multiplier, so the two agree here " +
                "by construction — the choice only bites on a mesh with area costs)");
        }

        /// <summary>
        /// One agent walks <paramref name="start"/> → <paramref name="goal"/>. <c>abstractCell</c> 0
        /// is FLAT (no graph — asked for explicitly, since the default tuning would install one).
        /// Besides the total, the walk is cut at every leg hand-off so the MIDDLE of the route —
        /// first and last leg excluded — can be read on its own: the two ends are priced from the
        /// agent's real position and destination and are nearly straight, so a total ratio dilutes
        /// whatever the hops between them still cost (V-M0).
        /// </summary>
        private static (string status, int ticks, double distance, int legs,
                double midDistance, double midStraight, double firstLeg, double lastLeg)
            WalkOne(FPVector3 start, FPVector3 goal, double abstractCell)
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(FIELD_CELLS);
            var system = NavAgentTestHelper.CreateSystem(mesh, null, NavAgentTestHelper.NoAutoGraph, out _);
            if (abstractCell > 0)
                system.SetAbstractGraph(new FPNavAbstractGraph(
                    mesh, FP64.FromDouble(abstractCell), FPNavAbstractCostFold.Min,
                    FPNavAgentSystem.DEFAULT_AREA_MASK));
            else
                Assert.IsNull(system.AbstractGraph, "flat means no graph installed");

            var query = new FPNavMeshQuery(mesh, null);
            int tri = query.FindTriangle(start.ToXZ(), start.y);
            var frame = NavAgentTestHelper.CreateFrameWithAgent(start, tri, out var entity, out var entities);
            ref var nav0 = ref frame.Get<NavAgentComponent>(entity);
            NavAgentComponent.SetDestination(ref nav0, goal);

            FPVector3 prev = start;
            double travelled = 0;
            int legsSeen = 0;
            FPVector3 firstHandoff = start, lastHandoff = start;
            double travelledAtFirst = 0, travelledAtLast = 0;
            const int budget = 20000;
            for (int tick = 1; tick <= budget; tick++)
            {
                system.Update(ref frame, entities, 1, tick, NavAgentTestHelper.DT);
                ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
                travelled += FPVector2.Distance(prev.ToXZ(), nav.Position.ToXZ()).ToDouble();
                prev = nav.Position;

                if (system.DebugLegAdvanceCount > legsSeen)
                {
                    legsSeen = system.DebugLegAdvanceCount;
                    if (legsSeen == 1) { firstHandoff = nav.Position; travelledAtFirst = travelled; }
                    lastHandoff = nav.Position;
                    travelledAtLast = travelled;
                }

                if (nav.Status == (byte)FPNavAgentStatus.Arrived || nav.Status == (byte)FPNavAgentStatus.PathFailed
                    || tick == budget)
                {
                    string status = nav.Status == (byte)FPNavAgentStatus.Arrived ? "Arrived"
                        : nav.Status == (byte)FPNavAgentStatus.PathFailed ? "PathFailed" : "timeout";
                    int legs = system.DebugLegAdvanceCount;
                    double mid = legs >= 2 ? travelledAtLast - travelledAtFirst : 0;
                    double midStraight = legs >= 2
                        ? FPVector2.Distance(firstHandoff.ToXZ(), lastHandoff.ToXZ()).ToDouble() : 0;
                    double lastLeg = legs >= 1 ? travelled - travelledAtLast : travelled;
                    return (status, tick, travelled, legs, mid, midStraight, travelledAtFirst, lastLeg);
                }
            }
            return ("timeout", budget, travelled, system.DebugLegAdvanceCount, 0, 0, 0, 0);
        }

        #endregion

        #region P0 — the middle ratio and the abstract-search baseline (Plan-MidHopEntryPoint)

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
        /// V-M0 — the ratio of the MIDDLE of a route, first and last leg excluded. The two ends are
        /// priced from the agent's position and the destination (the parent plan's endpoint
        /// insertion), so they are nearly straight; the hops between them are still priced centre
        /// to centre, and a total ratio hides what that costs. The middle ratio should sit ABOVE the
        /// total and not improve with distance — if it does not, the plan's premise is wrong.
        /// </summary>
        [Test]
        public void VM0_TheMiddleRatio_OffTheDiagonal()
        {
            TestContext.Out.WriteLine("=== V-M0 — middle ratio (first and last leg excluded), ~27 degrees ===");
            TestContext.Out.WriteLine("");
            foreach (int k in new[] { 12, 32, 44 })
            {
                FPVector3 start = NavAgentTestHelper.CellCenter(2, 2);
                FPVector3 goal = NavAgentTestHelper.CellCenter(2 + 2 * k, 2 + k);
                double straight = FPVector2.Distance(start.ToXZ(), goal.ToXZ()).ToDouble();
                TestContext.Out.WriteLine($"--- k={k,3}  (straight line {straight:F1}) ---");
                foreach (double cell in new[] { 0.0, 16.0, 32.0 })
                {
                    var r = WalkOne(start, goal, cell);
                    string mid = r.legs >= 2 && r.midStraight > 0
                        ? $"middle {r.midDistance / r.midStraight,6:F3}x over {r.midStraight,6:F1} ({r.legs - 1} legs)"
                        : "middle    n/a";
                    TestContext.Out.WriteLine(
                        $"  {(cell == 0 ? "flat   " : $"cell {cell,4:F0}")}  {r.status,-11} " +
                        $"total {r.distance / straight,6:F3}x  {mid}  first {r.firstLeg,5:F1}  last {r.lastLeg,5:F1}");
                }
                TestContext.Out.WriteLine("");
            }
        }

        private static void ReportSearchCost(string name, FPNavMesh mesh, double cell)
        {
            var g = new FPNavAbstractGraph(mesh, FP64.FromDouble(cell), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK);
            int triCount = mesh.Triangles.Length;
            FPVector2 Centroid(int t)
            {
                var tri = mesh.Triangles[t];
                return (mesh.Vertices[tri.v0].ToXZ() + mesh.Vertices[tri.v1].ToXZ() + mesh.Vertices[tri.v2].ToXZ())
                    * (FP64.One / FP64.FromInt(3));
            }

            // Goal: the walkable triangle whose centroid has the largest x+z (a far corner, like PG).
            int goalTri = -1; FP64 best = FP64.MinValue;
            for (int t = 0; t < triCount; t++)
            {
                if (g.NodeOf(t) < 0) continue;
                var c = Centroid(t);
                if (c.x + c.y > best) { best = c.x + c.y; goalTri = t; }
            }
            int goalNode = g.NodeOf(goalTri);
            FPVector2 goalXZ = Centroid(goalTri);

            // Starts: 800 walkable triangles spread evenly over the mesh, not in the goal node.
            var startNode = new List<int>(); var startXZ = new List<FPVector2>();
            for (int t = 0; t < triCount && startNode.Count < 800; t += System.Math.Max(1, triCount / 900))
            {
                int n = g.NodeOf(t);
                if (n < 0 || n == goalNode) continue;
                startNode.Add(n); startXZ.Add(Centroid(t));
            }
            int count = startNode.Count;

            int found = 0;
            for (int i = 0; i < count; i++)
                if (g.TryFindFirstHop(startNode[i], goalNode, startXZ[i], goalXZ, out _, out _, out _)) found++;

            // Untimed pass: what a search does, read off the scratch after each one.
            long relax = 0, closed = 0, touched = 0;
            g.DebugCountMissedImprovements = true;      // untimed: the count costs a root per closed relaxation
            int missedBefore = g.DebugMissedImprovements;
            for (int i = 0; i < count; i++)
            {
                g.TryFindFirstHop(startNode[i], goalNode, startXZ[i], goalXZ, out _, out _, out _);
                for (int n = 0; n < g.NodeCount; n++)
                {
                    if (!g.DebugSearchTouched(n)) continue;
                    touched++;
                    if (!g.DebugSearchClosed(n)) continue;
                    closed++;
                    g.EdgeRange(n, out int es, out int ee);
                    relax += ee - es;
                }
            }
            int missed = g.DebugMissedImprovements - missedBefore;
            g.DebugCountMissedImprovements = false;

            var batches = new List<double>();
            var sw = new Stopwatch();
            for (int b = 0; b < 20; b++)
            {
                sw.Restart();
                for (int i = 0; i < count; i++)
                    g.TryFindFirstHop(startNode[i], goalNode, startXZ[i], goalXZ, out _, out _, out _);
                sw.Stop();
                batches.Add(sw.Elapsed.TotalMilliseconds);
            }
            batches.Sort();

            // Portal length: the freedom a mid-point cost model gives away is half of this.
            var lens = new List<double>(g.EdgeCount);
            for (int e = 0; e < g.EdgeCount; e++)
            {
                g.EdgePortalSegment(e, out var a, out var bEnd);
                lens.Add(FPVector2.Distance(a.ToXZ(), bEnd.ToXZ()).ToDouble());
            }
            lens.Sort();
            long pairs = 0;
            for (int n = 0; n < g.NodeCount; n++) { g.EdgeRange(n, out int es, out int ee); long d = ee - es; pairs += d * d; }

            TestContext.Out.WriteLine(
                $"{name,-12} cell {cell,3:F0}  {g.NodeCount,5} nodes {g.EdgeCount,6} edges ({(double)g.EdgeCount / g.NodeCount,5:F1}/node)  " +
                $"{count} searches ({found} found): min {batches[0],7:F3} ms  median {batches[10],7:F3} ms  = {batches[0] * 1000 / count,6:F2} us/search");
            TestContext.Out.WriteLine(
                $"{"",-12}          per search: touched {(double)touched / count,6:F1}  closed {(double)closed / count,6:F1}  " +
                $"relaxations {(double)relax / count,7:F1}  missed improvements {missed} (total over {count})");
            TestContext.Out.WriteLine(
                $"{"",-12}          portal length: median {lens[lens.Count / 2]:F2}  p99 {lens[(int)(lens.Count * 0.99)]:F2}  max {lens[lens.Count - 1]:F2}" +
                $"  | pair table (sum outdeg^2) {pairs} entries = {pairs * 8 / 1024} KB");
        }

        private static void ReportArithmeticUnitCosts()
        {
            // Pseudo-random FP64 inputs from an LCG so nothing can be hoisted out of the loops.
            const int N = 1_000_000; ulong seed = 12345;
            FP64 Rnd() { seed = seed * 6364136223846793005UL + 1442695040888963407UL; return FP64.FromRaw((long)((seed >> 20) & 0xFFFFFFFFFUL)) - FP64.FromInt(2048); }
            var ps = new FPVector2[4096]; var aa = new FPVector2[4096]; var bb = new FPVector2[4096];
            for (int i = 0; i < 4096; i++)
            {
                ps[i] = new FPVector2(Rnd(), Rnd()); aa[i] = new FPVector2(Rnd(), Rnd());
                bb[i] = aa[i] + new FPVector2(FP64.FromDouble(1.3), FP64.FromDouble(-0.7));
            }
            var table = new FP64[4096]; for (int i = 0; i < 4096; i++) table[i] = Rnd();
            long acc = 0; var sw = new Stopwatch();
            double tLookup = 0, tDist = 0, tCp = 0, tBoth = 0, tSqr = 0;
            for (int rep = 0; rep < 3; rep++)
            {
                sw.Restart(); for (int i = 0; i < N; i++) { int k = i & 4095; acc += (table[k] + table[(k + 1) & 4095]).RawValue; } sw.Stop(); tLookup = sw.Elapsed.TotalMilliseconds * 1e6 / N;
                sw.Restart(); for (int i = 0; i < N; i++) { int k = i & 4095; acc += FPVector2.Distance(ps[k], aa[k]).RawValue; } sw.Stop(); tDist = sw.Elapsed.TotalMilliseconds * 1e6 / N;
                sw.Restart(); for (int i = 0; i < N; i++) { int k = i & 4095; acc += FPNavMeshQuery.ClosestPointOnSegment2D(ps[k], aa[k], bb[k]).x.RawValue; } sw.Stop(); tCp = sw.Elapsed.TotalMilliseconds * 1e6 / N;
                sw.Restart(); for (int i = 0; i < N; i++) { int k = i & 4095; var cp = FPNavMeshQuery.ClosestPointOnSegment2D(ps[k], aa[k], bb[k]); acc += FPVector2.Distance(ps[k], cp).RawValue; } sw.Stop(); tBoth = sw.Elapsed.TotalMilliseconds * 1e6 / N;
                sw.Restart(); for (int i = 0; i < N; i++) { int k = i & 4095; acc += FPVector2.SqrDistance(ps[k], aa[k]).RawValue; } sw.Stop(); tSqr = sw.Elapsed.TotalMilliseconds * 1e6 / N;
            }
            TestContext.Out.WriteLine(
                $"arithmetic ns/op: table lookup+add {tLookup:F1} | Distance (sqrt) {tDist:F1} | ClosestPointOnSegment2D {tCp:F1} | " +
                $"closest+Distance {tBoth:F1} | SqrDistance {tSqr:F1}   (sink {acc & 1})");
        }

        /// <summary>
        /// V-M3 baseline — what the abstract search costs today, per search and per 800, with the
        /// counts that explain it (relaxations, closed, touched) and the unit cost of the arithmetic
        /// a relaxation could be asked to do. The order tick's share is read against
        /// <see cref="PG_TheOrderTick_LegsOffVersusOn"/> on the same machine.
        /// </summary>
        [Test]
        public void VM3_AbstractSearchCost_Baseline()
        {
            TestContext.Out.WriteLine("=== V-M3 — abstract search: cost per search, relaxations, arithmetic unit costs ===");
            ReportArithmeticUnitCosts();
            TestContext.Out.WriteLine("");

            var synth = NavAgentTestHelper.CreateOpenFieldNavMesh(FIELD_CELLS);
            foreach (double cell in new[] { 16.0, 32.0 })
                ReportSearchCost("Synthetic96", synth, cell);

            string path = Path.Combine(RepoRoot(), FieldAsset);
            if (!File.Exists(path)) { TestContext.Out.WriteLine($"{FieldAsset}: MISSING"); return; }
            var field = FPNavMeshSerializer.Deserialize(path);
            foreach (double cell in new[] { 16.0, 32.0 })
                ReportSearchCost("Field", field, cell);
        }

        #endregion

        [Test]
        public void ClusterSize_Sweep_ShowsWhatTheSplitTrades()
        {
            // The split is not a free win. Shrinking the cluster cuts the ORCA neighbour scan
            // (each agent only sees its own cluster), but it also un-caps the position-correction
            // pass: one call of 800 corrects the first 64 agents and drops the rest, while 13 calls
            // of 62 correct all 800. The sweep is where those two curves cross.
            TestContext.Out.WriteLine("=== cluster size sweep, 800 agents ===");
            const int n = 800;
            foreach (int size in new[] { 16, 32, 64, 128, 256, n })
            {
                var crowd = BuildCrowd(n, avoidance: true);
                crowd.Clusters = Partition(crowd.Entities, n, size);
                int tick = 2;
                var cost = Measure(() => RunSplit(crowd, tick++));

                int correctedPerCall = System.Math.Min(size, FPNavAgentSystem.MAX_AGENTS);
                Report($"  cluster {size,4}  ({crowd.Clusters.Length,3} calls)", cost,
                    $"corrects {correctedPerCall * crowd.Clusters.Length,5} of {n} agents");
            }
        }

        [Test]
        public void ClusterSplit_AllocatesNothingPerTick()
        {
            // V-2: the split call pattern must keep the zero-GC contract. Allocation here would
            // mean the recommended pattern trades a GC spike for the O(N²) it saves.
            foreach (int n in Sizes)
            {
                var crowd = BuildCrowd(n, avoidance: true);
                int tick = 2;
                long bytes = MeasureAlloc(() => RunSplit(crowd, tick++));
                TestContext.Out.WriteLine($"{n,5} agents  {crowd.Clusters.Length,3} clusters   {bytes,8} B/tick");
                Assert.AreEqual(0, bytes, $"cluster-split Update allocated {bytes} B at {n} agents");
            }
        }
    }
}
