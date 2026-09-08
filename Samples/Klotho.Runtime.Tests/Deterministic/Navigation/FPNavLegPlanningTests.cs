using NUnit.Framework;

using xpTURN.Klotho.ECS;
using xpTURN.Klotho.Deterministic.Math;
using xpTURN.Klotho.Helper.Tests;
using xpTURN.Klotho.Logging;

namespace xpTURN.Klotho.Deterministic.Navigation.Tests
{
    /// <summary>
    /// P2 contract for planning in legs. The gate that decides this feature's fate is a
    /// measurement, so what is pinned here is the behaviour those measurements rest on: that the
    /// off switch is exact, that a leg hand-off does not stall the agent, and that the failures the
    /// leg planner introduces are counted rather than silent.
    /// </summary>
    [TestFixture]
    public class FPNavLegPlanningTests
    {
        private static (FPNavAgentSystem system, Frame frame, EntityRef entity, EntityRef[] entities)
            Walker(FPNavMesh mesh, FPVector3 start, FPVector3 destination, double cellSize)
        {
            var system = NavAgentTestHelper.CreateSystem(mesh, null);
            if (cellSize > 0)
                system.SetAbstractGraph(new FPNavAbstractGraph(
                    mesh, FP64.FromDouble(cellSize), FPNavAbstractCostFold.Min,
                    FPNavAgentSystem.DEFAULT_AREA_MASK));

            var query = new FPNavMeshQuery(mesh, null);
            int tri = query.FindTriangle(start.ToXZ(), start.y);
            Assert.GreaterOrEqual(tri, 0, "the walker starts on the mesh");

            var frame = NavAgentTestHelper.CreateFrameWithAgent(start, tri, out var entity, out var entities);
            ref var nav = ref frame.Get<NavAgentComponent>(entity);
            NavAgentComponent.SetDestination(ref nav, destination);
            return (system, frame, entity, entities);
        }

        /// <summary>Runs until the agent stops moving or the budget runs out; returns the tick count.</summary>
        private static int Walk(FPNavAgentSystem system, ref Frame frame, EntityRef entity,
            EntityRef[] entities, int maxTicks = 4000)
        {
            for (int tick = 1; tick <= maxTicks; tick++)
            {
                system.Update(ref frame, entities, entities.Length, tick, NavAgentTestHelper.DT);
                byte status = frame.Get<NavAgentComponent>(entity).Status;
                if (status == (byte)FPNavAgentStatus.Arrived || status == (byte)FPNavAgentStatus.PathFailed)
                    return tick;
            }
            return maxTicks;
        }

        #region The off switch (V-8's half)

        [Test]
        public void WithNoGraph_TheLegTargetIsAlwaysTheDestination()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var (system, frame, entity, entities) = Walker(
                mesh, NavAgentTestHelper.CellCenter(0, 0), NavAgentTestHelper.CellCenter(10, 10), 0);

            for (int tick = 1; tick <= 40; tick++)
            {
                system.Update(ref frame, entities, entities.Length, tick, NavAgentTestHelper.DT);
                ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
                if (!nav.HasPath) continue;
                Assert.AreEqual(nav.Destination, nav.PathTarget,
                    "with the graph off, nothing may retarget the path");
            }
            Assert.AreEqual(0, system.DebugLegAdvanceCount);
        }

        #endregion

        #region Legs actually happen, and end at the destination

        [Test]
        public void AWalkAcrossManyNodes_AdvancesThroughLegsAndStillArrives()
        {
            // The serpentine forces a route through a long chain of nodes, so a leg planner has to
            // hand off repeatedly and still land on the destination — the end-to-end claim.
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out var endCell);
            FPVector3 start = NavAgentTestHelper.CellCenter(0, 0);
            FPVector3 goal = NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz);

            var (system, frame, entity, entities) = Walker(mesh, start, goal, cellSize: 4.0);
            Walk(system, ref frame, entity, entities);

            ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
            Assert.AreEqual((byte)FPNavAgentStatus.Arrived, nav.Status,
                $"walker ended {(FPNavAgentStatus)nav.Status} after {system.DebugLegAdvanceCount} legs");
            Assert.Greater(system.DebugLegAdvanceCount, 1,
                "a route this long must have been planned in more than one leg");
            Assert.AreEqual(0, system.DebugAbstractSearchFailedCount);
        }

        [Test]
        public void ALegHandOffDoesNotParkTheAgentForTheRepathCooldown()
        {
            // The cooldown exists to stop an agent replanning its DESTINATION every tick. Applying
            // it to a leg hand-off would stop the unit at every portal for its length — which is
            // silent, because nothing fails and the unit merely crawls.
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 5, out var endCell);
            FPVector3 start = NavAgentTestHelper.CellCenter(0, 0);
            FPVector3 goal = NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz);

            var (legged, legFrame, legEntity, legEntities) = Walker(mesh, start, goal, cellSize: 4.0);
            int leggedTicks = Walk(legged, ref legFrame, legEntity, legEntities);

            var (flat, flatFrame, flatEntity, flatEntities) = Walker(mesh, start, goal, cellSize: 0);
            int flatTicks = Walk(flat, ref flatFrame, flatEntity, flatEntities);

            Assert.AreEqual((byte)FPNavAgentStatus.Arrived,
                legFrame.GetReadOnly<NavAgentComponent>(legEntity).Status);
            Assert.AreEqual((byte)FPNavAgentStatus.Arrived,
                flatFrame.GetReadOnly<NavAgentComponent>(flatEntity).Status);

            // A cooldown stall would add PathRepathCooldown ticks per leg; the whole walk staying
            // inside twice the flat one is a wide margin that still catches that.
            Assert.Less(leggedTicks, flatTicks * 2,
                $"{leggedTicks} ticks over {legged.DebugLegAdvanceCount} legs against {flatTicks} flat — " +
                "a leg hand-off is stalling");
        }

        #endregion

        #region Shipping: the fingerprint, and the mask safety net (V-9 · V-10)

        [Test]
        public void WithNoGraph_TheNavFingerprintIsUntouched()
        {
            // The whole reason the leg planner rides the fingerprint instead of the behaviour
            // revision: it is opt-in with an exact off switch, so a build that ships it but does
            // not enable it plans what it always did. Bumping the revision would refuse every
            // recorded replay for a feature nobody turned on.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var plain = NavAgentTestHelper.CreateSystem(mesh, null);
            var alsoPlain = NavAgentTestHelper.CreateSystem(mesh, null);

            Assert.AreEqual(plain.GetNavFingerprint(), alsoPlain.GetNavFingerprint());
        }

        [Test]
        public void InstallingAGraph_MovesTheFingerprint_AndSoDoItsParameters()
        {
            // What must be caught instead: two peers disagreeing about the graph — one planning in
            // legs and one not, or two legged peers with different cell sizes. They would otherwise
            // meet, agree on the mesh, and walk different routes.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(8);

            var off = NavAgentTestHelper.CreateSystem(mesh, null);
            long flat = off.GetNavFingerprint();

            var small = NavAgentTestHelper.CreateSystem(mesh, null);
            small.SetAbstractGraph(new FPNavAbstractGraph(
                mesh, FP64.FromInt(8), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK));

            var large = NavAgentTestHelper.CreateSystem(mesh, null);
            large.SetAbstractGraph(new FPNavAbstractGraph(
                mesh, FP64.FromInt(32), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK));

            Assert.AreNotEqual(flat, small.GetNavFingerprint(), "legs on must differ from legs off");
            Assert.AreNotEqual(small.GetNavFingerprint(), large.GetNavFingerprint(),
                "two legged peers with different cell sizes must differ");
        }

        [Test]
        public void AnAgentPlanningUnderADifferentMask_TakesTheFlatPathAndIsCounted()
        {
            // D-5 (c). The graph was derived for one mask, so planning an agent whose resolved mask
            // is a DIFFERENT one against it would promise crossings that mask forbids — a
            // correctness failure, not a trade. The count is what decides whether per-mask graphs
            // (D-5 (a)) would earn their memory.
            //
            // Mask 1 against a graph derived under DEFAULT_AREA_MASK (= ~BUILDING_MASK): different
            // values, so the graph may not speak for this agent. The sibling test below covers the
            // case the older "does an override exist" form got wrong.
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out var endCell);
            var (system, frame, entity, entities) = Walker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz), cellSize: 4.0);

            ref var nav = ref frame.Get<NavAgentComponent>(entity);
            NavAgentComponent.SetAreaMask(ref nav, planMask: 1, walkMask: 1);
            NavAgentComponent.SetDestination(ref nav, NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz));

            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);

            Assert.AreNotEqual(FPNavAgentSystem.DEFAULT_AREA_MASK, 1,
                "the fixture only means anything while the two masks differ");
            Assert.AreEqual(1, system.DebugMaskFallbackCount, "a differently masked agent skipped the graph");
            Assert.AreEqual(0, system.DebugLegAdvanceCount, "and planned flat, so no leg was taken");
            Assert.AreEqual(nav.Destination, frame.GetReadOnly<NavAgentComponent>(entity).PathTarget,
                "a flat plan aims at the destination");
        }

        [Test]
        public void AnOverrideEqualToTheGraphsMask_StillUsesTheGraph()
        {
            // The case the older rule got wrong. It asked "does this agent carry an override?",
            // which is a proxy for "might its mask differ" — so writing the DEFAULT mask explicitly,
            // the most natural way to say "use the default", opted the agent out of the graph while
            // changing nothing about where it may walk. Nothing reported it either: the resolved
            // mask is identical, so the route it plans is the only observable difference, and the
            // visualizer renders both states with the same text.
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out var endCell);
            var (system, frame, entity, entities) = Walker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz), cellSize: 4.0);

            ref var nav = ref frame.Get<NavAgentComponent>(entity);
            NavAgentComponent.SetAreaMask(ref nav,
                FPNavAgentSystem.DEFAULT_AREA_MASK, FPNavAgentSystem.DEFAULT_AREA_MASK);
            NavAgentComponent.SetDestination(ref nav, NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz));

            Assert.AreNotEqual(0, nav.PlanAreaMaskOverride,
                "the fixture must actually store an override, or it proves nothing");

            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);

            Assert.AreEqual(0, system.DebugMaskFallbackCount,
                "an override resolving to the graph's own mask is not a different mask");
            Assert.AreNotEqual(frame.GetReadOnly<NavAgentComponent>(entity).Destination,
                frame.GetReadOnly<NavAgentComponent>(entity).PathTarget,
                "so it plans in legs, aiming at a portal");
        }

        [Test]
        public void AnAgentWithoutAnOverride_StillUsesTheGraph()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out var endCell);
            var (system, frame, entity, entities) = Walker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz), cellSize: 4.0);

            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);

            Assert.AreEqual(0, system.DebugMaskFallbackCount);
            Assert.AreNotEqual(frame.GetReadOnly<NavAgentComponent>(entity).Destination,
                frame.GetReadOnly<NavAgentComponent>(entity).PathTarget,
                "a legged plan aims at a portal, not the destination");
        }

        [Test]
        public void TheLegTargetFollowsTheAgent_NotItsNodeCentre()
        {
            // Endpoint insertion (the plan's (b)). Two agents in the SAME node with the SAME
            // destination stand at opposite ends of it; if the abstract search still priced its
            // first hop from the node centre, both would be sent to the same portal — the one that
            // is cheapest from a point neither of them occupies. That is what bent the route: an
            // agent that has just changed legs is on its node's boundary, about as far from the
            // centre as it gets, and it would be steered back across the node to a portal chosen
            // for someone else.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(48);
            var system = NavAgentTestHelper.CreateSystem(mesh, null);
            system.SetAbstractGraph(new FPNavAbstractGraph(
                mesh, FP64.FromInt(16), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK));

            FPVector3 goal = NavAgentTestHelper.CellCenter(46, 24);
            FPVector3 north = NavAgentTestHelper.CellCenter(3, 6);
            FPVector3 south = NavAgentTestHelper.CellCenter(3, 1);

            var query = new FPNavMeshQuery(mesh, null);
            var graph = new FPNavAbstractGraph(
                mesh, FP64.FromInt(16), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK);
            Assert.AreEqual(
                graph.NodeOf(query.FindTriangle(north.ToXZ(), north.y)),
                graph.NodeOf(query.FindTriangle(south.ToXZ(), south.y)),
                "the fixture only means anything while both agents start in the SAME node");

            FPVector3 TargetOf(FPVector3 start)
            {
                var frame = NavAgentTestHelper.CreateFrameWithAgent(
                    start, query.FindTriangle(start.ToXZ(), start.y), out var entity, out var entities);
                ref var nav = ref frame.Get<NavAgentComponent>(entity);
                NavAgentComponent.SetDestination(ref nav, goal);
                system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);
                ref readonly var after = ref frame.GetReadOnly<NavAgentComponent>(entity);
                Assert.AreEqual((byte)FPNavAgentStatus.Moving, after.Status, "the agent planned");
                Assert.AreNotEqual(after.Destination, after.PathTarget, "and planned in legs");
                return after.PathTarget;
            }

            FPVector3 fromNorth = TargetOf(north);
            FPVector3 fromSouth = TargetOf(south);

            Assert.AreNotEqual(fromNorth, fromSouth,
                "two agents at opposite ends of one node must not be sent to the same portal — " +
                "the first hop is priced from where each agent stands, not from the node centre");
            Assert.Greater(fromNorth.z.ToDouble(), fromSouth.z.ToDouble(),
                "and each should be sent to the portal on its own side");
        }

        [Test]
        public void LegAdvancesTrackNodeCrossings_RatherThanCirclingAtEachBoundary()
        {
            // A leg ends when the agent reaches its portal — but the portal IS the shared edge, so
            // arriving at it leaves the agent on the near side and its node unchanged. Without a
            // guard the abstract search hands back the crossing just finished, the new leg target is
            // where the agent already stands, and the leg ends again next tick. The hand-off keeps
            // velocity, so nothing stalls and nothing fails: the unit CIRCLES at the boundary until
            // it drifts across. Nothing in the counters said so — DebugLegAdvanceCount simply ran
            // high, which reads like a long route.
            //
            // Measured before the guard on this fixture: 66 advances against 17 node changes.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(96);
            var graph = new FPNavAbstractGraph(mesh, FP64.FromInt(16), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK);
            var (system, frame, entity, entities) = Walker(
                mesh, NavAgentTestHelper.CellCenter(2, 2),
                NavAgentTestHelper.CellCenter(90, 46), cellSize: 16.0);

            int nodeChanges = 0, last = -1;
            for (int tick = 1; tick <= 20000; tick++)
            {
                system.Update(ref frame, entities, entities.Length, tick, NavAgentTestHelper.DT);
                ref readonly var nav = ref frame.GetReadOnly<NavAgentComponent>(entity);
                int node = graph.NodeOf(nav.CurrentTriangleIndex);
                if (node != last) { nodeChanges++; last = node; }
                if (nav.Status == (byte)FPNavAgentStatus.Arrived) break;
                Assert.AreNotEqual((byte)FPNavAgentStatus.PathFailed, nav.Status, "the walk must finish");
            }

            Assert.AreEqual((byte)FPNavAgentStatus.Arrived,
                frame.GetReadOnly<NavAgentComponent>(entity).Status);
            Assert.Greater(nodeChanges, 8, "the fixture must actually cross a chain of nodes");
            Assert.LessOrEqual(system.DebugLegAdvanceCount, nodeChanges + nodeChanges / 2,
                $"{system.DebugLegAdvanceCount} leg advances against {nodeChanges} node changes — "
                + "the agent is re-planning the crossing it has already made");
            Assert.Greater(system.DebugLegAdvanceRepeatCount, 0,
                "and the guard that prevents it must be the reason, not luck");
        }

        #endregion

        #region What the public surface refuses (V-2's guarantee, at install time)

        [Test]
        public void AGraphWiderThanTheCorridorCap_IsRefused()
        {
            // V-2 stops being a hope here. A leg never leaves its node, so the widest node bounds
            // the longest leg; accepting a graph wider than the cap would hand the planner corridors
            // it has to clamp, and a clamped corridor is exactly the silent replanning loop legs
            // exist to remove. Better to refuse at setup than to crawl at runtime.
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(96);
            var system = NavAgentTestHelper.CreateSystem(mesh, null);
            var tooCoarse = new FPNavAbstractGraph(
                mesh, FP64.FromInt(512), FPNavAbstractCostFold.Min,   // one node swallows the field
                FPNavAgentSystem.DEFAULT_AREA_MASK);

            Assert.Greater(tooCoarse.MaxNodeDiameter, FPNavMeshPathfinder.MAX_CORRIDOR,
                "the fixture has to actually exceed the cap, or the refusal proves nothing");

            var ex = Assert.Throws<System.ArgumentException>(() => system.SetAbstractGraph(tooCoarse));
            StringAssert.Contains("corridor cap", ex.Message);
        }

        /// <summary>
        /// The boundary, which nothing used to stand on. The guard compared
        /// <c>MaxNodeDiameter</c> — a count of HOPS — against <c>CorridorCap</c>, a count of
        /// TRIANGLES, so a node sitting exactly on the cap was accepted and then produced a corridor
        /// two triangles too long: one because a path of d hops visits d+1 triangles, one because
        /// the leg aims at a point ON the portal and the endpoint lookup may resolve it to the
        /// triangle across it.
        ///
        /// <para><b>The cap is a tuning field, so the fixture is small.</b> No 129-triangle node is
        /// needed — the same boundary exists at any cap, and a tiny one makes it visible. The old
        /// tests only ever exceeded the cap by a lot, which is why this survived them.</para>
        /// </summary>
        [Test]
        public void AGraphExactlyOnTheCorridorCap_IsRefused_BecauseHopsAreNotTriangles()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);

            // Find a cell size whose widest node lands exactly on some cap, then set the cap there.
            var graph = new FPNavAbstractGraph(
                mesh, FP64.FromInt(8), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK);
            int hops = graph.MaxNodeDiameter;
            Assert.Greater(hops, 0, "fixture: a multi-triangle node");

            Assert.AreEqual(hops + 2, graph.MaxLegCorridorTriangles,
                "+1 for hops -> triangles, +1 for the far side of the portal");

            // Cap set to the DIAMETER: the old comparison read this as "fits exactly" and accepted.
            var tuning = new FPNavTuning(corridorCap: hops);
            var query = new FPNavMeshQuery(mesh, null, tuning);
            var system = new FPNavAgentSystem(
                mesh, query, new FPNavMeshPathfinder(mesh, query, null, tuning),
                new FPNavMeshFunnel(mesh, query, null, tuning), null, tuning);

            var ex = Assert.Throws<System.ArgumentException>(() => system.SetAbstractGraph(graph));
            StringAssert.Contains("corridor cap", ex.Message);

            // And the first cap that genuinely fits is the diameter plus two, not the diameter.
            var fits = new FPNavTuning(corridorCap: hops + 2);
            var q2 = new FPNavMeshQuery(mesh, null, fits);
            var ok = new FPNavAgentSystem(
                mesh, q2, new FPNavMeshPathfinder(mesh, q2, null, fits),
                new FPNavMeshFunnel(mesh, q2, null, fits), null, fits);
            Assert.DoesNotThrow(() => ok.SetAbstractGraph(graph),
                "two more than the diameter is exactly what a leg through it can ask for");
        }

        [Test]
        public void AGraphFromAnotherMesh_IsRefused()
        {
            // Node ids index the triangles of the mesh they were derived from. Installing a graph
            // built against a different one plans a route through geometry that is not there — and
            // every peer doing it would agree, so it would never surface as a desync.
            var mine = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var other = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var system = NavAgentTestHelper.CreateSystem(mine, null);

            var ex = Assert.Throws<System.ArgumentException>(() => system.SetAbstractGraph(
                new FPNavAbstractGraph(other, FP64.FromInt(8), FPNavAbstractCostFold.Min,
                    FPNavAgentSystem.DEFAULT_AREA_MASK)));
            StringAssert.Contains("different mesh", ex.Message);
        }

        [Test]
        public void AGraphWithinTheCap_IsAccepted_AndNullClearsIt()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var system = NavAgentTestHelper.CreateSystem(mesh, null);
            var graph = new FPNavAbstractGraph(mesh, FP64.FromInt(16), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK);

            Assert.LessOrEqual(graph.MaxNodeDiameter, FPNavMeshPathfinder.MAX_CORRIDOR);
            Assert.DoesNotThrow(() => system.SetAbstractGraph(graph));

            long withGraph = system.GetNavFingerprint();
            system.SetAbstractGraph(null);
            Assert.AreNotEqual(withGraph, system.GetNavFingerprint(),
                "clearing the graph must move the fingerprint back");
        }

        #endregion

        #region The failures this planner introduces are visible (V-12 · V-13)

        [Test]
        public void WhenNoNodeRouteExists_ItIsCountedRatherThanBlamedOnTheBudget()
        {
            // Two patches with no edge between them: the abstract search gives up before the
            // triangle search ever runs, so the pathfinder's budget counter cannot see it. If this
            // were not counted here, the failure would be invisible from every counter we have.
            var mesh = NavAgentTestHelper.CreateSplitFieldNavMesh(8, out var farCell);
            var query = new FPNavMeshQuery(mesh, null);
            FPVector3 start = NavAgentTestHelper.CellCenter(0, 0);
            FPVector3 goal = NavAgentTestHelper.CellCenter(farCell.gx, farCell.gz);
            Assert.GreaterOrEqual(query.FindTriangle(goal.ToXZ(), goal.y), 0, "the goal is on-mesh");

            var (system, frame, entity, entities) = Walker(mesh, start, goal, cellSize: 4.0);
            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);

            Assert.AreEqual(1, system.DebugAbstractSearchFailedCount,
                "the abstract search failed and said so");
            Assert.AreEqual((byte)FPNavAgentStatus.PathFailed,
                frame.GetReadOnly<NavAgentComponent>(entity).Status,
                "and the agent lands somewhere a caller can see");
        }

        [Test]
        public void ALegTheRealSearchCannotSolve_FallsBackInsteadOfStalling()
        {
            // Block the portal the abstract route wants after the graph was derived: the node graph
            // still claims the crossing, the triangle search refuses it. That is the shape both
            // D-3 (cost scales disagreeing) and a mis-built node produce, and neither is a desync.
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(8, 4, out var endCell);
            var (system, frame, entity, entities) = Walker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz), cellSize: 4.0);

            // Everything past the first run becomes unreachable, so whatever leg the graph picks
            // beyond it cannot be solved.
            for (int t = 0; t < mesh.Triangles.Length; t++)
                if (mesh.Triangles[t].centerXZ.y > FP64.FromInt(2))
                    mesh.TrianglesMutable[t].isBlocked = true;

            Walk(system, ref frame, entity, entities, maxTicks: 200);

            Assert.Greater(system.DebugLegResolveFailedCount, 0,
                "a leg the real search refuses must be counted, not swallowed");
        }

        #endregion
        #region V-A7 — the signal fires on "nothing shortened this search", not on "no graph"

        /// <summary>
        /// Same walker, with a budget small enough that the flat search runs out and an optional
        /// plan-mask override. The tight budget is the fixture: exhaustion on the shipped 4096 needs
        /// a mesh far larger than anything a unit test should build per case.
        /// </summary>
        private static (FPNavAgentSystem system, Frame frame, EntityRef entity, EntityRef[] entities)
            TightBudgetWalker(FPNavMesh mesh, FPVector3 start, FPVector3 destination,
                double cellSize, int maxIterations, int planMask = 0, IKLogger logger = null)
        {
            // Automatic install off: this walker installs a graph itself when cellSize > 0 and is
            // the FLAT fixture when it is 0, and a tight budget is exactly what would make the 0.13
            // default install one on its own.
            var tuning = new FPNavTuning(maxIterations: maxIterations, autoInstallAbstractGraph: false);
            var query = new FPNavMeshQuery(mesh, logger, tuning);
            var pathfinder = new FPNavMeshPathfinder(mesh, query, logger, tuning);
            var funnel = new FPNavMeshFunnel(mesh, query, logger, tuning);
            var system = new FPNavAgentSystem(mesh, query, pathfinder, funnel, logger, tuning);

            if (cellSize > 0)
                system.SetAbstractGraph(new FPNavAbstractGraph(
                    mesh, FP64.FromDouble(cellSize), FPNavAbstractCostFold.Min,
                    FPNavAgentSystem.DEFAULT_AREA_MASK));

            int tri = query.FindTriangle(start.ToXZ(), start.y);
            Assert.GreaterOrEqual(tri, 0, "the walker starts on the mesh");

            var frame = NavAgentTestHelper.CreateFrameWithAgent(start, tri, out var entity, out var entities);
            ref var nav = ref frame.Get<NavAgentComponent>(entity);
            if (planMask != 0)
                NavAgentComponent.SetAreaMask(ref nav, planMask: planMask, walkMask: planMask);
            NavAgentComponent.SetDestination(ref nav, destination);
            return (system, frame, entity, entities);
        }

        /// <summary>
        /// <b>No graph and a search that ran out: the signal fires.</b> The plain case, and the one
        /// that was already knowable from the budget counter alone.
        /// </summary>
        [Test]
        public void ExhaustionWithNoGraph_RaisesTheSignal()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out var endCell);
            var (system, frame, entity, entities) = TightBudgetWalker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                cellSize: 0, maxIterations: 8);

            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);

            Assert.AreEqual(1, system.DebugExhaustedWithoutLegsCount,
                "a full-distance flat search that ran out of budget is the case this signal is for");
        }

        /// <summary>
        /// <b>A graph is installed, and the signal still fires.</b> The case the obvious predicate
        /// (<i>exhausted AND no graph</i>) would have thrown away — and the one most likely to be
        /// live in a real game, because it looks like a healthy leg-planning build from outside.
        ///
        /// <para>The agent plans under <c>ALL_AREAS</c> while the graph was derived under
        /// <c>DEFAULT_AGENT_MASK</c>. That is a legal, quiet configuration: the mask permits strictly
        /// more, so nothing about the flat search breaks — it simply plans the whole way, on the
        /// budget, with the hierarchy standing right there not being used.</para>
        /// </summary>
        [Test]
        public void ExhaustionWhileAGraphIsInstalledButUnused_StillRaisesTheSignal()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out var endCell);
            var (system, frame, entity, entities) = TightBudgetWalker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                cellSize: 4.0, maxIterations: 8, planMask: FPNavMeshAreas.ALL_AREAS);

            Assert.AreNotEqual(FPNavAgentSystem.DEFAULT_AREA_MASK, FPNavMeshAreas.ALL_AREAS,
                "the fixture only means anything while the two masks differ");

            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);

            Assert.AreEqual(1, system.DebugMaskFallbackCount, "fixture: the graph was skipped");
            Assert.AreEqual(1, system.DebugExhaustedWithoutLegsCount,
                "an installed graph is not the predicate — what matters is whether this search was "
                + "the one the graph shortened, and this one was not");
        }

        /// <summary>
        /// <b>Start and goal in one node: no signal, even though the search ran out.</b> The
        /// hierarchy looked and had nothing to shorten, which is the opposite of not being consulted.
        /// This is the half of the old combined branch that had to be split back out.
        /// </summary>
        [Test]
        public void ExhaustionInsideASingleNode_RaisesNoSignal()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out var endCell);
            var (system, frame, entity, entities) = TightBudgetWalker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                cellSize: 4096.0, maxIterations: 8);

            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);

            Assert.AreEqual(FPNavAgentStatus.PathFailed, (FPNavAgentStatus)
                frame.GetReadOnly<NavAgentComponent>(entity).Status,
                "fixture: the budget must actually have run out, or this proves nothing");
            Assert.Zero(system.DebugExhaustedWithoutLegsCount,
                "one cell over the whole mesh puts both ends in the same node — the graph answered, "
                + "and telling the reader to install one would be telling them to do what they did");
        }

        /// <summary>
        /// <b>No exhaustion, no signal</b> — with or without a graph. The conjunction has to bind on
        /// both sides or the counter becomes noise nobody reads, which is how this feature's
        /// predecessor got missed.
        /// </summary>
        [TestCase(0.0)]
        [TestCase(4.0)]
        public void WithoutExhaustion_ThereIsNoSignal(double cellSize)
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out var endCell);
            var (system, frame, entity, entities) = TightBudgetWalker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                cellSize, maxIterations: FPNavMeshPathfinder.MAX_ITERATIONS);

            Walk(system, ref frame, entity, entities);

            Assert.Zero(system.DebugExhaustedWithoutLegsCount,
                "the search never ran out, so there is nothing to report whatever the graph did");
        }

        /// <summary>
        /// <b>The counter is not the signal — the line is.</b> Every other gate here reads
        /// <c>DebugExhaustedWithoutLegsCount</c>, so deleting the <c>KWarning</c> call would leave
        /// the whole suite green while the one thing a game actually notices disappears. That is the
        /// exact shape of the defect this feature exists to remove, which makes leaving it ungated
        /// the wrong kind of irony.
        ///
        /// <para>The wording is asserted too, not just the count: <b>exhaustion does not mean a
        /// route was there to find</b>, and a line that said "turn legs on and this is fixed" would
        /// be telling the reader something nobody measured.</para>
        /// </summary>
        [Test]
        public void TheSignal_IsAlsoALine_AndItDoesNotOverclaim()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out var endCell);
            var log = new LogCapture();
            var (system, frame, entity, entities) = TightBudgetWalker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                cellSize: 0, maxIterations: 8, logger: log);

            system.Update(ref frame, entities, entities.Length, 1, NavAgentTestHelper.DT);

            Assert.AreEqual(1, system.DebugExhaustedWithoutLegsCount, "fixture: the signal fired");
            Assert.IsTrue(log.Contains(KLogLevel.Warning, "ran out of its 8-triangle budget"),
                "the counter rising with nothing said is how the predecessor of this signal went "
                + "unread for an entire session");
            Assert.IsTrue(log.Contains(KLogLevel.Warning, "not the same as there being no route"),
                "exhaustion says the search stopped before it decided, not that a route exists — "
                + "the line has to keep that distinction or it is guessing on the reader's behalf");
        }

        /// <summary>
        /// <b>Once per system, then silence.</b> Eight hundred agents repathing every tick would
        /// bury the sentence in its own repetition; the counter carries the rest.
        /// </summary>
        [Test]
        public void TheWarning_IsSaidOnce_AndThenOnlyCounted()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 4, out var endCell);
            var log = new LogCapture();
            var (system, frame, entity, entities) = TightBudgetWalker(
                mesh, NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                cellSize: 0, maxIterations: 8, logger: log);

            // Re-issuing the destination each tick is what makes the failure recur: a plan that
            // fails leaves the agent without one, and the repath cooldown otherwise spaces the
            // retries out past this loop.
            for (int tick = 1; tick <= 200; tick++)
            {
                ref var nav = ref frame.Get<NavAgentComponent>(entity);
                NavAgentComponent.SetDestination(
                    ref nav, NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz));
                system.Update(ref frame, entities, entities.Length, tick, NavAgentTestHelper.DT);
            }

            Assert.Greater(system.DebugExhaustedWithoutLegsCount, 1,
                "fixture: the failure has to recur, or 'once' proves nothing");
            Assert.AreEqual(1, log.CountAt(KLogLevel.Warning),
                "the second occurrence onward is counted, not logged");
        }

        #endregion

        #region What a swap costs to rebuild the graph

        /// <summary>
        /// A swap re-derives the whole graph, on the deterministic command path, and until now said
        /// nothing about it. Two observables come out of P0 and they are deliberately different in
        /// kind: the COUNT is always on, because the frequency is what decides whether the cost is
        /// worth moving and an increment is free; the TIMING is opt-in, because reading the clock on
        /// that path should not happen for a diagnostic nobody asked for.
        /// </summary>
        [Test]
        public void ASwapRederivesTheGraph_CountedAlways_TimedOnlyWhenAsked()
        {
            var meshA = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var meshB = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var log = new LogCapture();
            var system = NavAgentTestHelper.CreateSystem(meshA, log);
            system.SetAbstractGraph(new FPNavAbstractGraph(
                meshA, FP64.FromInt(8), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK));

            Assert.AreEqual(0, system.DebugGraphRederiveCount, "nothing has swapped yet");

            // Off: the swap still re-derives, and still counts it.
            system.SwapNavMesh(meshB);
            Assert.AreEqual(1, system.DebugGraphRederiveCount, "the count does not need the switch");
            Assert.IsFalse(log.Contains(KLogLevel.Information, "re-derived on the swap"),
                "the clock is not read for a diagnostic nobody asked for");

            // On: the same swap now says what it cost.
            system.DebugTimeGraphDerivation = true;
            system.SwapNavMesh(meshA);
            Assert.AreEqual(2, system.DebugGraphRederiveCount);
            Assert.IsTrue(log.Contains(KLogLevel.Information, "re-derived on the swap in"),
                "asked for, so said — with the number, the cell size and the swap ordinal");
            Assert.IsTrue(log.Contains(KLogLevel.Information, "swap #2"),
                "the ordinal is what turns one line into a rate");
        }

        /// <summary>
        /// The count stays zero on a game that installs no graph — which is most of them, and the
        /// reason this cost went unnoticed until a code review went looking.
        /// </summary>
        [Test]
        public void WithNoGraph_ASwapCostsNothingToRederive()
        {
            var meshA = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var meshB = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var system = NavAgentTestHelper.CreateSystem(meshA, null);
            system.DebugTimeGraphDerivation = true;

            system.SwapNavMesh(meshB);

            Assert.AreEqual(0, system.DebugGraphRederiveCount,
                "no graph, nothing to re-derive — the switch cannot make work appear");
        }

        #endregion

        #region Preparing the graph off-tick, and adopting it at the swap

        private static (FPNavAgentSystem system, FPNavMesh a, FPNavMesh b) SwapPair(double cell = 8)
        {
            var meshA = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var meshB = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var system = NavAgentTestHelper.CreateSystem(meshA, null);
            system.SetAbstractGraph(new FPNavAbstractGraph(
                meshA, FP64.FromDouble(cell), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK));
            return (system, meshA, meshB);
        }

        /// <summary>
        /// V-2 — <b>the whole safety argument, as one assertion.</b> Moving the derivation off the
        /// tick is only free if the graph that arrives is the graph that would have arrived. If this
        /// ever fails, peers that prepared and peers that did not are running different partitions
        /// while their state hashes agree.
        /// </summary>
        [Test]
        public void APreparedGraph_IsTheSameGraphASynchronousDerivationWouldGive()
        {
            var (prepared, _, meshB) = SwapPair();
            var (synchronous, _, otherB) = SwapPair();

            prepared.PrepareAbstractGraphFor(meshB);
            prepared.SwapNavMesh(meshB);
            synchronous.SwapNavMesh(otherB);

            Assert.AreEqual(1, prepared.DebugGraphPreparedAdoptedCount, "fixture: it adopted");
            Assert.AreEqual(0, prepared.DebugGraphRederiveCount, "and paid nothing on the tick");
            Assert.AreEqual(1, synchronous.DebugGraphRederiveCount, "fixture: the control derived");

            Assert.AreEqual(synchronous.CurrentAbstractGraph.Checksum,
                prepared.CurrentAbstractGraph.Checksum,
                "same inputs, same graph — the only difference preparing may make is WHEN");
            Assert.AreEqual(synchronous.GetNavFingerprint(), prepared.GetNavFingerprint(),
                "and therefore the same fingerprint, which is what a peer refuses on");
        }

        /// <summary>
        /// V-3 — a system that never prepares must be indistinguishable from the one that existed
        /// before any of this. The golden it is checked against lives in <c>FPNavTuningTests</c> and
        /// was pinned BEFORE the change, which is the only moment such a value can be captured.
        /// </summary>
        [Test]
        public void NeverPreparing_LeavesTheSystemExactlyAsItWas()
        {
            var (system, _, meshB) = SwapPair();
            system.SwapNavMesh(meshB);

            Assert.AreEqual(0, system.DebugGraphPreparedAdoptedCount);
            Assert.AreEqual(0, system.DebugGraphPreparedMissedCount,
                "a miss is only counted when something WAS prepared — silence must stay silent");
            Assert.AreEqual(0, system.DebugGraphInstancesCreated,
                "and no buffer is allocated for a feature the game never called");
            Assert.AreEqual(1, system.DebugGraphRederiveCount, "it derived, exactly as before");
        }

        /// <summary>
        /// V-5 — identity is REQUIRED, not preferred: <c>SetAbstractGraph</c> enforces that a
        /// graph's mesh is the system's mesh, because node ids index one mesh's triangles. A graph
        /// prepared against an identical-but-separate mesh is therefore not installable, however
        /// equal the contents.
        /// </summary>
        [Test]
        public void APreparationForAnotherInstance_IsNotAdopted_EvenWithIdenticalContent()
        {
            var (system, _, meshB) = SwapPair();
            var twin = NavAgentTestHelper.CreateOpenFieldNavMesh(8);   // same content, other object

            system.PrepareAbstractGraphFor(twin);
            system.SwapNavMesh(meshB);

            Assert.AreEqual(0, system.DebugGraphPreparedAdoptedCount);
            Assert.AreEqual(1, system.DebugGraphPreparedMissedCount, "it was there and did not fit");
            Assert.AreEqual(1, system.DebugGraphRederiveCount, "so the tick paid, as it must");
        }

        /// <summary>
        /// V-6 — <b>the check the first draft of the plan did not have.</b> The rebake driver pools
        /// meshes and retires the one a commit replaces, so a reference can be recycled and
        /// rewritten. Adopting on identity alone would install a graph indexing geometry that is no
        /// longer there, identically on every peer, with the state hash agreeing — the worst shape
        /// in this family. The fingerprint is what closes it.
        /// </summary>
        [Test]
        public void APreparationWhoseMeshWasRewritten_IsNotAdopted()
        {
            var (system, _, meshB) = SwapPair();

            system.PrepareAbstractGraphFor(meshB);

            // Recycling, in miniature: the same instance, different content.
            meshB.TrianglesMutable[0].isBlocked = true;

            system.SwapNavMesh(meshB);

            Assert.AreEqual(0, system.DebugGraphPreparedAdoptedCount,
                "the reference matched and the content did not — identity alone would have taken it");
            Assert.AreEqual(1, system.DebugGraphPreparedMissedCount);
            Assert.AreEqual(1, system.DebugGraphRederiveCount);
        }

        /// <summary>
        /// V-7 — the double buffer, asserted where it can actually run. The natural way to prepare
        /// is a fresh graph each time, which throws away what <c>Rebind</c> exists for and leaves a
        /// graph's worth of garbage per rebake. Byte-level gates in this repository are skipped by
        /// default, so the invariant is counted instead.
        /// </summary>
        [Test]
        public void PreparingRepeatedly_AllocatesOneGraphEver()
        {
            var (system, meshA, meshB) = SwapPair();

            for (int i = 0; i < 8; i++)
            {
                FPNavMesh next = (i % 2 == 0) ? meshB : meshA;
                system.PrepareAbstractGraphFor(next);
                system.SwapNavMesh(next);
            }

            Assert.AreEqual(8, system.DebugGraphPreparedAdoptedCount, "fixture: every swap adopted");
            Assert.AreEqual(0, system.DebugGraphRederiveCount, "and none of them paid on the tick");
            Assert.AreEqual(1, system.DebugGraphInstancesCreated,
                "one spare, forever — the pair alternates and nothing else is built");
        }

        /// <summary>
        /// Preparing is free to be wrong about the future and free to be called too often: the
        /// heartbeat runs every frame, and the mesh it sees sits there until a tick takes it.
        /// </summary>
        [Test]
        public void PreparingIsIdempotent_AndIgnoresNullAndTheLiveMesh()
        {
            var (system, meshA, meshB) = SwapPair();

            system.PrepareAbstractGraphFor(null);
            system.PrepareAbstractGraphFor(meshA);          // already live
            Assert.AreEqual(0, system.DebugGraphInstancesCreated, "neither is work");

            for (int i = 0; i < 5; i++)
                system.PrepareAbstractGraphFor(meshB);      // the same one, five frames running

            system.SwapNavMesh(meshB);
            Assert.AreEqual(1, system.DebugGraphPreparedAdoptedCount);
            Assert.AreEqual(1, system.DebugGraphInstancesCreated, "five calls, one derivation");
        }

        /// <summary>With no graph installed there is nothing to prepare, and asking costs nothing.</summary>
        [Test]
        public void WithNoGraph_PreparingIsANoOp()
        {
            var meshA = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var meshB = NavAgentTestHelper.CreateOpenFieldNavMesh(8);
            var system = NavAgentTestHelper.CreateSystem(meshA, null);

            system.PrepareAbstractGraphFor(meshB);
            system.SwapNavMesh(meshB);

            Assert.AreEqual(0, system.DebugGraphInstancesCreated);
            Assert.AreEqual(0, system.DebugGraphPreparedAdoptedCount);
            Assert.AreEqual(0, system.DebugGraphPreparedMissedCount);
        }

        #endregion

        #region The reach radius against the node (F-2's signal)

        /// <summary>
        /// A leg the agent was already standing on top of when it was planned. The reach radius is
        /// <c>v² / a</c> — the tightest arc the agent can hold — and nothing bounds it against the
        /// node width, so past that width every leg target is "reached" on the tick it is chosen.
        /// The hand-off then clears the repath cooldown, and the agent runs a full A* every tick
        /// instead of once per leg: the cost this feature exists to remove, reintroduced.
        ///
        /// <para><b>Why a counter of its own.</b> <c>DebugLegAdvanceRepeatCount</c> cannot say this
        /// — it rises about once per leg in healthy operation AND about once per leg here, so its
        /// ratio to <c>DebugLegAdvanceCount</c> is 1:1 either way. This one is what separates them:
        /// a leg that takes even one tick to walk never lands here.</para>
        /// </summary>
        [Test]
        public void ALegThatEndedOnItsPlanTick_IsCounted_AndTheRadiusIsSaidOnce()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var log = new LogCapture();
            var system = NavAgentTestHelper.CreateSystem(mesh, log);
            system.SetAbstractGraph(new FPNavAbstractGraph(
                mesh, FP64.FromDouble(4.0), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK));

            FPVector3 start = NavAgentTestHelper.CellCenter(0, 0);
            var query = new FPNavMeshQuery(mesh, null);
            var frame = NavAgentTestHelper.CreateFrameWithAgent(
                start, query.FindTriangle(start.ToXZ(), start.y), out var entity, out var entities);

            ref var nav = ref frame.Get<NavAgentComponent>(entity);
            // Fast and slow to turn: the ceiling is 10² / 1 = 100 against a node 4 wide. The speed
            // has to be REACHED, not just configured — ReachRadius reads CurrentSpeed — so the
            // walk needs runway before the radius swallows a node.
            nav.Speed = FP64.FromInt(10);
            nav.Acceleration = FP64.One;
            NavAgentComponent.SetDestination(ref nav, NavAgentTestHelper.CellCenter(11, 11));

            for (int tick = 1; tick <= 2000; tick++)
            {
                system.Update(ref frame, entities, entities.Length, tick, NavAgentTestHelper.DT);
                if (frame.Get<NavAgentComponent>(entity).Status == (byte)FPNavAgentStatus.Arrived)
                    break;
            }

            Assert.Greater(system.DebugLegAdvanceCount, 0, "fixture: the agent crossed nodes");
            Assert.Greater(system.DebugLegEndedOnPlanTickCount, 0,
                "a radius of up to 100 against a node 4 wide reaches its leg target on the plan tick");
            Assert.IsTrue(log.Contains(KLogLevel.Warning, "wider than a node"),
                "the counter rising with nothing said is how this class of defect goes unread");
            Assert.AreEqual(1, log.CountAt(KLogLevel.Warning),
                "said once per system: the mismatch holds for the whole run, and repeating it every "
                + "tick would bury the log of the match it explains");
        }

        /// <summary>
        /// The control, and the reason the counter is worth reading: with the radius well inside a
        /// node, legs advance and NONE of them ends on the tick it was planned.
        /// </summary>
        [Test]
        public void WithTheRadiusInsideTheNode_NoLegEndsOnItsPlanTick()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(12);
            var log = new LogCapture();
            var system = NavAgentTestHelper.CreateSystem(mesh, log);
            // Default agent: 5² / 10 = 2.5 against a node 8 wide.
            system.SetAbstractGraph(new FPNavAbstractGraph(
                mesh, FP64.FromDouble(8.0), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK));

            FPVector3 start = NavAgentTestHelper.CellCenter(0, 0);
            var query = new FPNavMeshQuery(mesh, null);
            var frame = NavAgentTestHelper.CreateFrameWithAgent(
                start, query.FindTriangle(start.ToXZ(), start.y), out var entity, out var entities);
            ref var nav = ref frame.Get<NavAgentComponent>(entity);
            NavAgentComponent.SetDestination(ref nav, NavAgentTestHelper.CellCenter(11, 11));

            Walk(system, ref frame, entity, entities);

            Assert.Greater(system.DebugLegAdvanceCount, 0, "fixture: the agent crossed nodes");
            Assert.AreEqual(0, system.DebugLegEndedOnPlanTickCount,
                "every leg took at least a tick to walk");
            Assert.AreEqual(0, log.CountAt(KLogLevel.Warning), "and nothing to warn about");
        }

        #endregion

    }
}
