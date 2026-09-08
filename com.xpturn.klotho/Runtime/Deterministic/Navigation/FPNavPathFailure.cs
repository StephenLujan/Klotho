using xpTURN.Klotho.Deterministic.Math;

namespace xpTURN.Klotho.Deterministic.Navigation
{
    /// <summary>
    /// Why an agent sits at <see cref="FPNavAgentStatus.PathFailed"/>.
    ///
    /// <para>The order of the members is the order <see cref="FPNavMeshPathfinder.FindPath"/>
    /// refuses in, and <see cref="FPNavPathFailure.Diagnose"/> walks it in that order. Renumbering
    /// them would silently reorder the diagnosis.</para>
    /// </summary>
    public enum FPNavPathFailureReason
    {
        None,
        AgentOffMesh,
        AgentOnBlockedGround,
        DestinationOffMesh,
        DestinationBlocked,
        DestinationAreaMasked,
        /// <summary>The failure predates the mesh now installed: its cause is gone.</summary>
        StaleFailure,
        /// <summary>
        /// Every endpoint check passes and the diagnosis had no pathfinder to ask which of the two
        /// remaining causes it was. <see cref="NoRoute"/> and <see cref="BudgetExhausted"/> are the
        /// split; this is what a caller gets that cannot afford, or was not given, the re-search.
        /// </summary>
        NoRouteOrBudget,
        /// <summary>The search drained its open set: no route joins the agent to its destination.</summary>
        NoRoute,
        /// <summary>
        /// The search ran out of its iteration budget. With partial paths off that is the whole
        /// story; with them on it means the closest node the budget reached was no closer to the
        /// goal than the agent already stands — a pocket that faces the goal.
        /// </summary>
        BudgetExhausted,
    }

    /// <summary>
    /// Names the cause of a <see cref="FPNavAgentStatus.PathFailed"/> without a counter, by
    /// re-walking the refusals <see cref="FPNavMeshPathfinder.FindPath"/> makes.
    ///
    /// <para><b>Public, and in the runtime, for the same reason
    /// <see cref="FPNavMeshQuery.FindTriangleForEndpoint"/> is.</b> Both editor tools need this,
    /// and they are not one assembly: Unity's is fixed and could have been named in
    /// <c>InternalsVisibleTo</c>, but the Godot adapter ships as SOURCE inside
    /// <c>addons/klotho/Adapters/</c> and is compiled into whatever assembly the consuming Godot
    /// project happens to be — including projects outside this repository. There is no list to
    /// write, so <c>internal</c> cannot reach it. Writing the diagnosis twice instead is what this
    /// type replaced: the two copies had already drifted in their comments.</para>
    ///
    /// <para><b>Not a simulation input.</b> Nothing here is called from the tick; the members read
    /// state and never write it. <see cref="Diagnose"/> resolves the endpoint with
    /// <see cref="FPNavMeshQuery.FindTriangleForEndpoint"/> — the same member the engine uses — so
    /// a tool cannot report a mask refusal the engine never made.</para>
    /// </summary>
    public static class FPNavPathFailure
    {
        /// <summary>
        /// The cause, or <see cref="FPNavPathFailureReason.None"/> when the agent is not failed or
        /// the caller has no mesh yet (a tool asks before a load).
        ///
        /// <para>This is <see cref="DiagnoseEndpoints"/> followed, only when that answers
        /// <see cref="FPNavPathFailureReason.NoRouteOrBudget"/> and a pathfinder was given, by
        /// <see cref="SearchVerdict"/>. The two halves are public on their own because they cost
        /// differently: the endpoint checks are a few lookups and must be re-asked against the mesh
        /// as it is NOW, while the search pops up to <c>MaxIterations</c> triangles and answers a
        /// question whose inputs (the agent's position and destination, the mesh) do not change
        /// while the agent sits at <c>PathFailed</c> — so a tool that repaints asks the first half
        /// every time and the second half once.</para>
        /// </summary>
        /// <param name="nav">The agent to explain.</param>
        /// <param name="query">Query over <paramref name="mesh"/>; null answers None.</param>
        /// <param name="mesh">The mesh the agent is on; null answers None.</param>
        /// <param name="failurePredatesSwap">
        /// Whether this failure was already standing when the current mesh was installed. The
        /// caller owns that bookkeeping — it is the only way to tell
        /// <see cref="FPNavPathFailureReason.StaleFailure"/> from
        /// <see cref="FPNavPathFailureReason.NoRouteOrBudget"/>, which look identical from here.
        /// </param>
        /// <param name="pathfinder">
        /// <b>A pathfinder of the tool's own</b>, or null. Given one, the last two verdicts are told
        /// apart by searching again from where the agent stands: a drained open set is
        /// <see cref="FPNavPathFailureReason.NoRoute"/>, a spent budget is
        /// <see cref="FPNavPathFailureReason.BudgetExhausted"/>. The search overwrites that
        /// instance's corridor buffer and moves its counters, which is why it must never be the
        /// engine's — nor the instance an editor shares with its agent system and its Find Path
        /// preview, which IS the engine's; and it must be built with the engine's tuning, or it
        /// answers a different question. A re-search that FINDS a path cannot be explained from
        /// here (the stack it was asked on disagrees with the one that failed) and reports
        /// <see cref="FPNavPathFailureReason.NoRouteOrBudget"/>.
        /// </param>
        public static FPNavPathFailureReason Diagnose(
            in NavAgentComponent nav, FPNavMeshQuery query, FPNavMesh mesh, bool failurePredatesSwap,
            FPNavMeshPathfinder pathfinder = null)
            => Diagnose(in nav, query, mesh, failurePredatesSwap, pathfinder,
                FP64.FromDouble(FPNavAgentSystem.DEFAULT_WAYPOINT_THRESHOLD));

        /// <summary>
        /// <see cref="Diagnose(in NavAgentComponent, FPNavMeshQuery, FPNavMesh, bool, FPNavMeshPathfinder)"/>
        /// with the agent system's <see cref="FPNavAgentSystem.WaypointThreshold"/>, so the
        /// re-search asks for the same minimum progress the engine did. The two differ only when a
        /// system changed the threshold AND the agent's <c>Speed² / Acceleration</c> is below it
        /// (the threshold is the floor of <see cref="FPNavAgentSystem.PartialMinProgress"/>); a
        /// tool that runs its own agent system passes the system's field, a tool that has none
        /// uses the other overload.
        /// </summary>
        public static FPNavPathFailureReason Diagnose(
            in NavAgentComponent nav, FPNavMeshQuery query, FPNavMesh mesh, bool failurePredatesSwap,
            FPNavMeshPathfinder pathfinder, FP64 waypointThreshold)
        {
            FPNavPathFailureReason reason = DiagnoseEndpoints(in nav, query, mesh, failurePredatesSwap);
            if (reason != FPNavPathFailureReason.NoRouteOrBudget || pathfinder == null)
                return reason;
            return SearchVerdict(in nav, pathfinder, waypointThreshold);
        }

        /// <summary>
        /// The half of <see cref="Diagnose"/> that needs no search: the agent's footing, then the
        /// endpoint in <c>FindPath</c>'s order, then the swap marker. Answers
        /// <see cref="FPNavPathFailureReason.NoRouteOrBudget"/> when every check passes — the
        /// caller then either stops there or asks <see cref="SearchVerdict"/>.
        ///
        /// <para>Cheap, and judged against the mesh as it is now. A tool re-asks this on every
        /// repaint: a rebake can turn a standing failure stale, and a fresh destination can move
        /// the endpoint off the mesh, without the agent's status changing.</para>
        /// </summary>
        public static FPNavPathFailureReason DiagnoseEndpoints(
            in NavAgentComponent nav, FPNavMeshQuery query, FPNavMesh mesh, bool failurePredatesSwap)
        {
            if (nav.Status != (byte)FPNavAgentStatus.PathFailed)
                return FPNavPathFailureReason.None;
            if (query == null || mesh == null)
                return FPNavPathFailureReason.None;

            // The agent's own footing first: the reseed-lost case is the one no counter explains.
            if (nav.CurrentTriangleIndex < 0)
                return FPNavPathFailureReason.AgentOffMesh;
            if (mesh.Triangles[nav.CurrentTriangleIndex].isBlocked)
                return FPNavPathFailureReason.AgentOnBlockedGround;

            // Then the endpoint, in FindPath's order: lookup, isBlocked, mask. The START's mask is
            // deliberately absent — the engine exempts it and only reports it, so naming it as a
            // cause would be a lie.
            int mask = FPNavAgentSystem.ResolvePlanMask(nav);
            int endTri = query.FindTriangleForEndpoint(
                nav.Destination.ToXZ(), nav.Destination.y, mask);
            if (endTri < 0)
                return FPNavPathFailureReason.DestinationOffMesh;

            ref readonly var endTriangle = ref mesh.Triangles[endTri];
            if (endTriangle.isBlocked)
                return FPNavPathFailureReason.DestinationBlocked;
            if ((mask & endTriangle.areaMask) == 0)
                return FPNavPathFailureReason.DestinationAreaMasked;

            // Every endpoint check passes. Either the cause is gone (the mesh changed under a
            // failure that was never re-planned) or the search itself failed. Only the first is
            // knowable from state — hence the swap marker; the second needs a search to split.
            if (failurePredatesSwap)
                return FPNavPathFailureReason.StaleFailure;
            return FPNavPathFailureReason.NoRouteOrBudget;
        }

        /// <summary>
        /// The half of <see cref="Diagnose"/> that searches: splits
        /// <see cref="FPNavPathFailureReason.NoRouteOrBudget"/> into
        /// <see cref="FPNavPathFailureReason.NoRoute"/> or
        /// <see cref="FPNavPathFailureReason.BudgetExhausted"/> by running the engine's call again
        /// from where the agent stands. Meant to follow a <see cref="DiagnoseEndpoints"/> that
        /// answered NoRouteOrBudget; asked about an agent that is not <c>PathFailed</c> it answers
        /// None, and with no pathfinder it answers NoRouteOrBudget, so a caller that skipped the
        /// first half still gets nothing false.
        ///
        /// <para>Its inputs do not change while the agent stands there — a <c>PathFailed</c> agent
        /// does not move and is not re-planned — so the answer can be kept until the status, the
        /// destination or the mesh changes. See the <paramref name="pathfinder"/> remarks on
        /// <see cref="Diagnose"/> for whose instance this may be.</para>
        /// </summary>
        public static FPNavPathFailureReason SearchVerdict(
            in NavAgentComponent nav, FPNavMeshPathfinder pathfinder)
            => SearchVerdict(in nav, pathfinder, FP64.FromDouble(FPNavAgentSystem.DEFAULT_WAYPOINT_THRESHOLD));

        /// <summary>
        /// <see cref="SearchVerdict(in NavAgentComponent, FPNavMeshPathfinder)"/> with the agent
        /// system's <see cref="FPNavAgentSystem.WaypointThreshold"/> — see the matching
        /// <c>Diagnose</c> overload for when the two differ.
        /// </summary>
        public static FPNavPathFailureReason SearchVerdict(
            in NavAgentComponent nav, FPNavMeshPathfinder pathfinder, FP64 waypointThreshold)
        {
            if (nav.Status != (byte)FPNavAgentStatus.PathFailed)
                return FPNavPathFailureReason.None;
            if (pathfinder == null)
                return FPNavPathFailureReason.NoRouteOrBudget;

            // The same call the engine made, from where the agent stands (a PathFailed agent does
            // not move), toward the destination — the last search that fails an agent is always the
            // flat one to the destination, because a leg search that fails falls back to it — and
            // with the same minimum progress the engine asked for, which is the caller's threshold
            // under the agent's own turning radius.
            int mask = FPNavAgentSystem.ResolvePlanMask(nav);
            int exhaustedBefore = pathfinder.DebugIterationExhaustedCount;
            bool found = pathfinder.FindPath(
                nav.Position, nav.Destination, mask,
                FPNavAgentSystem.PartialMinProgress(in nav, waypointThreshold),
                out _, out _, out _, out _);
            if (found)
                return FPNavPathFailureReason.NoRouteOrBudget;
            return pathfinder.DebugIterationExhaustedCount != exhaustedBefore
                ? FPNavPathFailureReason.BudgetExhausted
                : FPNavPathFailureReason.NoRoute;
        }

        /// <summary>
        /// The reason as a suffix a tool can append to an agent row. Empty for
        /// <see cref="FPNavPathFailureReason.None"/> — a caller appends unconditionally.
        ///
        /// <para>These strings live here rather than in each tool so the two engines cannot
        /// describe the same state differently. That makes the wording a contract: changing it is
        /// a CHANGELOG entry.</para>
        /// </summary>
        public static string Describe(FPNavPathFailureReason reason)
        {
            switch (reason)
            {
                case FPNavPathFailureReason.AgentOffMesh:
                    return " ← agent is off the mesh (a rebake left it there; nothing re-acquires it)";
                case FPNavPathFailureReason.AgentOnBlockedGround:
                    return " ← the agent stands on blocked ground";
                case FPNavPathFailureReason.DestinationOffMesh:
                    return " ← the destination is off the mesh";
                case FPNavPathFailureReason.DestinationBlocked:
                    return " ← the destination is blocked (closed in code, not by a mask)";
                case FPNavPathFailureReason.DestinationAreaMasked:
                    return " ← the destination's area is outside this agent's plan mask";
                case FPNavPathFailureReason.StaleFailure:
                    return " ← stale: nothing blocks it now (the mesh changed) — set the destination again";
                case FPNavPathFailureReason.NoRouteOrBudget:
                    return " ← no route (or the search budget ran out)";
                case FPNavPathFailureReason.NoRoute:
                    return " ← no route joins the agent to its destination";
                // Mode-neutral on purpose: whether partial paths were on is the caller's to say (an
                // editor knows its agent system's tuning), and a parenthetical about a mode the
                // reader is not in reads as a description of a different failure.
                case FPNavPathFailureReason.BudgetExhausted:
                    return " ← the search budget ran out before it could decide";
                default:
                    return "";
            }
        }
    }
}
