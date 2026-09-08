using System;
using xpTURN.Klotho.Logging;

using xpTURN.Klotho.Deterministic.Math;

namespace xpTURN.Klotho.Deterministic.Navigation
{
    /// <summary>
    /// A* triangle graph search.
    /// Holds an FPNavMesh reference and performs zero-GC search using pre-allocated arrays.
    /// </summary>
    public class FPNavMeshPathfinder
    {
        private FPNavMesh _navMesh;
        private readonly FPNavMeshQuery _query;
        private readonly IKLogger _logger;

        // Pre-allocated A* buffers
        private FPNavMeshBinaryHeap _openSet;
        private FP64[] _gScores;
        private int[] _cameFrom;
        private bool[] _closed;
        private int[] _nodeGeneration;
        private FPVector2[] _entryPoints;
        private int _generation;

        // Corridor result buffer
        private readonly int[] _corridor;

        /// <remarks><b>Defaults, not this instance's values</b> — the search is sized from
        /// <see cref="FPNavTuning.CorridorCap"/> and <see cref="FPNavTuning.MaxIterations"/>.
        /// <c>MAX_CORRIDOR</c> is also the compile-time ceiling for the search buffer, which the
        /// cap is validated against.</remarks>
        public const int MAX_CORRIDOR = 128;
        /// <inheritdoc cref="MAX_CORRIDOR"/>
        public const int MAX_ITERATIONS = 4096;

        // Diagnostic counters. Deliberately never reset: they are totals, and a caller that wants
        // a delta takes two readings. What "lifetime" means across a navmesh swap depends on which
        // overload the swap went through, and the two differ — SwapNavMesh(mesh) rebinds THIS
        // instance (Rebind resizes buffers and leaves these alone), so the totals carry across a
        // runtime rebake, which is the path FPNavAgentInstaller takes and what you want in a match
        // that rebakes repeatedly. The four-argument overload installs a caller-built pathfinder,
        // so the totals start again from zero there. FPNavAgentSystem's own counter survives both,
        // being on the system rather than here.
        private int _corridorTruncatedCount;
        private int _iterationExhaustedCount;
        private int _blockedEndpointCount;
        private int _areaMaskRejectedCount;
        private int _maskedStartCount;
        private int _partialPathCount;
        private int _partialRejectedCount;

        // Per-call: what the most recent FindPath handed back (see DebugLastPathWasPartial).
        private bool _lastPartial;

        // The node that got closest to the goal during the current search, by the heuristic the
        // search itself computes at push time (Detour's m_lastBestNode). Ties break toward the lower
        // triangle index. This is deterministic — same mesh, same endpoints, same answer on every
        // peer — but it is not a pure function of the mesh: the heuristic is taken at the node's
        // ENTRY edge, and a later, cheaper visit through a different edge moves that entry point
        // without raising _bestH, so a node can stay best on the strength of an edge it no longer
        // enters by. TryPartialPath re-measures the winner at its centre before trusting it.
        private int _bestNode;
        private FP64 _bestH;

        /// <summary>
        /// Diagnostic: paths whose triangle chain was longer than <see cref="MAX_CORRIDOR"/> and
        /// therefore returned truncated (the end-side triangles are dropped, so the agent runs off
        /// the corridor end and repaths). Accumulated over this instance's lifetime, never reset;
        /// outside the state hash.
        /// </summary>
        public int DebugCorridorTruncatedCount => _corridorTruncatedCount;

        /// <summary>
        /// Diagnostic: <c>FindPath</c> calls that failed with search work still queued — i.e. the
        /// <see cref="MAX_ITERATIONS"/> budget ran out rather than the graph being exhausted. This
        /// is the half of the silent <c>false</c> that means "ask again with a shorter path", as
        /// opposed to "there is genuinely no route". Accumulated over this instance's lifetime.
        /// </summary>
        public int DebugIterationExhaustedCount => _iterationExhaustedCount;

        private int _lastSearchIterations;

        /// <summary>
        /// Triangles the MOST RECENT <c>FindPath</c> CALL popped, against
        /// <see cref="FPNavTuning.MaxIterations"/>. <b>Zero when that call did not search</b> — an
        /// endpoint off the mesh, a blocked or masked endpoint, or a start and end in the same
        /// triangle all return before A* begins, and zero is what those spent.
        ///
        /// <para><b>It belongs to a CALL, not to an agent.</b> There is one of these per
        /// pathfinder, and a whole tick of agents shares one pathfinder — as does the visualizer's
        /// Find Path button. After <c>FPNavAgentSystem.Update</c> this holds whatever the last
        /// agent to plan spent, and that agent need not be one that failed. Anything reading it
        /// per agent is claiming an attribution that does not exist here.</para>
        ///
        /// <para><b>Not monotonic, unlike every other counter here</b> — it is overwritten by each
        /// call, so it means something only when read immediately after one. It exists because
        /// "the budget ran out" is not by itself actionable: the budget is spent in TRIANGLES, so
        /// whether 4096 is a lot depends entirely on how finely the mesh is cut. A tool that can say
        /// "4096 of 4096 popped to cover 22 units" turns a bare failure into a diagnosis — the
        /// direct line must be blocked, because an open 22 units would never cost that many.</para>
        ///
        /// <para>Diagnostic only: written during the search, read by tools, and never an input to
        /// one. It does not reach the hash, the wire, or a replay.</para>
        /// </summary>
        public int DebugLastSearchIterations => _lastSearchIterations;

        /// <summary>
        /// Diagnostic: calls rejected because the start or end triangle is flagged
        /// <c>isBlocked</c>. This is the one silent failure a game produces on purpose — nothing in
        /// the engine sets that flag, and the rebaker carves geometry away rather than blocking it,
        /// so a non-zero count means someone closed a triangle through
        /// <c>FPNavMesh.TrianglesMutable</c> (a gate, a door) and units are now being ordered
        /// through it. The search never starts, so this is not a budget problem.
        /// </summary>
        public int DebugBlockedEndpointCount => _blockedEndpointCount;

        /// <summary>
        /// Diagnostic: calls rejected because the requested <c>areaMask</c> shares no bit with the
        /// <b>end</b> triangle — a destination on ground the mask forbids.
        ///
        /// <para>The START is not counted here and no longer refuses the call at all; it is exempt,
        /// and <see cref="DebugMaskedStartCount"/> reports it instead. Before that exemption this
        /// counter conflated the two, and the start case is the one a game is likely to care about
        /// (a unit standing on ground it may not use), so they are split.</para>
        /// </summary>
        public int DebugAreaMaskRejectedCount => _areaMaskRejectedCount;

        /// <summary>
        /// Diagnostic: searches that STARTED on ground the requested <c>areaMask</c> forbids.
        ///
        /// <para>Not a failure — the search runs, and the escape rule lets it leave the forbidden
        /// region it began in. It is here because the alternative is silence: the agent gets an
        /// ordinary route out and nothing in the result says it was ever stuck inside a building.
        /// A game that wants to notice that (to play a rescue, to refund, to warn) has this and
        /// nothing else. Same conventions as the other counters — outside the state hash, the wire
        /// and replay, never reset, and a resimulated tick counts again.</para>
        /// </summary>
        public int DebugMaskedStartCount => _maskedStartCount;

        /// <summary>
        /// Diagnostic: searches that ran out of budget and returned a <b>partial</b> corridor — the
        /// chain to the node that got closest to the goal — because
        /// <see cref="FPNavTuning.PartialPathOnExhaustion"/> is on and that node made progress.
        /// Every one of these also counts in <see cref="DebugIterationExhaustedCount"/>: the budget
        /// did run out; what changed is what the caller got. Same conventions as the other counters.
        /// </summary>
        public int DebugPartialPathCount => _partialPathCount;

        /// <summary>
        /// Diagnostic: searches that ran out of budget with partial paths ON and still returned
        /// <c>false</c>, because the closest node was no closer than the start by the caller's
        /// minimum. This is the "no progress" half of exhaustion — a unit standing in a pocket that
        /// faces the goal — and the only way a partial-path search fails other than "no route".
        /// </summary>
        public int DebugPartialRejectedCount => _partialRejectedCount;

        /// <summary>
        /// Whether the MOST RECENT <c>FindPath</c> call returned a partial corridor. Per call, not
        /// per agent, like <see cref="DebugLastSearchIterations"/>: read it immediately after the
        /// call whose result you are labelling. A caller of the five-argument overload reads this to
        /// say that the path stops short of the goal; a caller that also needs WHERE it stops uses
        /// the eight-argument overload, whose <c>partialEnd</c> is the only source of that point.
        /// </summary>
        public bool DebugLastPathWasPartial => _lastPartial;

        private readonly FPNavTuning _tuning;

        /// <summary>The sizes this instance was built with (see <see cref="FPNavTuning"/>).</summary>
        public FPNavTuning Tuning => _tuning;

        public FPNavMeshPathfinder(FPNavMesh navMesh, FPNavMeshQuery query, IKLogger logger,
            FPNavTuning? tuning = null)
        {
            _navMesh = navMesh;
            _query = query;
            _logger = logger;

            _tuning = tuning ?? FPNavTuning.Default;
            _tuning.Validate();

            int triCount = navMesh.Triangles.Length;
            _openSet = new FPNavMeshBinaryHeap(triCount);
            _gScores = new FP64[triCount];
            _cameFrom = new int[triCount];
            _closed = new bool[triCount];
            _nodeGeneration = new int[triCount];
            _entryPoints = new FPVector2[triCount];
            _generation = 0;
            _corridor = new int[_tuning.CorridorCap];
        }

        /// <summary>
        /// Points this pathfinder at a different mesh, keeping the working arrays.
        /// Part of a navmesh swap — see FPNavAgentSystem.SwapNavMesh, which is the only thing
        /// that should call it.
        /// </summary>
        internal void Rebind(FPNavMesh newMesh)
        {
            _navMesh = newMesh;

            int triCount = newMesh.Triangles.Length;
            if (triCount > _gScores.Length)
            {
                // All of these were sized together, so they run out together. The heap is rebuilt
                // through its constructor rather than patched: it fills _positions with -1, and a
                // hand-grown one would be all zeros — which Contains reads as "in the heap at
                // slot 0", making every triangle look like it is already queued.
                var openSet = new FPNavMeshBinaryHeap(triCount);
                var gScores = new FP64[triCount];
                var cameFrom = new int[triCount];
                var closed = new bool[triCount];
                var nodeGeneration = new int[triCount];
                var entryPoints = new FPVector2[triCount];

                // Allocate first, install after — see FPNavMeshQuery.Rebind.
                _openSet = openSet;
                _gScores = gScores;
                _cameFrom = cameFrom;
                _closed = closed;
                _nodeGeneration = nodeGeneration;
                _entryPoints = entryPoints;
            }

            // _generation is deliberately left alone — see FPNavMeshQuery.Rebind for why. Reset()
            // pre-increments and the constructor starts at 0, so 0 is the permanent "never
            // touched" sentinel; a grown array reads as untouched for free.
        }

        /// <summary>
        /// A* pathfinding. Returns the corridor (triangle index sequence).
        /// </summary>
        /// <param name="start">Start 3D position</param>
        /// <param name="end">Target 3D position</param>
        /// <param name="areaMask">Area filter mask</param>
        /// <param name="corridor">Resulting corridor array. Warning: this is a reference to the internal buffer and is overwritten on the next FindPath call. Consume it immediately or copy it.</param>
        /// <param name="corridorLength">Corridor length</param>
        /// <returns>Whether a path was found</returns>
        /// <remarks>
        /// With <see cref="FPNavTuning.PartialPathOnExhaustion"/> on, <c>true</c> may also mean a
        /// PARTIAL corridor — one that stops at the node that got closest to the goal before the
        /// budget ran out. This overload accepts any strictly positive progress; callers that need
        /// to know, or that want a minimum, use the eight-argument overload, and tools that only
        /// draw read <see cref="DebugLastPathWasPartial"/> right after the call.
        /// </remarks>
        public bool FindPath(FPVector3 start, FPVector3 end, int areaMask,
            out int[] corridor, out int corridorLength)
            => FindPath(start, end, areaMask, FP64.Zero, out corridor, out corridorLength, out _, out _);

        /// <summary>
        /// <see cref="FindPath(FPVector3, FPVector3, int, out int[], out int)"/>, reporting whether
        /// the corridor is partial and where it ends.
        ///
        /// <para><b>A partial corridor is a corridor to the node that got closest to the goal</b>
        /// (by the search's own heuristic, ties to the lower triangle index) when the iteration
        /// budget ran out with work still queued. It is handed back only when that node is closer
        /// to the goal than the start by MORE than <paramref name="partialMinProgress"/> — measured
        /// at the node's centre, after the search, so a stale heuristic cannot vouch for it. A
        /// caller walking an agent passes the agent's reach radius here, which is what makes the
        /// re-plan at the corridor's end start from somewhere new; a tool passes zero.</para>
        ///
        /// <para><b>The corridor is clamped like any other</b> (agent side kept), so
        /// <paramref name="partialEnd"/> is the centre of the corridor's LAST triangle — the point
        /// the agent will actually walk to — which is the best node only when the chain fit. The
        /// progress test is on the best node regardless: measured over the shipped assets, judging
        /// the clipped end instead refused every winding route (a serpentine's clipped end lies
        /// further from the goal than its start), and judging the best node reached the goal in
        /// every one of those with no cycle in 541 chains.</para>
        ///
        /// <para><b>When the chain is clipped, the corridor ends at the triangle of the kept prefix
        /// that lies FARTHEST from the start</b>, not at the cap. The best node is always outside
        /// the caller's reach radius (its progress is at least that radius, and progress cannot
        /// exceed distance), but a cap cut at a fixed count can land anywhere along a chain that
        /// doubles back — on a switchback it landed straight across the wall from the agent, inside
        /// the radius at full speed, so the hand-off fired on the tick the plan was made and the
        /// cleared cooldown planned again the next tick, a full budget per tick until the agent's
        /// own motion carried the end away. Cutting at the farthest point of the prefix keeps a
        /// valid corridor (a prefix of a valid chain), drops exactly the part that came back, and on
        /// a switchback puts the end at the turn. Unclipped partials are untouched.</para>
        ///
        /// <para><b>A goal already in the open set when the budget runs out is a whole path</b>, not
        /// a partial: the chain behind it is real, only not yet proven shortest. The exhaustion is
        /// still counted. Without this the best-node rule handed back a partial to the triangle
        /// beside the goal (its entry edge is usually nearer the goal point than the goal triangle's
        /// own) and the agent spent one more hop and one more full-budget search to finish.</para>
        ///
        /// <para>Graph exhaustion (open set drained) is still <c>false</c>: "there is no route" keeps
        /// its answer. Everything that returns before the search starts is unchanged too, and so is
        /// everything with the switch off.</para>
        /// </summary>
        public bool FindPath(FPVector3 start, FPVector3 end, int areaMask, FP64 partialMinProgress,
            out int[] corridor, out int corridorLength, out bool partial, out FPVector3 partialEnd)
        {
            corridor = _corridor;
            corridorLength = 0;
            partial = false;
            partialEnd = default;
            _lastPartial = false;
            // Cleared per CALL, not per search. Four of the returns below leave without searching,
            // and leaving the field alone there let a tool print an older, unrelated search as this
            // failure's evidence. Zero is what actually happened: no triangle was popped.
            // Writing it here rather than at each early return is the same behaviour — every path
            // out either returns before the loop or writes the real count on the way out.
            _lastSearchIterations = 0;

            // Triangle lookup that considers Y height. The END breaks height ties toward ground
            // this query may use: a snapped destination sits on a triangle EDGE, which belongs to
            // both neighbours at the same interpolated height, and the plain lookup would hand back
            // whichever has the lower index — refusing, half the time, a destination the projection
            // had just certified as walkable. The tie is broken only between candidates on the SAME
            // SURFACE, so the multi-floor answer is untouched (two floors are equidistant at the
            // midpoint between them, and that is not a tie this rule takes), and where every
            // same-surface candidate is passable it degenerates to the plain lookup's answer.
            //
            // The START keeps the plain lookup on purpose: its mask check is an exemption whose
            // whole value is being reported (DebugMaskedStartCount), and resolving an ambiguous
            // start toward walkable ground would silence that at exactly the boundary positions it
            // exists to report.
            // Moving either lookup changes the corridor for unchanged inputs — bump
            // FPNavAgentSystem.NAV_BEHAVIOUR_REVISION with it, or old and new builds diverge silently.
            int startTri = _query.FindTriangle(start.ToXZ(), start.y);
            int endTri = _query.FindTriangleForEndpoint(end.ToXZ(), end.y, areaMask);

            if (startTri < 0 || endTri < 0)
            {
                if (startTri < 0)
                    _logger?.KError($"[FindPath] start={start} is outside NavMesh (startTri=-1)");

                if (endTri < 0)
                    _logger?.KError($"[FindPath] end={end} is outside NavMesh (endTri=-1)");

                return false;
            }

            // The two rejections below happen before the search starts, so counting them costs
            // nothing on the hot path — and without the counters they are indistinguishable from
            // "no route exists", which is the failure they most look like from the outside.
            if (_navMesh.Triangles[startTri].isBlocked || _navMesh.Triangles[endTri].isBlocked)
            {
                _blockedEndpointCount++;
                return false;
            }

            // The END is still refused: a destination on ground the mask forbids has no answer,
            // and that is the half of the filter callers rely on.
            if ((areaMask & _navMesh.Triangles[endTri].areaMask) == 0)
            {
                _areaMaskRejectedCount++;
                return false;
            }

            // The START is exempt, matching the walk, which never gates the triangle an agent is
            // already standing on. Refusing it produced a unit that could not be given a route out
            // of ground it had been placed on — a building dropped on top of it, most concretely —
            // and since the agent system only walks along a corridor, no corridor meant no
            // movement at all. Counted separately so the situation stays observable: the endpoint
            // counter above would otherwise fall silent on the one case a game most wants to see.
            if ((areaMask & _navMesh.Triangles[startTri].areaMask) == 0)
                _maskedStartCount++;

            // Same triangle
            if (startTri == endTri)
            {
                _corridor[0] = startTri;
                corridorLength = 1;
                return true;
            }

            // A* initialization
            Reset();

            FPVector2 endXZ = end.ToXZ();
            TouchNode(startTri);
            _entryPoints[startTri] = start.ToXZ();
            FP64 h = FPVector2.Distance(start.ToXZ(), endXZ);
            _gScores[startTri] = FP64.Zero;
            _cameFrom[startTri] = -1;
            _openSet.Push(startTri, h);
            _bestNode = startTri;
            _bestH = h;

            int iterations = 0;

            while (_openSet.Count > 0 && iterations < _tuning.MaxIterations)
            {
                iterations++;
                int current = _openSet.Pop();

                if (current == endTri)
                {
                    _lastSearchIterations = iterations;
                    corridorLength = ReconstructCorridor(current);
                    return corridorLength > 0;
                }

                _closed[current] = true;

                // Iterate in neighbor0, neighbor1, neighbor2 order (deterministic)
                for (int e = 0; e < 3; e++)
                {
                    int neighbor = _navMesh.Triangles[current].GetNeighbor(e);
                    if (neighbor < 0)
                        continue;
                    if (IsClosed(neighbor))
                        continue;
                    if (_navMesh.Triangles[neighbor].isBlocked)
                        continue;
                    // The escape rule, identical to the walk's (FPNavMeshQuery.MoveAlongSurface):
                    // a neighbour the mask refuses is still expandable when the node we expand FROM
                    // is refused too, so a path can leave forbidden ground but never enter it.
                    //
                    // Admissibility therefore depends on the parent, which normally breaks a closed
                    // set — the same node would be reachable through one parent and not another.
                    // It does not break here, and the reason is worth keeping: the rule forbids
                    // accepted -> refused, so a refused node is only ever reached from a refused
                    // parent, chaining back to the start. The refused nodes in the search are
                    // exactly the start's own connected refused component, and every accepted
                    // node's admissibility is parent-independent as before. No (node, inside) state
                    // pairing is needed. Observed by
                    // FPNavMaskedStartEscapeTests.E7_TheEscapeIsConfinedToTheRegionYouStandIn.
                    if ((areaMask & _navMesh.Triangles[neighbor].areaMask) == 0
                        && (areaMask & _navMesh.Triangles[current].areaMask) != 0)
                        continue;

                    TouchNode(neighbor);

                    _navMesh.Triangles[current].GetEdgeVertices(e, out int va, out int vb);
                    FPVector2 edgeMid = (_navMesh.Vertices[va].ToXZ() + _navMesh.Vertices[vb].ToXZ()) * FP64.Half;

                    FP64 edgeCost = FPVector2.Distance(_entryPoints[current], edgeMid)
                        * _navMesh.Triangles[neighbor].costMultiplier;

                    FP64 tentativeG = _gScores[current] + edgeCost;

                    if (_openSet.Contains(neighbor))
                    {
                        if (tentativeG < _gScores[neighbor])
                        {
                            _gScores[neighbor] = tentativeG;
                            _cameFrom[neighbor] = current;
                            _entryPoints[neighbor] = edgeMid;
                            FP64 hN = FPVector2.Distance(edgeMid, endXZ);
                            FP64 f = tentativeG + hN;
                            _openSet.DecreaseKey(neighbor, f);
                            TrackBest(neighbor, hN);
                        }
                    }
                    else
                    {
                        _gScores[neighbor] = tentativeG;
                        _cameFrom[neighbor] = current;
                        _entryPoints[neighbor] = edgeMid;
                        FP64 hN = FPVector2.Distance(edgeMid, endXZ);
                        FP64 f = tentativeG + hN;
                        _openSet.Push(neighbor, f);
                        TrackBest(neighbor, hN);
                    }
                }
            }

            // Budget exhaustion vs a genuinely exhausted graph. The discriminator is the OPEN SET,
            // not the iteration count: a search whose last pop empties the set on exactly the
            // MAX_ITERATIONS-th iteration completed its work and found nothing, and counting that
            // as a truncation would report a budget problem that does not exist.
            _lastSearchIterations = iterations;
            if (_openSet.Count == 0)
                return false;

            _iterationExhaustedCount++;
            if (!_tuning.PartialPathOnExhaustion)
                return false;

            // The goal was reached but not yet popped: the budget ran out inside the window between
            // the push that discovered it and the pop that would have finished (about ten pops on
            // the shipped Field asset, one on a switchback). The chain behind it is a real path —
            // every cameFrom edge was an expansion — just not yet proven shortest, which a partial
            // never is either. Hand it back whole rather than as a partial to the triangle beside
            // it, which is what the best-node rule would do: the goal's heuristic is measured at its
            // entry edge, and a neighbour's edge midpoint is usually nearer the goal point than
            // that, so the goal itself was the best node in only one such search in six. The
            // exhaustion stays counted — the budget did run out. Under the switch only: with it off
            // this search answered false before this change, and turning that into a path would
            // move the frame hash without a NAV_BEHAVIOUR_REVISION bump.
            if (_openSet.Contains(endTri))
            {
                corridorLength = ReconstructCorridor(endTri);
                return corridorLength > 0;
            }

            return TryPartialPath(startTri, start.ToXZ(), endXZ, partialMinProgress,
                out corridorLength, out partial, out partialEnd);
        }

        /// <summary>
        /// Push-time tracking of the node closest to the goal, using the heuristic the search has
        /// just computed for it — no extra work on the hot path. Strictly smaller wins; an equal
        /// value wins only with a smaller index, so a TIE does not depend on the order the heap
        /// happened to expand in. The value itself still can: <c>_bestH</c> only ever falls, so a
        /// node re-parented through a farther edge (DecreaseKey moves its entry point) keeps the
        /// smaller heuristic it was first pushed with. Deterministic, and at worst a partial end
        /// that is not the closest available — the golden tests pin the result as it is.
        /// </summary>
        private void TrackBest(int node, FP64 h)
        {
            if (h < _bestH || (h == _bestH && node < _bestNode))
            {
                _bestH = h;
                _bestNode = node;
            }
        }

        /// <summary>
        /// The exhausted-with-partials-on tail of <c>FindPath</c>. Judges the BEST node's progress
        /// (re-measured at its centre — the tracked heuristic was taken at an entry point that a
        /// later, cheaper visit may have moved), then hands back the clamped chain to it.
        /// </summary>
        private bool TryPartialPath(int startTri, FPVector2 startXZ, FPVector2 endXZ, FP64 minProgress,
            out int corridorLength, out bool partial, out FPVector3 partialEnd)
        {
            corridorLength = 0;
            partial = false;
            partialEnd = default;

            int best = _bestNode;
            if (best == startTri)
            {
                _partialRejectedCount++;
                return false;
            }

            FP64 progress = FPVector2.Distance(startXZ, endXZ)
                - FPVector2.Distance(_navMesh.Triangles[best].centerXZ, endXZ);
            if (progress <= FP64.Zero || progress < minProgress)
            {
                _partialRejectedCount++;
                return false;
            }

            corridorLength = ReconstructCorridor(best);
            if (corridorLength == 0)
            {
                // A malformed cameFrom chain; ReconstructCorridor's own guard tripped.
                _partialRejectedCount++;
                return false;
            }

            // Clipped (the chain did not reach the best node): end at the prefix's farthest
            // triangle from the start rather than at the cap. See the FindPath remarks — a cut at
            // a fixed count can land where a doubled-back chain passes the agent again, inside the
            // reach radius, and that is a hand-off on the plan tick. Strictly farther wins, so an
            // equal distance keeps the earlier triangle; index 0 is the start's own triangle and is
            // never chosen over a later one, so a clipped corridor is always at least two long.
            if (_corridor[corridorLength - 1] != best)
            {
                int farthest = 0;
                FP64 farthestDist = FP64.Zero;
                for (int i = 0; i < corridorLength; i++)
                {
                    FP64 d = FPVector2.Distance(startXZ, _navMesh.Triangles[_corridor[i]].centerXZ);
                    if (d > farthestDist)
                    {
                        farthestDist = d;
                        farthest = i;
                    }
                }
                if (farthest > 0)
                    corridorLength = farthest + 1;
            }

            partialEnd = TriangleCentre(_corridor[corridorLength - 1]);
            partial = true;
            _lastPartial = true;
            _partialPathCount++;
            return true;
        }

        /// <summary>The triangle's centre in 3D: the baked XZ centre at the mean height of its vertices.</summary>
        private FPVector3 TriangleCentre(int tri)
        {
            ref readonly FPNavMeshTriangle t = ref _navMesh.Triangles[tri];
            FP64 y = (_navMesh.Vertices[t.v0].y + _navMesh.Vertices[t.v1].y + _navMesh.Vertices[t.v2].y)
                / FP64.FromInt(3);
            return new FPVector3(t.centerXZ.x, y, t.centerXZ.y);
        }

        private void Reset()
        {
            _openSet.Clear();
            _generation++;
            if (_generation == int.MaxValue)
            {
                // Generation 0 is the permanent "never touched" sentinel, so wrap to 1, not 0.
                // Only _nodeGeneration needs clearing — TouchNode rewrites the other four
                // whenever it sees a generation mismatch, so they follow from this one.
                Array.Clear(_nodeGeneration, 0, _nodeGeneration.Length);
                _generation = 1;
            }
        }


        private void TouchNode(int idx)
        {
            if (_nodeGeneration[idx] != _generation)
            {
                _nodeGeneration[idx] = _generation;
                _gScores[idx] = FP64.MaxValue;
                _cameFrom[idx] = -1;
                _closed[idx] = false;
                _entryPoints[idx] = FPVector2.Zero;
            }
        }

        private bool IsClosed(int idx)
        {
            return _nodeGeneration[idx] == _generation && _closed[idx];
        }

        private int ReconstructCorridor(int endTri)
        {
            // cameFrom walks end -> start. Count the full chain first so that, on overflow,
            // we can skip the triangles nearest the destination and keep the ones nearest
            // the agent's actual start triangle instead. Keeping the end-side segment (the
            // previous behavior) produces a corridor that never touches the agent's current
            // triangle, which desyncs corridor-advance tracking in FPNavAgentSystem.
            // The totalLength bound guards against a malformed (cyclic) cameFrom chain.
            int totalLength = 0;
            int node = endTri;
            while (node >= 0 && totalLength <= _cameFrom.Length)
            {
                totalLength++;
                node = _cameFrom[node];
            }

            int corridorCap = _tuning.CorridorCap;
            int skip = totalLength > corridorCap ? totalLength - corridorCap : 0;
            if (skip > 0)
                _corridorTruncatedCount++;

            int count = 0;
            int index = 0;
            node = endTri;
            while (node >= 0 && count < corridorCap)
            {
                if (index >= skip)
                {
                    _corridor[count] = node;
                    count++;
                }
                index++;
                node = _cameFrom[node];
            }

            // Reverse into start -> end order (returns the collected partial path even on overflow)
            for (int i = 0; i < count / 2; i++)
            {
                int tmp = _corridor[i];
                _corridor[i] = _corridor[count - 1 - i];
                _corridor[count - 1 - i] = tmp;
            }

            return count;
        }
    }
}
