using UnityEditor;
using UnityEngine;

using xpTURN.Klotho.Deterministic.Math;
using xpTURN.Klotho.Deterministic.Navigation;
using xpTURN.Klotho.ECS;

namespace xpTURN.Klotho.Editor
{
    /// <summary>
    /// Simulates NavMesh agent pathfinding in the editor.
    /// Uses a lightweight Frame + NavAgentComponent.
    /// </summary>
    internal class FPNavMeshAgentSimulator
    {
        public const int MAX_AGENTS = 32;

        // Simulation state
        public bool IsRunning;
        public int CurrentTick;
        public float SimulationSpeed = 1.0f;

        // Default agent settings
        public float DefaultSpeed = 5.0f;
        public float DefaultRadius = 0.5f;
        public float DefaultAcceleration = 10.0f;
        public bool EnableAvoidance = true;
        // Bake inset carried by the loaded mesh boundary. Synced from the asset's recorded
        // BakeAgentRadius on Initialize; edit live (SetObstacleRadiusInset) to override for
        // diagnosis (0 = uncorrected double clearance, reproducing the edge slowdown).
        public float ObstacleRadiusInset = 0f;
        // How far a destination click may be moved to reach ground the agent's plan mask allows.
        // Synced to the mesh's GridCellSize on Initialize, which is the distance the projection
        // ALWAYS covers: its fallback searches the click's cell plus the 8 around it, so anything
        // within one cell of the click is certain to be considered. Larger values still help but
        // depend on where in its cell the click landed, and past ~2.83 cells (the 3x3 block's far
        // corner) there is nothing left to find.
        public float DestinationSnapMaxDist = 1.0f;

        // Why the last destination click was refused, or null. Held here rather than logged
        // because the Godot simulator has no status log, and because the symptom is a lasting
        // state ("it will not move") rather than a one-shot event. NOT per agent: cleared on the
        // next successful destination so a stale reason cannot linger after another agent works.
        public string LastDestinationRefusal;

        // For ORCA visualization
        public FPNavAvoidance Avoidance => _avoidance;
        public int LastOrcaComputedAgentIndex { get; private set; } = -1;

        public int AgentCount => _entityCount;

        // Internal
        private Frame _simFrame;
        private EntityRef[] _entities = new EntityRef[MAX_AGENTS];
        private int _entityCount;

        private FPNavAgentSystem _agentSystem;
        private FPNavAvoidance _avoidance;
        private FPNavMeshVisualizerData _data;
        private double _lastUpdateTime;
        private double _accumulator;
        private readonly FP64 _dt = FP64.FromDouble(1.0 / 60.0);
        private const double FIXED_DT = 1.0 / 60.0;

        // Remember initial positions (for reset)
        private Vector3[] _initialPositions = new Vector3[MAX_AGENTS];

        // Whether an agent's PathFailed predates the mesh currently installed. Set by reading the
        // status AFTER a swap, which is enough: the reseed revives Moving/PathPending/Blocked but
        // NOT PathFailed, and ProcessPathRequest refuses to re-plan a PathFailed agent — so
        // anything still sitting there crossed the swap. Distinguishing that from a genuine
        // "no route" is impossible from the endpoints alone (both leave every endpoint check
        // passing), which is why this is a timestamp rather than another geometry query.
        private bool[] _failurePredatesSwap = new bool[MAX_AGENTS];

        // The pathfinder the diagnosis searches with. NOT _data.Pathfinder: that instance is the
        // agent system's (Initialize hands it over) and the Find Path preview's, and
        // FPNavPathFailure.Diagnose says why it must not be — the re-search overwrites its corridor
        // buffer and moves its counters, so every repaint added an exhaustion to "budget exhausted"
        // that no agent caused. Built with the engine's tuning so it asks the same question, and
        // over its own query so it owes nothing to the order the data layer rebinds its trio in.
        // Rebuilt on a swap rather than rebound: FPNavMeshPathfinder.Rebind is internal to the
        // runtime and this assembly is not on its list.
        private FPNavMeshPathfinder _diagPathfinder;

        // The search half of the diagnosis, kept per agent. FPNavPathFailure.SearchVerdict's inputs
        // do not change while an agent stands at PathFailed, so it is asked once — on the transition
        // when StepOnce sees it, or on the first repaint after — and the endpoint half is re-asked
        // live. None = not asked yet. Cleared wherever an input changes: the status leaves
        // PathFailed, a new destination, a swap. A parallel array with the same slot rules as
        // _failurePredatesSwap: reset on add, slid on remove, cleared on clear.
        private FPNavPathFailureReason[] _failedReason = new FPNavPathFailureReason[MAX_AGENTS];

        public void Initialize(FPNavMeshVisualizerData data)
        {
            _data = data;
            if (data == null || !data.IsLoaded) return;

            _simFrame = new Frame(MAX_AGENTS, null);

            _agentSystem = new FPNavAgentSystem(
                data.NavMesh, data.Query, data.Pathfinder, data.Funnel, data.Logger);
            BuildDiagnosisPathfinder(data.NavMesh, data.Logger);
            System.Array.Clear(_failedReason, 0, _failedReason.Length);

            _avoidance = new FPNavAvoidance();
            // Load the NavMesh boundary as ORCA static obstacles once (retained on _avoidance across
            // enable/disable toggles), so the ORCA-lines overlay shows wall half-planes, not just
            // agent-agent ones. Set avoidance to load, then honor the EnableAvoidance toggle.
            // LoadNavMeshObstacles applies the asset's recorded bake radius as the obstacle inset;
            // sync the knob so the UI shows the applied value (still editable as an override).
            _agentSystem.SetAvoidance(_avoidance);
            _agentSystem.LoadNavMeshObstacles();
            ObstacleRadiusInset = _avoidance.ObstacleRadiusInset.ToFloat();
            if (!EnableAvoidance)
                _agentSystem.SetAvoidance(null);

            DestinationSnapMaxDist = data.NavMesh.GridCellSize.ToFloat();
            LastDestinationRefusal = null;

            // Read the graph off the system rather than assuming a fresh one has none: since 0.13
            // the constructor installs one itself on a mesh past the search budget
            // (FPNavTuning.AutoInstallAbstractGraph). The window and the overlay show what THIS
            // field says, so a stale value here is the state where the panel says one thing and the
            // simulation does another — the class of lie this whole wiring exists to remove.
            _abstractGraph = _agentSystem.AbstractGraph;
            _abstractGraphIsAutomatic = _abstractGraph != null;

            CurrentTick = 0;
            _accumulator = 0;
            _lastUpdateTime = EditorApplication.timeSinceStartup;
        }

        #region Planning in legs

        private FPNavAbstractGraph _abstractGraph;
        private bool _abstractGraphIsAutomatic;

        /// <summary>The installed graph, or null. Read for the stats the window shows.</summary>
        public FPNavAbstractGraph AbstractGraph => _abstractGraph;

        /// <summary>
        /// Whether the graph in use is the one the agent system installed itself at construction
        /// (0.13 default) rather than one Apply installed. Display only — the two are the same kind of
        /// graph — but the panel should say which decision it is showing.
        /// </summary>
        public bool AbstractGraphIsAutomatic => _abstractGraph != null && _abstractGraphIsAutomatic;

        /// <summary>
        /// The corridor cap the stats line should quote — read from the agent system this tool
        /// runs, not from <see cref="FPNavMeshPathfinder.MAX_CORRIDOR"/>. The constant is the
        /// compile-time ceiling; the TUNING is what refuses a graph
        /// (<see cref="FPNavAgentSystem.SetAbstractGraph"/>) and what truncates a corridor, and
        /// quoting the other one shows a number nothing enforces.
        ///
        /// <para>Exposed here rather than read off <c>_data.Pathfinder</c> because that one goes
        /// null while a graph is still installed: the play-mode bridge calls
        /// <c>FPNavMeshVisualizerData.LoadFromNavMesh</c>, which nulls the pathfinder and funnel
        /// without re-initialising this simulator. A graph, on the other hand, can only be
        /// installed through <c>_agentSystem</c>, so it is non-null wherever one exists.</para>
        /// </summary>
        public int CorridorCap => _agentSystem != null
            ? _agentSystem.Tuning.CorridorCap
            : FPNavMeshPathfinder.MAX_CORRIDOR;

        /// <summary>
        /// Derives a graph over the loaded mesh and installs it, or explains why it will not.
        ///
        /// <para><b>A refusal is a value here, not an exception.</b> <c>SetAbstractGraph</c> throws
        /// on a node wider than the corridor cap, and that is a value the cell-size field reaches
        /// on purpose — raise the size far enough on any real mesh and you get there. An exception
        /// out of <c>OnGUI</c> aborts that repaint's layout and repeats every frame while the state
        /// stands, so the two numbers that would let you FIX it (the diameter and the cap) would
        /// scroll past in the console instead of sitting on screen. Same shape as the placement
        /// probe's <c>PlacementUnsupportedReason</c>.</para>
        ///
        /// <para>Derivation is not free — measured at 12.6 ms on the Field asset at cell 16 — so
        /// this belongs on a button, never on a slider drag or anything OnGUI reaches every
        /// repaint.</para>
        /// </summary>
        public bool TryInstallAbstractGraph(
            float cellSize, FPNavAbstractCostFold fold, int areaMask, out string reason)
        {
            reason = null;
            if (_agentSystem == null || _data == null || !_data.IsLoaded)
            {
                reason = "Load a NavMesh first.";
                return false;
            }
            if (cellSize <= 0f)
            {
                reason = "Cell size must be positive.";
                return false;
            }

            FPNavAbstractGraph graph;
            try
            {
                graph = new FPNavAbstractGraph(
                    _data.NavMesh, FP64.FromFloat(cellSize), fold, areaMask, _data.Logger);
            }
            catch (System.Exception ex)
            {
                reason = ex.Message;
                return false;
            }

            // Pre-checked rather than caught: the cap refusal is the one a user meets by turning a
            // dial, and what they need is the pair of numbers, not a stack trace.
            int cap = _agentSystem.Tuning.CorridorCap;
            if (graph.MaxLegCorridorTriangles > cap)
            {
                reason =
                    $"Cell {cellSize:F1} gives a widest node of {graph.MaxNodeDiameter} hops, so a "
                    + $"leg through it can ask for {graph.MaxLegCorridorTriangles} triangles "
                    + $"against a corridor cap of {cap}. A leg never leaves its node, so legs "
                    + $"through it would come back clamped. Use a smaller cell size.";
                return false;
            }

            // Installing mid-walk changes nothing until each agent replans, and an agent already
            // holding a corridor would keep walking the flat one — the tool would look like the
            // button did nothing. Same reasoning as OnBuildingPlaced: act on a tick boundary, then
            // hand every agent back to the planner so the next tick shows the new answer.
            bool wasRunning = IsRunning;
            if (wasRunning) Pause();

            _agentSystem.SetAbstractGraph(graph);
            _abstractGraph = graph;
            _abstractGraphIsAutomatic = false;
            ReplanAllAgents();

            if (wasRunning) Start();
            return true;
        }

        /// <summary>Removes the graph — the exact off switch, restoring the flat plan.</summary>
        public void ClearAbstractGraph()
        {
            if (_agentSystem == null) return;

            bool wasRunning = IsRunning;
            if (wasRunning) Pause();

            _agentSystem.SetAbstractGraph(null);
            _abstractGraph = null;
            ReplanAllAgents();

            if (wasRunning) Start();
        }

        /// <summary>
        /// Drops every agent's corridor so the next tick plans under whatever is installed now.
        /// Deliberately NOT <c>SetAreaMask</c>, which would do this and also write an override —
        /// the very thing that used to switch legs off by accident.
        /// </summary>
        private void ReplanAllAgents()
        {
            if (_simFrame == null) return;
            for (int i = 0; i < _entityCount; i++)
            {
                ref var nav = ref _simFrame.Get<NavAgentComponent>(_entities[i]);
                if (!nav.HasNavDestination) continue;
                nav.HasPath = false;
                nav.PathIsValid = false;
                nav.CorridorLength = 0;
                nav.OffCorridorTicks = 0;
                nav.LastRepathTick = 0;
                nav.Status = (byte)FPNavAgentStatus.PathPending;
            }
        }

        /// <summary>
        /// Diagnostic counters, read by the window. Monotonic and never reset — the agent-system
        /// ones start over only when <see cref="Initialize"/> builds a new system, the pathfinder
        /// ones only when the data layer loads a fresh mesh. A rebake clears NEITHER, which is what
        /// lets a placement be compared against the ticks before it.
        /// </summary>
        public (int abstractSearchFailed, int legResolveFailed, int legAdvance, int maskFallback,
                int corridorCopyTruncated, int corridorTruncated, int iterationExhausted,
                int exhaustedWithoutLegs, int legEndedOnPlanTick)
            ReadNavCounters()
        {
            if (_agentSystem == null) return default;
            var pf = _data?.Pathfinder;
            return (_agentSystem.DebugAbstractSearchFailedCount,
                    _agentSystem.DebugLegResolveFailedCount,
                    _agentSystem.DebugLegAdvanceCount,
                    _agentSystem.DebugMaskFallbackCount,
                    _agentSystem.DebugCorridorCopyTruncatedCount,
                    pf?.DebugCorridorTruncatedCount ?? 0,
                    pf?.DebugIterationExhaustedCount ?? 0,
                    _agentSystem.DebugExhaustedWithoutLegsCount,
                    _agentSystem.DebugLegEndedOnPlanTickCount);
        }

        /// <summary>
        /// What this mesh can do to a flat search, said at load rather than after a unit has already
        /// stopped. Null when neither failure is reachable — which is most stages, and saying
        /// nothing there is the point.
        ///
        /// <para><b>It says "can", not "will".</b> Both conditions are necessary, not sufficient: a
        /// mesh past the line makes the failure reachable, and whether any actual route reaches it
        /// depends on the shape. Wording it as a certainty would train the reader to ignore it on
        /// the stages where it never fires.</para>
        ///
        /// <para><b>And it does not install anything.</b> The tool has to be able to show legs off —
        /// this whole feature was found by looking at that state. Deriving on load would also take
        /// the choice away from the game, which is the one thing the engine deliberately does not
        /// do.</para>
        /// </summary>
        /// <param name="canExhaust">
        /// True when the budget line is the one that was crossed. That is the failure that leaves
        /// units standing still, so it is worth interrupting for; past only the corridor cap the
        /// cost is route quality, and shouting about it on every mesh over 128 triangles would
        /// teach the reader to ignore the notice before it ever mattered.
        /// </param>
        public string DescribeFlatPlanningRisk(out bool canExhaust)
        {
            canExhaust = false;
            if (_agentSystem == null || _data?.NavMesh == null) return null;

            int triangles = _data.NavMesh.Triangles.Length;
            var tuning = _agentSystem.Tuning;
            canExhaust = triangles > tuning.MaxIterations;
            bool canClamp = triangles > tuning.CorridorCap;
            if (!canExhaust && !canClamp) return null;

            string head = canExhaust
                ? $"{triangles} triangles is past the {tuning.MaxIterations}-triangle search budget, "
                  + $"so a flat search on this mesh CAN run out before it decides — units then "
                  + $"report PathFailed while standing on walkable ground."
                : $"{triangles} triangles is past the {tuning.CorridorCap}-triangle corridor cap, so "
                  + $"a flat path CAN come back clamped and be silently replanned.";

            // Shown only while no graph is installed (the callers gate on AbstractGraph == null), and
            // since 0.13 that state has a cause worth naming: the automatic install is off in the
            // tool tuning, the ladder found no cell size that fits, or Clear removed it.
            return head
                + " Planning in legs makes each search local instead. It is on by default "
                + "(FPNavTuning.AutoInstallAbstractGraph) and this stack has none — the automatic "
                + "install is off in the tool tuning, no cell size fit this mesh, or it was cleared. "
                + "Apply installs one here; a game turns the automatic install back on.";
        }

        #endregion

        /// <summary>
        /// Installs a rebaked mesh WITHOUT rebuilding the simulation — the engine's own swap
        /// protocol, so an editor experiment survives a building being placed.
        ///
        /// <para><see cref="Initialize"/> is the alternative and it is the wrong one here: it
        /// recreates the frame and resets the tick, so every placement would wipe the agents whose
        /// behaviour is the thing being looked at. What this does instead is what a match does —
        /// <c>FPNavAgentInstaller.Swap</c> rebinds the query/pathfinder/funnel, drops the
        /// graph-local obstacle CSR, re-extracts the ORCA obstacles (a carve adds a hole ring, so
        /// that set really does change) and re-collects the agent list; then <c>ReseedAgents</c>
        /// re-queries every agent's triangle index on the new mesh. Skipping the reseed leaves
        /// agents holding indices into the mesh that was just replaced, which is not an exception —
        /// they simply walk somewhere else.</para>
        ///
        /// <para>Returns false when there is nothing to swap into (no simulation built yet); the
        /// caller then has no agents to worry about either.</para>
        /// </summary>
        public bool SwapNavMesh(FPNavMesh newMesh)
        {
            if (_agentSystem == null || _simFrame == null || newMesh == null)
                return false;

            // Prepare the graph for the new mesh before the swap adopts it. The tool's rebake is
            // synchronous, so this is the same derivation a beat earlier — but it is the path the
            // engine's swap wants (PrepareAbstractGraphFor then adopt), and without it every placement
            // on a mesh with a graph (the 0.13 default on a large one) would take the re-derive
            // fallback and its once-per-system warning in the editor console.
            _agentSystem.PrepareAbstractGraphFor(newMesh);

            int collected = FPNavAgentInstaller.Swap(
                ref _simFrame, _agentSystem, newMesh, ref _entities);
            _agentSystem.ReseedAgents(ref _simFrame, _entities, collected);
            _entityCount = collected;

            // Anything at PathFailed now crossed the swap (see the field's remarks). Read after the
            // reseed rather than before it, so this needs no index bookkeeping across the
            // re-collection Swap performs.
            for (int i = 0; i < collected; i++)
            {
                ref readonly var nav = ref _simFrame.GetReadOnly<NavAgentComponent>(_entities[i]);
                _failurePredatesSwap[i] = nav.Status == (byte)FPNavAgentStatus.PathFailed;
                // A verdict searched on the mesh that just left is not a verdict on this one.
                _failedReason[i] = FPNavPathFailureReason.None;
            }
            BuildDiagnosisPathfinder(newMesh, _data?.Logger);

            // Re-apply the inset the tool is holding. The swap runs LoadNavMeshObstacles again,
            // which re-derives ObstacleRadiusInset from the new mesh's BakeAgentRadius — correct
            // for a game (the asset is what keeps lockstep peers symmetric) and wrong for a
            // diagnosis, because a rebake inherits the base's radius and so every building placed
            // would silently undo the knob mid-experiment. The mirror field IS the bake radius
            // when nobody has touched it, so this is a no-op unless it was overridden.
            _avoidance.ObstacleRadiusInset = FP64.FromFloat(ObstacleRadiusInset);

            // Every placement and removal comes through here, so a refusal the OLD mesh caused
            // must not outlive it — the banner quoted that mesh's cell size and stayed up after
            // the building responsible for it was removed.
            LastDestinationRefusal = null;
            return true;
        }

        /// <summary>
        /// Live knob: writes through to the retained avoidance so toggling the inset (e.g. 0 vs
        /// bake radius) is visible without reloading the mesh.
        /// </summary>
        public void SetObstacleRadiusInset(float v)
        {
            ObstacleRadiusInset = v;
            if (_avoidance != null)
                _avoidance.ObstacleRadiusInset = FP64.FromFloat(v);
        }

        public int AddAgent(Vector3 position)
        {
            if (_entityCount >= MAX_AGENTS) return -1;
            if (_data == null || !_data.IsLoaded || _simFrame == null) return -1;

            var entity = _simFrame.CreateEntity();
            _simFrame.Add(entity, default(NavAgentComponent));
            ref var nav = ref _simFrame.Get<NavAgentComponent>(entity);

            FPVector3 fpPos = position.ToFPVector3();
            NavAgentComponent.Init(ref nav, fpPos);
            nav.Speed = FP64.FromFloat(DefaultSpeed);
            nav.Radius = FP64.FromFloat(DefaultRadius);
            nav.Acceleration = FP64.FromFloat(DefaultAcceleration);
            nav.CurrentTriangleIndex = _data.FindTriangleAtPosition(position);

            int idx = _entityCount;
            _entities[idx] = entity;
            _initialPositions[idx] = position;
            // Third parallel array, same slot. A reused slot must not inherit the verdict of the
            // agent that occupied it — the tool would then report a fresh agent's genuine failure
            // as one that predates a swap, and offer an action that changes nothing.
            _failurePredatesSwap[idx] = false;
            _failedKind[idx] = FailureKind.Unknown;
            _failedReason[idx] = FPNavPathFailureReason.None;
            _entityCount++;
            return idx;
        }

        public void RemoveAgent(int index)
        {
            if (index < 0 || index >= _entityCount) return;

            _entityCount--;
            if (index < _entityCount)
            {
                _entities[index] = _entities[_entityCount];
                _initialPositions[index] = _initialPositions[_entityCount];
                // The tail slid down here, so its verdict has to slide with it. Leaving this array
                // out of the compaction is what let the moved agent inherit the removed one's.
                _failurePredatesSwap[index] = _failurePredatesSwap[_entityCount];
                // These two were left out of that compaction, so an agent already PathFailed
                // when it slid down kept the removed agent's budget attribution and verdict.
                _failedKind[index] = _failedKind[_entityCount];
                _failedReason[index] = _failedReason[_entityCount];
            }
            // The banner is not per agent, so removing an agent can strand the reason it showed.
            LastDestinationRefusal = null;
        }

        /// <summary>
        /// Sets a destination the agent can actually be given, snapping the click to the nearest
        /// ground its PLAN mask allows. Returns false when no such ground is within
        /// <see cref="DestinationSnapMaxDist"/>, in which case the agent is stopped and
        /// <see cref="LastDestinationRefusal"/> says why.
        ///
        /// <para><b>Why the raw click could not be used</b>: a retained building is ON the mesh and
        /// carries <c>FPNavMeshAreas.BUILDING_MASK</c>, so a click on it produced a destination
        /// that <c>FindPath</c> refuses by mask. The agent then sat at <c>PathFailed</c> with a
        /// destination and no corridor, and the screen said "it has somewhere to go and will not
        /// move" — indistinguishable from being stuck.</para>
        ///
        /// <para>The mask comes from <see cref="FPNavAgentSystem.ResolvePlanMask"/> rather than a
        /// local <c>!= 0 ?</c>, because this tool's whole purpose includes
        /// <see cref="SetAgentAreaMask"/>: the override is normally set here, and a wrong fold of
        /// zero would make every triangle impassable with the same symptom this method fixes.</para>
        ///
        /// <para>Y is resampled at the snapped XZ. Keeping the click's own y would pair a projected
        /// XZ with an unrelated height, which on a multi-floor mesh puts the destination on the
        /// wrong floor.</para>
        /// </summary>
        public bool SetAgentDestination(int index, FPVector3 dest)
        {
            // The contract this method's callers rely on is "false means LastDestinationRefusal
            // says why". These two returned false silently, so a click on a selected-then-removed
            // agent either printed nothing or reprinted an older refusal that named the snap knob
            // for a problem that had nothing to do with it.
            if (index < 0 || index >= _entityCount || _simFrame == null)
            {
                LastDestinationRefusal = _entityCount == 0
                    ? "No agent to give a destination to — place one first"
                    : $"No agent #{index} — the selection is stale (there are {_entityCount})";
                return false;
            }
            if (_data == null || _data.Query == null)
            {
                LastDestinationRefusal = "No navmesh loaded";
                return false;
            }

            ref var nav = ref _simFrame.Get<NavAgentComponent>(_entities[index]);
            int areaMask = FPNavAgentSystem.ResolvePlanMask(nav);

            var destXZ = new FPVector2(dest.x, dest.z);
            FP64 maxDist = FP64.FromFloat(DestinationSnapMaxDist);

            FPVector3 target;
            // The ENGINE's endpoint rule, not the y-blind one. FindPassableTriangle ignores height,
            // so on a multi-floor mesh a click on forbidden ground with walkable ground beneath it
            // was accepted as-is and kept the click's own y — the snap below never ran, and
            // FindPath then refused the destination by mask.
            if (_data.Query.FindPassableTriangleForEndpoint(destXZ, dest.y, areaMask) >= 0)
            {
                target = dest;
            }
            else
            {
                FPVector2 snapped = _data.Query.ProjectToPassable(
                    destXZ, maxDist, areaMask, out int triIdx);
                if (triIdx < 0)
                {
                    // Stop rather than leave the previous destination running: the click would
                    // otherwise be refused while the agent kept walking to wherever it was already
                    // headed, which reads as "I clicked and it went somewhere else". Stop also puts
                    // the status at Idle, so a refusal is distinguishable on screen from the
                    // PathFailed this whole change exists to remove.
                    NavAgentComponent.Stop(ref nav);
                    _failurePredatesSwap[index] = false;
                    _failedReason[index] = FPNavPathFailureReason.None;
                    LastDestinationRefusal =
                        $"No passable ground within snap distance {DestinationSnapMaxDist:F2} "
                        + $"(the search covers one cell ring; ground closer than "
                        + $"{_data.NavMesh.GridCellSize.ToFloat():F2} is always found)";
                    return false;
                }

                FP64 height = _data.Query.SampleHeight(snapped, triIdx);
                target = new FPVector3(snapped.x, height, snapped.y);
            }

            NavAgentComponent.SetDestination(ref nav, target);
            LastDestinationRefusal = null;
            _failurePredatesSwap[index] = false;   // a fresh destination gets a fresh verdict
            _failedReason[index] = FPNavPathFailureReason.None;
            return true;
        }

        public void StopAgent(int index)
        {
            if (index < 0 || index >= _entityCount || _simFrame == null) return;

            ref var nav = ref _simFrame.Get<NavAgentComponent>(_entities[index]);
            NavAgentComponent.Stop(ref nav);
        }

        /// <summary>
        /// Gives ONE agent its own plan and walk area masks, so a scene can hold agents that
        /// disagree about which ground they may use. Pass 0 for either to mean "no override".
        ///
        /// <para>Per agent rather than a simulator-wide default because the combination worth
        /// looking at is two agents on the same mesh with the same destination and different walk
        /// masks — one enters a retained footprint and the other stops at its edge on the same
        /// tick. A global setting could only produce that by being changed between spawns.</para>
        ///
        /// <para>Goes through <c>NavAgentComponent.SetAreaMask</c>, which also drops the corridor
        /// the old masks planned; the agent replans on the next tick. That is the behaviour to
        /// watch when narrowing a mask under an agent that is already walking.</para>
        /// </summary>
        public void SetAgentAreaMask(int index, int planMask, int walkMask)
        {
            if (index < 0 || index >= _entityCount || _simFrame == null) return;

            ref var nav = ref _simFrame.Get<NavAgentComponent>(_entities[index]);
            NavAgentComponent.SetAreaMask(ref nav, planMask, walkMask);
        }

        public void ClearAllAgents()
        {
            _entityCount = 0;
            IsRunning = false;
            LastDestinationRefusal = null;
            System.Array.Clear(_failurePredatesSwap, 0, _failurePredatesSwap.Length);
            System.Array.Clear(_failedReason, 0, _failedReason.Length);
            System.Array.Clear(_failedKind, 0, _failedKind.Length);
            if (_simFrame != null)
                _simFrame = new Frame(MAX_AGENTS, null);
        }

        public void Start()
        {
            if (_agentSystem == null) return;

            if (EnableAvoidance)
                _agentSystem.SetAvoidance(_avoidance);
            else
                _agentSystem.SetAvoidance(null);

            IsRunning = true;
            _lastUpdateTime = EditorApplication.timeSinceStartup;
            _accumulator = 0;
        }

        public void Pause()
        {
            IsRunning = false;
        }

        // Which agents failed because the A* budget ran out, rather than because no route exists,
        // judged from the tick-wide exhaustion delta. The per-agent re-search (_failedReason) is
        // the exact answer and the one ExplainFailure prefers; this is kept for the cases it cannot
        // answer. The delta substitutes for a per-agent record only under an assumption planning
        // in legs breaks — see StepOnce.
        private FailureKind[] _failedKind = new FailureKind[MAX_AGENTS];

        /// <summary>
        /// The tick-wide attribution of a new failure to the budget or to the map, for the one tick
        /// where that is knowable. <see cref="Unknown"/> is not a placeholder — it is the honest
        /// answer when the exhaustion delta cannot be pinned to a particular agent.
        ///
        /// <para><b>A fallback now, not the verdict.</b> The per-agent re-search
        /// (<see cref="FPNavPathFailure.SearchVerdict"/>, run once per failure by
        /// <see cref="DiagnosePathFailure"/>) answers the same question exactly, with or without a
        /// graph. This is consulted only where that answer is unavailable — the play-mode bridge
        /// owns the data layer, or the re-search found a path — and it costs nothing to keep.</para>
        /// </summary>
        public enum FailureKind : byte { Unknown = 0, BudgetRanOut, NoRouteProven }

        /// <summary>
        /// Whether this simulation's agent system hands back partial paths on exhaustion — the one
        /// piece of tuning the failure sentence needs, since <c>FPNavPathFailure.Describe</c> is
        /// deliberately mode-neutral. False until a simulation is built.
        /// </summary>
        public bool PartialPathsOn => _agentSystem != null && _agentSystem.Tuning.PartialPathOnExhaustion;

        /// <summary>
        /// Advances one tick and attributes any NEW path failure to the budget or to the map.
        ///
        /// <para><b>One direction is always sound</b>: no search exhausted its budget this tick
        /// means none of this tick's failures was a budget failure, so they ran to completion and
        /// found nothing.</para>
        ///
        /// <para><b>The other direction holds only without a graph.</b> Reading "every new failure
        /// came with an exhausted search" as "all of them were budget" needs each agent to
        /// contribute at most one exhaustion, and only by failing. Planning in legs breaks both:
        /// <c>FPNavAgentSystem.ProcessPathRequest</c> searches twice (leg target, then flat retry),
        /// and an agent whose leg search exhausted may then succeed on the retry — raising the
        /// delta while failing nothing. So with legs on this reports <see cref="FailureKind.Unknown"/>
        /// rather than naming a disconnected agent a budget failure, which is the opposite
        /// prescription.</para>
        ///
        /// <para><b>Tick-wide numbers are logged once, on their own line.</b> The pathfinder keeps
        /// one <c>DebugLastSearchIterations</c> for every agent that plans and for the Find Path
        /// button, so repeating it inside each agent's sentence claimed an attribution that does
        /// not exist.</para>
        /// </summary>
        private void StepOnce(int tick)
        {
            var pathfinder = _data?.Pathfinder;
            int exhaustedBefore = pathfinder?.DebugIterationExhaustedCount ?? 0;

            bool[] wasFailed = null;
            if (pathfinder != null)
            {
                wasFailed = new bool[_entityCount];
                for (int i = 0; i < _entityCount; i++)
                    wasFailed[i] = _simFrame.GetReadOnly<NavAgentComponent>(_entities[i]).Status
                        == (byte)FPNavAgentStatus.PathFailed;
            }

            _agentSystem.Update(ref _simFrame, _entities, _entityCount, tick, _dt);

            if (pathfinder == null) return;

            int exhausted = pathfinder.DebugIterationExhaustedCount - exhaustedBefore;
            int newlyFailed = 0;
            for (int i = 0; i < _entityCount; i++)
            {
                bool failedNow = _simFrame.GetReadOnly<NavAgentComponent>(_entities[i]).Status
                    == (byte)FPNavAgentStatus.PathFailed;
                if (failedNow && !wasFailed[i]) newlyFailed++;
                else if (!failedNow)
                {
                    _failedKind[i] = FailureKind.Unknown;
                    _failedReason[i] = FPNavPathFailureReason.None;
                }
            }
            if (newlyFailed == 0) return;

            // Ambiguous means the delta cannot be pinned to particular agents, not that nothing is
            // known — the failure still gets reported, just without the budget claim.
            //
            // ONE direction is sound whatever is installed: no exhaustion anywhere this tick means
            // no failure this tick was a budget failure, so they ran to completion and genuinely
            // found nothing. That is a stronger statement than the combined reason enum can make.
            //
            // The other direction needs "an agent contributes at most one exhaustion, and only by
            // failing". That holds with no graph — one FindPath per agent per tick, the counter
            // rises only on the failing path, and an agent already PathFailed does not re-plan.
            // Planning in legs breaks BOTH halves: FPNavAgentSystem.ProcessPathRequest searches
            // twice (leg target, then flat retry), so one agent can raise the delta by two, and an
            // agent whose leg search exhausted may then SUCCEED flat — raising it while failing
            // nothing. Reading exhausted >= newlyFailed there would name a genuinely disconnected
            // agent "the budget ran out", which is the opposite prescription.
            bool oneExhaustionPerFailure = _abstractGraph == null;
            FailureKind kind =
                  exhausted == 0 ? FailureKind.NoRouteProven
                : oneExhaustionPerFailure && exhausted >= newlyFailed ? FailureKind.BudgetRanOut
                : FailureKind.Unknown;

            // The tick's numbers, said ONCE and labelled as the tick's. They used to ride inside
            // every agent's sentence below, where "last search popped 4096" reads as this agent's
            // search — and it is not: the pathfinder keeps one last-search value for every agent
            // that plans, and for the Find Path button besides.
            Debug.LogWarning(
                $"[NavMeshSim] tick {tick}: {newlyFailed} new path failure(s), "
                + $"{exhausted} search(es) out of budget"
                + (oneExhaustionPerFailure ? "" : " (legs ON — one agent can spend two searches, "
                    + "and a search that exhausted may belong to an agent that then succeeded)")
                // The budget is spent in TRIANGLES, so the raw number means nothing without the
                // ground it had to cover and how finely that ground is cut. Printed together, the
                // three say whether the ceiling was reached crossing open space (it should not be)
                // or flooding around something in the way (it will be).
                + $"\n  the tick's LAST search popped {pathfinder.DebugLastSearchIterations}"
                + $" of {pathfinder.Tuning.MaxIterations} triangles"
                + $" (mesh has {_data.NavMesh.Triangles.Length}) — the tick's, not any one agent's"
                + $"\n  counters since load: budget exhausted {pathfinder.DebugIterationExhaustedCount}"
                + $", corridor clamped {pathfinder.DebugCorridorTruncatedCount}"
                + (_abstractGraph == null
                    ? "  (planning in legs is OFF — the automatic install is off, found no cell size, or was cleared; Apply installs one)"
                    : $"  (legs ON{(_abstractGraphIsAutomatic ? ", automatic" : "")}, {_abstractGraph.NodeComponentCount} connected piece"
                      + $"{(_abstractGraph.NodeComponentCount == 1 ? "" : "s")})"));

            for (int i = 0; i < _entityCount; i++)
            {
                ref readonly var nav = ref _simFrame.GetReadOnly<NavAgentComponent>(_entities[i]);
                if (nav.Status != (byte)FPNavAgentStatus.PathFailed || wasFailed[i]) continue;

                _failedKind[i] = kind;

                // Logged HERE, on the transition, and nowhere else. The agent list is rebuilt every
                // repaint, so logging from the UI would print the same failure for as long as it
                // stands. This fires once, when it happens — which is also the only moment the
                // budget attribution above is available.
                //
                // Everything on this line is THIS agent's. The tick-wide numbers went to the line
                // above on purpose: repeated per agent they read as per-agent evidence.
                float straight = Vector3.Distance(
                    nav.Position.ToVector3(), nav.Destination.ToVector3());
                Debug.LogWarning(
                    $"[NavMeshSim] Agent #{i} PathFailed on tick {tick} — "
                    + ExplainFailure(DiagnosePathFailure(i, nav), kind, PartialPathsOn)
                    + $"\n  at {nav.Position.ToVector3()} heading for {nav.Destination.ToVector3()}"
                    + $" — {straight:F1} units in a straight line, triangle {nav.CurrentTriangleIndex}");
            }
        }

        /// <summary>
        /// One sentence for why an agent is stuck, shared by the log and the agent list so the two
        /// cannot drift into saying different things about the same state.
        ///
        /// <para>Two causes take opposite responses: a split map is not this tool's to fix, while a
        /// search that ran out of budget is exactly what planning in legs removes. The re-search
        /// (<c>reason</c>) tells them apart per agent; the tick-wide attribution (<c>kind</c>) is
        /// the fallback for the cases the re-search cannot answer. <paramref name="partialPathsOn"/>
        /// is the agent system's tuning — see <see cref="PartialPathsOn"/>.</para>
        /// </summary>
        public static string ExplainFailure(FPNavPathFailureReason reason, FailureKind kind, bool partialPathsOn)
        {
            switch (reason)
            {
                // The per-agent verdict from the re-search (FPNavPathFailure.SearchVerdict) — exact,
                // with or without a graph. An exhausted search means the open set was still not
                // empty when the budget ran out; it does NOT mean a route exists, only that the
                // search was stopped before it could decide either way. The parenthetical is the
                // tool's to add: it knows its agent system's tuning, the shared Describe does not.
                case FPNavPathFailureReason.BudgetExhausted:
                    return "the search budget ran out before it could decide"
                         + (partialPathsOn ? " (partial paths are on: no point closer to the goal was in reach)" : "")
                         + " — planning in legs makes each search local, which is what removes this";

                // No verdict from the re-search: the play-mode bridge owns the data layer, or the
                // re-search FOUND a path and the stack it was asked on disagrees with the one that
                // failed. The tick-wide attribution is the fallback — sound in one direction always
                // and in the other only without a graph (see StepOnce).
                case FPNavPathFailureReason.NoRouteOrBudget:
                    switch (kind)
                    {
                        case FailureKind.BudgetRanOut:
                            return "the A* budget ran out before the search could decide — "
                                 + "planning in legs makes each search local, which is what removes this";
                        // This one IS decisive: nothing exhausted, so the search finished and found nothing.
                        case FailureKind.NoRouteProven:
                            return "no route — the search ran to completion and found none";
                        default:
                            return FPNavPathFailure.Describe(reason).TrimStart(' ', '←').Trim();
                    }

                default:
                    return FPNavPathFailure.Describe(reason).TrimStart(' ', '←').Trim();
            }
        }

        public void Step()
        {
            if (_agentSystem == null || _entityCount == 0 || _simFrame == null) return;

            if (EnableAvoidance)
                _agentSystem.SetAvoidance(_avoidance);
            else
                _agentSystem.SetAvoidance(null);

            // Debug: record state before update
            var prevStatus = new byte[_entityCount];
            for (int i = 0; i < _entityCount; i++)
            {
                ref readonly var nav = ref _simFrame.GetReadOnly<NavAgentComponent>(_entities[i]);
                prevStatus[i] = nav.Status;
            }

            CurrentTick++;
            StepOnce(CurrentTick);

            // Debug: log status changes
            for (int i = 0; i < _entityCount; i++)
            {
                ref readonly var nav = ref _simFrame.GetReadOnly<NavAgentComponent>(_entities[i]);
                if (nav.Status != prevStatus[i])
                {
                    Debug.Log($"[NavMeshSim] Agent #{i}: {(FPNavAgentStatus)prevStatus[i]} -> {(FPNavAgentStatus)nav.Status}" +
                        $" (dest={nav.HasNavDestination}, path={nav.HasPath}, tri={nav.CurrentTriangleIndex})");
                }
                else if (prevStatus[i] == (byte)FPNavAgentStatus.PathPending)
                {
                    Debug.LogWarning($"[NavMeshSim] Agent #{i}: PathPending persists" +
                        $" (tri={nav.CurrentTriangleIndex}, dest={nav.Destination.ToVector3()})");
                }
            }

            UpdateLastOrcaAgent();
        }

        public void Reset()
        {
            IsRunning = false;
            CurrentTick = 0;
            _accumulator = 0;

            if (_simFrame == null) return;

            for (int i = 0; i < _entityCount; i++)
            {
                ref var nav = ref _simFrame.Get<NavAgentComponent>(_entities[i]);
                Vector3 pos = _initialPositions[i];
                NavAgentComponent.Init(ref nav, pos.ToFPVector3());
                nav.Speed = FP64.FromFloat(DefaultSpeed);
                nav.Radius = FP64.FromFloat(DefaultRadius);
                nav.Acceleration = FP64.FromFloat(DefaultAcceleration);
                if (_data != null)
                    nav.CurrentTriangleIndex = _data.FindTriangleAtPosition(pos);
            }

            ClearAllAgents();
        }

        public void OnEditorUpdate()
        {
            if (!IsRunning || _agentSystem == null || _entityCount == 0 || _simFrame == null) return;

            double now = EditorApplication.timeSinceStartup;
            double delta = now - _lastUpdateTime;
            _lastUpdateTime = now;

            if (delta > 0.1) delta = 0.1;

            _accumulator += delta * SimulationSpeed;

            bool updated = false;
            bool logOnce = (CurrentTick <= 1);
            while (_accumulator >= FIXED_DT)
            {
                _accumulator -= FIXED_DT;
                CurrentTick++;
                StepOnce(CurrentTick);
                updated = true;

                if (logOnce)
                {
                    logOnce = false;
                    for (int i = 0; i < _entityCount; i++)
                    {
                        ref readonly var nav = ref _simFrame.GetReadOnly<NavAgentComponent>(_entities[i]);
                        Debug.Log($"[NavMeshSim] Agent #{i}: status={(FPNavAgentStatus)nav.Status}" +
                            $" dest={nav.HasNavDestination} path={nav.HasPath}" +
                            $" tri={nav.CurrentTriangleIndex} pos={nav.Position.ToVector3()}");
                    }
                }
            }

            if (updated)
            {
                UpdateLastOrcaAgent();
                SceneView.RepaintAll();
            }
        }

        public struct AgentRenderData
        {
            public Vector3 position;
            public Vector2 velocity;
            public Vector2 desiredVelocity;
            public float radius;
            public float speed;
            public Vector3 destination;
            public bool hasDestination;
            public bool hasPath;
            public FPNavAgentStatus status;
            public int currentTriangleIndex;
            // The agent's own masks, so a mixed scene can be told apart in the list. 0 = no
            // override, i.e. FPNavAgentSystem.DEFAULT_AREA_MASK.
            public int planAreaMask;
            public int walkAreaMask;
            // Why PathFailed, judged against the mesh as it is NOW. None unless the status is
            // PathFailed. An enum rather than a string so the window owns the wording and the
            // pattern tests can compare exactly.
            public FPNavPathFailureReason failureReason;
            public int[] corridor;
            public int corridorLength;
            // Where THIS plan aims. Equal to the destination unless a graph is installed, in which
            // case it is the portal the current leg ends at — the one value that shows legs are
            // steering rather than just being counted.
            public Vector3 pathTarget;
            // PathFailed only. True when the A* budget ran out rather than the map being split —
            // the same status and the same reason enum, opposite responses.
            public FailureKind failedKind;
        }

        /// <summary>
        /// Why this agent sits at <c>PathFailed</c> — the shared runtime diagnosis, in its two
        /// halves. The endpoint half runs on every call, against the mesh as it is now; the search
        /// half runs once per failure on this tool's own pathfinder and is remembered in
        /// <c>_failedReason</c>. See <see cref="FPNavPathFailure.Diagnose"/> for the order the
        /// refusals are walked in and why the START's mask is deliberately absent.
        ///
        /// <para>No search while the play-mode bridge owns the data layer: <c>LoadFromNavMesh</c>
        /// nulls <c>_data.Pathfinder</c> without re-initialising this simulator, so the diagnosis
        /// pathfinder would be over a mesh the window no longer shows. NoRouteOrBudget is the honest
        /// answer there, as it was before the search half existed.</para>
        /// </summary>
        internal FPNavPathFailureReason DiagnosePathFailure(int index, in NavAgentComponent nav)
        {
            FPNavPathFailureReason reason = FPNavPathFailure.DiagnoseEndpoints(
                in nav, _data?.Query, _data?.NavMesh, _failurePredatesSwap[index]);
            if (reason != FPNavPathFailureReason.NoRouteOrBudget)
                return reason;
            if (_failedReason[index] != FPNavPathFailureReason.None)
                return _failedReason[index];
            if (_diagPathfinder == null || _data?.Pathfinder == null)
                return reason;
            // The system's own threshold: the engine's minimum progress was
            // PartialMinProgress(nav, WaypointThreshold), and the re-search must ask for the same.
            _failedReason[index] = FPNavPathFailure.SearchVerdict(in nav, _diagPathfinder, _agentSystem.WaypointThreshold);
            return _failedReason[index];
        }

        /// <summary>
        /// The diagnosis stack, over <paramref name="mesh"/> and with the agent system's tuning —
        /// the shape <c>FPNavPartialPathTests</c> builds for the same purpose. Its own query, not
        /// <c>_data.Query</c>: that one is rebound by the data layer on a rebake, and building on
        /// it here would make this correct only for one order of two calls.
        /// </summary>
        private void BuildDiagnosisPathfinder(FPNavMesh mesh, xpTURN.Klotho.Logging.IKLogger logger)
        {
            var tuning = _agentSystem.Tuning;
            var query = new FPNavMeshQuery(mesh, logger, tuning);
            _diagPathfinder = new FPNavMeshPathfinder(mesh, query, logger, tuning);
        }

        public unsafe AgentRenderData GetAgentRenderData(int index)
        {
            if (index < 0 || index >= _entityCount || _simFrame == null)
                return default;

            ref readonly var nav = ref _simFrame.GetReadOnly<NavAgentComponent>(_entities[index]);

            var rd = new AgentRenderData
            {
                position = nav.Position.ToVector3(),
                velocity = nav.Velocity.ToVector2(),
                desiredVelocity = nav.DesiredVelocity.ToVector2(),
                radius = nav.Radius.ToFloat(),
                speed = nav.CurrentSpeed.ToFloat(),
                destination = nav.Destination.ToVector3(),
                hasDestination = nav.HasNavDestination,
                hasPath = nav.HasPath,
                status = (FPNavAgentStatus)nav.Status,
                currentTriangleIndex = nav.CurrentTriangleIndex,
                planAreaMask = nav.PlanAreaMaskOverride,
                walkAreaMask = nav.WalkAreaMaskOverride,
                failureReason = DiagnosePathFailure(index, nav),
                pathTarget = nav.PathTarget.ToVector3(),
                failedKind = _failedKind[index],
            };

            if (nav.HasPath && nav.PathIsValid && nav.CorridorLength > 0)
            {
                rd.corridorLength = nav.CorridorLength;
                rd.corridor = new int[nav.CorridorLength];
                fixed (int* src = nav.Corridor)
                {
                    for (int i = 0; i < nav.CorridorLength; i++)
                        rd.corridor[i] = src[i];
                }
            }

            return rd;
        }

        private void UpdateLastOrcaAgent()
        {
            LastOrcaComputedAgentIndex = -1;
            if (_simFrame == null) return;

            for (int i = 0; i < _entityCount; i++)
            {
                ref readonly var nav = ref _simFrame.GetReadOnly<NavAgentComponent>(_entities[i]);
                if (nav.Status == (byte)FPNavAgentStatus.Moving)
                {
                    LastOrcaComputedAgentIndex = i;
                    break;
                }
            }
        }
    }
}
