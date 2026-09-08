using UnityEditor;
using UnityEngine;
using xpTURN.Klotho.Unity;
using xpTURN.Klotho.ECS;
using xpTURN.Klotho.Deterministic.Math;
using xpTURN.Klotho.Deterministic.Navigation;

namespace xpTURN.Klotho.Editor
{
    /// <summary>
    /// Overlay-renders NavMesh geometry and agent paths in the Scene view.
    /// </summary>
    internal class FPNavMeshSceneOverlay
    {
        // Geometry layer
        public bool ShowTriangles = true;
        public bool ShowEdges = true;
        public bool ShowBoundaryEdges = true;
        public bool ShowObstacleRings;
        public bool ShowVertices;
        public bool ShowTriangleIndices;
        public bool ShowTriangleCenters;
        public bool ShowBlockedTriangles = true;
        public bool ShowCostHeatmap;

        /// <summary>
        /// Colour the triangle fill by abstract node instead of by area, and outline where one node
        /// ends and the next begins. Off by default: it replaces the fill that says blocked/building,
        /// so it is a lens you reach for, not the normal view.
        /// </summary>
        public bool ShowNodes;

        /// <summary>
        /// The graph to colour by, or null. Set by the window from the simulator — the overlay does
        /// NOT derive its own: a second derivation could disagree with the one the agents plan
        /// against, and the only symptom would be a picture that is quietly wrong.
        /// </summary>
        public FPNavAbstractGraph AbstractGraph;

        // Path layer
        public bool ShowCorridor = true;
        public bool ShowWaypoints = true;
        public bool ShowPortals;
        public bool ShowStartEndMarkers = true;

        // Grid layer
        public bool ShowGrid;
        public bool ShowGridLabels;

        // Agent layer
        public bool ShowAgents = true;
        public bool ShowAgentPaths = true;
        public bool ShowAgentVelocities = true;
        public bool ShowOrcaLines;

        private FPNavMeshVisualizerData _data;
        private FPNavMeshAgentSimulator _agentSim;

        // For runtime agent path recalculation (editor only)
        private FPNavMeshPathfinder _runtimePathfinder;
        private FPNavMeshFunnel _runtimeFunnel;
        private FPNavMesh _runtimeNavMeshRef;

        private readonly Vector3[] _runtimeWaypointBuf = new Vector3[64];

        // PlayMode runtime snapshots
        private NavAgentSnapshot[] _runtimeSnapshotBuffer;
        public int RuntimeAgentSnapshotCount { get; set; }
        public int SelectedEntityIndex { get; set; } = -1;

        // Hover info
        public int HoveredTriangleIndex = -1;
        public (int col, int row) HoveredCell = (-1, -1);

        public void SetData(FPNavMeshVisualizerData data)
        {
            _data = data;
        }

        public void SetAgentSimulator(FPNavMeshAgentSimulator sim)
        {
            _agentSim = sim;
        }

        public void SetRuntimeSnapshotBuffer(NavAgentSnapshot[] buffer)
        {
            _runtimeSnapshotBuffer = buffer;
            RuntimeAgentSnapshotCount = 0;
        }

        public void OnSceneGUI(SceneView sceneView)
        {
            if (_data == null || !_data.IsLoaded) return;

            if (ShowTriangles)
                DrawTriangles();

            if (ShowEdges)
                DrawInternalEdges();

            if (ShowBoundaryEdges)
                DrawBoundaryEdges();

            if (ShowObstacleRings)
                DrawObstacleRings();

            if (ShowVertices)
                DrawVertices();

            if (ShowTriangleIndices)
                DrawTriangleIndices(sceneView);

            if (ShowTriangleCenters)
                DrawTriangleCenters();

            if (ShowCostHeatmap)
                DrawCostHeatmap();

            if (ShowNodes && AbstractGraph != null)
                DrawNodeBoundaries();

            if (ShowGrid)
                DrawGrid();

            if (ShowGridLabels)
                DrawGridLabels(sceneView);

            if (ShowCorridor && _data.HasPath)
                DrawCorridor();

            if (ShowPortals && _data.HasPath)
                DrawPortals();

            if (ShowWaypoints && _data.HasPath)
                DrawWaypoints();

            if (ShowStartEndMarkers)
                DrawStartEndMarkers();

            if (ShowAgents && _agentSim != null)
                DrawAgents();

            if (ShowAgentPaths && _agentSim != null)
                DrawAgentPaths();

            if (ShowAgentVelocities && _agentSim != null)
                DrawAgentVelocities();

            if (ShowOrcaLines && _agentSim != null)
                DrawOrcaVisualization();

            if (ShowAgents && _runtimeSnapshotBuffer != null && RuntimeAgentSnapshotCount > 0)
                DrawRuntimeAgents();
        }

        #region Geometry

        private void DrawTriangles()
        {
            // Node colouring is a MODE of this fill, not a pass of its own. The cost heatmap is a
            // second full sweep drawn on top, which is affordable on a test mesh and is not on the
            // Field asset: 22,321 triangles, no culling, every repaint. Choosing a colour here
            // costs nothing extra.
            var graph = ShowNodes ? AbstractGraph : null;

            for (int i = 0; i < _data.CachedTriangles.Length; i++)
            {
                ref TriangleRenderData tri = ref _data.CachedTriangles[i];

                if (tri.isBlocked && ShowBlockedTriangles)
                    Handles.color = FPNavMeshVisualizerStyles.TriangleFillBlocked;
                else if (tri.isBlocked)
                    continue;
                else if (graph != null)
                {
                    // -1 is "no node claims this triangle" — masked out or blocked — and it must NOT
                    // share a colour with node 0, or ground the graph cannot route through reads as
                    // ordinary walkable surface.
                    int node = graph.NodeOf(i);
                    Handles.color = node < 0
                        ? FPNavMeshVisualizerStyles.TriangleFillOutsideGraph
                        : FPNavMeshVisualizerStyles.NodeFill(node);
                }
                // Equality, not a bit test: the rebaker stamps a retained footprint EXCLUSIVELY
                // (see FPNavMeshAreas), so a triangle carrying the bit alongside a baked area is
                // not something this build produces — and shading it as a building would be a
                // guess. isBlocked still wins: it is the stronger claim and owns the skip above.
                else if (tri.areaMask == FPNavMeshAreas.BUILDING_MASK)
                    Handles.color = FPNavMeshVisualizerStyles.TriangleFillBuilding;
                else
                    Handles.color = FPNavMeshVisualizerStyles.TriangleFill;

                Handles.DrawAAConvexPolygon(tri.v0, tri.v1, tri.v2);
            }
        }

        private void DrawInternalEdges()
        {
            Handles.color = FPNavMeshVisualizerStyles.EdgeInternal;
            for (int i = 0; i < _data.InternalEdges.Count; i++)
            {
                var edge = _data.InternalEdges[i];
                Handles.DrawLine(edge.a, edge.b, FPNavMeshVisualizerStyles.EdgeInternalWidth);
            }
        }

        private void DrawBoundaryEdges()
        {
            Handles.color = FPNavMeshVisualizerStyles.EdgeBoundary;
            for (int i = 0; i < _data.BoundaryEdges.Count; i++)
            {
                var edge = _data.BoundaryEdges[i];
                Handles.DrawLine(edge.a, edge.b, FPNavMeshVisualizerStyles.EdgeBoundaryWidth);
            }
        }

        private void DrawObstacleRings()
        {
            if (_data.ObstacleRings == null) return;

            for (int r = 0; r < _data.ObstacleRings.Count; r++)
            {
                var ring = _data.ObstacleRings[r];
                int n = ring.Points.Length;
                if (n < 2) continue;

                // Ring edges — colored by winding (outer CW vs hole/pillar CCW).
                Handles.color = ring.IsCCW
                    ? FPNavMeshVisualizerStyles.ObstacleRingHole
                    : FPNavMeshVisualizerStyles.ObstacleRingOuter;
                Vector3 centroid = Vector3.zero;
                for (int i = 0; i < n; i++)
                {
                    Handles.DrawLine(ring.Points[i], ring.Points[(i + 1) % n],
                        FPNavMeshVisualizerStyles.ObstacleRingWidth);
                    centroid += ring.Points[i];
                }
                centroid /= n;

                // Per-vertex convexity dots (convex = white, reflex = amber).
                for (int i = 0; i < n; i++)
                {
                    Handles.color = ring.Convex[i]
                        ? FPNavMeshVisualizerStyles.ObstacleConvex
                        : FPNavMeshVisualizerStyles.ObstacleReflex;
                    Handles.DotHandleCap(0, ring.Points[i], Quaternion.identity,
                        FPNavMeshVisualizerStyles.ObstacleDotSize, EventType.Repaint);
                }

                // Ring label at the centroid (index · winding · vertex count). Stacked labels at one
                // spot reveal coincident/duplicate rings from raised (multi-level) geometry.
                Handles.color = Color.white;
                Handles.Label(centroid, $"r{r}·{(ring.IsCCW ? "H" : "O")}·{n}");
            }
        }

        private void DrawVertices()
        {
            Handles.color = FPNavMeshVisualizerStyles.Vertex;
            for (int i = 0; i < _data.CachedVertices.Length; i++)
            {
                Handles.DotHandleCap(0, _data.CachedVertices[i], Quaternion.identity,
                    FPNavMeshVisualizerStyles.VertexSize, EventType.Repaint);
            }
        }

        private void DrawTriangleIndices(SceneView sceneView)
        {
            var camPos = sceneView.camera.transform.position;
            float maxDist = 30f;
            float maxDistSqr = maxDist * maxDist;

            var style = new GUIStyle(GUI.skin.label)
            {
                fontSize = 10,
                alignment = TextAnchor.MiddleCenter,
                normal = { textColor = Color.white }
            };

            for (int i = 0; i < _data.CachedTriangles.Length; i++)
            {
                ref TriangleRenderData tri = ref _data.CachedTriangles[i];
                if ((tri.center - camPos).sqrMagnitude > maxDistSqr)
                    continue;

                Handles.Label(tri.center + Vector3.up * 0.1f, i.ToString(), style);
            }
        }

        private void DrawTriangleCenters()
        {
            Handles.color = FPNavMeshVisualizerStyles.TriangleCenter;
            for (int i = 0; i < _data.CachedTriangles.Length; i++)
            {
                Handles.DotHandleCap(0, _data.CachedTriangles[i].center, Quaternion.identity,
                    0.05f, EventType.Repaint);
            }
        }

        /// <summary>
        /// Where one node ends and the next begins — every mesh edge whose two triangles belong to
        /// different nodes. That set is EXACTLY the abstract graph's portals: an abstract edge is
        /// "two triangles that are mesh neighbours in different nodes", with no folding and no
        /// chosen representative, so nothing here is an approximation of what the planner uses.
        ///
        /// <para>Derived from the mesh rather than read out of the graph because the graph exposes
        /// one accessor by design (<c>NodeOf</c>) and this needs nothing more. What it deliberately
        /// does NOT reconstruct is edge COST: that would be a second copy of a formula, and a copy
        /// that drifts would show a wrong number with nothing to contradict it.</para>
        ///
        /// <para><c>_data.InternalEdges</c> is not the source: it holds vertex pairs and not the
        /// triangles they came from, so it cannot say which side belongs to which node.</para>
        /// </summary>
        private void DrawNodeBoundaries()
        {
            var mesh = _data.NavMesh;
            var graph = AbstractGraph;

            for (int t = 0; t < mesh.Triangles.Length; t++)
            {
                int a = graph.NodeOf(t);
                if (a < 0) continue;

                ref readonly FPNavMeshTriangle tri = ref mesh.Triangles[t];
                for (int e = 0; e < 3; e++)
                {
                    int nb = tri.GetNeighbor(e);
                    if (nb < 0) continue;
                    // Each shared edge between two IN-GRAPH triangles is visited from both sides;
                    // draw it once, from the lower index, so the line is not overdrawn (and so the
                    // two directed edges the graph holds map to one stroke).
                    //
                    // A RIM edge is exempt, and the exemption is what makes the rim complete. Its
                    // outer triangle is dropped by the `a < 0` continue above, so the edge is
                    // reached from the in-graph side ONLY — deduping it there dropped every rim
                    // edge whose outside neighbour happened to have the lower index, which is about
                    // half of them, with nothing to say they were missing.
                    if (nb < t && graph.NodeOf(nb) >= 0) continue;

                    int b = graph.NodeOf(nb);
                    // b < 0 is the graph's OUTER RIM, not a node boundary: the neighbour is masked
                    // out or blocked. Different meaning, different colour — merging them would draw
                    // a portal where the planner has none.
                    if (b == a) continue;

                    tri.GetEdgeVertices(e, out int va, out int vb);
                    Handles.color = b < 0
                        ? FPNavMeshVisualizerStyles.GraphRimLine
                        : FPNavMeshVisualizerStyles.NodeBoundaryLine;
                    Handles.DrawLine(
                        mesh.Vertices[va].ToVector3(), mesh.Vertices[vb].ToVector3(),
                        FPNavMeshVisualizerStyles.NodeBoundaryLineWidth);
                }
            }
        }

        private void DrawCostHeatmap()
        {
            // Find costMultiplier range
            float minCost = float.MaxValue;
            float maxCost = float.MinValue;
            for (int i = 0; i < _data.CachedTriangles.Length; i++)
            {
                float cost = _data.CachedTriangles[i].costMultiplier;
                if (cost < minCost) minCost = cost;
                if (cost > maxCost) maxCost = cost;
            }

            float range = maxCost - minCost;
            if (range < 0.001f) return;

            for (int i = 0; i < _data.CachedTriangles.Length; i++)
            {
                ref TriangleRenderData tri = ref _data.CachedTriangles[i];
                float t = (tri.costMultiplier - minCost) / range;
                Handles.color = Color.Lerp(
                    new Color(0f, 1f, 0f, 0.3f),
                    new Color(1f, 0f, 0f, 0.3f), t);
                Handles.DrawAAConvexPolygon(tri.v0, tri.v1, tri.v2);
            }
        }

        #endregion

        #region Grid

        private void DrawGrid()
        {
            if (_data.NavMesh == null) return;

            float originX = _data.NavMesh.GridOrigin.x.ToFloat();
            float originZ = _data.NavMesh.GridOrigin.y.ToFloat();
            float cellSize = _data.NavMesh.GridCellSize.ToFloat();
            int gridW = _data.NavMesh.GridWidth;
            int gridH = _data.NavMesh.GridHeight;

            Handles.color = FPNavMeshVisualizerStyles.GridLine;

            // Vertical lines
            for (int c = 0; c <= gridW; c++)
            {
                float x = originX + c * cellSize;
                Handles.DrawLine(
                    new Vector3(x, 0, originZ),
                    new Vector3(x, 0, originZ + gridH * cellSize),
                    FPNavMeshVisualizerStyles.GridLineWidth);
            }

            // Horizontal lines
            for (int r = 0; r <= gridH; r++)
            {
                float z = originZ + r * cellSize;
                Handles.DrawLine(
                    new Vector3(originX, 0, z),
                    new Vector3(originX + gridW * cellSize, 0, z),
                    FPNavMeshVisualizerStyles.GridLineWidth);
            }

            // Hovered cell highlight
            if (HoveredCell.col >= 0 && HoveredCell.row >= 0 &&
                HoveredCell.col < gridW && HoveredCell.row < gridH)
            {
                float cx = originX + HoveredCell.col * cellSize;
                float cz = originZ + HoveredCell.row * cellSize;
                Handles.color = FPNavMeshVisualizerStyles.GridHighlight;
                Handles.DrawAAConvexPolygon(
                    new Vector3(cx, 0.01f, cz),
                    new Vector3(cx + cellSize, 0.01f, cz),
                    new Vector3(cx + cellSize, 0.01f, cz + cellSize),
                    new Vector3(cx, 0.01f, cz + cellSize));
            }
        }

        private void DrawGridLabels(SceneView sceneView)
        {
            if (_data.NavMesh == null) return;

            var camPos = sceneView.camera.transform.position;
            float maxDist = 40f;
            float maxDistSqr = maxDist * maxDist;

            float originX = _data.NavMesh.GridOrigin.x.ToFloat();
            float originZ = _data.NavMesh.GridOrigin.y.ToFloat();
            float cellSize = _data.NavMesh.GridCellSize.ToFloat();
            int gridW = _data.NavMesh.GridWidth;
            int gridH = _data.NavMesh.GridHeight;

            var style = new GUIStyle(GUI.skin.label)
            {
                fontSize = 9,
                alignment = TextAnchor.MiddleCenter,
                normal = { textColor = FPNavMeshVisualizerStyles.GridCellLabel }
            };

            for (int r = 0; r < gridH; r++)
            {
                for (int c = 0; c < gridW; c++)
                {
                    Vector3 center = new Vector3(
                        originX + (c + 0.5f) * cellSize,
                        0.05f,
                        originZ + (r + 0.5f) * cellSize);

                    if ((center - camPos).sqrMagnitude > maxDistSqr)
                        continue;

                    int cellIdx = r * gridW + c;
                    int triCount = _data.NavMesh.GridCells[cellIdx * 2 + 1];
                    if (triCount > 0)
                        Handles.Label(center, $"({c},{r})\n{triCount}", style);
                }
            }
        }

        #endregion

        #region Path

        private void DrawCorridor()
        {
            Handles.color = FPNavMeshVisualizerStyles.CorridorFill;
            for (int i = 0; i < _data.CorridorLength; i++)
            {
                int triIdx = _data.Corridor[i];
                ref TriangleRenderData tri = ref _data.CachedTriangles[triIdx];
                Handles.DrawAAConvexPolygon(tri.v0, tri.v1, tri.v2);
            }
        }

        private void DrawWaypoints()
        {
            if (_data.WaypointCount < 2) return;

            // Connecting lines
            Handles.color = FPNavMeshVisualizerStyles.WaypointLine;
            for (int i = 0; i < _data.WaypointCount - 1; i++)
            {
                Handles.DrawLine(_data.Waypoints[i], _data.Waypoints[i + 1],
                    FPNavMeshVisualizerStyles.WaypointLineWidth);
            }

            // Dots
            Handles.color = FPNavMeshVisualizerStyles.WaypointDot;
            for (int i = 0; i < _data.WaypointCount; i++)
            {
                Handles.DotHandleCap(0, _data.Waypoints[i], Quaternion.identity,
                    FPNavMeshVisualizerStyles.WaypointDotSize, EventType.Repaint);
            }
        }

        private void DrawPortals()
        {
            Handles.color = FPNavMeshVisualizerStyles.PortalLine;
            for (int i = 0; i < _data.Portals.Count; i++)
            {
                var portal = _data.Portals[i];
                Handles.DrawLine(portal.left, portal.right,
                    FPNavMeshVisualizerStyles.PortalLineWidth);
            }
        }

        private void DrawStartEndMarkers()
        {
            if (_data.HasStart)
            {
                Handles.color = FPNavMeshVisualizerStyles.StartMarker;
                Handles.SphereHandleCap(0, _data.StartPoint, Quaternion.identity,
                    FPNavMeshVisualizerStyles.MarkerSize, EventType.Repaint);
            }

            if (_data.HasEnd)
            {
                Handles.color = FPNavMeshVisualizerStyles.EndMarker;
                Handles.SphereHandleCap(0, _data.EndPoint, Quaternion.identity,
                    FPNavMeshVisualizerStyles.MarkerSize, EventType.Repaint);
            }
        }

        #endregion

        #region Agents

        private void DrawAgents()
        {
            if (_agentSim == null) return;
            for (int i = 0; i < _agentSim.AgentCount; i++)
            {
                var rd = _agentSim.GetAgentRenderData(i);
                Handles.color = FPNavMeshVisualizerStyles.AgentBody;
                Handles.DrawWireDisc(rd.position, Vector3.up, rd.radius, 2f);

                // Agent index label
                Handles.Label(rd.position + Vector3.up * (rd.radius + 0.2f), $"#{i}");

                // Destination marker
                if (rd.hasDestination)
                {
                    Handles.color = FPNavMeshVisualizerStyles.AgentDestination;
                    Handles.SphereHandleCap(0, rd.destination, Quaternion.identity,
                        0.15f, EventType.Repaint);
                }

                // Where THIS leg ends, drawn only when it is not the destination — which is exactly
                // when a graph is steering. Without it the counters say legs are being taken and
                // nothing on screen shows where, so this is the picture that matches the number.
                if (ShowNodes && rd.hasPath && rd.pathTarget != rd.destination)
                {
                    Handles.color = FPNavMeshVisualizerStyles.LegTargetMarker;
                    Handles.DrawLine(rd.position, rd.pathTarget, 1.5f);
                    Handles.DotHandleCap(0, rd.pathTarget, Quaternion.identity,
                        FPNavMeshVisualizerStyles.LegTargetMarkerSize, EventType.Repaint);
                }
            }
        }

        private void DrawAgentPaths()
        {
            if (_agentSim == null) return;
            for (int i = 0; i < _agentSim.AgentCount; i++)
            {
                var rd = _agentSim.GetAgentRenderData(i);
                if (!rd.hasPath) continue;

                // Corridor-based path visualization
                if (rd.corridor != null && rd.corridorLength > 0 && _data != null)
                {
                    // Corridor triangle highlight
                    Handles.color = FPNavMeshVisualizerStyles.CorridorFill;
                    for (int c = 0; c < rd.corridorLength; c++)
                    {
                        int triIdx = rd.corridor[c];
                        if (triIdx >= 0 && triIdx < _data.CachedTriangles.Length)
                        {
                            ref TriangleRenderData tri = ref _data.CachedTriangles[triIdx];
                            Handles.DrawAAConvexPolygon(tri.v0, tri.v1, tri.v2);
                        }
                    }

                    // Extract dynamic corners via FindCorners and draw path lines
                    if (_data.Funnel != null)
                    {
                        FPVector3 currentPos = rd.position.ToFPVector3();
                        FPVector3 target = rd.destination.ToFPVector3();
                        int cornerCount = _data.Funnel.FindCorners(
                            rd.corridor, rd.corridorLength, currentPos, target, 8);
                        FPVector3[] corners = _data.Funnel.Corners;

                        if (cornerCount > 0)
                        {
                            Handles.color = FPNavMeshVisualizerStyles.WaypointLine;
                            Vector3 prev = rd.position;
                            for (int c = 0; c < cornerCount; c++)
                            {
                                Vector3 cp = corners[c].ToVector3();
                                Handles.DrawLine(prev, cp,
                                    FPNavMeshVisualizerStyles.WaypointLineWidth);
                                prev = cp;
                            }

                            Handles.color = FPNavMeshVisualizerStyles.WaypointDot;
                            for (int c = 0; c < cornerCount; c++)
                                Handles.DotHandleCap(0, corners[c].ToVector3(),
                                    Quaternion.identity,
                                    FPNavMeshVisualizerStyles.WaypointDotSize,
                                    EventType.Repaint);
                        }
                    }
                }
                // Legacy waypoint visualization removed — NavAgentComponent uses corridor-based only
            }
        }

        private void DrawAgentVelocities()
        {
            if (_agentSim == null) return;
            for (int i = 0; i < _agentSim.AgentCount; i++)
            {
                var rd = _agentSim.GetAgentRenderData(i);

                // Actual velocity
                if (rd.velocity.sqrMagnitude > 0.001f)
                {
                    Vector3 velEnd = rd.position + new Vector3(rd.velocity.x, 0, rd.velocity.y);
                    DrawArrow(rd.position, velEnd, FPNavMeshVisualizerStyles.AgentVelocity);
                }

                // Desired velocity
                if (rd.desiredVelocity.sqrMagnitude > 0.001f)
                {
                    Vector3 desEnd = rd.position + new Vector3(rd.desiredVelocity.x, 0, rd.desiredVelocity.y);
                    DrawArrow(rd.position, desEnd, FPNavMeshVisualizerStyles.AgentDesiredVel);
                }
            }
        }

        private void DrawOrcaVisualization()
        {
            if (_agentSim == null || _agentSim.Avoidance == null) return;

            var avoidance = _agentSim.Avoidance;
            int lineCount = avoidance.DebugOrcaLineCount;
            if (lineCount <= 0) return;

            // Visualize based on the last agent for which ORCA was computed
            int selectedIdx = _agentSim.LastOrcaComputedAgentIndex;
            if (selectedIdx < 0 || selectedIdx >= _agentSim.AgentCount) return;

            var agentRd = _agentSim.GetAgentRenderData(selectedIdx);
            Vector3 agentPos = agentRd.position;

            float lineLen = 3f;

            for (int i = 0; i < lineCount; i++)
            {
                var line = avoidance.DebugOrcaLines[i];
                Vector3 point = agentPos + new Vector3(
                    line.point.x.ToFloat(), 0, line.point.y.ToFloat());
                Vector3 dir = new Vector3(
                    line.direction.x.ToFloat(), 0, line.direction.y.ToFloat());

                Handles.color = FPNavMeshVisualizerStyles.OrcaLine;
                Handles.DrawLine(point - dir * lineLen, point + dir * lineLen, 1.5f);

                // Normal (into the half-plane)
                Vector3 normal = new Vector3(-dir.z, 0, dir.x);
                Handles.color = new Color(1f, 0.5f, 0f, 0.3f);
                Handles.DrawLine(point, point + normal * 0.5f, 1f);
            }

            // ORCA corrected velocity
            Vector3 orcaVelEnd = agentPos + new Vector3(
                agentRd.desiredVelocity.x, 0, agentRd.desiredVelocity.y);
            DrawArrow(agentPos, orcaVelEnd, FPNavMeshVisualizerStyles.OrcaVelocity, 0.2f);
        }

        private void DrawRuntimeAgents()
        {
            if (SelectedEntityIndex < 0 || RuntimeAgentSnapshotCount == 0)
                return;

            int snapshotIdx = -1;
            for (int i = 0; i < RuntimeAgentSnapshotCount; i++)
            {
                if (_runtimeSnapshotBuffer[i].Entity.Index == SelectedEntityIndex)
                {
                    snapshotIdx = i;
                    break;
                }
            }
            if (snapshotIdx < 0) return;

            const float agentRadius = 0.4f;
            ref NavAgentSnapshot snap = ref _runtimeSnapshotBuffer[snapshotIdx];

            // Agent circle
            Vector3 snapPos = snap.Position.ToVector3();
            Handles.color = FPNavMeshVisualizerStyles.AgentBody;
            Handles.DrawWireDisc(snapPos, Vector3.up, agentRadius, 2f);
            Handles.Label(snapPos + Vector3.up * (agentRadius + 0.2f),
                $"#{snap.Entity.Index}");

            // Highlight currently occupied triangle
            if (_data != null && snap.CurrentTriangleIndex >= 0 &&
                snap.CurrentTriangleIndex < _data.CachedTriangles.Length)
            {
                ref TriangleRenderData tri = ref _data.CachedTriangles[snap.CurrentTriangleIndex];
                Handles.color = new Color(0f, 0.5f, 1f, 0.25f);
                Handles.DrawAAConvexPolygon(tri.v0, tri.v1, tri.v2);
            }

            // Destination marker + path visualization
            if (snap.HasDestination)
            {
                Vector3 snapDest = snap.Destination.ToVector3();
                Handles.color = FPNavMeshVisualizerStyles.AgentDestination;
                Handles.DrawWireDisc(snapDest, Vector3.up, agentRadius, 2f);

                if (snap.HasPath)
                    DrawRuntimeAgentWaypoints(ref snap, snapPos, snapDest);
                else
                {
                    Handles.color = new Color(
                        FPNavMeshVisualizerStyles.AgentDestination.r,
                        FPNavMeshVisualizerStyles.AgentDestination.g,
                        FPNavMeshVisualizerStyles.AgentDestination.b, 0.9f);
                    Handles.DrawLine(snapPos, snapDest, FPNavMeshVisualizerStyles.WaypointLineWidth);
                }
            }
        }

        private void DrawRuntimeAgentWaypoints(ref NavAgentSnapshot snap, Vector3 snapPos, Vector3 snapDest)
        {
            var bridge = EcsDebugBridge.Instance;
            if (bridge == null) return;

            // Read once: the bridge reads through to the live provider, so repeated accesses
            // can straddle a rebake swap and pair a new mesh with an old query.
            FPNavMesh mesh = bridge.NavMesh;
            FPNavMeshQuery query = bridge.NavQuery;
            if (mesh == null || query == null) return;

            // Recreate Pathfinder/Funnel if the NavMesh has changed — with the GAME's tuning, read
            // off the bridge's query. This reconstructs the route a live agent would take, and a
            // default-tuned stack does not: a game that raised or lowered its budget, or turned
            // partial paths on, gets a different corridor here than its agents got, and where the
            // default budget runs out this drew the "no path" straight line under a unit that was
            // walking a partial. The query is the game's already; the pathfinder and funnel were
            // the odd ones out.
            if (_runtimeNavMeshRef != mesh)
            {
                _runtimeNavMeshRef = mesh;
                _runtimePathfinder = new FPNavMeshPathfinder(mesh, query, null, query.Tuning);
                _runtimeFunnel = new FPNavMeshFunnel(mesh, query, null, query.Tuning);
            }

            FPVector3 start = snap.Position;
            FPVector3 end = snap.Destination;

            // The AGENT's mask, not ~0. This reconstructs the route a live agent would take, and
            // the two stopped being the same value: ~0 admits a retained building footprint while
            // the agent's mask refuses it, so ~0 here would draw a path through a building the
            // agent will never enter — a wrong answer wearing the agent's label.
            // Zero minimum progress, like the Find Path tool: the agent's own minimum is its reach
            // radius, and this drawing only asks what ground the search reached.
            if (!_runtimePathfinder.FindPath(start, end, FPNavAgentSystem.DEFAULT_AREA_MASK, FP64.Zero,
                    out int[] corridor, out int corridorLength, out bool partial, out FPVector3 partialEnd))
            {
                // No path found — fallback to straight line
                Handles.color = new Color(
                    FPNavMeshVisualizerStyles.AgentDestination.r,
                    FPNavMeshVisualizerStyles.AgentDestination.g,
                    FPNavMeshVisualizerStyles.AgentDestination.b, 0.9f);
                Handles.DrawLine(snapPos, snapDest, FPNavMeshVisualizerStyles.WaypointLineWidth);
                return;
            }

            // Toward the corridor's actual end: a partial's last triangle, not the destination —
            // see FPNavMeshVisualizerData.FindPath for why the funnel must not be handed the goal.
            _runtimeFunnel.Funnel(corridor, corridorLength, start, partial ? partialEnd : end,
                out FPVector3[] waypoints, out int waypointCount);

            if (waypointCount < 1) return;

            for (int i = 0; i < waypointCount; i++)
                _runtimeWaypointBuf[i] = waypoints[i].ToVector3();

            // Same style as DrawWaypoints
            Handles.color = FPNavMeshVisualizerStyles.WaypointLine;
            for (int i = 0; i < waypointCount - 1; i++)
                Handles.DrawLine(_runtimeWaypointBuf[i], _runtimeWaypointBuf[i + 1],
                    FPNavMeshVisualizerStyles.WaypointLineWidth);

            Handles.color = FPNavMeshVisualizerStyles.WaypointDot;
            for (int i = 0; i < waypointCount; i++)
                Handles.DotHandleCap(0, _runtimeWaypointBuf[i], Quaternion.identity,
                    FPNavMeshVisualizerStyles.WaypointDotSize, EventType.Repaint);
        }

        #endregion

        #region Utilities

        private static void DrawArrow(Vector3 from, Vector3 to, Color color, float headSize = 0.15f)
        {
            Handles.color = color;
            Handles.DrawLine(from, to, 2f);

            Vector3 dir = (to - from).normalized;
            if (dir.sqrMagnitude < 0.001f) return;

            Vector3 right = Vector3.Cross(dir, Vector3.up).normalized;
            if (right.sqrMagnitude < 0.001f)
                right = Vector3.Cross(dir, Vector3.forward).normalized;

            Vector3 head = to - dir * headSize;
            Handles.DrawLine(to, head + right * headSize * 0.5f);
            Handles.DrawLine(to, head - right * headSize * 0.5f);
        }

        #endregion
    }
}
