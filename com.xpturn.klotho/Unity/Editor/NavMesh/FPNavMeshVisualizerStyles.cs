using UnityEngine;

namespace xpTURN.Klotho.Editor
{
    /// <summary>
    /// GUI style definitions for the NavMesh visualizer.
    /// </summary>
    internal static class FPNavMeshVisualizerStyles
    {
        // NavMesh geometry
        public static readonly Color TriangleFill = new Color(0.2f, 0.6f, 0.9f, 0.15f);
        public static readonly Color TriangleFillBlocked = new Color(0.9f, 0.2f, 0.2f, 0.25f);
        // A retained building footprint (FPNavMeshAreas.BUILDING_AREA): walkable geometry that the
        // default agent mask treats as a wall. Between the two above in meaning, and deliberately
        // between them in hue — without this it renders as ordinary ground and retain has no visual
        // trace at all (a carve at least leaves a hole).
        public static readonly Color TriangleFillBuilding = new Color(0.65f, 0.35f, 1.0f, 0.28f);
        public static readonly Color EdgeInternal = new Color(0.3f, 0.3f, 0.3f, 0.4f);
        public static readonly Color EdgeBoundary = new Color(1.0f, 0.4f, 0.0f, 0.9f);
        public static readonly Color Vertex = new Color(1.0f, 1.0f, 0.0f, 0.8f);
        public static readonly Color TriangleCenter = new Color(0.5f, 0.5f, 0.5f, 0.6f);

        public const float EdgeInternalWidth = 1.0f;
        public const float EdgeBoundaryWidth = 3.0f;

        // ORCA static-obstacle rings (extracted from the NavMesh boundary)
        public static readonly Color ObstacleRingOuter = new Color(1.0f, 0.15f, 0.55f, 0.95f); // CW: outer boundary
        public static readonly Color ObstacleRingHole  = new Color(0.15f, 0.85f, 1.0f, 0.95f); // CCW: hole / pillar
        public static readonly Color ObstacleConvex    = new Color(1.0f, 1.0f, 1.0f, 1.0f);    // convex obstacle vertex
        public static readonly Color ObstacleReflex    = new Color(1.0f, 0.85f, 0.0f, 1.0f);   // reflex obstacle vertex
        public const float ObstacleRingWidth = 2.5f;
        public const float ObstacleDotSize   = 0.06f;
        public const float VertexSize = 0.08f;

        // Path
        public static readonly Color CorridorFill = new Color(1.0f, 0.8f, 0.0f, 0.3f);
        public static readonly Color CorridorEdge = new Color(1.0f, 0.8f, 0.0f, 0.7f);
        public static readonly Color WaypointLine = new Color(0.0f, 1.0f, 0.3f, 0.9f);
        public static readonly Color WaypointDot = new Color(0.0f, 1.0f, 0.0f, 1.0f);
        public static readonly Color PortalLine = new Color(0.8f, 0.0f, 1.0f, 0.6f);

        public const float WaypointLineWidth = 3.0f;
        public const float WaypointDotSize = 0.15f;
        public const float PortalLineWidth = 2.0f;

        // Start/end markers
        public static readonly Color StartMarker = new Color(0.0f, 0.8f, 0.0f, 1.0f);
        public static readonly Color EndMarker = new Color(0.9f, 0.0f, 0.0f, 1.0f);
        public const float MarkerSize = 0.3f;

        // Agents
        public static readonly Color AgentBody = new Color(0.0f, 0.5f, 1.0f, 0.8f);
        public static readonly Color AgentVelocity = new Color(0.0f, 1.0f, 0.5f, 0.9f);
        public static readonly Color AgentDesiredVel = new Color(1.0f, 1.0f, 0.0f, 0.6f);
        public static readonly Color AgentDestination = new Color(1.0f, 0.0f, 0.5f, 0.8f);
        public static readonly Color AgentPath = new Color(0.0f, 0.7f, 1.0f, 0.5f);

        // ORCA
        public static readonly Color OrcaLine = new Color(1.0f, 0.5f, 0.0f, 0.7f);
        public static readonly Color OrcaVelocity = new Color(1.0f, 0.0f, 1.0f, 0.8f);

        // Spatial grid
        public static readonly Color GridLine = new Color(0.5f, 0.5f, 0.5f, 0.2f);
        public static readonly Color GridHighlight = new Color(1.0f, 1.0f, 0.0f, 0.15f);
        public static readonly Color GridCellLabel = new Color(1.0f, 1.0f, 1.0f, 0.6f);

        public const float GridLineWidth = 1.0f;

        // Abstract graph (planning in legs)
        public static readonly Color TriangleFillOutsideGraph = new Color(0.35f, 0.35f, 0.35f, 0.18f);
        public static readonly Color NodeBoundaryLine = new Color(1.0f, 1.0f, 1.0f, 0.75f);
        public static readonly Color GraphRimLine = new Color(0.35f, 0.35f, 0.35f, 0.5f);
        public static readonly Color LegTargetMarker = new Color(0.2f, 1.0f, 0.6f, 0.95f);

        public const float NodeBoundaryLineWidth = 2.0f;
        public const float LegTargetMarkerSize = 0.35f;

        /// <summary>
        /// A colour for a node id. Golden-ratio hue rotation, so ids that are near each other — and
        /// node ids ARE handed out in triangle order, which makes neighbours adjacent numbers — land
        /// far apart on the wheel and the partition reads as distinct regions.
        ///
        /// <para>Node ids are not stable across a rebake: they are reassigned in triangle order
        /// every derivation, so every colour changes when a building is placed. That is the ids
        /// being what they are, not the palette being unstable.</para>
        /// </summary>
        public static Color NodeFill(int node)
        {
            float hue = (node * 0.61803399f) % 1f;
            Color c = Color.HSVToRGB(hue, 0.55f, 0.95f);
            c.a = 0.30f;
            return c;
        }
    }
}
