using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using xpTURN.Klotho.Deterministic.Math;

namespace xpTURN.Klotho.Deterministic.Navigation.Tests
{
    /// <summary>
    /// IMP111 P0 — the measurement the partial-path threshold (D-3) is chosen from. Nothing here
    /// touches production code: it runs the real <see cref="FPNavMeshPathfinder"/> over the shipped
    /// assets, and after each search that ran out of budget it reads the search's own arrays back
    /// through reflection to find the node that got closest to the goal — exactly what the tracked
    /// best node will be once it exists.
    ///
    /// <para>Three questions, in the order the plan needs them answered:</para>
    /// <list type="number">
    /// <item><b>How much progress does the best node make?</b> As a fraction of the start's
    /// straight-line distance, for both endpoint candidates (entry point / triangle centre) and for
    /// the clipped corridor end when the chain exceeds the corridor cap (함정 2).</item>
    /// <item><b>Push- vs pop-time tracking.</b> The best over every touched node against the best
    /// over popped nodes only — whether tracking at push (free) picks a different node.</item>
    /// <item><b>Does chaining partial paths converge?</b> A unit is teleported to each partial end
    /// and asked again, under several margins; how many hops it takes and whether it arrives.</item>
    /// </list>
    ///
    /// Run explicitly, in Release:
    ///   dotnet test -c Release --filter FullyQualifiedName~FPNavPartialPathAnalysisTests
    /// </summary>
    [TestFixture]
    [Explicit("IMP111 P0 measurement — run in Release with an explicit filter")]
    public class FPNavPartialPathAnalysisTests
    {
        private static string RepoRoot()
        {
            var dir = new DirectoryInfo(AppContext.BaseDirectory);
            while (dir != null && !Directory.Exists(Path.Combine(dir.FullName, "com.xpturn.klotho")))
                dir = dir.Parent;
            Assert.IsNotNull(dir, "repo root not found from test base directory");
            return dir.FullName;
        }

        private static readonly string[] Assets =
        {
            "Samples/Brawler/Assets/NavMesh/Data/Field.NavMeshData.bytes",
            "Samples/Brawler/Assets/Brawler/Data/Stage01.NavMeshData.bytes",
            "Samples/Brawler/Assets/Brawler/Data/Stage02.NavMeshData.bytes",
            "Samples/Brawler/Assets/NavMesh/Data/8_heightmesh.NavMeshData.bytes",
        };

        /// <summary>Default agent: Speed 5, Acceleration 10 → reach radius v²/a = 2.5.</summary>
        private const double ReachRadius = 2.5;
        private static readonly double[] Margins = { 0.0, 0.01, 0.02, 0.05, 0.10 };
        private const int PairsPerConfig = 256;
        private const int MaxHops = 64;

        #region Reading the search back

        private static T Priv<T>(object o, string name)
        {
            var f = o.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.IsNotNull(f, $"private field {name} not found — the pathfinder changed shape");
            return (T)f.GetValue(o);
        }

        private sealed class Search
        {
            public int StartTri, EndTri;
            public FPVector2 StartXZ, EndXZ;
            public double H0;
            public int Touched, Popped;
            public int BestAny, BestPopped;      // argmin h over touched / over popped
            public double HBestAnyEntry, HBestAnyCentre;
            public double HBestPoppedEntry;
            public int ChainLen;                // start..bestAny
            public int WalkedTri;               // chain end after the corridor clamp (agent side kept)
            public double HWalkedCentre, HWalkedEntry;
        }

        private static double Dist(FPVector2 a, FPVector2 b) => FPVector2.Distance(a, b).ToDouble();

        private static FPVector3 Centre3(FPNavMesh mesh, int tri)
        {
            ref readonly var t = ref mesh.Triangles[tri];
            FP64 y = (mesh.Vertices[t.v0].y + mesh.Vertices[t.v1].y + mesh.Vertices[t.v2].y) / FP64.FromInt(3);
            return new FPVector3(t.centerXZ.x, y, t.centerXZ.y);
        }

        /// <summary>Reads the exhausted search that <c>FindPath</c> just left behind.</summary>
        private static Search Inspect(FPNavMeshPathfinder pf, FPNavMesh mesh, FPNavMeshQuery query,
            FPVector3 start, FPVector3 end, int corridorCap)
        {
            var gen = Priv<int[]>(pf, "_nodeGeneration");
            int generation = Priv<int>(pf, "_generation");
            var entry = Priv<FPVector2[]>(pf, "_entryPoints");
            var cameFrom = Priv<int[]>(pf, "_cameFrom");
            var closed = Priv<bool[]>(pf, "_closed");

            var s = new Search
            {
                StartTri = query.FindTriangle(start.ToXZ(), start.y),
                EndTri = query.FindTriangleForEndpoint(end.ToXZ(), end.y, FPNavAgentSystem.DEFAULT_AREA_MASK),
                StartXZ = start.ToXZ(),
                EndXZ = end.ToXZ(),
            };
            s.H0 = Dist(s.StartXZ, s.EndXZ);

            int bestAny = s.StartTri, bestPopped = s.StartTri;
            FP64 hAny = FPVector2.Distance(s.StartXZ, s.EndXZ), hPopped = hAny;
            for (int i = 0; i < gen.Length; i++)
            {
                if (gen[i] != generation) continue;
                s.Touched++;
                FP64 h = FPVector2.Distance(entry[i], s.EndXZ);
                if (h < hAny || (h == hAny && i < bestAny)) { hAny = h; bestAny = i; }
                if (closed[i])
                {
                    s.Popped++;
                    if (h < hPopped || (h == hPopped && i < bestPopped)) { hPopped = h; bestPopped = i; }
                }
            }
            s.BestAny = bestAny;
            s.BestPopped = bestPopped;
            s.HBestAnyEntry = hAny.ToDouble();
            s.HBestPoppedEntry = hPopped.ToDouble();
            s.HBestAnyCentre = Dist(mesh.Triangles[bestAny].centerXZ, s.EndXZ);

            // Chain start..best, then the clamp that keeps the agent side.
            var chain = new List<int>();
            for (int n = bestAny; n >= 0 && chain.Count <= gen.Length; n = cameFrom[n]) chain.Add(n);
            chain.Reverse();
            s.ChainLen = chain.Count;
            int walkedIdx = System.Math.Min(chain.Count, corridorCap) - 1;
            s.WalkedTri = chain[walkedIdx];
            s.HWalkedCentre = Dist(mesh.Triangles[s.WalkedTri].centerXZ, s.EndXZ);
            s.HWalkedEntry = Dist(entry[s.WalkedTri], s.EndXZ);
            return s;
        }

        #endregion

        #region Statistics

        private static string Quantiles(List<double> xs, string fmt = "F3")
        {
            if (xs.Count == 0) return "n/a";
            xs.Sort();
            double Q(double q) => xs[System.Math.Min(xs.Count - 1, (int)(q * xs.Count))];
            return $"min {xs[0].ToString(fmt)}  p10 {Q(0.10).ToString(fmt)}  p50 {Q(0.50).ToString(fmt)}  " +
                   $"p90 {Q(0.90).ToString(fmt)}  max {xs[xs.Count - 1].ToString(fmt)}";
        }

        private sealed class Config
        {
            public string Name;
            public FPNavMesh Mesh;
            public int Budget;
            public Func<int, (FPVector3 start, FPVector3 end)> Pair;   // k -> pair
            public int Pairs = PairsPerConfig;
        }

        private static IEnumerable<Config> AssetConfigs(string root)
        {
            foreach (string rel in Assets)
            {
                string path = Path.Combine(root, rel);
                if (!File.Exists(path)) continue;
                FPNavMesh mesh = FPNavMeshSerializer.Deserialize(path);
                var walkable = new List<int>();
                for (int t = 0; t < mesh.Triangles.Length; t++)
                    if (!mesh.Triangles[t].isBlocked) walkable.Add(t);
                int n = walkable.Count;
                string name = Path.GetFileNameWithoutExtension(rel).Replace(".NavMeshData", "");

                FPVector3 Centre(int tri) => Centre3(mesh, tri);
                (FPVector3, FPVector3) Pair(int k) => (
                    Centre(walkable[(int)((k * 7919L) % n)]),
                    Centre(walkable[(int)((k * 104729L + 12345L) % n)]));

                foreach (int budget in new[] { FPNavMeshPathfinder.MAX_ITERATIONS, 1024 })
                    yield return new Config { Name = $"{name} ({mesh.Triangles.Length} tris)", Mesh = mesh, Budget = budget, Pair = Pair };
            }
        }

        private static IEnumerable<Config> SyntheticConfigs()
        {
            {
                // The crowd-scaling field: 96x96 cells, orders across the map, budget 512 (the
                // configuration the legs plan measured 45% failures on).
                var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(96);
                (FPVector3, FPVector3) Pair(int k)
                {
                    int sx = (k * 37) % 96, sz = (k * 53) % 96;
                    int ex = (sx + 48 + (k * 11) % 40) % 96, ez = (sz + 48 + (k * 17) % 40) % 96;
                    return (NavAgentTestHelper.CellCenter(sx, sz), NavAgentTestHelper.CellCenter(ex, ez));
                }
                yield return new Config { Name = "open field 96x96 (budget 512)", Mesh = mesh, Budget = 512, Pair = Pair };
            }
            {
                // The worst case: the heuristic points across the rows while the route doubles back.
                var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(64, 40, out var endCell);
                (FPVector3, FPVector3) Pair(int k)
                {
                    int row = (k * 7) % 40;
                    int gx = (row % 2 == 0) ? (k * 13) % 64 : 63 - (k * 13) % 64;
                    return (NavAgentTestHelper.CellCenter(gx, row * 2), NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz));
                }
                yield return new Config { Name = "serpentine 64x40 (default budget)", Mesh = mesh, Budget = FPNavMeshPathfinder.MAX_ITERATIONS, Pair = Pair, Pairs = 64 };
            }
        }

        #endregion

        [Test]
        public void P0_HowFarDoesTheBestNodeGet_AndDoPartialHopsConverge()
        {
            string root = RepoRoot();
            TestContext.Out.WriteLine("=== IMP111 P0 — progress of the best node on exhausted searches, and hop convergence ===");
            TestContext.Out.WriteLine($"    reach radius (default agent v²/a) = {ReachRadius}; margins tried = {string.Join(", ", Margins)}");
            TestContext.Out.WriteLine("");

            var configs = new List<Config>();
            configs.AddRange(AssetConfigs(root));
            configs.AddRange(SyntheticConfigs());

            foreach (var cfg in configs)
                Measure(cfg);
        }

        private static void Measure(Config cfg)
        {
            // Off, named: this harness inspects the search a budget failure leaves behind, and with
            // the 0.13 default (on) FindPath would hand back a partial and there would be nothing to inspect.
            var tuning = new FPNavTuning(maxIterations: cfg.Budget, partialPathOnExhaustion: false);
            var query = new FPNavMeshQuery(cfg.Mesh, null, tuning);
            var pf = new FPNavMeshPathfinder(cfg.Mesh, query, null, tuning);
            int cap = tuning.CorridorCap;

            int searched = 0, found = 0, noRoute = 0, exhausted = 0, clipped = 0, bestDiffers = 0, bestIsStart = 0;
            var fracEntry = new List<double>();
            var fracCentre = new List<double>();
            var fracWalked = new List<double>();
            var absWalked = new List<double>();
            var h0s = new List<double>();
            var hops2 = new Dictionary<(double, bool), List<int>>();
            var reached2 = new Dictionary<(double, bool), int>();
            var stoppedNoProgress2 = new Dictionary<(double, bool), int>();
            var stoppedNoRoute2 = new Dictionary<(double, bool), int>();
            var stoppedMaxHops2 = new Dictionary<(double, bool), int>();
            var cycles2 = new Dictionary<(double, bool), int>();
            foreach (double m in Margins)
                foreach (bool jb in new[] { false, true })
                {
                    var key = (m, jb);
                    hops2[key] = new List<int>(); reached2[key] = 0; stoppedNoProgress2[key] = 0;
                    stoppedNoRoute2[key] = 0; stoppedMaxHops2[key] = 0; cycles2[key] = 0;
                }

            for (int k = 0; k < cfg.Pairs; k++)
            {
                var (start, end) = cfg.Pair(k);
                int exhaustedBefore = pf.DebugIterationExhaustedCount;
                bool ok = pf.FindPath(start, end, FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _);
                if (pf.DebugLastSearchIterations == 0) continue;     // did not search (off-mesh, same tri, ...)
                searched++;
                if (ok) { found++; continue; }
                if (pf.DebugIterationExhaustedCount == exhaustedBefore) { noRoute++; continue; }
                exhausted++;

                var s = Inspect(pf, cfg.Mesh, query, start, end, cap);
                if (s.BestAny == s.StartTri) bestIsStart++;
                if (s.BestAny != s.BestPopped) bestDiffers++;
                if (s.ChainLen > cap) clipped++;
                h0s.Add(s.H0);
                fracEntry.Add((s.H0 - s.HBestAnyEntry) / s.H0);
                fracCentre.Add((s.H0 - s.HBestAnyCentre) / s.H0);
                fracWalked.Add((s.H0 - s.HWalkedCentre) / s.H0);
                absWalked.Add(s.H0 - s.HWalkedCentre);

                // Hop simulation: teleport to the walked point (centre of the clipped chain end) and
                // ask again, until found / no route / no progress under the margin / too many hops.
                // Two rules for "progress": R1 judges the WALKED point (the clipped chain end), R2
                // judges the BEST node (the evidence the search found) and walks the chain regardless.
                foreach (double m in Margins)
                {
                    foreach (bool judgeBest in new[] { false, true })
                    {
                        var key = (m, judgeBest);
                        FPVector3 pos = start;
                        int hop = 0;
                        string stop = null;
                        var seen = new HashSet<int>();
                        while (true)
                        {
                            int exB = pf.DebugIterationExhaustedCount;
                            bool f = pf.FindPath(pos, end, FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _);
                            if (f) { stop = "reached"; break; }
                            if (pf.DebugIterationExhaustedCount == exB) { stop = "noroute"; break; }
                            var hs = Inspect(pf, cfg.Mesh, query, pos, end, cap);
                            double required = m * hs.H0 + ReachRadius;
                            double progress = judgeBest ? hs.H0 - hs.HBestAnyCentre : hs.H0 - hs.HWalkedCentre;
                            if (progress < required || hs.WalkedTri == hs.StartTri) { stop = "noprogress"; break; }
                            if (!seen.Add(hs.WalkedTri)) { stop = "cycle"; break; }
                            hop++;
                            if (hop >= MaxHops) { stop = "maxhops"; break; }
                            pos = Centre3(cfg.Mesh, hs.WalkedTri);
                        }
                        hops2[key].Add(hop);
                        if (stop == "reached") reached2[key]++;
                        else if (stop == "noroute") stoppedNoRoute2[key]++;
                        else if (stop == "noprogress") stoppedNoProgress2[key]++;
                        else if (stop == "cycle") cycles2[key]++;
                        else stoppedMaxHops2[key]++;
                    }
                }
            }

            TestContext.Out.WriteLine($"--- {cfg.Name}, budget {cfg.Budget}, corridor cap {cap} ---");
            TestContext.Out.WriteLine($"  searches {searched}: found {found}, no route {noRoute}, EXHAUSTED {exhausted}");
            if (exhausted == 0) { TestContext.Out.WriteLine(""); return; }
            TestContext.Out.WriteLine($"  h0 (start→goal, units)         {Quantiles(h0s, "F1")}");
            TestContext.Out.WriteLine($"  best == start (no progress)    {bestIsStart}/{exhausted}");
            TestContext.Out.WriteLine($"  best(touched) != best(popped)  {bestDiffers}/{exhausted}   (push-time vs pop-time tracking)");
            TestContext.Out.WriteLine($"  chain > cap (clipped)          {clipped}/{exhausted}");
            TestContext.Out.WriteLine($"  progress fraction, entry(best)   {Quantiles(fracEntry)}");
            TestContext.Out.WriteLine($"  progress fraction, centre(best)  {Quantiles(fracCentre)}");
            TestContext.Out.WriteLine($"  progress fraction, centre(walked){Quantiles(fracWalked)}");
            TestContext.Out.WriteLine($"  progress units,    centre(walked){Quantiles(absWalked, "F2")}   vs reach radius {ReachRadius}");
            foreach (bool jb in new[] { false, true })
            {
                TestContext.Out.WriteLine(jb
                    ? "  R2 — judge the BEST node, walk the clipped chain:"
                    : "  R1 — judge the WALKED point (clipped chain end):");
                foreach (double m in Margins)
                {
                    var key = (m, jb);
                    var hl = hops2[key];
                    TestContext.Out.WriteLine(
                        $"    margin {m,4:P0}+{ReachRadius}: reached {reached2[key]}/{exhausted}, " +
                        $"no-progress {stoppedNoProgress2[key]}, no-route {stoppedNoRoute2[key]}, " +
                        $"CYCLE {cycles2[key]}, max-hops {stoppedMaxHops2[key]}; " +
                        $"hops {Quantiles(hl.ConvertAll(x => (double)x), "F0")}");
                }
            }
            TestContext.Out.WriteLine("");
        }
    }
}
