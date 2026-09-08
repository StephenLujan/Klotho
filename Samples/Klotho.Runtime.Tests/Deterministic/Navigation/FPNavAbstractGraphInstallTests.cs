using System;
using System.IO;

using NUnit.Framework;

using xpTURN.Klotho.Deterministic.Math;
using xpTURN.Klotho.Helper.Tests;
using xpTURN.Klotho.Logging;

namespace xpTURN.Klotho.Deterministic.Navigation.Tests
{
    /// <summary>
    /// <see cref="FPNavAgentSystem.TryInstallAbstractGraphIfBeneficial"/> — the one-line form of
    /// wiring legs: it decides both whether this mesh needs a graph and what cell size to build it
    /// at, so a game does not have to. These pin that decision — the two thresholds it reads, the
    /// cell-size search, the refusals it returns as values, and the fingerprint it moves.
    ///
    /// <para><b>The two thresholds are tested through a reduced tuning, not the shipped one.</b>
    /// That is not a shortcut: the helper has to read the tuning this system runs on rather than
    /// the constants those defaults come from, and a fixture that only ever uses the defaults
    /// cannot tell the two apart. Shrinking the caps makes the boundary reachable with a mesh
    /// small enough to be exact about, and proves the helper reads the right value at the same
    /// time.</para>
    ///
    /// <para>The serpentine is the fixture for both: it is the worst case for A*, so if any query
    /// on a mesh under the budget could exhaust, this one would.</para>
    /// </summary>
    [TestFixture]
    public class FPNavAbstractGraphInstallTests
    {
        private static string RepoRoot()
        {
            var dir = new DirectoryInfo(AppContext.BaseDirectory);
            while (dir != null && !Directory.Exists(Path.Combine(dir.FullName, "com.xpturn.klotho")))
                dir = dir.Parent;
            Assert.IsNotNull(dir, "repo root not found from test base directory");
            return dir.FullName;
        }

        private static FPNavMesh LoadAsset(string relative)
        {
            string path = Path.Combine(RepoRoot(), relative);
            if (!File.Exists(path))
                Assert.Ignore($"asset not present: {relative}");
            return FPNavMeshSerializer.Deserialize(path);
        }

        private const string FieldAsset = "Samples/Brawler/Assets/NavMesh/Data/Field.NavMeshData.bytes";
        private const string Stage01Asset = "Samples/Brawler/Assets/Brawler/Data/Stage01.NavMeshData.bytes";
        private const string Stage02Asset = "Samples/Brawler/Assets/Brawler/Data/Stage02.NavMeshData.bytes";

        // Since 0.13 the default tuning installs a graph from the constructor on any mesh past the
        // budget. The tests below that exercise the EXPLICIT call on such a mesh build on this, so
        // that the call has something to do — with the default they would see AlreadyInstalled.
        private static readonly FPNavTuning NoAuto = new FPNavTuning(autoInstallAbstractGraph: false);

        private static FPNavAgentSystem CreateSystem(
            FPNavMesh mesh, FPNavTuning tuning, out FPNavMeshPathfinder pathfinder,
            IKLogger logger = null)
        {
            var query = new FPNavMeshQuery(mesh, logger, tuning);
            pathfinder = new FPNavMeshPathfinder(mesh, query, logger, tuning);
            var funnel = new FPNavMeshFunnel(mesh, query, logger, tuning);
            return new FPNavAgentSystem(mesh, query, pathfinder, funnel, logger, tuning);
        }

        #region V-A1 — the two thresholds are exact necessary conditions

        /// <summary>
        /// <b>At or below the corridor cap, clamping is impossible.</b> The <c>cameFrom</c> chain is
        /// acyclic — <c>IsClosed</c> is generation-guarded, so a triangle closes once — and
        /// <c>ReconstructCorridor</c> counts that chain in triangles. A chain can therefore never be
        /// longer than the mesh, so a mesh no larger than the cap cannot produce a corridor the cap
        /// has to cut.
        /// </summary>
        [Test]
        public void VA1_AtOrBelowTheCorridorCap_ClampingIsImpossible()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(6, 5, out var endCell);
            int triangles = mesh.Triangles.Length;
            Assert.LessOrEqual(triangles, FPNavMeshPathfinder.MAX_CORRIDOR,
                "fixture: the cap must be settable to the triangle count, and the storage ceiling binds");

            var tuning = new FPNavTuning(corridorCap: triangles);
            CreateSystem(mesh, tuning, out var pathfinder);

            Assert.IsTrue(pathfinder.FindPath(
                NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out int length));
            Assert.Greater(length, 1, "fixture: the serpentine must actually route");

            Assert.Zero(pathfinder.DebugCorridorTruncatedCount,
                "a mesh no larger than the corridor cap cannot hand back a clamped corridor — "
                + "the chain is bounded by the triangle count");
        }

        /// <summary>
        /// <b>Past the cap, clamping becomes possible — not certain.</b> This is one instance of the
        /// possibility, which is all the direction can be: the condition is necessary, so a mesh
        /// past the line only means the failure is reachable.
        /// </summary>
        [Test]
        public void VA1_PastTheCorridorCap_ClampingBecomesPossible()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(6, 5, out var endCell);
            var tuning = new FPNavTuning(corridorCap: 8);
            Assert.Greater(mesh.Triangles.Length, tuning.CorridorCap, "fixture: past the line");

            CreateSystem(mesh, tuning, out var pathfinder);
            pathfinder.FindPath(
                NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _);

            Assert.Greater(pathfinder.DebugCorridorTruncatedCount, 0,
                "the serpentine forces a corridor longer than a cap of 8");
        }

        /// <summary>
        /// <b>At or below the iteration budget, exhaustion is impossible.</b> The open set is an
        /// indexed heap (<c>Contains</c> + <c>DecreaseKey</c>), so a triangle enters it at most once
        /// and pops can never exceed the triangle count. Reaching the budget therefore means every
        /// triangle was popped, which leaves the open set empty — and the counter is guarded on the
        /// open set, not on the iteration count.
        /// </summary>
        [Test]
        public void VA1_AtOrBelowTheIterationBudget_ExhaustionIsImpossible()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 9, out var endCell);
            var tuning = new FPNavTuning(maxIterations: mesh.Triangles.Length);
            CreateSystem(mesh, tuning, out var pathfinder);

            // The extremes force near-total expansion; the other pairs keep this from resting on a
            // single query on the worst-case mesh.
            pathfinder.FindPath(
                NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _);
            pathfinder.FindPath(
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                NavAgentTestHelper.CellCenter(0, 0),
                FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _);
            pathfinder.FindPath(
                NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(0, 0),
                FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _);

            Assert.Zero(pathfinder.DebugIterationExhaustedCount,
                "a mesh no larger than the budget cannot run out of it — pops are bounded by the "
                + "triangle count, and the last pop empties the open set");
        }

        /// <summary>
        /// <b>Past the budget, exhaustion becomes possible.</b> The mirror of the cap case, and the
        /// same caveat: one instance of a possibility, not a proof of necessity.
        /// </summary>
        [Test]
        public void VA1_PastTheIterationBudget_ExhaustionBecomesPossible()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(10, 9, out var endCell);
            var tuning = new FPNavTuning(maxIterations: 8);
            Assert.Greater(mesh.Triangles.Length, tuning.MaxIterations, "fixture: past the line");

            CreateSystem(mesh, tuning, out var pathfinder);
            pathfinder.FindPath(
                NavAgentTestHelper.CellCenter(0, 0),
                NavAgentTestHelper.CellCenter(endCell.gx, endCell.gz),
                FPNavAgentSystem.DEFAULT_AREA_MASK, out _, out _);

            Assert.Greater(pathfinder.DebugIterationExhaustedCount, 0,
                "the serpentine cannot be crossed inside 8 pops");
        }

        #endregion

        #region V-A2 / V-A10 — what the helper builds, and what it costs

        /// <summary>
        /// <b>The reported cell size rebuilds the identical graph.</b> The helper chooses the size,
        /// so without it coming back out there is nothing to compare against and no way for a tool
        /// or another peer to reconstruct the partition. Fingerprints are the comparison because the
        /// graph's checksum folds into one and the system keeps the instance private.
        ///
        /// <para>The size itself is pinned: it is build identity, and the ladder landing somewhere
        /// else means every peer's route changed.</para>
        /// </summary>
        [Test]
        public void VA2_TheReportedCellSizeRebuildsTheIdenticalGraph()
        {
            var mesh = LoadAsset(FieldAsset);
            var helped = CreateSystem(mesh, NoAuto, out _);

            var outcome = helped.TryInstallAbstractGraphIfBeneficial(out FP64 cellSize);
            Assert.AreEqual(FPNavAbstractGraphInstall.Installed, outcome);
            Assert.AreEqual(32.0, cellSize.ToDouble(), 1e-9,
                "the ladder starts at GridCellSize x 16 (64) and halves; 64 leaves a node wider "
                + "than the cap on Field, 32 does not");

            var byHand = CreateSystem(mesh, NoAuto, out _);
            byHand.SetAbstractGraph(new FPNavAbstractGraph(
                mesh, cellSize, FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK));

            Assert.AreEqual(byHand.GetNavFingerprint(), helped.GetNavFingerprint(),
                "the helper must build exactly what the reported arguments build, or a peer that "
                + "wires it by hand walks a different partition");
        }

        /// <summary>
        /// <b>Installing moves the navigation fingerprint.</b> The executable form of the contract:
        /// a game that adds the call loses the replays it recorded without it. V-A6 is the other
        /// half — not calling it changes nothing.
        /// </summary>
        [Test]
        public void VA10_InstallingMovesTheNavFingerprint()
        {
            var mesh = LoadAsset(FieldAsset);
            var system = CreateSystem(mesh, NoAuto, out _);

            long before = system.GetNavFingerprint();
            Assert.AreEqual(FPNavAbstractGraphInstall.Installed,
                system.TryInstallAbstractGraphIfBeneficial(out _));
            long after = system.GetNavFingerprint();

            Assert.AreNotEqual(before, after,
                "the graph digest folds into the fingerprint, so installing one is a different "
                + "build — which is what makes the Ready exchange catch a peer that did not");
        }

        #endregion

        #region V-A3 / V-A4 / V-A5 — the three ways it declines

        /// <summary>
        /// <b>Small stages get no graph, and the reason is <c>NotNeeded</c>.</b> Neither failure is
        /// reachable there, so a graph would move the fingerprint for nothing.
        /// </summary>
        [TestCase(Stage01Asset)]
        [TestCase(Stage02Asset)]
        public void VA3_AStageUnderBothThresholdsGetsNoGraph(string asset)
        {
            var mesh = LoadAsset(asset);
            var system = CreateSystem(mesh, FPNavTuning.Default, out _);
            long before = system.GetNavFingerprint();

            Assert.AreEqual(FPNavAbstractGraphInstall.NotNeeded,
                system.TryInstallAbstractGraphIfBeneficial(out FP64 cellSize));
            Assert.AreEqual(0.0, cellSize.ToDouble(), 1e-9, "nothing was installed, so there is no size");
            Assert.AreEqual(before, system.GetNavFingerprint(),
                "declining must be free — a stage that does not need legs must record replays that "
                + "still load after this call is added");
        }

        /// <summary>
        /// <b>A graph the game installed itself is left alone, and the outcome says which of the
        /// three no-graph answers this was.</b> Folding them into one <c>false</c> would throw away
        /// the difference between <i>your stage is small</i> and <i>you already decided</i>.
        /// </summary>
        [Test]
        public void VA4_AnAlreadyInstalledGraphIsLeftAlone_AndSaysSo()
        {
            var mesh = LoadAsset(FieldAsset);
            var system = CreateSystem(mesh, FPNavTuning.Default, out _);
            system.SetAbstractGraph(new FPNavAbstractGraph(
                mesh, FP64.FromInt(16), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK));
            long chosenByHand = system.GetNavFingerprint();

            Assert.AreEqual(FPNavAbstractGraphInstall.AlreadyInstalled,
                system.TryInstallAbstractGraphIfBeneficial(out FP64 cellSize));
            Assert.AreEqual(0.0, cellSize.ToDouble(), 1e-9);
            Assert.AreEqual(chosenByHand, system.GetNavFingerprint(),
                "the game's own cell size must survive — silently replacing it moves every agent "
                + "onto a partition the game did not choose");
        }

        /// <summary>
        /// <b>A mesh no cell size can serve is a value, not an exception.</b> Games call this on the
        /// initialization path, where <see cref="FPNavAgentSystem.SetAbstractGraph"/>'s throw is a
        /// failed boot.
        /// </summary>
        [Test]
        public void VA5_WhenNoCellSizeFits_ItRefusesByValue()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var tuning = new FPNavTuning(corridorCap: 4);
            var system = CreateSystem(mesh, tuning, out _);
            Assert.Greater(mesh.Triangles.Length, tuning.CorridorCap, "fixture: the helper must get past its gate");

            FPNavAbstractGraphInstall outcome = default;
            FP64 cellSize = FP64.Zero;
            Assert.DoesNotThrow(
                () => outcome = system.TryInstallAbstractGraphIfBeneficial(out cellSize),
                "an initialization-path helper must never throw");

            Assert.AreEqual(FPNavAbstractGraphInstall.NoCellSizeFits, outcome);
            Assert.AreEqual(0.0, cellSize.ToDouble(), 1e-9);
        }

        #endregion

        #region V-A6 — not calling it changes nothing

        /// <summary>
        /// <b>The no-graph fingerprint is pinned.</b> The claim is that a build which never calls
        /// the helper is bit-for-bit what it was before the helper existed; the executable form of
        /// that is a golden, because anything short of one only says the value is stable within this
        /// run. If this fails, existing replays stopped loading for every game — whether or not it
        /// adopted legs.
        /// </summary>
        [Test]
        public void VA6_NotCallingItLeavesTheFingerprintWhereItWas()
        {
            var mesh = LoadAsset(FieldAsset);

            // Redefined in 0.13: "not calling it" became "turning the automatic install off". The
            // OFF stack is the pre-0.13 system bit for bit, and the default one now carries the
            // graph the constructor installed — pinned to the value the explicit helper produced on
            // this mesh before the flip, which V-B10 in FPNavAutoLegsTests proves is the same graph.
            var off = CreateSystem(mesh, NoAuto, out _);
            Assert.AreEqual(FieldNoGraphFingerprint, off.GetNavFingerprint(),
                "the navigation fingerprint of a system with no abstract graph must not move — the "
                + "graph digest contributes zero when there is none, and that zero is the promise "
                + "that turning the automatic install off costs nothing to a game that does");

            var auto = CreateSystem(mesh, FPNavTuning.Default, out _);
            Assert.AreEqual(unchecked((long)0xB5E5C29564219934UL), auto.GetNavFingerprint(),
                "the default stack on Field carries the automatic graph (cell 32, 81 nodes) — this "
                + "is what a 0.13 game that never named a tuning is refused against");
        }

        /// <summary>
        /// Measured on Field with the shipped tuning and no graph installed (0x5BB050AA23A9B9B3 as
        /// unsigned; 0xC68B2F847FC3A2F4 before 0.13 turned partial paths on by default — the flip
        /// moved it by exactly the partial-path term). Regenerate only alongside a deliberate
        /// NAV_BEHAVIOUR_REVISION bump or a default-tuning change that is meant to refuse old
        /// replays — a surprise change here is the failure V-A6 exists to catch.
        /// </summary>
        private const long FieldNoGraphFingerprint = unchecked((long)0x5BB050AA23A9B9B3UL);

        #endregion

        #region The decision is readable from a game's boot log

        /// <summary>
        /// <b>Installing says so, with the numbers and the fingerprint.</b> A game wired like the
        /// sample reads this line at boot; it is where the cell size the ladder settled on and the
        /// fact that replays just moved both become visible.
        /// </summary>
        [Test]
        public void Installing_SaysWhatItChoseAndWhatItCost()
        {
            var mesh = LoadAsset(FieldAsset);
            var log = new LogCapture();
            var system = CreateSystem(mesh, NoAuto, out _, log);

            Assert.AreEqual(FPNavAbstractGraphInstall.Installed,
                system.TryInstallAbstractGraphIfBeneficial(out _));

            Assert.IsTrue(log.Contains(KLogLevel.Information, "planning in legs at cell 32.00"),
                "the chosen cell size has to be in the log — it is build identity, and a peer that "
                + "disagrees about it is the failure the fingerprint exists to catch");
            Assert.IsTrue(log.Contains(KLogLevel.Information, "replays recorded without a graph will refuse"),
                "and the cost has to be said where the person who added the call will read it");
        }

        /// <summary>
        /// <b>The quiet branch is the one that most needed a line.</b> A small stage takes it every
        /// boot, and while it was silent "legs are off because this mesh does not need them" could
        /// not be told apart from "the call was never wired" — two states with very different fixes.
        ///
        /// <para>This is the same failure the feature exists to remove, one level up: a fact that
        /// was true, knowable, and had no readable form.</para>
        /// </summary>
        [TestCase(Stage01Asset)]
        [TestCase(Stage02Asset)]
        public void DecliningAsNotNeeded_IsStillSaidOutLoud(string asset)
        {
            var mesh = LoadAsset(asset);
            var log = new LogCapture();
            var system = CreateSystem(mesh, FPNavTuning.Default, out _, log);

            Assert.AreEqual(FPNavAbstractGraphInstall.NotNeeded,
                system.TryInstallAbstractGraphIfBeneficial(out _));

            Assert.IsTrue(log.Contains(KLogLevel.Information, "planning in legs: off — not needed"),
                "silence here is indistinguishable from the call not having run");
            Assert.IsTrue(log.Contains(KLogLevel.Information, mesh.Triangles.Length.ToString()),
                "with the triangle count, so the margin to the caps can be read before one is crossed");
        }

        /// <summary>
        /// <b>Leaving a game's own graph alone is also a decision, so it is logged.</b> Otherwise a
        /// game that installed one by hand and then added the helper cannot tell which of the two
        /// its agents are planning against.
        /// </summary>
        [Test]
        public void LeavingAnExistingGraphAlone_IsLogged()
        {
            var mesh = LoadAsset(FieldAsset);
            var log = new LogCapture();
            var system = CreateSystem(mesh, FPNavTuning.Default, out _, log);
            system.SetAbstractGraph(new FPNavAbstractGraph(
                mesh, FP64.FromInt(16), FPNavAbstractCostFold.Min, FPNavAgentSystem.DEFAULT_AREA_MASK));
            log.Clear();

            Assert.AreEqual(FPNavAbstractGraphInstall.AlreadyInstalled,
                system.TryInstallAbstractGraphIfBeneficial(out _));

            Assert.IsTrue(log.Contains(KLogLevel.Information, "already on and left alone"));
        }

        /// <summary>
        /// <b>And the refusal is an error, because it is the one outcome a game should act on.</b>
        /// The mesh wanted legs and could not have them.
        /// </summary>
        [Test]
        public void RefusingForWantOfACellSize_IsAnError()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(16);
            var log = new LogCapture();
            var system = CreateSystem(mesh, new FPNavTuning(corridorCap: 4), out _, log);

            Assert.AreEqual(FPNavAbstractGraphInstall.NoCellSizeFits,
                system.TryInstallAbstractGraphIfBeneficial(out _));

            Assert.IsTrue(log.Contains(KLogLevel.Error, "no cell size fits"));
        }

        /// <summary>
        /// <b>Whatever happens, exactly one line comes out.</b> Stated as a property over all four
        /// outcomes rather than four separate assertions, because the thing that breaks is a new
        /// early return added later with no line on it — and that is invisible to a test that only
        /// names the branches that exist today.
        /// </summary>
        [Test]
        public void EveryOutcome_ProducesExactlyOneLine()
        {
            (FPNavMesh mesh, FPNavTuning tuning, bool preInstall)[] cases =
            {
                (LoadAsset(FieldAsset), FPNavTuning.Default, false),                          // Installed
                (LoadAsset(Stage02Asset), FPNavTuning.Default, false),                        // NotNeeded
                (LoadAsset(FieldAsset), FPNavTuning.Default, true),                           // AlreadyInstalled
                (NavAgentTestHelper.CreateOpenFieldNavMesh(16), new FPNavTuning(corridorCap: 4), false),
            };

            foreach (var (mesh, tuning, preInstall) in cases)
            {
                var log = new LogCapture();
                var system = CreateSystem(mesh, tuning, out _, log);
                if (preInstall)
                    system.SetAbstractGraph(new FPNavAbstractGraph(
                        mesh, FP64.FromInt(16), FPNavAbstractCostFold.Min,
                        FPNavAgentSystem.DEFAULT_AREA_MASK));
                log.Clear();

                var outcome = system.TryInstallAbstractGraphIfBeneficial(out _);

                Assert.AreEqual(1, log.Entries.Count,
                    $"{outcome} produced {log.Entries.Count} lines; every outcome says what it "
                    + "decided, exactly once — a silent branch is one a game cannot tell from the "
                    + "call not having run");
            }
        }

        #endregion
    }
}
