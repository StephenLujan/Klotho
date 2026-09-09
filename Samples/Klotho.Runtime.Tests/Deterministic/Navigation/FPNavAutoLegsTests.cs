using System.IO;
using System.Linq;
using NUnit.Framework;

using xpTURN.Klotho.Deterministic.Math;
using xpTURN.Klotho.Helper.Tests;
using xpTURN.Klotho.Logging;

namespace xpTURN.Klotho.Deterministic.Navigation.Tests
{
    /// <summary>
    /// Legs on by default (0.13): <see cref="FPNavTuning.AutoInstallAbstractGraph"/> makes the agent
    /// system's constructor run <see cref="FPNavAgentSystem.TryInstallAbstractGraphIfBeneficial"/>
    /// for the budget-exhaustion condition. These are the gates of IMP110/Plan-AutoLegs (V-B1..V-B7,
    /// V-B10): a large mesh gets a graph, a small one does not change by a bit, the named-off stack
    /// is the pre-0.13 one, the game's own graph wins, and — the safety argument — the automatic
    /// graph is the very graph the explicit call would have installed.
    /// </summary>
    [TestFixture]
    public class FPNavAutoLegsTests
    {
        private const string FieldAsset = "Samples/Brawler/Assets/NavMesh/Data/Field.NavMeshData.bytes";

        private static string RepoRoot()
        {
            var dir = new DirectoryInfo(TestContext.CurrentContext.TestDirectory);
            while (dir != null && !Directory.Exists(Path.Combine(dir.FullName, "com.xpturn.klotho")))
                dir = dir.Parent;
            Assert.IsNotNull(dir, "repo root not found from test base directory");
            return dir.FullName;
        }

        private static FPNavMesh Field() => FPNavMeshSerializer.Deserialize(Path.Combine(RepoRoot(), FieldAsset));

        private static FPNavAgentSystem Sys(FPNavMesh mesh, FPNavTuning tuning, IKLogger logger = null)
            => NavAgentTestHelper.CreateSystem(mesh, logger, tuning, out _);

        private static readonly FPNavTuning Off = NavAgentTestHelper.NoAutoGraph;

        #region V-B1 / V-B3 — the default stack has a graph on a large mesh; the OFF stack is the old one

        [Test]
        public void VB1_ADefaultStack_OnAMeshPastTheBudget_HasAGraph_AndADifferentFingerprint()
        {
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(64, 40, out _);   // 5198 > 4096
            var auto = Sys(mesh, FPNavTuning.Default);
            var off = Sys(mesh, Off);

            Assert.IsNotNull(auto.AbstractGraph, "the constructor installed a graph");
            Assert.IsNull(off.AbstractGraph, "the named-off stack has none");
            Assert.AreNotEqual(off.GetNavFingerprint(), auto.GetNavFingerprint(),
                "a graph is part of the fingerprint — an on peer and an off peer refuse each other");
            Assert.AreEqual(FPNavAbstractGraphInstall.AlreadyInstalled,
                auto.TryInstallAbstractGraphIfBeneficial(out _),
                "the explicit call finds the constructor's graph and leaves it alone");
        }

        [Test]
        public void VB3_TheOffStack_OnField_IsThePre013Fingerprint()
        {
            var off = Sys(Field(), Off);
            Assert.IsNull(off.AbstractGraph);
            Assert.AreEqual(unchecked((long)0x5BB050AA23A9B9B3UL), off.GetNavFingerprint(),
                "Field with no graph, after the 0.13 partial flip and before any automatic install — "
                + "what a game that names autoInstallAbstractGraph: false gets, bit for bit");
        }

        #endregion

        #region V-B2 — a mesh within the budget does not change by a bit, whatever the cap says

        [TestCase(8)]    // 128 triangles: at the corridor cap, not past it
        [TestCase(12)]   // 288: past the corridor cap, within the budget — the case D-B3 excludes
        public void VB2_AMeshWithinTheBudget_IsUntouched_EvenPastTheCorridorCap(int cells)
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(cells);
            Assert.LessOrEqual(mesh.Triangles.Length, FPNavTuning.Default.MaxIterations, "fixture: within the budget");

            var auto = Sys(mesh, FPNavTuning.Default);
            var off = Sys(mesh, Off);
            Assert.IsNull(auto.AbstractGraph, "the automatic install asks only about exhaustion");
            Assert.AreEqual(off.GetNavFingerprint(), auto.GetNavFingerprint(), "not a bit differs");

            if (cells == 12)
                Assert.AreEqual(FPNavAbstractGraphInstall.Installed,
                    Sys(mesh, Off).TryInstallAbstractGraphIfBeneficial(out _),
                    "the EXPLICIT call still installs here — it asks about the clamp too, as it always did");
        }

        [Test]
        public void VB2_TheDefault8CellFingerprint_DidNotMove()
        {
            var auto = Sys(NavAgentTestHelper.CreateOpenFieldNavMesh(8), FPNavTuning.Default);
            Assert.AreEqual(unchecked((long)0xAD047D83C6DF1916UL), auto.GetNavFingerprint(),
                "the default fingerprint pinned after the partial flip — the automatic install does not touch a small mesh");
        }

        [Test]
        public void VB2_TheSerpentineTests_ThatLowerTheBudget_GetAGraphOnlyBecauseTheyDo()
        {
            // 202 triangles: within the default budget (no graph), past a budget of 60 (graph). The
            // budget is the threshold — that is the coupling FPNavTuning.AutoInstallAbstractGraph documents.
            var mesh = NavAgentTestHelper.CreateSerpentineNavMesh(16, 6, out _);
            Assert.IsNull(Sys(mesh, FPNavTuning.Default).AbstractGraph);
            Assert.IsNotNull(Sys(mesh, new FPNavTuning(maxIterations: 60)).AbstractGraph);
            Assert.IsNull(Sys(mesh, new FPNavTuning(maxIterations: 60, autoInstallAbstractGraph: false)).AbstractGraph);
        }

        #endregion

        #region V-B4 — the game's own graph wins

        [Test]
        public void VB4_SetAbstractGraph_ReplacesTheAutomaticOne_AndSaysSoOnce()
        {
            var mesh = NavAgentTestHelper.CreateOpenFieldNavMesh(48);   // 4608 > 4096
            var log = new LogCapture();
            var system = Sys(mesh, FPNavTuning.Default, log);
            Assert.IsNotNull(system.AbstractGraph);
            FP64 autoCell = system.AbstractGraph.CellSize;

            var own = new FPNavAbstractGraph(mesh, autoCell / FP64.FromInt(2), FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK);
            system.SetAbstractGraph(own);
            Assert.AreSame(own, system.AbstractGraph, "the game's graph is the one in use");
            Assert.IsTrue(log.Contains(KLogLevel.Information, "replaced the graph this system already had"),
                "the discarded derivation is said, once, with the way to skip it");

            system.SetAbstractGraph(new FPNavAbstractGraph(mesh, autoCell, FPNavAbstractCostFold.Min,
                FPNavAgentSystem.DEFAULT_AREA_MASK));
            Assert.AreEqual(1, log.Entries.Count(e => e.Level == KLogLevel.Information && e.Message.Contains("replaced the graph this system already had")),
                "said once per system, not once per replacement");
        }

        #endregion

        #region V-B5 — Equals sees the switch, Digest does not

        [Test]
        public void VB5_TheSwitch_IsInEquals_AndNotInTheDigest()
        {
            Assert.IsTrue(FPNavTuning.Default.AutoInstallAbstractGraph, "on by default since 0.13");
            Assert.AreNotEqual(FPNavTuning.Default, Off, "Equals must see it, or a stack could be half on");
            Assert.AreNotEqual(FPNavTuning.Default.GetHashCode(), Off.GetHashCode());
            Assert.AreEqual(0L, FPNavTuning.Default.Digest, "the default digest is still the identity");
            Assert.AreEqual(FPNavTuning.Default.Digest, Off.Digest, "no fingerprint term of its own — the graph's checksum is the term");
            Assert.DoesNotThrow(() => Off.Validate());
        }

        #endregion

        #region V-B6 — a swap that nothing prepared for warns once

        [Test]
        public void VB6_ASwapWithoutAPreparedGraph_WarnsOnce_AndCountsEveryTime()
        {
            var a = NavAgentTestHelper.CreateOpenFieldNavMesh(48);
            var b = NavAgentTestHelper.CreateOpenFieldNavMesh(48);
            var log = new LogCapture();
            var system = Sys(a, FPNavTuning.Default, log);
            Assert.IsNotNull(system.AbstractGraph, "fixture: the automatic graph is what the swap has to re-derive");

            system.SwapNavMesh(b);
            system.SwapNavMesh(a);

            Assert.AreEqual(2, system.DebugGraphRederiveCount, "both swaps re-derived");
            Assert.AreEqual(1, log.Entries.Count(e => e.Level == KLogLevel.Warning && e.Message.Contains("no graph was prepared for this mesh ahead of the swap")),
                "said once per system");
            Assert.IsTrue(log.Contains(KLogLevel.Warning, "PrepareAbstractGraphFor"), "and it names the fix");
        }

        #endregion

        #region V-B10 — automatic == explicit

        [Test]
        public void VB10_TheAutomaticGraph_IsTheGraphTheExplicitCallInstalls()
        {
            var mesh = Field();
            var auto = Sys(mesh, FPNavTuning.Default);

            var explicitly = Sys(mesh, Off);
            Assert.AreEqual(FPNavAbstractGraphInstall.Installed, explicitly.TryInstallAbstractGraphIfBeneficial(out FP64 cell));

            Assert.AreEqual(explicitly.AbstractGraph.Checksum, auto.AbstractGraph.Checksum, "same graph");
            Assert.AreEqual(cell, auto.AbstractGraph.CellSize, "same cell size from the same ladder");
            Assert.AreEqual(explicitly.GetNavFingerprint(), auto.GetNavFingerprint(),
                "same fingerprint — the constructor did exactly what the one-line call did");
            Assert.AreEqual(unchecked((long)0x5B82AF48D6337593UL), auto.GetNavFingerprint(),   // 0xB5E5C29564219934 before rule revision 6 (pair-table costs)
                "and it is the value measured with the explicit helper before the constructor learned to call it");
        }

        [Test]
        public void VB10_ANullMeshSystem_StillHasNoGraph_AndAZeroFingerprint()
        {
            var system = new FPNavAgentSystem(null, null, null, null, null);
            Assert.IsNull(system.AbstractGraph);
            Assert.AreEqual(0L, system.GetNavFingerprint());
        }

        #endregion
    }
}
