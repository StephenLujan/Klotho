namespace xpTURN.Klotho.Deterministic.Navigation
{
    /// <summary>
    /// A navigation system that can build, ahead of time, whatever a coming navmesh swap will make
    /// it rebuild. <see cref="FPNavAgentSystem"/> implements it for the abstract graph planning in
    /// legs uses, which a swap otherwise re-derives inside the tick.
    ///
    /// <para><b>Why an interface and not a direct call.</b> The engine paces rebake slices
    /// (<c>FPNavMeshRebakeDriver.AdvanceSlice</c>) because leaving that to the game let a whole host
    /// miss it for a release. The same argument applies here and more sharply: the work is invisible
    /// when it is missed — nothing breaks, a tick just costs ten milliseconds more — so a game would
    /// never learn it had forgotten. This seam lets the engine do it, while keeping
    /// <c>Core/</c> free of any knowledge of what a graph is.</para>
    ///
    /// <para><b>Every implementation must be safe to call with null, with the live mesh, and with
    /// the same mesh many frames running.</b> The caller is a frame heartbeat that knows nothing
    /// about what is worth preparing.</para>
    /// </summary>
    public interface INavGraphPreparer
    {
        /// <summary>
        /// Prepares for <paramref name="mesh"/> becoming live. Off the deterministic path, on the
        /// caller's thread, and free to do nothing.
        /// </summary>
        void PrepareAbstractGraphFor(FPNavMesh mesh);
    }
}
