using System;

namespace xpTURN.Klotho.Deterministic.Math
{
    /// <summary>
    /// Signed 128-bit integer accumulator for exact geometric predicates.
    /// Two's-complement representation (Hi: signed high 64 bits, Lo: unsigned low 64 bits).
    /// Value type, zero GC, pure integer bit operations -- cross-platform bit-identical
    /// by construction (no floating point, no division).
    ///
    /// Named FPInt128 (not Int128) to avoid colliding with System.Int128 on .NET 7+ runtimes
    /// used by the dotnet test projects. Reference implementation: dotnet/runtime Int128 (MIT).
    ///
    /// Minimal surface on purpose: exact signed 64x64->128
    /// multiply, addition, subtraction/negation, sign, and one truncated 128/64 division
    /// (<see cref="DivRem"/> — added for the clip stage's X' cell location, the single
    /// place the rebaker needs a quotient; everything else stays division-free). Overflow
    /// past 128 bits wraps (two's complement); predicate inputs are domain-guarded so
    /// accumulations stay far below 2^127.
    /// </summary>
    public readonly struct FPInt128 : IEquatable<FPInt128>
    {
        public readonly long Hi;
        public readonly ulong Lo;

        public static readonly FPInt128 Zero = new FPInt128(0L, 0UL);

        public FPInt128(long hi, ulong lo)
        {
            Hi = hi;
            Lo = lo;
        }

        /// <summary>
        /// Exact signed 64x64 -> 128-bit product. No truncation, no saturation
        /// (contrast: FP64.SafeMultiply folds to Q32.32 and saturates).
        /// long.MinValue operands are outside the supported domain (sign-split would
        /// overflow); predicate inputs are bounded far below that by the domain assert.
        /// </summary>
        public static FPInt128 Mul64(long a, long b)
        {
            System.Diagnostics.Debug.Assert(a != long.MinValue && b != long.MinValue,
                "FPInt128.Mul64: long.MinValue operand is outside the supported domain");

            bool negative = (a < 0) ^ (b < 0);
            ulong ua = (ulong)(a < 0 ? -a : a);
            ulong ub = (ulong)(b < 0 ? -b : b);

            ulong hi = MulUnsigned(ua, ub, out ulong lo);
            var magnitude = new FPInt128((long)hi, lo);
            return negative ? Negate(magnitude) : magnitude;
        }

        /// <summary>
        /// Exact unsigned 64x64 -> 128; returns the high word and writes the low one. The
        /// intrinsic where the host has one, a 32-bit schoolbook split elsewhere — the same
        /// integer either way, so peers on different hosts agree bit for bit.
        ///
        /// <para>This is the ONE such split in the engine. <see cref="Mul64"/> signs it, and the
        /// abstract graph's quotient comparison reads it raw; a second copy would be a second code
        /// path to audit, with the domain assert and the BigInteger oracle on only one of them.</para>
        /// </summary>
        internal static ulong MulUnsigned(ulong a, ulong b, out ulong lo)
        {
#if NET5_0_OR_GREATER
            return System.Math.BigMul(a, b, out lo);
#else
            return MulUnsignedByHalves(a, b, out lo);
#endif
        }

        /// <summary>The halves alone, so a test can hold them against the intrinsic on a host that
        /// has one — see <c>Mul64_MatchesTheIntrinsic</c>.</summary>
        internal static ulong MulUnsignedByHalves(ulong a, ulong b, out ulong lo)
        {
            ulong aLo = (uint)a, aHi = a >> 32, bLo = (uint)b, bHi = b >> 32;
            ulong ll = aLo * bLo, lh = aLo * bHi, hl = aHi * bLo, hh = aHi * bHi;
            ulong mid = (ll >> 32) + (uint)lh + (uint)hl;
            lo = (mid << 32) | (uint)ll;
            return hh + (lh >> 32) + (hl >> 32) + (mid >> 32);
        }

        public static FPInt128 Add(FPInt128 a, FPInt128 b)
        {
            ulong lo = a.Lo + b.Lo;
            long carry = lo < a.Lo ? 1L : 0L;
            return new FPInt128(a.Hi + b.Hi + carry, lo);
        }

        public static FPInt128 Sub(FPInt128 a, FPInt128 b)
        {
            return Add(a, Negate(b));
        }

        /// <summary>Two's-complement negation (~v + 1).</summary>
        public static FPInt128 Negate(FPInt128 v)
        {
            ulong lo = ~v.Lo + 1UL;
            long hi = ~v.Hi + (lo == 0UL ? 1L : 0L);
            return new FPInt128(hi, lo);
        }

        /// <summary>-1, 0, +1 by two's-complement value. Predicates consume only this.</summary>
        public int Sign()
        {
            if (Hi < 0)
                return -1;
            if (Hi > 0 || Lo != 0UL)
                return 1;
            return 0;
        }

        /// <summary>
        /// Truncated (toward-zero, C# semantics) division of a 128-bit dividend by a nonzero
        /// 64-bit divisor. The quotient must fit in a signed 64-bit value — the clip stage's
        /// inputs guarantee it (|numerator| ≤ |t-den| · 2·MAX_SNAPPED, quotient ≤ 2·MAX_SNAPPED)
        /// and a Debug assert guards the contract. Remainder carries the dividend's sign,
        /// matching <c>long.DivRem</c>. Bit-serial long division: 128 iterations, exact,
        /// branch-deterministic — this runs a handful of times per placement, never per tick.
        /// </summary>
        public static long DivRem(FPInt128 dividend, long divisor, out long remainder)
        {
            System.Diagnostics.Debug.Assert(divisor != 0, "FPInt128.DivRem: divisor is zero");
            System.Diagnostics.Debug.Assert(divisor != long.MinValue,
                "FPInt128.DivRem: long.MinValue divisor is outside the supported domain");

            bool negDividend = dividend.Hi < 0;
            FPInt128 mag = negDividend ? Negate(dividend) : dividend;
            ulong dv = (ulong)(divisor < 0 ? -divisor : divisor);

            ulong qHi = 0UL, qLo = 0UL, rem = 0UL;
            for (int i = 127; i >= 0; i--)
            {
                // rem = (rem << 1) | bit_i(mag); rem never reaches 2^64 because rem < dv ≤ 2^63
                // before the shift.
                ulong bit = i >= 64
                    ? ((ulong)mag.Hi >> (i - 64)) & 1UL
                    : (mag.Lo >> i) & 1UL;
                rem = (rem << 1) | bit;
                if (rem >= dv)
                {
                    rem -= dv;
                    if (i >= 64)
                        qHi |= 1UL << (i - 64);
                    else
                        qLo |= 1UL << i;
                }
            }

            bool negQuotient = negDividend ^ (divisor < 0);
            System.Diagnostics.Debug.Assert(
                qHi == 0UL && qLo <= (negQuotient ? (ulong)long.MaxValue + 1UL : (ulong)long.MaxValue),
                "FPInt128.DivRem: quotient does not fit in a signed 64-bit value");

            long q = negQuotient ? -(long)qLo : (long)qLo;
            remainder = negDividend ? -(long)rem : (long)rem;
            return q;
        }

        public bool Equals(FPInt128 other)
        {
            return Hi == other.Hi && Lo == other.Lo;
        }

        public override bool Equals(object obj)
        {
            return obj is FPInt128 other && Equals(other);
        }

        public override int GetHashCode()
        {
            return (Hi.GetHashCode() * 397) ^ Lo.GetHashCode();
        }
    }
}
