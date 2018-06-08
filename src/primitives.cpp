#include "primitives.hpp"

void PackedBezier::construct()
{
    auto B = p1y-p0y;
    auto A = B+p1y-p2y;
    auto M = A-B;

    bool bgz = B>0;
    bool agz = A>0;
    bool mgz = M>0;
    U32 lookup1 = 0; // Replaces  if (A != 0)  body.

    /// !cgz, !kgz
    lookup1 |= 0x01*((bgz ? agz : 0) && (mgz ? 1 : !agz));
    lookup1 |= 0x02*((bgz ? 1 : !agz) && (mgz ? agz : 0));
    /// cgz, !kgz
    lookup1 |= 0x04*((bgz ? agz : 1) && (mgz ? 1 : !agz));
    lookup1 |= 0x08*((bgz ? 0 : !agz) && (mgz ? agz : 0));
    /// !cgz, kgz
    lookup1 |= 0x10*((bgz ? agz : 0) && (mgz ? 0 : !agz));
    lookup1 |= 0x20*((bgz ? 1 : !agz) && (mgz ? agz : 1));
    /// cgz, kgz
    lookup1 |= 0x40*((bgz ? agz : 1) && (mgz ? 0 : !agz));
    lookup1 |= 0x80*((bgz ? 0 : !agz) && (mgz ? agz : 1));

    // Lower bit contains minusGood and higher bit contains plusGood.
    // Indexed with (C == 0) + 2*(K == 0)
    U32 lookup2 = 0; // Replaces  pointHit  bodies.
    /// C!=0, K!=0
    lookup2 |= 0x01*(M > B);
    lookup2 |= 0x02*(M <= B);
    /// C==0, K!=0
    lookup2 |= 0x04*(M > B);
    lookup2 |= 0x08*(M <= B || bgz);
    /// C!=0, K==0
    lookup2 |= 0x10*(M > B || mgz);
    lookup2 |= 0x20*(M <= B);
    /// C==0, K==0
    lookup2 |= 0x40*(M > B || mgz);
    lookup2 |= 0x80*(M <= B || bgz);

    lookup = (lookup2 << 8) | lookup1;
}
