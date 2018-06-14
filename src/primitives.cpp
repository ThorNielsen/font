#include "primitives.hpp"

void PackedBezier::construct()
{
    S16 B = p1y-p0y;
    S16 A = B+p1y-p2y;
    S16 M = A-B;

    bool bgz = B>0;
    bool agz = A>0;
    bool mgz = M>0;
    lookup = 0; // Replaces  if (A != 0)  body.

    // Lower bit contains minusGood and higher bit contains plusGood.
    // Indexed with (C >= 0) + 2*(K >= 0)
    /// !cgz, !kgz
    lookup |= 0x01*((bgz ? agz : 0) && (mgz ? 1 : !agz));
    lookup |= 0x02*((bgz ? 1 : !agz) && (mgz ? agz : 0));
    /// cgz, !kgz
    lookup |= 0x04*((bgz ? agz : 1) && (mgz ? 1 : !agz));
    lookup |= 0x08*((bgz ? 0 : !agz) && (mgz ? agz : 0));
    /// !cgz, kgz
    lookup |= 0x10*((bgz ? agz : 0) && (mgz ? 0 : !agz));
    lookup |= 0x20*((bgz ? 1 : !agz) && (mgz ? agz : 1));
    /// cgz, kgz
    lookup |= 0x40*((bgz ? agz : 1) && (mgz ? 0 : !agz));
    lookup |= 0x80*((bgz ? 0 : !agz) && (mgz ? agz : 1));
}
