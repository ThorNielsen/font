#ifndef PRIMITIVES_HPP_INCLUDED
#define PRIMITIVES_HPP_INCLUDED

#include "vector2.hpp"

template <typename T>
struct QuadraticBezier_t
{
    QuadraticBezier_t() = default;
    QuadraticBezier_t(vec2_t<T> p, vec2_t<T> q, vec2_t<T> r)
        : p0(p), p1(q), p2(r) {}
    vec2_t<T> p0;
    vec2_t<T> p1;
    vec2_t<T> p2;

    T minX() const
    {
        return std::min(p0.x, std::min(p1.x, p2.x));
    }
    T minY() const
    {
        return std::min(p0.y, std::min(p1.y, p2.y));
    }
    T maxX() const
    {
        return std::max(p0.x, std::max(p1.x, p2.x));
    }
    T maxY() const
    {
        return std::max(p0.y, std::max(p1.y, p2.y));
    }
};

using QuadraticBezier = QuadraticBezier_t<S32>;


struct PackedBezier
{
    // All variables of type 'T' in this object will always lie in the range
    // [0, 2^15), they will in particular always be unsigned. S16 is chosen
    // since measurements show this gives the highest performance (although S64
    // is somewhat faster, it is not very cache-friendly and adds padding). Do
    // note that the differences are minuscule, however (about ~2%).
    using T = S16;
    PackedBezier() = default;
    template <typename T>
    PackedBezier(vec2_t<T> p, vec2_t<T> q, vec2_t<T> r)
        : p0x(p.x), p1x(q.x), p2x(r.x),
          p0y(p.y), p1y(q.y), p2y(r.y)
    {
        construct();
    }

    U32 lookup;
    T p0x;
    T p1x;
    T p2x;
    T p0y;
    T p1y;
    T p2y;

    T minX() const
    {
        return std::min(p0x, std::min(p1x, p2x));
    }
    T minY() const
    {
        return std::min(p0y, std::min(p1y, p2y));
    }
    T maxX() const
    {
        return std::max(p0x, std::max(p1x, p2x));
    }
    T maxY() const
    {
        return std::max(p0y, std::max(p1y, p2y));
    }
private:
    void construct();
};

#endif // PRIMITIVES_HPP_INCLUDED

