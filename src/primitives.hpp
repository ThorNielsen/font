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
    PackedBezier() = default;
    template <typename T>
    PackedBezier(vec2_t<T> p, vec2_t<T> q, vec2_t<T> r)
        : p0x(p.x), p1x(q.x), p2x(r.x),
          p0y(p.y), p1y(q.y), p2y(r.y)
    {
        // Todo: magic
    }

    U32 magic;
    S16 p0x;
    S16 p1x;
    S16 p2x;
    S16 p0y;
    S16 p1y;
    S16 p2y;

    S16 minX() const
    {
        return std::min(p0x, std::min(p1x, p2x));
    }
    S16 minY() const
    {
        return std::min(p0y, std::min(p1y, p2y));
    }
    S16 maxX() const
    {
        return std::max(p0x, std::max(p1x, p2x));
    }
    S16 maxY() const
    {
        return std::max(p0y, std::max(p1y, p2y));
    }
};

#endif // PRIMITIVES_HPP_INCLUDED

