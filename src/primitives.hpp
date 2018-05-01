#ifndef PRIMITIVES_HPP_INCLUDED
#define PRIMITIVES_HPP_INCLUDED

#include "vector2.hpp"

// Ray consists of all points on the form p+t*d for t in [0, infinity).
template <typename T>
struct Ray_t
{
    vec2_t<T> pos;
    vec2_t<T> dir;
};

using Ray = Ray_t<S32>;

// Line segment consists of all points on the form p+t*d for t in [0, 1].
template <typename T>
struct LineSegment_t
{
    LineSegment_t() = default;
    LineSegment_t(vec2_t<T> p0, vec2_t<T> p1)
        : pos(p0), dir(p1-p0) {}
    vec2_t<T> pos;
    vec2_t<T> dir;
};

using LineSegment = LineSegment_t<S32>;

template <typename T>
struct QuadraticBezier_t
{
    QuadraticBezier_t() = default;
    QuadraticBezier_t(vec2_t<T> p, vec2_t<T> q, vec2_t<T> r)
        : p0(p), p1(q), p2(r) {}
    vec2_t<T> p0;
    vec2_t<T> p1;
    vec2_t<T> p2;
};

using QuadraticBezier = QuadraticBezier_t<S32>;

#endif // PRIMITIVES_HPP_INCLUDED

