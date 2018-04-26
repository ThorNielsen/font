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
    vec2_t<T> pos;
    vec2_t<T> dir;
};

using LineSegment = LineSegment_t<S32>;

#endif // PRIMITIVES_HPP_INCLUDED

