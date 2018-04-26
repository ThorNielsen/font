#ifndef VECTOR2_HPP_INCLUDED
#define VECTOR2_HPP_INCLUDED

#include "types.hpp"

#include <cmath>
#include <cstddef>
#include <type_traits>

#include <ostream>

template <typename T>
struct vec2_t
{
    union {T x, r, s;};
    union {T y, g, t;};

    inline T& operator[](size_t n)
    {
        return (&x)[n];
    }

    inline const T& operator[](size_t n) const
    {
        return (&x)[n];
    }

    inline T& operator()(size_t n)
    {
        return (&x)[n];
    }

    inline const T& operator()(size_t n) const
    {
        return (&x)[n];
    }

    // Vector-vector operations
    inline vec2_t& operator+=(const vec2_t& o)
    {
        x += o.x;
        y += o.y;
        return *this;
    }
    inline vec2_t& operator-=(const vec2_t& o)
    {
        x -= o.x;
        y -= o.y;
        return *this;
    }

    // Vector-scalar operations
    inline vec2_t& operator+=(const T& o)
    {
        x += o;
        y += o;
        return *this;
    }
    inline vec2_t& operator-=(const T& o)
    {
        x -= o;
        y -= o;
        return *this;
    }
    inline vec2_t& operator*=(const T& o)
    {
        x *= o;
        y *= o;
        return *this;
    }
    inline vec2_t& operator/=(const T& o)
    {
        x /= o;
        y /= o;
        return *this;
    }

    // Equality operators
    inline bool operator==(const vec2_t& o) const
    {
        return (flt_eq(x, o.x) && flt_eq(y, o.y));
    }

    inline bool operator!=(const vec2_t& o) const
    {
        return !(*this == o);
    }

    // Unary operators
    inline vec2_t operator+() const
    {
        return {+x, +y};
    }
    inline vec2_t operator-() const
    {
        return {-x, -y};
    }
};

// Vector-vector operations
template <typename T>
inline vec2_t<T> operator+(vec2_t<T> a, const vec2_t<T>& b)
{
    return a += b;
}

template <typename T>
inline vec2_t<T> operator-(vec2_t<T> a, const vec2_t<T>& b)
{
    return a -= b;
}

// Vector-scalar operations
template <typename T>
inline vec2_t<T> operator+(vec2_t<T> a, const T& b)
{
    return a += b;
}
template <typename T>
inline vec2_t<T> operator+(T a, const vec2_t<T>& b)
{
    vec2_t<T> c = b;
    return c += a;
}
template <typename T>
inline vec2_t<T> operator-(vec2_t<T> a, const T& b)
{
    return a -= b;
}
template <typename T>
inline vec2_t<T> operator*(vec2_t<T> a, const T& b)
{
    return a *= b;
}
template <typename T>
inline vec2_t<T> operator*(T a, const vec2_t<T>& b)
{
    vec2_t<T> c = b;
    return c *= a;
}
template <typename T>
inline vec2_t<T> operator/(vec2_t<T> a, const T& b)
{
    return a /= b;
}

// Common vector functions
template <typename T>
inline T dot(const vec2_t<T>& a, const vec2_t<T>& b)
{
    return a.x*b.x+a.y*b.y;
}

template <typename T>
inline vec2_t<T> perp(const vec2_t<T>& a)
{
    return {-a.y, a.x};
}

template <typename T>
inline T length(const vec2_t<T>& a)
{
    return std::sqrt(dot(a,a));
}

template <typename T>
inline vec2_t<T> normalise(const vec2_t<T>& a)
{
    T len = T(1) / length(a);
    vec2_t<T> b;
    b.x = a.x * len;
    b.y = a.y * len;
    return b;
}

template <typename T>
inline T projectionLength(const vec2_t<T>& a, const vec2_t<T>& b)
{
    return std::abs(dot(a, b) / dot(b, b));
}

template <typename T>
inline T squaredLength(const vec2_t<T>& v)
{
    return dot(v, v);
}

template <typename T>
inline vec2_t<T> vabs(const vec2_t<T>& v)
{
    return {std::abs(v.x), std::abs(v.y)};
}

template <typename T>
inline std::ostream& operator<<(std::ostream& ost, const vec2_t<T>& v)
{
    ost << "{" << v.x << ", " << v.y << "}";
    return ost;
}

// Typedefs
using vec2 = vec2_t<float>;
using dvec2 = vec2_t<double>;
using ivec2 = vec2_t<S32>;
using uvec2 = vec2_t<U32>;

// Assert that assumptions about the type are correct at compile-time:
// * Elements are in consecutive order.
// * Arrays are tightly packed.
// * There is no padding.
static_assert(std::is_pod<vec2>::value, "vec2 must be POD");
static_assert(sizeof(vec2) == sizeof(float)*2, "Size of vec2 is wrong.");
static_assert(sizeof(vec2[2]) == sizeof(vec2)*2, "vec2 not tightly packed.");
static_assert(std::is_pod<dvec2>::value, "dvec2 must be POD");
static_assert(sizeof(dvec2) == sizeof(double)*2, "Size of dvec2 is wrong.");
static_assert(sizeof(dvec2[2]) == sizeof(dvec2)*2, "dvec2 not tightly packed.");
static_assert(std::is_pod<ivec2>::value, "ivec2 must be POD");
static_assert(sizeof(ivec2) == sizeof(S32)*2, "Size of ivec2 is wrong.");
static_assert(sizeof(ivec2[2]) == sizeof(ivec2)*2, "ivec2 not tightly packed.");
static_assert(std::is_pod<uvec2>::value, "uvec2 must be POD");
static_assert(sizeof(uvec2) == sizeof(U32)*2, "Size of uvec2 is wrong.");
static_assert(sizeof(uvec2[2]) == sizeof(uvec2)*2, "uvec2 not tightly packed.");

template <typename T>
vec2 lowp_cast(const vec2_t<T>& o)
{
    return {static_cast<float>(o.x), static_cast<float>(o.y)};
}

template <typename T>
dvec2 highp_cast(const vec2_t<T>& o)
{
    return {static_cast<double>(o.x), static_cast<double>(o.y)};
}

#endif // VECTOR2_HPP_INCLUDED
