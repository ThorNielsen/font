#ifndef MATRIX2_HPP_INCLUDED
#define MATRIX2_HPP_INCLUDED

#include "vector2.hpp"

#include <ostream>

template <typename T>
struct mat2_t;

// mat2*mat2
template <typename T>
inline mat2_t<T> operator*(const mat2_t<T>& a, const mat2_t<T>& b);

template <typename T>
struct mat2_t
{
public:
    mat2_t() = default;
    mat2_t(T val)
    {
        cols[0] = {val, T(0)};
        cols[1] = {T(0), val};
    }
    mat2_t(T a00, T a01, T a10, T a11)
    {
        (*this)(0, 0) = a00; (*this)(0, 1) = a01;
        (*this)(1, 0) = a10; (*this)(1, 1) = a11;
    }
    mat2_t(const vec2_t<T>& firstCol, const vec2_t<T>& secondCol)
    {
        cols[0] = firstCol;
        cols[1] = secondCol;
    }
    mat2_t(const mat2_t&) = default;

    T& operator()(size_t row_, size_t col_)
    {
        return cols[col_][row_];
    }
    const T& operator()(size_t row_, size_t col_) const
    {
        return cols[col_][row_];
    }

    vec2_t<T> row(size_t r) const
    {
        return {(*this)(r, 0), (*this)(r, 1)};
    }
    const vec2_t<T>& col(size_t c) const
    {
        return cols[c];
    }

    void setRow(size_t r, const vec2_t<T>& val)
    {
        (*this)(r, 0) = val[0];
        (*this)(r, 1) = val[1];
    }
    void setCol(size_t c, const vec2_t<T>& val)
    {
        cols[c] = val;
    }

    // Data is in column-major order. This means that the matrix
    // a00 a01
    // a10 a11
    // will have the following memory layout:
    // [a00, a10, a01, a11].
    T* data()
    {
        return static_cast<T*>(&cols[0][0]);
    }
    const T* data() const
    {
        return static_cast<const T*>(&cols[0][0]);
    }

    // Arithmetic operators
    mat2_t& operator+=(const mat2_t& o)
    {
        cols[0] += o.cols[0];
        cols[1] += o.cols[1];
        return *this;
    }

    mat2_t& operator-=(const mat2_t& o)
    {
        cols[0] -= o.cols[0];
        cols[1] -= o.cols[1];
        return *this;
    }

    mat2_t& operator*=(const mat2_t& o)
    {
        return *this = *this * o;
    }

    mat2_t& operator*=(const T& o)
    {
        cols[0] *= o;
        cols[1] *= o;
        return *this;
    }

    // Equality operators
    bool operator==(const mat2_t& o) const
    {
        return ((cols[0] == o.cols[0]) && (cols[1] == o.cols[1]));
    }

    bool operator!=(const mat2_t& o) const
    {
        return !(*this == o);
    }
private:
    vec2_t<T> cols[2];
};

// Binary operators
template <typename T>
inline mat2_t<T> operator+(mat2_t<T> a, const mat2_t<T>& b)
{
    return a += b;
}

template <typename T>
inline mat2_t<T> operator-(mat2_t<T> a, const mat2_t<T>& b)
{
    return a -= b;
}

template <typename T>
inline mat2_t<T> operator*(const mat2_t<T>& a, const mat2_t<T>& b)
{
    mat2_t<T> r;
    r(0, 0) = dot(a.row(0), b.col(0));
    r(0, 1) = dot(a.row(0), b.col(1));
    r(1, 0) = dot(a.row(1), b.col(0));
    r(1, 1) = dot(a.row(1), b.col(1));
    return r;
}

template <typename T>
inline vec2_t<T> operator*(const mat2_t<T>& a, const vec2_t<T>& b)
{
    return a.col(0) * b[0] + a.col(1) * b[1];
}

template <typename T>
inline mat2_t<T> operator*(mat2_t<T> m, const T& s)
{
    return m *= s;
}

template <typename T>
inline mat2_t<T> operator*(T s, const mat2_t<T>& m)
{
    mat2_t<T> a = m;
    return a *= s;
}


// Common matrix functions
template <typename T>
inline mat2_t<T> transpose(const mat2_t<T>& m)
{
    return {m.row(0), m.row(1)};
}

template <typename T>
inline T det(const mat2_t<T>& m)
{
    return m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
}

template <typename T>
inline mat2_t<T> inverse(const mat2_t<T>& m)
{
    T d = T(1)/det(m);
    mat2_t<T> a;
    a(0, 0) =  m(1, 1)*d;
    a(0, 1) = -m(0, 1)*d;
    a(1, 0) = -m(1, 0)*d;
    a(1, 1) =  m(0, 0)*d;
    return a;
}

template <typename T>
inline mat2_t<T> mabs(const mat2_t<T>& m)
{
    return mat2_t<T>(vabs(m.col(0)), vabs(m.col(1)));
}

template <typename T>
inline std::ostream& operator<<(std::ostream& ost, const mat2_t<T>& m)
{
    ost << "{" << m.row(0) << ",\n " << m.row(1) << "}";
    return ost;
}

// Typedefs
using mat2 = mat2_t<float>;
using dmat2 = mat2_t<double>;
using imat2 = mat2_t<int>;

// Assert that assumptions about the type are correct at compile-time:
// * Elements are in consecutive order.
// * Arrays are tightly packed.
// * There is no padding.
static_assert(std::is_pod<mat2>::value, "mat2 must be POD");
static_assert(sizeof(mat2) == sizeof(vec2)*2, "Size of mat2 is wrong.");
static_assert(sizeof(mat2[2]) == sizeof(mat2)*2, "mat2 not tightly packed.");
static_assert(std::is_pod<dmat2>::value, "dmat2 must be POD");
static_assert(sizeof(dmat2) == sizeof(dvec2)*2, "Size of dmat2 is wrong.");
static_assert(sizeof(dmat2[2]) == sizeof(dmat2)*2, "dmat2 not tightly packed.");
static_assert(std::is_pod<imat2>::value, "imat2 must be POD");
static_assert(sizeof(imat2) == sizeof(ivec2)*2, "Size of imat2 is wrong.");
static_assert(sizeof(imat2[2]) == sizeof(imat2)*2, "imat2 not tightly packed.");

template <typename T>
mat2 lowp_cast(const mat2_t<T>& o)
{
    return {lowp_cast(o.col(0)), lowp_cast(o.col(1))};
}

template <typename T>
dmat2 highp_cast(const mat2_t<T>& o)
{
    return {highp_cast(o.col(0)), highp_cast(o.col(1))};
}

#endif // MATRIX2_HPP_INCLUDED
