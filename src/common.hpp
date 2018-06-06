#ifndef COMMON_HPP_INCLUDED
#define COMMON_HPP_INCLUDED


template <typename T>
T sign(T val)
{
    if (val > T(0)) return T(1);
    else if (val < T(0)) return T(-1);
    return T(0);
}

template <U8 BitPos, typename T>
U32 bit(T val)
{
    return (val>>BitPos)&1;
}

#endif // COMMON_HPP_INCLUDED

