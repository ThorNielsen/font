#ifndef COMMON_HPP_INCLUDED
#define COMMON_HPP_INCLUDED

template <U8 BitPos>
U32 bit(U8 val)
{
    return (val>>BitPos)&1;
}

#endif // COMMON_HPP_INCLUDED

