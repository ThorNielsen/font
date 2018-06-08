#ifndef CRC_HPP_INCLUDED
#define CRC_HPP_INCLUDED

#include "types.hpp"

U32 crc(const U8* data, const U8* end);
inline U32 crc(const U8* data, size_t length)
{
    return crc(data, data+length);
}

#endif // CRC_HPP_INCLUDED

