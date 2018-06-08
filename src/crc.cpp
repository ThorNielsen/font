#include "crc.hpp"
#include <iostream>
#include <array>
namespace
{

std::array<U32, 256> crcTable;

void generateCRCTable()
{
    for (U32 i = 0; i < 256; ++i)
    {
        U32 b = i;
        for (U32 j = 0; j < 8; ++j)
        {
            if (b & 1) b = 0xedb88320 ^ (b >> 1);
            else b >>=1;
        }
        crcTable[i] = b;
    }
}

} // end anonymous namespace

U32 crc(const U8* data, const U8* end)
{
    static bool init = false;
    if (!init)
    {
        init = true;
        generateCRCTable();
    }

    U32 crc = 0xffffffff;
    while (data != end)
    {
        crc = crcTable[(crc ^ (*data++)) & 0xff] ^ (crc >> 8);
    }
    return crc ^ 0xffffffff;
}
