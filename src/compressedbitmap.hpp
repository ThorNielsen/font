#ifndef COMPRESSEDBITMAP_HPP_INCLUDED
#define COMPRESSEDBITMAP_HPP_INCLUDED

#include "types.hpp"
#include <vector>

class CompressedBitmap
{
public:
    void setResolution(size_t logWidth);

    CompressedBitmap downscale(int levels = 1) const;

    U32 operator()(size_t x, size_t y)
    {
        return (m_data[m_bmWidth*y+(x>>2)]>>((x&3)<<1))&3;
    }

    void setValue(size_t x, size_t y, U32 val)
    {
        m_data[m_bmWidth*y+(x>>2)] |= val << ((x&3)<<1);
    }

private:
    std::vector<U8> m_data;
    size_t m_bmWidth;
    size_t m_pixCount;
};

#endif // COMPRESSEDBITMAP_HPP_INCLUDED

