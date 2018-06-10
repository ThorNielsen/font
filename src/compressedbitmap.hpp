#ifndef COMPRESSEDBITMAP_HPP_INCLUDED
#define COMPRESSEDBITMAP_HPP_INCLUDED

#include "types.hpp"
#include <vector>
#include <iostream>

class CompressedBitmap
{
public:
    void setResolution(size_t logLength);

    CompressedBitmap downscale(int levels = 1) const;

    U32 operator()(size_t x, size_t y) const
    {
        return (m_data[m_byteWidth*y+(x>>2)]>>((x&3)<<1))&3;
    }

    void setValue(size_t x, size_t y, U32 val)
    {
        m_data[m_byteWidth*y+(x>>2)] |= val << ((x&3)<<1);
    }

    size_t width() const { return m_bmLength; }
    size_t byteLength() const { return m_byteWidth; }
    size_t rows() const { return m_rows; }


private:
    std::vector<U8> m_data;
    size_t m_byteWidth;
    size_t m_bmLength;
    size_t m_rows;
};

#endif // COMPRESSEDBITMAP_HPP_INCLUDED

