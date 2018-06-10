#include "compressedbitmap.hpp"

#include <stdexcept>

void CompressedBitmap::setResolution(size_t logLength)
{
    if (logLength > 14)
    {
        throw std::domain_error("It's log(length), you dolt.");
    }
    size_t w = 1 << logLength;
    // Pad if image width fits in less than a pixel.
    m_bmLength = w >= 4 ? w : 4;
    m_byteWidth = m_bmLength >> 2;
    m_rows = w;
    m_data.resize((m_bmLength>>2) * m_rows);
}
