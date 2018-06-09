#include "compressedbitmap.hpp"

#include <stdexcept>

void CompressedBitmap::setResolution(size_t logWidth)
{
    if (logWidth > 14)
    {
        throw std::domain_error("It's log(width), you dolt.");
    }
    size_t width = 1 << logWidth;
    // Pad if image width fits in less than a pixel.
    m_bmWidth = width >= 4 ? width : 4;
    m_pixCount = width;
    m_data.resize((m_bmWidth>>2) * m_pixCount);
}
