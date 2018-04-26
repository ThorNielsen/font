#ifndef GLYPH_HPP_INCLUDED
#define GLYPH_HPP_INCLUDED

#include "freetype.hpp"
#include "vector2.hpp"

#include <vector>

class Glyph
{
public:
    Glyph(FT_Outline);

    void dumpInfo() const;

private:
    std::vector<size_t> m_contourEnd;
    std::vector<ivec2> m_position;
    std::vector<bool> m_isControlPoint;
    std::vector<bool> m_isThirdOrder;
};

#endif // GLYPH_HPP_INCLUDED

